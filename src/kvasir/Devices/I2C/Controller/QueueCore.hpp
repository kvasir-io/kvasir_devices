#pragma once
// The chip-independent half of a queued I2C master: the queue, the request on the wire, cancel and deadlines, the
// transfer timeout, the dead-bus watchdog, the post-failure settle, the bus recovery's clock, the fault log and the
// counters. The chip packages' I2CBehaviorQueued (chip_rp_common I2CQueued.hpp, chip_atsam_common
// Sercom_I2CQueued.hpp) are this core over a hardware policy that holds only the register work (QueueHw.hpp).
//
// Contexts: submit(), submitTracked(), cancel(), reset(), requestRecovery() and takeLatency() may be called from the
// main loop, a callback or the bus interrupt; each runs with the bus interrupt masked, depth-counted
// (Bus/MaskDepth.hpp: a callback run inside a masked section may submit(); only the outermost section masks and
// unmasks). turn() runs from the main loop. Every accepted request gets exactly one callback. A callback runs with
// its request already over (`state_` idle, `active_` false) and may submit(), cancel() or reset(); what it submits
// starts after it has returned (Bus/Callbacks.hpp: nothing starts while a callback runs) and after the post-failure
// settle checks of the request that called it - the order the SPI QueueCore has too.
#include "../../Bus/Callbacks.hpp"
#include "../../Bus/DeadBus.hpp"
#include "../../Bus/MaskDepth.hpp"
#include "../../Bus/RequestQueue.hpp"
#include "../../Bus/Ticks.hpp"
#include "../../BusTypes.hpp"
#include "../../Features.hpp"
#include "../../Log.hpp"
#include "QueueHw.hpp"
#include "kvasir/Log/FaultLog.hpp"
#include "kvasir/Log/RateLimiter.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace I2C {

    template<typename Hw, typename Clock, std::size_t Depth, std::size_t CallbackCapacity>
        requires QueueHw<Hw, Clock>
    struct QueueCore {
        static constexpr std::size_t QueueDepth   = Depth;
        static constexpr std::size_t CallbackSize = CallbackCapacity;
        using tp                                  = typename Clock::time_point;
        using Recovery                            = typename Hw::Recovery;
        using Request                             = typename Hw::Request;

        static constexpr bool Cancellable    = Hw::cancellable;
        static constexpr bool Deadlines      = Hw::deadlines;
        static constexpr bool Tracked        = Cancellable || Deadlines;
        static constexpr bool PerDeviceClock = Hw::perDeviceClock;
        /// The optional parts (Bus::QueueFeatures, the config's queueFeatures): off, a part costs nothing and its
        /// accessors do not compile.
        static constexpr ::Kvasir::Bus::QueueFeatures Features = Hw::busFeatures;
        using Result
          = std::conditional_t<Tracked, ::Kvasir::Bus::TrackedOutcome, ::Kvasir::Bus::Outcome>;
        using State           = QueueState;
        using TimeoutSnapshot = I2C::TimeoutSnapshot<typename Hw::Snapshot>;

        /// Bus faults in an unbroken row that mean the bus is dead (the rule: Bus/DeadBus.hpp - a NAK resets it).
        static constexpr std::uint32_t DeadBusFailures = 40;

        /// The rate limiter's fault kinds: keys only, never printed.
        enum class Fault : std::uint8_t {
            abortSend = 1,
            abortRecv,
            timeout,
            sdaStuck,
            recovered,
        };

        /// The queue's view of a request (Bus/RequestQueue.hpp): the entry is the request.
        struct QueueSpec {
            using Result                    = QueueCore::Result;
            static constexpr bool Tracked   = QueueCore::Tracked;
            static constexpr bool Deadlines = QueueCore::Deadlines;

            static Request& request(Request& r) { return r; }
        };

        // QueueDepth requests wait at once, besides the one on the wire.
        inline static ::Kvasir::Bus::RequestQueue<Request, QueueDepth, Clock, QueueSpec> queue_{};
        inline static Request currentRequest_{};
        inline static bool    active_{false};
        inline static State   state_{State::idle};
        // std::size_t: a request's spans are not capped at 255 bytes.
        inline static std::size_t sendIndex_{0};
        inline static std::size_t receivedCount_{0};
        inline static tp          timeoutTime_{};
        // Fault logging goes through this: a bad bus faults on every transaction.
        inline static Kvasir::FaultLog<Clock, Fault> faultLog_{};
        // address NAKs (debug): apart, so a scan does not spend the faults' budget and summary
        inline static Kvasir::LogRateLimiter<Clock> nakLog_{};
        // Set while reset() / requestRecovery() run the failure callbacks: a callback that
        // submits only queues (and mask_ keeps the interrupt masked).
        inline static bool resetting_{};

        // The bus interrupt masked around thread code that shares state with it (Hw::mask: ICER, dsb; isb), nested:
        // a callback run inside a section (expireDeadlines_, a timeout's completion, reset()'s failures) may
        // submit(), and only the outermost section masks and unmasks the line. Every lock() has its unlock().
        inline static ::Kvasir::Bus::MaskDepth<Hw> mask_{};
        // Every callback runs through it; tryStart_ starts nothing while one runs.
        inline static ::Kvasir::Bus::Invoker inv_{};

        // -- the bus's API ----------------------------------------------------------------------

        /// Every submitted request gets exactly one callback: the one on the wire and the
        /// ones queued behind it are failed here, before the block is reset, so nothing waits
        /// on a request the reset threw away. What a callback submits from here is not failed:
        /// it waits, and starts once the block is back (as on the SPI QueueCore).
        static void reset() { reset_(true); }

        static bool submit(Request const& req) {
            mask_.lock();
            // inside the masked window, the push too: submit() also runs from completion callbacks
            // in the ISR, and the queue has one producer only if the two cannot interleave
            if(queue_.full()) {
                mask_.unlock();
                return false;
            }
            queue_.push(req);
            ++x_.transfers;
            tryStart_(Clock::now());
            // From a callback reset() or requestRecovery() runs, the interrupt stays masked:
            // they unmask it when they are done.
            mask_.unlock();
            return true;
        }

        /// submit(), with a ticket for cancel(); an invalid ticket (refused, no callback) when submit() would be false.
        static ::Kvasir::Bus::Ticket submitTracked(Request req)
            requires(Tracked)
        {
            mask_.lock();
            req.ticket = ::Kvasir::Bus::nextTicket(ticketCounter_);
            mask_.unlock();
            return submit(req) ? ::Kvasir::Bus::Ticket{req.ticket} : ::Kvasir::Bus::Ticket{};
        }

        /// removed: queued, never started - its callback (cancelled) ran before this returns. stopping: on the
        /// wire - Hw::requestStop() asks the block for a STOP, the interrupt completes it (cancelled); its buffers
        /// belong to the bus until then, and a stop that never comes is the transfer timeout's. tooLate: completed
        /// already, or an unknown ticket.
        static ::Kvasir::Bus::Cancel cancel(::Kvasir::Bus::Ticket t)
            requires(Cancellable)
        {
            if(!t.valid()) { return ::Kvasir::Bus::Cancel::tooLate; }
            mask_.lock();
            auto r = ::Kvasir::Bus::Cancel::tooLate;
            if(active_ && state_ != State::idle && currentRequest_.ticket == t.id) {
                if(!cancelPending_) {
                    cancelPending_ = true;
                    stopAs_        = Result::cancelled;
                    Hw::requestStop();
                }
                r = ::Kvasir::Bus::Cancel::stopping;
            } else {
                if(queue_.cancelQueued(t.id, inv_)) {
                    r = ::Kvasir::Bus::Cancel::removed;
                    // what the callback submitted waited for it to return; on an idle bus it starts now
                    tryStart_(Clock::now());
                }
            }
            mask_.unlock();
            return r;
        }

        // The bus-level turn — once per main loop per bus.
        // Owns: timeout detection.  Delegates bus-health to Recovery.
        static void turn() {
            auto const now = Clock::now();

            // Recovery state machine — highest priority, blocks normal operation
            auto const rv = Recovery::tick(now);
            if(rv == Recovery::TickResult::needsReinit) {
                // the address the request before the recovery had: the re-init may start the next one
                auto const address = currentRequest_.address;
                // The block back after the recovery. It fails nothing queued: what waited for the
                // recovery (queued while it ran, or resubmitted by a failure callback) starts now.
                reset_(false);
                // The limiter is shared with the ISR's fault lines, and reset_() has just
                // unmasked the interrupt: the decision is taken with it masked again.
                mask_.lock();
                [[maybe_unused]] auto const recoveredLine
                  = faultLog_.note(Fault::recovered, now, address, 0);
                mask_.unlock();
                // The line states right after the sequence are the diagnosis: both high
                // and the bus is back, SDA low means a slave still holds data, SCL low
                // means no master can help.
                KVASIR_LOG_LIMITED(recoveredLine,
                                   UC_LOG_W,
                                   "i2c{} recovery #{} complete -- SDA {}, SCL {}, {} of 9 "
                                   "clocks unused, {} forced STOP(s) so far",
                                   Hw::instance,
                                   Recovery::recoveries(),
                                   std::string_view{Recovery::sdaIsHigh() ? "high" : "LOW"},
                                   std::string_view{Recovery::sclIsHigh() ? "high" : "LOW"},
                                   Recovery::clocksLeft(),
                                   Recovery::forcedStops());
                return;
            }
            if(rv == Recovery::TickResult::busy) { return; }

            // What the ISR writes too, taken in one go with its interrupt masked: the faults the
            // log rate limiter dropped (reported now that the bus is quiet), the failure streak,
            // and the post-abort settle gate -- a 64-bit time, which an unmasked read could see
            // half-written. The ISR can land between any two of these.
            mask_.lock();
            auto const droppedFaults = faultLog_.takeSummary(now);
            auto const droppedNaks   = nakLog_.takeSummary(now);
            bool const deadBus       = x_.deadBus.takeIfDead();
            bool const settled       = Recovery::isPastSettle(now);
            mask_.unlock();
            if(droppedFaults != 0) {
                UC_LOG_W("i2c{} +{} faults not logged", Hw::instance, droppedFaults);
            }
            if(droppedNaks != 0) {
                UC_LOG_D("i2c{} +{} address NAKs not logged", Hw::instance, droppedNaks);
            }

            // Dead-bus watchdog: line-state checks miss failures on a healthy wire (an
            // unconfigured peripheral, a dropped address write), so this asks instead
            // whether anything works at all. A NAK counts as a completed transfer, so
            // absent devices cannot trip it.
            if(deadBus) {
                // Alternate the cheap fix and the expensive one.
                if((x_.deadBus.resuscitations() % 2U) == 1U) {
                    UC_LOG_W(
                      "i2c{} {} transfers failed in a row with no success -- "
                      "re-initialising the peripheral (SDA {}, SCL {})",
                      Hw::instance,
                      DeadBusFailures,
                      std::string_view{Recovery::sdaIsHigh() ? "high" : "LOW"},
                      std::string_view{Recovery::sclIsHigh() ? "high" : "LOW"});
                    reset();
                } else {
                    UC_LOG_W("i2c{} still dead after a re-initialise -- full bus recovery",
                             Hw::instance);
                    requestRecovery();
                }
                return;
            }

            // Post-abort settle: bus was sick, wait before starting next transaction
            if(!settled) { return; }

            // Nothing active: a stuck SDA is looked for on every idle turn, not only when a
            // request waits, because a stuck bus parks every device as absent and that
            // leaves the queue empty for seconds. Then whatever waits may start, on the same
            // terms submit() starts it (tryStart_). A recovery begun here fails what is queued then;
            // what is queued while it runs waits for it, and starts after the recovery's re-init.
            if constexpr(Deadlines) {
                mask_.lock();
                expireDeadlines_(now);
                mask_.unlock();
            }

            if(!active_) {
                if(Recovery::checkBusStuck(now)) {
                    // A recovery begun, like requestRecovery(): what is queued now fails (a bus that stays stuck
                    // still answers every request); what is queued while it runs waits for its re-init.
                    mask_.lock();
                    x_.drained += queue_.drain(queue_.size(), inv_);
                    mask_.unlock();
                    return;
                }
                mask_.lock();
                tryStart_(now);
                mask_.unlock();
                return;
            }

            // Active transaction: check for timeout
            if(now > timeoutTime_) {
                mask_.lock();
                // Read again, masked: the interrupt may have finished the request (and started
                // the next, with a later deadline) since the check above, which also read the
                // 64-bit timeoutTime_ unmasked. Failing it now would fail the wrong request or
                // complete the finished one twice.
                if(!active_ || now <= timeoutTime_) {
                    mask_.unlock();
                    return;
                }
                ++x_.timeouts;
                if constexpr(Features.timeoutSnapshot) { lastTimeout_ = snapshot_(); }
                Hw::logTimeout(now);
                Hw::timeoutAbort();
                // a stop the caller asked for (cancel, deadline) that never completed: still that outcome
                complete(stoppingResult_(Result::failed));
                mask_.unlock();
            }
        }

        // Request a full bus recovery sequence non-blocking.
        // Safe to call at any time. Any active transaction is immediately failed, and so is everything queued;
        // what a failure callback submits from here waits for the recovery and starts after it.
        static void requestRecovery() {
            mask_.lock();
            auto const outer   = resetting_;
            resetting_         = true;
            auto const waiting = queue_.size();
            failActive_();
            x_.drained += queue_.drain(waiting, inv_);
            Recovery::begin();
            resetting_ = outer;
            mask_.unlock();
        }

        static bool isRecovering() { return Recovery::isActive(); }

        /// Failures with no success between them, and how often the watchdog stepped in.
        static std::uint32_t consecutiveFailures() { return x_.deadBus.consecutiveFailures(); }

        static std::uint32_t resuscitations()
            requires(Features.counters)
        {
            return x_.deadBus.resuscitations();
        }

        // -- fault counters -------------------------------------------------------------------
        //
        // Kept as numbers rather than only logged: the fault log's rate limiter shares one
        // budget across every kind of fault, and the log transport may drop lines under load,
        // so on a busy bus the log cannot say how often any of these happened. A poller reads
        // them and reports when one moves.

        /// Transactions turn() gave up on after the transfer timeout.
        static std::uint32_t timeouts()
            requires(Features.counters)
        {
            return x_.timeouts;
        }

        /// Requests accepted for the wire since boot: what tells "nothing talks on this bus"
        /// from "nothing went wrong on it".
        static std::uint32_t transfers()
            requires(Features.counters)
        {
            return x_.transfers;
        }

        /// Failures after which SDA read low the moment the master went idle. Each one defers
        /// the next start; whether the line is really held is the idle watchdog's to decide,
        /// and Recovery::idleStuck() counts the times it did.
        static std::uint32_t sdaLowAfterAbort()
            requires(Features.counters)
        {
            return x_.sdaLowAfterAbort;
        }

        /// Queued requests failed without going on the wire (a recovery emptying the queue).
        static std::uint32_t drainedRequests()
            requires(Features.counters)
        {
            return x_.drained;
        }

        /// The block's handshake waits that ran out of their bound (the chip's: RP IC_ENABLE_STATUS, SAM SYNCBUSY).
        static std::uint32_t waitsExhausted() { return Hw::waitsExhausted(); }

        /// Times a start rewrote the block's timing for a device at another clock (PerDeviceClock only, 0 without).
        static std::uint32_t clockSwitches() {
            if constexpr(PerDeviceClock) {
                return Hw::clockSwitches();
            } else {
                return 0;
            }
        }

        static TimeoutSnapshot const& lastTimeout()
            requires(Features.timeoutSnapshot)
        {
            return lastTimeout_;
        }

        /// lastTimeout() as a log line, the chip's: `hw` is its block's registers.
        static void logLastTimeout()
            requires(Features.timeoutSnapshot)
        {
            Hw::logLastTimeout(lastTimeout_);
        }

        /// Interrupt entries that found nothing to do for the state the request was in (the chip's
        /// onIsr counts them).
        static std::uint32_t spuriousIsr() { return spuriousIsr_; }

        /// The longest a request waited for an interrupt since the last call: from its start to
        /// its first entry, and between two entries of one request. Taken and cleared.
        struct Latency {
            std::uint32_t firstIsrUs{};
            std::uint32_t isrGapUs{};
        };

        static Latency takeLatency()
            requires(Features.latency)
        {
            mask_.lock();
            LatencyTicks const l = x_.latency;
            x_.latency           = {};
            mask_.unlock();
            return Latency{::Kvasir::Bus::usOfTicks<Clock>(l.firstIsr),
                           ::Kvasir::Bus::usOfTicks<Clock>(l.isrGap)};
        }

        /// One kind of fault at one address with one cause, for the log rate limiter.
        template<typename Cause = std::uint32_t>
        static std::uint32_t faultKey(Fault kind,
                                      Cause cause = 0) {
            return Kvasir::rateLimitKey(kind, currentRequest_.address, cause);
        }

        // -- what the chip's interrupt and hooks share ---------------------------------------------

        // outside the optional parts: the chips' interrupts write it by name
        inline static std::uint32_t spuriousIsr_{};

        // only odr-used (so only defined) on a tracked bus
        inline static std::uint16_t ticketCounter_{};
        inline static bool
          cancelPending_{};               // our stop is out; the interrupt completes the request
        inline static Result stopAs_{};   // cancelled or timedOut, for that completion

        /// The request on the wire is over: the block let go (Hw::finish), `state_` idle, the
        /// dead-bus rule, `active_` false, its callback, then - unless the callback began a recovery, or the
        /// failure left the bus busy or SDA low, which only defers - the next request. Interrupt masked, or in the
        /// ISR.
        static void complete(Result result) {
            if constexpr(Tracked) { cancelPending_ = false; }
            Hw::finish();

            // The request is over before its callback runs. An interrupt raised between the
            // ISR's status read and the mask above can already be pending in the NVIC
            // (RP: ENABLE.ABORT after a NAK raises a second TX_ABRT, ABRT_USER_ABRT, by itself),
            // and the re-entry has to find no request to finish: left at `receiving` it read
            // the empty RX FIFO as the byte asked for and completed the NAKed request again.
            state_ = State::idle;

            // A fault counts towards a dead bus, ok and NAK (a working wire) reset it, a cancel and a deadline cut
            // leave it (Bus/DeadBus.hpp). Our own reset's failures come through failActive_, which notes nothing.
            x_.deadBus.note(result, false);

            // Over before its callback: a reset() or requestRecovery() from the callback finds nothing on the wire
            // to fail a second time. Nothing starts while it runs (tryStart_); the start comes below.
            active_ = false;
            inv_.invoke(currentRequest_.callback, result);
            // the callback asked for a recovery: what waits is the recovery's
            if(Recovery::isActive()) { return; }

            if(result != Result::ok) {
                // Guard against cascading timeouts: if the master is still active after
                // aborting, the bus may still be held. Defer startNext() for a brief settle.
                // This is the normal path after a NACK (the STOP is still propagating),
                // so it is not logged; the abort itself was.
                if(Hw::busActive()) {
                    Recovery::deferSettle(Clock::now() + std::chrono::milliseconds{1});
                    return;
                }

                // The master is idle: a STOP has gone out, so SDA should be on its way up.
                // One look at it this early is not a stuck bus: on a long, heavily loaded
                // wire the pull-ups take their time, and this read is the instant after the
                // master went idle. So it only defers the next start; the idle watchdog
                // (checkBusStuck) then asks for a recovery if the line is still held after
                // kStuckThreshold, and tryStart_ starts nothing while SDA is low.
                //
                // SDA only: a momentarily low SCL here is ordinary while the abort
                // settles. A genuinely held clock is left to the idle watchdog.
                if(!Recovery::sdaIsHigh()) {
                    ++x_.sdaLowAfterAbort;
                    Recovery::deferSettle(Clock::now() + std::chrono::milliseconds{1});
                    return;
                }
            }
            startNext_();
        }

        /// The interrupt: how long the request waited for this entry, then the chip's work.
        static void onIsr() {
            if constexpr(Stamped) {
                auto const now = Clock::now();
                if constexpr(Features.latency) {
                    // How long the request waited for this entry: from its start for the first,
                    // from the entry before for the rest. A byte at 400 kHz is ~25 us, so anything
                    // in milliseconds is the interrupt held off, or a part stretching the clock.
                    // In the clock's own ticks (Bus/Ticks.hpp): microseconds only where takeLatency() reads them.
                    auto&      st     = x_.stamps;
                    auto const waited = ::Kvasir::Bus::ticks32(
                      now - (st.isrEntries == 0 ? st.requestStart : st.lastIsr));
                    auto& longest = st.isrEntries == 0 ? x_.latency.firstIsr : x_.latency.isrGap;
                    if(state_ != State::idle) { ::Kvasir::Bus::raiseMax(longest, waited); }
                }
                ++x_.stamps.isrEntries;
                x_.stamps.lastIsr = now;
            }
            Hw::onIsr();
        }

    private:
        /// The interrupt entries of the request on the wire, for takeLatency() and the timeout snapshot.
        static constexpr bool Stamped = Features.latency || Features.timeoutSnapshot;

        struct IsrStamps {
            std::uint32_t isrEntries{};
            tp            lastIsr{};
            tp            requestStart{};
        };

        // The longest waits, in the clock's ticks: microseconds would be a 64-bit division per entry.
        struct LatencyTicks {
            std::uint32_t firstIsr{};
            std::uint32_t isrGap{};
        };

        template<typename T>
        using Counter = ::Kvasir::detail::StatCount<Features.counters, T>;

        /// The dead-bus watch and the optional parts as members of ONE static object: a static member takes a byte
        /// even when empty, a [[no_unique_address]] member of a struct takes none (the SPI core's Extras).
        struct Extras {
            // logic: the streak, and the resuscitation count that picks the reaction
            ::Kvasir::Bus::DeadBusWatch<DeadBusFailures> deadBus{};
            [[no_unique_address]] Counter<std::uint32_t> timeouts{};
            [[no_unique_address]] Counter<std::uint32_t> transfers{};
            [[no_unique_address]] Counter<std::uint32_t> sdaLowAfterAbort{};
            [[no_unique_address]] Counter<std::uint32_t> drained{};
            [[no_unique_address]] std::conditional_t<Stamped, IsrStamps, ::Kvasir::detail::NoStamp>
              stamps{};
            [[no_unique_address]] std::
              conditional_t<Features.latency, LatencyTicks, ::Kvasir::detail::NoStamp> latency{};
        };

        inline static Extras x_{};

        /// The timeout snapshot is an object of its own, not a member of x_: the optimiser drops the fields of it no
        /// one reads (a release image reads at most the chip's status word) only from an object it can split into
        /// its fields, and x_ is not one (its members are handed to the parts). In x_ the two microsecond fields
        /// stayed, with their 64-bit divisions: +1.7 KB on a Cortex-M0+. Off, nothing names it and it
        /// is not in the image.
        inline static std::
          conditional_t<Features.timeoutSnapshot, TimeoutSnapshot, ::Kvasir::detail::NoStamp>
            lastTimeout_{};

        /// The request on the wire as it timed out, before the abort: the block's registers first, then the core's.
        static TimeoutSnapshot snapshot_() {
            auto const hw  = Hw::snapshot();
            auto const now = Clock::now();
            return TimeoutSnapshot{
              .hw         = hw,
              .sent       = static_cast<std::uint16_t>(sendIndex_),
              .toSend     = static_cast<std::uint16_t>(currentRequest_.sendBytes()),
              .received   = static_cast<std::uint16_t>(receivedCount_),
              .toReceive  = static_cast<std::uint16_t>(currentRequest_.receiveData.size()),
              .address    = currentRequest_.address,
              .state      = static_cast<std::uint8_t>(state_),
              .sdaHigh    = Recovery::sdaIsHigh(),
              .sclHigh    = Recovery::sclIsHigh(),
              .isrEntries = x_.stamps.isrEntries,
              .usSinceIsr = static_cast<std::uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - x_.stamps.lastIsr)
                  .count()),
              .usAge = static_cast<std::uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - x_.stamps.requestStart)
                  .count()),
            };
        }

        /// What a request stopped by us ends as when the transfer timeout catches it first.
        static Result stoppingResult_(Result otherwise) {
            if constexpr(Tracked) {
                if(cancelPending_) { return stopAs_; }
            }
            return otherwise;
        }

        /// Interrupt masked: the request on the wire past its deadline is stopped like a cancel (timedOut);
        /// queued ones past theirs become tombstones with their callback (timedOut).
        static void expireDeadlines_(tp now)
            requires(Deadlines)
        {
            if(active_ && state_ != State::idle && !cancelPending_
               && now > currentRequest_.deadline)
            {
                cancelPending_ = true;
                stopAs_        = Result::timedOut;
                Hw::requestStop();
            }
            // What a callback submits waits for turn()'s start right after this sweep (and its stuck-bus look).
            x_.drained += queue_.expireQueued(now, inv_);
        }

        /// reset(), or the re-init that ends a recovery (`failQueued` false: what waited for it stays). What was
        /// queued when it was called fails; what a callback submits meanwhile starts once the block is back.
        static void reset_(bool failQueued) {
            mask_.lock();
            auto const outer   = resetting_;
            resetting_         = true;
            auto const waiting = failQueued ? queue_.size() : std::size_t{0};
            failActive_();
            Recovery::resetState();
            x_.drained += queue_.drain(waiting, inv_);
            Hw::reinit();
            resetting_ = outer;
            if(!outer) { tryStart_(Clock::now()); }
            // the initial set-up's NVIC enable, unless reset() runs inside a masked section
            mask_.unlock();
        }

        /// The request on the wire, if there is one, is over: its callback runs once, as
        /// failed. The block is left for the caller to reset or abort, and the dead-bus
        /// streak is not touched -- our own reset's failure (Bus/DeadBus.hpp), not a request
        /// reporting on the bus. Interrupts disabled.
        static void failActive_() {
            if(!active_) { return; }
            active_ = false;
            state_  = State::idle;
            // a stop of ours still out (cancel, deadline) ends with this request: the next one starts with none
            if constexpr(Tracked) { cancelPending_ = false; }
            inv_.invoke(currentRequest_.callback, Result::failed);
        }

        /// Start the next queued request if the bus may take one: no reset() or
        /// requestRecovery() failing requests, nothing on the wire, no callback running, no recovery running, the
        /// post-abort settle over, SDA released, and something waiting. The one definition of
        /// "may start", shared by submit() and turn(); the ISR's own startNext_() after a
        /// success needs none of these checks. Interrupts disabled.
        static bool tryStart_(tp now) {
            if(resetting_ || active_ || inv_.inCallback() || Recovery::isActive()
               || !Recovery::isPastSettle(now))
            {
                return false;
            }
            if(queue_.empty() || !Recovery::sdaIsHigh()) { return false; }
            startNext_();
            return true;
        }

        static void startNext_() {
            // tombstones dropped, the out-of-time failed (timedOut): their callbacks' submits only queue
            Request       req{};
            std::uint32_t expired = 0;
            bool const    live    = queue_.popLive(req, inv_, expired);
            x_.drained += expired;
            if(!live) {
                active_ = false;
                return;
            }
            currentRequest_ = req;
            active_         = true;
            sendIndex_      = 0;
            receivedCount_  = 0;
            if constexpr(Stamped) { x_.stamps.isrEntries = 0; }
            // Every start goes through here -- submit(), turn(), and the ISR chaining the
            // next request after a success -- and each one is the bus seen free.
            Recovery::noteBusFree();

            auto const totalBytes
              = currentRequest_.sendBytes() + currentRequest_.receiveData.size();
            auto const started = Clock::now();
            if constexpr(Stamped) { x_.stamps.requestStart = started; }
            timeoutTime_ = started + Hw::transferTimeout(totalBytes, currentRequest_);

            // Nothing to transfer: not something a driver asks for, and no block finishes it.
            if(totalBytes == 0) {
                complete(Result::failed);
                return;
            }
            Hw::start(currentRequest_);
        }
    };
}}   // namespace Kvasir::I2C
