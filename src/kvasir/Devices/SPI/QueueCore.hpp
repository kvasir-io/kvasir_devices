#pragma once
// The chip-independent half of a queued SPI master for any SPI block with DMA both ways; the chip
// packages' SPIQueued / Sercom_SPIQueued are this core over a hardware policy.
//
// Contexts: submit(), releaseHold(), turn(), reset() and takeLatency() may be called from the
// main loop, a callback or any interrupt; each runs under the policy's mask, depth-counted (Bus/MaskDepth.hpp). A
// callback runs where the completion is found (DMA interrupt on the RP, turn() on the SAM), from
// a copy of itself, and may submit(), releaseHold() or reset(); what it submits starts after it.
//
// The polled path: a policy with `transferPolled()` (QueueHw.hpp) moves a SHORT request - wire time at most
// `Timing::pollUpTo` - through the FIFOs itself when it would start at once anyway (bus idle, nothing queued, not
// inside a callback, or the continuation of the hold it belongs to). That runs inside submit(), under the mask, and
// the callback runs there too, before submit() returns: on a DMA master a short transfer costs ~10x its wire time in
// DMA set-up and completion interrupt, more than the polling. A driver already has to expect that (the DMA can finish
// before submit() returns); the masked time is the wire time plus the FIFO accesses.
#include "../Bus/Callbacks.hpp"
#include "../Bus/DeadBus.hpp"
#include "../Bus/MaskDepth.hpp"
#include "../Bus/RequestQueue.hpp"
#include "../Bus/Ticks.hpp"
#include "../BusTypes.hpp"
#include "../Features.hpp"
#include "../Log.hpp"
#include "QueueHw.hpp"
#include "kvasir/Hal/Spi.hpp"
#include "kvasir/Log/FaultLog.hpp"
#include "kvasir/Log/LogLimited.hpp"
#include "kvasir/Log/RateLimiter.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <algorithm>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace SPI {

    /// Microseconds per bit at `hz` in Q22.10, rounded up, made at compile time so wireTime()
    /// needs no 64-bit division on the target (~300 cycles on a Cortex-M0+).
    [[nodiscard]] constexpr std::uint32_t usPerBitQ10(std::uint32_t hz) {
        return hz == 0 ? 0 : (1'024'000'000U + hz - 1U) / hz;
    }

    /// Called from the completion interrupt, so CS goes up the moment the last bit is in.
    /// `select` is also the device's identity for a hold.
    struct Lines {
        void (*select)(){};
        void (*deselect)(){};
        void (*prepare)(){};   ///< before each transfer (a display's D/C); may be null
        void (*between)(){};   ///< between `command` and data, CS low; may be null

        constexpr bool operator==(Lines const&) const = default;
    };

    /// One frame, or one part of a frame (`hold`).
    ///
    /// - only `rx`: the master clocks out 0xFF (an SD card needs MOSI high, SD spec 9.10, 7.2).
    /// - `repeat` != 0: `tx` holds ONE frame that goes out `repeat` times.
    /// - `wide`: 16-bit frames, MSB first, from native-endian halfwords; spans in bytes.
    /// - `command`: 8-bit bytes first under the same CS, then `lines.between`, then the data.
    /// - `hold`: CS stays low and the device's next submit() runs before anything queued. Ends
    ///   with a request without `hold`, a failure, releaseHold() or the hold timeout.
    template<typename Setup, std::size_t CallbackSize>
    struct Request {
        Setup                                            setup{};
        Lines                                            lines{};
        std::span<std::byte const>                       command{};
        std::span<std::byte const>                       tx{};
        std::span<std::byte>                             rx{};
        std::uint32_t                                    repeat{};
        bool                                             wide{};
        bool                                             hold{};
        StaticFunction<void(Bus::Outcome), CallbackSize> callback{};
    };

    /// Request's fields with the tracked result, a ticket, the tombstone flag, `abortable` and an optional deadline
    /// (BusTypes.hpp): a type of its own, so a queue with the features off keeps Request - and its mangled name -
    /// untouched.
    template<typename Setup, std::size_t CallbackSize, typename TimePoint, bool Deadlines>
    struct TrackedRequest {
        Setup                                                                       setup{};
        Lines                                                                       lines{};
        std::span<std::byte const>                                                  command{};
        std::span<std::byte const>                                                  tx{};
        std::span<std::byte>                                                        rx{};
        std::uint32_t                                                               repeat{};
        bool                                                                        wide{};
        bool                                                                        hold{};
        StaticFunction<void(Bus::TrackedOutcome), CallbackSize>                     callback{};
        std::uint16_t                                                               ticket{};
        bool                                                                        cancelled{};
        bool                                                                        abortable{true};
        [[no_unique_address]] ::Kvasir::Detail::IfFeature<Deadlines, TimePoint, 30> deadline{
          TimePoint::max()};
    };

    /// A request resolved to what the policy's DMA set-up needs.
    struct Transfer {
        std::byte const* tx{};
        std::byte*       rx{};
        std::uint32_t    frames{};
        bool             txIncrement{};
        bool             rxIncrement{};
        bool             wide{};
    };

    struct QueueCoreDefaults {
        /// Timeout = 2 x wire time + this.
        static constexpr std::chrono::microseconds timeoutMargin{2000};
        /// Longest pause between two requests of a hold; renewed by each completed one.
        static constexpr std::chrono::milliseconds holdTimeout{50};
        /// Failures in a row after which the block is re-initialised.
        static constexpr std::uint32_t deadBusFailures = 40;
        /// A request whose wire time is at most this runs polled (a policy with `transferPolled()`, the comment at
        /// the top); 0 = never. Interrupts stay masked for that long.
        static constexpr std::chrono::microseconds pollUpTo{4};
    };

    /// Hw, the chip's half: the QueueHw concept (QueueHw.hpp, the contract written out there).
    template<QueueHw Hw,
             typename Clock,
             std::size_t Depth,
             std::size_t CallbackCapacity,
             typename Timing = QueueCoreDefaults>
    struct QueueCore {
        static constexpr std::size_t queueDepth   = Depth;
        static constexpr std::size_t callbackSize = CallbackCapacity;
        using Setup                               = typename Hw::Setup;
        using tp                                  = typename Clock::time_point;

        /// `Timing::queueFeatures` (BusTypes.hpp), else its defaults (latency and the timeout snapshot).
        static constexpr Bus::QueueFeatures features
          = Bus::QueueFeatures{KVASIR_CONFIG_OR(Timing, queueFeatures, Bus::QueueFeatures{})};

        static constexpr bool tracked = features.cancel || features.deadlines;
        using RequestT
          = std::conditional_t<tracked,
                               TrackedRequest<Setup, callbackSize, tp, features.deadlines>,
                               Request<Setup, callbackSize>>;
        /// The I2C bus's results (BusTypes.hpp); an SPI transfer never ends `nack`.
        using Result = std::conditional_t<tracked, Bus::TrackedOutcome, Bus::Outcome>;

        static constexpr auto timeoutMargin = std::chrono::duration_cast<std::chrono::microseconds>(
          KVASIR_CONFIG_OR(Timing, timeoutMargin, QueueCoreDefaults::timeoutMargin));
        static constexpr auto holdTimeout = std::chrono::duration_cast<std::chrono::microseconds>(
          KVASIR_CONFIG_OR(Timing,
                           holdTimeout,
                           std::chrono::microseconds{QueueCoreDefaults::holdTimeout}));
        static constexpr std::uint32_t deadBusFailures
          = KVASIR_CONFIG_OR(Timing, deadBusFailures, QueueCoreDefaults::deadBusFailures);
        static constexpr auto pollUpTo = std::chrono::duration_cast<std::chrono::microseconds>(
          KVASIR_CONFIG_OR(Timing, pollUpTo, QueueCoreDefaults::pollUpTo));
        /// The polled path (the comment at the top) exists: the policy has it and pollUpTo is not 0.
        static constexpr bool polls = requires(Transfer const& t) {
            { Hw::transferPolled(t) } -> std::same_as<bool>;
        } && pollUpTo.count() > 0;

        /// Taken before the abort.
        struct TimeoutSnapshot {
            typename Hw::Snapshot hw{};
            std::uint32_t         frames{};
            std::uint32_t         usExpected{};
            std::uint32_t         usAge{};
            std::uint32_t         queued{};
            bool                  held{};
        };

        struct Latency {
            std::uint32_t queueWaitUs{};   ///< longest from submit() to its transfer starting
            std::uint32_t lateUs{};        ///< longest a completion came after its wire time
        };

        // The same two as the queue keeps them while it runs: in the clock's own ticks, 32 bits,
        // saturated. Microseconds would be a 64-bit division per transfer; takeLatency() converts.
        struct LatencyTicks {
            std::uint32_t queueWait{};
            std::uint32_t late{};
        };

        // -- the bus's own API ---------------------------------------------------------------

        static constexpr bool          supportsWide = KVASIR_CONFIG_OR(Hw, supportsWide, true);
        static constexpr std::uint32_t maxFrames
          = KVASIR_CONFIG_OR(Hw, maxFrames, std::numeric_limits<std::uint32_t>::max());

        /// A hold needs a `select`: a holder with no identity would adopt any device's request.
        static constexpr bool wellFormed(RequestT const& r) {
            if(r.wide && !supportsWide) { return false; }
            if(r.hold && r.lines.select == nullptr) { return false; }
            std::size_t const unit = r.wide ? 2U : 1U;
            if(r.command.size() > maxFrames) { return false; }
            if(r.repeat != 0) {
                return r.tx.size() == unit && r.rx.empty() && r.repeat <= maxFrames;
            }
            if(!r.tx.empty() && !r.rx.empty() && r.tx.size() != r.rx.size()) { return false; }
            auto const bytes = std::max(r.tx.size(), r.rx.size());
            if(bytes == 0) { return !r.command.empty(); }
            return bytes % unit == 0 && bytes / unit <= maxFrames;
        }

        /// False (no callback) when full, a continuation already waits, or not wellFormed().
        /// Every request accepted gets exactly one callback.
        static bool submit(RequestT const& req) {
            if(!wellFormed(req)) {
                ++x_.refused;
                return false;
            }
            mask_.lock();
            if constexpr(polls) {
                if(startsAtOnce_(req)) {
                    auto const t = resolve_(req);
                    if(shortEnough_(req, t)) {
                        runPolled_(req, t);
                        mask_.unlock();
                        return true;
                    }
                }
            }
            bool accepted = false;
            if(holding_ && req.lines.select == holder_.select) {
                if(!haveContinuation_) {
                    continuation_     = entryOf_(req);
                    haveContinuation_ = true;
                    accepted          = true;
                }
            } else if(!queue_.full()) {
                queue_.push(entryOf_(req));
                accepted = true;
            }
            if(accepted) { tryStart_(); }
            mask_.unlock();
            return accepted;
        }

        /// submit(), with a ticket for cancel(); an invalid ticket (refused, no callback) when submit() would be false.
        static Bus::Ticket submitTracked(RequestT req)
            requires(tracked)
        {
            mask_.lock();
            req.ticket = Bus::nextTicket(ticketCounter_);
            mask_.unlock();
            return submit(req) ? Bus::Ticket{req.ticket} : Bus::Ticket{};
        }

        /// removed: queued (never started), or on the wire and stopped at once (SPI's abort is synchronous) - its
        /// callback (cancelled) has run before this returns. tooLate: completed already, an unknown ticket, or on
        /// the wire with `abortable` false (it runs to its end).
        static Bus::Cancel cancel(Bus::Ticket t)
            requires(features.cancel)
        {
            if(!t.valid()) { return Bus::Cancel::tooLate; }
            mask_.lock();
            auto r = Bus::Cancel::tooLate;
            if(active_ && current_.request.ticket == t.id) {
                if(current_.request.abortable) {
                    Hw::abort();
                    finish_(Result::cancelled);
                    if(!resetting_) { tryStart_(); }
                    r = Bus::Cancel::removed;
                }
            } else if(haveContinuation_ && continuation_.request.ticket == t.id) {
                haveContinuation_ = false;   // the hold stays until its timeout or releaseHold()
                auto const cb     = continuation_.request.callback;
                inv_.invoke(cb, Result::cancelled);
                r = Bus::Cancel::removed;
            } else if(queue_.cancelQueued(t.id, inv_)) {
                r = Bus::Cancel::removed;
            }
            mask_.unlock();
            return r;
        }

        static void releaseHold(Lines const& lines) {
            mask_.lock();
            if(holding_ && !active_ && holder_.select == lines.select) {
                endHold_();
                tryStart_();
            }
            mask_.unlock();
        }

        /// Once per main-loop turn: timeouts, dead-bus watchdog, held-back fault lines.
        static void turn() {
            if constexpr(requires { Hw::poll(); }) {
                mask_.lock();
                Hw::poll();
                mask_.unlock();
            }
            auto const now = Clock::now();

            mask_.lock();
            auto const dropped = x_.faultLog.takeSummary(now);
            bool const deadBus = x_.deadBus.takeIfDead();
            mask_.unlock();
            if(dropped != 0) { UC_LOG_W("spi{} +{} faults not logged", Hw::instance, dropped); }

            if(deadBus) {
                UC_LOG_W(
                  "spi{} {} transfers failed in a row with no success -- "
                  "re-initialising the block",
                  Hw::instance,
                  deadBusFailures);
                reset();
                return;
            }

            mask_.lock();
            if constexpr(features.deadlines) { expireDeadlines_(now); }
            if(active_ && now > deadline_) {
                ++x_.timeouts;
                if constexpr(features.timeoutSnapshot) { x_.lastTimeout = snapshot_(now); }
                KVASIR_LOG_LIMITED(x_.faultLog.note(Fault::timeout, now),
                                   UC_LOG_W,
                                   "spi{} transfer of {} frame(s) lost: {} us, {} expected",
                                   Hw::instance,
                                   frames_,
                                   usSince_(now),
                                   static_cast<std::uint32_t>(expected_.count()));
                Hw::abort();
                finish_(Result::failed);
                tryStart_();
            } else if(holding_ && !active_ && !haveContinuation_ && now > holdDeadline_) {
                ++x_.holdTimeouts;
                KVASIR_LOG_LIMITED(x_.faultLog.note(Fault::holdTimeout, now),
                                   UC_LOG_W,
                                   "spi{} a device held the bus {} us with nothing to send -- "
                                   "released",
                                   Hw::instance,
                                   static_cast<std::uint32_t>(holdTimeout.count()));
                endHold_();
                tryStart_();
            } else if(!active_) {
                tryStart_();
            }
            mask_.unlock();
        }

        /// Fails everything queued, then re-initialises the block. What a callback submits from
        /// here is not drained: it starts once the block is back.
        static void reset() {
            mask_.lock();
            auto const outer   = resetting_;
            resetting_         = true;
            auto const waiting = queue_.size();
            if(active_) {
                Hw::abort();
                finish_(Result::failed);
            }
            if(holding_) { endHold_(); }
            if(haveContinuation_) {
                haveContinuation_ = false;
                ++x_.drained;
                auto const cb = continuation_.request.callback;
                inv_.invoke(cb, Result::failed);
            }
            x_.drained += queue_.drain(waiting, inv_);
            Hw::reinit();
            configured_ = false;
            resetting_  = outer;
            if(!outer) { tryStart_(); }
            mask_.unlock();
        }

        [[nodiscard]] static bool busy() { return active_ || holding_ || !queue_.empty(); }

        // -- counters ---------------------------------------------------------------------------

        static std::uint32_t timeouts()
            requires(features.counters)
        {
            return x_.timeouts;
        }

        static std::uint32_t overruns()
            requires(features.counters)
        {
            return x_.overruns;
        }

        static std::uint32_t holdTimeouts()
            requires(features.counters)
        {
            return x_.holdTimeouts;
        }

        /// Failed without going on the wire.
        static std::uint32_t drainedRequests()
            requires(features.counters)
        {
            return x_.drained;
        }

        static std::uint32_t consecutiveFailures() { return x_.deadBus.consecutiveFailures(); }

        static std::uint32_t resuscitations()
            requires(features.counters)
        {
            return x_.deadBus.resuscitations();
        }

        /// Completions that came after their transfer's timeout.
        static std::uint32_t staleCompletions()
            requires(features.counters)
        {
            return x_.stale;
        }

        static std::uint32_t transfers()
            requires(features.counters)
        {
            return x_.transfers;
        }

        /// Not wellFormed(): a driver bug.
        static std::uint32_t refused()
            requires(features.counters)
        {
            return x_.refused;
        }

        static TimeoutSnapshot const& lastTimeout()
            requires(features.timeoutSnapshot)
        {
            return x_.lastTimeout;
        }

        static void logLastTimeout()
            requires(features.timeoutSnapshot)
        {
            [[maybe_unused]] auto const& t = x_.lastTimeout;
            UC_LOG_W(
              "spi{} last timeout: {} frame(s), {} us old, {} us expected, {} queued "
              "behind it, {}",
              Hw::instance,
              t.frames,
              t.usAge,
              t.usExpected,
              t.queued,
              std::string_view{t.held ? "in a hold" : "a frame of its own"});
            Hw::log(t.hw);
        }

        /// Taken and cleared.
        static Latency takeLatency()
            requires(features.latency)
        {
            mask_.lock();
            LatencyTicks const l = x_.latency;
            x_.latency           = {};
            mask_.unlock();
            return Latency{.queueWaitUs = ::Kvasir::Bus::usOfTicks<Clock>(l.queueWait),
                           .lateUs      = ::Kvasir::Bus::usOfTicks<Clock>(l.late)};
        }

        /// The policy's RX completion; `gen` from start(), a stale one is dropped.
        static void complete(std::uint32_t gen,
                             bool          overrun) {
            mask_.lock();
            if(!active_ || gen != generation_) {
                ++x_.stale;
                mask_.unlock();
                return;
            }
            auto const now = Clock::now();
            if constexpr(features.latency) {
                auto const late = now - startedAt_
                                - std::chrono::duration_cast<typename Clock::duration>(expected_);
                if(late.count() > 0) {
                    ::Kvasir::Bus::raiseMax(x_.latency.late, ::Kvasir::Bus::ticks32(late));
                }
            }
            if(overrun) {
                ++x_.overruns;
                KVASIR_LOG_LIMITED(x_.faultLog.note(Fault::overrun, now),
                                   UC_LOG_W,
                                   "spi{} receive overrun: the frame is failed",
                                   Hw::instance);
            }
            if(!overrun && inCommand_ && hasData_(current_.request)) {
                inCommand_ = false;
                if(current_.request.lines.between != nullptr) { current_.request.lines.between(); }
                startPhase_(transferOf_(current_.request), current_.request.setup.usPerBitQ10, now);
                mask_.unlock();
                return;
            }
            finish_(overrun ? Result::failed : Result::ok);
            if(!resetting_) { tryStart_(); }
            mask_.unlock();
        }

        static constexpr std::chrono::microseconds wireTime(std::uint32_t frames,
                                                            bool          wide,
                                                            std::uint32_t usPerBitQ10) {
            auto const bits = std::uint64_t{frames} * (wide ? 16U : 8U);
            return std::chrono::microseconds{
              static_cast<std::int64_t>((bits * usPerBitQ10 + 1023U) >> 10U)};
        }

    private:
        struct Entry {
            RequestT request{};
            [[no_unique_address]] std::
              conditional_t<features.latency, tp, ::Kvasir::Detail::NoStamp> queuedAt{};
        };

        enum class Fault : std::uint8_t { timeout = 1, overrun, holdTimeout };

        /// The queue's view of an entry (Bus/RequestQueue.hpp): its request.
        struct QueueSpec {
            using Result                    = QueueCore::Result;
            static constexpr bool tracked   = QueueCore::tracked;
            static constexpr bool deadlines = features.deadlines;

            static RequestT& request(Entry& e) { return e.request; }
        };

        inline static ::Kvasir::Bus::RequestQueue<Entry, queueDepth, Clock, QueueSpec> queue_{};
        inline static Entry                                                            current_{};
        inline static Entry                        continuation_{};
        inline static bool                         haveContinuation_{};
        inline static bool                         active_{};
        inline static bool                         inCommand_{};
        inline static bool                         holding_{};
        inline static Lines                        holder_{};
        inline static bool                         resetting_{};
        inline static ::Kvasir::Bus::Invoker       inv_{};
        inline static ::Kvasir::Bus::MaskDepth<Hw> mask_{};
        inline static bool                         configured_{};
        inline static Setup                        setup_{};
        inline static std::uint32_t                generation_{};
        inline static tp                           startedAt_{};
        inline static tp                           deadline_{};
        inline static tp                           holdDeadline_{};
        inline static std::chrono::microseconds    expected_{};
        inline static std::uint32_t                frames_{};
        // only odr-used (so only defined) on a tracked queue
        inline static std::uint16_t ticketCounter_{};

        /// The optional parts (Bus::QueueFeatures, and the log limiter without logging) as members
        /// of ONE static object, next to a counter that is always there: a static member takes a
        /// byte even when empty, a [[no_unique_address]] member of a struct takes none.
        template<typename T>
        using Counter = ::Kvasir::Detail::StatCount<features.counters, T>;

        struct Extras {
            // logic: the streak; the resuscitations a counter (Bus/DeadBus.hpp)
            ::Kvasir::Bus::DeadBusWatch<deadBusFailures, Counter<std::uint32_t>> deadBus{};
            [[no_unique_address]] Counter<std::uint32_t>                         timeouts{};
            [[no_unique_address]] Counter<std::uint32_t>                         overruns{};
            [[no_unique_address]] Counter<std::uint32_t>                         holdTimeouts{};
            [[no_unique_address]] Counter<std::uint32_t>                         drained{};
            [[no_unique_address]] Counter<std::uint32_t>                         stale{};
            [[no_unique_address]] Counter<std::uint32_t>                         transfers{};
            [[no_unique_address]] Counter<std::uint32_t>                         refused{};
            [[no_unique_address]] std::
              conditional_t<features.latency, LatencyTicks, ::Kvasir::Detail::NoStamp> latency{};
            [[no_unique_address]] std::conditional_t<features.timeoutSnapshot,
                                                     TimeoutSnapshot,
                                                     ::Kvasir::Detail::NoStamp> lastTimeout{};
            [[no_unique_address]] Kvasir::FaultLog<Clock, Fault>                faultLog{};
        };

        inline static Extras x_{};

        // Non-incrementing DMA source/sink: MOSI high while reading, what a write clocks in.
        alignas(2) inline static std::byte const fillBytes[2]{std::byte{0xFF},
                                                              std::byte{0xFF}};
        alignas(2) inline static std::byte sink_[2]{};

        /// Microseconds since the transfer on the wire started.
        static std::uint32_t usSince_(tp now) {
            return static_cast<std::uint32_t>(
              std::chrono::duration_cast<std::chrono::microseconds>(now - startedAt_).count());
        }

        static Entry entryOf_(RequestT const& r) {
            Entry e{.request = r};
            if constexpr(features.latency) { e.queuedAt = Clock::now(); }
            return e;
        }

        static TimeoutSnapshot snapshot_(tp now) {
            return TimeoutSnapshot{
              .hw         = Hw::snapshot(),
              .frames     = frames_,
              .usExpected = static_cast<std::uint32_t>(expected_.count()),
              .usAge      = usSince_(now),
              .queued     = static_cast<std::uint32_t>(queue_.size()),
              .held       = holding_,
            };
        }

        static bool hasData_(RequestT const& r) {
            return r.repeat != 0 || !r.tx.empty() || !r.rx.empty();
        }

        static Transfer commandOf_(RequestT const& r) {
            return Transfer{.tx          = r.command.data(),
                            .rx          = sink_,
                            .frames      = static_cast<std::uint32_t>(r.command.size()),
                            .txIncrement = true,
                            .rxIncrement = false,
                            .wide        = false};
        }

        static Transfer transferOf_(RequestT const& r) { return resolve_(r); }

        /// transferOf_()'s body; inlined into the polled path, which takes it once per request.
        [[gnu::always_inline]] static Transfer resolve_(RequestT const& r) {
            auto const unit = r.wide ? 2U : 1U;
            Transfer   t{.wide = r.wide};
            if(r.repeat != 0) {
                t.tx          = r.tx.data();
                t.txIncrement = false;
                t.frames      = r.repeat;
            } else {
                auto const bytes = std::max(r.tx.size(), r.rx.size());
                t.frames         = static_cast<std::uint32_t>(bytes / unit);
                t.tx             = r.tx.empty() ? fillBytes : r.tx.data();
                t.txIncrement    = !r.tx.empty();
            }
            t.rx          = r.rx.empty() ? sink_ : r.rx.data();
            t.rxIncrement = !r.rx.empty();
            return t;
        }

        /// Under the lock. Does nothing inside a callback: its submits start after it.
        static void tryStart_() {
            if(active_ || resetting_ || inv_.inCallback()) { return; }
            bool continuation = false;
            if(holding_) {
                if(!haveContinuation_) { return; }
                current_          = continuation_;
                haveContinuation_ = false;
                continuation      = true;
            } else {
                std::uint32_t expired = 0;   // tombstones dropped, the late failed
                bool const    live    = queue_.popLive(current_, inv_, expired);
                x_.drained += expired;
                if(!live) { return; }
            }
            start_(continuation);
        }

        /// Under the lock: `r` would start at once - what tryStart_() would pick next. A request with a deadline goes
        /// through the queue (its expiry is the queue's).
        [[gnu::always_inline]] static bool startsAtOnce_(RequestT const& r)
            requires(polls)
        {
            if(active_ || resetting_ || inv_.inCallback()) { return false; }
            if(holding_) {
                if(haveContinuation_ || r.lines.select != holder_.select) { return false; }
            } else if(!queue_.empty()) {
                return false;
            }
            if constexpr(features.deadlines) {
                if(r.deadline != tp::max()) { return false; }
            }
            return true;
        }

        /// The wire time of `command` + `t` at most pollUpTo. 64 frames a part keep bits x usPerBitQ10 in 32 bits
        /// down to the slowest PL022 / SERCOM clock (~2.3 kHz: 444 000 per bit, x 1536 bits).
        [[gnu::always_inline]] static bool shortEnough_(RequestT const& r,
                                                        Transfer const& t)
            requires(polls)
        {
            constexpr std::size_t partLimit = 64;
            if(r.command.size() > partLimit || t.frames > partLimit) { return false; }
            auto const bits
              = static_cast<std::uint32_t>(r.command.size() * 8U + t.frames * (t.wide ? 16U : 8U));
            return bits * r.setup.usPerBitQ10 <= pollUpToQ10;
        }

        static constexpr std::uint32_t pollUpToQ10 = [] {
            static_assert(
              pollUpTo <= std::chrono::milliseconds{1},
              "pollUpTo above 1 ms: interrupts stay masked for the whole polled transfer");
            return static_cast<std::uint32_t>(pollUpTo.count()) * 1024U;
        }();

        /// Under the lock, from submit(): `r` (resolved to `t`) through the policy's FIFOs, its callback, then what
        /// that callback submitted (by DMA: tryStart_() runs no polled transfer, so a callback chain does not nest).
        /// The steps of start_() + complete() + finish_() without the DMA, the clock and the interrupt; a plain ok
        /// frame (the common case) is finished here, a hold or a failure by finish_().
        [[gnu::always_inline]] static void runPolled_(RequestT const& r,
                                                      Transfer const& t)
            requires(polls)
        {
            bool const continuation = holding_;
            if(!continuation && !(configured_ && r.setup == setup_)) {
                Hw::configure(r.setup);
                setup_      = r.setup;
                configured_ = true;
            }
            if(r.lines.prepare != nullptr) { r.lines.prepare(); }
            if(!continuation && r.lines.select != nullptr) { r.lines.select(); }
            ++x_.transfers;
            bool ok = true;
            if(!r.command.empty()) {
                ok = Hw::transferPolled(commandOf_(r));
                if(ok && hasData_(r)) {
                    if(r.lines.between != nullptr) { r.lines.between(); }
                    ok = Hw::transferPolled(t);
                }
            } else {
                ok = Hw::transferPolled(t);
            }
            if(ok && !r.hold) {
                x_.deadBus.note(Result::ok, false);
                if(r.lines.deselect != nullptr) { r.lines.deselect(); }
                holding_ = false;
                // in place: StaticFunction calls a copy of its captures, and `r` is the caller's
                inv_.invoke(r.callback, Result::ok);
            } else {
                if(!ok) { lostPolled_(); }
                current_.request
                  = r;   // finish_() works on current_ (a hold keeps its lines from there)
                finish_(ok ? Result::ok : Result::failed);
            }
            if(haveContinuation_ || !queue_.empty()) { tryStart_(); }
        }

        /// A polled transfer whose frames stopped moving (the policy gave up): counted and logged as a timeout,
        /// the block aborted; the caller fails the request.
        static void lostPolled_()
            requires(polls)
        {
            auto const now = Clock::now();
            ++x_.timeouts;
            KVASIR_LOG_LIMITED(x_.faultLog.note(Fault::timeout, now),
                               UC_LOG_W,
                               "spi{} polled transfer stalled: no frame moved -- aborted",
                               Hw::instance);
            if constexpr(features.timeoutSnapshot) {
                x_.lastTimeout
                  = TimeoutSnapshot{.hw     = Hw::snapshot(),
                                    .queued = static_cast<std::uint32_t>(queue_.size()),
                                    .held   = holding_};
            }
            Hw::abort();
        }

        /// Under the lock: the request on the wire past its deadline is stopped (timedOut), queued ones past theirs
        /// become tombstones with their callback (timedOut).
        static void expireDeadlines_(tp now)
            requires(features.deadlines)
        {
            if(active_ && now > current_.request.deadline) {
                Hw::abort();
                finish_(Result::timedOut);
            }
            x_.drained += queue_.expireQueued(now, inv_);
            if(haveContinuation_ && now > continuation_.request.deadline) {
                haveContinuation_ = false;
                auto const cb     = continuation_.request.callback;
                inv_.invoke(cb, Result::timedOut);
            }
            if(!active_ && !resetting_) { tryStart_(); }
        }

        static void start_(bool continuation) {
            auto const& r  = current_.request;
            inCommand_     = !r.command.empty();
            auto const t   = inCommand_ ? commandOf_(r) : transferOf_(r);
            auto const now = Clock::now();

            if constexpr(features.latency) {
                auto const waited = now - current_.queuedAt;
                if(waited.count() > 0) {
                    ::Kvasir::Bus::raiseMax(x_.latency.queueWait, ::Kvasir::Bus::ticks32(waited));
                }
            }

            // Before CS goes low, so SCK settles to a new idle level first.
            if(!continuation && (!configured_ || !(r.setup == setup_))) {
                Hw::configure(r.setup);
                setup_      = r.setup;
                configured_ = true;
            }
            if(r.lines.prepare != nullptr) { r.lines.prepare(); }
            if(!continuation && r.lines.select != nullptr) { r.lines.select(); }

            active_ = true;
            ++x_.transfers;
            startPhase_(t, r.setup.usPerBitQ10, now);
        }

        static void startPhase_(Transfer const& t,
                                std::uint32_t   usPerBit,
                                tp              now) {
            frames_    = t.frames;
            expected_  = wireTime(t.frames, t.wide, usPerBit);
            startedAt_ = now;
            deadline_  = now + 2 * expected_ + timeoutMargin;
            ++generation_;
            Hw::start(t, generation_, &complete);
        }

        static void finish_(Result result) {
            active_        = false;
            auto const& r  = current_.request;
            auto const  cb = r.callback;
            // A fault counts towards a dead bus, ok resets it; a cancel, a deadline cut and the failure of our own
            // reset() (`resetting_`) leave it (Bus/DeadBus.hpp).
            x_.deadBus.note(result, resetting_);
            if(result == Result::ok) {
                if(r.hold) {
                    holding_      = true;
                    holder_       = r.lines;
                    holdDeadline_ = Clock::now() + holdTimeout;
                } else {
                    if(r.lines.deselect != nullptr) { r.lines.deselect(); }
                    holding_ = false;
                }
            } else {
                // A failure ends the device's transaction: the part sees CS go high.
                if(r.lines.deselect != nullptr) { r.lines.deselect(); }
                holding_ = false;
                if(haveContinuation_) {
                    haveContinuation_ = false;
                    ++x_.drained;
                    auto const ccb = continuation_.request.callback;
                    inv_.invoke(ccb, Result::failed);
                }
            }
            inv_.invoke(cb, result);
        }

        static void endHold_() {
            if(holder_.deselect != nullptr) { holder_.deselect(); }
            holding_ = false;
            holder_  = {};
        }
    };

}}   // namespace Kvasir::SPI
