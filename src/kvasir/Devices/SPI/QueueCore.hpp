#pragma once
// The chip-independent half of a queued SPI master for any SPI block with DMA both ways; the chip
// packages' SPIQueued / Sercom_SPIQueued are this core over a hardware policy.
//
// Contexts: submit(), releaseHold(), handler(), reset() and takeLatency() may be called from the
// main loop, a callback or any interrupt; each runs under the policy's mask, depth-counted. A
// callback runs where the completion is found (DMA interrupt on the RP, handler() on the SAM), from
// a copy of itself, and may submit(), releaseHold() or reset(); what it submits starts after it.
#include "../Log.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Util/RateLimiter.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

namespace Kvasir { namespace SPI {

    enum class TransferResult : std::uint8_t { failed, succeeded };

    /// CPOL/CPHA; the PL022's SPO/SPH (TRM DDI0194H 2.4.4), SERCOM CPOL/CPHA (DS40001882L 27.6.2.5).
    enum class ClockMode : std::uint8_t { _0, _1, _2, _3 };

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
        Setup                                              setup{};
        Lines                                              lines{};
        std::span<std::byte const>                         command{};
        std::span<std::byte const>                         tx{};
        std::span<std::byte>                               rx{};
        std::uint32_t                                      repeat{};
        bool                                               wide{};
        bool                                               hold{};
        StaticFunction<void(TransferResult), CallbackSize> callback{};
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
        static constexpr std::chrono::microseconds TimeoutMargin{2000};
        /// Longest pause between two requests of a hold; renewed by each completed one.
        static constexpr std::chrono::milliseconds HoldTimeout{50};
        /// Failures in a row after which the block is re-initialised.
        static constexpr std::uint32_t DeadBusFailures = 40;
    };

    /// Hw, the chip's half:
    ///   using Setup;   mode, dividers, frame size, `hz`, `usPerBitQ10`; equality comparable
    ///   static constexpr unsigned Instance;
    ///   static void mask(); static void unmask();    masks every interrupt that may submit()
    ///   static void configure(Setup const&);         only with nothing on the wire
    ///   static void start(Transfer const&, std::uint32_t gen, void (*done)(std::uint32_t, bool));
    ///                  both DMA directions; `done(gen, overrun)` from the RX completion
    ///   static void abort();                         stop both channels, wait (bounded) for the
    ///                  shifter to go idle, empty the RX FIFO (PL022: no flush but a reset, TRM 2.3.1)
    ///   static void reinit();
    ///   using Snapshot; static Snapshot snapshot(); static void log(Snapshot const&);
    /// and optionally:
    ///   static void poll();                          no completion interrupt: called masked
    ///   static constexpr bool SupportsWide;          false: no 16-bit frames
    ///   static constexpr std::uint32_t MaxFrames;    the DMA's count limit
    template<typename Hw,
             typename Clock,
             std::size_t QueueDepth_,
             std::size_t CallbackSize_,
             typename Timing = QueueCoreDefaults>
    struct QueueCore {
        static constexpr std::size_t QueueDepth   = QueueDepth_;
        static constexpr std::size_t CallbackSize = CallbackSize_;
        using Setup                               = typename Hw::Setup;
        using RequestT                            = Request<Setup, CallbackSize>;
        using Result                              = TransferResult;
        using tp                                  = typename Clock::time_point;

        static_assert(
          requires(Setup const& s) {
              { s.hz } -> std::convertible_to<std::uint32_t>;
              { s.usPerBitQ10 } -> std::convertible_to<std::uint32_t>;
          },
          "a policy's Setup carries hz and usPerBitQ10 (SPI::usPerBitQ10(hz))");

        static constexpr auto TimeoutMargin = [] {
            if constexpr(requires { Timing::TimeoutMargin; }) {
                return std::chrono::duration_cast<std::chrono::microseconds>(Timing::TimeoutMargin);
            } else {
                return QueueCoreDefaults::TimeoutMargin;
            }
        }();
        static constexpr auto HoldTimeout = [] {
            if constexpr(requires { Timing::HoldTimeout; }) {
                return std::chrono::duration_cast<std::chrono::microseconds>(Timing::HoldTimeout);
            } else {
                return std::chrono::microseconds{QueueCoreDefaults::HoldTimeout};
            }
        }();
        static constexpr std::uint32_t DeadBusFailures = [] {
            if constexpr(requires { Timing::DeadBusFailures; }) {
                return Timing::DeadBusFailures;
            } else {
                return QueueCoreDefaults::DeadBusFailures;
            }
        }();

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

        // -- the bus's own API ---------------------------------------------------------------

        static constexpr bool SupportsWide = [] {
            if constexpr(requires { Hw::SupportsWide; }) {
                return Hw::SupportsWide;
            } else {
                return true;
            }
        }();
        static constexpr std::uint32_t MaxFrames = [] {
            if constexpr(requires { Hw::MaxFrames; }) {
                return Hw::MaxFrames;
            } else {
                return std::numeric_limits<std::uint32_t>::max();
            }
        }();

        /// A hold needs a `select`: a holder with no identity would adopt any device's request.
        static constexpr bool wellFormed(RequestT const& r) {
            if(r.wide && !SupportsWide) { return false; }
            if(r.hold && r.lines.select == nullptr) { return false; }
            std::size_t const unit = r.wide ? 2U : 1U;
            if(r.command.size() > MaxFrames) { return false; }
            if(r.repeat != 0) {
                return r.tx.size() == unit && r.rx.empty() && r.repeat <= MaxFrames;
            }
            if(!r.tx.empty() && !r.rx.empty() && r.tx.size() != r.rx.size()) { return false; }
            auto const bytes = std::max(r.tx.size(), r.rx.size());
            if(bytes == 0) { return !r.command.empty(); }
            return bytes % unit == 0 && bytes / unit <= MaxFrames;
        }

        /// False (no callback) when full, a continuation already waits, or not wellFormed().
        /// Every request accepted gets exactly one callback.
        static bool submit(RequestT const& req) {
            if(!wellFormed(req)) {
                ++refused_;
                return false;
            }
            lock_();
            bool accepted = false;
            if(holding_ && req.lines.select == holder_.select) {
                if(!haveContinuation_) {
                    continuation_     = Entry{req, Clock::now()};
                    haveContinuation_ = true;
                    accepted          = true;
                }
            } else if(queue_.size() < queue_.max_size()) {
                queue_.push(Entry{req, Clock::now()});
                accepted = true;
            }
            if(accepted) { tryStart_(); }
            unlock_();
            return accepted;
        }

        static void releaseHold(Lines const& lines) {
            lock_();
            if(holding_ && !active_ && holder_.select == lines.select) {
                endHold_();
                tryStart_();
            }
            unlock_();
        }

        /// Once per main-loop turn: timeouts, dead-bus watchdog, held-back fault lines.
        static void handler() {
            if constexpr(requires { Hw::poll(); }) {
                lock_();
                Hw::poll();
                unlock_();
            }
            auto const now = Clock::now();

            lock_();
            auto const dropped = faultLog_.takeSummary(now);
            bool const deadBus = consecutiveFailures_ >= DeadBusFailures;
            if(deadBus) { consecutiveFailures_ = 0; }
            unlock_();
            if(dropped != 0) { UC_LOG_W("spi{} +{} faults not logged", Hw::Instance, dropped); }

            if(deadBus) {
                ++resuscitations_;
                UC_LOG_W(
                  "spi{} {} transfers failed in a row with no success -- "
                  "re-initialising the block",
                  Hw::Instance,
                  DeadBusFailures);
                reset();
                return;
            }

            lock_();
            if(active_ && now > deadline_) {
                ++timeouts_;
                lastTimeout_ = snapshot_(now);
                KVASIR_LOG_LIMITED(faultLog_.allow(rateLimitKey(Fault::timeout), now),
                                   UC_LOG_W,
                                   "spi{} transfer of {} frame(s) lost: {} us, {} expected",
                                   Hw::Instance,
                                   lastTimeout_.frames,
                                   lastTimeout_.usAge,
                                   lastTimeout_.usExpected);
                Hw::abort();
                finish_(Result::failed);
                tryStart_();
            } else if(holding_ && !active_ && !haveContinuation_ && now > holdDeadline_) {
                ++holdTimeouts_;
                KVASIR_LOG_LIMITED(faultLog_.allow(rateLimitKey(Fault::holdTimeout), now),
                                   UC_LOG_W,
                                   "spi{} a device held the bus {} us with nothing to send -- "
                                   "released",
                                   Hw::Instance,
                                   static_cast<std::uint32_t>(HoldTimeout.count()));
                endHold_();
                tryStart_();
            } else if(!active_) {
                tryStart_();
            }
            unlock_();
        }

        /// Fails everything queued, then re-initialises the block. What a callback submits from
        /// here is not drained: it starts once the block is back.
        static void reset() {
            lock_();
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
                ++drained_;
                auto const cb = continuation_.request.callback;
                invoke_(cb, Result::failed);
            }
            drainQueue_(waiting);
            Hw::reinit();
            configured_ = false;
            resetting_  = outer;
            if(!outer) { tryStart_(); }
            unlock_();
        }

        [[nodiscard]] static bool busy() { return active_ || holding_ || !queue_.empty(); }

        // -- counters ---------------------------------------------------------------------------

        static std::uint32_t timeouts() { return timeouts_; }

        static std::uint32_t overruns() { return overruns_; }

        static std::uint32_t holdTimeouts() { return holdTimeouts_; }

        /// Failed without going on the wire.
        static std::uint32_t drainedRequests() { return drained_; }

        static std::uint32_t consecutiveFailures() { return consecutiveFailures_; }

        static std::uint32_t resuscitations() { return resuscitations_; }

        /// Completions that came after their transfer's timeout.
        static std::uint32_t staleCompletions() { return stale_; }

        static std::uint32_t transfers() { return transfers_; }

        /// Not wellFormed(): a driver bug.
        static std::uint32_t refused() { return refused_; }

        static TimeoutSnapshot const& lastTimeout() { return lastTimeout_; }

        static void logLastTimeout() {
            [[maybe_unused]] auto const& t = lastTimeout_;
            UC_LOG_W(
              "spi{} last timeout: {} frame(s), {} us old, {} us expected, {} queued "
              "behind it, {}",
              Hw::Instance,
              t.frames,
              t.usAge,
              t.usExpected,
              t.queued,
              std::string_view{t.held ? "in a hold" : "a frame of its own"});
            Hw::log(t.hw);
        }

        /// Taken and cleared.
        static Latency takeLatency() {
            lock_();
            Latency const l = latency_;
            latency_        = {};
            unlock_();
            return l;
        }

        /// The policy's RX completion; `gen` from start(), a stale one is dropped.
        static void complete(std::uint32_t gen,
                             bool          overrun) {
            lock_();
            if(!active_ || gen != generation_) {
                ++stale_;
                unlock_();
                return;
            }
            auto const now = Clock::now();
            auto const late
              = std::chrono::duration_cast<std::chrono::microseconds>(now - startedAt_ - expected_)
                  .count();
            if(late > 0 && static_cast<std::uint32_t>(late) > latency_.lateUs) {
                latency_.lateUs = static_cast<std::uint32_t>(late);
            }
            if(overrun) {
                ++overruns_;
                KVASIR_LOG_LIMITED(faultLog_.allow(rateLimitKey(Fault::overrun), now),
                                   UC_LOG_W,
                                   "spi{} receive overrun: the frame is failed",
                                   Hw::Instance);
            }
            if(!overrun && inCommand_ && hasData_(current_.request)) {
                inCommand_ = false;
                if(current_.request.lines.between != nullptr) { current_.request.lines.between(); }
                startPhase_(transferOf_(current_.request), current_.request.setup.usPerBitQ10, now);
                unlock_();
                return;
            }
            finish_(overrun ? Result::failed : Result::succeeded);
            if(!resetting_) { tryStart_(); }
            unlock_();
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
            tp       queuedAt{};
        };

        enum class Fault : std::uint8_t { timeout = 1, overrun, holdTimeout };

        inline static Kvasir::Atomic::Queue<Entry, QueueDepth + 1> queue_{};
        inline static Entry                                        current_{};
        inline static Entry                                        continuation_{};
        inline static bool                                         haveContinuation_{};
        inline static bool                                         active_{};
        inline static bool                                         inCommand_{};
        inline static bool                                         holding_{};
        inline static Lines                                        holder_{};
        inline static bool                                         resetting_{};
        inline static bool                                         inCallback_{};
        inline static std::uint32_t                                lockDepth_{};
        inline static bool                                         configured_{};
        inline static Setup                                        setup_{};
        inline static std::uint32_t                                generation_{};
        inline static tp                                           startedAt_{};
        inline static tp                                           deadline_{};
        inline static tp                                           holdDeadline_{};
        inline static std::chrono::microseconds                    expected_{};
        inline static std::uint32_t                                frames_{};

        inline static std::uint32_t   timeouts_{};
        inline static std::uint32_t   overruns_{};
        inline static std::uint32_t   holdTimeouts_{};
        inline static std::uint32_t   drained_{};
        inline static std::uint32_t   consecutiveFailures_{};
        inline static std::uint32_t   resuscitations_{};
        inline static std::uint32_t   stale_{};
        inline static std::uint32_t   transfers_{};
        inline static std::uint32_t   refused_{};
        inline static Latency         latency_{};
        inline static TimeoutSnapshot lastTimeout_{};

        inline static Kvasir::RateLimiter<Clock> faultLog_{};

        // Non-incrementing DMA source/sink: MOSI high while reading, what a write clocks in.
        alignas(2) inline static std::byte const fill_[2]{std::byte{0xFF},
                                                          std::byte{0xFF}};
        alignas(2) inline static std::byte sink_[2]{};

        /// Depth-counted: only the outermost pair reaches the policy. An interrupt between the
        /// outermost read and write of the counter runs its own balanced pair first.
        static void lock_() {
            if(lockDepth_++ == 0) { Hw::mask(); }
        }

        static void unlock_() {
            if(--lockDepth_ == 0) { Hw::unmask(); }
        }

        static TimeoutSnapshot snapshot_(tp now) {
            return TimeoutSnapshot{
              .hw         = Hw::snapshot(),
              .frames     = frames_,
              .usExpected = static_cast<std::uint32_t>(expected_.count()),
              .usAge      = static_cast<std::uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - startedAt_).count()),
              .queued = static_cast<std::uint32_t>(queue_.size()),
              .held   = holding_,
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

        static Transfer transferOf_(RequestT const& r) {
            auto const unit = r.wide ? 2U : 1U;
            Transfer   t{.wide = r.wide};
            if(r.repeat != 0) {
                t.tx          = r.tx.data();
                t.txIncrement = false;
                t.frames      = r.repeat;
            } else {
                auto const bytes = std::max(r.tx.size(), r.rx.size());
                t.frames         = static_cast<std::uint32_t>(bytes / unit);
                t.tx             = r.tx.empty() ? fill_ : r.tx.data();
                t.txIncrement    = !r.tx.empty();
            }
            t.rx          = r.rx.empty() ? sink_ : r.rx.data();
            t.rxIncrement = !r.rx.empty();
            return t;
        }

        /// Under the lock. Does nothing inside a callback: its submits start after it.
        static void tryStart_() {
            if(active_ || resetting_ || inCallback_) { return; }
            bool continuation = false;
            if(holding_) {
                if(!haveContinuation_) { return; }
                current_          = continuation_;
                haveContinuation_ = false;
                continuation      = true;
            } else if(!queue_.pop_into(current_)) {
                return;
            }
            start_(continuation);
        }

        static void start_(bool continuation) {
            auto const& r  = current_.request;
            inCommand_     = !r.command.empty();
            auto const t   = inCommand_ ? commandOf_(r) : transferOf_(r);
            auto const now = Clock::now();

            auto const waited
              = std::chrono::duration_cast<std::chrono::microseconds>(now - current_.queuedAt)
                  .count();
            if(waited > 0 && static_cast<std::uint32_t>(waited) > latency_.queueWaitUs) {
                latency_.queueWaitUs = static_cast<std::uint32_t>(waited);
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
            ++transfers_;
            startPhase_(t, r.setup.usPerBitQ10, now);
        }

        static void startPhase_(Transfer const& t,
                                std::uint32_t   usPerBit,
                                tp              now) {
            frames_    = t.frames;
            expected_  = wireTime(t.frames, t.wide, usPerBit);
            startedAt_ = now;
            deadline_  = now + 2 * expected_ + TimeoutMargin;
            ++generation_;
            Hw::start(t, generation_, &complete);
        }

        static void invoke_(StaticFunction<void(TransferResult),
                                           CallbackSize> const& cb,
                            Result                              result) {
            if(!cb) { return; }
            auto const outer = inCallback_;
            inCallback_      = true;
            cb(result);
            inCallback_ = outer;
        }

        static void finish_(Result result) {
            active_        = false;
            auto const& r  = current_.request;
            auto const  cb = r.callback;
            if(result == Result::succeeded) {
                consecutiveFailures_ = 0;
                if(r.hold) {
                    holding_      = true;
                    holder_       = r.lines;
                    holdDeadline_ = Clock::now() + HoldTimeout;
                } else {
                    if(r.lines.deselect != nullptr) { r.lines.deselect(); }
                    holding_ = false;
                }
            } else {
                if(consecutiveFailures_ != std::numeric_limits<std::uint32_t>::max()) {
                    ++consecutiveFailures_;
                }
                // A failure ends the device's transaction: the part sees CS go high.
                if(r.lines.deselect != nullptr) { r.lines.deselect(); }
                holding_ = false;
                if(haveContinuation_) {
                    haveContinuation_ = false;
                    ++drained_;
                    auto const ccb = continuation_.request.callback;
                    invoke_(ccb, Result::failed);
                }
            }
            invoke_(cb, result);
        }

        static void endHold_() {
            if(holder_.deselect != nullptr) { holder_.deselect(); }
            holding_ = false;
            holder_  = {};
        }

        static void drainQueue_(std::size_t n) {
            Entry e{};
            while(n-- != 0 && queue_.pop_into(e)) {
                ++drained_;
                invoke_(e.request.callback, Result::failed);
            }
        }
    };

}}   // namespace Kvasir::SPI
