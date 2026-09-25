#pragma once
// Included by a chip's I2C driver, after its Io.hpp: the pin actions (makeInput, makeOutput,
// read, clear) are found by ADL when `Base` is known.
#include "../Log.hpp"
#include "kvasir/Util/RateLimiter.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>

namespace Kvasir { namespace I2C {

    // Non-blocking I2C bus recovery state machine, for any chip: it works the two lines as
    // GPIOs through the pin actions and knows nothing of the I2C block but what `Base` says.
    // Owns: clock-recovery pulses, SDA-stuck detection, post-abort settle gate.
    // Does NOT own the request queue or transaction state — the caller handles
    // draining/failing requests before calling begin().
    //
    // Base (the chip driver's Detail::I2CBase<Config>) has:
    //   I2CConfig::sdaPinLocation, sclPinLocation, baudRate
    //   Instance             the block's number, for the log lines
    //   initStepPinConfig    gives the two pins back to the I2C block
    //   softAbortRequest     asks the block to let go of a transfer (applied by begin())
    //
    // Until 2026-09-20 this was chip_rp2350's rp_common/I2CBusRecovery.hpp, which is now an
    // alias of it; chip_atsam_common's Sercom_I2CQueued.hpp is its second user. It sat in
    // Kvasir_SDK (kvasir/Io/I2CLineRecovery.hpp) until 2026-09-21: I2C lives here.
    template<typename Base, typename Clock>
    struct LineRecovery {
        using base = Base;
        using tp   = typename Clock::time_point;

        // The lines are driven the way I2C drives them: low is the pad pulling down, high is
        // the pad let go and the bus pull-ups doing the rest (driveLow_ / release_ below).
        // A push-pull high would fight a slave that is still holding the line, and it is
        // the slave that is being freed here.
        //
        // With one exception, and only when that has failed: a slave that still holds SDA
        // after the nine clocks is not in the middle of a byte, it is waiting for a STOP -- and
        // a STOP is SDA rising while SCL is high, which an open-drain master cannot make against
        // a line that is held. Seen twice on one bench (i2c_testing, 2026-09-19): SDA held for
        // 700 s through 356 of these sequences, and a NAU7802 that takes SDA one clock after a
        // read of its address, lets go for one clock and takes it again. Both times the line
        // was free the moment SDA was pushed high for 20 us with SCL high (ForceStop below):
        // the pad against one open-drain pull-down, for the length of eight bits at 400 kHz.
        enum class Phase : std::uint8_t {
            Idle,
            Aborting,       // softAbortRequest issued; waiting 100 µs for STOP to propagate
            PinTakeover,    // SCL/SDA over to GPIO, both released (instantaneous)
            PulseLow,       // SCL low; waiting 20 µs
            PulseHigh,      // SCL released; waiting 20 µs; pulseCount_ decrements back to PulseLow
            StopSdaLow,     // SDA low; waiting 20 µs
            StopSdaHigh,    // SDA released; waiting 20 µs; then -> ForceStop
            ForceStop,      // SDA high by itself: -> Reinit. Still held: pushed high for 20 µs
            ForceRelease,   // SDA released again; waiting 20 µs; then -> Reinit
            Reinit,         // restore pin functions; caller must reset the peripheral
        };

        enum class TickResult { busy, idle, needsReinit };

        inline static Phase phase_{Phase::Idle};
        inline static int   pulseCount_{0};
        inline static tp    phaseDeadline_{};
        inline static tp    sickUntil_{};    // post-abort settle gate
        inline static tp    stuckSince_{};   // when a line was first seen low while idle
        inline static Kvasir::RateLimiter<Clock> log_{};   // a held-low SDA recovers in a loop
        inline static std::uint32_t              recoveries_{};
        inline static std::uint32_t              idleStuck_{};
        inline static std::uint32_t              forcedStops_{};

        // A line must be continuously low for this long before recovery fires.
        // Scale with baud rate: ~500 bit-periods gives comfortable margin
        // (a legitimate STOP settles SDA in ~0.5 bit-periods).
        //   100 kHz -> 5 ms,  400 kHz -> 1.25 ms
        // Floor at 1 ms to avoid spurious triggers from noise/polling jitter.
        // explicit std::max<std::uint32_t>: uint32_t differs between gcc and clang, breaking deduction
        static constexpr auto kStuckThreshold = std::chrono::microseconds{
          std::max<std::uint32_t>(1000U, 500'000'000U / base::I2CConfig::baudRate)};

        /// Backoff for a bus no clocking can free; reset once both lines read high again.
        static constexpr auto kBackoffMin = std::chrono::milliseconds{100};
        static constexpr auto kBackoffMax = std::chrono::milliseconds{2000};

        inline static tp                        retryNotBefore_{};
        inline static std::chrono::milliseconds backoff_{kBackoffMin};

        static bool isActive() { return phase_ != Phase::Idle; }

        /// Open-drain by hand: an output that is low, or an input that leaves the line to
        /// the pull-ups. The pad is never pushed high.
        template<typename Pin>
        static void driveLow_(Pin pin) {
            apply(makeOutput(pin));
        }

        template<typename Pin>
        static void release_(Pin pin) {
            apply(makeInput(pin));
        }

        static bool sdaIsHigh() {
            return get<0>(apply(read(base::I2CConfig::sdaPinLocation))) != 0;
        }

        /// A slave holding SCL cannot be clocked out of it: for reporting only.
        static bool sclIsHigh() {
            return get<0>(apply(read(base::I2CConfig::sclPinLocation))) != 0;
        }

        /// Recovery sequences begun since reset.
        static std::uint32_t recoveries() { return recoveries_; }

        /// Times SDA was found held low on an idle bus for kStuckThreshold. Each one asks for
        /// a recovery, which the backoff may hold off, so this and recoveries() can differ.
        static std::uint32_t idleStuck() { return idleStuck_; }

        /// Sequences that ended in a forced STOP because the nine clocks did not free SDA.
        static std::uint32_t forcedStops() { return forcedStops_; }

        /// Clocks left over from the nine when SDA came back; 0 means it never did.
        static int clocksLeft() { return pulseCount_; }

        /// SDA held low while the bus is idle: start recovery.
        ///
        /// SDA only. A briefly low SCL between transactions is normal (the block is
        /// disabled after every transaction, a STOP may still be propagating), and a
        /// genuinely held clock cannot be recovered by a master anyway.
        static bool checkBusStuck(tp now) {
            if(sdaIsHigh()) {
                stuckSince_ = tp{};
                backoff_    = kBackoffMin;
                return false;
            }
            if(stuckSince_ == tp{}) { stuckSince_ = now; }
            if(now - stuckSince_ >= kStuckThreshold) {
                KVASIR_LOG_LIMITED(log_.allow(0, now),
                                   UC_LOG_W,
                                   "i2c{} SDA stuck low while idle -- requesting recovery",
                                   base::Instance);
                stuckSince_ = tp{};
                ++idleStuck_;
                return beginThrottled(now);
            }
            return false;
        }

        // Returns true if the post-abort settle period has elapsed (or was never set).
        static bool isPastSettle(tp now) { return now >= sickUntil_; }

        // Defer the next transaction start by a brief settle period.
        static void deferSettle(tp until) { sickUntil_ = until; }

        /// Begin unless a recent attempt is still backing off; says whether it did.
        /// Every automatic trigger goes through this.
        static bool beginThrottled(tp now) {
            if(now < retryNotBefore_) { return false; }
            retryNotBefore_ = now + backoff_;
            backoff_        = backoff_ * 2 > kBackoffMax ? kBackoffMax : backoff_ * 2;
            begin();
            return true;
        }

        // Begin a full bus recovery sequence, unconditionally: the explicit
        // requestRecovery() asked for it, so it is not subject to the backoff.
        // The caller must have already failed/drained any active transactions.
        static void begin() {
            ++recoveries_;
            apply(base::softAbortRequest);
            phase_         = Phase::Aborting;
            phaseDeadline_ = Clock::now() + std::chrono::microseconds{100};
        }

        // Advance the recovery state machine.  Call once per main-loop tick.
        // Returns:
        //   idle        — recovery is not active, nothing to do
        //   busy        — recovery in progress, caller should return early
        //   needsReinit — recovery finished, caller must reset() the peripheral
        static TickResult tick(tp now) {
            if(phase_ == Phase::Idle) { return TickResult::idle; }
            if(now < phaseDeadline_) { return TickResult::busy; }

            switch(phase_) {
            case Phase::Aborting: phase_ = Phase::PinTakeover; [[fallthrough]];
            case Phase::PinTakeover:
                // Both released: the slave must be able to drive SDA while it shifts out,
                // and sdaIsHigh() must read the bus, not our own output. The OUT latches are
                // cleared first: on the RP chips makeOutput() sets OE before it clears OUT, so
                // a latch left high would push the line high for an instant in driveLow_().
                apply(clear(base::I2CConfig::sdaPinLocation));
                apply(clear(base::I2CConfig::sclPinLocation));
                release_(base::I2CConfig::sdaPinLocation);
                release_(base::I2CConfig::sclPinLocation);
                pulseCount_ = 9;
                phase_      = Phase::PulseLow;
                break;
            case Phase::PulseLow:
                driveLow_(base::I2CConfig::sclPinLocation);
                phaseDeadline_ = now + std::chrono::microseconds{20};
                phase_         = Phase::PulseHigh;
                break;
            case Phase::PulseHigh:
                release_(base::I2CConfig::sclPinLocation);
                phaseDeadline_ = now + std::chrono::microseconds{20};
                // Stop as soon as the slave has let SDA go; pulseCount_ keeps the rest.
                phase_ = (--pulseCount_ > 0 && !sdaIsHigh()) ? Phase::PulseLow : Phase::StopSdaLow;
                break;
            case Phase::StopSdaLow:
                // SDA is taken back for the STOP: low while SCL is high, then released.
                driveLow_(base::I2CConfig::sdaPinLocation);
                phaseDeadline_ = now + std::chrono::microseconds{20};
                phase_         = Phase::StopSdaHigh;
                break;
            case Phase::StopSdaHigh:
                release_(base::I2CConfig::sdaPinLocation);
                phaseDeadline_ = now + std::chrono::microseconds{20};
                phase_         = Phase::ForceStop;
                break;
            case Phase::ForceStop:
                // 20 us after the STOP's release: a line that is high has had its STOP. One that
                // is still low -- the clocks never freed it, or the slave took it again before
                // the STOP -- gets the forced one, unless SCL is held too, which nothing here cures.
                if(sdaIsHigh() || !sclIsHigh()) {
                    phase_ = Phase::Reinit;
                    break;
                }
                ++forcedStops_;
                apply(makeOutputInitHigh(base::I2CConfig::sdaPinLocation));
                phaseDeadline_ = now + std::chrono::microseconds{20};
                phase_         = Phase::ForceRelease;
                break;
            case Phase::ForceRelease:
                apply(clear(base::I2CConfig::
                              sdaPinLocation));   // the latch low again, as PinTakeover left it
                release_(base::I2CConfig::sdaPinLocation);
                phaseDeadline_ = now + std::chrono::microseconds{20};
                phase_         = Phase::Reinit;
                break;
            case Phase::Reinit:
                apply(base::initStepPinConfig);
                phase_ = Phase::Idle;
                return TickResult::needsReinit;
            default: break;
            }
            return TickResult::busy;
        }

        static void resetState() {
            phase_      = Phase::Idle;
            stuckSince_ = tp{};
        }

        /// How long the next attempt is held off for.
        static std::chrono::milliseconds backoff() { return backoff_; }
    };

}}   // namespace Kvasir::I2C
