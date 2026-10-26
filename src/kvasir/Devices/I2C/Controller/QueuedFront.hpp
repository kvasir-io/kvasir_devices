#pragma once
// The parts of a queued I2C master that are the same on every chip: chip_rp_common I2CQueued and chip_atsam_common
// Sercom_I2CQueued are their hardware half (registers, the Hw policy, the QueueCore over it) and these:
//
//     template<...>
//     struct I2CBehaviorQueued
//       : Detail::I2CQueuedHalf<...>                       // I2CBase, QueuedShape, Recovery, Hw, Core
//       , Kvasir::I2C::QueuedFront<typename Detail::I2CQueuedHalf<...>::Core> {
//         using Isr = ...;                                  // the chip's interrupt line(s)
//     };
//
//  * QueuedShape - what the bus is, from its config: the features, the request type, the clock a device gets
//    (timing(hz)). The half derives from it: its Hw needs the request type before the core exists.
//  * TimingCache - the SCL timing in the block on a perDeviceClock bus: written only when it changes.
//  * QueuedFront - the API over the core, one line each.
//
// Why the front is a base of its own and not over the whole master: the base clause needs the core, so a complete
// Hw, and a policy nested in the class it is a base of is incomplete there (queue_core_test names its core where the
// policy is complete for the same reason). No name is declared in two of the bases: `Bus::Core` is the half's,
// `Bus::Request` the shape's, `Bus::Result` the front's.
#include "../../Bus/QueuedFront.hpp"
#include "../../BusTypes.hpp"
#include "QueueCore.hpp"
#include "QueueHw.hpp"
#include "Request.hpp"
#include "kvasir/Util/Config.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace I2C {

    namespace Detail {
        /// Not constexpr: reaching one in timing() is the compile error that says why.
        inline void i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus() {}

        inline void i2cDeviceClockBelowMinBaudRateOfTheBus() {}
    }   // namespace Detail

    /// A chip's SCL timing for one rate: `ClockTiming` (a value a request carries, `==` comparable) and
    /// `clockTiming(clockSpeed, hz, maxBaudRateError)` (consteval, its refusals the chip's).
    template<typename T>
    concept ClockTimingPolicy = std::is_default_constructible_v<typename T::ClockTiming>;

    /// What a queued bus is, from its resolved config (`I2CBase::I2CConfig`: baudRate, clockSpeed, perDeviceClock,
    /// maxBaudRateError, minBaudRate, and `queueFeatures` (Bus::QueueFeatures, BusTypes.hpp) if the config sets it).
    template<typename Cfg, typename Clock, std::size_t CallbackSize, ClockTimingPolicy Timing>
    struct QueuedShape {
        /// Re-exported from the config so a bus can size its traffic against the bandwidth.
        static constexpr auto BaudRate = Cfg::baudRate;

        /// The bus's optional parts, the same struct as the SPI master's (BusTypes.hpp); latency and the
        /// timeout snapshot on by default. Not `Features`: on a bus that name is the engine's (a port's EngineFeatures).
        static constexpr ::Kvasir::Bus::QueueFeatures BusFeatures = ::Kvasir::Bus::QueueFeatures{
          KVASIR_CONFIG_OR(Cfg, queueFeatures, ::Kvasir::Bus::QueueFeatures{})};

        /// Tickets / cancel() and per-request deadlines (queueFeatures.cancel, .deadlines).
        static constexpr bool Cancellable = BusFeatures.cancel;
        static constexpr bool Deadlines   = BusFeatures.deadlines;
        static constexpr bool Tracked     = Cancellable || Deadlines;

        /// Each device at its own clock (I2CConfig::perDeviceClock). Off, a request has no timing member and a start
        /// never looks at the block's clock registers: nothing of this is in the image.
        static constexpr bool PerDeviceClock = Cfg::perDeviceClock;

        using ClockTiming = typename Timing::ClockTiming;

        /// The timing of baudRate itself, what a request without one of its own carries.
        static constexpr ClockTiming DefaultTiming = [] {
            if constexpr(PerDeviceClock) {
                return Timing::clockTiming(Cfg::clockSpeed,
                                           static_cast<std::uint32_t>(BaudRate),
                                           Cfg::maxBaudRateError);
            } else {
                return ClockTiming{};
            }
        }();

        using Request = Queue::QueuedRequestFor<CallbackSize,
                                                typename Clock::time_point,
                                                Tracked,
                                                Deadlines,
                                                PerDeviceClock,
                                                ClockTiming,
                                                DefaultTiming>;

        /// What a request carries for a device clocked at most at `hz` (kvasir_devices' Device fills it in from
        /// Config::busClock / Chip::I2cMaxClock): the timing for min(hz, baudRate). On a bus without perDeviceClock a
        /// device slower than the bus is a compile error, and the answer is nothing.
        static consteval auto timing(std::uint32_t hz) {
            if constexpr(PerDeviceClock) {
                auto const f = std::min(hz, static_cast<std::uint32_t>(BaudRate));
                if(f < Cfg::minBaudRate) { Detail::i2cDeviceClockBelowMinBaudRateOfTheBus(); }
                return Timing::clockTiming(Cfg::clockSpeed, f, Cfg::maxBaudRateError);
            } else {
                if(hz < BaudRate) {
                    Detail::i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus();
                }
                return std::false_type{};
            }
        }
    };

    /// The timing a perDeviceClock bus's block holds: a start writes a request's timing only when needs() says
    /// it differs (or nothing is known, after a reset), then took(). An object of the chip's Hw: two buses
    /// never share one.
    template<typename ClockTiming>
    struct TimingCache {
        [[nodiscard]] constexpr bool needs(ClockTiming const& t) const {
            return !valid_ || !(t == timing_);
        }

        constexpr void took(ClockTiming const& t) {
            timing_ = t;
            valid_  = true;
            ++switches_;
        }

        /// The block is back at baudRate's timing (a reset): the next start writes its own.
        constexpr void invalidate() { valid_ = false; }

        /// Times a start rewrote the block's timing for a device at another clock.
        [[nodiscard]] constexpr std::uint32_t switches() const { return switches_; }

    private:
        ClockTiming   timing_{};
        bool          valid_{};
        std::uint32_t switches_{};
    };

    /// The API of a queued I2C master over its `Core` (a QueueCore): the members both buses share are
    /// Bus::QueuedFront's (Bus/QueuedFront.hpp), the I2C-only ones here. `Request` is the shape's (QueuedShape), so
    /// the shared front gets it from the core and declares none.
    template<typename Core>
    struct QueuedFront : ::Kvasir::Bus::QueuedFront<Core, typename Core::Request> {
        using State = QueueState;

        /// A full bus recovery sequence, non-blocking. The transaction on the wire and what is queued now are
        /// failed at once; what is submitted later waits for the recovery's closing re-init.
        static void requestRecovery() { Core::requestRecovery(); }

        static bool isRecovering() { return Core::isRecovering(); }

        /// Failures after which SDA read low the moment the master went idle (Core::sdaLowAfterAbort).
        static std::uint32_t sdaLowAfterAbort()
            requires(Core::Features.counters)
        {
            return Core::sdaLowAfterAbort();
        }

        /// Times the block did not follow its own handshake within its bound (RP: IC_ENABLE_STATUS after a
        /// disable, SAM: SYNCBUSY), so a register write after it may have been dropped.
        static std::uint32_t disableWaitsExhausted() { return Core::waitsExhausted(); }

        /// Times a start rewrote the block's timing for a device at another clock (perDeviceClock only, 0 without).
        static std::uint32_t clockSwitches() { return Core::clockSwitches(); }

        /// Interrupt entries that found nothing to do for the state the request was in (the chip's Hw::onIsr).
        static std::uint32_t spuriousIsr() { return Core::spuriousIsr(); }

        static void onIsr() { Core::onIsr(); }

        static_assert(
          std::is_same_v<typename QueuedFront::Extends,
                         typename ::Kvasir::Bus::QueuedFront<Core,
                                                             typename Core::Request>::Extends>,
          "the MainLoop hook is the shared front's (an ambiguous Extends reads as absent)");
    };
}}   // namespace Kvasir::I2C
