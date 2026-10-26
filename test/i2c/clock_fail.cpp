/// What must not compile, built by ctest and judged by its error (test/i2c/CMakeLists.txt):
///   CLOCK_FAIL_SLOW_PART      a 100 kHz part on a 400 kHz bus that runs every device at one clock
///   CLOCK_FAIL_ABOVE_CHIP     Config::busClock above the chip's own I2cMaxClock
///   CLOCK_FAIL_BELOW_MIN      a perDeviceClock bus asked for a device clock below its minBaudRate
#include "Harness.hpp"

#include <cstdint>
#include <kvasir/Devices/I2C/Controller/QueuedFront.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Ds1307.hpp>
#include <ratio>

namespace {

/// A bus as the chip drivers are without I2CConfig::perDeviceClock: no timing in the request, and timing() only
/// refuses a part slower than the bus - the chips' own, kvasir_devices' QueuedShape (I2C/Controller/QueuedFront.hpp).
struct OneClockCfg {
    static constexpr std::uint32_t baudRate         = 400'000;
    static constexpr std::uint32_t clockSpeed       = 48'000'000;
    static constexpr bool          perDeviceClock   = false;
    static constexpr auto          maxBaudRateError = std::ratio<1, 100>{};
    static constexpr std::uint32_t minBaudRate      = baudRate;
};

/// Never asked: a bus without perDeviceClock computes no timing.
struct NoTiming {
    struct ClockTiming {};
};

struct OneClockBus : Kvasir::Test::FakeBus {
    using Shape = Kvasir::I2C::QueuedShape<OneClockCfg, Kvasir::Test::FakeClock, 16, NoTiming>;
    static constexpr std::uint32_t BaudRate = Shape::BaudRate;

    static consteval auto timing(std::uint32_t hz) { return Shape::timing(hz); }
};

/// A bus whose devices each get their own clock, down to minBaudRate and no lower.
struct PerDeviceCfg : OneClockCfg {
    static constexpr bool          perDeviceClock = true;
    static constexpr std::uint32_t minBaudRate    = 100'000;
};

/// A chip's timing as one number (QueuedShape's ClockTimingPolicy).
struct HzTiming {
    struct ClockTiming {
        std::uint32_t hz{};

        constexpr bool operator==(ClockTiming const&) const = default;
    };

    template<std::intmax_t N,
             std::intmax_t D>
    static consteval ClockTiming clockTiming(std::uint32_t,
                                             std::uint32_t hz,
                                             std::ratio<N,
                                                        D>) {
        return ClockTiming{hz};
    }
};

using PerDeviceShape
  = Kvasir::I2C::QueuedShape<PerDeviceCfg, Kvasir::Test::FakeClock, 16, HzTiming>;
static_assert(PerDeviceShape::timing(100'000) == HzTiming::ClockTiming{100'000},
              "minBaudRate itself is taken");

struct TooFast {
    static constexpr Kvasir::Units::Hertz busClock = Kvasir::Units::hertz(400'000);
};

}   // namespace

int main() {
#if defined(CLOCK_FAIL_SLOW_PART)
    [[maybe_unused]] Kvasir::I2C::
      Device<OneClockBus, Kvasir::Test::FakeClock, Kvasir::I2C::Chips::Ds1307> rtc{};
#elif defined(CLOCK_FAIL_ABOVE_CHIP)
    [[maybe_unused]] Kvasir::I2C::
      Device<Kvasir::Test::FakeBus, Kvasir::Test::FakeClock, Kvasir::I2C::Chips::Ds1307, TooFast>
        rtc{};
#elif defined(CLOCK_FAIL_BELOW_MIN)
    [[maybe_unused]] constexpr auto t = PerDeviceShape::timing(50'000);
#endif
    return 0;
}
