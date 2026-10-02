/// What must not compile, built by ctest and judged by its error (test/i2c/CMakeLists.txt):
///   CLOCK_FAIL_SLOW_PART      a 100 kHz part on a 400 kHz bus that runs every device at one clock
///   CLOCK_FAIL_ABOVE_CHIP     Config::BusClock above the chip's own I2cMaxClock
#include "Harness.hpp"

#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Ds1307.hpp>

namespace {

/// A bus as the chip drivers are without I2CConfig::perDeviceClock: no timing in the request,
/// and timing() only refuses a part slower than the bus (chip_rp_common I2CQueued.hpp).
void i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus() {}

struct OneClockBus : Kvasir::Test::FakeBus {
    static constexpr std::uint32_t BaudRate = 400'000;

    static consteval auto timing(std::uint32_t hz) {
        if(hz < BaudRate) { i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus(); }
        return std::false_type{};
    }
};

struct TooFast {
    static constexpr Kvasir::Units::Hertz BusClock = Kvasir::Units::hertz(400'000);
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
#endif
    return 0;
}
