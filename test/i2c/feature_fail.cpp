/// What must not compile, built by ctest and judged by its error (test/i2c/CMakeLists.txt):
///   FEATURE_FAIL_ENABLED   a device with Config::enabled() on a bus without EngineFeatures::switchable
#include "Harness.hpp"

#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Ds1307.hpp>

namespace {

/// A bus as a chip package's driver is: no Features member, so the defaults - switchable off.
struct PlainBus : Kvasir::Test::FakeBus {
    static constexpr Kvasir::I2C::EngineFeatures Features{};
};

struct Switched {
    static bool enabled() { return true; }
};

}   // namespace

int main() {
#if defined(FEATURE_FAIL_ENABLED)
    [[maybe_unused]] Kvasir::I2C::
      Device<PlainBus, Kvasir::Test::FakeClock, Kvasir::I2C::Chips::Ds1307, Switched> rtc{};
#endif
    return 0;
}
