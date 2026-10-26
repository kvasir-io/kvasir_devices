#pragma once

/// The Config::enabled() descriptions both enabled_test (enabled() alone) and bridge_test (enabled() behind a bridge)
/// put a part on: a module the application says is fitted or not.
#include <cstdint>
#include <kvasir/Devices/I2C/Bridge.hpp>

namespace Kvasir::Test::EnabledFixtures {

/// Whether the module the part sits on is fitted, as the application knows it.
struct Fitted {
    static inline bool yes{false};
};

struct Enabled {
    static constexpr std::uint8_t address = 0x23;

    static bool enabled() { return Fitted::yes; }
};

struct EnabledKeeping : Enabled {
    static constexpr auto whileDisabled = Kvasir::Engine::WhileOff::disconnected;
};

}   // namespace Kvasir::Test::EnabledFixtures
