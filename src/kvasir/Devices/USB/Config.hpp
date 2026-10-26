#pragma once

#include "kvasir/Util/Config.hpp"

#include <cstdint>

namespace Kvasir::USB {

/// What the device and its backend both read out of the application's config struct, with the
/// defaults for what it leaves out. What only one chip knows (an interrupt priority, double
/// buffering) the backend reads itself.
///
///   startOfFrameCallback(std::uint16_t)   its presence turns the start-of-frame interrupt on
///   busPower                              mA, 500
///   busPowered                            true
template<typename ConfigT>
struct ConfigTraits {
    static constexpr bool UseSof = requires { ConfigT::startOfFrameCallback(std::uint16_t{}); };

    static constexpr auto BusPower = KVASIR_CONFIG_OR(ConfigT, busPower, 500);

    static constexpr auto BusPowered = KVASIR_CONFIG_OR(ConfigT, busPowered, true);
};
}   // namespace Kvasir::USB
