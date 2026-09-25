#pragma once

#include <cstdint>

namespace Kvasir::USB {

/// What the device and its backend both read out of the application's config struct, with the
/// defaults for what it leaves out. What only one chip knows (an interrupt priority, double
/// buffering) the backend reads itself.
///
///   StartOfFrameCallback(std::uint16_t)   its presence turns the start-of-frame interrupt on
///   BusPower                              mA, 500
///   BusPowered                            true
template<typename ConfigT>
struct ConfigTraits {
    static constexpr bool UseSof = requires { ConfigT::StartOfFrameCallback(std::uint16_t{}); };

    static constexpr auto BusPower = [] {
        if constexpr(requires { ConfigT::BusPower; }) {
            return ConfigT::BusPower;
        } else {
            return 500;
        }
    }();

    static constexpr auto BusPowered = [] {
        if constexpr(requires { ConfigT::BusPowered; }) {
            return ConfigT::BusPowered;
        } else {
            return true;
        }
    }();
};
}   // namespace Kvasir::USB
