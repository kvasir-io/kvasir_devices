#pragma once

/// What the device engine does beyond talking to the part, switched per bus (per engine port,
/// Engine.hpp BusPort): a feature that is off costs no RAM in any device's state and no code in
/// the engine.
///
/// A bus says what it wants with a `static constexpr EngineFeatures Features` member; a bus that
/// has none gets the defaults. A driver from a chip package is given one with WithFeatures:
///
///     using I2c1 = Kvasir::I2C::WithFeatures<HW::I2cDriver<...>, {.switchable = true}>;
///
/// Never per device: devices with different features would be two ports, two copies of the
/// engine. A device that needs a feature its bus has switched off does not build.

#include <cstdint>
#include <type_traits>

namespace Kvasir::I2C {

struct EngineFeatures {
    /// Bridges (Bridge.hpp) and Config::enabled(): link() can say `offline`. Off by default -
    /// no firmware used it when the switch came.
    bool switchable = false;
    /// Parking a part that NAKs, probing for it with a backing-off interval, and link()
    /// saying `absent` (Presence.hpp). Off, a NAKing part is retried as any failure is.
    bool presence = true;
    /// The engine's own timeout on a request the bus has not answered (DeviceOps
    /// inFlightTimeoutMs). Off for a bus whose driver has a timeout of its own.
    bool inFlightNet = true;
    /// The bus completion time of every sample, read in the interrupt: stamp<G>() and
    /// takeGaps<G>() for a group that says Timestamped (TouchController reads it).
    bool timestamps = true;
    /// The counters a firmware reports from: samples(), rejected(), writes(), errors(),
    /// turns(), unidentified() - and the numbers in logHealth's line. Nothing in the engine
    /// decides by them.
    bool stats = true;
    /// period<G>(ms): a read group's rate changed at run time. Off, every group runs at its
    /// description's (or its State's) period and the slot keeps no period of its own.
    bool runtimePeriod = true;
    /// After a failed write, read-back or bring-up, wait `FaultBackoff` before trying again.
    /// Off, the next turn tries again at once. (The re-init after `FaultsBeforeReinit` failures
    /// in a row is recovery, not back-off: it stays.)
    bool faultBackoff = true;
    /// Resting turns (Engine::rest): a device with nothing due skips its turn until it is. Off,
    /// every turn runs in full - more work per loop turn, no resting state in RAM.
    bool rest = true;

    constexpr bool operator==(EngineFeatures const&) const = default;
};

/// Every feature on: what a test fake is, so that the tests of each feature run on it.
inline constexpr EngineFeatures AllEngineFeatures{.switchable    = true,
                                                  .presence      = true,
                                                  .inFlightNet   = true,
                                                  .timestamps    = true,
                                                  .stats         = true,
                                                  .runtimePeriod = true,
                                                  .faultBackoff  = true,
                                                  .rest          = true};

/// A bus driver with the engine features @p F: everything of `Bus`, one member more.
template<typename Bus, EngineFeatures F>
struct WithFeatures : Bus {
    static constexpr EngineFeatures Features = F;
};

namespace detail {
    /// What a feature that is off leaves of a time point: nothing.
    struct NoStamp {};

    /// A statistics counter that counts nothing (EngineFeatures::stats off): `++` still
    /// compiles, so the engine's code is the same with and without; reading it does not.
    struct NoCount {
        constexpr NoCount& operator++() { return *this; }
    };

    template<bool On, typename T = std::uint32_t>
    using StatCount = std::conditional_t<On, T, NoCount>;

    /// A table field for a feature that is off (DeviceOps): no bytes, and it takes whatever
    /// the initializer gives, so a device's table is written the same with and without. Each
    /// field has its own Tag - two empty members of one type may not share an address, and
    /// would take a byte each.
    template<int Tag>
    struct Absent {
        constexpr Absent() = default;

        template<typename U>
        constexpr Absent(U const&) {}   // NOLINT: taking any initializer is the point
    };

    template<bool On, typename T, int Tag>
    using IfFeature = std::conditional_t<On, T, Absent<Tag>>;

    template<typename Bus>
    consteval EngineFeatures featuresOf() {
        if constexpr(requires { Bus::Features; }) {
            return Bus::Features;
        } else {
            return EngineFeatures{};
        }
    }
}   // namespace detail

}   // namespace Kvasir::I2C
