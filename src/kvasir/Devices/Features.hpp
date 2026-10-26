#pragma once

// "Empty when off" members for the optional features of a bus, an engine or a driver: no bytes, and the code that
// touches them compiles the same with the feature on and off. Bus-neutral: the device engine (Engine/EngineFeatures.hpp),
// the queued buses (BusTypes.hpp, SPI/QueueCore.hpp) and the chip I2C drivers use them. `Kvasir::Engine::detail` takes
// them in with using-declarations for its own code. Also the one kit that passes a part's Startup members (Claims,
// Provides, the init steps) on to the driver built on it (ForwardClaims, ...).

#include <cstdint>
#include <type_traits>

namespace Kvasir::detail {
/// What a feature that is off leaves of a time point: nothing.
struct NoStamp {};

/// A statistics counter that counts nothing (EngineFeatures::stats off): `++` still
/// compiles, so the engine's code is the same with and without; reading it does not.
struct NoCount {
    constexpr NoCount& operator++() { return *this; }

    constexpr NoCount& operator+=(std::uint32_t) { return *this; }
};

template<bool On, typename T = std::uint32_t>
using StatCount = std::conditional_t<On, T, NoCount>;

/// A table field for a feature that is off (DeviceKnobs, DeviceOps): no bytes, and it takes whatever
/// the initializer gives, so a device's table is written the same with and without. Each
/// field has its own Tag - two empty members of one type may not share an address, and
/// would take a byte each.
template<int Tag>
struct Absent {
    constexpr Absent() = default;

    template<typename U>
    constexpr Absent(U const&) {}   // taking any initializer is the point
};

template<bool On, typename T, int Tag>
using IfFeature = std::conditional_t<On, T, Absent<Tag>>;

/// The Startup members of a part a driver is built on, passed on where that part has them: a driver that derives from
/// `ForwardClaims<Reset>` claims what its reset line claims (a pin), and a Startup that is handed the driver sees it
/// taken. Empty where the part has none. ResetLine (a Device, TouchController), Bridge, PowerRail (its switch) and the
/// USB Device (its backend: every one of them) use these.
template<typename P>
struct ForwardClaims {};

template<typename P>
    requires requires { typename P::Claims; }
struct ForwardClaims<P> {
    using Claims = typename P::Claims;
};

template<typename P>
struct ForwardProvides {};

template<typename P>
    requires requires { typename P::Provides; }
struct ForwardProvides<P> {
    using Provides = typename P::Provides;
};

template<typename P>
struct ForwardPowerClockEnable {};

template<typename P>
    requires requires { P::powerClockEnable; }
struct ForwardPowerClockEnable<P> {
    static constexpr auto powerClockEnable = P::powerClockEnable;
};

template<typename P>
struct ForwardPinConfig {};

template<typename P>
    requires requires { P::initStepPinConfig; }
struct ForwardPinConfig<P> {
    static constexpr auto initStepPinConfig = P::initStepPinConfig;
};

template<typename P>
struct ForwardPeripheryConfig {};

template<typename P>
    requires requires { P::initStepPeripheryConfig; }
struct ForwardPeripheryConfig<P> {
    static constexpr auto initStepPeripheryConfig = P::initStepPeripheryConfig;
};

template<typename P>
struct ForwardInterruptConfig {};

template<typename P>
    requires requires { P::initStepInterruptConfig; }
struct ForwardInterruptConfig<P> {
    static constexpr auto initStepInterruptConfig = P::initStepInterruptConfig;
};
}   // namespace Kvasir::detail
