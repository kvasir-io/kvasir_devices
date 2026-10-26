#pragma once

#include "Device.hpp"
#include "SetOps.hpp"

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

namespace Kvasir::Engine {

/// What a DeviceSet takes as a member, by shape. A Bus: its chips (`ChipCores`, an Engine::List of DeviceCores), its
/// port, clock and flags, built unbound (`Bus(DeferBinding)`) and bound with the set's SetOps.
template<typename M>
concept BusLike = requires {
    typename M::ChipCores;
    typename M::PortT;
    typename M::ClockT;
    M::Flags;
};

/// A driver that holds its engine device inside (TouchController, Max31865, Ad7124, Max7219): `DeviceT`, `device()`
/// and `turn(busTurn)`, which runs `busTurn(device())` in place of the device's own turn and then does its own part.
template<typename M>
concept WrapsDevice = !BusLike<M> && requires(M& m) {
    typename M::DeviceT::Core;
    { m.device() } -> std::same_as<typename M::DeviceT&>;
    m.turn([](typename M::DeviceT&) {});
};

/// An engine Device itself (its chip's DeviceCore as `Core`).
template<typename M>
concept DeviceLike = !BusLike<M> && !WrapsDevice<M> && requires { typename M::Core; };

/// Every part of one port, driven as ONE device set (SetOps.hpp): one copy of the engine and
/// of each chip's code for all of them, however they are held. A member is
/// - a Bus (its switches, gates and bridges stay its own; built unbound and bound here),
/// - a driver that holds its device inside (TouchController, Max31865, Ad7124, Max7219: it runs
///   its device's turn through `turn(busTurn)`), or
/// - a Device.
///
///     Kvasir::Engine::DeviceSet<SensorBus, Touch> parts{};
///     parts.turn();                     // once a loop turn, instead of each one's turn()
///     parts.get<Touch>().latest();
///
/// Two Buses on two I2C blocks with the same chips, a Bus next to a part on its own: one set,
/// one engine. Parts of another port (SPI next to I2C) need a set of their own.
template<typename... Ms>
class DeviceSet {
    static_assert(
      ((BusLike<Ms> || WrapsDevice<Ms> || DeviceLike<Ms>) && ...),
      "a DeviceSet member is a Bus (BusLike), a driver with a device inside (WrapsDevice: DeviceT, "
      "device(), turn(busTurn)), or a Device (DeviceLike)");

    template<typename M>
    struct Traits;

    template<typename M>
        requires BusLike<M>
    struct Traits<M> {
        using Cores                             = typename M::ChipCores;
        using Port                              = typename M::PortT;
        using Clock                             = typename M::ClockT;
        static constexpr detail::SetFlags Flags = M::Flags;
    };

    template<typename M>
        requires WrapsDevice<M>
    struct Traits<M> {
        using Cores                             = List<typename M::DeviceT::Core>;
        using Port                              = typename M::DeviceT::Core::PortT;
        using Clock                             = typename M::DeviceT::Core::ClockT;
        static constexpr detail::SetFlags Flags = detail::flagsOf<typename M::DeviceT>();
    };

    template<typename M>
        requires DeviceLike<M>
    struct Traits<M> {
        using Cores                             = List<typename M::Core>;
        using Port                              = typename M::Core::PortT;
        using Clock                             = typename M::Core::ClockT;
        static constexpr detail::SetFlags Flags = detail::flagsOf<M>();
    };

    using First = std::tuple_element_t<0, std::tuple<Ms...>>;

public:
    using PortT  = typename Traits<First>::Port;
    using ClockT = typename Traits<First>::Clock;

    static_assert((std::is_same_v<typename Traits<Ms>::Port,
                                  PortT>
                   && ...),
                  "one DeviceSet per port: its parts share the engine, so they share the bus's "
                  "request and result types and the engine features (a second set for the rest)");
    static_assert((std::is_same_v<typename Traits<Ms>::Clock,
                                  ClockT>
                   && ...),
                  "the parts of a DeviceSet share one clock");

    /// The set: the distinct chips of every member.
    static constexpr detail::SetFlags Flags{.anyGate  = (Traits<Ms>::Flags.anyGate || ...),
                                            .anyReset = (Traits<Ms>::Flags.anyReset || ...)};
    using SetOpsT = typename detail::SetOpsOf<PortT,
                                              ClockT,
                                              Flags,
                                              typename detail::Unique<typename detail::Concat<
                                                typename Traits<Ms>::Cores...>::type>::type>::type;

    DeviceSet() {
        std::apply([](auto&... slot) { (bind_(slot.m), ...); }, slots_);
    }

    DeviceSet(DeviceSet const&)            = delete;
    DeviceSet& operator=(DeviceSet const&) = delete;

    /// Once per loop turn: every member's turn, with one clock reading.
    void turn() {
        auto const now = ClockT::now();
        std::apply([&](auto&... slot) { (turn_(slot.m, now), ...); }, slots_);
    }

    template<typename M,
             typename Self>
    [[nodiscard]] constexpr auto& get(this Self&& self) {
        return std::get<Slot<M>>(self.slots_).m;
    }

    /// One Device member's turn alone (a Bus: `bus.turnOf<D, SetOpsT>()`).
    template<typename M>
    void turnOf() {
        turn_(get<M>(), ClockT::now());
    }

    /// A member's turn as a function, for whatever runs it in place of the set: a PowerRail
    /// (`rail.turn(set.get<Bus>(), set.engineStep())` in place of the Bus's turn in turn()).
    [[nodiscard]] static auto engineStep() {
        return [](auto& m) { turn_(m, ClockT::now()); };
    }

    /// Every part starts over from its reset (Device::restart()).
    void restart() {
        std::apply([](auto&... slot) { (restart_(slot.m), ...); }, slots_);
    }

private:
    using Engine = detail::Engine<PortT, ClockT>;

    template<typename M>
    struct Slot {
        M m;

        Slot()
            requires BusLike<M>
          : m{DeferBinding{}} {}

        Slot()
            requires(!BusLike<M>)
          : m{} {}
    };

    template<typename M>
    static void bind_(M& m) {
        if constexpr(BusLike<M>) {
            m.template bindWith<SetOpsT>();
        } else if constexpr(WrapsDevice<M>) {
            m.device().setIndex_ = SetOpsT::template indexOf<typename M::DeviceT>;
        } else {
            m.setIndex_ = SetOpsT::template indexOf<M>;
        }
    }

    template<typename M>
    static void turn_(M&                          m,
                      typename ClockT::time_point now) {
        if constexpr(BusLike<M>) {
            m.template turnWith<SetOpsT>(now);
        } else if constexpr(WrapsDevice<M>) {
            m.turn([&](auto& d) { Engine::turn(d, SetOpsT{}, now); });
        } else {
            Engine::turn(m, SetOpsT{}, now);
        }
    }

    template<typename M>
    static void restart_(M& m) {
        if constexpr(WrapsDevice<M>) {
            m.device().restart();
        } else {
            m.restart();
        }
    }

    std::tuple<Slot<Ms>...> slots_{};
};

}   // namespace Kvasir::Engine
