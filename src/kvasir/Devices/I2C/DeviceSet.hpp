#pragma once

#include "Bus.hpp"
#include "SetOps.hpp"

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace Kvasir::I2C {

/// Every part of one port, driven as ONE device set (SetOps.hpp): one copy of the engine and
/// of each chip's code for all of them, however they are held (kvasir_work
/// plans/engine_minimal/HOOKS.md). A member is
/// - a Bus (its switches, gates and bridges stay its own; built unbound and bound here),
/// - a driver that holds its device inside (TouchController, Max31865, Ad7124, Max7219: it runs
///   its device's turn through `handler(turn)`), or
/// - a Device.
///
///     Kvasir::I2C::DeviceSet<SensorBus, Touch> parts{};
///     parts.handler();                  // once a loop turn, instead of each one's handler()
///     parts.get<Touch>().latest();
///
/// Two Buses on two I2C blocks with the same chips, a Bus next to a part on its own: one set,
/// one engine. Parts of another port (SPI next to I2C) need a set of their own.
template<typename... Ms>
class DeviceSet {
    template<typename M>
    static constexpr bool IsBus = requires {
        typename M::ChipCores;
        typename M::PortT;
    };

    template<typename M>
    static constexpr bool IsWrapper = !IsBus<M> && requires { typename M::DeviceT::Core; };

    template<typename M>
    static constexpr bool IsDevice = !IsBus<M> && !IsWrapper<M> && requires { typename M::Core; };

    static_assert(((IsBus<Ms> || IsWrapper<Ms> || IsDevice<Ms>) && ...),
                  "a DeviceSet member is a Bus, a driver with a device inside (DeviceT, device(), "
                  "handler(turn)), or a Device");

    template<typename M>
    struct Traits;

    template<typename M>
        requires IsBus<M>
    struct Traits<M> {
        using Cores                             = typename M::ChipCores;
        using Port                              = typename M::PortT;
        using Clock                             = typename M::ClockT;
        static constexpr detail::SetFlags Flags = M::Flags;
    };

    template<typename M>
        requires IsWrapper<M>
    struct Traits<M> {
        using Cores                             = detail::setops::List<typename M::DeviceT::Core>;
        using Port                              = typename M::DeviceT::Core::PortT;
        using Clock                             = typename M::DeviceT::Core::ClockT;
        static constexpr detail::SetFlags Flags = detail::flagsOf<typename M::DeviceT>();
    };

    template<typename M>
        requires IsDevice<M>
    struct Traits<M> {
        using Cores                             = detail::setops::List<typename M::Core>;
        using Port                              = typename M::Core::PortT;
        using Clock                             = typename M::Core::ClockT;
        static constexpr detail::SetFlags Flags = detail::flagsOf<M>();
    };

    template<typename... Ls>
    struct Concat {
        using type = detail::setops::List<>;
    };

    template<typename... As, typename... Bs, typename... Ls>
    struct Concat<detail::setops::List<As...>, detail::setops::List<Bs...>, Ls...>
      : Concat<detail::setops::List<As..., Bs...>, Ls...> {};

    template<typename... As>
    struct Concat<detail::setops::List<As...>> {
        using type = detail::setops::List<As...>;
    };

    template<typename L>
    struct UniqueOf;

    template<typename... Cs>
    struct UniqueOf<detail::setops::List<Cs...>>
      : detail::setops::Unique<detail::setops::List<>, Cs...> {};

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
    using SetOpsT = typename detail::SetOpsOf<
      PortT,
      ClockT,
      Flags,
      typename UniqueOf<typename Concat<typename Traits<Ms>::Cores...>::type>::type>::type;

    DeviceSet() {
        std::apply([](auto&... slot) { (bind_(slot.m), ...); }, slots_);
    }

    DeviceSet(DeviceSet const&)            = delete;
    DeviceSet& operator=(DeviceSet const&) = delete;

    /// Once per loop turn: every member's turn, with one clock reading.
    void handler() {
        auto const now = ClockT::now();
        std::apply([&](auto&... slot) { (turn_(slot.m, now), ...); }, slots_);
    }

    template<typename M,
             typename Self>
    [[nodiscard]] constexpr auto& get(this Self&& self) {
        return std::get<Slot<M>>(self.slots_).m;
    }

    /// One Device member's turn alone (a Bus: `bus.handlerOf<D, SetOpsT>()`).
    template<typename M>
    void handlerOf() {
        turn_(get<M>(), ClockT::now());
    }

    /// A member's turn as a function, for whatever runs it in place of the set: a PowerRail
    /// (`rail.handler(set.get<Bus>(), set.turn())` in place of the Bus's turn in handler()).
    [[nodiscard]] static auto turn() {
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
            requires IsBus<M>
          : m{DeferBinding{}} {}

        Slot()
            requires(!IsBus<M>)
          : m{} {}
    };

    template<typename M>
    static void bind_(M& m) {
        if constexpr(IsBus<M>) {
            m.template bindWith<SetOpsT>();
        } else if constexpr(IsWrapper<M>) {
            m.device().setIndex_ = SetOpsT::template indexOf<typename M::DeviceT>;
        } else {
            m.setIndex_ = SetOpsT::template indexOf<M>;
        }
    }

    template<typename M>
    static void turn_(M&                          m,
                      typename ClockT::time_point now) {
        if constexpr(IsBus<M>) {
            m.template handlerWith<SetOpsT>(now);
        } else if constexpr(IsWrapper<M>) {
            m.handler([&](auto& d) { Engine::handler(d, SetOpsT{}, now); });
        } else {
            Engine::handler(m, SetOpsT{}, now);
        }
    }

    template<typename M>
    static void restart_(M& m) {
        if constexpr(IsWrapper<M>) {
            m.device().restart();
        } else {
            m.restart();
        }
    }

    std::tuple<Slot<Ms>...> slots_{};
};

}   // namespace Kvasir::I2C
