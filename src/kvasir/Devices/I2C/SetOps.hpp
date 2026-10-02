#pragma once

#include "Engine.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir::I2C { struct NoGate; }   // namespace Kvasir::I2C

namespace Kvasir::I2C::detail {

/// What some device of a set has, so the engine's path for it is left out of a set where none
/// has it (Engine::AnyGate / AnyReset). Part of the set's type: two Buses with the same chips but
/// one of them behind a switch are two sets.
struct SetFlags {
    bool anyGate  = true;
    bool anyReset = true;

    constexpr bool operator==(SetFlags const&) const = default;
};

/// A device's flags; a DeviceCore alone (no gate type known) says "has both".
template<typename D>
consteval SetFlags flagsOf() {
    if constexpr(requires {
                     typename D::GateT;
                     D::HasResetLine;
                 })
    {
        return {.anyGate = !std::is_same_v<typename D::GateT, NoGate>, .anyReset = D::HasResetLine};
    } else {
        return {};
    }
}

template<typename... Ds>
consteval SetFlags flagsOfAll() {
    return {.anyGate = (flagsOf<Ds>().anyGate || ...), .anyReset = (flagsOf<Ds>().anyReset || ...)};
}

namespace setops {
    template<typename... Ts>
    struct List {};

    template<typename L, typename T>
    struct Append;

    template<typename... Ts, typename T>
    struct Append<List<Ts...>, T> {
        using type
          = std::conditional_t<(std::is_same_v<T, Ts> || ...), List<Ts...>, List<Ts..., T>>;
    };

    template<typename L, typename... Ts>
    struct Unique {
        using type = L;
    };

    template<typename L, typename T, typename... Ts>
    struct Unique<L, T, Ts...> : Unique<typename Append<L, T>::type, Ts...> {};
}   // namespace setops

/// A device set's hooks, known at compile time (kvasir_work plans/engine_minimal/HOOKS.md): the
/// same names as a chip's run-time table (Ops), each a static function over the set's distinct
/// chips. One chip: a direct call of its `static constexpr` entry. Several: an array of the
/// chips' entries indexed by the device's place (EngineState::setIndex_). A hook no chip of the
/// set needs is a constant, and the engine's path behind it folds away.
///
/// The engine takes it where it took the table (`ops`): Engine<Port, Clock> is one copy for
/// the table and one per device set, never one per chip. Every hook is out of line: inlined,
/// each engine call site got the whole switch with every chip's code in it (i2c_testing
/// +17 KB, 2026-09-30) - out of line it is one switch per hook, each chip's code once.
template<typename Port, typename Clock, SetFlags F, typename... Chips>
struct SetOps {
    static constexpr bool     IsSetOps = true;
    static constexpr SetFlags Flags    = F;

    using State     = EngineState<Port, Clock>;
    using TimePoint = typename Clock::time_point;
    using ReadSlotT = ReadSlotBase<Clock, Port::Features>;

    template<typename... Ts>
    using List = setops::List<Ts...>;

    /// The set's distinct chips (DeviceCores), in order of first appearance: keyed by the chips
    /// and not by the devices, so two Buses with the same chips are one set and one engine
    /// (rgb_rotary's three KTD2061 buses).
    using Cores = List<Chips...>;

    template<typename L>
    struct Size;

    template<typename... Ts>
    struct Size<List<Ts...>> : std::integral_constant<std::size_t, sizeof...(Ts)> {};

    static constexpr std::size_t CoreCount = Size<Cores>::value;
    static_assert(CoreCount > 0,
                  "a device set with no device");
    static_assert(CoreCount <= 255,
                  "a device set holds up to 255 distinct chips");

    template<std::size_t I, typename L>
    struct At;

    template<std::size_t I, typename T, typename... Ts>
    struct At<I, List<T, Ts...>> : At<I - 1, List<Ts...>> {};

    template<typename T, typename... Ts>
    struct At<0, List<T, Ts...>> {
        using type = T;
    };

    template<typename C, typename L>
    struct IndexIn;

    template<typename C, typename T, typename... Ts>
    struct IndexIn<C, List<T, Ts...>>
      : std::integral_constant<std::size_t,
                               std::is_same_v<C, T> ? 0 : 1 + IndexIn<C, List<Ts...>>::value> {};

    template<typename C>
    struct IndexIn<C, List<>> : std::integral_constant<std::size_t, 0> {};

    /// What the Bus writes into each device's EngineState::setIndex_.
    template<typename D>
    static constexpr std::uint8_t indexOf
      = static_cast<std::uint8_t>(IndexIn<typename D::Core, Cores>::value);

    // -- the hooks: each the chip's own table entry, called directly -------------------------

/// One chip type: a direct call to its entry (inlined where the compiler likes). Several: a
/// constexpr array of the chips' entries, indexed by the device's place - as compact as the
/// tables were (4 bytes a chip), and measured: a switch over ~50 chips took 9.2 KB more
/// (i2c_testing, 2026-09-30), a compare, branch and call per chip per hook.
#define KVASIR_SET_HOOK(name, Result)                                                           \
    static constexpr auto name##Table                                                           \
      = []<typename... Cs>(List<Cs...>) { return std::array{Cs::EngineOps.name...}; }(Cores{}); \
    template<typename... Args>                                                                  \
    static Result name(State& e, Args... args) {                                                \
        if constexpr(CoreCount == 1) {                                                          \
            return At<0, Cores>::type::EngineOps.name(e, args...);                              \
        } else {                                                                                \
            return name##Table[e.setIndex_](e, args...);                                        \
        }                                                                                       \
    }

/// The same for a hook some chips do not need: a constant when no chip of the set needs it.
#define KVASIR_SET_HOOK_IF(name, Result, Needs, none)                                           \
    static constexpr bool name##Needed                                                          \
      = []<typename... Cs>(List<Cs...>) { return (Cs::Needs || ...); }(Cores{});                \
    static constexpr auto name##Table                                                           \
      = []<typename... Cs>(List<Cs...>) { return std::array{Cs::EngineOps.name...}; }(Cores{}); \
    template<typename... Args>                                                                  \
    static Result name(State& e, Args... args) {                                                \
        if constexpr(!name##Needed) {                                                           \
            static_cast<void>(e);                                                               \
            ((static_cast<void>(args)), ...);                                                   \
            return none;                                                                        \
        } else if constexpr(CoreCount == 1) {                                                   \
            return At<0, Cores>::type::EngineOps.name(e, args...);                              \
        } else {                                                                                \
            return name##Table[e.setIndex_](e, args...);                                        \
        }                                                                                       \
    }

    KVASIR_SET_HOOK_IF(sizeRead,
                       void,
                       NeedsSizedRead,
                       void())
    KVASIR_SET_HOOK_IF(ready,
                       bool,
                       NeedsReady,
                       true)
    KVASIR_SET_HOOK_IF(oracle,
                       bool,
                       NeedsOracle,
                       true)
    KVASIR_SET_HOOK_IF(wakeReset,
                       void,
                       NeedsWake,
                       void())
    KVASIR_SET_HOOK_IF(wakeRetry,
                       bool,
                       NeedsWake,
                       false)
    KVASIR_SET_HOOK_IF(dynamicPeriod,
                       std::chrono::milliseconds,
                       NeedsDynamicPeriod,
                       std::chrono::milliseconds{})
    KVASIR_SET_HOOK_IF(finishVerify,
                       void,
                       NeedsVerify,
                       void())
#undef KVASIR_SET_HOOK_IF

    KVASIR_SET_HOOK(script,
                    std::span<Step const>)
    KVASIR_SET_HOOK(buffer,
                    std::span<std::byte>)
    KVASIR_SET_HOOK(countRead,
                    void)
    KVASIR_SET_HOOK(identify,
                    bool)
    KVASIR_SET_HOOK(clearInit,
                    void)
    KVASIR_SET_HOOK(redirtyItem,
                    void)
    KVASIR_SET_HOOK(unserve,
                    void)
    KVASIR_SET_HOOK(forgetSamples,
                    void)
    KVASIR_SET_HOOK(setupFinal,
                    bool)
    KVASIR_SET_HOOK(startupSchedule,
                    void)
    KVASIR_SET_HOOK(readSlot,
                    ReadSlotT&)
    KVASIR_SET_HOOK(periodNow,
                    std::chrono::milliseconds)
    KVASIR_SET_HOOK(prepareRead,
                    void)
    KVASIR_SET_HOOK(decode,
                    DecodeResult)
    KVASIR_SET_HOOK(writeSlot,
                    WriteSlotBase<Clock>&)
    KVASIR_SET_HOOK(encodeWrite,
                    void)
    KVASIR_SET_HOOK(afterWrite,
                    void)
#undef KVASIR_SET_HOOK

    /// The two hooks that call back into the engine, with this set's ops (so the engine is
    /// never instantiated for the table too), and the read-back deadline: per chip a thunk.
    template<typename C>
    static void finishInitOf(State&    e,
                             TimePoint now) {
        C::self_(e).finishInit_(SetOps{}, now);
    }

    template<typename C>
    static bool startVerifyOf(State&       e,
                              std::uint8_t w,
                              TimePoint    now) {
        if constexpr(C::NeedsVerify) {
            return C::self_(e).startVerify_(SetOps{}, w, now);
        } else {
            static_cast<void>(e);
            static_cast<void>(w);
            static_cast<void>(now);
            return false;
        }
    }

    template<typename C>
    static TimePoint verifyDueOf(State& e) {
        if constexpr(C::EngineOps.verifyDue != nullptr) {
            return C::EngineOps.verifyDue(e);
        } else {
            return TimePoint::max();
        }
    }

#define KVASIR_SET_THUNK(name, of, Result, ...)                                      \
    static constexpr auto name##Table                                                \
      = []<typename... Cs>(List<Cs...>) { return std::array{&of<Cs>...}; }(Cores{}); \
    template<typename... Args>                                                       \
    static Result name(State& e, Args... args) {                                     \
        if constexpr(CoreCount == 1) {                                               \
            return of<typename At<0, Cores>::type>(e, args...);                      \
        } else {                                                                     \
            return name##Table[e.setIndex_](e, args...);                             \
        }                                                                            \
    }

    KVASIR_SET_THUNK(finishInit,
                     finishInitOf,
                     void)
    KVASIR_SET_THUNK(startVerifyGroup,
                     startVerifyOf,
                     bool)
    KVASIR_SET_THUNK(verifyDueAny,
                     verifyDueOf,
                     TimePoint)

    /// No chip of the set reads back: never due, and nothing started - constants the engine's
    /// verify paths fold on.
    static constexpr bool VerifyNeeded
      = []<typename... Cs>(List<Cs...>) { return (Cs::NeedsVerify || ...); }(Cores{});

    static TimePoint verifyDue(State& e) {
        if constexpr(VerifyNeeded) {
            return verifyDueAny(e);
        } else {
            static_cast<void>(e);
            return TimePoint::max();
        }
    }

#undef KVASIR_SET_THUNK

    // -- the constants: data, one entry a chip ---------------------------------------------------

#define KVASIR_SET_CONSTANT(name, Type)                                   \
    static constexpr auto name##Table = []<typename... Cs>(List<Cs...>) { \
        return std::array<Type, CoreCount>{Cs::EngineOps.name...};        \
    }(Cores{});                                                           \
    static Type name(State const& e) {                                    \
        if constexpr(CoreCount == 1) {                                    \
            return At<0, Cores>::type::EngineOps.name;                    \
        } else {                                                          \
            return name##Table[e.setIndex_];                              \
        }                                                                 \
    }

    KVASIR_SET_CONSTANT(reads,
                        std::span<ReadGroupInfo const>)
    KVASIR_SET_CONSTANT(writes,
                        std::span<WriteGroupInfo const>)
    KVASIR_SET_CONSTANT(name,
                        std::string_view)
    KVASIR_SET_CONSTANT(registerBytes,
                        std::uint8_t)
    KVASIR_SET_CONSTANT(initEmpty,
                        bool)
    KVASIR_SET_CONSTANT(startupDelayMs,
                        std::uint32_t)
    KVASIR_SET_CONSTANT(resetLowMs,
                        std::uint32_t)
    KVASIR_SET_CONSTANT(resetSettleMs,
                        std::uint32_t)
#undef KVASIR_SET_CONSTANT
};

template<typename Port, typename Clock, SetFlags F, typename L>
struct SetOpsOf;

template<typename Port, typename Clock, SetFlags F, typename... Cs>
struct SetOpsOf<Port, Clock, F, setops::List<Cs...>> {
    using type = SetOps<Port, Clock, F, Cs...>;
};

/// The device set of a Bus's devices: SetOps over their distinct chips, with their flags.
template<typename Port, typename Clock, typename... Ds>
using SetOpsFor =
  typename SetOpsOf<Port,
                    Clock,
                    flagsOfAll<Ds...>(),
                    typename setops::Unique<setops::List<>, typename Ds::Core...>::type>::type;

}   // namespace Kvasir::I2C::detail
