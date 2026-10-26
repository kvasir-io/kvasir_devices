#pragma once

#include "Description.hpp"
#include "Engine.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir::Engine {
struct NoGate;

/// A set member built unbound (`Bus(DeferBinding)`), for a DeviceSet to bind with its own device set.
struct DeferBinding {};
}   // namespace Kvasir::Engine

namespace Kvasir::Engine::detail {

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

/// A device set's hooks, known at compile time: the
/// same names as a chip's run-time table (Ops), each a static function over the set's distinct
/// chips. One chip: a direct call of its `static constexpr` entry. Several: an array of the
/// chips' entries indexed by the device's place (EngineState::setIndex_). A hook no chip of the
/// set needs is a constant, and the engine's path behind it folds away.
///
/// The engine takes it where it took the table (`ops`): Engine<Port, Clock> is one copy for
/// the table and one per device set, never one per chip. Every hook is out of line: inlined,
/// each engine call site got the whole switch with every chip's code in it
/// (+17 KB) - out of line it is one switch per hook, each chip's code once.
template<typename Port, typename Clock, SetFlags F, typename... Chips>
struct SetOps {
    static constexpr bool     IsSetOps = true;
    static constexpr SetFlags Flags    = F;

    using State     = EngineState<Port, Clock>;
    using TimePoint = typename Clock::time_point;
    using ReadSlotT = ReadSlotBase<Clock, Port::Features>;
    /// The name the hook list (Hooks.hpp) gives the read slot's type.
    using ReadSlotBaseT = ReadSlotT;

    /// The set's distinct chips (DeviceCores), in order of first appearance: keyed by the chips
    /// and not by the devices, so two Buses with the same chips are one set and one engine
    /// (three buses of KTD2061s).
    using Cores = List<Chips...>;

    static constexpr std::size_t CoreCount = Cores::size;
    static_assert(CoreCount > 0,
                  "a device set with no device");
    static_assert(CoreCount <= 255,
                  "a device set holds up to 255 distinct chips");

    /// The one chip of a single-chip set.
    using FirstCore = typename Nth<Cores>::template type<0>;

    /// What the Bus writes into each device's EngineState::setIndex_.
    template<typename D>
    static constexpr std::uint8_t indexOf
      = static_cast<std::uint8_t>(IndexOf<typename D::Core, Cores>::value);

    // -- the hooks: each the chip's own table entry, called directly -------------------------

/// One chip type: a direct call to its entry (inlined where the compiler likes). Several: a
/// constexpr array of the chips' entries, indexed by the device's place - as compact as the
/// tables were (4 bytes a chip), and measured: a switch over ~50 chips took 9.2 KB more
/// - a compare, branch and call per chip per hook.
#define KVASIR_SET_HOOK(name, Result)                                                           \
    static constexpr auto name##Table                                                           \
      = []<typename... Cs>(List<Cs...>) { return std::array{Cs::EngineOps.name...}; }(Cores{}); \
    template<typename... Args>                                                                  \
    static Result name(State& e, Args... args) {                                                \
        if constexpr(CoreCount == 1) {                                                          \
            return FirstCore::EngineOps.name(e, args...);                                       \
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
            return FirstCore::EngineOps.name(e, args...);                                       \
        } else {                                                                                \
            return name##Table[e.setIndex_](e, args...);                                        \
        }                                                                                       \
    }

    /// The hook that calls back into the engine, with this set's ops (so the engine is never
    /// instantiated for the table too): per chip a thunk.
    template<typename C>
    static void finishInitOf(State&    e,
                             TimePoint now) {
        C::self_(e).finishInit_(SetOps{}, now);
    }

#define KVASIR_SET_THUNK(name, of, Result, ...)                                      \
    static constexpr auto name##Table                                                \
      = []<typename... Cs>(List<Cs...>) { return std::array{&of<Cs>...}; }(Cores{}); \
    template<typename... Args>                                                       \
    static Result name(State& e, Args... args) {                                     \
        if constexpr(CoreCount == 1) {                                               \
            return of<FirstCore>(e, args...);                                        \
        } else {                                                                     \
            return name##Table[e.setIndex_](e, args...);                             \
        }                                                                            \
    }

    /// No chip of the set reads back: never due, and nothing started - a constant the engine's
    /// verify paths fold on (Engine::verifyDue_).
    static constexpr bool VerifyNeeded
      = []<typename... Cs>(List<Cs...>) { return (Cs::NeedsVerify || ...); }(Cores{});
    /// No chip of the set has an Initial: scheduleWrites never applies one.
    static constexpr bool InitialNeeded
      = []<typename... Cs>(List<Cs...>) { return (Cs::NeedsInitial || ...); }(Cores{});

    // -- the constants: data, one entry a chip ---------------------------------------------------

#define KVASIR_SET_CONSTANT(name, Type)                                   \
    static constexpr auto name##Table = []<typename... Cs>(List<Cs...>) { \
        return std::array<Type, CoreCount>{Cs::EngineOps.name...};        \
    }(Cores{});                                                           \
    static Type name(State const& e) {                                    \
        if constexpr(CoreCount == 1) {                                    \
            return FirstCore::EngineOps.name;                             \
        } else {                                                          \
            return name##Table[e.setIndex_];                              \
        }                                                                 \
    }

    // Every hook and constant of the one list (Hooks.hpp). The read-back and bring-up hooks (verifySlot,
    // encodeForVerify, verifyMatches, applyInitial) are called only for a group that verifies or has an Initial,
    // which a chip's WriteGroupInfo says; the null entries of a chip without one are in its writeOpsTable, behind
    // that check (Engine::startVerifyGroup, finishVerify, scheduleWrites).
#define KVASIR_SET_FROM_LIST_HOOK(name, Result, Params) KVASIR_SET_HOOK(name, Result)
#define KVASIR_SET_FROM_LIST_HOOK_IF(name, Result, Params, Needs, ...) \
    KVASIR_SET_HOOK_IF(name, Result, Needs, __VA_ARGS__)
#define KVASIR_SET_FROM_LIST_THUNK(name, Result, Params) KVASIR_SET_THUNK(name, name##Of, Result)
    KVASIR_ENGINE_HOOKS(KVASIR_SET_FROM_LIST_HOOK,
                        KVASIR_SET_FROM_LIST_HOOK_IF,
                        KVASIR_SET_FROM_LIST_THUNK,
                        KVASIR_SET_CONSTANT)
#undef KVASIR_SET_FROM_LIST_HOOK
#undef KVASIR_SET_FROM_LIST_HOOK_IF
#undef KVASIR_SET_FROM_LIST_THUNK
#undef KVASIR_SET_HOOK
#undef KVASIR_SET_HOOK_IF
#undef KVASIR_SET_THUNK
#undef KVASIR_SET_CONSTANT
};

template<typename Port, typename Clock, SetFlags F, typename L>
struct SetOpsOf;

template<typename Port, typename Clock, SetFlags F, typename... Cs>
struct SetOpsOf<Port, Clock, F, List<Cs...>> {
    using type = SetOps<Port, Clock, F, Cs...>;
};

/// The device set of a Bus's devices: SetOps over their distinct chips, with their flags.
template<typename Port, typename Clock, typename... Ds>
using SetOpsFor = typename SetOpsOf<Port,
                                    Clock,
                                    flagsOfAll<Ds...>(),
                                    typename Unique<List<typename Ds::Core...>>::type>::type;

}   // namespace Kvasir::Engine::detail
