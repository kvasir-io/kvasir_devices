#pragma once

#include "../Bytes.hpp"
#include "../Features.hpp"
#include "../Log.hpp"
#include "../Quantities.hpp"
#include "../ResetLine.hpp"
#include "Description.hpp"
#include "DeviceConfig.hpp"
#include "Engine.hpp"
#include "EngineLog.hpp"
#include "Pending.hpp"
#include "Presence.hpp"
#include "RegisterCheck.hpp"
#include "SetOps.hpp"
#include "Step.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Link.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

/// The part of an engine device that depends on the chip alone (DeviceCore) and the detail it builds on; the
/// driver itself, what only one device has, is Device.hpp.
namespace Kvasir::Engine {

using detail::Answer;

namespace detail {
    /// Per read group, only when it keeps history: what the last frames amounted to and when
    /// the last one landed. An empty member otherwise, so a group that keeps no history costs
    /// nothing -- the same trick VerifySlot uses.
    /// `Timed`: the port keeps sample timestamps (EngineFeatures::timestamps); without them
    /// only the unchanged count is kept.
    template<bool Enable, bool Timed, typename TP>
    struct HistorySlot {};

    template<typename TP>
    struct HistorySlot<true, false, TP> {
        /// Frames that were well formed and said nothing new (Outcome::unchanged).
        std::uint32_t unchanged{};
    };

    template<typename TP>
    struct HistorySlot<true, true, TP> {
        using Duration = typename TP::duration;
        /// Frames that were well formed and said nothing new (Outcome::unchanged).
        std::uint32_t unchanged{};
        /// The bus completion of the last sample; TP{} until there was one.
        TP at{};
        /// Since the last takeGaps(): the longest gap between two samples, and the ones of
        /// two periods or more.
        Duration      longestGap{};
        std::uint32_t lateGaps{};
    };

    /// Per read group that confirms changes: the bytes last taken and the ones awaiting a match.
    template<std::size_t Bytes, bool Enable>
    struct ConfirmSlot {};

    template<std::size_t Bytes>
    struct ConfirmSlot<Bytes, true> {
        std::array<std::byte, Bytes> taken{};
        std::array<std::byte, Bytes> candidate{};
        std::uint8_t                 takenBytes{};
        std::uint8_t                 candidateBytes{};
        bool                         hasCandidate{};
        std::uint8_t                 unconfirmed{};
        std::uint32_t                disagreements{};
    };

    /// On the Device, only when some write group verifies: one scratch pair, because a
    /// verify is never on the wire at the same time as anything else.
    template<bool Enable, std::size_t N>
    struct VerifyBufs {};

    template<std::size_t N>
    struct VerifyBufs<true, N> {
        std::array<std::byte, N> got{};     ///< what the chip returned
        std::array<std::byte, N> want{};    ///< what was written
        Step                     wrote{};   ///< the write step, to recover register and count
        std::uint8_t             len{};
    };

    /// On the Device, only when some write group's encode() returns several Steps: the
    /// transactions the item being written is, in order. A chip whose writes are one
    /// transaction each keeps its single step in `current_` and this is empty -- the same
    /// trick as VerifyBufs above, and the reason a multi-step write costs such a chip
    /// nothing.
    template<bool Enable, std::size_t N>
    struct WriteScript {};

    template<std::size_t N>
    struct WriteScript<true, N> {
        std::array<Step, N> steps{};
        std::uint8_t        count{};
    };

    /// A part that NAKs while it wakes (Chip::WakeRetries): the retries of the transaction on
    /// the wire, and all of them. An empty member for every other chip.
    template<bool Enable>
    struct WakeSlot {};

    template<>
    struct WakeSlot<true> {
        std::uint8_t  tries{};     ///< of the transaction on the wire, so far
        std::uint32_t retries{};   ///< over the device's life
    };

    template<template<typename> class Slot, typename L>
    struct Slots;

    template<template<typename> class Slot, typename... Gs>
    struct Slots<Slot, List<Gs...>> {
        using type = std::tuple<Slot<Gs>...>;
    };

    using Kvasir::detail::ForwardClaims;
}   // namespace detail

/// Everything of a Device that depends on the chip alone: slots, buffers, the chip's State, the
/// application's API and the engine's Ops. Devices of one chip on one port type (PortOf) with
/// equal `Confirms` (detail::confirmsOf, it sizes the slots) share it; the rest is DeviceOps.

template<typename Port, typename Clock, typename ChipT, auto Confirms>
struct DeviceCore : detail::EngineState<Port, Clock> {
    /// A Bus's device set calls this chip's hooks directly (SetOps.hpp).
    template<typename, typename, detail::SetFlags, typename...>
    friend struct detail::SetOps;

    using Core       = DeviceCore;   ///< for a device set (SetOps): the chip's shared part
    using Chip       = ChipT;
    using PortT      = Port;
    using ClockT     = Clock;
    using TimePoint  = typename Clock::time_point;
    using Duration   = typename Clock::duration;
    using PendingT   = Pending<Port, Clock>;
    using State      = typename detail::StateOf<Chip>::type;
    using DeviceOpsT = detail::DeviceOps<Port, Clock>;

    /// Read group G keeps the bus completion time of its samples: it keeps history
    /// (detail::KeepsHistory) and the port keeps timestamps (EngineFeatures::timestamps).
    template<typename G>
    static constexpr bool Timed = detail::KeepsHistory<G> && Port::Features.timestamps;

    /// A read group's slot as the engine walks it (the statistics are in it when the port has them).
    using ReadSlotBaseT = detail::ReadSlotBase<Clock, Port::Features>;

    /// The state machine's own fields and its two enums (Engine.hpp). A base that depends on a
    /// template parameter is not searched by unqualified lookup, so every name it brings is
    /// named here once instead of `this->` at some hundreds of use sites.
    using EngineStateT = detail::EngineState<Port, Clock>;
    using Phase        = detail::Phase;
    using Running      = detail::Running;

    using EngineStateT::acked_;
    using EngineStateT::afterDelay_;
    using EngineStateT::bringUps_;
    using EngineStateT::consecutiveFailures_;
    using EngineStateT::current_;
    using EngineStateT::errors_;
    using EngineStateT::group_;
    using EngineStateT::holdUntil_;
    using EngineStateT::identified_;
    using EngineStateT::inFlight_;
    using EngineStateT::initFailed_;
    using EngineStateT::item_;
    using EngineStateT::offline_;
    using EngineStateT::oracleFailed_;
    using EngineStateT::pending_;
    using EngineStateT::phase_;
    using EngineStateT::presence_;
    using EngineStateT::running_;
    using EngineStateT::step_;
    using EngineStateT::submittedAt_;
    using EngineStateT::unidentified_;
    using EngineStateT::unidentifiedLogged_;
    using EngineStateT::up_;
    using EngineStateT::waiting_;
    using EngineStateT::waitUntil_;
    using Reads  = typename detail::ReadsOf<Chip>::type;
    using Writes = typename detail::WritesOf<Chip>::type;

    /// What latest(), fresh(), seq() mean without a group name: `Chip::Primary` when the
    /// description names one, else its only read group. Naming a Primary is what lets a
    /// description grow a second read group without breaking every caller of latest().
    using PrimaryRead                    = typename detail::PrimaryOf<Chip, Reads>::type;
    static constexpr bool HasPrimaryRead = !std::is_void_v<PrimaryRead>;

    static constexpr std::size_t RegisterBytes = Chip::RegisterBytes;

    /// The description's Init, behind the identity reads and their comparison when the
    /// description names its oracle (detail::initWithIdentity).
    static constexpr bool HasIdentity = detail::HasIdentity<Chip>;

    static constexpr std::span<Step const> InitSteps = [] {
        if constexpr(HasIdentity) {
            return std::span<Step const>{detail::EffectiveInit<Chip>::Steps};
        } else {
            return detail::writtenInit<Chip>();
        }
    }();

    /// Where the identity registers are read to: behind what the description's own Init reads.
    static constexpr std::size_t IdentityOffset = bufferBytes(detail::writtenInit<Chip>());

    static_assert(!HasIdentity || Chip::RegisterBytes != 0,
                  "an Identity is registers, and this chip has none");

    static constexpr std::chrono::milliseconds StartupDelay
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Chip, StartupDelay, std::chrono::milliseconds{0}));

    static constexpr std::chrono::milliseconds ResetLow
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Chip, ResetLow, std::chrono::milliseconds{1}));

    static constexpr std::chrono::milliseconds ResetSettle
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Chip, ResetSettle, std::chrono::milliseconds{1}));

    /// Each read group's ConfirmChange (from `Confirms`).
    template<typename G>
    static constexpr std::uint8_t ConfirmChange = Confirms[detail::IndexOf<G, Reads>::value];

    /// A NAK is the part still waking: resubmit it this often, this long apart, before it counts.
    static constexpr std::uint8_t WakeRetries
      = static_cast<std::uint8_t>(KVASIR_CONFIG_OR(Chip, WakeRetries, std::uint8_t{0}));

    static constexpr std::chrono::milliseconds WakeRetryDelay
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Chip, WakeRetryDelay, std::chrono::milliseconds{1}));

    /// What the bus's CallbackSize has to hold: the completion lambda the engine hands it.
    /// Checked against the bus when it says what it holds.
    static constexpr std::size_t CallbackBytes
      = sizeof(decltype(std::declval<PendingT&>().callback()));

    // -- compile-time checks of the description --------------------------------------------

    static_assert(RegisterBytes <= 2,
                  "a register address is zero, one or two bytes");
    static_assert(
      wellFormed(InitSteps,
                 RegisterBytes,
                 false),
      "the chip's Init script: an inline payload over eight bytes, a register past 0xFF on a "
      "one-byte chip, a register on a chip without registers, a read of nothing, a check "
      "step (only read groups have ready()), a counted read whose count is not one or two "
      "bytes inside an earlier read, or a first transaction that may NAK (it is the probe)");

    template<typename G>
    static constexpr bool groupOk() {
        if constexpr(ReadGroup<G>) {
            return wellFormed(std::span<Step const>{G::Steps}, RegisterBytes)
                && bufferBytes(std::span<Step const>{G::Steps}) <= 255
                && (!detail::hasCheck(std::span<Step const>{G::Steps}) || detail::HasReady<G>)
                // a prepare() that returns a count sizes THE read of the group, so there
                // has to be exactly one for the count to be unambiguous
                &&(!detail::HasSizedPrepare<G>
                   || detail::readSteps(std::span<Step const>{G::Steps}) == 1);
        } else {
            // Transient says "not a state, do not replay"; Initial says "a state, restore it
            // after every bring-up". A group cannot mean both.
            return detail::itemsOf<G>() >= 1 && detail::itemsOf<G>() <= 32
                && !(detail::HasTransient<G> && detail::HasInitial<G>)
                     // A Step's offset and count are one byte, so no step can address a group
                     // buffer past 255: commandBuffer(0, Bytes) would truncate in silence.
                     &&static_cast<std::size_t>(G::Bytes)
                     <= 255
                // encode() gives back one Step or a std::array<Step, N>, N >= 1: an empty
                // script would leave the item owed for ever.
                && detail::writeStepsOf<G>() >= 1
                // A read-back compares one register against one encoded payload; a
                // multi-transaction item has no single register to read back from.
                && (detail::writeStepsOf<G>() == 1 || !detail::Verifies<G>);
        }
    }

    static_assert(
      [] {
          bool ok = true;
          detail::forEach<Reads>([&](auto i) {
              ok = ok && groupOk<typename detail::Nth<Reads>::template type<decltype(i)::value>>();
          });
          return ok;
      }(),
      "a read group's Steps script is not well formed (see Step.hpp scriptFault: a counted read's "
      "count must be one or two bytes inside an earlier read step), or its "
      "prepare() returns a count while the script has other than exactly one read step, or it "
      "has a check step and no ready(Bytes)");
    static_assert(
      [] {
          bool ok = true;
          detail::forEach<Writes>([&](auto i) {
              ok = ok && groupOk<typename detail::Nth<Writes>::template type<decltype(i)::value>>();
          });
          return ok;
      }(),
      "a write group has Items outside 1..32, or declares both Transient (a one-shot command, not "
      "replayed after a reset) and Initial (a state, restored after every bring-up), which "
      "contradict each other, or its Bytes is over 255 (a step's offset and count are one byte), "
      "or its encode() returns an empty std::array<Step, 0>, or it asks to be verified while "
      "encoding to several Steps (there is no one register to read back)");

    // -- the slots -------------------------------------------------------------------------

    template<typename G>
    struct ReadSlot : ReadSlotBaseT {
        static constexpr std::size_t Bytes = [] {
            auto const n = bufferBytes(std::span<Step const>{G::Steps});
            return n == 0 ? std::size_t{1} : n;
        }();
        using Sample  = typename detail::SampleOf<G>::type;
        using Request = typename detail::RequestOf<G>::type;

        /// The three fields of the base whose starting value only this group knows.
        constexpr ReadSlot() {
            this->len        = static_cast<std::uint8_t>(Bytes);
            this->validBytes = static_cast<std::uint8_t>(Bytes);
            this->period     = periodOf<G>();
        }

        std::array<std::byte, Bytes> buf{};
        Sample                       sample{};
        Request                      request{};
        [[no_unique_address]] detail::HistorySlot<detail::KeepsHistory<G>, Timed<G>, TimePoint>
                                                                                history{};
        [[no_unique_address]] detail::ConfirmSlot<Bytes, ConfirmChange<G> != 0> confirm{};
    };

    template<typename G>
    struct WriteSlot
      : std::conditional_t<detail::Verifies<G>,
                           detail::VerifiedWriteSlotBase<Clock>,
                           detail::WriteSlotBase<Clock>> {
        static constexpr std::size_t Items = detail::itemsOf<G>();
        using Value                        = typename G::Value;

        /// Every item's dirty bit. Computed in 64 bits: Items may be 32.
        static constexpr std::uint32_t AllItems
          = static_cast<std::uint32_t>((1ULL << Items) - 1ULL);

        std::array<std::byte, static_cast<std::size_t>(G::Bytes)> buf{};
        /// Zero-filled: a Value is read only after its first set<G>() or Initial. Without the `{}`
        /// the owners' constructors spell the same zeros out differently (product images move).
        // NOLINTNEXTLINE(bugprone-invalid-enum-default-initialization) // never read before set<G>()
        std::array<Value, Items> values{};
    };

    /// One entry per cyclic read group, so the engine can walk them without their types.
    static constexpr auto ReadInfos = [] {
        std::array<detail::ReadGroupInfo, Reads::size == 0 ? 1 : Reads::size> infos{};
        detail::forEach<Reads>([&](auto i) {
            using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
            infos[decltype(i)::value]
              = detail::ReadGroupInfo{.periodMs = static_cast<std::uint32_t>(periodOf<G>().count()),
                                      .dynamicPeriod = detail::HasDynamicPeriod<G>};
        });
        return infos;
    }();

    /// One entry per write group, so the engine can walk them without their types.
    static constexpr auto WriteInfos = [] {
        std::array<detail::WriteGroupInfo, Writes::size == 0 ? 1 : Writes::size> infos{};
        detail::forEach<Writes>([&](auto i) {
            using W = typename detail::Nth<Writes>::template type<decltype(i)::value>;
            infos[decltype(i)::value] = detail::WriteGroupInfo{
              .allItems         = WriteSlot<W>::AllItems,
              .periodMs         = static_cast<std::uint32_t>(periodOf<W>().count()),
              .verifyDelayMs    = static_cast<std::uint32_t>(detail::verifyDelayOf<W>().count()),
              .verifyIntervalMs = static_cast<std::uint32_t>(detail::verifyIntervalOf<W>().count()),
              .verifies         = detail::Verifies<W>,
              .initial          = detail::HasInitial<W>,
              .transient        = detail::HasTransient<W>};
        });
        return infos;
    }();

    static constexpr std::size_t writeTableSize = Writes::size == 0 ? 1 : Writes::size;

    /// What the engine's write paths need of each write group that depends on its type: a
    /// constexpr table of small functions, one entry per group, reached through EngineOps; the
    /// paths themselves are the engine's, one copy for every chip. A null entry is "the group
    /// has none".
    struct WriteOps {
        /// Every item not in `known` to Initial (HasInitial).
        void (*applyInitial)(DeviceCore&,
                             std::uint32_t known){};
        /// What the chip State makes of `item` now that the part holds it (W::applied).
        void (*applied)(DeviceCore&,
                        std::uint8_t item){};
        /// `item` re-encoded into verify_ (want, wrote, len): what to read back (count 0 for
        /// nothing, a bare pointer set).
        detail::VerifyRead (*encodeForVerify)(DeviceCore&,
                                              std::uint32_t item){};
        /// The read-back says what was written (W::verify, else the bytes are equal).
        bool (*matches)(std::span<std::byte const> want,
                        std::span<std::byte const> got){};
    };

    static constexpr std::array<WriteOps, writeTableSize> writeOpsTable = [] {
        std::array<WriteOps, writeTableSize> ops{};
        detail::forEach<Writes>([&](auto i) {
            static constexpr std::size_t I = decltype(i)::value;
            using W                        = typename detail::Nth<Writes>::template type<I>;
            if constexpr(detail::HasInitial<W>) {
                ops[I].applyInitial = [](DeviceCore& d, std::uint32_t known) {
                    // Per item: an application that set item 3 of a group of 16 still owes the
                    // other 15 their Initial, not a default-constructed Value.
                    auto& s = std::get<I>(d.writes_);
                    for(std::size_t item = 0; item < WriteSlot<W>::Items; ++item) {
                        if(((known >> item) & 1U) == 0) { s.values[item] = W::Initial; }
                    }
                };
            }
            if constexpr(requires(typename W::Value const& v, State& st) { W::applied(v, st); }) {
                ops[I].applied = [](DeviceCore& d, std::uint8_t item) {
                    W::applied(std::get<I>(d.writes_).values[item], d.state_);
                };
            }
            if constexpr(detail::Verifies<W>) {
                // A read-back compares one register against one encoded payload; a
                // multi-transaction item has no single register to read back from. groupOk()
                // rejects the combination -- this says why here, at the place that relies on it.
                static_assert(
                  detail::writeStepsOf<W>() == 1,
                  "a write group that asks to be verified must encode to a single Step");
                // `auto&`: d.verify_ is checked only where this branch is taken -- a chip that
                // verifies nothing has a VerifyBufs without members.
                ops[I].encodeForVerify
                  = [](auto& d, [[maybe_unused]] std::uint32_t item) -> detail::VerifyRead {
                    auto& s = std::get<I>(d.writes_);
                    Step  wrote{};
                    if constexpr(detail::HasItems<W>) {
                        wrote = W::encode(s.values[item],
                                          static_cast<std::size_t>(item),
                                          std::span<std::byte>{s.buf});
                    } else {
                        wrote = W::encode(s.values[0], std::span<std::byte>{s.buf});
                    }
                    if(wrote.count == 0) { return {}; }
                    constexpr std::size_t MaxBytes = detail::maxVerifyBytesOf<Writes>();
                    auto const            n = static_cast<std::size_t>(wrote.count) > MaxBytes
                                              ? MaxBytes
                                              : static_cast<std::size_t>(wrote.count);
                    for(std::size_t j = 0; j < n; ++j) {
                        d.verify_.want[j] = wrote.fromBuffer
                                            ? s.buf[wrote.offset + j]
                                            : static_cast<std::byte>(wrote.bytes[j]);
                    }
                    d.verify_.wrote = wrote;
                    d.verify_.len   = static_cast<std::uint8_t>(n);
                    d.verify_.got   = {};
                    return {.reg         = wrote.reg,
                            .count       = static_cast<std::uint8_t>(n),
                            .hasRegister = wrote.hasRegister};
                };
                ops[I].matches
                  = [](std::span<std::byte const> want, std::span<std::byte const> got) {
                        if constexpr(detail::HasVerifyFn<W>) {
                            return static_cast<bool>(W::verify(Bytes{want}, Bytes{got}));
                        } else {
                            return std::ranges::equal(want, got);
                        }
                    };
            }
        });
        return ops;
    }();

    using ReadSlots  = typename detail::Slots<ReadSlot, Reads>::type;
    using WriteSlots = typename detail::Slots<WriteSlot, Writes>::type;

    /// One item of a write group may be several transactions (a window command and the
    /// display RAM behind it). One when every group's encode() returns a single Step, and
    /// then the script store below is empty.
    static constexpr std::size_t MaxWriteSteps   = detail::maxWriteStepsOf<Writes>();
    static constexpr bool        MultiStepWrites = MaxWriteSteps > 1;

    static constexpr std::size_t InitBytes = [] {
        auto const n = bufferBytes(InitSteps);
        return n == 0 ? std::size_t{1} : n;
    }();

    static constexpr bool AnyVerify = detail::anyVerifies<Writes>();

    /// Whether this chip needs a hook at all (SetOps): a hook no chip of a Bus's device set
    /// needs is a constant there, and the engine's path behind it folds away at compile time.
    template<typename P>
    static consteval bool anyRead_(P p) {
        bool any = false;
        detail::forEach<Reads>([&](auto i) {
            using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
            any     = any || p.template operator()<G>();
        });
        return any;
    }

    static constexpr bool NeedsWake = WakeRetries > 0;
    static constexpr bool NeedsSizedRead
      = anyRead_([]<typename G>() { return detail::HasSizedPrepare<G>; });
    static constexpr bool NeedsReady = anyRead_([]<typename G>() { return detail::HasReady<G>; });
    static constexpr bool NeedsDynamicPeriod
      = anyRead_([]<typename G>() { return detail::HasDynamicPeriod<G>; });
    static constexpr bool        NeedsOracle    = HasIdentity;
    static constexpr bool        NeedsVerify    = AnyVerify;
    static constexpr bool        NeedsInitial   = detail::anyInitial<Writes>();
    static constexpr std::size_t MaxVerifyBytes = detail::maxVerifyBytesOf<Writes>();

    // A read-back is addressed by the register the write went to; on a chip without registers
    // (an I2C switch, a DAC with one word) it is a bare read of as many bytes as were written,
    // which is what such parts return.

    /// `dev` is the Device's own table (Device::Hooks).
    constexpr explicit DeviceCore(DeviceOpsT const* dev) { this->dev_ = dev; }

private:
    /// The state the engine works on is this object: every trampoline below casts back.
    [[nodiscard]] static DeviceCore& self_(EngineStateT& e) { return static_cast<DeviceCore&>(e); }

    /// What the engine (Engine.hpp) needs of this chip: one static function per hook and one constant per value of
    /// the hook list (Hooks.hpp), each of exactly the type of its `detail::Ops` field. This is where the chip's
    /// groups are switched over, once, instead of at every call site.
    struct ChipHooks {
        static std::span<Step const> script(EngineStateT& e) {
            auto& d = self_(e);
            switch(d.running_) {
            case Running::init: return InitSteps;
            case Running::read: return d.readSteps_(d.group_);
            case Running::write:
                // One item may be several transactions: the window command, then the display
                // RAM behind it. On a chip whose writes are one transaction each the script is
                // `current_` itself and there is no store to walk.
                if constexpr(MultiStepWrites) {
                    return std::span<Step const>{d.writeScript_.steps.data(), d.writeScript_.count};
                } else {
                    return std::span<Step const>{&d.current_, 1};
                }
            case Running::verify: return std::span<Step const>{&d.current_, 1};
            case Running::none:   break;
            }
            return {};
        }

        static std::span<std::byte> buffer(EngineStateT& e) {
            auto& d = self_(e);
            switch(d.running_) {
            case Running::init:  return std::span<std::byte>{d.init_};
            case Running::read:  return d.readBuffer_(d.group_);
            case Running::write: return d.writeBuffer_(d.group_);
            case Running::verify:
                if constexpr(AnyVerify) {
                    return std::span<std::byte>{d.verify_.got};
                } else {
                    break;
                }
            case Running::none: break;
            }
            return {};
        }

        /// A group whose prepare() returned a count reads that many bytes instead of the
        /// script's. Such a group has exactly one read step (static_asserted), so the count is
        /// simply what was asked for.
        static void sizeRead(EngineStateT& e) {
            auto& d = self_(e);
            detail::withIndex(
              d.group_,
              [&](auto i) {
                  using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                  if constexpr(detail::HasSizedPrepare<G>) {
                      d.current_.count = std::get<decltype(i)::value>(d.reads_).len;
                  }
              },
              std::make_index_sequence<Reads::size>{});
        }

        /// A counted read of the running group reads `n` bytes: decode sees up to there.
        static void countRead(EngineStateT& e,
                              std::uint8_t  n) {
            auto& d = self_(e);
            detail::withIndex(
              d.group_,
              [&](auto i) {
                  auto& s      = std::get<decltype(i)::value>(d.reads_);
                  s.len        = n;
                  s.validBytes = static_cast<std::uint8_t>(d.current_.offset + n);
              },
              std::make_index_sequence<Reads::size>{});
        }

        static bool ready(EngineStateT& e) { return self_(e).ready_(); }

        static bool oracle(EngineStateT& e) {
            auto& d            = self_(e);
            d.identityMatched_ = d.identityMatches_();
            if(!d.identityMatched_) { d.oracleFailed_ = true; }
            return d.identityMatched_;
        }

        static bool identify(EngineStateT& e) {
            auto& d = self_(e);
            if constexpr(requires(State& s) {
                             { Chip::setup(Bytes{}, s) } -> std::convertible_to<bool>;
                         })
            {
                // A copy, so that a verdict here changes nothing -- but of the State as it is,
                // which carries what identified() made of the Identity.
                State probe = d.state_;
                return Chip::setup(Bytes{std::span<std::byte const>{d.init_}}, probe);
            } else {
                static_cast<void>(d);
                return true;
            }
        }

        static void clearInit(EngineStateT& e) { self_(e).init_.fill(std::byte{0}); }

        static void finishInit(EngineStateT& e,
                               TimePoint     now) {
            self_(e).finishInit_(EngineOps, now);
        }

        static void wakeReset(EngineStateT& e) {
            if constexpr(WakeRetries > 0) {
                self_(e).wake_.tries = 0;
            } else {
                static_cast<void>(e);
            }
        }

        /// The write item on the wire is still owed: a fault says nothing about whether the
        /// bytes landed, so it is written again.
        static void redirtyItem(EngineStateT& e) {
            auto& d = self_(e);
            if(d.withdrawnOnWire_) {
                d.withdrawnOnWire_ = false;   // withdraw<W>() said it is no longer wanted
                return;
            }
            d.writeSlotBase_(d.group_).dirty.fetch_or(1U << d.item_, std::memory_order_relaxed);
        }

        /// The read on the wire is abandoned, not failed: the requests it took on are owed again
        /// and run after the bring-up, their tickets still pending.
        static void unserve(EngineStateT& e) {
            auto& d = self_(e);
            detail::withIndex(
              d.group_,
              [&](auto i) {
                  auto& s = std::get<decltype(i)::value>(d.reads_);
                  if(s.serving) {
                      s.serving = false;
                      s.started = s.served;
                  }
              },
              std::make_index_sequence<Reads::size>{});
        }

        /// A bring-up starts over: what the part said before it is not a reading of the part as it
        /// is now, so valid() waits for a new sample of every group. The lifetime counts stay.
        static void forgetSamples(EngineStateT& e) {
            auto& d = self_(e);
            detail::forEach<Reads>(
              [&](auto i) { std::get<decltype(i)::value>(d.reads_).current = false; });
        }

        static bool setupFinal(EngineStateT& e) {
            auto& d = self_(e);
            if constexpr(requires(State& s) {
                             { Chip::setup(Bytes{}, s) } -> std::convertible_to<bool>;
                         })
            {
                return Chip::setup(Bytes{std::span<std::byte const>{d.init_}}, d.state_);
            } else {
                static_cast<void>(d);
                return true;
            }
        }

        /// Every read group's first deadline after a bring-up.
        static void startupSchedule(EngineStateT& e,
                                    TimePoint     now) {
            auto& d = self_(e);
            detail::forEach<Reads>([&](auto i) {
                using G   = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                auto& s   = std::get<decltype(i)::value>(d.reads_);
                s.retries = 0;
                if constexpr(periodOf<G>() > std::chrono::milliseconds::zero()) {
                    bool parked = false;
                    if constexpr(Port::Features.runtimePeriod) {
                        parked = s.periodSet && s.period == std::chrono::milliseconds::zero();
                    }
                    s.due = parked ? TimePoint::max() : now;
                } else {
                    s.due = now;
                }
            });
            // The write groups are the engine's (Engine::scheduleWrites), after this.
        }

        /// A NAK the part may give while it wakes: true when it is put on the wire again after
        /// WakeRetryDelay, the gate let go meanwhile, rather than counted.
        static bool wakeRetry(EngineStateT& e,
                              TimePoint     now) {
            if constexpr(WakeRetries > 0) {
                auto& d = self_(e);
                if(d.wake_.tries >= WakeRetries) {
                    d.wake_.tries = 0;
                    return false;
                }
                ++d.wake_.tries;
                ++d.wake_.retries;
                d.waiting_    = true;
                d.afterDelay_ = false;   // the same step again, not the next one
                d.waitUntil_  = now + WakeRetryDelay;
                d.letGateGo_();
                return true;
            } else {
                static_cast<void>(e);
                static_cast<void>(now);
                return false;
            }
        }

        static constexpr std::uint32_t startupDelayMs
          = static_cast<std::uint32_t>(StartupDelay.count());
        static constexpr std::uint32_t resetLowMs = static_cast<std::uint32_t>(ResetLow.count());
        static constexpr std::uint32_t resetSettleMs
          = static_cast<std::uint32_t>(ResetSettle.count());
        static constexpr std::span<detail::ReadGroupInfo const> reads
          = std::span<detail::ReadGroupInfo const>{ReadInfos}.first(Reads::size);

        static ReadSlotBaseT& readSlot(EngineStateT& e,
                                       std::uint8_t  g) {
            ReadSlotBaseT* r = nullptr;
            detail::withIndex(
              g,
              [&](auto i) { r = &std::get<decltype(i)::value>(self_(e).reads_); },
              std::make_index_sequence<Reads::size>{});
            // g is always one of the groups. Said so, rather than left to `*r`: a sanitized build
            // keeps the null path that `*r` alone would make dead, and its type-mismatch handler
            // returns, so gcc's LTO sees an atomic load from address 0 there (-Wstringop-overflow).
            // Unreachable is a panic with a report under UBSan and nothing otherwise.
            if(r == nullptr) { std::unreachable(); }
            return *r;
        }

        static std::chrono::milliseconds periodNow(EngineStateT& e,
                                                   std::uint8_t  g) {
            std::chrono::milliseconds p{};
            detail::withIndex(
              g,
              [&](auto i) {
                  using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                  p       = self_(e).template periodNow_<G>();
              },
              std::make_index_sequence<Reads::size>{});
            return p;
        }

        static std::chrono::milliseconds dynamicPeriod(EngineStateT& e,
                                                       std::uint8_t  g) {
            std::chrono::milliseconds p{};
            detail::withIndex(
              g,
              [&](auto i) {
                  using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                  if constexpr(detail::HasDynamicPeriod<G>) {
                      p = G::periodNow(std::get<decltype(i)::value>(self_(e).reads_).sample);
                  }
              },
              std::make_index_sequence<Reads::size>{});
            return p;
        }

        /// The group's prepare() and the lengths this run works with; the engine picks the group
        /// and does the rest (Engine::startRead).
        static void prepareRead(EngineStateT& e,
                                std::uint8_t  g) {
            auto& d = self_(e);
            detail::withIndex(
              g,
              [&](auto i) {
                  using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                  auto& s = std::get<decltype(i)::value>(d.reads_);
                  if constexpr(detail::HasSizedPrepare<G>) {
                      constexpr auto off = detail::firstReadOffset(std::span<Step const>{G::Steps});
                      constexpr auto max = ReadSlot<G>::Bytes - off;
                      auto const     n   = G::prepare(s.request, std::span<std::byte>{s.buf});
                      if(n > max) { detail::logOversizedRead(Chip::Name, d.dev_->address, n, max); }
                      s.len        = static_cast<std::uint8_t>(n > max ? max : n);
                      s.validBytes = static_cast<std::uint8_t>(off + s.len);
                  } else if constexpr(detail::HasRequest<G>) {
                      G::prepare(s.request, std::span<std::byte>{s.buf});
                  }
                  if constexpr(detail::hasCounted(std::span<Step const>{G::Steps})) {
                      // until a counted read of this run says how much it read
                      s.len        = static_cast<std::uint8_t>(ReadSlot<G>::Bytes);
                      s.validBytes = static_cast<std::uint8_t>(ReadSlot<G>::Bytes);
                  }
              },
              std::make_index_sequence<Reads::size>{});
        }

        static detail::DecodeResult decode(EngineStateT& e,
                                           std::uint8_t  g) {
            auto&                d = self_(e);
            detail::DecodeResult r{};
            detail::withIndex(
              g,
              [&](auto i) {
                  using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                  auto& s = std::get<decltype(i)::value>(d.reads_);
                  if constexpr(ConfirmChange<G> != 0) {
                      if(!d.template confirmed_<G>(s)) {
                          r = {DecodeKind::retry, 0};
                          return;
                      }
                  }
                  if constexpr(detail::HasSample<G>) {
                      // the description's verdict is the engine's (one DecodeKind); only a retry carries a time
                      auto const outcome = d.template decode_<G>(s);
                      r = {outcome.kind,
                           outcome.kind == DecodeKind::retry
                             ? static_cast<std::uint32_t>(outcome.retryAfter.count())
                             : 0U};
                      if(outcome.kind == DecodeKind::ok) {
                          s.sample = outcome.value;
                          if constexpr(Timed<G>) {
                              d.template noteSample_<G>(s.history, d.pending_.stamp());
                          }
                      } else if(outcome.kind == DecodeKind::unchanged) {
                          if constexpr(detail::KeepsHistory<G>) { ++s.history.unchanged; }
                      }
                  } else {
                      r = {DecodeKind::ok, 0};
                  }
              },
              std::make_index_sequence<Reads::size>{});
            return r;
        }

        static constexpr std::span<detail::WriteGroupInfo const> writes
          = std::span<detail::WriteGroupInfo const>{WriteInfos}.first(Writes::size);

        static detail::WriteSlotBase<Clock>& writeSlot(EngineStateT& e,
                                                       std::uint8_t  w) {
            detail::WriteSlotBase<Clock>* r = nullptr;
            detail::withIndex(
              w,
              [&](auto i) { r = &std::get<decltype(i)::value>(self_(e).writes_); },
              std::make_index_sequence<Writes::size>{});
            if(r == nullptr) { std::unreachable(); }   // as in readSlot
            return *r;
        }

        static void encodeWrite(EngineStateT& e,
                                std::uint8_t  w,
                                std::uint8_t  item) {
            auto& d = self_(e);
            detail::withIndex(
              w,
              [&](auto i) {
                  using W = typename detail::Nth<Writes>::template type<decltype(i)::value>;
                  d.template encodeWrite_<W>(std::get<decltype(i)::value>(d.writes_), item);
              },
              std::make_index_sequence<Writes::size>{});
        }

        static void afterWrite(EngineStateT& e,
                               std::uint8_t  w,
                               std::uint8_t  item,
                               TimePoint) {
            auto& d            = self_(e);
            d.withdrawnOnWire_ = false;
            // The part now holds the value, so what the chip State says about it follows: a
            // full scale that changes what decode() multiplies by. (The read-back it owes is
            // the engine's: finishWrite.)
            if(auto const applied = writeOpsTable[w].applied) { applied(d, item); }
        }

        static detail::VerifiedWriteSlotBase<Clock>& verifySlot(EngineStateT& e,
                                                                std::uint8_t  w) {
            return self_(e).verifySlot_(w);
        }

        static detail::VerifyRead encodeForVerify(EngineStateT& e,
                                                  std::uint8_t  w,
                                                  std::uint32_t item) {
            return writeOpsTable[w].encodeForVerify(self_(e), item);
        }

        // verify_ is looked into only by a chip that verifies (VerifyBufs has no members otherwise), and
        // the hook is called only for a group that does: a member of a class template, so the discarded
        // branch is not instantiated.
        static bool verifyMatches(EngineStateT& e,
                                  std::uint8_t  w) {
            if constexpr(AnyVerify) {
                auto&      d = self_(e);
                auto const n = static_cast<std::size_t>(d.verify_.len);
                return writeOpsTable[w].matches(std::span<std::byte const>{d.verify_.want}.first(n),
                                                std::span<std::byte const>{d.verify_.got}.first(n));
            } else {
                static_cast<void>(e);
                static_cast<void>(w);
                return true;
            }
        }

        static void applyInitial(EngineStateT& e,
                                 std::uint8_t  w,
                                 std::uint32_t known) {
            writeOpsTable[w].applyInitial(self_(e), known);
        }

        static constexpr std::string_view name          = Chip::Name;
        static constexpr std::uint8_t     registerBytes = static_cast<std::uint8_t>(RegisterBytes);
        static constexpr bool             initEmpty     = InitSteps.empty();
    };

    /// The hooks the engine's reset() needs, called directly (restart() runs it outside any
    /// turn, so it cannot name the engine that drives the device - a Bus's device set or the
    /// table): this chip's own hooks as a device set's dispatch, nothing else of the engine.
    struct SelfOps : ChipHooks {
        static constexpr bool IsSetOps = true;
    };

    /// The chip's table: `static constexpr`, so it is one table in flash per DeviceCore and nothing per object --
    /// the engine takes it as an argument rather than the device carrying a pointer. Generated from the hook list,
    /// in its order (the order of `detail::Ops`' fields by construction).
#define KVASIR_DEVICE_OPS_INIT_HOOK(name, ...)   .name = &ChipHooks::name,
#define KVASIR_DEVICE_OPS_INIT_CONST(name, Type) .name = ChipHooks::name,
    static constexpr detail::Ops<Port, Clock> EngineOps{
      KVASIR_ENGINE_HOOKS(KVASIR_DEVICE_OPS_INIT_HOOK,
                          KVASIR_DEVICE_OPS_INIT_HOOK,
                          KVASIR_DEVICE_OPS_INIT_HOOK,
                          KVASIR_DEVICE_OPS_INIT_CONST)};
#undef KVASIR_DEVICE_OPS_INIT_HOOK
#undef KVASIR_DEVICE_OPS_INIT_CONST

public:
    // -- the application's side ------------------------------------------------------------

    /// Once per loop turn: everything happens here (Engine.hpp).
    void turn() { turn(Clock::now()); }

    /// The same with the clock read by the caller (Bus::turn).
    ///
    /// A device on its own is driven through its chip's table (EngineOps), so every chip on
    /// the port shares one engine. With EngineFeatures::directOps it is a device set of one
    /// (SetOps.hpp) instead: direct calls, no table read -- and an engine per chip type.
    /// A Bus or a DeviceSet drives its parts through the set's dispatch either way.
    void turn(TimePoint now) {
        if constexpr(Port::Features.directOps) {
            detail::Engine<Port, Clock>::turn(*this,
                                              detail::SetOpsFor<Port, Clock, DeviceCore>{},
                                              now);
        } else {
            detail::Engine<Port, Clock>::turn(*this, EngineOps, now);
        }
    }

    /// Start over from the reset, as after power-on: for a caller that did to the part what the
    /// engine cannot see -- cycled the supply it shares with others (PowerRail.hpp). Whatever
    /// was on the wire is abandoned, a write in flight is owed again, the part counts as
    /// present and is brought up from the start (StartupDelay, the identity, Init), and
    /// everything the application had written is re-sent after it, as after any bring-up.
    void restart() {
        // SelfOps whatever the dispatch: a device a Bus or a DeviceSet drives would otherwise
        // pull this chip's whole table, and with it a table engine, into an image that has none.
        detail::Engine<Port, Clock>::reset(*this, SelfOps{});
        presence_.restart();
        ring_();
    }

    /// Where the device is with its part:
    /// - `absent` while it is parked after NAKs in a row, and probed now and then (Presence.hpp);
    /// - `answering` once it is through its bring-up *and* the part has acknowledged a
    ///   transaction since the device (re)started;
    /// - `starting` in between;
    /// - `offline` while the bridge it is behind is not active (Bridge.hpp): nothing is sent,
    ///   counted or probed, latest() keeps the last sample and valid() is false.
    ///
    /// A chip with no Init script sends nothing to come up, so it is `starting` until its first
    /// read or write is acknowledged: an ADS1115 left in the box is never answering. The engine
    /// does not wait for it -- reads and writes go out while `starting` -- so a part with no Init
    /// and no cyclic read answers with the first thing the application asks of it.
    [[nodiscard]] Link link() const {
        if constexpr(Port::Features.switchable) {
            if(this->dev_->isSwitchable()) {
                if(offline_.offline() || gateOffline_() || disabled_()) { return Link::offline; }
            }
        }
        if(!presence_.present()) { return Link::absent; }
        return up_ && acked_ ? Link::answering : Link::starting;
    }

    /// One line for a log: what the device is, where, and how it is doing. What Bus::logHealth
    /// says per device, for a firmware with one device and no Bus.
    void logHealth() const {
        if constexpr(Port::Features.stats) {
            detail::logHealth(Chip::Name,
                              this->dev_->address,
                              link(),
                              samples(),
                              rejected(),
                              errors());
        } else {
            detail::logHealth(Chip::Name, this->dev_->address, link());
        }
    }

    [[nodiscard]] bool answering() const { return link() == Link::answering; }

    [[nodiscard]] bool absent() const { return link() == Link::absent; }

    /// Behind a bridge that is not active (Bridge.hpp): not talked to, on purpose.
    [[nodiscard]] bool offline() const { return link() == Link::offline; }

    /// Bring-ups finished since power-up. It steps whenever the device is configured again --
    /// a probe that found the part back, the fault threshold, a reset -- which is when anything
    /// the application set up on the part beyond the engine's own write groups is owed again.
    [[nodiscard]] std::uint16_t bringUps() const { return bringUps_; }

    /// True once per bring-up for the reader that keeps `seen`: the place to redo whatever the
    /// application set up on the part beyond the engine's own write groups. Like fresh(seen).
    [[nodiscard]] bool broughtUp(std::uint16_t& seen) const {
        if(seen == bringUps_) { return false; }
        seen = bringUps_;
        return true;
    }

    /// NAKs in a row, towards being parked.
    [[nodiscard]] std::uint8_t consecutiveNaks() const { return presence_.consecutiveNaks(); }

    /// Answering, and every *cyclic* read group with a Sample has delivered one since the last
    /// bring-up: a reading from before the part went away is not a reading of the part now.
    ///
    /// A group with no Period only runs when request<G>() asks for it, so it cannot be part
    /// of "the readings are good": counting it would leave valid() false for ever on a chip
    /// that also polls (an MPR121 with its filtered-data group), and always false on one
    /// that only answers on demand (a memory). Ask those with valid<G>() or fresh<G>().
    [[nodiscard]] bool valid() const {
        if(!answering()) { return false; }
        bool ok = true;
        detail::forEach<Reads>([&](auto i) {
            using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
            if constexpr(detail::HasSample<G> && periodOf<G>() > std::chrono::milliseconds::zero())
            {
                ok = ok && std::get<ReadSlot<G>>(reads_).current;
            }
        });
        return ok;
    }

    /// Answering, and read group G has delivered since the last bring-up.
    template<typename G>
    [[nodiscard]] bool valid() const {
        return answering() && std::get<ReadSlot<G>>(reads_).current;
    }

    /// The latest Sample of read group G.
    template<typename G>
    [[nodiscard]] auto const& latest() const {
        return std::get<ReadSlot<G>>(reads_).sample;
    }

    /// The same, with the group as a tag -- `dev.latest(Chips::Sht3x::Measurement{})` -- for
    /// generic code, where `dev.template latest<G>()` is the alternative.
    template<ReadGroup G>
    [[nodiscard]] auto const& latest(G) const {
        return latest<G>();
    }

    template<ReadGroup G>
    [[nodiscard]] std::uint32_t seq(G) const {
        return seq<G>();
    }

    template<ReadGroup G>
    [[nodiscard]] bool fresh(G) {
        return fresh<G>();
    }

    template<ReadGroup G>
    [[nodiscard]] bool fresh(G,
                             std::uint32_t& seen) const {
        return fresh<G>(seen);
    }

    template<ReadGroup G>
    [[nodiscard]] bool valid(G) const {
        return valid<G>();
    }

    template<ReadGroup G>
    Ticket request(G) {
        return request<G>();
    }

    template<WriteGroup W>
    bool set(W,
             typename W::Value const& v) {
        return set<W>(0, v);
    }

    template<WriteGroup W>
    bool set(W,
             std::size_t              i,
             typename W::Value const& v) {
        return set<W>(i, v);
    }

    /// The period read group G runs at now: what period<G>(ms) set, else what the chip State
    /// says (G::periodNow(State const&)), else the description's Period.
    template<typename G>
    [[nodiscard]] std::chrono::milliseconds period() const {
        return periodNow_<G>();
    }

    /// Run cyclic read group G every `p` from now on; 0 parks it until request<G>() asks. The
    /// description's Period stays the nominal rate the bus load is planned for; this is the rate
    /// a firmware picks at run time, and the late-gap count follows it -- slower while a display is off, faster while a
    /// value is being watched.
    template<typename G>
    void period(std::chrono::milliseconds p)
        requires(Port::Features.runtimePeriod)
    {
        static_assert(periodOf<G>() > std::chrono::milliseconds::zero(),
                      "period<G>(p): G is not a cyclic read group (it has no Period), so it "
                      "only runs when request<G>() asks");
        auto& s     = std::get<ReadSlot<G>>(reads_);
        s.period    = p;
        s.periodSet = true;
        s.due       = p > std::chrono::milliseconds::zero() ? Clock::now() + p : TimePoint::max();
        ring_();
    }

    /// The latest Sample of the primary read group (PrimaryRead).
    [[nodiscard]] auto const& latest() const
        requires(HasPrimaryRead)
    {
        return latest<PrimaryRead>();
    }

    /// Steps with every new sample of G.
    template<typename G>
    [[nodiscard]] std::uint32_t seq() const {
        return std::get<ReadSlot<G>>(reads_).seq;
    }

    [[nodiscard]] std::uint32_t seq() const
        requires(HasPrimaryRead)
    {
        return seq<PrimaryRead>();
    }

    /// True once per new sample of G. One cursor for every caller: call it from one place,
    /// or give each reader its own with fresh<G>(seen).
    template<typename G>
    [[nodiscard]] bool fresh() {
        auto& s = std::get<ReadSlot<G>>(reads_);
        if(s.seen == s.seq) { return false; }
        s.seen = s.seq;
        return true;
    }

    /// True once per new sample of G for the reader that keeps `seen`, so any number of
    /// readers see every sample without taking it from each other.
    template<typename G>
    [[nodiscard]] bool fresh(std::uint32_t& seen) const {
        auto const seq = std::get<ReadSlot<G>>(reads_).seq;
        if(seen == seq) { return false; }
        seen = seq;
        return true;
    }

    [[nodiscard]] bool fresh()
        requires(HasPrimaryRead)
    {
        return fresh<PrimaryRead>();
    }

    [[nodiscard]] bool fresh(std::uint32_t& seen) const
        requires(HasPrimaryRead)
    {
        return fresh<PrimaryRead>(seen);
    }

    template<typename G>
    [[nodiscard]] std::uint32_t samples() const
        requires(Port::Features.stats)
    {
        return std::get<ReadSlot<G>>(reads_).samples;
    }

    /// Samples across every read group, over the device's life.
    [[nodiscard]] std::uint32_t samples() const
        requires(Port::Features.stats)
    {
        std::uint32_t n = 0;
        detail::forEach<Reads>([&](auto i) { n += std::get<decltype(i)::value>(reads_).samples; });
        return n;
    }

    /// Changed readings of G that the next read contradicted (ConfirmChange).
    template<typename G>
        requires(ConfirmChange<G> != 0)
    [[nodiscard]] std::uint32_t disagreements() const {
        return std::get<ReadSlot<G>>(reads_).confirm.disagreements;
    }

    /// Frames of G the decode rejected (CRC, busy past maxRetries, impossible values).
    template<typename G>
    [[nodiscard]] std::uint32_t rejected() const
        requires(Port::Features.stats)
    {
        return std::get<ReadSlot<G>>(reads_).rejected;
    }

    /// Frames of G that were well formed and said nothing new (Outcome::unchanged). Kept
    /// only for a group whose decode takes the previous Sample, or that says Timestamped.
    template<typename G>
    [[nodiscard]] std::uint32_t unchanged() const
        requires(detail::KeepsHistory<G>)
    {
        return std::get<ReadSlot<G>>(reads_).history.unchanged;
    }

    /// When G's latest Sample arrived on the bus, on the clock: the instant the chip's answer
    /// completed, taken in the interrupt, not the loop turn that noticed it.
    template<typename G>
    [[nodiscard]] TimePoint stamp() const
        requires(Timed<G>)
    {
        return std::get<ReadSlot<G>>(reads_).history.at;
    }

    /// The gaps between G's samples since the last call, measured between the bus completions
    /// of the samples rather than the loop turns that noticed them: the longest, and how many
    /// were two periods or more. Taken and cleared, so one reader; the loop's.
    template<typename G>
    [[nodiscard]] SampleGaps<Duration> takeGaps()
        requires(Timed<G>)
    {
        auto&                      h = std::get<ReadSlot<G>>(reads_).history;
        SampleGaps<Duration> const gaps{.longest = h.longestGap, .late = h.lateGaps};
        h.longestGap = Duration{};
        h.lateGaps   = 0;
        return gaps;
    }

    [[nodiscard]] std::uint32_t rejected() const
        requires(Port::Features.stats)
    {
        std::uint32_t n = 0;
        detail::forEach<Reads>([&](auto i) { n += std::get<decltype(i)::value>(reads_).rejected; });
        return n;
    }

    /// Run read group G once, now (its period, if any, continues): an on-demand read. The
    /// ticket is for answer<G>(); the requests made before a run starts are all served by it.
    /// An atomic step, so an interrupt may ask (a touch controller's INT line).
    template<typename G>
    Ticket request() {
        auto&        s = std::get<ReadSlot<G>>(reads_);
        Ticket const t{s.asked.fetch_add(1, std::memory_order_acq_rel) + 1};
        ring_();
        return t;
    }

    /// Run G once with a Request the group's prepare() turns into bytes. Unlike request<G>()
    /// this stores the Request first and is the loop's to call: an interrupt that wants a
    /// read with an argument sets a flag and lets the loop ask.
    template<typename G>
    Ticket request(typename G::Request const& r) {
        auto& s   = std::get<ReadSlot<G>>(reads_);
        s.request = r;
        Ticket const t{s.asked.fetch_add(1, std::memory_order_acq_rel) + 1};
        ring_();
        return t;
    }

    /// How the request behind `ticket` went: `pending` until a run that started after it has
    /// ended, then how the latest run that served requests of G ended. A request made while the
    /// part is away is not failed for it: it stays pending, and runs once the part is back.
    template<typename G>
    [[nodiscard]] Answer answer(Ticket ticket) const {
        auto const& s = std::get<ReadSlot<G>>(reads_);
        return ticket.id == 0 || ticket.id > s.served ? Answer::pending : s.last;
    }

    /// Set write group W's value; written on the next free turn. False, with nothing to send,
    /// when the group already holds that value -- a chip is not told what it knows, so a caller
    /// can hand over its state every turn -- except for a Transient or AlwaysWrite group, whose
    /// every set is a command. rewrite<W>(v) sends it regardless.
    template<typename W>
    bool set(typename W::Value const& v) {
        return set<W>(0, v);
    }

    /// Set item `i` of write group W.
    template<typename W>
    bool set(std::size_t              i,
             typename W::Value const& v) {
        auto& s = std::get<WriteSlot<W>>(writes_);
        if(i >= WriteSlot<W>::Items) { return false; }
        if constexpr(SkipsUnchanged<W>) {
            if(((s.known >> i) & 1U) != 0 && s.values[i] == v) { return false; }
        }
        store_<W>(s, i, v);
        return true;
    }

    /// Set item `i` of write group W and send it, even when the chip holds that value already:
    /// a register that changed under the engine's feet, or a command bit a description does
    /// not mark AlwaysWrite.
    template<typename W>
    void rewrite(std::size_t              i,
                 typename W::Value const& v) {
        auto& s = std::get<WriteSlot<W>>(writes_);
        if(i >= WriteSlot<W>::Items) { return; }
        store_<W>(s, i, v);
    }

    template<typename W>
    void rewrite(typename W::Value const& v) {
        rewrite<W>(0, v);
    }

    /// Change part of write group W's value in place, leaving the rest of it alone: the
    /// field-at-a-time edit of a config register the application only partly owns.
    ///
    ///     dev.modify<Config>([](auto& c) { c.conversionRate = Rate::Ms110; });
    ///
    /// A Value of named fields plus this is what a hand-written driver spells as a
    /// read-modify-write of the shadow bytes, one shift and mask per setter.
    template<typename W,
             typename F>
    bool modify(F&& f) {
        return modify<W>(0, std::forward<F>(f));
    }

    /// Change part of item `i` of write group W.
    template<typename W,
             typename F>
    bool modify(std::size_t i,
                F&&         f) {
        auto& s = std::get<WriteSlot<W>>(writes_);
        if(i >= WriteSlot<W>::Items) { return false; }
        auto v = s.values[i];
        f(v);
        return set<W>(i, v);
    }

    /// Write group W's values, for a caller that draws into them rather than handing them
    /// over: the page RAM of a display, so the frame exists once instead of twice. Say what
    /// changed with touch<W>(i) -- nothing is sent until you do.
    template<typename W>
    [[nodiscard]] std::span<typename W::Value,
                            WriteSlot<W>::Items>
    items() {
        return std::span<typename W::Value, WriteSlot<W>::Items>{
          std::get<WriteSlot<W>>(writes_).values};
    }

    /// Item i of write group W has been changed in place: send it on the next free turn.
    template<typename W>
    void touch(std::size_t i) {
        if(i >= WriteSlot<W>::Items) { return; }
        auto& s = std::get<WriteSlot<W>>(writes_);
        s.owned = true;
        s.known = s.known | (1U << i);
        s.dirty.fetch_or(1U << i, std::memory_order_release);
        ring_();
    }

    /// Every item of write group W has been changed in place.
    template<typename W>
    void touchAll() {
        auto& s = std::get<WriteSlot<W>>(writes_);
        s.owned = true;
        s.known = WriteSlot<W>::AllItems;
        s.dirty.store(WriteSlot<W>::AllItems, std::memory_order_release);
        ring_();
    }

    template<typename W>
    [[nodiscard]] typename W::Value const& value(std::size_t i = 0) const {
        return std::get<WriteSlot<W>>(writes_).values[i];
    }

    template<typename W>
    [[nodiscard]] std::uint32_t writes() const {
        return std::get<WriteSlot<W>>(writes_).writes;
    }

    /// Writes of every group, over the device's life.
    [[nodiscard]] std::uint32_t writes() const {
        std::uint32_t n = 0;
        detail::forEach<Writes>([&](auto i) { n += std::get<decltype(i)::value>(writes_).writes; });
        return n;
    }

    /// Write group W has something set and not yet written, or on the wire now.
    template<typename W>
    [[nodiscard]] bool pending() const {
        return std::get<WriteSlot<W>>(writes_).dirty.load(std::memory_order_relaxed) != 0
            || (running_ == Running::write && group_ == detail::IndexOf<W, Writes>::value);
    }

    /// Drop what write group W still owes; one on the wire is not retried. For a Transient
    /// write (an EEPROM page) that must not land long after the caller gave up. Returns
    /// whether anything was owed.
    template<typename W>
    bool withdraw() {
        auto&      s    = std::get<WriteSlot<W>>(writes_);
        bool const owed = s.dirty.exchange(0, std::memory_order_acq_rel) != 0;
        bool const onWire
          = running_ == Running::write && group_ == detail::IndexOf<W, Writes>::value;
        if(onWire) { withdrawnOnWire_ = true; }
        ring_();
        return owed || onWire;
    }

    /// Read-backs of write group W that did not match what was written.
    template<typename W>
    [[nodiscard]] std::uint32_t mismatches() const
        requires(detail::Verifies<W>)
    {
        return std::get<WriteSlot<W>>(writes_).verify.mismatches;
    }

    /// Something set and not yet on the wire (or still on it).
    [[nodiscard]] bool pending() const {
        bool any = running_ == Running::write;
        detail::forEach<Writes>([&](auto i) {
            any = any
               || std::get<decltype(i)::value>(writes_).dirty.load(std::memory_order_relaxed) != 0;
        });
        return any;
    }

    /// Full turns run; resting turns (Engine::rest) are not counted.
    [[nodiscard]] std::uint32_t turns() const
        requires(Port::Features.stats)
    {
        return this->turns_;
    }

    /// Transactions that failed (NAK, bus fault, timeout), all phases. A NAK a wake retry
    /// (Chip::WakeRetries) recovered from is not one.
    [[nodiscard]] std::uint32_t errors() const
        requires(Port::Features.stats)
    {
        return errors_;
    }

    /// NAKs that were put on the wire again because the part may still be waking
    /// (Chip::WakeRetries), over the device's life; 0 for a chip that declares none.
    [[nodiscard]] std::uint32_t wakeRetryCount() const {
        if constexpr(WakeRetries > 0) {
            return wake_.retries;
        } else {
            return 0;
        }
    }

    /// What setup() made of the bring-up (the chip id, trimming...).
    [[nodiscard]] State const& state() const { return state_; }

    /// setup() accepted the chip (true when there is no setup()). While it is false and
    /// unidentifiedRetry is set, the description runs nothing but its bring-up (link() stays
    /// `starting`, valid() false); with unidentifiedRetry 0 it runs regardless.
    [[nodiscard]] bool identified() const { return identified_; }

    /// The part's identity registers read what its data sheet says (the description's
    /// `Identity`, RegisterCheck.hpp) in the latest bring-up that got that far; true for a
    /// description that names none. A part that fails this is never `identified()`, so never
    /// `answering()`, and nothing but the identity reads has gone to it.
    [[nodiscard]] bool identityMatched() const { return identityMatched_; }

    /// Bring-ups whose setup() said "not this chip".
    [[nodiscard]] std::uint16_t unidentified() const
        requires(Port::Features.stats)
    {
        return unidentified_;
    }

private:
    /// set() compares against what the group holds: only a group that is a state (not
    /// Transient, not AlwaysWrite) and whose Value can be compared.
    template<typename W>
    static constexpr bool SkipsUnchanged = !detail::HasTransient<W> && !detail::AlwaysWrites<W>
                                        && std::equality_comparable<typename W::Value>;

    template<typename W,
             typename S>
    void store_(S&                       s,
                std::size_t              i,
                typename W::Value const& v) {
        s.values[i] = v;
        s.owned     = true;
        s.known     = s.known | (1U << i);
        s.dirty.fetch_or(1U << i, std::memory_order_release);
        ring_();
    }

    /// Wake the engine (EngineState::doorbell_); after the store it announces.
    void ring_() {
        if constexpr(Port::Features.rest) {
            this->doorbell_.store(true, std::memory_order_release);
        }
    }

    /// A timestamped sample of G landed at `at`: the gap since the one before, for takeGaps().
    template<typename G,
             typename H>
    void noteSample_(H&        h,
                     TimePoint at) const {
        if(h.at != TimePoint{}) {
            auto const gap = at - h.at;
            h.longestGap   = std::max(h.longestGap, gap);
            if constexpr(periodOf<G>() > std::chrono::milliseconds::zero()
                         && !detail::HasDynamicPeriod<G>)
            {
                if(gap >= periodNow_<G>() * 2) { ++h.lateGaps; }
            }
        }
        h.at = at;
    }

    // -- the turn --------------------------------------------------------------------------

    [[nodiscard]] bool gateOffline_() const {
        auto const offline = this->dev_->gateOffline;
        return offline != nullptr && offline(*this);
    }

    [[nodiscard]] bool disabled_() const { return this->disabledNow(); }

    // -- running a script ------------------------------------------------------------------

    /// The run is about to wait -- a step's delay, a wait step, a check or a decode asking to
    /// be run again -- so the gate is let go for the wait and claimed again before the next
    /// transaction (Engine::submit). Nothing is on the wire for this device while it waits, and a
    /// part behind a switch does not see the switch move. Held through the wait, a part polling
    /// a flag every 15 ms on a 100 ms period would keep its channel for most of each period and
    /// starve the parts on the other channels.
    void letGateGo_() { detail::Engine<Port, Clock>::releaseGate(*this); }

    /// Write group @p w's slot, as the part every group has: a compare per group, no body per
    /// group (the paths that use it are one body for all of them).
    template<std::size_t I = 0>
    detail::WriteSlotBase<Clock>& writeSlotBase_(std::size_t w) {
        if constexpr(I >= Writes::size) {
            std::unreachable();   // w is always one of the groups
        } else {
            if(w == I) { return std::get<I>(writes_); }
            return writeSlotBase_<I + 1>(w);
        }
    }

    /// The same for a group that reads back; never called for one that does not.
    template<std::size_t I = 0>
    detail::VerifiedWriteSlotBase<Clock>& verifySlot_(std::size_t w) {
        if constexpr(I >= Writes::size) {
            std::unreachable();
        } else {
            if constexpr(detail::Verifies<typename detail::Nth<Writes>::template type<I>>) {
                if(w == I) { return std::get<I>(writes_); }
            }
            return verifySlot_<I + 1>(w);
        }
    }

    [[nodiscard]] std::span<Step const> readSteps_(std::size_t g) const {
        std::span<Step const> r{};
        detail::withIndex(
          g,
          [&](auto i) {
              using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
              r       = std::span<Step const>{G::Steps};
          },
          std::make_index_sequence<Reads::size>{});
        return r;
    }

    [[nodiscard]] std::span<std::byte> readBuffer_(std::size_t g) {
        std::span<std::byte> r{};
        detail::withIndex(
          g,
          [&](auto i) { r = std::span<std::byte>{std::get<decltype(i)::value>(reads_).buf}; },
          std::make_index_sequence<Reads::size>{});
        return r;
    }

    [[nodiscard]] std::span<std::byte> writeBuffer_(std::size_t g) {
        std::span<std::byte> r{};
        detail::withIndex(
          g,
          [&](auto i) { r = std::span<std::byte>{std::get<decltype(i)::value>(writes_).buf}; },
          std::make_index_sequence<Writes::size>{});
        return r;
    }

    /// The running read group's ready() over its buffer.
    [[nodiscard]] bool ready_() const {
        bool r = true;
        detail::withIndex(
          group_,
          [&](auto i) {
              using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
              if constexpr(detail::HasReady<G>) {
                  auto const& sl = std::get<decltype(i)::value>(reads_);
                  r = G::ready(Bytes{std::span<std::byte const>{sl.buf}.first(sl.validBytes)});
              }
          },
          std::make_index_sequence<Reads::size>{});
        return r;
    }

    // -- what happens at the end of a script -----------------------------------------------

    /// The identity registers the bring-up has just read, against the oracle's values under
    /// their masks. The values live on the stack for the compare and the description's
    /// identified() hook only: what of them a description keeps is in its State (deviceId, a
    /// scale), the one place a part's identity is kept.
    [[nodiscard]] bool identityMatches_() {
        if constexpr(HasIdentity) {
            std::array<std::uint32_t, Chip::Identity.size()> ids{};
            std::size_t                                      offset = IdentityOffset;
            for(std::size_t i = 0; i < Chip::Identity.size(); ++i) {
                auto const& check = Chip::Identity[i];
                ids[i]            = check.value(init_.data() + offset);
                offset += check.width;
            }
            for(std::size_t i = 0; i < Chip::Identity.size(); ++i) {
                auto const& check = Chip::Identity[i];
                if(!check.matches(ids[i])) {
                    if(!unidentifiedLogged_) {
                        detail::logIdentityMismatch(Chip::Name,
                                                    this->dev_->address,
                                                    check.name,
                                                    static_cast<std::uint16_t>(check.reg),
                                                    ids[i],
                                                    static_cast<std::uint32_t>(check.expect),
                                                    static_cast<std::uint32_t>(check.mask));
                    }
                    return false;
                }
            }
            // The part is the chip: what of its identity the description acts on (a scale that
            // differs between two members of a family) goes into its State before Init runs.
            if constexpr(requires(State& st) {
                             Chip::identified(std::span<std::uint32_t const>{}, st);
                         })
            {
                Chip::identified(std::span<std::uint32_t const>{ids}, state_);
            }
        }
        return true;
    }

    template<typename O>
    void finishInit_(O const&  ops,
                     TimePoint now) {
        detail::Engine<Port, Clock>::finishInit(*this, ops, now);
    }

    /// Whether G's bytes may be decoded (ConfirmChange). A candidate lives within one run, so
    /// nothing a part answers after a fault confirms what it said before it; the first
    /// reading after a bring-up is confirmed too.
    template<typename G,
             typename S>
    bool confirmed_(S& slot) {
        auto&      c    = slot.confirm;
        auto const data = std::span<std::byte const>{slot.buf}.first(slot.validBytes);
        auto const same = [&](auto const& bytes, std::uint8_t n) {
            return n == data.size() && std::ranges::equal(data, std::span{bytes}.first(n));
        };
        if(slot.retries == 0) {
            c.hasCandidate = false;
            c.unconfirmed  = 0;
        }
        bool const unchanged = slot.current && same(c.taken, c.takenBytes);
        bool const agrees    = c.hasCandidate && same(c.candidate, c.candidateBytes);
        if(c.hasCandidate && !agrees) { ++c.disagreements; }
        if(!unchanged && !agrees && c.unconfirmed < ConfirmChange<G>) {
            std::ranges::copy(data, c.candidate.begin());
            c.candidateBytes = static_cast<std::uint8_t>(data.size());
            c.hasCandidate   = true;
            ++c.unconfirmed;
            return false;
        }
        std::ranges::copy(data, c.taken.begin());
        c.takenBytes   = static_cast<std::uint8_t>(data.size());
        c.hasCandidate = false;
        c.unconfirmed  = 0;
        return true;
    }

    /// The group's decode, with the chip State or the previous Sample when it takes one,
    /// its result lifted to an Outcome when it returns a plain Sample.
    ///
    /// The previous-Sample form is for a chip whose frames only mean something next to the
    /// last one: a touch controller's "no finger" is a release the first time and nothing
    /// the ten times after it, which is what Outcome::unchanged() is for. A group takes one
    /// or the other, never both -- the two would be told apart only by their types.
    template<typename G,
             typename S>
    [[nodiscard]] auto decode_(S const& slot) const {
        Bytes const data{std::span<std::byte const>{slot.buf}.first(slot.validBytes)};
        using Sample = typename G::Sample;

        static constexpr bool withState = requires { G::decode(data, state_); };
        static constexpr bool withPrev  = requires { G::decode(data, slot.sample); };
        static_assert(!(withState && withPrev),
                      "a read group's decode() takes the chip State or the previous Sample, "
                      "not both");

        auto const call = [&] {
            if constexpr(withState) {
                return G::decode(data, state_);
            } else if constexpr(withPrev) {
                return G::decode(data, slot.sample);
            } else {
                return G::decode(data);
            }
        };
        if constexpr(std::is_same_v<decltype(call()), Sample>) {
            return Outcome<Sample>::ok(call());
        } else {
            return call();
        }
    }

    /// The period group G runs at now: what period<G>(ms) set, else what `G::periodNow(State
    /// const&)` makes of the chip State when the group has one -- a rate written to the part
    /// at run time (IIS2DULPX's CTRL5) is then the rate it is read at from the next sample on
    /// -- else its constant `Period`.
    template<typename G>
    [[nodiscard]] std::chrono::milliseconds periodNow_() const {
        auto const& s = std::get<ReadSlot<G>>(reads_);
        if constexpr(Port::Features.runtimePeriod) {
            if(s.periodSet) { return s.period; }
        }
        if constexpr(!detail::HasDynamicPeriod<G>
                     && requires(State const& st) {
                            { G::periodNow(st) } -> std::convertible_to<std::chrono::milliseconds>;
                        })
        {
            std::chrono::milliseconds const p = G::periodNow(state_);
            return p > std::chrono::milliseconds::zero() ? p : periodOf<G>();
        } else {
            return periodOf<G>();
        }
    }

    // -- choosing what runs next -----------------------------------------------------------

    /// One item of a write group encoded into the script the engine then walks. Four
    /// shapes, two questions: does the group have Items (does encode() take the index), and
    /// is the item one transaction or several.
    template<typename W,
             typename S>
    void encodeWrite_(S&          s,
                      std::size_t item) {
        auto const call = [&] {
            if constexpr(detail::HasItems<W>) {
                return W::encode(s.values[item], item, std::span<std::byte>{s.buf});
            } else {
                return W::encode(s.values[0], std::span<std::byte>{s.buf});
            }
        };
        if constexpr(detail::MultiStepWrite<W>) {
            auto const steps   = call();
            writeScript_.count = static_cast<std::uint8_t>(steps.size());
            std::ranges::copy(steps, writeScript_.steps.begin());
        } else if constexpr(MultiStepWrites) {
            // a single-step group on a chip that also has a multi-step one
            writeScript_.steps[0] = call();
            writeScript_.count    = 1;
        } else {
            current_ = call();
        }
    }

    // Where the run is, what it waits for and what has happened to it: detail::EngineState,
    // the base of this class (Engine.hpp), because none of it depends on the chip.
    State                            state_{};
    std::array<std::byte, InitBytes> init_{};
    bool                             identityMatched_{!HasIdentity};
    [[no_unique_address]] detail::VerifyBufs<AnyVerify, MaxVerifyBytes>       verify_{};
    [[no_unique_address]] detail::WriteScript<MultiStepWrites, MaxWriteSteps> writeScript_{};
    [[no_unique_address]] detail::WakeSlot<(WakeRetries > 0)>                 wake_{};
    bool                                                                      withdrawnOnWire_{};
    ReadSlots                                                                 reads_{};
    WriteSlots                                                                writes_{};
};

}   // namespace Kvasir::Engine
