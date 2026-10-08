#pragma once

/// The state machine `Device<>` runs, in one place that knows nothing about the chip.
///
/// `Device<I2c, Clock, Chip, Config, Reset, Gate>` is instantiated once per device, so every
/// line of the engine used to be printed into the image that often: 96 KB of `Device<>` members
/// plus 40 KB of `Bus::handler()` with each device's `handler()` inlined, in a 312 KB firmware
/// with 57 devices. `--icf=all` is already in the SDK's link flags
/// and folds none of it -- the copies differ by constants and types -- so the only way down is
/// to share by design.
///
/// The state is what makes that possible: of `Device`'s members, the ones below know nothing
/// about the chip. Where the run is (`phase_`, `running_`, `group_`, `step_`, `item_`), what it
/// waits for (`waiting_`, `waitUntil_`, `holdUntil_`), what the bus owes it (`inFlight_`,
/// `submittedAt_`), and what has happened to it so far (`errors_`, `bringUps_`,
/// `consecutiveFailures_`) are the same fields for a thermometer and a display. Only the
/// buffers, the decoded samples and the chip's own functions differ, and those stay in
/// `Device`.
///
/// `docs/engine-factoring.md` has the plan this is the second stage of; `Clock` is the only
/// template parameter, so a firmware has one of these however many devices it has.
#include "../Duration.hpp"
#include "EngineFeatures.hpp"
#include "EngineLog.hpp"
#include "Pending.hpp"
#include "Presence.hpp"
#include "Step.hpp"

#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

namespace Kvasir::I2C::detail {

/// What became of an on-demand read (`Device::request<G>()` and `answer<G>()`): what a run of
/// the group did for the requests it took on.
enum class Answer : std::uint8_t {
    pending,     ///< no run has ended since the request: not started yet, or on the wire
    ok,          ///< a run after the request delivered a sample
    unchanged,   ///< it ran, and the part had nothing new to say (Outcome::unchanged)
    rejected,    ///< it ran, and decode rejected what came back or its retries ran out
    failed,      ///< the part or the bus failed the run: a NAK, a bus fault, the in-flight net
};

/// Where a device is with its part, from power-on to running.
enum class Phase : std::uint8_t { reset, resetHeld, settle, run };

/// Which kind of script is on the wire, if any.
enum class Running : std::uint8_t { none, init, read, write, verify };

template<typename Port, typename Clock>
struct DeviceOps;

/// A bus as the engine sees it: buses with the same Request, Result and engine features (an
/// RP2350's I2C0 and I2C1) are one port and share the engine and each chip's code.
template<typename Req, typename Res, EngineFeatures F = EngineFeatures{}>
struct BusPort {
    using Request                            = Req;
    using Result                             = Res;
    static constexpr EngineFeatures Features = F;
};

template<typename Bus>
using PortOf = BusPort<typename Bus::Request, typename Bus::Result, featuresOf<Bus>()>;

/// Stand-ins for the state of a feature the port leaves out (EngineFeatures): no bytes.
struct NoBridge {
    [[nodiscard]] static constexpr bool offline() { return false; }
};

/// Why a switchable part is offline; the reasons come and go independently.
struct BridgeState {
    // The words first: 12 bytes, not 16.
    std::uint32_t generation{};
    /// `*DeviceOps::enabledGeneration` when `disabled` was last taken from Config::enabled();
    /// `enabledKnown` is false until it has been once.
    std::uint32_t enabledSeen{};
    bool          gateOff{};     ///< offline because the bridge is not active
    bool          disabled{};    ///< offline because Config::enabled() is false
    bool          lostPower{};   ///< reset() done since the part went offline
    bool          enabledKnown{};

    [[nodiscard]] bool offline() const { return gateOff || disabled; }
};

/// The engine's own state: every member of `Device` that does not depend on the chip. `Port`
/// and `Clock` are one type each per firmware, so this exists once however many devices there
/// are.
template<typename Port, typename Clock>
struct EngineState {
    using TimePoint = typename Clock::time_point;
    using PendingT  = Pending<Port, Clock>;

    Phase   phase_{Phase::reset};
    Running running_{Running::none};

    bool up_{false};
    bool acked_{false};   ///< the part acknowledged a transaction since the last start
    bool identified_{false};
    bool waiting_{false};
    bool afterDelay_{false};
    bool inFlight_{false};
    /// The last bring-up failed: back off before the next (EngineFeatures::faultBackoff).
    [[no_unique_address]] IfFeature<Port::Features.faultBackoff, bool, 20> initFailed_{};
    bool unidentifiedLogged_{false};
    bool oracleFailed_{false};   ///< this bring-up: an Identity register did not match

    std::uint8_t group_{};
    std::uint8_t step_{};
    std::uint8_t item_{};
    std::uint8_t consecutiveFailures_{};
    /// Which of its device set's chips this device is (SetOps::indexOf, set by the Bus); 0,
    /// and unused, for a device the table drives.
    std::uint8_t setIndex_{};

    /// The transaction on the wire, or the one about to go.
    Step current_{};

    TimePoint waitUntil_{};
    /// Nothing but the running script until then, after a fault (EngineFeatures::faultBackoff).
    [[no_unique_address]] std::conditional_t<Port::Features.faultBackoff, TimePoint, NoStamp>
      holdUntil_{};
    /// When the request on the wire was submitted: the in-flight net measures from it
    /// (EngineFeatures::inFlightNet; nothing without it).
    [[no_unique_address]] std::conditional_t<Port::Features.inFlightNet, TimePoint, NoStamp>
      submittedAt_{};

    /// Statistics (EngineFeatures::stats), with turns_: nothing without them.
    [[no_unique_address]] StatCount<Port::Features.stats> errors_{};
    std::uint16_t                                         bringUps_{};
    /// Bring-ups setup() turned down.
    [[no_unique_address]] StatCount<Port::Features.stats, std::uint16_t> unidentified_{};

    /// The handshake with the bus: the callback handed to it, and what came back.
    PendingT pending_{};

    /// This device's own table (DeviceOps), set by its constructor.
    DeviceOps<Port, Clock> const* dev_{};
    /// Full turns run: what the device costs the loop.
    [[no_unique_address]] StatCount<Port::Features.stats> turns_{};

    /// Whether the part is there (Presence.hpp), and why it is offline, if it is.
    /// Nothing for a feature the port leaves out (EngineFeatures): NoPresence answers
    /// "present", NoBridge "online".
    [[no_unique_address]] std::
      conditional_t<Port::Features.presence, Presence<Clock>, NoPresence<Clock>> presence_{};
    [[no_unique_address]] std::conditional_t<Port::Features.switchable, BridgeState, NoBridge>
      bridge_{};

    /// Resting (Engine::handler): turns before `quietUntil_` are skipped unless `doorbell_`
    /// rang (request, set, period, restart) or the bridge or Config::enabled() changed. The two
    /// flags sit in front of the time point, in the tail of `bridge_`'s word: 144 bytes, not 152.
    /// Nothing of the three without resting turns (EngineFeatures::rest).
    [[no_unique_address]] IfFeature<Port::Features.rest, bool, 21>              resting_{};
    [[no_unique_address]] IfFeature<Port::Features.rest, std::atomic<bool>, 22> doorbell_{};
    [[no_unique_address]] IfFeature<Port::Features.rest, TimePoint, 23>         quietUntil_{};

    /// The request on the wire has been out no longer than the device's in-flight timeout:
    /// always, without the in-flight net (whose timeout is then not in the table either).
    template<typename Ops>
    [[nodiscard]] bool withinInFlightNet(TimePoint  now,
                                         Ops const& d) const {
        if constexpr(Port::Features.inFlightNet) {
            return now - submittedAt_ <= std::chrono::milliseconds{d.inFlightTimeoutMs};
        } else {
            return true;
        }
    }

    /// Stamps a submit for the in-flight net; nothing without it (no clock read either).
    void stampSubmit() {
        if constexpr(Port::Features.inFlightNet) { submittedAt_ = Clock::now(); }
    }

    /// `*DeviceOps::enabledGeneration` now, 0 without one. Read fresh every time (the
    /// application may step it from an interrupt) and before Config::enabled() is asked.
    [[nodiscard]] std::uint32_t enabledGenerationNow() const {
        auto const generation = dev_->enabledGeneration;
        if(generation == nullptr) { return 0; }
        std::uint32_t const value = *static_cast<std::uint32_t const volatile*>(generation);
        std::atomic_signal_fence(std::memory_order_acquire);
        return value;
    }

    /// Config::enabled() is false - asked only when its generation moved since `bridge_.disabled`
    /// was taken. @p generation is the one read BEFORE asking: a step that comes while the
    /// function runs then makes the next turn ask again, instead of being taken for this answer.
    [[nodiscard]] bool disabledNow(std::uint32_t generation) const {
        if(dev_->enabled == nullptr) { return false; }
        if(dev_->enabledGeneration != nullptr && bridge_.enabledKnown
           && generation == bridge_.enabledSeen)
        {
            return bridge_.disabled;
        }
        return !dev_->enabled(dev_->enabledArg);
    }

    [[nodiscard]] bool disabledNow() const { return disabledNow(enabledGenerationNow()); }

    /// Records that `bridge_.disabled` is Config::enabled()'s answer as of @p generation (read
    /// before it was asked, disabledNow()).
    void noteEnabledSeen(std::uint32_t generation) {
        if(dev_->enabledGeneration != nullptr) {
            bridge_.enabledSeen  = generation;
            bridge_.enabledKnown = true;
        }
    }
};

/// The half of a read group's slot that does not depend on the chip: where the group is in
/// its cycle, what its last run did, and who is waiting for one. `ReadSlot<G>` adds the
/// buffer, the decoded Sample and the group's Request; the engine only ever needs these, so a
/// group can be walked without knowing its type.
template<typename Clock, EngineFeatures F = EngineFeatures{}>
struct ReadSlotBase {
    using TimePoint = typename Clock::time_point;

    // In order of alignment, the one-byte fields last: 48 bytes (was 64 with the fields in
    // topic order and the period as std::chrono::milliseconds).

    TimePoint due{};

    std::uint32_t seq{};
    std::uint32_t seen{};
    /// Statistics (EngineFeatures::stats): nothing without them.
    [[no_unique_address]] StatCount<F.stats> samples{};
    [[no_unique_address]] StatCount<F.stats> rejected{};

    /// The period the group runs at, from period<G>(ms); the description's Period until the
    /// application sets one (periodSet). 0 parks the group until request<G>() asks.
    [[no_unique_address]] IfFeature<F.runtimePeriod, Millis32, 10> period{};

    /// Requests made, from any context; the runs take them in order. A run is owed while
    /// `asked` is ahead of `started`, and the requests a run took on are answered with its
    /// outcome when it ends. The rest is the loop's.
    std::atomic<std::uint32_t> asked{0};
    std::uint32_t              started{};
    std::uint32_t              served{};

    /// Bytes the read step transfers, and how much of the buffer decode then sees (the read
    /// lands at an offset). Both are the whole buffer unless prepare() sized the run; the
    /// derived slot sets them, since only it knows how big its buffer is.
    std::uint8_t len{};
    std::uint8_t validBytes{};

    bool current{};   ///< a sample since the last bring-up (valid())
    /// `period` holds what period<G>(ms) set (EngineFeatures::runtimePeriod; nothing without).
    [[no_unique_address]] IfFeature<F.runtimePeriod, bool, 11> periodSet{};
    Answer                                                     last{Answer::pending};
    bool         serving{};   ///< the run on the wire took requests on
    std::uint8_t retries{};
};

/// The half of a write group's slot that does not depend on the chip: which items the chip
/// still owes, which it has been told, and when the group is next written as a whole.
/// `WriteSlot<G>` adds the values, the buffer and the read-back state.
template<typename Clock>
struct WriteSlotBase {
    using TimePoint = typename Clock::time_point;

    /// Every item's dirty bit, set from any context.
    std::atomic<std::uint32_t> dirty{0};
    /// The application has set this group's value, so `Initial` is no longer what the group
    /// holds -- even if nothing has reached the chip yet.
    bool owned{};
    /// Items whose value is what the chip holds or is owed, so that set() with that value
    /// again has nothing to send.
    std::uint32_t known{};
    /// Not a statistic only: a verify that has not been written yet waits (Device startVerify_).
    std::uint32_t writes{};
    TimePoint     due{};
};

/// What the engine needs of one write group, beside its slot.
struct WriteGroupInfo {
    /// Every item's bit set: what a periodic write re-dirties.
    std::uint32_t allItems{};
    /// The description's Period in milliseconds; 0 when the group is not written cyclically.
    std::uint32_t periodMs{};
    /// The group asks for its registers to be read back after they are written.
    bool verifies{};
};

/// What a read group's decode() made of the bytes, as the engine needs it: the sample itself
/// is the chip's business and the trampoline has already stored it.
enum class DecodeKind : std::uint8_t {
    ok,          ///< a new sample
    unchanged,   ///< a well-formed frame that says nothing new
    rejected,    ///< a bad CRC, an impossible value
    retry,       ///< not ready: run the sequence again after `retryMs`
};

struct DecodeResult {
    DecodeKind    kind{};
    std::uint32_t retryMs{};
};

/// What the engine needs of one read group of the chip, beside its slot: everything it would
/// otherwise have had to ask the group's type for.
struct ReadGroupInfo {
    /// The description's Period in milliseconds; 0 when the group is not cyclic.
    std::uint32_t periodMs{};
    /// The group's period depends on its last sample (`period(Sample const&)`), so the next
    /// deadline is measured from the answer rather than advanced on a grid.
    bool dynamicPeriod{};
};

/// What the engine needs of the chip, as constants and function pointers: one `static
/// constexpr` table per `Device` instantiation, in flash, shared by every object of that type.
/// Each pointer's target is a one-line trampoline in `Device` that casts the state back to
/// itself -- so the switch over the chip's groups is written once per chip instead of once per
/// call site per group.
template<typename Port, typename Clock>
struct Ops {
    using ReadSlotBaseT = ReadSlotBase<Clock, Port::Features>;

    using State     = EngineState<Port, Clock>;
    using TimePoint = typename Clock::time_point;

    /// What the chip is called, for the lines the engine logs about it.

    /// The script the current run walks, and the buffer its steps read into or write from.
    std::span<Step const> (*script)(State&){};
    std::span<std::byte> (*buffer)(State&){};

    /// A read group whose prepare() sized this run, and a counted read whose length an
    /// earlier step read.
    void (*sizeRead)(State&){};
    void (*countRead)(State&,
                      std::uint8_t){};
    /// The running read group's ready(): a check or stopUnless step asks it.
    bool (*ready)(State&){};

    /// The Identity registers against the data sheet, and setup() on a probe copy of the
    /// State: both say whether the script may go on.
    bool (*oracle)(State&){};
    bool (*identify)(State&){};

    /// The bring-up's buffer, cleared before a run: setup() and an identify step see only
    /// this run's bytes.
    void (*clearInit)(State&){};
    void (*finishInit)(State&,
                       TimePoint){};
    /// The wake-retry count, which every new transaction starts from.
    void (*wakeReset)(State&){};
    /// The write item on the wire is still owed; the read on the wire is owed again with its
    /// requests still pending; and no sample stands for a part that is being brought up again.
    void (*redirtyItem)(State&){};
    void (*unserve)(State&){};
    void (*forgetSamples)(State&){};

    /// setup() over what the bring-up read: false is "answers, but is not this chip". How
    /// long before such a part is tried again; 0 runs the description anyway.
    bool (*setupFinal)(State&){};
    /// Every group's first deadline after a bring-up, and the values the chip owes again
    /// because it came back at its defaults.
    void (*startupSchedule)(State&,
                            TimePoint){};

    /// A NAK the part gives while it wakes: true when the transaction was put on the wire
    /// again rather than counted.
    bool (*wakeRetry)(State&,
                      TimePoint){};

    /// The three waits of a power-on.
    std::uint32_t startupDelayMs{};
    std::uint32_t resetLowMs{};
    std::uint32_t resetSettleMs{};

    /// The chip's cyclic read groups: one entry each, and the half of their slots the engine
    /// can touch without knowing the group's type.
    std::span<ReadGroupInfo const> reads{};
    ReadSlotBaseT& (*readSlot)(State&,
                               std::uint8_t){};
    /// The period a group runs at now (what `period<G>(ms)` set, or the chip State's), and
    /// the one a group whose period follows its readings asks for after this sample.
    std::chrono::milliseconds (*periodNow)(State&,
                                           std::uint8_t){};
    std::chrono::milliseconds (*dynamicPeriod)(State&,
                                               std::uint8_t){};
    /// The group's prepare() and the buffer lengths this run works with.
    void (*prepareRead)(State&,
                        std::uint8_t){};
    /// The group's decode() over what came back: it stores the sample (and the timestamp,
    /// where the group keeps one) and says what the engine has to do about it.
    DecodeResult (*decode)(State&,
                           std::uint8_t){};

    /// The chip's write groups, the same way.
    std::span<WriteGroupInfo const> writes{};
    WriteSlotBase<Clock>& (*writeSlot)(State&,
                                       std::uint8_t){};
    /// One item encoded into the script the engine then walks.
    void (*encodeWrite)(State&,
                        std::uint8_t,
                        std::uint8_t){};
    /// The chip now holds the value: what the description makes of that (`applied`), and the
    /// read-back it owes.
    void (*afterWrite)(State&,
                       std::uint8_t,
                       std::uint8_t,
                       TimePoint){};
    /// A read-back that is due, if any: true when one went on the wire.
    bool (*startVerifyGroup)(State&,
                             std::uint8_t,
                             TimePoint){};
    /// What came back compared with what was written.
    void (*finishVerify)(State&,
                         TimePoint){};
    /// The earliest read-back due (max() if none); null when the chip verifies nothing.
    TimePoint (*verifyDue)(State&){};

    // The small fields last, together. Between the pointers each cost three bytes of padding,
    // and the six durations above were 8-byte `std::chrono::milliseconds` that aligned the
    // whole table to 8: 224 bytes x 57 devices was 12.8 KB of one firmware.
    std::string_view name{};
    /// 0, 1 or 2: how a register is addressed on this chip.
    std::uint8_t registerBytes{};
    /// True when the chip sends nothing to come up: link() then stays `starting` until its
    /// first transaction is acknowledged.
    bool initEmpty{};
};

/// What the engine needs of one device beside its chip, so that Ops and the chip's code are
/// shared by every device of that chip. A null hook is "none".
template<typename Port, typename Clock>
struct DeviceOps {
    using State = EngineState<Port, Clock>;

    /// The bus the part is on.
    bool (*submit)(typename Port::Request const&){};
    /// The gate in front of the part (a switch channel): false means "not yet, try next turn".
    bool (*claimGate)(State&){};
    void (*releaseGate)(State&){};
    // A field of a feature the port leaves out (EngineFeatures) is Absent: no bytes.
    static constexpr bool Switchable = Port::Features.switchable;

    /// Config::enabled(), called with `enabledArg` (one function can serve many devices).
    [[no_unique_address]] IfFeature<Switchable, bool (*)(void const*), 1> enabled{};
    [[no_unique_address]] IfFeature<Switchable, void const*, 2>           enabledArg{};
    /// Optional: a counter the application steps whenever what `enabled` answers may have
    /// changed - AFTER the change, from an interrupt on this core too. While it has not moved,
    /// the engine takes the last answer (`bridge_.disabled`) instead of calling `enabled` -
    /// which it otherwise does every loop turn, resting or not.
    [[no_unique_address]] IfFeature<Switchable, std::uint32_t const*, 3> enabledGeneration{};
    /// Behind a bridge (Bridge.hpp): off now, and its generation (steps on every activation).
    [[no_unique_address]] IfFeature<Switchable, bool (*)(State const&), 4> gateOffline{};
    [[no_unique_address]] IfFeature<Switchable, std::uint32_t (*)(State const&), 5>
      gateGeneration{};
    /// A chip with a reset line of its own.
    void (*resetHold)(){};
    void (*resetRelease)(){};

    [[no_unique_address]] IfFeature<Port::Features.presence, PresenceKnobs, 6> presence{};
    /// How long a failed write, read-back or bring-up waits before it is tried again.
    [[no_unique_address]] IfFeature<Port::Features.faultBackoff, std::uint32_t, 12>
      faultBackoffMs{};
    /// A request the bus never answered: the net under its "exactly one callback".
    [[no_unique_address]] IfFeature<Port::Features.inFlightNet, std::uint32_t, 7>
                  inFlightTimeoutMs{};
    std::uint32_t unidentifiedRetryMs{};
    std::uint8_t  address{};
    /// Failures in a row past the bring-up that configure the chip again from the start.
    std::uint8_t faultsBeforeReinit{};
    /// How often a decode or a check step may ask for a retry before the sample is rejected,
    /// and how often a read-back mismatch rewrites a register.
    std::uint8_t maxRetries{};
    /// WhileOff::unpowered: brought up from the start after an offline spell.
    [[no_unique_address]] IfFeature<Switchable, bool, 8> bridgeUnpowered{};
    [[no_unique_address]] IfFeature<Switchable, bool, 9> disabledUnpowered{};

    [[nodiscard]] constexpr bool switchable() const {
        if constexpr(Switchable) {
            return enabled != nullptr || gateOffline != nullptr;
        } else {
            return false;
        }
    }
};

/// The state machine. Every function here is one copy in the image for the whole firmware:
/// what it needs of the chip comes from `Ops`, what it needs of the run from `EngineState`.
template<typename Port, typename Clock>
struct Engine {
    using State     = EngineState<Port, Clock>;
    using OpsT      = Ops<Port, Clock>;
    using TimePoint = typename Clock::time_point;
    using PendingT  = Pending<Port, Clock>;

    static constexpr EngineFeatures Features = Port::Features;

    /// The bus bounds a request end to end (requestDeadlines: a `deadline` on the engine's clock): the engine's
    /// in-flight net is replaced by it outright. The engine stamps the deadline from the
    /// device's in-flight timeout, and the bus guarantees the callback by then - queued: timedOut without going
    /// out; on the wire: stopped - so the request is never forgotten while the bus still holds its buffer.
    static constexpr bool BusDeadline = [] {
        if constexpr(Features.inFlightNet && requires(typename Port::Request& r) { r.deadline; }) {
            return std::is_same_v<
              std::remove_cvref_t<decltype(std::declval<typename Port::Request&>().deadline)>,
              TimePoint>;
        } else {
            return false;
        }
    }();

    // The register goes out as the request's prefix and the payload straight from where it
    // lives (the group buffer, the running step): nothing is copied to put the two together.
    static_assert(
      requires(typename Port::Request r) {
          r.prefixBytes;
          r.prefix[1];
      } && Port::Request::MaxPrefix >= 2,
      "the port's Request needs prefix / prefixBytes with MaxPrefix >= 2 (the queued I2C "
      "drivers' I2CRequest, SPI's TransportRequest, FakeBusRequest): the engine sends the "
      "register as the prefix instead of copying it in front of the payload");

    /// `ops` is a device set's compile-time dispatch (SetOps) rather than a chip's table (Ops).
    template<typename O>
    static constexpr bool IsSetOps = requires { O::IsSetOps; };

    // The chip's constants, from either kind of `ops`: the run-time table (Ops, one per chip,
    // read through a pointer), or a device set's compile-time dispatch (SetOps, a static
    // function per constant that switches on the device's place in the set).
#define KVASIR_ENGINE_CONSTANT(name, Type)                            \
    template<typename O>                                              \
    [[nodiscard]] static Type name##_(O const& ops, State const& e) { \
        if constexpr(IsSetOps<O>) {                                   \
            return O::name(e);                                        \
        } else {                                                      \
            static_cast<void>(e);                                     \
            return ops.name;                                          \
        }                                                             \
    }
    KVASIR_ENGINE_CONSTANT(reads,
                           std::span<ReadGroupInfo const>)
    KVASIR_ENGINE_CONSTANT(writes,
                           std::span<WriteGroupInfo const>)
    KVASIR_ENGINE_CONSTANT(name,
                           std::string_view)
    KVASIR_ENGINE_CONSTANT(registerBytes,
                           std::uint8_t)
    KVASIR_ENGINE_CONSTANT(initEmpty,
                           bool)
    KVASIR_ENGINE_CONSTANT(startupDelayMs,
                           std::uint32_t)
    KVASIR_ENGINE_CONSTANT(resetLowMs,
                           std::uint32_t)
    KVASIR_ENGINE_CONSTANT(resetSettleMs,
                           std::uint32_t)
#undef KVASIR_ENGINE_CONSTANT

    /// When the chip's next read-back is due; never for a chip without one (a null hook in the
    /// table, or a set's case that has none).
    template<typename O>
    [[nodiscard]] static TimePoint verifyDue_(O const& ops,
                                              State&   e) {
        if constexpr(IsSetOps<O>) {
            return O::verifyDue(e);
        } else {
            return ops.verifyDue != nullptr ? ops.verifyDue(e) : TimePoint::max();
        }
    }

    using ReadSlotBaseT = ReadSlotBase<Clock, Port::Features>;

    /// When a wait of `delay` from `now` is over. `now` was read at some point inside a tick of
    /// the clock, so one tick more makes the wait at least `delay` long however late in its
    /// tick that was: on a millisecond clock a 1 ms delay would otherwise end on the very next
    /// tick, which can be microseconds away. A zero delay stays zero.
    /// A duration from one of the table's millisecond counts.
    [[nodiscard]] static constexpr std::chrono::milliseconds ms(std::uint32_t count) {
        return std::chrono::milliseconds{count};
    }

    /// A device set with no gated device, or none with a reset line (SetOps::Flags): the
    /// engine's path for it is left out, not only skipped.
    template<typename O>
    static consteval bool anyGate_() {
        if constexpr(requires { O::Flags; }) {
            return O::Flags.anyGate;
        } else {
            return true;   // the table: the gate is looked at at run time
        }
    }

    template<typename O>
    static consteval bool anyReset_() {
        if constexpr(requires { O::Flags; }) {
            return O::Flags.anyReset;
        } else {
            return true;
        }
    }

    template<typename O>
    static constexpr bool AnyGate = anyGate_<O>();
    template<typename O>
    static constexpr bool AnyReset = anyReset_<O>();

    template<typename O>
    [[nodiscard]] static bool claimGate(State& e,
                                        O const&) {
        if constexpr(AnyGate<O>) {
            return claimGate(e);
        } else {
            return true;
        }
    }

    template<typename O>
    static void releaseGate(State& e,
                            O const&) {
        if constexpr(AnyGate<O>) { releaseGate(e); }
    }

    template<typename O>
    static void endRun(State&   e,
                       O const& ops) {
        releaseGate(e, ops);
        e.running_ = Running::none;
    }

    template<typename O>
    static void wait(State&                    e,
                     O const&                  ops,
                     TimePoint                 now,
                     std::chrono::milliseconds delay,
                     bool                      thenNextStep) {
        e.waiting_    = true;
        e.afterDelay_ = thenNextStep;
        e.waitUntil_  = atLeast(now, delay);
        releaseGate(e, ops);
    }

    /// The gate in front of the part, if any.
    [[nodiscard]] static bool claimGate(State& e) {
        auto const claim = e.dev_->claimGate;
        return claim == nullptr || claim(e);
    }

    static void releaseGate(State& e) {
        if(auto const release = e.dev_->releaseGate) { release(e); }
    }

    [[nodiscard]] static TimePoint atLeast(TimePoint                 now,
                                           std::chrono::milliseconds delay) {
        if(delay == std::chrono::milliseconds::zero()) { return now; }
        return now + delay + typename Clock::duration{1};
    }

    /// The run is about to wait, so the gate is let go for it and claimed again before the
    /// next transaction: a part behind a switch does not hold its channel while it waits.
    static void wait(State&                    e,
                     TimePoint                 now,
                     std::chrono::milliseconds delay,
                     bool                      thenNextStep) {
        e.waiting_    = true;
        e.afterDelay_ = thenNextStep;
        e.waitUntil_  = atLeast(now, delay);
        releaseGate(e);
    }

    /// A run is over -- finished, failed, rejected, or abandoned by a reset. Every exit from
    /// a script goes through here, so a gate is never held a turn past the end of a run.
    static void endRun(State& e) {
        releaseGate(e);
        e.running_ = Running::none;
    }

    /// Start over from the reset: parked, probed, or the in-flight net. Whatever was on the
    /// wire is abandoned -- but not what it was for: a write in flight is owed again (its
    /// dirty bit went at startWrite_), or a Transient one -- an EEPROM page, a clock set --
    /// would be lost, since the bring-up does not replay it. And the gate is let go.
    template<typename O>
    static void reset(State&   e,
                      O const& ops) {
        abandonRun_(e, ops);
        e.phase_ = Phase::reset;
        e.up_    = false;
        e.acked_ = false;
        if constexpr(Features.faultBackoff) { e.initFailed_ = false; }
        ops.forgetSamples(e);
    }

    /// The bridge went, and the parts behind it keep their registers (WhileOff::disconnected):
    /// the run on the wire is abandoned as in reset() -- a write owed again, a read's requests
    /// still pending -- but the bring-up stands. The samples do not: valid() waits for one
    /// taken after the return.
    template<typename O>
    static void pause(State&   e,
                      O const& ops) {
        abandonRun_(e, ops);
        ops.forgetSamples(e);
    }

    /// The bring-up from its first step, or straight to its end where the chip sends nothing.
    template<typename O>
    static void beginInit(State&    e,
                          O const&  ops,
                          TimePoint now) {
        e.running_ = Running::init;
        e.step_    = 0;
        ops.clearInit(e);
        e.oracleFailed_ = false;
        if(initEmpty_(ops, e)) {
            // Nothing to send, so nothing has answered: link() stays `starting` until the
            // chip's first transaction -- a cyclic read, or whatever the application asks --
            // is acknowledged (progress_). Declared up at once, a missing chip without Init
            // would come "up" at every probe while its first read NAKs.
            ops.finishInit(e, now);
        } else {
            startStep(e, ops, now);
        }
    }

    /// The run of a read group is over: the requests it took on have their answer.
    static void answered(ReadSlotBaseT& s,
                         Answer         how) {
        if(!s.serving) { return; }
        s.serving = false;
        s.served  = s.started;
        s.last    = how;
    }

    /// The next deadline one period on, or one period from now when the last one is already
    /// past: a group that fell behind resumes at its period rather than bursting to catch up.
    static void advanceDue(TimePoint&                due,
                           TimePoint                 now,
                           std::chrono::milliseconds period) {
        if(now < due) { return; }
        due += period;
        if(due < now) { due = now + period; }
    }

    /// The next deadline of a periodic group, when this run was the periodic one; a run that
    /// was only requested leaves the period alone.
    ///
    /// A group with a period(sample) is measured from this completion instead, because what
    /// it is asking for is "so long after the last answer", and the period it asks for
    /// depends on that answer. A period of 0 parks the group until request<G>() asks for it.
    template<typename O>
    static void reschedule(State&       e,
                           O const&     ops,
                           std::uint8_t g,
                           TimePoint    now) {
        auto&       s  = ops.readSlot(e, g);
        auto const& gi = reads_(ops, e)[g];
        if(gi.dynamicPeriod) {
            std::chrono::milliseconds p{};
            bool                      set = false;
            if constexpr(Features.runtimePeriod) {
                if(s.periodSet) {
                    p   = s.period;
                    set = true;
                }
            }
            if(!set) { p = ops.dynamicPeriod(e, g); }
            s.due = p > std::chrono::milliseconds::zero() ? now + p : TimePoint::max();
        } else if(gi.periodMs != 0) {
            auto const p = ops.periodNow(e, g);
            if(p > std::chrono::milliseconds::zero()) {
                advanceDue(s.due, now, p);
            } else {
                s.due = TimePoint::max();   // parked by period<G>(0)
            }
        }
    }

    /// Run the read group again from its first step after `retryAfter`, or give up on this
    /// run once the retry limit is reached: the sample is rejected.
    template<typename O>
    static void again(State&                    e,
                      O const&                  ops,
                      TimePoint                 now,
                      std::chrono::milliseconds retryAfter) {
        auto& s = ops.readSlot(e, e.group_);
        if(s.retries < e.dev_->maxRetries) {
            ++s.retries;
            e.step_ = 0;
            wait(e, ops, now, retryAfter, false);
        } else {
            ++s.rejected;
            s.retries = 0;
            answered(s, Answer::rejected);
            reschedule(e, ops, e.group_, now);
            endRun(e, ops);
        }
    }

    /// The due read group with the earliest deadline, a requested one first.
    template<typename O>
    static void startRead(State&    e,
                          O const&  ops,
                          TimePoint now) {
        bool         found = false;
        std::uint8_t pick  = 0;
        TimePoint    best{};
        for(std::uint8_t g = 0; g < static_cast<std::uint8_t>(reads_(ops, e).size()); ++g) {
            auto&      s         = ops.readSlot(e, g);
            bool const requested = s.asked.load(std::memory_order_acquire) != s.started;
            bool const due       = requested || (reads_(ops, e)[g].periodMs != 0 && now >= s.due);
            if(!due) { continue; }
            TimePoint const when = requested ? TimePoint{} : s.due;
            if(!found || when < best) {
                found = true;
                best  = when;
                pick  = g;
            }
        }
        if(!found) { return; }
        ops.prepareRead(e, pick);
        auto& s   = ops.readSlot(e, pick);
        s.retries = 0;
        // Taken at the start: a request() made while this run is on the wire is answered by
        // another run.
        auto const asked = s.asked.load(std::memory_order_acquire);
        s.serving        = asked != s.started;
        s.started        = asked;
        e.running_       = Running::read;
        e.group_         = pick;
        e.step_          = 0;
        startStep(e, ops, now);
    }

    /// The end of a script: what it was for decides what happens now.
    template<typename O>
    static void finish(State&    e,
                       O const&  ops,
                       TimePoint now) {
        switch(e.running_) {
        case Running::init:   finishInit(e, ops, now); break;
        case Running::read:   finishRead(e, ops, now); break;
        case Running::write:  finishWrite(e, ops, now); break;
        case Running::verify: ops.finishVerify(e, now); break;
        case Running::none:   break;
        }
    }

    /// A cyclic read is over: what its decode made of the bytes, then the group's next deadline.
    template<typename O>
    static void finishRead(State&    e,
                           O const&  ops,
                           TimePoint now) {
        auto const r = ops.decode(e, e.group_);
        if(r.kind == DecodeKind::retry) {
            // the whole sequence again, or rejected
            again(e, ops, now, ms(r.retryMs));
            return;
        }
        auto& s = ops.readSlot(e, e.group_);
        switch(r.kind) {
        case DecodeKind::ok:
            ++s.seq;
            ++s.samples;
            s.current = true;
            answered(s, Answer::ok);
            break;
        case DecodeKind::unchanged: answered(s, Answer::unchanged); break;
        case DecodeKind::rejected:
            ++s.rejected;
            answered(s, Answer::rejected);
            break;
        case DecodeKind::retry: break;   // handled above
        }
        s.retries = 0;
        reschedule(e, ops, e.group_, now);
        endRun(e, ops);
    }

    /// A write is on the chip: the description hears about it, and the read-back is owed.
    template<typename O>
    static void finishWrite(State&    e,
                            O const&  ops,
                            TimePoint now) {
        ++ops.writeSlot(e, e.group_).writes;
        ops.afterWrite(e, e.group_, e.item_, now);
        endRun(e, ops);
    }

    /// The first dirty item of the first dirty write group, encoded and on the wire.
    template<typename O>
    static bool startWrite(State&    e,
                           O const&  ops,
                           TimePoint now) {
        for(std::uint8_t w = 0; w < static_cast<std::uint8_t>(writes_(ops, e).size()); ++w) {
            auto&       s  = ops.writeSlot(e, w);
            auto const& gi = writes_(ops, e)[w];
            if(gi.periodMs != 0 && now >= s.due) {
                // Only `known` items: one never set keeps the chip's power-on value (an
                // MCP4725's EEPROM level).
                advanceDue(s.due, now, ms(gi.periodMs));
                if(auto const told = gi.allItems & s.known; told != 0) {
                    s.dirty.fetch_or(told, std::memory_order_relaxed);
                }
            }
            auto const d = s.dirty.load(std::memory_order_acquire);
            if(d == 0) { continue; }
            auto const item = static_cast<std::uint8_t>(std::countr_zero(d));
            // set() during the write re-dirties
            s.dirty.fetch_and(~(1U << item), std::memory_order_relaxed);
            ops.encodeWrite(e, w, item);
            e.running_ = Running::write;
            e.group_   = w;
            e.item_    = item;
            e.step_    = 0;
            // Through startStep like the read and init paths, so a script of more than one
            // transaction is walked by the same code.
            startStep(e, ops, now);
            return true;
        }
        return false;
    }

    /// Read one written register back, where a group asks for it and one is due.
    template<typename O>
    static bool startVerify(State&    e,
                            O const&  ops,
                            TimePoint now) {
        for(std::uint8_t w = 0; w < static_cast<std::uint8_t>(writes_(ops, e).size()); ++w) {
            if(!writes_(ops, e)[w].verifies) { continue; }
            if(ops.startVerifyGroup(e, w, now)) { return true; }
        }
        return false;
    }

    /// False while the engine has to keep still: the part is behind a bridge that is not
    /// active, or its Config disabled it. A transaction on the wire when the part goes offline
    /// is waited for (the held gate keeps a switched bridge from cutting it) and its result
    /// dropped. Any power loss in an offline spell, a bounce seen only by the bridge's
    /// generation included, brings the part up from the start once.
    template<typename O>
    static bool bridgeTurn(State&    e,
                           O const&  ops,
                           TimePoint now) {
        auto const& d        = *e.dev_;
        bool const  bridged  = d.gateOffline != nullptr;
        bool const  gateOff  = bridged && d.gateOffline(e);
        auto const  seen     = e.enabledGenerationNow();
        bool const  disabled = e.disabledNow(seen);
        bool const  bounced  = bridged && e.bridge_.generation != d.gateGeneration(e);
        if((gateOff || disabled) && e.inFlight_) {
            if(e.pending_.take() == PendingT::Outcome::running && e.withinInFlightNet(now, d)) {
                return false;   // not a word to the part until it is answered
            }
            e.inFlight_ = false;
            e.pending_.clear();
        }

        bool const wasOffline = e.bridge_.offline();
        bool       powerCut   = false;
        if(bridged) {
            if(gateOff != e.bridge_.gateOff) {
                e.bridge_.gateOff = gateOff;
                logBridge(name_(ops, e), d.address, !gateOff);
            } else if(bounced && !gateOff) {   // off and on again, both unseen
                logBridge(name_(ops, e), d.address, false);
                logBridge(name_(ops, e), d.address, true);
            }
            powerCut             = (gateOff || bounced) && d.bridgeUnpowered;
            e.bridge_.generation = d.gateGeneration(e);
        }
        if(disabled != e.bridge_.disabled) {
            e.bridge_.disabled = disabled;
            logEnabled(name_(ops, e), d.address, !disabled);
            powerCut = powerCut || (disabled && d.disabledUnpowered);
        }
        e.noteEnabledSeen(seen);   // only here: the early return above leaves bridge_ as it was

        bool const offline = gateOff || disabled;
        if(!wasOffline && !offline && !bounced) { return true; }   // the common case

        if(powerCut && !e.bridge_.lostPower) {
            reset(e, ops);
            e.bridge_.lostPower = true;
        } else if(!wasOffline) {
            pause(e, ops);   // the registers stay; a reset later in this spell still comes
        }
        if(offline) { return false; }
        e.bridge_.lostPower = false;
        e.presence_.restart();
        return true;
    }

    /// Once per loop turn: a full turn, or nothing while the device rests (EngineState).
    template<typename O>
    static void handler(State&    e,
                        O const&  ops,
                        TimePoint now) {
        if constexpr(Features.rest) {
            if(e.resting_ && now < e.quietUntil_ && !e.doorbell_.load(std::memory_order_acquire)
               && settled(e))
            {
                return;
            }
            // Cleared before the turn: a request made during it rings again.
            e.doorbell_.exchange(false, std::memory_order_acq_rel);
        }
        ++e.turns_;
        turnOnce(e, ops, now);
        if constexpr(Features.rest) { rest(e, ops); }
    }

    /// switchSettled() where the port has switchable devices; nothing to settle elsewhere.
    [[nodiscard]] static bool settled(State& e) {
        if constexpr(Features.switchable) {
            return switchSettled(e);
        } else {
            return true;
        }
    }

    /// The bridge and Config::enabled() still say what the last full turn saw (`bridge_`).
    /// An answer that moved generation but not value is noted, so the next turn does not ask.
    [[nodiscard]] static bool switchSettled(State& e) {
        auto const& d = *e.dev_;
        if(!d.switchable()) { return true; }
        auto const seen     = e.enabledGenerationNow();
        bool const disabled = e.disabledNow(seen);
        if(disabled != e.bridge_.disabled) { return false; }
        e.noteEnabledSeen(seen);
        if(d.gateOffline != nullptr) {
            return d.gateOffline(e) == e.bridge_.gateOff
                && d.gateGeneration(e) == e.bridge_.generation;
        }
        return true;
    }

    /// After a full turn: rest until the time point the device waits for, if it waits for one.
    /// Mirrors turn()/schedule(); anything else runs the next turn.
    template<typename O>
    static void rest(State&   e,
                     O const& ops) {
        e.resting_ = false;
        if(e.inFlight_) { return; }
        auto const quiet = [&](TimePoint until) {
            e.quietUntil_ = until;
            e.resting_    = true;
        };
        if constexpr(Features.switchable) {
            if(e.dev_->switchable() && e.bridge_.offline()) {
                quiet(TimePoint::max());   // until switchSettled() says otherwise
                return;
            }
        }
        TimePoint probe{};
        switch(e.presence_.rest(e.dev_->presence, probe)) {
        case Presence<Clock>::Rest::busy:   return;
        case Presence<Clock>::Rest::parked: quiet(probe); return;
        case Presence<Clock>::Rest::talk:   break;
        }
        switch(e.phase_) {
        case Phase::reset:     return;
        case Phase::resetHeld:
        case Phase::settle:
            // turn() moves on once now > waitUntil_: one tick after it
            quiet(e.waitUntil_ + typename Clock::duration{1});
            return;
        case Phase::run: break;
        }
        if(e.running_ != Running::none) {
            if(e.waiting_) { quiet(e.waitUntil_); }
            return;   // not waiting and not in flight: a submit to try again
        }
        if(!e.up_) { return; }   // the bring-up begins next turn

        // Up and idle: until the earliest group is due, and not before the hold after a fault.
        TimePoint due = TimePoint::max();
        for(std::uint8_t g = 0; g < static_cast<std::uint8_t>(reads_(ops, e).size()); ++g) {
            auto const& s = ops.readSlot(e, g);
            if(s.asked.load(std::memory_order_acquire) != s.started) {
                due = TimePoint::min();
            } else if(reads_(ops, e)[g].periodMs != 0 && s.due < due) {
                due = s.due;
            }
        }
        for(std::uint8_t w = 0; w < static_cast<std::uint8_t>(writes_(ops, e).size()); ++w) {
            auto const& s = ops.writeSlot(e, w);
            if(s.dirty.load(std::memory_order_acquire) != 0) {
                due = TimePoint::min();
            } else if(writes_(ops, e)[w].periodMs != 0 && s.due < due) {
                due = s.due;
            }
        }
        if(auto const v = verifyDue_(ops, e); v < due) { due = v; }
        if constexpr(Features.faultBackoff) {
            if(due < e.holdUntil_) { due = e.holdUntil_; }
        }
        quiet(due);
    }

    /// A full turn.
    template<typename O>
    static void turnOnce(State&    e,
                         O const&  ops,
                         TimePoint now) {
        if constexpr(Features.switchable) {
            if(e.dev_->switchable() && !bridgeTurn(e, ops, now)) { return; }
        }
        switch(e.presence_.turn(now, e.dev_->address, e.dev_->presence)) {
        case PresenceTurn::wait:   return;   // parked, no probe due: nothing runs
        case PresenceTurn::park:             // just parked: start over, then wait
            reset(e, ops);
            return;
        case PresenceTurn::probe:   // the bring-up is the probe
            reset(e, ops);
            break;
        case PresenceTurn::talk: break;
        }
        if constexpr(Features.inFlightNet && !BusDeadline) {
            if(e.inFlight_ && !e.withinInFlightNet(now, *e.dev_)) {
                logNoBusAnswer(name_(ops, e), e.dev_->address, ms(e.dev_->inFlightTimeoutMs));
                e.inFlight_ = false;
                e.pending_.clear();
                ++e.errors_;
                e.presence_.fault();
                fail(e, ops, now);
            }
        }
        turn(e, ops, now);
    }

    /// Where the device is with its part decides what the turn does.
    template<typename O>
    static void turn(State&    e,
                     O const&  ops,
                     TimePoint now) {
        switch(e.phase_) {
        case Phase::reset:
            {
                // A bring-up that failed is not tried again on the very next turn: a chip
                // with no startupDelay on a faulting bus would otherwise be asked every turn.
                auto backoff = std::chrono::milliseconds{0};
                if constexpr(Features.faultBackoff) {
                    if(e.initFailed_) { backoff = ms(e.dev_->faultBackoffMs); }
                    e.initFailed_ = false;
                }
                if(auto const hold = AnyReset<O> ? e.dev_->resetHold : nullptr) {
                    hold();
                    e.waitUntil_ = now + ms(resetLowMs_(ops, e)) + backoff;
                    e.phase_     = Phase::resetHeld;
                } else {
                    e.waitUntil_ = now + ms(startupDelayMs_(ops, e)) + backoff;
                    e.phase_     = Phase::settle;
                }
            }
            break;
        case Phase::resetHeld:
            if(now > e.waitUntil_) {
                if constexpr(AnyReset<O>) { e.dev_->resetRelease(); }
                e.waitUntil_ = now + ms(resetSettleMs_(ops, e)) + ms(startupDelayMs_(ops, e));
                e.phase_     = Phase::settle;
            }
            break;
        case Phase::settle:
            if(now > e.waitUntil_) {
                e.phase_ = Phase::run;
                beginInit(e, ops, now);
            }
            break;
        case Phase::run:
            if(e.running_ != Running::none) {
                progress(e, ops, now);
            } else if(e.up_) {
                schedule(e, ops, now);
            } else {
                beginInit(e, ops, now);
            }
            break;
        }
    }

    /// Nothing is running: what starts next. A verify goes between the writes and the reads --
    /// it only runs when nothing is dirty, so it cannot delay a write, and at a bounded
    /// interval it does not meaningfully slow the reads.
    template<typename O>
    static void schedule(State&    e,
                         O const&  ops,
                         TimePoint now) {
        if constexpr(Features.faultBackoff) {
            if(now < e.holdUntil_) { return; }
        }
        if(startWrite(e, ops, now)) { return; }
        if(startVerify(e, ops, now)) { return; }
        startRead(e, ops, now);
    }

    /// A run is under way: the wait it is in, the transaction it owes, or what the bus said.
    template<typename O>
    static void progress(State&    e,
                         O const&  ops,
                         TimePoint now) {
        if(e.waiting_) {
            if(now < e.waitUntil_) { return; }
            e.waiting_ = false;
            if(e.afterDelay_) {
                e.afterDelay_ = false;
                nextStep(e, ops, now);   // the step's delay is over
            } else {
                startStep(e, ops, now);   // a retry: step_ was put back to the start
            }
            return;
        }
        if(!e.inFlight_) {
            e.inFlight_ = submit(e, ops, ops.buffer(e));   // refused last turn
            return;
        }
        switch(e.pending_.take()) {
        case PendingT::Outcome::running: return;
        case PendingT::Outcome::ok:
            e.inFlight_            = false;
            e.consecutiveFailures_ = 0;
            ops.wakeReset(e);
            e.presence_.ack(now, e.dev_->address);
            if(!e.acked_) {
                e.acked_ = true;
                // no Init: the first ACK is the chip coming up
                if(e.up_) { logUp(name_(ops, e), e.dev_->address, e.identified_); }
            }
            afterTransaction(e, ops, now);
            return;
        case PendingT::Outcome::notAcknowledged:
            // The device did not answer: what parks it, in the end -- unless the step is a
            // write the part is known not to acknowledge (Step mayNak), or it is a part that
            // NAKs while it wakes and has retries left for this transaction.
            e.inFlight_ = false;
            if(e.current_.mayNak) {
                afterTransaction(e, ops, now);
                return;
            }
            if(ops.wakeRetry(e, now)) { return; }
            ++e.errors_;
            e.presence_.nak();
            fail(e, ops, now);
            return;
        case PendingT::Outcome::failed:
            // A bus fault: says nothing about the device. After a write the part is known not
            // to acknowledge it is no news either: a part that resets in the middle of the
            // byte lets go of the bus wherever it happens to be, and the controller sees that
            // as often as a lost arbitration as a NAK (LTR390 software reset:
            // thirty of thirty). The script goes on as after
            // the NAK; a bus that is really at fault fails the step after it.
            e.inFlight_ = false;
            if(e.current_.mayNak) {
                afterTransaction(e, ops, now);
                return;
            }
            ++e.errors_;
            e.presence_.fault();
            fail(e, ops, now);
            return;
        }
    }

    /// The bring-up is over: what the chip said about itself decides whether it is the chip
    /// the description is for, and then every group starts from now.
    template<typename O>
    static void finishInit(State&    e,
                           O const&  ops,
                           TimePoint now) {
        endRun(e, ops);
        e.identified_ = ops.setupFinal(e);
        // The data sheet's identity has the last word: a part that failed it is not the chip,
        // whatever setup() makes of a buffer the script never got to fill.
        if(e.oracleFailed_) { e.identified_ = false; }
        if(!e.identified_ && ms(e.dev_->unidentifiedRetryMs) > std::chrono::milliseconds::zero()) {
            // Not the chip this description is for: nothing else goes to it. The bring-up
            // runs again after unidentifiedRetry -- a part that was still booting, or that
            // gets its id right after a reset, comes through then. link() stays `starting`.
            if(!e.unidentifiedLogged_) {
                e.unidentifiedLogged_ = true;
                logUnidentified(name_(ops, e), e.dev_->address, ms(e.dev_->unidentifiedRetryMs));
            }
            ++e.unidentified_;
            e.waiting_   = false;
            e.waitUntil_ = now + ms(e.dev_->unidentifiedRetryMs);
            e.phase_     = Phase::settle;
            return;
        }
        e.up_ = true;
        ++e.bringUps_;
        if(e.acked_) { logUp(name_(ops, e), e.dev_->address, e.identified_); }
        ops.startupSchedule(e, now);
    }

    /// Nothing but the running script for `faultBackoff` (EngineFeatures::faultBackoff).
    static void backOff(State&    e,
                        TimePoint now) {
        if constexpr(Features.faultBackoff) { e.holdUntil_ = now + ms(e.dev_->faultBackoffMs); }
    }

    /// A transaction failed: what that means for the run it was part of, and for the device.
    template<typename O>
    static void fail(State&    e,
                     O const&  ops,
                     TimePoint now) {
        ops.wakeReset(e);   // the next transaction starts with every wake retry
        if(e.consecutiveFailures_ != std::numeric_limits<std::uint8_t>::max()) {
            ++e.consecutiveFailures_;
        }
        switch(e.running_) {
        case Running::init:
            // Start over from the reset, after faultBackoff; Presence parks the device after
            // enough NAKs and the first Init step becomes its probe.
            e.phase_ = Phase::reset;
            if constexpr(Features.faultBackoff) { e.initFailed_ = true; }
            break;
        case Running::read:
            {
                // The run is over and says nothing about the part: its requests are answered
                // and the group waits for its next period.
                auto& s   = ops.readSlot(e, e.group_);
                s.retries = 0;
                answered(s, Answer::failed);
                reschedule(e, ops, e.group_, now);
            }
            break;
        case Running::write:
            ops.redirtyItem(e);
            backOff(e, now);
            break;
        case Running::verify:
            // a bus fault says nothing about the register: still owed, looked at again
            // shortly. Only a mismatch counts against the rewrite budget.
            backOff(e, now);
            break;
        case Running::none: break;
        }
        endRun(e, ops);
        // Past the bring-up and still failing: the chip is configured again from the
        // start. After the per-phase handling above, so a write is owed before the
        // bring-up that would otherwise not replay a Transient one.
        if(e.phase_ == Phase::run && e.consecutiveFailures_ >= e.dev_->faultsBeforeReinit) {
            e.consecutiveFailures_ = 0;
            e.waiting_             = false;
            e.up_                  = false;
            e.acked_               = false;
            e.phase_               = Phase::reset;
            ops.forgetSamples(e);
        }
    }

    /// Puts the running step (`e.current_`) on the bus. False when it could not go (the gate
    /// said not yet, the device is parked with no probe due, the bus queue is full): the step
    /// is tried again next turn. The register goes in the request's prefix; the payload is a
    /// view of the group buffer or of `e.current_.bytes`, neither of which changes while the
    /// step is in flight.
    template<typename O>
    static bool submit(State&               e,
                       O const&             ops,
                       std::span<std::byte> buf) {
        Step const& s = e.current_;
        // Parked: nothing goes out but the one probe per interval, and that one before the
        // gate is asked, so a parked device never holds a switch.
        if(!e.presence_.mayTalk()) { return false; }
        // The gate is asked every turn until it says yes, and held from here until the run
        // waits or ends. What a switch must not do is move in the middle of a transaction --
        // a register read is a write and a read with a repeated start between them -- and that
        // is one request on the queue: a switch write another channel asks for once this
        // device lets go is queued behind it, so it cannot land inside it. "Not yet" leaves
        // the request unsubmitted and the engine retries next turn, which is the path a full
        // bus queue already takes.
        if(!claimGate(e, ops)) { return false; }
        e.pending_.clear();
        typename Port::Request req{};
        req.address  = e.dev_->address;
        req.callback = e.pending_.callback();
        if(s.hasRegister) {
            // big-endian from the step, or as the buffer holds it (readIndirect); SPI's
            // Transport turns the register byte into the chip's command
            auto const rb = registerBytes_(ops, e);
            for(std::size_t i = 0; i < rb; ++i) {
                req.prefix[i] = s.regFromBuffer
                                ? buf[s.reg + i]
                                : static_cast<std::byte>(s.reg >> (8U * (rb - 1U - i)));
            }
            req.prefixBytes = static_cast<std::uint8_t>(rb);
        }
        if(s.kind == Step::Kind::write) {
            if(s.fromBuffer) {
                req.sendData = std::span<std::byte const>{buf}.subspan(s.offset, s.count);
            } else {
                req.sendData = std::as_bytes(std::span{s.bytes}).first(s.count);
            }
        } else {
            req.receiveData = buf.subspan(s.offset, s.count);
        }
        // Stamped before the submit: a one-byte write can complete in the interrupt before
        // submit() returns, and the net must not measure from after that.
        e.stampSubmit();
        if constexpr(BusDeadline) {
            req.deadline = Clock::now() + std::chrono::milliseconds{e.dev_->inFlightTimeoutMs};
        }
        return e.dev_->submit(req);
    }

    /// The step `step_` of the running script: a wait, a question the buffer answers, or a
    /// transaction on the wire.
    template<typename O>
    static void startStep(State&    e,
                          O const&  ops,
                          TimePoint now) {
        auto const steps = ops.script(e);
        e.current_       = steps[e.step_];
        if(e.running_ == Running::read && e.current_.kind == Step::Kind::read) { ops.sizeRead(e); }
        if(e.current_.kind == Step::Kind::read && e.current_.counted) {
            // The count an earlier step of this run read; 0 is nothing to read, so no
            // transaction and on to the next step.
            auto const n = e.current_.countIn(ops.buffer(e));
            if(e.running_ == Running::read) { ops.countRead(e, n); }
            if(n == 0) {
                nextStep(e, ops, now);
                return;
            }
            e.current_.count = n;
        }
        switch(e.current_.kind) {
        case Step::Kind::wait: wait(e, ops, now, e.current_.delay, true); return;
        case Step::Kind::oracle:
            if(ops.oracle(e)) {
                nextStep(e, ops, now);
            } else {
                finish(e, ops, now);
            }
            return;
        case Step::Kind::identify:
            // The same setup() the end of the bring-up runs, on a State of its own: false
            // ends the script, and finishInit_ reaches the same verdict.
            if(ops.identify(e)) {
                nextStep(e, ops, now);
            } else {
                finish(e, ops, now);
            }
            return;
        case Step::Kind::stopUnless:
            if(ops.ready(e)) {
                nextStep(e, ops, now);
            } else {
                finish(e, ops, now);
            }
            return;
        case Step::Kind::check:
            if(ops.ready(e)) {
                nextStep(e, ops, now);
            } else {
                again(e, ops, now, e.current_.delay);
            }
            return;
        default: break;
        }
        e.inFlight_ = submit(e, ops, ops.buffer(e));
    }

    /// The step's delay, then the next step.
    template<typename O>
    static void afterTransaction(State&    e,
                                 O const&  ops,
                                 TimePoint now) {
        if(e.current_.delay != std::chrono::milliseconds::zero()) {
            wait(e, ops, now, e.current_.delay, true);
        } else {
            nextStep(e, ops, now);
        }
    }

private:
    /// What reset() and pause() have in common: the run on the wire is let go of, and what it
    /// was for is owed again.
    template<typename O>
    static void abandonRun_(State&   e,
                            O const& ops) {
        if(e.running_ == Running::write) { ops.redirtyItem(e); }
        if(e.running_ == Running::read) { ops.unserve(e); }
        endRun(e, ops);
        e.waiting_             = false;
        e.inFlight_            = false;
        e.consecutiveFailures_ = 0;
        ops.wakeReset(e);
        e.pending_.clear();
    }

public:
    template<typename O>
    static void nextStep(State&    e,
                         O const&  ops,
                         TimePoint now) {
        ++e.step_;
        if(e.step_ < ops.script(e).size()) {
            startStep(e, ops, now);
        } else {
            finish(e, ops, now);
        }
    }
};

}   // namespace Kvasir::I2C::detail
