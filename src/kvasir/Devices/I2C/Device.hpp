#pragma once

#include "../Link.hpp"
#include "../Log.hpp"
#include "Bytes.hpp"
#include "Concepts.hpp"
#include "Engine.hpp"
#include "EngineLog.hpp"
#include "Pending.hpp"
#include "Presence.hpp"
#include "RegisterCheck.hpp"
#include "Step.hpp"

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

/// One driver for any I2C device that is a description: what it is (address, register
/// width), how it is brought up (a Step script, an optional setup() over what the script
/// read), what is read from it cyclically (ReadGroups: a Step script at a period, decoded
/// into a Sample) and what is written to it when the application changes it (WriteGroups:
/// a Value encoded into one transaction -- or a fixed sequence of them -- per item). The
/// engine below owns the state machine: the wait after each step, the pending handshake, the
/// register prefix, the period, and the retry and failure policy -- including whether the
/// device is there at all (Presence.hpp: parked after three NAKs, probed at 1 s .. 30 s)
/// and the one safety net under it, a request the bus never answered. One place decides
/// what a failed transaction means, and nothing above or below it decides differently.
///
///     using Sensor = Kvasir::I2C::Device<I2c1, Clock, Chips::Sht3x>;
///     ...
///     Sensor sht{};                                       // a local in main(), or a member
///     sht.handler();                                      // once per loop turn
///     if(sht.valid()) { auto const& s = sht.latest(); }   // the one read group's Sample
///
/// A device keeps all of its state in members and the bus callback captures `this` when a
/// transfer is submitted, so nothing has to find it by name: it belongs wherever the
/// application keeps its objects. Static storage is what an interrupt that must reach the
/// device or a second core asks for -- not the engine.
///
/// What a write group may add beyond Value/Bytes/encode:
///   Items         -- one dirty bit and one Value each: set<W>(i, v)
///   Initial       -- written after every bring-up unless the application set one first
///   Period        -- rewritten cyclically
///   Transient     -- a one-shot command, not a state: NOT re-sent after a reset. Everything
///                    else the application has written is, because the chip came back at its
///                    defaults (an EEPROM page write and setting a clock are the exceptions).
///   encode() may return std::array<Step, N> instead of a Step: the item is N transactions
///                    run back to back, each honouring its own delay, and only when the
///                    last one is done does the item count as written. A failure anywhere
///                    re-dirties the item and the sequence starts again from its first step.
///                    Not compatible with the read-back verify below. (An SSD1306 page: the
///                    window command, then the RAM bytes behind it.)
///   VerifyDelay   -- read the register back this long after writing it
///   VerifyInterval-- and look again this often
///   verify(Kvasir::I2C::Bytes written, Kvasir::I2C::Bytes readBack) -- what counts as a
///                    match, when bytewise equality does not (a self-clearing bit, a status
///                    bit sharing the byte). Spell the type out: the group's own Bytes member
///                    shadows it. A mismatch rewrites the item, at most MaxRetries times.
///
/// What a cyclic read group may add beyond Period/Steps/Sample/decode:
///   prepare()     -- may return a std::size_t instead of void: how many bytes this run
///                    reads, so one description serves a four-byte and a sixty-four-byte
///                    read. Such a group has exactly one read step.
///   Step::readCounted / receiveCounted -- a read whose length an earlier read of the same
///                    run put in the buffer (one or two bytes, big-endian), clamped to the
///                    step's most: a GNSS receiver's "bytes available", then that many of
///                    its stream. A count of 0 skips the transaction; decode sees the buffer
///                    up to the counted read's offset + count.
///   period(Sample const&) -- the wait after this answer, instead of the constant Period
///                    (which stays as the compile-time "this group is cyclic" and the first
///                    deadline). A touch controller polled every 50 ms while a finger is
///                    down and every 100 ms when the glass is idle. 0 parks the group until
///                    request<G>() asks for it.
///   decode(Bytes, Sample const& previous) -- for a chip whose frames only mean something
///                    next to the last one. With Outcome::unchanged(), which keeps the
///                    sample, does not step seq and is counted by unchanged<G>(), a
///                    controller's "no new data" bit and a repeated "no finger" stop waking
///                    a reader that watches seq. A group takes the chip State or the
///                    previous Sample, not both.
///   Timestamped   -- keep stamp<G>(), the Clock::time_point the chip's answer arrived on the
///                    bus, from the interrupt that completed it rather than the loop turn
///                    that noticed. A group with the decode above keeps it anyway, together
///                    with unchanged<G>(); every other read group carries neither and pays
///                    nothing for them.
///
/// What a description may add for a part that NAKs while it wakes:
///   WakeRetries   -- `static constexpr std::uint8_t WakeRetries = N;`: a NAKed transaction is
///                    put on the wire again up to N times, WakeRetryDelay apart (default 1 ms,
///                    the gate let go meanwhile), before it counts as a NAK -- towards
///                    errors(), presence and the failure policy. The retries count nowhere
///                    but wakeRetries(). A CY8CMBR3xxx NAKs until it has woken from its
///                    sleep; a description without WakeRetries pays nothing for it.
///
/// The chip descriptions are in chips/.
namespace Kvasir::I2C {

using detail::Answer;

template<typename... Ts>
struct List {
    static constexpr std::size_t size = sizeof...(Ts);
};

/// What a decode made of the bytes: a sample, nothing new, a rejection (a bad CRC, an
/// impossible value: counted, the previous sample kept), or "not ready yet, run the
/// sequence again after so many milliseconds" (a busy bit; see also Step::check for a flag
/// polled mid-sequence).
template<typename S>
struct Outcome {
    enum class Kind : std::uint8_t { ok, reject, retry, unchanged };

    Kind                      kind{};
    S                         value{};
    std::chrono::milliseconds retryAfter{};

    static constexpr Outcome ok(S value) { return {.kind = Kind::ok, .value = value}; }

    static constexpr Outcome reject() { return {.kind = Kind::reject}; }

    static constexpr Outcome retry(std::chrono::milliseconds after) {
        return {.kind = Kind::retry, .retryAfter = after};
    }

    /// A well-formed frame that says nothing new (a controller's "no new data" bit, or a
    /// reading the application has already been told about): the sample stands, seq does
    /// not step, and it is counted by unchanged<G>() rather than as a rejection. A reader
    /// that watches seq is not woken by an idle chip polled ten times a second.
    static constexpr Outcome unchanged() { return {.kind = Kind::unchanged}; }
};

struct Empty {};

/// A config that only sets the address: `Device<I2c, Clock, Chips::Bme280, At<0x77>>`.
template<std::uint8_t A>
struct At {
    static constexpr std::uint8_t Address = A;
};

/// No config: the chip's default address.
struct DefaultConfig {};

/// No timing: a description's own defaults. A description that takes a `Timing` type reads the
/// std::chrono members it names (`Period`, `StartupDelay`, ...) from it and falls back to its
/// own figure for any that is missing -- a duration is not a structural type, so it cannot be
/// a template parameter itself:
///     struct Fast { static constexpr std::chrono::milliseconds Period{20}; };
///     using Meter = Chips::Ina219<Units::microOhm(100000), Units::microAmp(100), Fast>;
struct DefaultTiming {};

/// The engine's retry and failure policy. A Device's Config may redeclare any of them:
///     struct Config { static constexpr std::uint8_t MaxRetries = 3; };
struct EngineDefaults {
    /// How often a decode or a check step may ask for a retry before the sample is rejected,
    /// and how often a read-back mismatch rewrites a register before it is left alone.
    static constexpr std::uint8_t MaxRetries = 8;
    /// How long a write, a read-back or a bring-up waits after a failed transaction before it
    /// is tried again. What was owed stays owed; this only keeps a broken bus from being
    /// hammered every loop turn.
    static constexpr auto FaultBackoff = std::chrono::milliseconds{5};
    /// Failed transactions in a row, past the bring-up, after which the chip is configured
    /// again from the start: a part that keeps faulting may have lost its registers.
    static constexpr std::uint8_t FaultsBeforeReinit = 5;
    /// A request whose callback never came. The bus guarantees exactly one callback per
    /// submitted request, so this never fires on a correct bus; it is the net under that
    /// guarantee, and when it does fire the request is treated as failed.
    static constexpr auto InFlightTimeout = std::chrono::seconds{2};
    /// setup() said "not the chip this description is for": how long to wait before the
    /// bring-up is tried again. Nothing else is sent to the part meanwhile -- a config write
    /// meant for one chip is not what an unknown part at the same address should get. 0 runs
    /// the description anyway, with identified() false, for a board that must talk to a part
    /// whose id it does not know.
    static constexpr auto UnidentifiedRetry = std::chrono::seconds{1};
};

/// No gate: the device may talk whenever the engine wants it to, which is every device that
/// is not behind an I2C switch. See Mux.hpp for the one that is.
///
/// A gate is an object the Device holds, not a set of static functions, so a gate that needs
/// to know *which* switch it is on can keep a pointer to it and the switch need not be a
/// global. This one is empty and `[[no_unique_address]]`, so an ungated device pays nothing.
struct NoGate {
    static constexpr bool Gated = false;
    static constexpr bool Front = true;   // not behind a switch (Mux.hpp, MuxFrontGate)

    bool claim() { return true; }

    void release() {}
};

/// What the parts behind a bridge go through while it is not active (Bridge.hpp), which is what
/// the engine owes them when it is active again.
enum class WhileOff : std::uint8_t {
    unpowered,      ///< they lose their supply: brought up from the start, as after power-on
    disconnected,   ///< they stay powered and keep their registers: the engine carries on
};

namespace detail {
    /// A gate that can say "not now, and for a reason": the part is behind a bridge
    /// (Bridge.hpp, BridgeGate). Besides claim() and release() it has `offline()`,
    /// `generation()` -- which steps every time the bridge becomes active -- and names the
    /// bridge, whose `WhileOff` the engine reads.
    template<typename G>
    concept BridgedGate = requires { requires G::Bridged; };
}   // namespace detail

/// No reset line, which is the common case.
///
/// `NoLine` is the marker `HasResetLine` below tests for. It is a marker rather than a type
/// comparison because more than one identical `NoReset` exists across the SDK, and a device
/// handed the wrong one would take the reset path and pulse nothing.
struct NoReset {
    static constexpr bool NoLine = true;

    static void hold() {}

    static void release() {}
};

/// Where a device is with the part on the wire (Device::link()): `starting` while nothing has
/// been acknowledged since the device (re)started, `answering` once it is through its bring-up
/// and the part has acknowledged a transaction since, `absent` while it is parked after NAKs
/// in a row and probed now and then (Presence.hpp). The same enum every bus reports
/// (../Link.hpp), so a status page need not know which bus a part is on. The log prints an
/// enum by its name, so there is no helper to spell it.
using Kvasir::Link;

/// What request<G>() hands back, for answer<G>() to be asked about. A value-initialised one
/// is `none()`: false, and answer<G>() says `pending` for it -- so a caller can keep one
/// Ticket and ask "did I request anything" without a flag beside it.
struct Ticket {
    std::uint32_t id{};

    [[nodiscard]] static constexpr Ticket none() { return {}; }

    [[nodiscard]] constexpr explicit operator bool() const { return id != 0; }

    [[nodiscard]] constexpr bool operator==(Ticket const&) const = default;
};

/// The gaps between one read group's samples over an interval (Device::takeGaps<G>()): the
/// longest, and how many were two periods or more -- a tick the part did not get.
template<typename Duration>
struct SampleGaps {
    Duration      longest{};
    std::uint32_t late{};
};

// -- what a chip may declare ------------------------------------------------------------

template<typename G>
concept ReadGroup = requires { std::span<Step const>{G::Steps}; };

template<typename G>
concept WriteGroup = requires {
    typename G::Value;
    { G::Bytes } -> std::convertible_to<std::size_t>;
};

namespace detail {
    template<typename C>
    concept HasInit = requires { std::span<Step const>{C::Init}; };

    /// The description says what its data sheet says tells the part from any other: a non-empty
    /// `static constexpr std::array Identity{RegisterCheck{...}, ...}` (RegisterCheck.hpp).
    template<typename C>
    concept HasIdentity = requires {
        { C::Identity.size() } -> std::convertible_to<std::size_t>;
        { C::Identity[0] } -> std::convertible_to<RegisterCheck const&>;
    } && (C::Identity.size() != 0);

    template<typename C>
    constexpr std::span<Step const> writtenInit() {
        if constexpr(HasInit<C>) {
            return std::span<Step const>{C::Init};
        } else {
            return std::span<Step const>{};
        }
    }

    /// The bring-up the engine runs for a description with an Identity: one read per identity
    /// register, into the bring-up buffer behind everything the description's own Init reads,
    /// then the comparison (Step::Kind::oracle), then the description's Init. So the identity is
    /// looked at first in every bring-up, whatever the Init script does, and nothing is written
    /// to a part that is not the chip.
    template<HasIdentity C>
    constexpr auto initWithIdentity() {
        constexpr auto                           written = writtenInit<C>();
        constexpr auto                           n       = C::Identity.size();
        std::array<Step, n + 1 + written.size()> steps{};
        auto                                     offset = bufferBytes(written);
        for(std::size_t i = 0; i < n; ++i) {
            auto const& check = C::Identity[i];
            steps[i]          = Step::read({.reg    = check.reg,
                                            .count  = check.width,
                                            .offset = static_cast<std::uint8_t>(offset)});
            offset += check.width;
        }
        steps[n] = Step::oracle();
        for(std::size_t i = 0; i < written.size(); ++i) { steps[n + 1 + i] = written[i]; }
        return steps;
    }

    template<typename C>
    struct EffectiveInit {
        static constexpr std::array<Step, 0> Steps{};
    };

    template<HasIdentity C>
    struct EffectiveInit<C> {
        static constexpr auto Steps = initWithIdentity<C>();
    };

    template<typename C>
    concept HasReads = requires { typename C::Reads; };
    template<typename C>
    concept HasWrites = requires { typename C::Writes; };
    template<typename C>
    concept HasState = requires { typename C::State; };
    template<typename G>
    concept HasPeriod = requires {
        { G::Period } -> std::convertible_to<std::chrono::milliseconds>;
    };
    template<typename G>
    concept HasSample = requires { typename G::Sample; };

    /// A read group whose decode is told what the last reading was. Such a group is the one
    /// that can answer Outcome::unchanged(), and the one whose completion instant is worth
    /// keeping, so it is also what turns on the extra state below. A group that only wants
    /// the timestamp says `static constexpr bool Timestamped = true;`.
    template<typename G>
    concept HasPrevDecode
      = requires(Bytes data, typename G::Sample const& prev) { G::decode(data, prev); };

    template<typename G>
    concept HasTimestamp = requires { G::Timestamped; };

    template<typename G>
    concept KeepsHistory = HasSample<G> && (HasPrevDecode<G> || HasTimestamp<G>);

    /// Per read group, only when it keeps history: what the last frames amounted to and when
    /// the last one landed. An empty member otherwise, so a group that keeps no history costs
    /// nothing -- the same trick VerifySlot uses.
    template<bool Enable, typename TP>
    struct HistorySlot {};

    template<typename TP>
    struct HistorySlot<true, TP> {
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

    /// A cyclic read group whose next deadline follows the reading: a touch controller
    /// polled fast while a finger is down and slowly when the glass is idle. It still
    /// declares Period -- the compile-time "this group is cyclic" -- and period(sample) is
    /// what is actually waited; 0 means "not again until request<G>() asks".
    template<typename G>
    concept HasDynamicPeriod = requires(typename G::Sample const& s) {
        { G::period(s) } -> std::convertible_to<std::chrono::milliseconds>;
    };
    template<typename G>
    concept HasRequest = requires { typename G::Request; };

    /// A read group whose prepare() returns how many bytes this run should read: the same
    /// description serves a four-byte and a sixty-four-byte read (a memory).
    template<typename G>
    concept HasSizedPrepare
      = HasRequest<G> && requires(typename G::Request const& r, std::span<std::byte> b) {
            { G::prepare(r, b) } -> std::convertible_to<std::size_t>;
        };
    template<typename G>
    concept HasItems = requires {
        { G::Items } -> std::convertible_to<std::size_t>;
    };
    template<typename G>
    concept HasInitial = requires { G::Initial; };
    /// A write group that is a one-shot command rather than a state the chip must hold: it
    /// is not replayed after a reset (an EEPROM page write, setting a clock).
    template<typename G>
    concept HasTransient = requires {
        { G::Transient } -> std::convertible_to<bool>;
    } && G::Transient;

    /// Read a written register back and compare: VerifyDelay is how long after the write to
    /// look, VerifyInterval how often to look again. Either may stand alone.
    template<typename G>
    concept HasVerifyDelay = requires {
        { G::VerifyDelay } -> std::convertible_to<std::chrono::milliseconds>;
    };
    template<typename G>
    concept HasVerifyInterval = requires {
        { G::VerifyInterval } -> std::convertible_to<std::chrono::milliseconds>;
    };
    template<typename G>
    concept Verifies = HasVerifyDelay<G> || HasVerifyInterval<G>;

    /// The escape hatch for a register that never reads back as written -- a self-clearing
    /// one-shot bit, a status bit sharing the byte. Default: the bytes must be equal.
    template<typename G>
    concept HasVerifyFn = requires(Bytes written, Bytes readBack) {
        { G::verify(written, readBack) } -> std::convertible_to<bool>;
    };

    template<typename G>
    constexpr std::chrono::milliseconds verifyDelayOf() {
        if constexpr(HasVerifyDelay<G>) {
            return Kvasir::asDuration(G::VerifyDelay);
        } else {
            return std::chrono::milliseconds{0};
        }
    }

    template<typename G>
    constexpr std::chrono::milliseconds verifyIntervalOf() {
        if constexpr(HasVerifyInterval<G>) {
            return Kvasir::asDuration(G::VerifyInterval);
        } else {
            return std::chrono::milliseconds{0};
        }
    }

    /// Per write group, only when it verifies: what is owed a read-back and when.
    template<bool Enable, typename TP>
    struct VerifySlot {};

    template<typename TP>
    struct VerifySlot<true, TP> {
        std::atomic<std::uint32_t> unverified{0};   ///< items written and not yet read back
        TP                         due{};
        std::uint32_t              mismatches{};
        std::uint8_t               tries{};   ///< consecutive rewrites of the same item
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

    template<typename G>
    concept HasReady = requires(Bytes data) {
        { G::ready(data) } -> std::convertible_to<bool>;
    };

    /// Where the first read step of a script puts its bytes: a sized read transfers
    /// `count` bytes there, so decode sees offset + count of the buffer.
    constexpr std::size_t firstReadOffset(std::span<Step const> steps) {
        for(auto const& s : steps) {
            if(s.kind == Step::Kind::read) { return s.offset; }
        }
        return 0;
    }

    constexpr std::size_t readSteps(std::span<Step const> steps) {
        std::size_t n = 0;
        for(auto const& s : steps) {
            if(s.kind == Step::Kind::read) { ++n; }
        }
        return n;
    }

    /// A script with a counted read (Step::readCounted): decode sees what it read.
    constexpr bool hasCounted(std::span<Step const> steps) {
        for(auto const& s : steps) {
            if(s.kind == Step::Kind::read && s.counted) { return true; }
        }
        return false;
    }

    /// A part that NAKs while it wakes (Chip::WakeRetries): the retries of the transaction on
    /// the wire, and all of them. An empty member for every other chip.
    template<bool Enable>
    struct WakeSlot {};

    template<>
    struct WakeSlot<true> {
        std::uint8_t  tries{};     ///< of the transaction on the wire, so far
        std::uint32_t retries{};   ///< over the device's life
    };

    /// A part behind a bridge (BridgedGate): whether the engine has put it offline, and the
    /// bridge's generation it last came back in. An empty member for every other device.
    template<bool Enable>
    struct BridgeSlot {};

    template<>
    struct BridgeSlot<true> {
        bool          offline{};
        std::uint32_t generation{};
    };

    constexpr bool hasCheck(std::span<Step const> steps) {
        for(auto const& s : steps) {
            if(s.kind == Step::Kind::check || s.kind == Step::Kind::stopUnless) { return true; }
        }
        return false;
    }

    template<typename G>
    concept HasClaims = requires { typename G::Claims; };

    template<typename C>
    concept HasPrimary = requires { typename C::Primary; };

    /// The read group a Device's latest(), fresh(), seq() mean without a group name:
    /// `Chip::Primary` when the description names one, else its only read group, else void.
    template<typename C, typename Rs>
    struct PrimaryOf {
        using type = void;
    };

    template<HasPrimary C, typename Rs>
    struct PrimaryOf<C, Rs> {
        using type = typename C::Primary;
    };

    template<typename C, typename R>
        requires(!HasPrimary<C>)
    struct PrimaryOf<C, List<R>> {
        using type = R;
    };

    /// A write group whose every set() goes on the wire, the value the chip already holds
    /// too: a register with a self-clearing command bit (the PCA9956B's CLRERR), or a command
    /// (a PMTK sentence). A Transient group behaves the same way.
    template<typename G>
    concept AlwaysWrites = requires {
        { G::AlwaysWrite } -> std::convertible_to<bool>;
    } && G::AlwaysWrite;

    template<typename C>
    struct ReadsOf {
        using type = List<>;
    };

    template<HasReads C>
    struct ReadsOf<C> {
        using type = typename C::Reads;
    };

    template<typename C>
    struct WritesOf {
        using type = List<>;
    };

    template<HasWrites C>
    struct WritesOf<C> {
        using type = typename C::Writes;
    };

    template<typename C>
    struct StateOf {
        using type = Empty;
    };

    template<HasState C>
    struct StateOf<C> {
        using type = typename C::State;
    };

    template<typename G>
    struct SampleOf {
        using type = Empty;
    };

    template<HasSample G>
    struct SampleOf<G> {
        using type = typename G::Sample;
    };

    template<typename G>
    struct RequestOf {
        using type = Empty;
    };

    template<HasRequest G>
    struct RequestOf<G> {
        using type = typename G::Request;
    };

    template<typename G>
    constexpr std::size_t itemsOf() {
        if constexpr(HasItems<G>) {
            return static_cast<std::size_t>(G::Items);
        } else {
            return 1;
        }
    }

    /// What a write group's encode() gives back. Nearly every chip returns one Step: one
    /// transaction per item. A chip whose item is several transactions -- a window command
    /// and the display RAM behind it -- returns a std::array<Step, N>, and the engine runs
    /// those N back to back for that one item.
    template<typename G>
    struct EncodeResultOf {
        using type = decltype(G::encode(std::declval<typename G::Value const&>(),
                                        std::declval<std::span<std::byte>>()));
    };

    template<HasItems G>
    struct EncodeResultOf<G> {
        using type = decltype(G::encode(std::declval<typename G::Value const&>(),
                                        std::size_t{0},
                                        std::declval<std::span<std::byte>>()));
    };

    /// A write group whose item is more than one transaction.
    template<typename G>
    concept MultiStepWrite = !std::is_same_v<typename EncodeResultOf<G>::type, Step>;

    /// How many transactions one item of a write group is.
    template<typename G>
    constexpr std::size_t writeStepsOf() {
        using R = typename EncodeResultOf<G>::type;
        if constexpr(std::is_same_v<R, Step>) {
            return 1;
        } else {
            static_assert(std::is_same_v<R, std::array<Step, std::tuple_size_v<R>>>,
                          "a write group's encode() returns a Step (one transaction) or a "
                          "std::array<Step, N> (N transactions, run back to back)");
            return std::tuple_size_v<R>;
        }
    }

    template<typename G>
    constexpr std::chrono::milliseconds periodOf() {
        if constexpr(HasPeriod<G>) {
            return Kvasir::asDuration(G::Period);
        } else {
            return std::chrono::milliseconds{0};
        }
    }

    template<template<typename> class Slot, typename L>
    struct Slots;

    template<template<typename> class Slot, typename... Gs>
    struct Slots<Slot, List<Gs...>> {
        using type = std::tuple<Slot<Gs>...>;
    };

    template<typename T, typename L>
    struct IndexOf;

    template<typename T, typename... Gs>
    struct IndexOf<T, List<Gs...>> {
        static constexpr std::size_t value = [] {
            std::size_t i = 0;
            std::size_t r = sizeof...(Gs);
            ((std::is_same_v<T, Gs> ? (r = i) : 0, ++i), ...);
            return r;
        }();
    };

    template<typename F,
             std::size_t... Is>
    constexpr void withIndex(std::size_t i,
                             F&&         f,
                             std::index_sequence<Is...>) {
        ((i == Is ? static_cast<void>(f(std::integral_constant<std::size_t, Is>{})) : void()), ...);
    }

    template<typename L,
             typename F>
    constexpr void forEach(F&& f) {
        [&]<std::size_t... Is>(std::index_sequence<Is...>) {
            (f(std::integral_constant<std::size_t, Is>{}), ...);
        }(std::make_index_sequence<L::size>{});
    }

    template<typename L>
    struct Nth;

    template<typename... Gs>
    struct Nth<List<Gs...>> {
        template<std::size_t I>
        using type = std::tuple_element_t<I, std::tuple<Gs...>>;
    };

    template<typename L>
    [[nodiscard]] constexpr std::size_t maxPayloadOf() {
        std::size_t n = 0;
        forEach<L>([&](auto i) {
            using G = typename Nth<L>::template type<decltype(i)::value>;
            if constexpr(ReadGroup<G>) {
                auto const p = maxPayload(std::span<Step const>{G::Steps});
                n            = p > n ? p : n;
            } else {
                n = static_cast<std::size_t>(G::Bytes) > n ? static_cast<std::size_t>(G::Bytes) : n;
            }
        });
        return n;
    }

    /// The most transactions any write group's item is; one when every group returns a
    /// single Step (and then the Device carries no script store at all).
    template<typename L>
    constexpr std::size_t maxWriteStepsOf() {
        std::size_t n = 1;
        forEach<L>([&](auto i) {
            auto const k = writeStepsOf<typename Nth<L>::template type<decltype(i)::value>>();
            n            = k > n ? k : n;
        });
        return n;
    }

    template<typename L>
    constexpr std::size_t maxVerifyBytesOf() {
        std::size_t n = 0;
        forEach<L>([&](auto i) {
            using G = typename Nth<L>::template type<decltype(i)::value>;
            if constexpr(Verifies<G>) {
                n = static_cast<std::size_t>(G::Bytes) > n ? static_cast<std::size_t>(G::Bytes) : n;
            }
        });
        return n;
    }

    template<typename L>
    constexpr bool anyVerifies() {
        bool any = false;
        forEach<L>([&](auto i) {
            any = any || Verifies<typename Nth<L>::template type<decltype(i)::value>>;
        });
        return any;
    }

    /// The reset line's pin claim, if it has one, becomes the driver's.
    template<typename R>
    struct ResetClaims {};

    template<HasClaims R>
    struct ResetClaims<R> {
        using Claims = typename R::Claims;
    };
}   // namespace detail

/// The driver. `Chip` is the description (chips/); `Config` may set `Address`, the presence
/// knobs (PresenceDefaults: AbsentAfterNaks, ProbeInterval, ProbeIntervalMax) and the engine's
/// own (EngineDefaults below: MaxRetries, FaultBackoff, FaultsBeforeReinit, InFlightTimeout,
/// UnidentifiedRetry); `Reset` is a line with hold()/release() (any GPIO reset callable) for a
/// chip that declares ResetLow/ResetSettle.
///
/// What a run-time caller may change: `period<G>(ms)` sets a cyclic read group's period from
/// then on (0 parks it until request<G>() asks; the description's Period stays the nominal
/// rate the bus load is computed from).
template<typename I2c,
         typename Clock,
         Chip ChipT,
         typename Config = DefaultConfig,
         typename Reset  = NoReset,
         typename Gate   = NoGate>
struct Device
  : detail::EngineState<I2c, Clock>
  , detail::ResetClaims<Reset> {
    using Chip = ChipT;
    /// The bus and the clock this device is on, so a Bus<> can check that the devices it
    /// was handed are all on the bus it names -- a mixed set would give a queue depth and a
    /// bus load for traffic that is not on one wire.
    using I2cBus    = I2c;
    using ClockT    = Clock;
    using GateT     = Gate;
    using ConfigT   = Config;
    using ResetT    = Reset;
    using TimePoint = typename Clock::time_point;
    using Duration  = typename Clock::duration;
    using PendingT  = Pending<I2c, Clock>;
    using PresenceT = Presence<Clock, Config>;
    using State     = typename detail::StateOf<Chip>::type;

    /// The state machine's own fields and its two enums (Engine.hpp). A base that depends on a
    /// template parameter is not searched by unqualified lookup, so every name it brings is
    /// named here once instead of `this->` at some hundreds of use sites.
    using EngineStateT = detail::EngineState<I2c, Clock>;
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
    using EngineStateT::oracleFailed_;
    using EngineStateT::pending_;
    using EngineStateT::phase_;
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

    /// Behind a bridge that can be switched off (Bridge.hpp): link() can say `offline`.
    static constexpr bool Bridged = detail::BridgedGate<Gate>;

    static constexpr std::uint8_t Address = [] {
        if constexpr(requires { Config::Address; }) {
            return static_cast<std::uint8_t>(Config::Address);
        } else {
            return static_cast<std::uint8_t>(Chip::Address);
        }
    }();

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

    static constexpr std::chrono::milliseconds StartupDelay = [] {
        if constexpr(requires { Chip::StartupDelay; }) {
            return Kvasir::asDuration(Chip::StartupDelay);
        } else {
            return std::chrono::milliseconds{0};
        }
    }();

    static constexpr bool HasResetLine = !requires { Reset::NoLine; };

    static constexpr std::chrono::milliseconds ResetLow = [] {
        if constexpr(requires { Chip::ResetLow; }) {
            return Kvasir::asDuration(Chip::ResetLow);
        } else {
            return std::chrono::milliseconds{1};
        }
    }();

    static constexpr std::chrono::milliseconds ResetSettle = [] {
        if constexpr(requires { Chip::ResetSettle; }) {
            return Kvasir::asDuration(Chip::ResetSettle);
        } else {
            return std::chrono::milliseconds{1};
        }
    }();

    // The policy knobs (EngineDefaults), each overridable by the Config.
    static constexpr std::uint8_t MaxRetries = [] {
        if constexpr(requires { Config::MaxRetries; }) {
            return static_cast<std::uint8_t>(Config::MaxRetries);
        } else {
            return EngineDefaults::MaxRetries;
        }
    }();

    static constexpr std::chrono::milliseconds FaultBackoff = [] {
        if constexpr(requires { Config::FaultBackoff; }) {
            return Kvasir::asDuration(Config::FaultBackoff);
        } else {
            return Kvasir::asDuration(EngineDefaults::FaultBackoff);
        }
    }();

    static constexpr std::uint8_t FaultsBeforeReinit = [] {
        if constexpr(requires { Config::FaultsBeforeReinit; }) {
            return static_cast<std::uint8_t>(Config::FaultsBeforeReinit);
        } else {
            return EngineDefaults::FaultsBeforeReinit;
        }
    }();

    static constexpr std::chrono::milliseconds InFlightTimeout = [] {
        if constexpr(requires { Config::InFlightTimeout; }) {
            return Kvasir::asDuration(Config::InFlightTimeout);
        } else {
            return Kvasir::asDuration(EngineDefaults::InFlightTimeout);
        }
    }();

    /// A NAK is the part still waking: resubmit it this often, this long apart, before it counts.
    static constexpr std::uint8_t WakeRetries = [] {
        if constexpr(requires { Chip::WakeRetries; }) {
            return static_cast<std::uint8_t>(Chip::WakeRetries);
        } else {
            return std::uint8_t{0};
        }
    }();

    static constexpr std::chrono::milliseconds WakeRetryDelay = [] {
        if constexpr(requires { Chip::WakeRetryDelay; }) {
            return Kvasir::asDuration(Chip::WakeRetryDelay);
        } else {
            return std::chrono::milliseconds{1};
        }
    }();

    static constexpr std::chrono::milliseconds UnidentifiedRetry = [] {
        if constexpr(requires { Config::UnidentifiedRetry; }) {
            return Kvasir::asDuration(Config::UnidentifiedRetry);
        } else {
            return Kvasir::asDuration(EngineDefaults::UnidentifiedRetry);
        }
    }();

    /// What the bus's CallbackSize has to hold: the completion lambda the engine hands it.
    /// Checked against the bus when it says what it holds.
    static constexpr std::size_t CallbackBytes
      = sizeof(decltype(std::declval<PendingT&>().callback()));

    static_assert(
      [] {
          if constexpr(requires { I2c::CallbackSize; }) {
              return static_cast<std::size_t>(I2c::CallbackSize) >= CallbackBytes;
          }
          return true;
      }(),
      "the bus behavior's CallbackSize is smaller than the engine's completion lambda "
      "(Device::CallbackBytes): raise the I2CBehaviorQueued CallbackSize template argument");

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
    static_assert(
      [] {
          if constexpr(detail::HasAddresses<Chip>) {
              for(auto const a : Chip::Addresses) {
                  if(a == Address) { return true; }
              }
              return false;
          }
          return true;
      }(),
      "Config::Address is not one of the addresses this chip can have (Chip::Addresses)");

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
    struct ReadSlot : detail::ReadSlotBase<Clock> {
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
            this->period     = detail::periodOf<G>();
        }

        std::array<std::byte, Bytes>                                                  buf{};
        Sample                                                                        sample{};
        Request                                                                       request{};
        [[no_unique_address]] detail::HistorySlot<detail::KeepsHistory<G>, TimePoint> history{};
    };

    template<typename G>
    struct WriteSlot : detail::WriteSlotBase<Clock> {
        static constexpr std::size_t Items = detail::itemsOf<G>();
        using Value                        = typename G::Value;

        /// Every item's dirty bit. Computed in 64 bits: Items may be 32.
        static constexpr std::uint32_t AllItems
          = static_cast<std::uint32_t>((1ULL << Items) - 1ULL);

        std::array<std::byte, static_cast<std::size_t>(G::Bytes)>                buf{};
        std::array<Value, Items>                                                 values{};
        [[no_unique_address]] detail::VerifySlot<detail::Verifies<G>, TimePoint> verify{};
    };

    /// One entry per cyclic read group, so the engine can walk them without their types.
    static constexpr auto ReadInfos = [] {
        std::array<detail::ReadGroupInfo, Reads::size == 0 ? 1 : Reads::size> infos{};
        detail::forEach<Reads>([&](auto i) {
            using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
            infos[decltype(i)::value] = detail::ReadGroupInfo{
              .periodMs      = static_cast<std::uint32_t>(detail::periodOf<G>().count()),
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
              .allItems = WriteSlot<W>::AllItems,
              .periodMs = static_cast<std::uint32_t>(detail::periodOf<W>().count()),
              .verifies = detail::Verifies<W>};
        });
        return infos;
    }();

    using ReadSlots  = typename detail::Slots<ReadSlot, Reads>::type;
    using WriteSlots = typename detail::Slots<WriteSlot, Writes>::type;

    static constexpr std::size_t MaxPayload = [] {
        auto const initBytes  = maxPayload(InitSteps);
        auto const readBytes  = detail::maxPayloadOf<Reads>();
        auto const writeBytes = detail::maxPayloadOf<Writes>();
        return std::max({initBytes, readBytes, writeBytes});
    }();

    /// One item of a write group may be several transactions (a window command and the
    /// display RAM behind it). One when every group's encode() returns a single Step, and
    /// then the script store below is empty.
    static constexpr std::size_t MaxWriteSteps   = detail::maxWriteStepsOf<Writes>();
    static constexpr bool        MultiStepWrites = MaxWriteSteps > 1;

    static constexpr std::size_t InitBytes = [] {
        auto const n = bufferBytes(InitSteps);
        return n == 0 ? std::size_t{1} : n;
    }();

    static constexpr bool        AnyVerify      = detail::anyVerifies<Writes>();
    static constexpr std::size_t MaxVerifyBytes = detail::maxVerifyBytesOf<Writes>();

    // A read-back is addressed by the register the write went to; on a chip without registers
    // (an I2C switch, a DAC with one word) it is a bare read of as many bytes as were written,
    // which is what such parts return.

    constexpr Device() = default;

private:
    /// The state the engine works on is this object: every trampoline below casts back.
    [[nodiscard]] static Device& self_(EngineStateT& e) { return static_cast<Device&>(e); }

    /// What the engine (Engine.hpp) needs of this chip: constants, and one trampoline per hole.
    /// `static constexpr`, so it is one table in flash per Device instantiation and nothing per
    /// object -- the engine takes it as an argument rather than the device carrying a pointer.
    /// This is where the chip's groups are switched over, once, instead of at every call site.
    static constexpr detail::Ops<I2c, Clock> EngineOps{
      .tx        = [](EngineStateT& e) { return std::span<std::byte>{self_(e).tx_}; },
      .script    = [](EngineStateT& e) { return self_(e).script_(); },
      .buffer    = [](EngineStateT& e) { return self_(e).buffer_(); },
      .mayTalk   = [](EngineStateT& e) -> bool { return self_(e).presence_.mayTalk(); },
      .claimGate = [](EngineStateT& e) -> bool {
          if constexpr(Gate::Gated) {
              return self_(e).gate_.claim();
          } else {
              static_cast<void>(e);
              return true;
          }
      },
      .releaseGate = [](EngineStateT& e) { self_(e).letGateGo_(); },
      .sizeRead    = [](EngineStateT& e) { self_(e).sizeRead_(); },
      .countRead   = [](EngineStateT& e, std::uint8_t n) { self_(e).countRead_(n); },
      .ready       = [](EngineStateT& e) -> bool { return self_(e).ready_(); },
      .oracle      = [](EngineStateT& e) -> bool {
          auto& d            = self_(e);
          d.identityMatched_ = d.identityMatches_();
          if(!d.identityMatched_) { d.oracleFailed_ = true; }
          return d.identityMatched_;
      },
      .identify = [](EngineStateT& e) -> bool {
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
      },
      .faultBackoffMs = static_cast<std::uint32_t>(FaultBackoff.count()),
      .clearInit      = [](EngineStateT& e) { self_(e).init_.fill(std::byte{0}); },
      .finishInit     = [](EngineStateT& e, TimePoint now) { self_(e).finishInit_(now); },
      .wakeReset      = [](EngineStateT& e) { self_(e).wakeReset_(); },
      .redirtyItem    = [](EngineStateT& e) { self_(e).redirtyItem_(); },
      .unserve        = [](EngineStateT& e) { self_(e).unserve_(); },
      .forgetSamples  = [](EngineStateT& e) { self_(e).forgetSamples_(); },
      .setupFinal     = [](EngineStateT& e) -> bool {
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
      },
      .unidentifiedRetryMs = static_cast<std::uint32_t>(UnidentifiedRetry.count()),
      .startupSchedule     = [](EngineStateT& e, TimePoint now) { self_(e).startupSchedule_(now); },
      .presenceTurn
      = [](EngineStateT& e, TimePoint now) { return self_(e).presence_.turn(now, Address); },
      .presenceAck   = [](EngineStateT& e, TimePoint now) { self_(e).presence_.ack(now, Address); },
      .presenceNak   = [](EngineStateT& e) { self_(e).presence_.nak(); },
      .presenceFault = [](EngineStateT& e) { self_(e).presence_.fault(); },
      .inFlightTimeoutMs = static_cast<std::uint32_t>(InFlightTimeout.count()),
      .wakeRetry         = [](EngineStateT& e, TimePoint now) { return self_(e).wakeRetry_(now); },
      .bridgeTurn        = [](EngineStateT& e, TimePoint now) -> bool {
          if constexpr(Bridged) {
              return self_(e).bridgeTurn_(now);
          } else {
              static_cast<void>(e);
              static_cast<void>(now);
              return true;
          }
      },
      .resetHold =
        [](EngineStateT& e) {
            static_cast<void>(e);
            if constexpr(HasResetLine) { Reset::hold(); }
        },
      .resetRelease =
        [](EngineStateT& e) {
            static_cast<void>(e);
            if constexpr(HasResetLine) { Reset::release(); }
        },
      .startupDelayMs = static_cast<std::uint32_t>(StartupDelay.count()),
      .resetLowMs     = static_cast<std::uint32_t>(ResetLow.count()),
      .resetSettleMs  = static_cast<std::uint32_t>(ResetSettle.count()),
      .reads          = std::span<detail::ReadGroupInfo const>{ReadInfos}.first(Reads::size),
      .readSlot       = [](EngineStateT& e, std::uint8_t g) -> detail::ReadSlotBase<Clock>& {
          detail::ReadSlotBase<Clock>* r = nullptr;
          detail::withIndex(
            g,
            [&](auto i) { r = &std::get<decltype(i)::value>(self_(e).reads_); },
            std::make_index_sequence<Reads::size>{});
          return *r;
      },
      .periodNow =
        [](EngineStateT& e, std::uint8_t g) {
            std::chrono::milliseconds p{};
            detail::withIndex(
              g,
              [&](auto i) {
                  using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                  p       = self_(e).template periodNow_<G>();
              },
              std::make_index_sequence<Reads::size>{});
            return p;
        },
      .dynamicPeriod =
        [](EngineStateT& e, std::uint8_t g) {
            std::chrono::milliseconds p{};
            detail::withIndex(
              g,
              [&](auto i) {
                  using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                  if constexpr(detail::HasDynamicPeriod<G>) {
                      p = G::period(std::get<decltype(i)::value>(self_(e).reads_).sample);
                  }
              },
              std::make_index_sequence<Reads::size>{});
            return p;
        },
      .prepareRead = [](EngineStateT& e, std::uint8_t g) { self_(e).prepareRead_(g); },
      .decode =
        [](EngineStateT& e, std::uint8_t g) {
            auto&                d = self_(e);
            detail::DecodeResult r{};
            detail::withIndex(
              g,
              [&](auto i) {
                  using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
                  auto& s = std::get<decltype(i)::value>(d.reads_);
                  if constexpr(detail::HasSample<G>) {
                      auto const outcome = d.template decode_<G>(s);
                      using Result       = std::remove_cvref_t<decltype(outcome)>;
                      if(outcome.kind == Result::Kind::retry) {
                          r = {detail::DecodeKind::retry,
                               static_cast<std::uint32_t>(outcome.retryAfter.count())};
                      } else if(outcome.kind == Result::Kind::ok) {
                          s.sample = outcome.value;
                          if constexpr(detail::KeepsHistory<G>) {
                              d.template noteSample_<G>(s.history, d.pending_.stamp());
                          }
                          r = {detail::DecodeKind::ok, 0};
                      } else if(outcome.kind == Result::Kind::unchanged) {
                          if constexpr(detail::KeepsHistory<G>) { ++s.history.unchanged; }
                          r = {detail::DecodeKind::unchanged, 0};
                      } else {
                          r = {detail::DecodeKind::rejected, 0};
                      }
                  } else {
                      r = {detail::DecodeKind::ok, 0};
                  }
              },
              std::make_index_sequence<Reads::size>{});
            return r;
        },
      .writes    = std::span<detail::WriteGroupInfo const>{WriteInfos}.first(Writes::size),
      .writeSlot = [](EngineStateT& e, std::uint8_t w) -> detail::WriteSlotBase<Clock>& {
          detail::WriteSlotBase<Clock>* r = nullptr;
          detail::withIndex(
            w,
            [&](auto i) { r = &std::get<decltype(i)::value>(self_(e).writes_); },
            std::make_index_sequence<Writes::size>{});
          return *r;
      },
      .encodeWrite =
        [](EngineStateT& e, std::uint8_t w, std::uint8_t item) {
            auto& d = self_(e);
            detail::withIndex(
              w,
              [&](auto i) {
                  using W = typename detail::Nth<Writes>::template type<decltype(i)::value>;
                  d.template encodeWrite_<W>(std::get<decltype(i)::value>(d.writes_), item);
              },
              std::make_index_sequence<Writes::size>{});
        },
      .afterWrite =
        [](EngineStateT& e, std::uint8_t w, std::uint8_t item, TimePoint now) {
            auto& d = self_(e);
            detail::withIndex(
              w,
              [&](auto i) {
                  using W = typename detail::Nth<Writes>::template type<decltype(i)::value>;
                  auto& s = std::get<decltype(i)::value>(d.writes_);
                  // The part now holds the value, so what the chip State says about it
                  // follows: a full scale that changes what decode() multiplies by.
                  if constexpr(requires(typename W::Value const& v, State& st) {
                                   W::applied(v, st);
                               }) {
                      W::applied(s.values[item], d.state_);
                  }
                  if constexpr(detail::Verifies<W>) {
                      // owed a read-back; the delay is the chip's settling time
                      s.verify.unverified.fetch_or(1U << item, std::memory_order_relaxed);
                      constexpr auto delay    = detail::verifyDelayOf<W>();
                      constexpr auto interval = detail::verifyIntervalOf<W>();
                      s.verify.due
                        = now + (delay != std::chrono::milliseconds::zero() ? delay : interval);
                  }
              },
              std::make_index_sequence<Writes::size>{});
        },
      .startVerifyGroup   = [](EngineStateT& e,
                               std::uint8_t  w,
                               TimePoint     now) { return self_(e).startVerify_(w, now); },
      .finishVerify       = [](EngineStateT& e, TimePoint now) { self_(e).finishVerify_(now); },
      .name               = Chip::Name,
      .address            = Address,
      .registerBytes      = static_cast<std::uint8_t>(RegisterBytes),
      .initEmpty          = InitSteps.empty(),
      .faultsBeforeReinit = FaultsBeforeReinit,
      .bridged            = Bridged,
      .hasResetLine       = HasResetLine,
      .maxRetries         = MaxRetries};

public:
    // -- the application's side ------------------------------------------------------------

    /// Once per loop turn: everything happens here.
    /// Once per loop turn: everything happens here (Engine.hpp).
    void handler() { detail::Engine<I2c, Clock>::handler(*this, EngineOps, Clock::now()); }

    /// Start over from the reset, as after power-on: for a caller that did to the part what the
    /// engine cannot see -- cycled the supply it shares with others (PowerRail.hpp). Whatever
    /// was on the wire is abandoned, a write in flight is owed again, the part counts as
    /// present and is brought up from the start (StartupDelay, the identity, Init), and
    /// everything the application had written is re-sent after it, as after any bring-up.
    void restart() {
        reset_();
        presence_.restart();
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
        if constexpr(Bridged) {
            if(bridge_.offline || gate_.offline()) { return Link::offline; }
        }
        if(!presence_.present()) { return Link::absent; }
        return up_ && acked_ ? Link::answering : Link::starting;
    }

    /// One line for a log: what the device is, where, and how it is doing. What Bus::logHealth
    /// says per device, for a firmware with one device and no Bus.
    void logHealth() const {
        detail::logHealth(Chip::Name, Address, link(), samples(), rejected(), errors());
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
            if constexpr(detail::HasSample<G>
                         && detail::periodOf<G>() > std::chrono::milliseconds::zero())
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
    /// says (G::period(State const&)), else the description's Period.
    template<typename G>
    [[nodiscard]] std::chrono::milliseconds period() const {
        return periodNow_<G>();
    }

    /// Run cyclic read group G every `p` from now on; 0 parks it until request<G>() asks. The
    /// description's Period stays the nominal rate the bus load is planned for; this is the rate
    /// a firmware picks at run time, and the late-gap count follows it -- slower while a display is off, faster while a
    /// value is being watched.
    template<typename G>
    void period(std::chrono::milliseconds p) {
        static_assert(detail::periodOf<G>() > std::chrono::milliseconds::zero(),
                      "period<G>(p): G is not a cyclic read group (it has no Period), so it "
                      "only runs when request<G>() asks");
        auto& s     = std::get<ReadSlot<G>>(reads_);
        s.period    = p;
        s.periodSet = true;
        s.due       = p > std::chrono::milliseconds::zero() ? Clock::now() + p : TimePoint::max();
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
    [[nodiscard]] std::uint32_t samples() const {
        return std::get<ReadSlot<G>>(reads_).samples;
    }

    /// Samples across every read group, over the device's life.
    [[nodiscard]] std::uint32_t samples() const {
        std::uint32_t n = 0;
        detail::forEach<Reads>([&](auto i) { n += std::get<decltype(i)::value>(reads_).samples; });
        return n;
    }

    /// Frames of G the decode rejected (CRC, busy past MaxRetries, impossible values).
    template<typename G>
    [[nodiscard]] std::uint32_t rejected() const {
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
        requires(detail::KeepsHistory<G>)
    {
        return std::get<ReadSlot<G>>(reads_).history.at;
    }

    /// The gaps between G's samples since the last call, measured between the bus completions
    /// of the samples rather than the loop turns that noticed them: the longest, and how many
    /// were two periods or more. Taken and cleared, so one reader; the loop's.
    template<typename G>
    [[nodiscard]] SampleGaps<Duration> takeGaps()
        requires(detail::KeepsHistory<G>)
    {
        auto&                      h = std::get<ReadSlot<G>>(reads_).history;
        SampleGaps<Duration> const gaps{.longest = h.longestGap, .late = h.lateGaps};
        h.longestGap = Duration{};
        h.lateGaps   = 0;
        return gaps;
    }

    [[nodiscard]] std::uint32_t rejected() const {
        std::uint32_t n = 0;
        detail::forEach<Reads>([&](auto i) { n += std::get<decltype(i)::value>(reads_).rejected; });
        return n;
    }

    /// Run read group G once, now (its period, if any, continues): an on-demand read. The
    /// ticket is for answer<G>(); the requests made before a run starts are all served by it.
    /// An atomic step, so an interrupt may ask (a touch controller's INT line).
    template<typename G>
    Ticket request() {
        auto& s = std::get<ReadSlot<G>>(reads_);
        return Ticket{s.asked.fetch_add(1, std::memory_order_acq_rel) + 1};
    }

    /// Run G once with a Request the group's prepare() turns into bytes. Unlike request<G>()
    /// this stores the Request first and is the loop's to call: an interrupt that wants a
    /// read with an argument sets a flag and lets the loop ask.
    template<typename G>
    Ticket request(typename G::Request const& r) {
        auto& s   = std::get<ReadSlot<G>>(reads_);
        s.request = r;
        return Ticket{s.asked.fetch_add(1, std::memory_order_acq_rel) + 1};
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
    }

    /// Every item of write group W has been changed in place.
    template<typename W>
    void touchAll() {
        auto& s = std::get<WriteSlot<W>>(writes_);
        s.owned = true;
        s.known = WriteSlot<W>::AllItems;
        s.dirty.store(WriteSlot<W>::AllItems, std::memory_order_release);
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

    /// Transactions that failed (NAK, bus fault, timeout), all phases. A NAK a wake retry
    /// (Chip::WakeRetries) recovered from is not one.
    [[nodiscard]] std::uint32_t errors() const { return errors_; }

    /// NAKs that were put on the wire again because the part may still be waking
    /// (Chip::WakeRetries), over the device's life; 0 for a chip that declares none.
    [[nodiscard]] std::uint32_t wakeRetries() const {
        if constexpr(WakeRetries > 0) {
            return wake_.retries;
        } else {
            return 0;
        }
    }

    /// What setup() made of the bring-up (the chip id, trimming...).
    [[nodiscard]] State const& state() const { return state_; }

    /// The gate, for an application that has to point it at something: a device behind an
    /// I2C switch is told which switch and which arbiter at start-up (Mux.hpp). An ungated
    /// device has an empty one and nothing to say to it.
    [[nodiscard]] Gate& gate() { return gate_; }

    /// setup() accepted the chip (true when there is no setup()). While it is false and
    /// UnidentifiedRetry is set, the description runs nothing but its bring-up (link() stays
    /// `starting`, valid() false); with UnidentifiedRetry 0 it runs regardless.
    [[nodiscard]] bool identified() const { return identified_; }

    /// The part's identity registers read what its data sheet says (the description's
    /// `Identity`, RegisterCheck.hpp) in the latest bring-up that got that far; true for a
    /// description that names none. A part that fails this is never `identified()`, so never
    /// `answering()`, and nothing but the identity reads has gone to it.
    [[nodiscard]] bool identityMatched() const { return identityMatched_; }

    /// What identity register `i` of the description's `Identity` read in the latest bring-up
    /// that got to it, as one value; 0 before that.
    [[nodiscard]] std::uint32_t identity(std::size_t i) const
        requires(HasIdentity)
    {
        return identity_[i];
    }

    /// Bring-ups whose setup() said "not this chip".
    [[nodiscard]] std::uint16_t unidentified() const { return unidentified_; }

private:
    /// set() compares against what the group holds: only a group that is a state (not
    /// Transient, not AlwaysWrite) and whose Value can be compared.
    template<typename W>
    static constexpr bool SkipsUnchanged = !detail::HasTransient<W> && !detail::AlwaysWrites<W>
                                        && std::equality_comparable<typename W::Value>;

    template<typename W,
             typename S>
    static void store_(S&                       s,
                       std::size_t              i,
                       typename W::Value const& v) {
        s.values[i] = v;
        s.owned     = true;
        s.known     = s.known | (1U << i);
        s.dirty.fetch_or(1U << i, std::memory_order_release);
    }

    /// A timestamped sample of G landed at `at`: the gap since the one before, for takeGaps().
    template<typename G,
             typename H>
    void noteSample_(H&        h,
                     TimePoint at) const {
        if(h.at != TimePoint{}) {
            auto const gap = at - h.at;
            h.longestGap   = std::max(h.longestGap, gap);
            if constexpr(detail::periodOf<G>() > std::chrono::milliseconds::zero()
                         && !detail::HasDynamicPeriod<G>)
            {
                if(gap >= periodNow_<G>() * 2) { ++h.lateGaps; }
            }
        }
        h.at = at;
    }

    // -- the turn --------------------------------------------------------------------------

    /// Start over from the reset: parked, probed, or the in-flight net. Whatever was on the
    /// wire is abandoned -- but not what it was for: a write in flight is owed again (its
    /// dirty bit went at startWrite_), or a Transient one -- an EEPROM page, a clock set --
    /// would be lost, since the bring-up does not replay it. And the gate is let go.
    void reset_() { detail::Engine<I2c, Clock>::reset(*this, EngineOps); }

    void wakeReset_() {
        if constexpr(WakeRetries > 0) { wake_.tries = 0; }
    }

    /// The bridge went, and the parts behind it keep their registers (WhileOff::disconnected):
    /// the run on the wire is abandoned as in reset_() -- a write owed again, a read's requests
    /// still pending -- but the bring-up stands. The samples do not: valid() waits for one
    /// taken after the return.
    void pause_() { detail::Engine<I2c, Clock>::pause(*this, EngineOps); }

    /// The part is behind a bridge: false while the engine has to keep still. A transaction
    /// still on the wire when the bridge stops being active is waited for -- the gate is held
    /// until then, which is what keeps a bridge the firmware switches from cutting it
    /// (BridgeLine, `closing`) -- and whatever it says is dropped: a bridge that went by itself
    /// fails it, and that is no news about the part.
    bool bridgeTurn_(TimePoint now) {
        if(gate_.offline()) {
            if(inFlight_) {
                if(pending_.take() == PendingT::Outcome::running
                   && now - submittedAt_ <= InFlightTimeout)
                {
                    return false;
                }
                inFlight_ = false;
                pending_.clear();
            }
            if(!bridge_.offline) { goOffline_(); }
            return false;
        }
        if(bridge_.generation != gate_.generation()) {
            if(!bridge_.offline) { goOffline_(); }   // off and on again between two turns
            bridge_.generation = gate_.generation();
            bridge_.offline    = false;
            presence_.restart();
            detail::logBridge(Chip::Name, Address, true);
        }
        return true;
    }

    void goOffline_() {
        bridge_.offline = true;
        if(Gate::Bridge::WhileOff == WhileOff::unpowered) {
            reset_();
        } else {
            pause_();
        }
        detail::logBridge(Chip::Name, Address, false);
    }

    /// A NAK the part may give while it wakes: true when it is put on the wire again after
    /// WakeRetryDelay, the gate let go meanwhile, rather than counted.
    bool wakeRetry_(TimePoint now) {
        if constexpr(WakeRetries > 0) {
            if(wake_.tries >= WakeRetries) {
                wake_.tries = 0;
                return false;
            }
            ++wake_.tries;
            ++wake_.retries;
            waiting_    = true;
            afterDelay_ = false;   // the same step again, not the next one
            waitUntil_  = now + WakeRetryDelay;
            letGateGo_();
            return true;
        } else {
            static_cast<void>(now);
            return false;
        }
    }

    /// The read on the wire is abandoned, not failed: the requests it took on are owed again
    /// and run after the bring-up, their tickets still pending.
    void unserve_() {
        detail::withIndex(
          group_,
          [&](auto i) {
              auto& s = std::get<decltype(i)::value>(reads_);
              if(s.serving) {
                  s.serving = false;
                  s.started = s.served;
              }
          },
          std::make_index_sequence<Reads::size>{});
    }

    /// A bring-up starts over: what the part said before it is not a reading of the part as it
    /// is now, so valid() waits for a new sample of every group. The lifetime counts stay.
    void forgetSamples_() {
        detail::forEach<Reads>(
          [&](auto i) { std::get<decltype(i)::value>(reads_).current = false; });
    }

    // -- running a script ------------------------------------------------------------------

    /// A run is over -- finished, failed, rejected, or abandoned by a reset. Every exit from
    /// a script goes through here, so a gate is never held a turn past the end of a run.
    void endRun_() { detail::Engine<I2c, Clock>::endRun(*this, EngineOps); }

    /// The run is about to wait -- a step's delay, a wait step, a check or a decode asking to
    /// be run again -- so the gate is let go for the wait and claimed again before the next
    /// transaction (submit_). Nothing is on the wire for this device while it waits, and a
    /// part behind a switch does not see the switch move. Held through the wait, a part polling
    /// a flag every 15 ms on a 100 ms period would keep its channel for most of each period and
    /// starve the parts on the other channels.
    void letGateGo_() {
        if constexpr(Gate::Gated) { gate_.release(); }
    }

    /// The write item on the wire is still owed: a fault says nothing about whether the
    /// bytes landed, so it is written again.
    void redirtyItem_() {
        detail::withIndex(
          group_,
          [&](auto i) {
              auto& s = std::get<decltype(i)::value>(writes_);
              s.dirty.fetch_or(1U << item_, std::memory_order_relaxed);
          },
          std::make_index_sequence<Writes::size>{});
    }

    void beginInit_(TimePoint now) { detail::Engine<I2c, Clock>::beginInit(*this, EngineOps, now); }

    [[nodiscard]] std::span<Step const> script_() const {
        switch(running_) {
        case Running::init: return InitSteps;
        case Running::read: return readSteps_(group_);
        case Running::write:
            // One item may be several transactions: the window command, then the display
            // RAM behind it. On a chip whose writes are one transaction each the script is
            // `current_` itself and there is no store to walk.
            if constexpr(MultiStepWrites) {
                return std::span<Step const>{writeScript_.steps.data(), writeScript_.count};
            } else {
                return std::span<Step const>{&current_, 1};
            }
        case Running::verify: return std::span<Step const>{&current_, 1};
        case Running::none:   break;
        }
        return {};
    }

    [[nodiscard]] std::span<std::byte> buffer_() {
        switch(running_) {
        case Running::init:  return std::span<std::byte>{init_};
        case Running::read:  return readBuffer_(group_);
        case Running::write: return writeBuffer_(group_);
        case Running::verify:
            if constexpr(AnyVerify) {
                return std::span<std::byte>{verify_.got};
            } else {
                break;
            }
        case Running::none: break;
        }
        return {};
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

    void startStep_(TimePoint now) { detail::Engine<I2c, Clock>::startStep(*this, EngineOps, now); }

    /// A group whose prepare() returned a count reads that many bytes instead of the
    /// script's. Such a group has exactly one read step (static_asserted), so the count is
    /// simply what was asked for.
    void sizeRead_() {
        detail::withIndex(
          group_,
          [&](auto i) {
              using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
              if constexpr(detail::HasSizedPrepare<G>) {
                  this->current_.count = std::get<decltype(i)::value>(reads_).len;
              }
          },
          std::make_index_sequence<Reads::size>{});
    }

    /// A counted read of the running group reads `n` bytes: decode sees up to there.
    void countRead_(std::uint8_t n) {
        detail::withIndex(
          group_,
          [&](auto i) {
              auto& s      = std::get<decltype(i)::value>(reads_);
              s.len        = n;
              s.validBytes = static_cast<std::uint8_t>(current_.offset + n);
          },
          std::make_index_sequence<Reads::size>{});
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

    /// Run the read group again from its first step after `retryAfter`, or give up on this
    /// run once the retry limit is reached: the sample is rejected.

    /// Puts one transaction on the bus. False when it could not go (the gate said not yet,
    /// the device is parked with no probe due, the bus queue is full): the step is tried
    /// again next turn.
    /// One transaction on the bus (Engine.hpp): false when it could not go and the step is
    /// tried again next turn.
    bool submit_(Step const&          s,
                 std::span<std::byte> buf) {
        return detail::Engine<I2c, Clock>::submit(*this, EngineOps, s, buf);
    }

    /// The step's delay, then the next step.
    /// The step's delay, then the next step.
    void afterTransaction_(TimePoint now) {
        detail::Engine<I2c, Clock>::afterTransaction(*this, EngineOps, now);
    }

    /// When a wait of `delay` from `now` is over. `now` was read at some point inside a tick of
    /// the clock, so one tick more makes the wait at least `delay` long however late in its
    /// tick that was: on a millisecond clock a 1 ms delay would otherwise end on the very next
    /// tick, which can be microseconds away. A zero delay stays zero.
    [[nodiscard]] static TimePoint atLeast_(TimePoint                 now,
                                            std::chrono::milliseconds delay) {
        if(delay == std::chrono::milliseconds::zero()) { return now; }
        return now + delay + Duration{1};
    }

    void nextStep_(TimePoint now) { detail::Engine<I2c, Clock>::nextStep(*this, EngineOps, now); }

    // -- what happens at the end of a script -----------------------------------------------

    /// The identity registers the bring-up has just read, against the oracle's values under
    /// their masks.
    [[nodiscard]] bool identityMatches_() {
        if constexpr(HasIdentity) {
            std::size_t offset = IdentityOffset;
            for(std::size_t i = 0; i < Chip::Identity.size(); ++i) {
                auto const& check = Chip::Identity[i];
                identity_[i]      = check.value(init_.data() + offset);
                offset += check.width;
            }
            for(std::size_t i = 0; i < Chip::Identity.size(); ++i) {
                auto const& check = Chip::Identity[i];
                if(!check.matches(identity_[i])) {
                    if(!unidentifiedLogged_) {
                        detail::logIdentityMismatch(Chip::Name,
                                                    Address,
                                                    check.name,
                                                    static_cast<std::uint16_t>(check.reg),
                                                    identity_[i],
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
                Chip::identified(std::span<std::uint32_t const>{identity_}, state_);
            }
        }
        return true;
    }

    void finishInit_(TimePoint now) {
        detail::Engine<I2c, Clock>::finishInit(*this, EngineOps, now);
    }

    /// Every group's first deadline after a bring-up, and what the chip is owed again.
    void startupSchedule_(TimePoint now) {
        detail::forEach<Reads>([&](auto i) {
            using G   = typename detail::Nth<Reads>::template type<decltype(i)::value>;
            auto& s   = std::get<decltype(i)::value>(reads_);
            s.retries = 0;
            if constexpr(detail::periodOf<G>() > std::chrono::milliseconds::zero()) {
                s.due = s.periodSet && s.period == std::chrono::milliseconds::zero()
                        ? TimePoint::max()
                        : now;
            } else {
                s.due = now;
            }
        });
        detail::forEach<Writes>([&](auto i) {
            using W = typename detail::Nth<Writes>::template type<decltype(i)::value>;
            auto& s = std::get<decltype(i)::value>(writes_);
            s.due   = now;
            if constexpr(detail::HasInitial<W>) {
                // A value the chip must hold from the start (and again after every reset)
                // unless the application has set one already -- set, not written: the
                // application's value stands even when the bring-up it was made before is
                // the one finishing here. Per item: an application that set item 3 of a group
                // of 16 still owes the other 15 their Initial, not a default-constructed Value.
                for(std::size_t item = 0; item < WriteSlot<W>::Items; ++item) {
                    if(((s.known >> item) & 1U) == 0) { s.values[item] = W::Initial; }
                }
                s.known = WriteSlot<W>::AllItems;
                s.dirty.store(WriteSlot<W>::AllItems, std::memory_order_release);
            } else if constexpr(!detail::HasTransient<W>) {
                // The chip is back at its defaults, so whatever the application has set
                // since is owed again -- without this the value is lost silently while
                // value<W>() still reports it. Only the items it did set (`known`): an item
                // it never touched holds a default-constructed Value the chip was never
                // told, and must not be told now. A Transient group is a one-shot command
                // and not a state: replaying it would burn an EEPROM write cycle or
                // re-apply a stale wall-clock time.
                if(s.owned) { s.dirty.store(s.known, std::memory_order_release); }
            }
        });
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
    auto decode_(S const& slot) const {
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

    /// The next deadline of a periodic group, when this run was the periodic one; a run
    /// that was only requested leaves the period alone.
    ///
    /// A group with a period(sample) is measured from this completion instead, because what
    /// it is asking for is "so long after the last answer", and the period it asks for
    /// depends on that answer. A period of 0 parks the group until request<G>() asks for it.

    /// The period group G runs at now: what period<G>(ms) set, else what `G::period(State
    /// const&)` makes of the chip State when the group has one -- a rate written to the part
    /// at run time (IIS2DULPX's CTRL5) is then the rate it is read at from the next sample on
    /// -- else its constant `Period`.
    template<typename G>
    [[nodiscard]] std::chrono::milliseconds periodNow_() const {
        auto const& s = std::get<ReadSlot<G>>(reads_);
        if(s.periodSet) { return s.period; }
        if constexpr(!detail::HasDynamicPeriod<G>
                     && requires(State const& st) {
                            { G::period(st) } -> std::convertible_to<std::chrono::milliseconds>;
                        })
        {
            std::chrono::milliseconds const p = G::period(state_);
            return p > std::chrono::milliseconds::zero() ? p : detail::periodOf<G>();
        } else {
            return detail::periodOf<G>();
        }
    }

    /// The next deadline one period on, or one period from now when the last one is already
    /// past: a group that fell behind resumes at its period rather than bursting to catch up.

    /// The read-back came in: compare it against what was written. A mismatch re-dirties
    /// the item, bounded by MaxRetries so a bit that cannot hold its value is not rewritten
    /// for ever.
    void finishVerify_(TimePoint now) {
        if constexpr(AnyVerify) {
            detail::withIndex(
              group_,
              [&](auto i) {
                  using W = typename detail::Nth<Writes>::template type<decltype(i)::value>;
                  if constexpr(detail::Verifies<W>) {
                      auto&       s = std::get<decltype(i)::value>(writes_);
                      auto const  n = static_cast<std::size_t>(verify_.len);
                      Bytes const want{std::span<std::byte const>{verify_.want}.first(n)};
                      Bytes const got{std::span<std::byte const>{verify_.got}.first(n)};
                      bool const  ok = [&] {
                          if constexpr(detail::HasVerifyFn<W>) {
                              return W::verify(want, got);
                          } else {
                              return std::ranges::equal(std::span{verify_.want}.first(n),
                                                        std::span{verify_.got}.first(n));
                          }
                      }();
                      if(ok) {
                          s.verify.unverified.fetch_and(~(1U << this->item_),
                                                        std::memory_order_relaxed);
                          s.verify.tries = 0;
                      } else {
                          ++s.verify.mismatches;
                          if(s.verify.tries < MaxRetries) {
                              ++s.verify.tries;
                              s.dirty.fetch_or(1U << this->item_, std::memory_order_relaxed);
                          } else {
                              detail::logVerifyStuck(
                                Chip::Name,
                                Address,
                                static_cast<std::uint16_t>(verify_.wrote.reg),
                                static_cast<std::uint32_t>(s.verify.mismatches));
                              s.verify.unverified.fetch_and(~(1U << this->item_),
                                                            std::memory_order_relaxed);
                              s.verify.tries = 0;
                          }
                      }
                      constexpr auto iv = detail::verifyIntervalOf<W>();
                      s.verify.due
                        = now
                        + (iv != std::chrono::milliseconds::zero() ? iv
                                                                   : detail::verifyDelayOf<W>());
                  }
              },
              std::make_index_sequence<Writes::size>{});
        }
        endRun_();
    }

    void fail_(TimePoint now) { detail::Engine<I2c, Clock>::fail(*this, EngineOps, now); }

    // -- choosing what runs next -----------------------------------------------------------

    /// Read one written register back. The write step `encode` produced carries both the
    /// register and the bytes, so a chip needs to declare nothing but the two durations.
    bool startVerify_(std::uint8_t w,
                      TimePoint    now) {
        if constexpr(!AnyVerify) {
            static_cast<void>(w);
            static_cast<void>(now);
            return false;
        } else {
            bool started = false;
            detail::withIndex(
              w,
              [&](auto i) {
                  using W = typename detail::Nth<Writes>::template type<decltype(i)::value>;
                  if constexpr(detail::Verifies<W>) {
                      auto& s = std::get<decltype(i)::value>(writes_);
                      if(now < s.verify.due) { return; }
                      auto u = s.verify.unverified.load(std::memory_order_acquire);
                      if(u == 0) {
                          constexpr auto iv = detail::verifyIntervalOf<W>();
                          if constexpr(iv == std::chrono::milliseconds::zero()) {
                              return;   // checked once after each write, and it matched
                          } else {
                              // The periodic look, at the items the chip was told: one never set
                              // holds a Value it was never sent and would read as a mismatch.
                              u = s.known;
                              if(s.writes == 0 || u == 0) {
                                  s.verify.due = now + iv;   // nothing written to check yet
                                  return;
                              }
                              s.verify.unverified.store(u, std::memory_order_release);
                          }
                      }
                      auto const item = static_cast<std::uint32_t>(std::countr_zero(u));

                      // A read-back compares one register against one encoded payload; a
                      // multi-transaction item has no single register to read back from.
                      // groupOk() rejects the combination -- this says why here, at the place
                      // that relies on it.
                      static_assert(detail::writeStepsOf<W>() == 1,
                                    "a write group that asks to be verified must encode to a "
                                    "single Step");

                      // Re-encode to recover the register and the bytes that were sent.
                      Step wrote{};
                      if constexpr(detail::HasItems<W>) {
                          wrote = W::encode(s.values[item],
                                            static_cast<std::size_t>(item),
                                            std::span<std::byte>{s.buf});
                      } else {
                          wrote = W::encode(s.values[0], std::span<std::byte>{s.buf});
                      }
                      if(wrote.count == 0) {
                          // a bare pointer set: there is nothing to read back
                          s.verify.unverified.fetch_and(~(1U << item), std::memory_order_relaxed);
                          return;
                      }
                      auto const n = static_cast<std::size_t>(wrote.count) > MaxVerifyBytes
                                     ? MaxVerifyBytes
                                     : static_cast<std::size_t>(wrote.count);
                      for(std::size_t j = 0; j < n; ++j) {
                          verify_.want[j] = wrote.fromBuffer
                                            ? s.buf[wrote.offset + j]
                                            : static_cast<std::byte>(wrote.bytes[j]);
                      }
                      verify_.wrote = wrote;
                      verify_.len   = static_cast<std::uint8_t>(n);
                      verify_.got   = {};

                      this->running_ = Running::verify;
                      this->group_   = static_cast<std::uint8_t>(decltype(i)::value);
                      this->item_    = static_cast<std::uint8_t>(item);
                      this->step_    = 0;
                      // Behind a register when the write went to one; a bare read of the same
                      // count on a chip without registers (an I2C switch returns its control
                      // byte, 7.5.4 of the TCA9548A).
                      this->current_
                        = wrote.hasRegister
                          ? Step::read({.reg    = wrote.reg,
                                        .count  = static_cast<std::uint8_t>(n),
                                        .offset = 0})
                          : Step::receive({.count = static_cast<std::uint8_t>(n), .offset = 0});
                      this->inFlight_ = submit_(this->current_, buffer_());
                      started         = true;
                  }
              },
              std::make_index_sequence<Writes::size>{});
            return started;
        }
    }

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

    /// The due read group with the earliest deadline, a requested one first.
    /// The group's prepare() and the lengths this run works with; the engine picks the group
    /// and does the rest (Engine::startRead).
    void prepareRead_(std::uint8_t g) {
        detail::withIndex(
          g,
          [&](auto i) {
              using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
              auto& s = std::get<decltype(i)::value>(reads_);
              if constexpr(detail::HasSizedPrepare<G>) {
                  constexpr auto off = detail::firstReadOffset(std::span<Step const>{G::Steps});
                  constexpr auto max = ReadSlot<G>::Bytes - off;
                  auto const     n   = G::prepare(s.request, std::span<std::byte>{s.buf});
                  if(n > max) { detail::logOversizedRead(Chip::Name, Address, n, max); }
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

    // Where the run is, what it waits for and what has happened to it: detail::EngineState,
    // the base of this class (Engine.hpp), because none of it depends on the chip.
    PresenceT                                             presence_{};
    State                                                 state_{};
    [[no_unique_address]] Gate                            gate_{};
    std::array<std::byte, RegisterBytes + MaxPayload + 1> tx_{};
    std::array<std::byte, InitBytes>                      init_{};
    bool                                                  identityMatched_{!HasIdentity};
    static constexpr std::size_t                          IdentityCount = [] {
        if constexpr(HasIdentity) {
            return Chip::Identity.size();
        } else {
            return std::size_t{1};
        }
    }();
    std::array<std::uint32_t, IdentityCount>                                  identity_{};
    [[no_unique_address]] detail::VerifyBufs<AnyVerify, MaxVerifyBytes>       verify_{};
    [[no_unique_address]] detail::WriteScript<MultiStepWrites, MaxWriteSteps> writeScript_{};
    [[no_unique_address]] detail::WakeSlot<(WakeRetries > 0)>                 wake_{};
    [[no_unique_address]] detail::BridgeSlot<Bridged>                         bridge_{};
    ReadSlots                                                                 reads_{};
    WriteSlots                                                                writes_{};
};

}   // namespace Kvasir::I2C
