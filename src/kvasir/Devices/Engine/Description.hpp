#pragma once
// The description vocabulary of the device engine: what a chip description (chips/) and a catalogue are written in and
// judged by - List, Outcome, DefaultTiming, the ReadGroup / WriteGroup concepts and the traits the engine and the
// checks read off a description (detail::Has*, *Of, EffectiveInit, Verifies, PrimaryOf, ...). No engine: a chip
// includes this, never Device.hpp (ctest i2c_chip_headers_lean). What a Device's Config names is DeviceConfig.hpp.
#include "../Bytes.hpp"
#include "../Duration.hpp"
#include "RegisterCheck.hpp"
#include "Step.hpp"

#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>

namespace Kvasir::Engine {

template<typename... Ts>
struct List {
    static constexpr std::size_t size = sizeof...(Ts);
};

/// What a decode made of the bytes (its DecodeKind, Step.hpp), with the sample or the time a retry waits.
template<typename S>
struct Outcome {
    DecodeKind                kind{};
    S                         value{};
    std::chrono::milliseconds retryAfter{};

    static constexpr Outcome ok(S value) { return {.kind = DecodeKind::ok, .value = value}; }

    static constexpr Outcome reject() { return {.kind = DecodeKind::reject}; }

    static constexpr Outcome retry(std::chrono::milliseconds after) {
        return {.kind = DecodeKind::retry, .retryAfter = after};
    }

    /// A well-formed frame that says nothing new (a controller's "no new data" bit, or a
    /// reading the application has already been told about): the sample stands, seq does
    /// not step, and it is counted by unchanged<G>() rather than as a rejection. A reader
    /// that watches seq is not woken by an idle chip polled ten times a second.
    static constexpr Outcome unchanged() { return {.kind = DecodeKind::unchanged}; }
};

struct Empty {};

/// No timing: a description's own defaults. A description that takes a `Timing` type reads the
/// std::chrono members it names (`Period`, `StartupDelay`, ...) from it and falls back to its
/// own figure for any that is missing -- a duration is not a structural type, so it cannot be
/// a template parameter itself:
///     struct Fast { static constexpr std::chrono::milliseconds Period{20}; };
///     using Meter = Chips::Ina219<Units::microOhm(100000), Units::microAmp(100), Fast>;
struct DefaultTiming {};

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

    /// A cyclic read group whose next deadline follows the reading: a touch controller
    /// polled fast while a finger is down and slowly when the glass is idle. It still
    /// declares Period -- the compile-time "this group is cyclic" -- and periodNow(sample) is
    /// what is actually waited; 0 means "not again until request<G>() asks".
    template<typename G>
    concept HasDynamicPeriod = requires(typename G::Sample const& s) {
        { G::periodNow(s) } -> std::convertible_to<std::chrono::milliseconds>;
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

}   // namespace detail

/// A read group's period (`static constexpr auto Period`), 0 ms for a group that has none (read on demand).
template<typename G>
constexpr std::chrono::milliseconds periodOf() {
    if constexpr(detail::HasPeriod<G>) {
        return Kvasir::asDuration(G::Period);
    } else {
        return std::chrono::milliseconds{0};
    }
}

namespace detail {

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

    /// L with T at its end, unless L holds T already.
    template<typename L, typename T>
    struct Append;

    template<typename... Ts, typename T>
    struct Append<List<Ts...>, T> {
        using type
          = std::conditional_t<(std::is_same_v<T, Ts> || ...), List<Ts...>, List<Ts..., T>>;
    };

    /// L's types once each, in order of first appearance.
    template<typename L, typename Seen = List<>>
    struct Unique {
        using type = Seen;
    };

    template<typename T, typename... Ts, typename Seen>
    struct Unique<List<T, Ts...>, Seen> : Unique<List<Ts...>, typename Append<Seen, T>::type> {};

    /// The lists one after the other.
    template<typename... Ls>
    struct Concat {
        using type = List<>;
    };

    template<typename... As>
    struct Concat<List<As...>> {
        using type = List<As...>;
    };

    template<typename... As, typename... Bs, typename... Ls>
    struct Concat<List<As...>, List<Bs...>, Ls...> : Concat<List<As..., Bs...>, Ls...> {};

    template<typename G,
             typename... Gs>
    constexpr bool inList(List<Gs...>*) {
        return (std::is_same_v<G, Gs> || ...);
    }

    template<typename G,
             typename Config>
    constexpr bool confirmListed() {
        if constexpr(requires { typename Config::Confirm; }) {
            return inList<G>(static_cast<typename Config::Confirm*>(nullptr));
        } else {
            return false;
        }
    }

    /// A group's ConfirmChange (see the file comment): its own, 0 included, else the Config's
    /// for the groups it names. An I2C read cannot tell a good byte from a bad one: a part
    /// losing its supply mid-read gives 0xFF, one clamping SDA 0x00, and the transfer succeeds.
    ///     struct Config {
    ///         using Confirm = List<Chips::Tca9555::Pins>;
    ///         static constexpr std::uint8_t confirmChange = 3;
    ///     };
    /// Named, not every group: a FIFO, a stream or a clear-on-read status cannot be read twice.
    template<typename G,
             typename Config>
    constexpr std::uint8_t confirmChangeOf() {
        if constexpr(HasSample<G> && requires { G::ConfirmChange; }) {
            return static_cast<std::uint8_t>(G::ConfirmChange);
        } else if constexpr(HasSample<G> && requires { Config::confirmChange; }) {
            return confirmListed<G, Config>() ? static_cast<std::uint8_t>(Config::confirmChange)
                                              : std::uint8_t{0};
        } else {
            return 0;
        }
    }

    /// What is wrong with a Config's ConfirmChange for a part with read groups `Reads`
    /// (Device static_asserts `ok`; a function so a test can ask it for a bad Config).
    enum class ConfirmConfig : std::uint8_t {
        ok,
        unnamed,     ///< ConfirmChange without a Confirm list
        uncounted,   ///< a Confirm list without ConfirmChange
        foreign,     ///< Confirm names a group that is not one of the part's reads
        counted,     ///< a confirming group has a counted read: it cannot be read twice
    };

    template<typename Config,
             typename Reads>
    constexpr ConfirmConfig checkConfirmConfig() {
        constexpr bool hasCount = requires { Config::confirmChange; };
        constexpr bool hasList  = requires { typename Config::Confirm; };
        if constexpr(hasCount && !hasList) {
            return ConfirmConfig::unnamed;
        } else if constexpr(hasList && !hasCount) {
            return ConfirmConfig::uncounted;
        } else {
            if constexpr(hasList) {
                bool const allReads = []<typename... Cs>(List<Cs...>*) {
                    return (inList<Cs>(static_cast<Reads*>(nullptr)) && ...);
                }(static_cast<typename Config::Confirm*>(nullptr));
                if(!allReads) { return ConfirmConfig::foreign; }
            }
            bool counted = false;
            forEach<Reads>([&](auto i) {
                using G = typename Nth<Reads>::template type<decltype(i)::value>;
                if constexpr(confirmChangeOf<G, Config>() != 0) {
                    counted = counted || hasCounted(std::span<Step const>{G::Steps});
                }
            });
            return counted ? ConfirmConfig::counted : ConfirmConfig::ok;
        }
    }

    /// Every read group's ConfirmChange for a Config, in the order of the chip's Reads.
    template<typename ChipT,
             typename Config>
    constexpr auto confirmsOf() {
        using Rs = typename ReadsOf<ChipT>::type;
        std::array<std::uint8_t, Rs::size> counts{};
        forEach<Rs>([&](auto i) {
            counts[decltype(i)::value]
              = confirmChangeOf<typename Nth<Rs>::template type<decltype(i)::value>, Config>();
        });
        return counts;
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

    template<typename L>
    constexpr bool anyInitial() {
        bool any = false;
        forEach<L>([&](auto i) {
            any = any || HasInitial<typename Nth<L>::template type<decltype(i)::value>>;
        });
        return any;
    }

}   // namespace detail

}   // namespace Kvasir::Engine
