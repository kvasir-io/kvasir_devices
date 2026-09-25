#pragma once

#include "Bridge.hpp"
#include "Device.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

/// The devices on one I2C bus, as one object.
///
/// A `Device` is independent; a `Bus` owns the *set* of them, because three facts are
/// properties of the set and of nothing else: that no two devices answer at the same address,
/// that the bus queue is deep enough for them all, and how much of the bus their periodic
/// traffic uses. All three are decided at compile time from data the descriptions carry.
///
///     using Sensors = Kvasir::I2C::Bus<I2c1, Clock, Bh1750, Veml6030, Bme280, Sht3x>;
///     ...
///     Sensors bus{};                        // a local in main(), or a member
///     bus.handler();                        // every device, once per loop turn
///     bus.get<Bh1750>().latest();           // by type, or get<0>() by index
///     bus.logHealth();                      // one line per device
///     static_assert(Sensors::busLoad() < 0.8);
///
/// It owns the devices by value and needs no static storage of its own -- a member of an
/// application object on main()'s stack is the usual place, and `constinit` at namespace
/// scope is for what genuinely needs to be found by name (an interrupt, a second core, a
/// reference template argument). `get<>()` is a `std::get` on a tuple member: it inlines
/// away, which matters because an interrupt handler calls through it.
namespace Kvasir::I2C {

namespace detail {

    /// Bit times one transaction spends on the wire: a START and a STOP, the address byte,
    /// and every payload byte, each of them nine bits because of the acknowledge. A read
    /// behind a register is addressed twice -- the register is written, then a repeated
    /// START turns the bus around -- which is where the second address byte comes from.
    /// A counted read (Step::readCounted) is reckoned at its most, which its `count` holds:
    /// the load is a ceiling, and a skipped count-0 read costs less than it says.
    [[nodiscard]] constexpr std::uint32_t stepBits(Step const& s,
                                                   std::size_t registerBytes) {
        if(!s.isTransaction()) { return 0; }
        std::uint32_t bytes = 1;   // the address
        if(s.hasRegister) {
            bytes += static_cast<std::uint32_t>(registerBytes);
            if(s.kind == Step::Kind::read) { ++bytes; }   // the repeated START's address
        }
        bytes += s.count;
        return bytes * 9U + 2U;   // START and STOP
    }

    constexpr std::uint32_t scriptBits(std::span<Step const> steps,
                                       std::size_t           registerBytes) {
        std::uint32_t n = 0;
        for(auto const& s : steps) { n += stepBits(s, registerBytes); }
        return n;
    }

    /// A bus type that says what it is clocked at, so a load can be a fraction rather than
    /// a bit rate. The chip packages export it beside QueueDepth; a fake bus need not.
    template<typename I2c>
    concept HasBaudRate = requires {
        { I2c::BaudRate } -> std::convertible_to<std::uint32_t>;
    };

    template<typename I2c>
    concept HasQueueDepth = requires {
        { I2c::QueueDepth } -> std::convertible_to<std::size_t>;
    };

    /// Bits per second one device's *cyclic* traffic puts on the wire: every read group with
    /// a Period, and every write group that is rewritten on one. A group whose period follows
    /// its sample (a touch controller) is counted at its constant `Period`, which the
    /// description sets to the slowest of the two, so the figure is the idle rate and the
    /// header says so.
    ///
    /// On-demand reads and application writes are not counted: they are not periodic, and a
    /// budget that guessed at them would be a worse number than one that says what it covers.
    template<typename D>
    [[nodiscard]] constexpr double cyclicBitsPerSecond() {
        double total = 0.0;
        forEach<typename D::Reads>([&](auto i) {
            using G = typename Nth<typename D::Reads>::template type<decltype(i)::value>;
            constexpr auto period = periodOf<G>();
            if constexpr(period > std::chrono::milliseconds::zero()) {
                total += static_cast<double>(
                           scriptBits(std::span<Step const>{G::Steps}, D::RegisterBytes))
                       / std::chrono::duration<double>{period}.count();
            }
        });
        forEach<typename D::Writes>([&](auto i) {
            using W = typename Nth<typename D::Writes>::template type<decltype(i)::value>;
            constexpr auto period = periodOf<W>();
            if constexpr(period > std::chrono::milliseconds::zero()) {
                // One item's worth: address, register, payload. A write group's Steps are
                // produced by encode() at run time, so this is the description's own Bytes.
                auto const bytes = 1U + static_cast<std::uint32_t>(D::RegisterBytes)
                                 + static_cast<std::uint32_t>(W::Bytes);
                total += static_cast<double>(bytes * 9U + 2U) * static_cast<double>(itemsOf<W>())
                       / std::chrono::duration<double>{period}.count();
            }
        });
        return total;
    }

    /// A gate that puts its device on a switch (Mux.hpp): behind one of its channels, or in
    /// front of it with the switch closed. Not the same as `Gated`, which only says that the
    /// engine has to ask: a BridgeGate around no switch is gated and on none.
    template<typename G>
    concept OnSwitch = requires { typename G::Mux; };

    /// On a switch and behind one of its channels: not a gate that claims the switch closed
    /// (`Front`, a MuxFrontGate), whose device is in front of it.
    template<typename G>
    concept BehindChannel = OnSwitch<G> && !requires { requires static_cast<bool>(G::Front); };

    /// The gate a device has towards the switches, whatever bridge is around it (Bridge.hpp).
    template<typename G>
    struct SwitchGateOf {
        using type = G;
    };

    template<BridgedGate G>
    struct SwitchGateOf<G> {
        using type = typename G::InnerT;
    };

    /// The bridge a gate is behind, or void.
    template<typename G>
    struct BridgeOfGate {
        using type = void;
    };

    template<BridgedGate G>
    struct BridgeOfGate<G> {
        using type = typename G::Bridge;
    };

    /// Two bridges of which at most one is ever active: different ones naming the same
    /// ExclusiveGroup (Bridge.hpp).
    template<typename X,
             typename Y>
    [[nodiscard]] constexpr bool exclusiveBridges() {
        if constexpr(std::is_void_v<X> || std::is_void_v<Y> || std::is_same_v<X, Y>) {
            return false;
        } else if constexpr(requires {
                                typename X::ExclusiveGroup;
                                typename Y::ExclusiveGroup;
                            })
        {
            return std::is_same_v<typename X::ExclusiveGroup, typename Y::ExclusiveGroup>;
        } else {
            return false;
        }
    }

    /// A bridge whose description takes the separation of addresses on itself.
    template<typename X>
    [[nodiscard]] constexpr bool saysSeparate() {
        if constexpr(std::is_void_v<X>) {
            return false;
        } else {
            return bridgeSeparates<X>;
        }
    }

    /// Two devices clash when they answer at the same address and can be on the wire at the
    /// same time. What keeps two apart is
    ///   - a switch: both behind one, on different gates. The gate is compared as a *type*:
    ///     `MuxGate<MuxDev, 0>` and `MuxGate<MuxDev, 3>` are different types, so the same part
    ///     on two channels of a switch is legal -- which is why a gate carries its channel as a
    ///     template parameter and its switch as a member (Mux.hpp). A part in front of the
    ///     switches (no gate, or a MuxFrontGate) is on every channel's wire, so nothing keeps
    ///     it apart from anybody;
    ///   - two bridges of one ExclusiveGroup (Bridge.hpp). A bridge alone does not: a part in
    ///     front of it and one behind it meet whenever it is active;
    ///   - the application's word, between two bridges: one that says
    ///     `BridgeAddresses::separate` keeps its parts apart from the parts behind another.
    /// Nothing keeps a part that is behind no bridge and no channel apart from anybody: it is
    /// on the wire all the time, so it answers with the part behind the bridge whenever that
    /// one is addressed -- whatever gate it has (a BridgeFrontGate or a MuxFrontGate decides
    /// when *it* talks, not when it listens).
    template<typename A,
             typename B>
    [[nodiscard]] constexpr bool clash() {
        if(A::Address != B::Address) { return false; }
        using GA = typename SwitchGateOf<typename A::GateT>::type;
        using GB = typename SwitchGateOf<typename B::GateT>::type;
        if(BehindChannel<GA> && BehindChannel<GB> && !std::is_same_v<GA, GB>) { return false; }
        using BA = typename BridgeOfGate<typename A::GateT>::type;
        using BB = typename BridgeOfGate<typename B::GateT>::type;
        if(exclusiveBridges<BA, BB>()) { return false; }
        return std::is_void_v<BA> || std::is_void_v<BB> || std::is_same_v<BA, BB>
            || !(saysSeparate<BA>() || saysSeparate<BB>());
    }

    /// No two of these devices clash. A free function rather than only a member, because
    /// instantiating a colliding `Bus` is a hard error by design and so leaves nothing to
    /// assert on; this can be called on its own and asked.
    template<typename... Ds>
    [[nodiscard]] constexpr bool addressesDistinct() {
        using L = List<Ds...>;
        bool ok = true;
        forEach<L>([&](auto i) {
            forEach<L>([&](auto j) {
                if constexpr(decltype(i)::value < decltype(j)::value) {
                    if(clash<typename Nth<L>::template type<decltype(i)::value>,
                             typename Nth<L>::template type<decltype(j)::value>>())
                    {
                        ok = false;
                    }
                }
            });
        });
        return ok;
    }

    /// Samples a second D's cyclic read groups ask for: each group's constant Period, the
    /// figure bitsPerSecond() is built on. A group that paces itself from its sample, or runs
    /// again on a stale result, is counted at its constant one, so 100 % is a ceiling.
    template<typename D>
    [[nodiscard]] constexpr double nominalSamplesPerSecond() {
        double rate = 0.0;
        forEach<typename D::Reads>([&](auto i) {
            using G = typename Nth<typename D::Reads>::template type<decltype(i)::value>;
            if constexpr(HasSample<G>) {
                constexpr auto period = periodOf<G>();
                if constexpr(period > std::chrono::milliseconds::zero()) {
                    rate += 1.0 / std::chrono::duration<double>{period}.count();
                }
            }
        });
        return rate;
    }

    /// Writes a second D's periodic write groups make, every item of them.
    template<typename D>
    [[nodiscard]] constexpr double nominalWritesPerSecond() {
        double rate = 0.0;
        forEach<typename D::Writes>([&](auto i) {
            using W = typename Nth<typename D::Writes>::template type<decltype(i)::value>;
            constexpr auto period = periodOf<W>();
            if constexpr(period > std::chrono::milliseconds::zero()) {
                rate += static_cast<double>(itemsOf<W>())
                      / std::chrono::duration<double>{period}.count();
            }
        });
        return rate;
    }

    /// A gate that claims its switch *closed*: the device is in front of the switch, on segment
    /// 0, and only its transactions wait for the switch (Mux.hpp, MuxFrontGate). A gate says so
    /// with `static constexpr bool Front = true`; one that says nothing is behind its switch.
    template<typename G>
    inline constexpr bool gateInFront = [] {
        if constexpr(requires { G::Front; }) {
            return static_cast<bool>(G::Front);
        } else {
            return false;
        }
    }();

    /// Which switch each gated device is behind, numbered in the order the list first reaches
    /// a device behind it, and how many switches there are.
    template<std::size_t N>
    struct SwitchMap {
        std::array<std::uint8_t, N> ordinal{};
        std::size_t                 switches{};
    };

    template<typename... Ds>
    [[nodiscard]] constexpr SwitchMap<sizeof...(Ds)> switchMap() {
        using L                      = List<Ds...>;
        constexpr std::size_t      N = sizeof...(Ds);
        std::array<std::size_t, N> firstOnSwitch{};   // the first device behind the same switch
        std::array<bool, N>        gated{};
        forEach<L>([&](auto i) {
            using A              = typename Nth<L>::template type<decltype(i)::value>;
            auto const index     = decltype(i)::value;
            firstOnSwitch[index] = index;
            if constexpr(OnSwitch<typename A::GateT>) {
                gated[index] = true;
                bool found   = false;
                forEach<L>([&](auto j) {
                    using B = typename Nth<L>::template type<decltype(j)::value>;
                    if constexpr(OnSwitch<typename B::GateT>) {
                        if constexpr(std::is_same_v<typename A::GateT::Mux, typename B::GateT::Mux>)
                        {
                            if(!found) {
                                firstOnSwitch[index] = decltype(j)::value;
                                found                = true;
                            }
                        }
                    }
                });
            }
        });
        SwitchMap<N>                map{};
        std::array<std::uint8_t, N> ordinalOfFirst{};
        for(std::size_t k = 0; k < N; ++k) {
            if(gated[k] && firstOnSwitch[k] == k) {
                ordinalOfFirst[k] = static_cast<std::uint8_t>(map.switches++);
            }
        }
        for(std::size_t k = 0; k < N; ++k) {
            if(gated[k]) { map.ordinal[k] = ordinalOfFirst[firstOnSwitch[k]]; }
        }
        return map;
    }

    /// The segment of every device (Bus::segmentOf).
    template<typename... Ds>
    [[nodiscard]] constexpr std::array<std::uint8_t,
                                       sizeof...(Ds)>
    segmentTable() {
        using L                                     = List<Ds...>;
        auto const                              map = switchMap<Ds...>();
        std::array<std::uint8_t, sizeof...(Ds)> segments{};
        forEach<L>([&](auto i) {
            using A = typename Nth<L>::template type<decltype(i)::value>;
            // A MuxFrontGate is gated and in front of its switch: segment 0, like an ungated part.
            if constexpr(BehindChannel<typename A::GateT>) {
                auto const channel = static_cast<unsigned>(std::countr_zero(A::GateT::Mask));
                segments[decltype(i)::value]
                  = static_cast<std::uint8_t>(1U + 8U * map.ordinal[decltype(i)::value] + channel);
            }
        });
        return segments;
    }

    /// The switch the gated devices of a Bus are behind, and its arbiter; void for both on a Bus
    /// without one. The first gated device says: on a Bus with more switches, the first.
    template<typename D>
    struct SwitchOfGated {
        using device  = typename D::GateT::Mux;
        using arbiter = typename D::GateT::Arbiter;
    };

    template<typename... Ds>
    struct SwitchOf {
        using device  = void;
        using arbiter = void;
    };

    template<typename D, typename... Ds>
    struct SwitchOf<D, Ds...>
      : std::conditional_t<OnSwitch<typename D::GateT>, SwitchOfGated<D>, SwitchOf<Ds...>> {};

    /// The index of the first device on switch K, numbered as Bus::Segments numbers them: the
    /// one whose gate names the switch and its arbiter. By the switch map and not by the segment,
    /// because a MuxFrontGate's device is on segment 0 and may be the only one on its switch.
    template<std::size_t K,
             typename... Ds>
    [[nodiscard]] constexpr std::size_t firstBehindSwitch() {
        using L           = List<Ds...>;
        auto const  map   = switchMap<Ds...>();
        std::size_t first = sizeof...(Ds);
        forEach<L>([&](auto i) {
            using A = typename Nth<L>::template type<decltype(i)::value>;
            if constexpr(OnSwitch<typename A::GateT>) {
                if(first == sizeof...(Ds) && map.ordinal[decltype(i)::value] == K) {
                    first = decltype(i)::value;
                }
            }
        });
        return first;
    }

    /// The lines of the bridges the devices are behind (Bridge.hpp), each bridge once, in the
    /// order the list first reaches a device behind it: a std::tuple of BridgeLine<Clock, B>.
    template<typename Clock, typename Tuple, typename... Ds>
    struct BridgeLines {
        using type = Tuple;
    };

    template<typename Clock, typename... Ls, typename D, typename... Ds>
    struct BridgeLines<Clock, std::tuple<Ls...>, D, Ds...> {
        using B    = typename BridgeOfGate<typename D::GateT>::type;
        using type = typename std::conditional_t<
          std::is_void_v<B> || (std::is_same_v<BridgeLine<Clock, B>, Ls> || ...),
          BridgeLines<Clock, std::tuple<Ls...>, Ds...>,
          BridgeLines<Clock, std::tuple<Ls..., BridgeLine<Clock, B>>, Ds...>>::type;
    };

    /// Whether a queue of `depth` requests fits the bus's own. A bus type that does not say how
    /// deep its queue is passes, there being nothing to check against. A function rather than an
    /// `||` in the static_assert: `I2c::QueueDepth` in the right-hand operand is looked up even
    /// when the left one is true, which fails to compile for exactly such a bus.
    template<typename I2c>
    constexpr bool queueFits(std::size_t depth) {
        if constexpr(HasQueueDepth<I2c>) { return depth <= I2c::QueueDepth; }
        return true;
    }

}   // namespace detail

template<typename I2c, typename Clock, typename... Ds>
class Bus {
public:
    using Devices = List<Ds...>;
    using I2cBus  = I2c;
    using ClockT  = Clock;

    /// The switch the gated devices are behind and the arbiter they share (Mux.hpp), or void.
    using SwitchDevice  = typename detail::SwitchOf<Ds...>::device;
    using SwitchArbiter = typename detail::SwitchOf<Ds...>::arbiter;

    /// Switch K of Switches and its arbiter, K numbered as Segments numbers the switches.
    template<std::size_t K>
    using SwitchDeviceAt = typename detail::Nth<Devices>::template type<
      detail::firstBehindSwitch<K, Ds...>()>::GateT::Mux;
    template<std::size_t K>
    using SwitchArbiterAt = typename detail::Nth<Devices>::template type<
      detail::firstBehindSwitch<K, Ds...>()>::GateT::Arbiter;

    static constexpr std::size_t Count = sizeof...(Ds);

    /// Whether a device is one of the Bus's parts: anything but a Broadcast (Concepts.hpp),
    /// which is written on demand and has nothing to answer until then.
    template<typename D>
    [[nodiscard]] static constexpr bool isPart() {
        return !Broadcast<typename D::Chip>;
    }

    /// The parts: the devices less any Broadcast. What answeringCount() is out of.
    static constexpr std::size_t Parts = (std::size_t{!Broadcast<typename Ds::Chip>} + ... + 0U);

    static_assert(Count > 0,
                  "a Bus with no devices on it");

    static_assert((std::is_same_v<I2c,
                                  typename Ds::I2cBus>
                   && ...),
                  "a device on this Bus is on a different I2C bus than the one named: the "
                  "queue depth and bus load below would be for traffic that is not on one wire");

    static_assert((std::is_same_v<Clock,
                                  typename Ds::ClockT>
                   && ...),
                  "a device on this Bus is on a different Clock than the one named");

    // -- what is true of the set and of nothing else ---------------------------------------

    /// No two devices answer at the same address. Two that did would take each other's
    /// answers and neither would be obviously wrong -- the engine schedules them
    /// independently, so the symptom is a sensor reading another sensor's registers.
    [[nodiscard]] static constexpr bool addressesDistinct() {
        return detail::addressesDistinct<Ds...>();
    }

    static_assert(addressesDistinct(),
                  "two devices on this bus have the same address and are on the same gate, so "
                  "they would take each other's answers. Check the address straps, use a "
                  "different Config::Address, or put them on different channels of a switch "
                  "(Mux.hpp)");

    /// Requests that can be outstanding at once: the engine keeps at most one per device in
    /// flight, so the set needs no more queue than it has devices.
    static constexpr std::size_t QueueDepth = Count;

    static_assert(detail::queueFits<I2c>(QueueDepth),
                  "the bus queue is shallower than this Bus has devices, so a device's "
                  "request can be refused for want of a slot: raise the I2CBehaviorQueued "
                  "QueueDepth template argument");

    /// Whether the bus queue also holds `extra` requests beside this Bus's: a device kept
    /// outside the Bus (a Touch::Controller, a BusScan's probe) is one each. The Bus cannot
    /// see those, so the firmware says: `static_assert(Sensors::queueFits(1));`
    [[nodiscard]] static constexpr bool queueFits(std::size_t extra) {
        return detail::queueFits<I2c>(QueueDepth + extra);
    }

    /// What the bus's CallbackSize has to hold: the largest completion lambda of any device.
    static constexpr std::size_t CallbackBytes = std::max({Ds::CallbackBytes...});

    /// Bits per second this bus's *cyclic* traffic puts on the wire. Reads with a Period and
    /// write groups rewritten on one; on-demand reads and application writes are not in it.
    [[nodiscard]] static constexpr double bitsPerSecond() {
        return (detail::cyclicBitsPerSecond<Ds>() + ... + 0.0);
    }

    /// The same as a fraction of the bus clock: 0.42 is "the periodic traffic wants 42% of a
    /// 400 kHz bus". Available when the bus type says what it is clocked at.
    [[nodiscard]] static constexpr double busLoad()
        requires(detail::HasBaudRate<I2c>)
    {
        return bitsPerSecond() / static_cast<double>(I2c::BaudRate);
    }

    // -- where the devices are --------------------------------------------------------------

    /// The lines of the bridges the devices are behind (Bridge.hpp), each bridge once.
    using BridgeLinesT = typename detail::BridgeLines<Clock, std::tuple<>, Ds...>::type;
    static constexpr std::size_t Bridges = std::tuple_size_v<BridgeLinesT>;

    /// The switches the devices are behind (Mux.hpp), each counted once.
    static constexpr std::size_t Switches = detail::switchMap<Ds...>().switches;

    /// A segment is one stretch of wire: 0 is the bus in front of every switch, and each
    /// channel of each switch is one more -- 1 + 8 * switch + channel, the switches numbered in
    /// the order the Bus first reaches a device behind each. One switch makes nine.
    static constexpr std::size_t Segments = 1 + 8 * Switches;

    /// Where D is in the Bus's order.
    template<typename D>
    [[nodiscard]] static constexpr std::size_t indexOf() {
        static_assert(detail::IndexOf<D, Devices>::value < Count, "that device is not on this Bus");
        return detail::IndexOf<D, Devices>::value;
    }

    /// The segment the device at `index` is on.
    [[nodiscard]] static constexpr std::size_t segmentOf(std::size_t index) {
        return SegmentOfDevice_[index];
    }

    /// The segment D is on.
    template<typename D>
    [[nodiscard]] static constexpr std::size_t segmentOf() {
        return segmentOf(indexOf<D>());
    }

    /// Behind a switch (any segment but 0): its channel, 0..7.
    [[nodiscard]] static constexpr std::uint8_t channelOfSegment(std::size_t segment) {
        return static_cast<std::uint8_t>((segment - 1) % 8);
    }

    /// Behind a switch: which one, in the order Segments numbers them.
    [[nodiscard]] static constexpr std::size_t switchOfSegment(std::size_t segment) {
        return (segment - 1) / 8;
    }

    /// The channel of its switch D is behind.
    template<typename D>
    [[nodiscard]] static constexpr std::uint8_t channelOf() {
        static_assert(detail::BehindChannel<typename D::GateT>,
                      "channelOf<D>(): D is not behind a switch");
        return channelOfSegment(segmentOf<D>());
    }

    /// A segment for a status line: "U" in front of the switches, the channel's digit behind
    /// the first switch ("3"), and the switch's letter and the digit behind any other ("B3").
    [[nodiscard]] static constexpr std::string_view segmentName(std::size_t segment) {
        auto const& name = SegmentNames_[segment];
        return {name.data(), name[1] == '\0' ? std::size_t{1} : std::size_t{2}};
    }

    /// What D's descriptions ask for, a second (detail::nominalSamplesPerSecond and friends).
    template<typename D>
    [[nodiscard]] static constexpr double nominalSamplesPerSecond() {
        return detail::nominalSamplesPerSecond<D>();
    }

    template<typename D>
    [[nodiscard]] static constexpr double nominalWritesPerSecond() {
        return detail::nominalWritesPerSecond<D>();
    }

    template<typename D>
    [[nodiscard]] static constexpr double bitsPerSecond() {
        return detail::cyclicBitsPerSecond<D>();
    }

    // -- the devices -----------------------------------------------------------------------

    /// The gated devices are bound to their switch and to the Bus's own arbiter for it: a
    /// firmware that keeps its parts in a Bus has nothing to bind. One that wants an arbiter
    /// of its own (a policy per switch chosen at run time) binds the gates again itself.
    ///
    /// The same for a device behind a bridge (Bridge.hpp): the Bus owns the bridge's line,
    /// binds the device to it, and binds the line to the part a PartBridge is switched through.
    Bus() {
        bindGates_(std::make_index_sequence<Count>{});
        bindBridgeParts_(std::make_index_sequence<Bridges>{});
    }

    Bus(Bus const&)            = delete;
    Bus& operator=(Bus const&) = delete;

    /// The line of bridge B: `bus.bridge<ExtPort>().on()`, off(), state().
    template<typename B,
             typename Self>
    [[nodiscard]] constexpr auto& bridge(this Self&& self) {
        return std::get<BridgeLine<Clock, B>>(self.bridges_);
    }

    /// The arbiter of switch K (numbered as Segments numbers the switches): what the gates
    /// behind it share, and what SwitchStats reads.
    template<std::size_t K,
             typename Self>
    [[nodiscard]] constexpr auto& arbiter(this Self&& self) {
        static_assert(K < Switches, "that switch is not on this Bus");
        return std::get<K>(self.arbiters_);
    }

    template<std::size_t I,
             typename Self>
    [[nodiscard]] constexpr auto& get(this Self&& self) {
        return std::get<I>(self.devices_);
    }

    /// By device type, which is how a firmware names them: `bus.get<Bh1750>()`.
    template<typename D,
             typename Self>
    [[nodiscard]] constexpr auto& get(this Self&& self) {
        static_assert(detail::IndexOf<D, Devices>::value < Count, "that device is not on this Bus");
        return std::get<detail::IndexOf<D, Devices>::value>(self.devices_);
    }

    /// The bridges' lines, then every device's handler in declaration order. Call once per
    /// main-loop turn, after the bus behavior's own handler.
    void handler() {
        handleBridges_(std::make_index_sequence<Bridges>{});
        std::apply([](auto&... d) { (d.handler(), ...); }, devices_);
    }

    /// Every device starts over from its reset (Device::restart()): what a supply the parts
    /// share being cycled asks for (PowerRail.hpp). The switches' arbiters are left alone --
    /// restart() lets each device's gate go, which is all a claim ever was -- and so are the
    /// bridges: a device behind one that is off starts over when it is back.
    void restart() {
        std::apply([](auto&... d) { (d.restart(), ...); }, devices_);
    }

    /// `f` over every device, in declaration order: what logHealth() does, for a caller
    /// that wants the same walk with its own line -- a status page that names each device
    /// and says whether it is answering, without the firmware repeating the list.
    ///
    ///     bus.forEach([](auto const& d) {
    ///         using D = std::remove_cvref_t<decltype(d)>;
    ///         UC_LOG_I("{} at {:#04x}: {}", D::Chip::Name, D::Address, d.link());
    ///     });
    template<typename Self,
             typename F>
    void forEach(this Self&& self,
                 F&&         f) {
        std::apply([&](auto&... device) { (f(device), ...); }, self.devices_);
    }

    /// Every device is answering and has delivered on all its cyclic read groups since its
    /// bring-up. One that is offline (behind a bridge that is not active) is not asked: it is
    /// not meant to deliver.
    [[nodiscard]] bool valid() const {
        return std::apply([](auto const&... d) { return ((d.offline() || d.valid()) && ...); },
                          devices_);
    }

    /// How many parts are offline right now: behind a bridge that is not active (Bridge.hpp).
    /// They are neither absent nor answering.
    [[nodiscard]] std::size_t offlineCount() const {
        return std::apply(
          [](auto const&... d) {
              return (std::size_t{isPart<std::remove_cvref_t<decltype(d)>>() && d.offline()} + ...
                      + std::size_t{0});
          },
          devices_);
    }

    /// How many parts are parked as absent right now (Broadcast devices left out, as in
    /// answeringCount()). What a PowerRail decides on.
    [[nodiscard]] std::size_t absentCount() const {
        return std::apply(
          [](auto const&... d) {
              return (std::size_t{isPart<std::remove_cvref_t<decltype(d)>>() && d.absent()} + ...
                      + std::size_t{0});
          },
          devices_);
    }

    /// How many parts are answering (Device::link()), out of Parts.
    [[nodiscard]] std::size_t answeringCount() const {
        return std::apply(
          [](auto const&... d) {
              return (std::size_t{isPart<std::remove_cvref_t<decltype(d)>>() && d.answering()} + ...
                      + 0U);
          },
          devices_);
    }

    [[nodiscard]] std::uint32_t errors() const {
        return std::apply([](auto const&... d) { return (d.errors() + ... + 0U); }, devices_);
    }

    /// `f` on the device at `index` in the Bus's order: a status page per device, chosen at run
    /// time.
    template<typename Self,
             typename F>
    void visit(this Self&& self,
               std::size_t index,
               F&&         f) {
        detail::withIndex(
          index,
          [&](auto i) { f(std::get<decltype(i)::value>(self.devices_)); },
          std::make_index_sequence<Count>{});
    }

    /// Per segment: its parts, how many of them are answering, and their errors. A Broadcast
    /// is in none of them.
    struct SegmentCounts {
        std::array<std::uint8_t, Segments>  parts{};
        std::array<std::uint8_t, Segments>  answering{};
        std::array<std::uint8_t, Segments>  offline{};
        std::array<std::uint32_t, Segments> errors{};
        std::size_t                         answeringAll{};
    };

    [[nodiscard]] SegmentCounts counts() const {
        SegmentCounts counts{};
        std::size_t   index = 0;
        forEach([&](auto const& d) {
            auto const segment = segmentOf(index++);
            if(!isPart<std::remove_cvref_t<decltype(d)>>()) { return; }
            ++counts.parts[segment];
            counts.errors[segment] += d.errors();
            if(d.offline()) { ++counts.offline[segment]; }
            if(d.answering()) {
                ++counts.answering[segment];
                ++counts.answeringAll;
            }
        });
        return counts;
    }

    /// A device's running totals, which a poller takes the differences of (Stats.hpp).
    struct Counters {
        std::uint32_t samples{};
        std::uint32_t writes{};
        std::uint32_t errors{};
        std::uint32_t rejected{};
    };

    void snapshot(std::array<Counters,
                             Count>& out) const {
        std::size_t index = 0;
        forEach([&](auto const& d) {
            out[index++] = Counters{.samples  = d.samples(),
                                    .writes   = d.writes(),
                                    .errors   = d.errors(),
                                    .rejected = d.rejected()};
        });
    }

    /// One line per device: what it is, where, and how it is doing.
    void logHealth() const {
        forEach([](auto const& d) { d.logHealth(); });
    }

private:
    template<std::size_t... Is>
    void bindGates_(std::index_sequence<Is...>) {
        (bindGate_<Is>(), ...);
    }

    template<std::size_t I>
    void bindGate_() {
        using D = typename detail::Nth<Devices>::template type<I>;
        if constexpr(detail::OnSwitch<typename D::GateT>) {
            // By the switch map and not by the segment: a MuxFrontGate is on segment 0.
            constexpr auto K = detail::switchMap<Ds...>().ordinal[I];
            std::get<I>(devices_).gate().bind(get<typename D::GateT::Mux>(),
                                              std::get<K>(arbiters_));
        }
        if constexpr(D::Bridged) {
            std::get<I>(devices_).gate().bind(bridge<typename D::GateT::Bridge>());
        }
        bindFrontGate_<I>(std::make_index_sequence<Bridges>{});
    }

    /// A BridgeFrontGate is bound to the line of every bridge it keeps off.
    template<std::size_t I,
             std::size_t... Ks>
    void bindFrontGate_(std::index_sequence<Ks...>) {
        using G = typename detail::Nth<Devices>::template type<I>::GateT;
        (
          [&] {
              using BK = typename std::tuple_element_t<Ks, BridgeLinesT>::Bridge;
              if constexpr(requires { G::template Closes<BK>; }) {
                  if constexpr(G::template Closes<BK>) {
                      std::get<I>(devices_).gate().bind(
                        static_cast<BridgeLink<BK>&>(std::get<Ks>(bridges_)));
                  }
              }
          }(),
          ...);
    }

    template<std::size_t... Ks>
    void bindBridgeParts_(std::index_sequence<Ks...>) {
        (bindBridgePart_<Ks>(), ...);
    }

    template<std::size_t K>
    void bindBridgePart_() {
        using Line = std::tuple_element_t<K, BridgeLinesT>;
        if constexpr(Line::PartDriven) {
            using Part = typename Line::Bridge::Part;
            static_assert(detail::IndexOf<Part, Devices>::value < Count,
                          "the part a PartBridge is switched through is not on this Bus");
            static_assert(!std::is_same_v<typename detail::BridgeOfGate<typename Part::GateT>::type,
                                          typename Line::Bridge>,
                          "a bridge cannot be switched through a part that is behind it");
            std::get<K>(bridges_).bindPart(get<Part>());
        }
    }

    /// Every line's turn, in the Bus's order. Which lines wait for the wire is looked at for
    /// all of them first: a line's handler takes its parts' requests off, and the lines after
    /// it must still see them. Whether the others are off is looked at line by line, so that of
    /// two that want to start in the same turn the second sees the first go.
    template<std::size_t... Ks>
    void handleBridges_(std::index_sequence<Ks...>) {
        if constexpr(Bridges != 0) {
            std::array<bool, Bridges> const waiting{std::get<Ks>(bridges_).waiting()...};
            (std::get<Ks>(bridges_).handler(mayStart_<Ks>(waiting, std::index_sequence<Ks...>{}),
                                            contested_<Ks>(waiting, std::index_sequence<Ks...>{})),
             ...);
        }
    }

    /// A bridge of an ExclusiveGroup may leave `off` only while every other one of the group
    /// is off: break before make. Two that are wanted at once: a switched one that has been on
    /// less recently goes first, so the bridges of a group take turns; else the first in the
    /// Bus's order.
    template<std::size_t K,
             std::size_t... Js>
    [[nodiscard]] bool mayStart_(std::array<bool,
                                            Bridges> const& waiting,
                                 std::index_sequence<Js...>) const {
        using BK = typename std::tuple_element_t<K, BridgeLinesT>::Bridge;
        return (
          (!detail::exclusiveBridges<BK, typename std::tuple_element_t<Js, BridgeLinesT>::Bridge>()
           || (std::get<Js>(bridges_).state() == BridgeState::off
               && !(waiting[Js] && goesFirst_<Js, K>())))
          && ...);
    }

    /// Line J has been on less recently than line K.
    template<std::size_t J,
             std::size_t K>
    [[nodiscard]] bool goesFirst_() const {
        auto const j = std::get<J>(bridges_).servedAt();
        auto const k = std::get<K>(bridges_).servedAt();
        return j < k || (j == k && J < K);
    }

    /// Another bridge of K's group waits for the wire: a switched K gives it up.
    template<std::size_t K,
             std::size_t... Js>
    [[nodiscard]] bool contested_(std::array<bool,
                                             Bridges> const& waiting,
                                  std::index_sequence<Js...>) const {
        using BK = typename std::tuple_element_t<K, BridgeLinesT>::Bridge;
        return (
          (detail::exclusiveBridges<BK, typename std::tuple_element_t<Js, BridgeLinesT>::Bridge>()
           && waiting[Js])
          || ...);
    }

    /// One arbiter per switch, in the order Segments numbers them.
    template<typename Seq>
    struct Arbiters;

    template<std::size_t... Ks>
    struct Arbiters<std::index_sequence<Ks...>> {
        using type = std::tuple<SwitchArbiterAt<Ks>...>;
    };

    static constexpr std::array<std::uint8_t, Count> SegmentOfDevice_
      = detail::segmentTable<Ds...>();

    static constexpr std::array<std::array<char, 2>, Segments> SegmentNames_ = [] {
        std::array<std::array<char, 2>, Segments> names{};
        names[0] = {'U', '\0'};
        for(std::size_t segment = 1; segment < Segments; ++segment) {
            auto const onSwitch = (segment - 1) / 8;
            auto const channel  = static_cast<char>('0' + (segment - 1) % 8);
            names[segment]      = onSwitch == 0
                                  ? std::array<char, 2>{channel, '\0'}
                                  : std::array<char, 2>{static_cast<char>('A' + onSwitch), channel};
        }
        return names;
    }();

    std::tuple<Ds...>                                                                 devices_{};
    [[no_unique_address]] typename Arbiters<std::make_index_sequence<Switches>>::type arbiters_{};
    [[no_unique_address]] BridgeLinesT                                                bridges_{};
};

}   // namespace Kvasir::I2C
