/// The Bus: what is true of the set of devices and of nothing else -- distinct addresses,
/// the queue depth, the cyclic load, segments and counters, the rates between two reports --
/// and the plain Scanner with its catalogue.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/MonoPanel.hpp>
#include <kvasir/Devices/I2C/Mux.hpp>
#include <kvasir/Devices/I2C/Scanner.hpp>
#include <kvasir/Devices/I2C/Stats.hpp>
#include <kvasir/Devices/I2C/chips/All.hpp>
#include <map>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

/// A scan that gives up sooner than ScanDefaults.
struct QuickScan {
    static constexpr auto ProbeTimeout = std::chrono::milliseconds{100};
};

void scanner() {
    testCase("bus scanner");
    fresh();
    using Cat                          = Catalogue<Chips::Bh1750, Chips::Ads1115<>, Chips::Tmp1075>;
    using Scan                         = Scanner<FakeBus, FakeClock, Cat>;
    bool                      busFault = false;
    std::vector<std::uint8_t> probed;
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            probed.push_back(a);
            if(busFault && a == 0x50) { return FakeBus::Result::failed; }
            if(a != 0x23 && a != 0x48) { return FakeBus::Result::notAcknowledged; }
            static_cast<void>(sent);
            if(!recv.empty()) { recv[0] = std::byte{0x00}; }
            return FakeBus::Result::succeeded;
        };
    Scan sc{};
    sc.start();
    for(int i = 0; i < 1000 && !sc.done(); ++i) {
        sc.handler();
        FakeBus::complete();
    }
    check(sc.done() && sc.scanned(), "a full scan finished");
    checkEq(sc.count(), std::size_t{2}, "two devices answered");
    check(sc.found(0x23) && sc.found(0x48), "the BH1750 and the temperature sensor");
    check(!sc.found(0x50), "and nothing at 0x50");
    checkEq(probed.front(), std::uint8_t{0x08}, "the scan starts past the reserved range");
    checkEq(probed.back(), std::uint8_t{0x77}, "and stops before it");
    check(Scan::hint(0x48).find("ADS1115") != std::string_view::npos
            && Scan::hint(0x48).find("TMP1075") != std::string_view::npos,
          "the catalogue names both candidates at 0x48");
    check(Scan::hint(0x23) == std::string_view{"BH1750"}, "and the one at 0x23");
    check(Scan::hint(0x60) == std::string_view{"unknown"}, "nothing known at 0x60");
    // The hints are packed back to back in the image: the neighbours of 0x48 must not run into it,
    // and a hint that outgrows its width still ends in "~".
    check(Scan::hint(0x48).starts_with("ADS1115") && Scan::hint(0x48).ends_with("TMP1075"),
          "0x48's hint is its own names and nothing of its neighbours'");
    bool packedRight = true;
    for(std::size_t a = 0; a < Cat::AddressCount; ++a) {
        auto const& entry = Cat::Table[a];
        auto const  want  = entry.length == 0 ? std::string_view{"unknown"} : entry.view();
        packedRight       = packedRight && Cat::hint(static_cast<std::uint8_t>(a)) == want;
    }
    check(packedRight, "every address's packed hint is the one the table was built with");
    using Narrow = SizedCatalogue<12, Chips::Bh1750, Chips::Ads1115<>, Chips::Tmp1075>;
    checkEq(Narrow::hint(0x48).size(), std::size_t{12}, "a hint is cut at its width");
    check(Narrow::hint(0x48).ends_with("~"), "and says so");
    check(Narrow::hint(0x23) == std::string_view{"BH1750"}, "a short one is whole");
    checkEq(Narrow::names(0x48).size(), Cat::names(0x48).size(), "names() is never cut");

    // a range
    probed.clear();
    sc.start(0x40, 0x4F);
    for(int i = 0; i < 500 && !sc.done(); ++i) {
        sc.handler();
        FakeBus::complete();
    }
    checkEq(probed.size(), std::size_t{16}, "sixteen addresses probed");
    check(sc.found(0x48) && !sc.found(0x23), "0x23 was outside the range and is not stale");

    // an explicit list
    probed.clear();
    std::array<std::uint8_t, 3> const wanted{0x23, 0x50, 0x77};
    sc.start(std::span<std::uint8_t const>{wanted});
    for(int i = 0; i < 500 && !sc.done(); ++i) {
        sc.handler();
        FakeBus::complete();
    }
    check(probed == std::vector<std::uint8_t>{0x23, 0x50, 0x77}, "only the listed addresses");
    checkEq(sc.count(), std::size_t{1}, "one of them answered");

    // a bus fault is not a NAK
    busFault = true;
    sc.start(0x50, 0x50);
    for(int i = 0; i < 100 && !sc.done(); ++i) {
        sc.handler();
        FakeBus::complete();
    }
    check(!sc.found(0x50) && sc.faulted(0x50), "recorded as a fault, not as a device");
    checkEq(sc.faults(), std::size_t{1}, "and counted");
    busFault = false;

    // a probe the bus lost: no callback, ever. Given up on after ProbeTimeout, counted as a
    // fault, the scan goes on -- and the answer, should it turn up late, is not taken for
    // the next address's.
    sc.start(0x23, 0x48);
    sc.handler();   // the probe for 0x23 is submitted
    auto const lost = FakeBus::pending.front();
    FakeBus::pending.clear();
    sc.handler();
    check(!sc.done(), "waiting on the lost probe");
    FakeClock::current += Scan::ProbeTimeout + 1ms;
    sc.handler();                                // gives up on 0x23
    sc.handler();                                // and submits 0x24
    lost.callback(FakeBus::Result::succeeded);   // the late answer for 0x23
    for(int i = 0; i < 100 && !sc.done(); ++i) {
        sc.handler();
        FakeBus::complete();
    }
    check(sc.done(), "the scan finished without the answer");
    check(sc.faulted(0x23) && !sc.found(0x23), "the lost probe is a fault, not a device");
    check(!sc.found(0x24), "and its late answer was not taken for the next address");
    check(sc.found(0x48), "the addresses after it were still probed");

    // a shorter timeout, set at run time: given up on at 10 ms, not a second
    sc.probeTimeout(10ms);
    sc.start(0x23, 0x23);
    sc.handler();
    static_cast<void>(FakeBus::lose());
    FakeClock::current += 5ms;
    sc.handler();
    check(!sc.done(), "still waiting at 5 ms");
    FakeClock::current += 6ms;
    sc.handler();
    check(sc.done() && sc.faulted(0x23), "given up on past 10 ms, as a fault");
    // and one from a Config, at compile time
    static_assert(Scanner<FakeBus, FakeClock, Cat, AddressProbe, QuickScan>::ProbeTimeout == 100ms,
                  "a Config's ProbeTimeout is the scanner's");

    // the register-write probe
    probed.clear();
    Scanner<FakeBus, FakeClock, Cat, RegisterProbe<0x0F>> rp{};
    rp.start(0x23, 0x23);
    for(int i = 0; i < 100 && !rp.done(); ++i) {
        rp.handler();
        FakeBus::complete();
    }
    check(rp.found(0x23), "the register probe finds it too");
    {
        bool wrote = false;
        for(auto const& probe : FakeBus::log) {
            if(probe.address == 0x23 && probe.sent == std::vector<std::uint8_t>{0x0F}) {
                wrote = true;
            }
        }
        check(wrote, "and sends the register byte first");
    }
    if(failures != 0) { dump(); }
}

// The two panel geometries the Bus cases below put on a Bus (panel_test.cpp has the same).
namespace Oled {
    struct Wide : Chips::Ssd1306Detail::PanelDefaults {
        static constexpr int      Width   = 128;
        static constexpr int      Height  = 32;
        static constexpr Address7 Address = 0x3C;
    };

    struct Small : Chips::Ssd1306Detail::PanelDefaults {
        static constexpr int          Width        = 64;
        static constexpr int          Height       = 48;
        static constexpr Address7     Address      = 0x3D;
        static constexpr int          ColumnOffset = 32;
        static constexpr std::uint8_t ComPins      = 0x12;
    };
}   // namespace Oled

/// What a frame renderer asks of a panel.
template<typename P>
concept PanelContract = requires(P& p, std::uint32_t pages) {
    { P::Width } -> std::convertible_to<int>;
    { P::Height } -> std::convertible_to<int>;
    { p.ready() } -> std::same_as<bool>;
    { p.failed() } -> std::same_as<bool>;
    { p.handler() };
    { p.pageRam() };
    { p.pagesChanged(pages) };
};

// -- the Bus: what is true of the set of devices and of nothing else ----------------------

namespace BusTest {

    /// A bus that says what the real behaviors say, so the queue-depth and load checks are on.
    struct RatedBus : FakeBus {
        static constexpr std::uint32_t BaudRate   = 400'000;
        static constexpr std::size_t   QueueDepth = 8;
    };

    template<typename C, typename Cfg = DefaultConfig>
    using RDev = Device<RatedBus, FakeClock, C, Cfg>;

    using Bh   = RDev<Chips::Bh1750>;       // 0x23, receive(2)        @ 200 ms
    using Veml = RDev<Chips::Veml6030<>>;   // 0x10, two 2-byte reads  @ 200 ms
    using Bme  = RDev<Chips::Bme280>;       // 0x76, one 8-byte read   @ 1000 ms
    using Sht  = RDev<Chips::Sht3x>;        // 0x44, command + receive @ 1000 ms

    using Sensors = Kvasir::I2C::Bus<RatedBus, FakeClock, Bh, Veml, Bme, Sht>;

    /// The four parts on the wire, each answering what its bring-up asks: the VEML6030's and
    /// the BME280's ids from a register file, Sensirion words with their CRC for the rest.
    struct Wiring {
        RegisterModel<1, 2> veml{Veml::Address};
        RegisterModel<1>    bme{Bme::Address};

        Wiring() {
            veml.set(0x07, {0x81, 0xC4});
            bme.set(0xD0, {0x60});
        }

        FakeBus::Result operator()(std::uint8_t               addr,
                                   std::span<std::byte const> sent,
                                   std::span<std::byte>       recv) {
            if(addr == Veml::Address) { return veml(addr, sent, recv); }
            if(addr == Bme::Address) { return bme(addr, sent, recv); }
            return crcZeros(addr, sent, recv);
        }
    };

    // The address check has to be able to say no; a Bus that collides is a hard error by design.
    static_assert(detail::addressesDistinct<Bh,
                                            Veml,
                                            Bme,
                                            Sht>(),
                  "four different addresses");
    static_assert(!detail::addressesDistinct<Bh,
                                             Veml,
                                             Bh>(),
                  "0x23 twice");
    static_assert(!detail::addressesDistinct<Sht,
                                             Sht>(),
                  "the same device twice");
    static_assert(detail::addressesDistinct<Bh>(),
                  "one device cannot collide");

    static_assert(Sensors::Count == 4);
    static_assert(Sensors::QueueDepth == 4 && Sensors::QueueDepth <= RatedBus::QueueDepth);

    // What the bus queue has room for beside the Bus's own devices: a scan's probe, a device
    // kept outside the Bus.
    static_assert(Sensors::queueFits(4) && !Sensors::queueFits(5),
                  "four more requests fit an eight-deep queue beside four devices, five do not");

    // The completion lambda every device hands the bus is the same size, and the Bus's figure
    // is the largest of them: what a bus behavior's CallbackSize has to hold.
    static_assert(Sensors::CallbackBytes == Bh::CallbackBytes
                  && Sensors::CallbackBytes == Sht::CallbackBytes);
    static_assert(Sensors::CallbackBytes >= sizeof(void*) + sizeof(std::uint32_t),
                  "the device and a generation");

    // A bus that does not say how deep its queue is: nothing to check against, and so nothing
    // to fail to compile.
    using Unrated = Kvasir::I2C::Bus<FakeBus, FakeClock, Dev<Chips::Bh1750>>;
    static_assert(Unrated::Count == 1 && Unrated::QueueDepth == 1);
    static_assert(Unrated::queueFits(1000),
                  "nothing to check against");

    /// Hand-computed from the four descriptions, nine bits a byte plus a START and a STOP:
    ///   BH1750   addr + 2                        = 3 B -> 29 bits, 5/s  = 145
    ///   VEML6030 (addr + reg + addr + 2) x 2     = 5 B -> 47 bits x 2, 5/s = 470
    ///   BME280   addr + reg + addr + 8           = 11 B -> 101 bits, 1/s = 101
    ///   SHT3x    (addr + 2) + (addr + 6)         = 29 + 65 bits, 1/s  =  94
    static_assert(Sensors::bitsPerSecond() == 810.0,
                  "the four scripts, by hand");

    // A panel for a device that lives inside a Bus. A Bus member cannot be named as a
    // reference template argument at all, so this is the form that has to work when neither
    // the bus nor the device is a global: monoPanel<Which>(bus) reaches through the bus and
    // keeps the reference.
    using OledWide  = RDev<Chips::Ssd1315<Oled::Wide>>;
    using OledSmall = RDev<Chips::Ssd1315<Oled::Small>>;
    using Panels    = Kvasir::I2C::Bus<RatedBus, FakeClock, OledWide, OledSmall>;

    using BusPanel = Kvasir::I2C::MonoPanelRef<OledWide>;

    static_assert(std::is_same_v<BusPanel::Device,
                                 OledWide>,
                  "the panel found its device");
    static_assert(BusPanel::Pages == 4,
                  "128 x 32 is four pages");
    static_assert(PanelContract<BusPanel>,
                  "and it is still what a frame renderer wants");

}   // namespace BusTest

void bus() {
    using namespace BusTest;

    // Both live here rather than at namespace scope: a Bus keeps all its state in members and
    // the completion callback captures `this` when a transfer is submitted, so nothing needs
    // to find either of them by name.
    Sensors sensors{};
    Panels  panels{};

    testCase("Bus: the load is the cyclic traffic as a fraction of the clock");
    checkNear(Sensors::busLoad(), 810.0 / 400000.0, 1e-12, "810 bits/s on a 400 kHz bus");
    check(Sensors::busLoad() < 0.01, "four slow sensors are nowhere near saturating the bus");

    testCase("Bus: one handler() turn drives every device on it");
    fresh();
    Wiring wiring{};
    FakeBus::respond = std::ref(wiring);
    runFor(sensors, 400ms);
    for(auto const addr : {Bh::Address, Veml::Address, Bme::Address, Sht::Address}) {
        bool seen = false;
        for(auto const& t : FakeBus::log) {
            if(t.isBus() && t.address == addr) { seen = true; }
        }
        check(seen, "every device got traffic from the one handler() call");
    }
    checkEq(sensors.answeringCount(), std::size_t{4}, "all four acknowledged");
    checkEq(sensors.errors(), 0U, "and none of them faulted");

    testCase("Bus: forEach walks every device once, in declaration order");
    {
        std::vector<std::uint8_t> seen{};
        sensors.forEach(
          [&](auto const& d) { seen.push_back(std::remove_cvref_t<decltype(d)>::Address); });
        check(
          seen == std::vector<std::uint8_t>{Bh::Address, Veml::Address, Bme::Address, Sht::Address},
          "the four addresses, in the order the Bus was written");
    }

    testCase("Bus: get by type and by index name the same device");
    check(&sensors.get<Bh>() == &sensors.get<0>(), "Bh1750 is device 0");
    check(&sensors.get<Sht>() == &sensors.get<3>(), "SHT3x is device 3");
    check(static_cast<void const*>(&sensors.get<Bh>())
            != static_cast<void const*>(&sensors.get<Veml>()),
          "different devices are different objects");

    testCase("Bus: a device inside it drives a MonoPanel");
    fresh();
    FakeBus::respond = [](std::uint8_t, std::span<std::byte const>, std::span<std::byte>) {
        return FakeBus::Result::succeeded;   // the panel is write-only; it only has to ACK
    };
    check(BusPanel::Width == 128 && BusPanel::Height == 32, "the geometry comes off the chip");
    auto busPanel = Kvasir::I2C::monoPanel<OledWide>(panels);
    runFor(panels, 400ms);
    check(busPanel.ready(), "the panel in the bus came up and is idle");
    check(&busPanel.device() == &panels.get<OledWide>(), "and it points at that bus's device");
    auto const before        = FakeBus::log.size();
    busPanel.pageRam()[1][7] = 0xFF;
    busPanel.pagesChanged(1U << 1);
    runFor(panels, 50ms);
    check(hasWrite({0x00, 0xB1, 0x00, 0x10}, before), "page 1's window went out, and only it");
    check(!hasWrite({0x00, 0xB2, 0x00, 0x10}, before),
          "page 2 was not touched, so it was not sent");
    if(failures != 0) { dump(); }
}

// -- a Bus's segments, counters and rates, and the gaps between samples -------------------------

namespace BusMetaTest {

    using MuxA = Dev<Chips::Tca9548a>;

    struct MuxBConfig {
        static constexpr Address7 Address = 0x74;
    };

    using MuxB = Device<FakeBus, FakeClock, Chips::Tca9548a, MuxBConfig>;

    template<typename C, typename M, std::uint8_t Ch>
    using Behind = Device<FakeBus, FakeClock, C, DefaultConfig, NoReset, MuxGate<M, Ch>>;

    using Front = Dev<Chips::Bh1750>;
    using A2    = Behind<Chips::Sht3x, MuxA, 2>;
    using B5    = Behind<Chips::Sht3x, MuxB, 5>;
    // At its other address: a part in front of the switches is on every channel's wire, so the
    // one on channel 7 cannot have the address of `Front`.
    using A7  = Device<FakeBus, FakeClock, Chips::Bh1750, At<0x5C>, NoReset, MuxGate<MuxA, 7>>;
    using Two = Kvasir::I2C::Bus<FakeBus, FakeClock, MuxA, Front, A2, MuxB, B5, A7>;

    static_assert(Two::Switches == 2 && Two::Segments == 17,
                  "two switches, sixteen channels and the front");
    static_assert(Two::segmentOf<MuxA>() == 0 && Two::segmentOf<Front>() == 0,
                  "in front of both switches");
    static_assert(Two::segmentOf<A2>() == 3 && Two::segmentOf<A7>() == 8,
                  "channels 2 and 7 of the first");
    static_assert(Two::segmentOf<B5>() == 14,
                  "channel 5 of the second: 1 + 8 + 5");
    static_assert(Two::channelOf<B5>() == 5 && Two::switchOfSegment(Two::segmentOf<B5>()) == 1);
    static_assert(Two::segmentName(0) == "U" && Two::segmentName(3) == "2"
                  && Two::segmentName(14) == "B5");
    static_assert(Two::nominalSamplesPerSecond<Front>() == 5.0
                  && Two::nominalSamplesPerSecond<A2>() == 1.0);
    static_assert(Two::nominalWritesPerSecond<Front>() == 0.0
                  && Two::nominalSamplesPerSecond<MuxA>() == 0.0);
    static_assert(Two::bitsPerSecond<Front>() == 145.0,
                  "BusTest's hand count for the BH1750");

    /// A 10 ms read whose bus completions are kept.
    struct Stamped {
        static constexpr std::string_view Name          = "STAMP";
        static constexpr Address7         Address       = 0x2C;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Data {
            static constexpr auto       Period      = std::chrono::milliseconds{10};
            static constexpr bool       Timestamped = true;
            static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        using Reads = List<Data>;
    };

}   // namespace BusMetaTest

void busMetadata() {
    using namespace BusMetaTest;

    testCase("Bus: counts per segment, a device by index, the counters in one snapshot");
    fresh();
    SwitchWire wiring{
      .switches = {MuxA::Address, MuxB::Address},
      .behind   = crcZeros
    };
    FakeBus::respond = std::ref(wiring);
    auto bus         = std::make_unique<Two>();
    runFor(*bus, 500ms);
    auto const counts = bus->counts();
    checkEq(counts.parts[0], 3, "three parts in front of the switches");
    checkEq(counts.parts[3] + counts.parts[8] + counts.parts[14],
            3,
            "one behind each channel used");
    checkEq(counts.answering[0], 3, "the two switches and the BH1750 answer");
    check(counts.answering[3] == 1 && counts.answering[8] == 1 && counts.answering[14] == 1,
          "and so does every part behind a channel, through the gate the Bus bound for it");
    checkEq(counts.answeringAll, std::size_t{6}, "six of six");
    check(bus->get<A2>().gate().bound() && bus->get<B5>().gate().bound(), "the gates are bound");
    {
        // The Bus's own arbiter per switch is what a SwitchStats reads: channels 2 and 7 of
        // the first switch took it, held it, and were never refused it by each other.
        SwitchStats<Two::SwitchArbiterAt<0>> stats{};
        check(!stats.update(bus->arbiter<0>()), "the first update only takes the totals");
        runFor(*bus, 1500ms);   // the SHT3x behind channel 2 measures once a second
        check(stats.update(bus->arbiter<0>()), "the second has deltas");
        auto const& two   = stats.channel(2);
        auto const& seven = stats.channel(7);
        check(two.grants > 0 && seven.grants > 0, "both channels took the switch");
        check(two.held > 0us && seven.held > 0us, "and held it for some time each");
        checkEq(stats.channel(0).grants, 0U, "a channel with nothing behind it never did");
        SwitchStats<Two::SwitchArbiterAt<1>> statsB{};
        static_cast<void>(statsB.update(bus->arbiter<1>()));
        runFor(*bus, 500ms);
        static_cast<void>(statsB.update(bus->arbiter<1>()));
        check(statsB.channel(5).grants > 0 && statsB.channel(2).grants == 0,
              "the second switch's arbiter is its own: channel 5, and not the first's channel 2");
    }
    std::string_view name{};
    bus->visit(1, [&](auto const& dev) { name = std::remove_cvref_t<decltype(dev)>::Chip::Name; });
    check(name == Chips::Bh1750::Name, "visit(1) is the BH1750");
    std::array<Two::Counters, Two::Count> counters{};
    bus->snapshot(counters);
    check(counters[1].samples > 0 && counters[1].samples == bus->get<Front>().samples(),
          "its samples");
    checkEq(counters[0].writes, bus->get<MuxA>().writes(), "the switch's writes");

    testCase("Bus: a Broadcast is not a part");
    {
        using Call     = BusTest::RDev<Chips::GeneralCall>;
        using WithCall = Kvasir::I2C::Bus<BusTest::RatedBus, FakeClock, BusTest::Bh, Call>;
        static_assert(WithCall::Count == 2 && WithCall::Parts == 1,
                      "two devices, one part: the general call is a Broadcast");
        static_assert(WithCall::isPart<BusTest::Bh>() && !WithCall::isPart<Call>());
        fresh();
        FakeBus::respond    = zeros;
        auto       withCall = std::make_unique<WithCall>();
        auto const turns    = [&](int n) {
            for(int i = 0; i < n; ++i) { turn(*withCall); }
        };
        turns(500);
        check(withCall->get<Call>().link() == Link::starting, "nothing asked of the call yet");
        checkEq(withCall->answeringCount(), std::size_t{1}, "one of one part answering");
        auto const c = withCall->counts();
        checkEq(c.parts[0], 1, "the call is in no segment's parts");
        checkEq(c.answeringAll, std::size_t{1}, "nor in the answering");
        withCall->get<Call>().set<Chips::GeneralCall::Command>(0xFF);
        turns(20);
        check(withCall->get<Call>().answering(), "the call went out and was acknowledged");
        checkEq(withCall->answeringCount(), std::size_t{1}, "and is still not counted");
    }

    testCase("BusRates: a part's rate against its nominal, and its segment's");
    fresh();
    FakeBus::respond   = zeros;
    auto       sensors = std::make_unique<BusTest::Sensors>();
    auto const run     = [&](int turns) {
        for(int i = 0; i < turns; ++i) { turn(*sensors); }
    };
    BusRates<BusTest::Sensors> rates{};
    run(500);
    check(!rates.update(*sensors, 500ms), "the first update only takes the totals");
    run(2000);
    check(rates.update(*sensors, 2s), "the second has deltas");
    auto const& bh = rates.part(0);
    check(bh.answering && !bh.writing && bh.nominal == 5.0, "the BH1750 reads, at 5/s nominal");
    checkNear(bh.rate, 5.0, 0.6, "and gets them");
    check(rates.segment(0).parts >= 1 && rates.segment(0).rate >= bh.rate,
          "its segment adds it up");

    testCase("takeGaps: the longest gap between a timestamped group's samples, and the late ones");
    fresh();
    FakeBus::respond = zeros;
    Dev<Stamped> st{};
    check(runUntil(st, [&] { return st.samples() >= 5; }, 200ms), "sampling");
    static_cast<void>(st.takeGaps<Stamped::Data>());
    for(int i = 0; i < 35; ++i) {   // 35 ms in which the bus completes nothing
        st.handler();
        FakeClock::current += 1ms;
    }
    auto const before = st.samples();
    check(runUntil(st, [&] { return st.samples() >= before + 5; }, 200ms), "sampling again");
    auto const gaps = st.takeGaps<Stamped::Data>();
    check(gaps.longest >= 30ms, "the stall is the longest gap");
    checkEq(gaps.late, 1U, "and the one gap of two periods or more");
    check(runUntil(st, [&] { return st.samples() >= before + 10; }, 200ms), "and on");
    checkEq(st.takeGaps<Stamped::Data>().late, 0U, "none since: taken and cleared");
}

// -- two buses: a FakeBusFor per tag, each with its own queue, model and transcript -------

namespace TwoBuses {
    struct I2c0 {};

    struct I2c1 {};

    using Wire0 = FakeBusFor<I2c0>;
    using Wire1 = FakeBusFor<I2c1>;

    // The same part at the same address on both, so only the bus tells them apart; I2C1 has
    // an SHT3x as well, which I2C0 must never see.
    using Left = Kvasir::I2C::Bus<Wire0, FakeClock, DevOn<I2c0, Chips::Bh1750>>;
    using Right
      = Kvasir::I2C::Bus<Wire1, FakeClock, DevOn<I2c1, Chips::Bh1750>, DevOn<I2c1, Chips::Sht3x>>;

    /// A BH1750 at 0x23 whose every reading is `reading`, and Sensirion zeros for the rest.
    struct Lux {
        std::array<std::uint8_t, 2> reading{};

        FakeBusResult operator()(std::uint8_t               addr,
                                 std::span<std::byte const> sent,
                                 std::span<std::byte>       recv) const {
            if(addr != Chips::Bh1750::Address) { return crcZeros(addr, sent, recv); }
            for(std::size_t i = 0; i < recv.size() && i < reading.size(); ++i) {
                recv[i] = static_cast<std::byte>(reading[i]);
            }
            return FakeBusResult::succeeded;
        }
    };
}   // namespace TwoBuses

void twoBuses() {
    using namespace TwoBuses;
    testCase("two buses: a Bus on I2C0 and one on I2C1 run side by side, transcripts apart");
    fresh();
    fresh<I2c0, I2c1>();
    Wire0::respond = Lux{
      {0x04, 0xB0}
    };   // 1200 counts
    Wire1::respond = Lux{
      {0x09, 0x60}
    };   // 2400 counts
    Left  left{};
    Right right{};
    check(runUntil<I2c0, I2c1>(
            all(left, right),
            [&] {
                return left.get<0>().samples() >= 2 && right.get<0>().samples() >= 2
                    && right.get<1>().samples() >= 1;
            },
            3s),
          "every device on both buses sampling from the one loop");
    checkEq(left.get<0>().latest().raw, 1200U, "I2C0's BH1750 read I2C0's model");
    checkEq(right.get<0>().latest().raw, 2400U, "I2C1's BH1750 read I2C1's model");

    auto const reads = [](std::vector<Transaction> const& log, std::uint8_t addr) {
        std::vector<std::vector<std::uint8_t>> got{};
        for(auto const& t : log) {
            if(t.isRead() && t.address == addr) { got.push_back(t.received); }
        }
        return got;
    };
    auto const only = [](std::vector<std::vector<std::uint8_t>> const& got,
                         std::vector<std::uint8_t> const&              bytes) {
        return !got.empty() && std::ranges::all_of(got, [&](auto const& r) { return r == bytes; });
    };
    check(only(reads(Wire0::log, 0x23), {0x04, 0xB0}), "I2C0's transcript: its own answers only");
    check(only(reads(Wire1::log, 0x23), {0x09, 0x60}), "I2C1's transcript: its own answers only");
    check(std::ranges::all_of(Wire0::log, [](auto const& t) { return t.address == 0x23; }),
          "no SHT3x traffic on I2C0");
    check(std::ranges::any_of(Wire1::log, [](auto const& t) { return t.address == 0x44; }),
          "the SHT3x's on I2C1");
    check(std::cmp_equal(Wire0::submitted, Wire0::log.size())
            && std::cmp_equal(Wire1::submitted, Wire1::log.size()),
          "each bus counted its own submits");
    check(writes<I2c0>()
            == std::vector<std::vector<std::uint8_t>>{{0x01}, {0x07}, {0x42}, {0x65}, {0x10}},
          "I2C0's writes are the one BH1750's bring-up");
    check(hasWrite<I2c1>({0x01}) && hasWrite<I2c1>({0x10}), "and I2C1 has its own");
    check(FakeBus::log.empty() && FakeBus::submitted == 0, "and the untagged bus saw nothing");
    if(failures != 0) {
        dump<I2c0>();
        dump<I2c1>();
    }

    testCase("two buses: a fault on one leaves the other alone");
    Wire0::respond         = alwaysNak;
    auto const rightBefore = right.get<0>().samples();
    runFor<I2c0, I2c1>(all(left, right), 1s);
    check(!left.get<0>().answering(), "I2C0's BH1750 is gone");
    check(right.get<0>().answering() && right.get<0>().samples() >= rightBefore + 4,
          "I2C1's goes on at 5/s");
    checkEq(right.errors(), 0U, "without a fault of its own");
}

}   // namespace

int main() {
    scanner();
    bus();
    busMetadata();
    twoBuses();
    return finish();
}
