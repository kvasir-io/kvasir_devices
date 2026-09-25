/// BusScan against a fake bus with a switch: the front with the switch closed and then every
/// channel, each probe with the switch where the scan says; the parts' own addresses left alone
/// while they run; a scan that finishes beside a part that is always due; every address at boot;
/// the front alone when the switch does not answer; a Bus without a switch; and two switches,
/// each channel probed with the other switch shut, and one of them missing.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/BusScan.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Mux.hpp>
#include <kvasir/Devices/I2C/chips/Bh1750.hpp>
#include <kvasir/Devices/I2C/chips/Sht4x.hpp>
#include <kvasir/Devices/I2C/chips/Tca9548a.hpp>
#include <memory>
#include <span>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

using MuxDev = Device<FakeBus, FakeClock, Chips::Tca9548a>;

template<typename C, std::uint8_t Ch>
using On = Device<FakeBus, FakeClock, C, DefaultConfig, NoReset, MuxGate<MuxDev, Ch>>;

/// A part that is always due: a one-byte register read every millisecond.
struct BusyChip {
    static constexpr std::string_view Name          = "BUSY";
    static constexpr Address7         Address       = 0x61;
    static constexpr std::size_t      RegisterBytes = 1;

    struct Data {
        static constexpr auto       Period = std::chrono::milliseconds{1};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
    };

    using Reads = List<Data>;
};

using Light = Device<FakeBus, FakeClock, Chips::Bh1750>;   // 0x23, in front of the switch
using Sht   = On<Chips::Sht4x, 2>;                         // 0x44, behind channel 2
using Busy  = On<BusyChip, 5>;                             // 0x61, behind channel 5
using Wire  = Kvasir::I2C::Bus<FakeBus, FakeClock, MuxDev, Light, Sht, Busy>;
using Scan  = BusScan<Wire>;

static_assert(Wire::Segments == 9 && Wire::segmentOf<Sht>() == 3 && Wire::segmentOf<Busy>() == 6);
static_assert(std::is_same_v<Wire::SwitchDevice,
                             MuxDev>
              && std::is_same_v<Wire::SwitchArbiter,
                                MuxArbiter>);

/// The wiring as the fake bus answers it: the switch keeps what it was last written, and a part
/// behind it answers only while its channel is open. Every probe -- a bare one-byte read -- is
/// kept with the switch as it was.
struct Wiring {
    struct Probe {
        std::uint8_t address{};
        std::uint8_t switchValue{};
        bool         ack{};
    };

    std::uint8_t              switchValue{};
    bool                      switchAnswers{true};
    bool                      switchFaults{};   ///< every transaction to it is a bus fault
    std::vector<Probe>        probes{};
    std::vector<std::uint8_t> shtReads{};   ///< the switch at every SHT4x result read

    FakeBus::Result operator()(std::uint8_t               address,
                               std::span<std::byte const> sent,
                               std::span<std::byte>       recv) {
        bool const probe  = sent.empty() && recv.size() == 1;
        auto const open   = [&](unsigned ch) { return ((switchValue >> ch) & 1U) != 0; };
        auto const answer = [&](bool present) {
            if(probe) { probes.push_back({address, switchValue, present}); }
            if(!present) { return FakeBus::Result::notAcknowledged; }
            // Sensirion words of zero with their CRC, which the other parts do not mind.
            for(std::size_t i = 0; i < recv.size(); ++i) {
                recv[i] = std::byte{static_cast<std::uint8_t>(i % 3 == 2 ? 0x81 : 0x00)};
            }
            return FakeBus::Result::succeeded;
        };
        if(address == MuxDev::Address) {
            // The switch: a write sets it, and it answers the engine's read-back with its
            // control byte -- which is not a probe of the scan's.
            if(!switchAnswers) { return FakeBus::Result::notAcknowledged; }
            if(switchFaults) { return FakeBus::Result::failed; }
            if(!sent.empty()) { switchValue = static_cast<std::uint8_t>(sent[0]); }
            for(auto& b : recv) { b = std::byte{switchValue}; }
            return FakeBus::Result::succeeded;
        }
        if(address == 0x44 && recv.size() == 6) { shtReads.push_back(switchValue); }
        switch(address) {
        case 0x23:
        case 0x60: return answer(true);   // in front of the switch
        case 0x44:
        case 0x50: return answer(open(2));
        case 0x61: return answer(open(5));
        default:   return answer(false);
        }
    }
};

struct Rig {
    Wiring                wiring{};
    std::unique_ptr<Wire> bus = std::make_unique<Wire>();
    MuxArbiter            arbiter{};
    std::unique_ptr<Scan> scan{};
    int                   turns{};

    Rig() {
        FakeBus::reset();
        FakeClock::current = FakeClock::time_point{} + 1s;
        Log::reset();
        FakeBus::respond = std::ref(wiring);
        auto& mux        = bus->get<MuxDev>();
        bus->forEach([&](auto& d) {
            using D = std::remove_cvref_t<decltype(d)>;
            if constexpr(D::GateT::Gated) { d.gate().bind(mux, arbiter); }
        });
        scan = std::make_unique<Scan>(*bus, arbiter);
    }

    /// One loop turn: the parts (or only the switch, before they start), then the scan.
    void turn(bool parts) {
        if(parts) {
            bus->handler();
        } else {
            bus->get<MuxDev>().handler();
        }
        scan->handler();
        FakeBus::complete();
        FakeClock::current += 1ms;
        ++turns;
    }

    bool runScan(bool parts,
                 int  maxTurns) {
        for(int i = 0; i < maxTurns; ++i) {
            if(scan->done()) { return true; }
            turn(parts);
        }
        return scan->done();
    }
};

void whileTheyRun() {
    testCase("BusScan: among running parts, every segment in order, their addresses left alone");
    Rig rig{};
    for(int i = 0; i < 300; ++i) { rig.turn(true); }
    check(rig.bus->get<Sht>().answering() && rig.bus->get<Light>().answering(), "the parts are up");
    auto const busyBefore  = rig.bus->get<Busy>().samples();
    auto const turnsBefore = rig.turns;
    rig.scan->start();
    check(rig.runScan(true, 30000), "the scan finished");
    auto const scanTurns = rig.turns - turnsBefore;
    auto const busyReads = rig.bus->get<Busy>().samples() - busyBefore;
    checkEq(rig.scan->scans(), 1U, "one scan");

    check(rig.scan->found(0, 0x60), "0x60 answers in front of the switch");
    check(rig.scan->found(3, 0x50) && !rig.scan->found(6, 0x50),
          "0x50 behind channel 2, and only there");
    check(!rig.scan->found(3, 0x60),
          "an address that answers in front is not probed again behind a channel");
    checkEq(rig.scan->others(), std::size_t{2}, "two devices that are not parts of the Bus");

    // A part's address is never probed where the part is: in front of the switch at any time
    // (the front is on every segment), behind a channel while that channel is open. With the
    // switch closed an address owned behind a channel is only wire, and may be probed.
    bool owned = false;
    for(auto const& p : rig.wiring.probes) {
        bool const o = p.address == 0x23 || p.address == MuxDev::Address
                    || (p.address == 0x44 && (p.switchValue & 0x04) != 0)
                    || (p.address == 0x61 && (p.switchValue & 0x20) != 0);
        if(o && !owned) {
            std::printf("    a probe of %02x with the switch at %02x\n", p.address, p.switchValue);
        }
        owned = owned || o;
    }
    check(!owned, "no probe of a part's address where the part is");

    std::size_t         lastSegment = 0;
    bool                inOrder     = true;
    std::array<bool, 9> seen{};
    for(auto const& p : rig.wiring.probes) {
        auto const segment = p.switchValue == 0
                             ? std::size_t{0}
                             : 1U + static_cast<std::size_t>(std::countr_zero(p.switchValue));
        inOrder            = inOrder && std::popcount(p.switchValue) <= 1 && segment >= lastSegment;
        lastSegment        = segment;
        seen[segment]      = true;
    }
    check(inOrder, "every probe went out with the switch on the segment being scanned, in order");
    check(std::all_of(seen.begin(), seen.end(), [](bool s) { return s; }),
          "and every segment was probed");

    bool shtOpen = !rig.wiring.shtReads.empty();
    for(auto const v : rig.wiring.shtReads) { shtOpen = shtOpen && v == 0x04; }
    check(shtOpen, "every SHT4x result was read with its channel open");
    checkEq(rig.bus->get<Sht>().errors(), 0U, "the SHT4x lost nothing to the scan");
    check(busyReads * 10U >= static_cast<unsigned>(scanTurns),
          "the always-due part kept reading through it");
    if(failures != 0) {
        std::printf("    %d turns, %zu probes, %u reads of the always-due part\n",
                    scanTurns,
                    rig.wiring.probes.size(),
                    busyReads);
    }
}

void atBoot() {
    testCase("BusScan: at boot, before the parts start, every address -- the parts' too");
    Rig rig{};
    rig.scan->start(Scan::Owned::probe);
    check(rig.runScan(false, 30000), "the scan finished");
    check(rig.scan->found(0, 0x23) && rig.scan->found(3, 0x44) && rig.scan->found(6, 0x61),
          "each part answers where the Bus has it");
    check(!rig.scan->found(6, 0x44) && !rig.scan->found(3, 0x61), "and nowhere else");
    check(!rig.scan->switchless(), "the switch was there");
}

void withoutTheSwitch() {
    testCase("BusScan: a switch that does not answer: the front is scanned, the channels are not");
    Rig rig{};
    rig.wiring.switchAnswers = false;
    rig.scan->start(Scan::Owned::probe);
    check(rig.runScan(false, 30000), "the scan finished");
    check(rig.scan->switchless(), "and says the switch was not there");
    check(rig.scan->found(0, 0x60) && !rig.scan->found(3, 0x50), "the front only");
}

/// A scan that gives up on a switch sooner than ScanDefaults.
struct QuickScan {
    static constexpr auto SelectTimeout = std::chrono::milliseconds{50};
};

static_assert(BusScan<Wire,
                      NoHints,
                      AddressProbe,
                      QuickScan>::SelectTimeout
                  == 50ms
                && Scan::SelectTimeout == 500ms,
              "a Config's SelectTimeout is the scan's, ScanDefaults otherwise");

void aSwitchThatIsNeverSet() {
    testCase(
      "BusScan: a switch that faults is never parked and never set: given up on after "
      "selectTimeout");
    Rig rig{};
    rig.wiring.switchFaults = true;
    rig.scan->selectTimeout(50ms);
    auto const t0 = FakeClock::now();
    rig.scan->start(Scan::Owned::probe);
    check(rig.runScan(false, 30000), "the scan finished");
    check(rig.scan->switchless(), "and counts the switch as not there");
    check(!rig.bus->get<MuxDev>().absent(), "though it was never parked: faults are not NAKs");
    check(FakeClock::now() - t0 < 400ms,
          "well inside the default 500 ms: the timeout set at run time");
    check(rig.scan->found(0, 0x60) && !rig.scan->found(3, 0x50), "the front only");
}

void plainBus() {
    testCase("BusScan: a Bus without a switch is one segment");
    FakeBus::reset();
    FakeClock::current = FakeClock::time_point{} + 1s;
    Log::reset();
    Wiring wiring{};
    FakeBus::respond = std::ref(wiring);
    using Plain      = Kvasir::I2C::Bus<FakeBus, FakeClock, Light>;
    static_assert(Plain::Segments == 1 && std::is_void_v<Plain::SwitchDevice>);
    auto           bus = std::make_unique<Plain>();
    BusScan<Plain> scan{*bus};
    scan.start();
    for(int i = 0; i < 2000 && !scan.done(); ++i) { turn(*bus, scan); }
    check(scan.done() && scan.found(0, 0x60) && !scan.found(0, 0x23),
          "0x60 found, the part's 0x23 left alone");
}

/// A second switch at 0x74, and a part behind it at another address.
struct MuxBConfig {
    static constexpr Address7 Address = 0x74;
};

using MuxB = Device<FakeBus, FakeClock, Chips::Tca9548a, MuxBConfig>;

struct BusyChipB : BusyChip {
    static constexpr std::string_view Name    = "BUSYB";
    static constexpr Address7         Address = 0x62;
};

using BusyB   = Device<FakeBus, FakeClock, BusyChipB, DefaultConfig, NoReset, MuxGate<MuxB, 1>>;
using TwoWire = Kvasir::I2C::Bus<FakeBus, FakeClock, MuxDev, MuxB, Light, Sht, Busy, BusyB>;
using TwoScan = BusScan<TwoWire>;

static_assert(TwoWire::Switches == 2 && TwoWire::Segments == 17
              && TwoWire::segmentOf<BusyB>() == 10);
static_assert(std::is_same_v<TwoWire::SwitchDeviceAt<0>,
                             MuxDev>
              && std::is_same_v<TwoWire::SwitchDeviceAt<1>,
                                MuxB>);

/// Two switches: 0x44/0x50 behind A2, 0x61 behind A5, 0x62/0x52 behind B1, and 0x55 behind
/// both A3 and B3.
struct TwoSwitchWiring {
    struct Probe {
        std::uint8_t address{};
        std::uint8_t a{};
        std::uint8_t b{};
    };

    std::uint8_t       a{};
    std::uint8_t       b{};
    bool               bAnswers{true};
    std::vector<Probe> probes{};

    FakeBus::Result operator()(std::uint8_t               address,
                               std::span<std::byte const> sent,
                               std::span<std::byte>       recv) {
        bool const probe  = sent.empty() && recv.size() == 1;
        auto const answer = [&](bool present) {
            if(probe) { probes.push_back({address, a, b}); }
            if(!present) { return FakeBus::Result::notAcknowledged; }
            for(std::size_t i = 0; i < recv.size(); ++i) {
                recv[i] = std::byte{static_cast<std::uint8_t>(i % 3 == 2 ? 0x81 : 0x00)};
            }
            return FakeBus::Result::succeeded;
        };
        if(address == MuxDev::Address) {
            if(!sent.empty()) { a = static_cast<std::uint8_t>(sent[0]); }
            for(auto& r : recv) { r = std::byte{a}; }   // the read-back, not a probe
            return FakeBus::Result::succeeded;
        }
        if(address == MuxBConfig::Address) {
            if(!bAnswers) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) { b = static_cast<std::uint8_t>(sent[0]); }
            for(auto& r : recv) { r = std::byte{b}; }
            return FakeBus::Result::succeeded;
        }
        auto const onA = [&](unsigned ch) { return ((a >> ch) & 1U) != 0; };
        auto const onB = [&](unsigned ch) { return ((b >> ch) & 1U) != 0; };
        switch(address) {
        case 0x23:
        case 0x60: return answer(true);
        case 0x44:
        case 0x50: return answer(onA(2));
        case 0x61: return answer(onA(5));
        case 0x52:
        case 0x62: return answer(onB(1));
        case 0x55: return answer(onA(3) || onB(3));
        default:   return answer(false);
        }
    }
};

struct TwoRig {
    TwoSwitchWiring          wiring{};
    std::unique_ptr<TwoWire> bus = std::make_unique<TwoWire>();
    MuxArbiter               arbiterA{};
    MuxArbiter               arbiterB{};
    std::unique_ptr<TwoScan> scan{};

    TwoRig() {
        FakeBus::reset();
        FakeClock::current = FakeClock::time_point{} + 1s;
        Log::reset();
        FakeBus::respond = std::ref(wiring);
        bus->forEach([&](auto& d) {
            using D = std::remove_cvref_t<decltype(d)>;
            if constexpr(D::GateT::Gated) {
                if constexpr(std::is_same_v<typename D::GateT::Mux, MuxDev>) {
                    d.gate().bind(bus->get<MuxDev>(), arbiterA);
                } else {
                    d.gate().bind(bus->get<MuxB>(), arbiterB);
                }
            }
        });
        scan = std::make_unique<TwoScan>(*bus, arbiterA, arbiterB);
    }

    bool runScan(bool parts,
                 int  maxTurns) {
        for(int i = 0; i < maxTurns && !scan->done(); ++i) { turn(parts); }
        return scan->done();
    }

    void turn(bool parts) {
        if(parts) {
            bus->handler();
        } else {
            bus->get<MuxDev>().handler();
            bus->get<MuxB>().handler();
        }
        scan->handler();
        FakeBus::complete();
        FakeClock::current += 1ms;
    }
};

void twoSwitches() {
    testCase("BusScan: two switches, every channel of each probed with the other switch shut");
    TwoRig rig{};
    for(int i = 0; i < 300; ++i) { rig.turn(true); }
    check(rig.bus->get<BusyB>().answering() && rig.bus->get<Sht>().answering(), "the parts are up");
    rig.scan->start();
    check(rig.runScan(true, 60000), "the scan finished");
    check(!rig.scan->switchless(), "both switches were there");
    check(rig.scan->found(3, 0x50) && rig.scan->found(10, 0x52), "0x50 behind A2, 0x52 behind B1");
    check(!rig.scan->found(10, 0x50) && !rig.scan->found(3, 0x52), "and each only there");
    check(rig.scan->found(4, 0x55) && rig.scan->found(12, 0x55),
          "an address behind a channel of each is found on both");
    checkEq(rig.scan->others(), std::size_t{5}, "0x60, 0x50, 0x52, and 0x55 twice");

    bool                 oneOpen = true;
    bool                 owned   = false;
    std::array<bool, 17> seen{};
    for(auto const& p : rig.wiring.probes) {
        auto const open    = std::popcount(p.a) + std::popcount(p.b);
        oneOpen            = oneOpen && open <= 1;
        auto const segment = open == 0 ? std::size_t{0}
                           : p.a != 0  ? 1U + static_cast<std::size_t>(std::countr_zero(p.a))
                                       : 9U + static_cast<std::size_t>(std::countr_zero(p.b));
        seen[segment]      = true;
        owned              = owned || p.address == 0x23 || p.address == MuxDev::Address
                          || p.address == MuxBConfig::Address || (p.address == 0x44 && (p.a & 0x04) != 0)
                          || (p.address == 0x61 && (p.a & 0x20) != 0)
                          || (p.address == 0x62 && (p.b & 0x02) != 0);
    }
    check(oneOpen, "no probe went out with more than one channel open across both switches");
    check(std::all_of(seen.begin(), seen.end(), [](bool s) { return s; }),
          "every segment of both was probed");
    check(!owned, "no probe of a part's address where the part is");
}

void aMissingSecondSwitch() {
    testCase("BusScan: a second switch that does not answer leaves only its own channels out");
    TwoRig rig{};
    rig.wiring.bAnswers = false;
    rig.scan->start(TwoScan::Owned::probe);
    check(rig.runScan(false, 60000), "the scan finished");
    checkEq(rig.scan->missingSwitches(), std::uint32_t{2}, "switch 1 was not there");
    check(rig.scan->found(0, 0x60) && rig.scan->found(3, 0x50) && rig.scan->found(6, 0x61),
          "the front and the first switch's channels were scanned");
    check(!rig.scan->found(10, 0x52) && !rig.scan->found(10, 0x62), "the second switch's were not");
}

}   // namespace

int main() {
    whileTheyRun();
    atBoot();
    withoutTheSwitch();
    aSwitchThatIsNeverSet();
    plainBus();
    twoSwitches();
    aMissingSecondSwitch();
    return finish();
}
