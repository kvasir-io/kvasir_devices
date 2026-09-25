/// Parts behind a bridge that is not always active (Bridge.hpp): nothing is said to them while
/// it is off, none of it counts against them, and they go on by themselves when it is back --
/// brought up again where they lost their supply, carrying on where they did not.
#include "Harness.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/I2C/Bridge.hpp>
#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Mux.hpp>
#include <kvasir/Devices/I2C/chips/Bh1750.hpp>
#include <kvasir/Devices/I2C/chips/Pcf8574.hpp>
#include <kvasir/Devices/I2C/chips/Sht3x.hpp>
#include <kvasir/Devices/I2C/chips/Tca9548a.hpp>
#include <memory>
#include <span>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

/// An enable pin, and what was done to it.
template<typename Tag>
struct Pin {
    static inline std::vector<bool> driven{};
    static inline bool              isOn{false};

    static void drive(bool active) {
        driven.push_back(active);
        isOn = active;
    }

    static void clear() {
        driven.clear();
        isOn = false;
    }
};

struct Cut : Pin<Cut> {
    static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
    static constexpr auto Settle   = 20ms;
};

struct Split : Pin<Split> {
    static constexpr auto WhileOff = Kvasir::I2C::WhileOff::disconnected;
    static constexpr auto Settle   = 5ms;
};

/// A port something else plugs and unplugs.
struct Plug {
    static inline bool plugged{false};

    static bool active() { return plugged; }
};

struct Dock : Kvasir::I2C::SensedBridge<Plug> {
    static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
    static constexpr auto Settle   = 50ms;
};

using CutLight   = BehindBridge<Cut, Dev<Chips::Bh1750>>;
using SplitLight = BehindBridge<Split, Dev<Chips::Bh1750>>;
using DockLight  = BehindBridge<Dock, Dev<Chips::Bh1750>>;
using CutSwitch  = BehindBridge<Cut, Dev<Chips::Tca9548a>>;

static_assert(CutLight::Bridged && !Dev<Chips::Bh1750>::Bridged);
static_assert(sizeof(Dev<Chips::Bh1750>)
                == sizeof(Kvasir::I2C::Device<FakeBus,
                                              FakeClock,
                                              Chips::Bh1750>),
              "a device that is behind no bridge pays nothing for them");

std::size_t transactionsTo(std::uint8_t address,
                           std::size_t  from = 0) {
    std::size_t n = 0;
    for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
        if(FakeBus::log[i].isBus() && FakeBus::log[i].address == address) { ++n; }
    }
    return n;
}

/// The Init of a BH1750 starts with its power-on command.
std::size_t powerOns(std::size_t from = 0) {
    std::size_t n = 0;
    for(auto const& w : writes(from)) {
        if(w == std::vector<std::uint8_t>{0x01}) { ++n; }
    }
    return n;
}

template<typename Line>
void driven(bool unpowered) {
    using B     = typename Line::Bridge;
    using Light = BehindBridge<B, Dev<Chips::Bh1750>>;
    Line  line{};
    Light light{};
    bool  talkedWhileOff = false;

    fresh();
    B::clear();
    light.gate().bind(line);
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(!B::isOn) {
            talkedWhileOff = true;
            return FakeBusResult::notAcknowledged;
        }
        return zeros(a, s, r);
    };
    auto both = all(line, light);

    testCase(unpowered
               ? "Bridge (unpowered): off from the start, the part is offline and left alone"
               : "Bridge (disconnected): off from the start, the part is offline");
    runFor(both, 5s);
    check(B::driven == std::vector<bool>{false}, "the pin is driven off once, and that is all");
    checkEq(transactionsTo(Light::Address), std::size_t{0}, "nothing is said to the part");
    check(light.link() == Link::offline, "it says offline");
    check(!light.absent() && !light.valid(), "which is not absent, and not valid");
    checkEq(light.errors(), 0U, "no error is counted");
    checkEq(light.consecutiveNaks(), 0U, "and no NAK");

    testCase("Bridge: on, the part waits for Settle and comes up");
    line.on();
    auto const onAt = FakeClock::now();
    check(runUntil(both, [&] { return transactionsTo(Light::Address) != 0; }, 1s), "it talks");
    check(FakeClock::now() - onAt >= Line::Settle, "not before the bridge has settled");
    check(runUntil(both, [&] { return light.valid(); }, 2s), "and delivers");
    check(light.link() == Link::answering, "answering");
    checkEq(light.bringUps(), 1U, "through one bring-up");
    checkEq(line.activations(), 1U, "one activation");

    testCase("Bridge: off, offline at once and silent");
    auto const samples = light.samples();
    line.off();
    check(light.link() == Link::offline, "offline with the call, not a turn later");
    runFor(both, 50ms);
    check(line.state() == BridgeState::off && !B::isOn, "the pin followed");
    auto const mark = FakeBus::log.size();
    runFor(both, 5s);
    checkEq(transactionsTo(Light::Address, mark), std::size_t{0}, "nothing more is said");
    check(!light.valid(), "the reading is not valid any more");
    checkEq(light.samples(), samples, "and no sample was made up");
    check(!light.absent(), "not absent");
    checkEq(light.errors(), 0U, "no errors");

    testCase(unpowered ? "Bridge (unpowered): back, the part is brought up from the start"
                       : "Bridge (disconnected): back, the part carries on without a bring-up");
    auto const back = FakeBus::log.size();
    line.on();
    check(runUntil(both, [&] { return light.valid(); }, 2s), "a new sample arrives");
    if(unpowered) {
        checkEq(light.bringUps(), 2U, "a second bring-up");
        checkEq(powerOns(back), std::size_t{1}, "with its Init on the wire again");
    } else {
        checkEq(light.bringUps(), 1U, "no second bring-up");
        checkEq(powerOns(back), std::size_t{0}, "and no Init on the wire");
    }
    check(light.link() == Link::answering, "answering");

    testCase("Bridge: off() while a transaction is on the wire does not cut it");
    // A turn of the handlers without the bus completing, until a request is pending.
    bool pendingSeen = false;
    for(int i = 0; i < 1000 && !pendingSeen; ++i) {
        line.handler();
        light.handler();
        pendingSeen = !FakeBus::pending.empty();
        if(!pendingSeen) {
            FakeBus::complete();
            FakeClock::current += 1ms;
        }
    }
    check(pendingSeen, "a transaction is on the wire");
    line.off();
    for(int i = 0; i < 5; ++i) {
        line.handler();
        light.handler();
        FakeClock::current += 1ms;
    }
    check(line.state() == BridgeState::closing && B::isOn, "the bridge waits for it");
    check(light.link() == Link::offline, "while the part is offline already");
    FakeBus::complete();
    runFor(both, 10ms);
    check(line.state() == BridgeState::off && !B::isOn, "and goes when the wire is free");
    checkEq(light.errors(), 0U, "no error from any of it");

    testCase("Bridge: nothing was ever said while the pin was off");
    check(!talkedWhileOff, "not one transaction");
}

void sensed() {
    BridgeLine<FakeClock, Dock> line{};
    DockLight                   light{};

    fresh();
    Plug::plugged = false;
    light.gate().bind(line);
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(!Plug::plugged) { return FakeBusResult::notAcknowledged; }
        return zeros(a, s, r);
    };
    auto both = all(line, light);

    testCase("Bridge (sensed): plugged in, the part comes up after Settle");
    runFor(both, 1s);
    check(light.link() == Link::offline, "offline while nothing is plugged");
    Plug::plugged = true;
    runFor(both, 10ms);
    Plug::plugged = false;   // a bounce
    runFor(both, 10ms);
    Plug::plugged = true;
    auto const at = FakeClock::now();
    check(runUntil(both, [&] { return line.active(); }, 1s), "active");
    check(FakeClock::now() - at >= 50ms, "a whole Settle after the last bounce");
    check(runUntil(both, [&] { return light.valid(); }, 2s), "the part delivers");

    testCase("Bridge (sensed): pulled in the middle of a transaction, and nothing is counted");
    bool pendingSeen = false;
    for(int i = 0; i < 1000 && !pendingSeen; ++i) {
        line.handler();
        light.handler();
        pendingSeen = !FakeBus::pending.empty();
        if(!pendingSeen) {
            FakeBus::complete();
            FakeClock::current += 1ms;
        }
    }
    check(pendingSeen, "a transaction is on the wire");
    Plug::plugged = false;
    runFor(both, 5s);   // the first turn completes it: a NAK
    check(light.link() == Link::offline, "offline");
    checkEq(light.errors(), 0U, "the NAK is not the part's");
    checkEq(light.consecutiveNaks(), 0U, "and starts no streak");
    check(!light.absent(), "not absent");

    testCase("Bridge (sensed): plugged again, a bring-up");
    Plug::plugged = true;
    check(runUntil(both, [&] { return light.valid(); }, 2s), "back");
    checkEq(light.bringUps(), 2U, "through a second bring-up");
}

void owed() {
    BridgeLine<FakeClock, Cut> line{};
    CutSwitch                  mux{};

    fresh();
    Cut::clear();
    mux.gate().bind(line);
    FakeBus::respond = zeros;
    auto both        = all(line, mux);

    testCase("Bridge: what the application sets while the part is offline is sent when it is back");
    mux.set<Chips::Tca9548a::Channels>(0x04);
    runFor(both, 1s);
    checkEq(transactionsTo(CutSwitch::Address), std::size_t{0}, "nothing goes out");
    check(mux.pending<Chips::Tca9548a::Channels>(), "the write stays owed");
    line.on();
    check(runUntil(both, [&] { return !mux.pending<Chips::Tca9548a::Channels>(); }, 1s), "sent");
    check(hasWrite({0x04}), "the control byte is on the wire");
}

void unbound() {
    CutLight light{};

    fresh();
    FakeBus::respond = zeros;

    testCase("Bridge: a device never bound to its line stays offline");
    runFor(light, 1s);
    check(light.link() == Link::offline, "offline");
    checkEq(transactionsTo(CutLight::Address), std::size_t{0}, "silent");
}

// -- in a Bus ---------------------------------------------------------------------------------

using Front    = Dev<Chips::Bh1750>;
using FarLight = BehindBridge<Cut, Dev<Chips::Bh1750, At<0x5C>>>;
using Mixed    = Kvasir::I2C::Bus<FakeBus, FakeClock, Front, FarLight>;

static_assert(Mixed::Bridges == 1 && Mixed::Switches == 0 && Mixed::Segments == 1);
static_assert(Kvasir::I2C::Bus<FakeBus,
                               FakeClock,
                               Front>::Bridges
              == 0);
static_assert(!detail::addressesDistinct<Front,
                                         CutLight>(),
              "a bridge alone keeps nobody apart: in front of it and behind it meet when it is on");
static_assert(!detail::addressesDistinct<CutLight,
                                         SplitLight>(),
              "and neither do two bridges that may be on together");

void inABus() {
    Mixed bus{};

    fresh();
    Cut::clear();
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(a == FarLight::Address && !Cut::isOn) { return FakeBusResult::notAcknowledged; }
        return zeros(a, s, r);
    };

    testCase("Bridge in a Bus: the Bus owns the line, and an offline part is nobody's problem");
    check(runUntil(
            bus,
            [&] { return bus.get<Front>().valid(); },
            2s),
          "the part in front delivers");
    runFor(bus, 5s);
    checkEq(bus.offlineCount(), std::size_t{1}, "one part offline");
    checkEq(bus.absentCount(), std::size_t{0}, "none absent");
    checkEq(bus.answeringCount(), std::size_t{1}, "one answering");
    checkEq(bus.errors(), 0U, "no errors");
    check(bus.valid(), "the Bus is valid: the offline part is not meant to deliver");
    checkEq(bus.counts().offline[0], 1U, "the segment counts say so too");
    checkEq(transactionsTo(FarLight::Address), std::size_t{0}, "and nothing was said to it");

    testCase("Bridge in a Bus: bus.bridge<B>().on()");
    bus.bridge<Cut>().on();
    check(runUntil(bus, [&] { return bus.answeringCount() == 2; }, 2s), "both answer");
    check(!bus.valid() || bus.get<FarLight>().valid(), "valid waits for the part that is back");
    check(runUntil(bus, [&] { return bus.valid(); }, 2s), "and comes");
    checkEq(bus.offlineCount(), std::size_t{0}, "nobody offline");

    testCase("Bridge in a Bus: restart() leaves the bridge alone");
    bus.bridge<Cut>().off();
    runFor(bus, 50ms);
    auto const mark = FakeBus::log.size();
    bus.restart();
    runFor(bus, 2s);
    checkEq(transactionsTo(FarLight::Address, mark), std::size_t{0}, "still silent behind it");
    check(bus.get<FarLight>().offline(), "still offline");
}

// -- a bridge and a switch --------------------------------------------------------------------

using MuxDev = Dev<Chips::Tca9548a>;
using Left   = Device<FakeBus, FakeClock, Chips::Sht3x, DefaultConfig, NoReset, MuxGate<MuxDev, 0>>;
using Right  = BehindBridge<
  Cut,
  Device<FakeBus, FakeClock, Chips::Sht3x, DefaultConfig, NoReset, MuxGate<MuxDev, 3>>>;
using Both = Kvasir::I2C::Bus<FakeBus, FakeClock, MuxDev, Left, Right>;

static_assert(Both::Switches == 1 && Both::Bridges == 1);
static_assert(Both::segmentOf<Left>() == 1 && Both::segmentOf<Right>() == 4,
              "a bridge around a channel's gate leaves the part on its segment");
static_assert(Both::channelOf<Right>() == 3);

void behindAChannel() {
    auto        busPtr = std::make_unique<Both>();
    auto&       bus    = *busPtr;
    SwitchWire  wire{};
    std::size_t wrongChannel = 0;
    std::size_t whileOff     = 0;
    bool        ch3Selected  = false;

    fresh();
    Cut::clear();
    // Both parts are at 0x44: which one a transaction is for is the channel that is open.
    wire.behind = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        auto const control = wire.control[0x70];
        if(control == 0x08) {
            if(!Cut::isOn) {
                ++whileOff;
                return FakeBusResult::notAcknowledged;
            }
        } else if(control != 0x01) {
            ++wrongChannel;
        }
        return crcZeros(a, s, r);
    };
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        auto const result = wire(a, s, r);
        if(wire.control[0x70] == 0x08) { ch3Selected = true; }
        return result;
    };

    testCase("Bridge and switch: offline, the part never takes the switch");
    check(runUntil(bus, [&] { return bus.get<Left>().valid(); }, 5s), "channel 0 delivers");
    runFor(bus, 3s);
    check(!ch3Selected, "channel 3 was never opened for a part that is offline");
    check(bus.get<Right>().offline(), "which says so");

    testCase("Bridge and switch: on, both channels deliver, each on its own");
    bus.bridge<Cut>().on();
    check(runUntil(bus, [&] { return bus.get<Right>().valid(); }, 5s), "channel 3 delivers");
    auto const left = bus.get<Left>().samples();
    runFor(bus, 3s);
    check(bus.get<Left>().samples() > left, "channel 0 goes on");
    checkEq(wrongChannel, std::size_t{0}, "nothing was said with a wrong channel open");
    checkEq(whileOff, std::size_t{0}, "and nothing through the bridge while it was off");
    checkEq(bus.errors(), 0U, "no errors");
}

/// The switch itself behind the bridge, and a part on one of its channels.
using FarMux  = BehindBridge<Cut, Dev<Chips::Tca9548a>>;
using FarPart = BehindBridge<
  Cut,
  Device<FakeBus, FakeClock, Chips::Sht3x, DefaultConfig, NoReset, MuxGate<FarMux, 1>>>;
using FarBus = Kvasir::I2C::Bus<FakeBus, FakeClock, FarMux, FarPart>;

static_assert(FarBus::Switches == 1 && FarBus::Bridges == 1 && FarBus::segmentOf<FarPart>() == 2);

void switchBehindABridge() {
    auto        busPtr = std::make_unique<FarBus>();
    auto&       bus    = *busPtr;
    SwitchWire  wire{};
    std::size_t closed = 0;

    fresh();
    Cut::clear();
    wire.behind = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(wire.control[0x70] != 0x02) {
            ++closed;
            return FakeBusResult::notAcknowledged;
        }
        return crcZeros(a, s, r);
    };
    bool talkedWhileOff = false;
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(!Cut::isOn) {
            talkedWhileOff = true;
            return FakeBusResult::notAcknowledged;
        }
        return wire(a, s, r);
    };

    testCase(
      "Switch behind a bridge: the switch is brought up again, and its channel written again");
    bus.bridge<Cut>().on();
    check(runUntil(bus, [&] { return bus.get<FarPart>().valid(); }, 5s), "the part delivers");
    bus.bridge<Cut>().off();
    runFor(bus, 100ms);
    check(!Cut::isOn, "off");
    wire.control.clear();   // the switch lost its supply: every channel shut
    bus.bridge<Cut>().on();
    check(runUntil(bus, [&] { return bus.get<FarPart>().valid(); }, 5s), "the part delivers again");
    checkEq(closed, std::size_t{0}, "and never talked into a shut channel");
    check(!talkedWhileOff, "nor anybody while the bridge was off");
    checkEq(bus.errors(), 0U, "no errors");
}

// -- a bridge on a pin of another part ---------------------------------------------------------

using Expander = Dev<Chips::Pcf8574>;

struct ViaPort
  : Kvasir::I2C::PartBridge<Expander, Chips::Pcf8574::Port, 0x10, BridgePolarity::activeLow> {
    static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
    static constexpr auto Settle   = 10ms;
};

using PortLight = BehindBridge<ViaPort, Dev<Chips::Bh1750>>;
using PortBus   = Kvasir::I2C::Bus<FakeBus, FakeClock, PortLight, Expander>;

void throughAPart() {
    PortBus      bus{};
    std::uint8_t port       = 0xFF;   // the PCF8574 after power-on: every pin high, bridge off
    bool         talkedShut = false;

    fresh();
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(a == Expander::Address) {
            if(!s.empty()) { port = static_cast<std::uint8_t>(s[0]); }
            for(auto& b : r) { b = std::byte{port}; }
            return FakeBusResult::succeeded;
        }
        if((port & 0x10) != 0) {
            talkedShut = true;
            return FakeBusResult::notAcknowledged;
        }
        return zeros(a, s, r);
    };

    testCase("PartBridge: the enable is a write of the expander, and the part waits for it");
    runFor(bus, 1s);
    check((port & 0x10) != 0, "off: the pin is high");
    check(bus.get<PortLight>().offline(), "the part is offline");
    bus.bridge<ViaPort>().on();
    check(runUntil(bus, [&] { return bus.get<PortLight>().valid(); }, 2s), "on, and it delivers");
    check((port & 0x10) == 0, "the pin is low");
    checkEq(static_cast<unsigned>(port | 0x10), 0xFFU, "and the other pins were left alone");

    testCase("PartBridge: the expander starts over, and the parts behind the bridge with it");
    auto const bringUps = bus.get<PortLight>().bringUps();
    port                = 0xFF;   // what its reset does to the pins
    bus.get<Expander>().restart();
    check(
      runUntil(
        bus,
        [&] { return bus.get<PortLight>().bringUps() > bringUps && bus.get<PortLight>().valid(); },
        2s),
      "the part behind is brought up again");
    check((port & 0x10) == 0, "the enable was written again by the expander's bring-up");

    testCase("PartBridge: off");
    bus.bridge<ViaPort>().off();
    check(runUntil(bus, [&] { return (port & 0x10) != 0; }, 1s), "the pin goes high");
    check(!talkedShut, "and nothing was ever said into the shut bridge");
    checkEq(bus.errors(), 0U, "no errors");
}

// -- bridges of which one is active at a time ----------------------------------------------------

struct Slots {};

struct SlotA : Pin<SlotA> {
    using ExclusiveGroup           = Slots;
    static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
};

struct SlotB : Pin<SlotB> {
    using ExclusiveGroup           = Slots;
    static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
};

using LightA  = BehindBridge<SlotA, Dev<Chips::Bh1750>>;
using LightB  = BehindBridge<SlotB, Dev<Chips::Bh1750>>;
using SlotBus = Kvasir::I2C::Bus<FakeBus, FakeClock, LightA, LightB>;

static_assert(detail::addressesDistinct<LightA,
                                        LightB>(),
              "the same address behind two bridges of one ExclusiveGroup");
static_assert(SlotBus::Bridges == 2);

void oneAtATime() {
    SlotBus bus{};
    bool    bothOn = false;

    fresh();
    SlotA::clear();
    SlotB::clear();
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(!SlotA::isOn && !SlotB::isOn) { return FakeBusResult::notAcknowledged; }
        return zeros(a, s, r);
    };
    auto watched = [&](auto done, std::chrono::milliseconds within) {
        auto const until = FakeClock::now() + within;
        while(FakeClock::now() < until) {
            turn(bus);
            bothOn = bothOn || (SlotA::isOn && SlotB::isOn);
            if(done()) { return true; }
        }
        return false;
    };

    testCase("ExclusiveGroup: both wanted, one is on; it goes, the other comes");
    bus.bridge<SlotA>().on();
    bus.bridge<SlotB>().on();
    check(watched([&] { return bus.get<LightA>().valid(); }, 2s), "A delivers");
    watched([] { return false; }, 1s);
    check(bus.bridge<SlotB>().state() == BridgeState::off, "B waits");
    check(bus.get<LightB>().offline(), "its part offline");
    bus.bridge<SlotA>().off();
    check(watched([&] { return bus.get<LightB>().valid(); }, 2s), "A off, B delivers");
    check(bus.get<LightA>().offline(), "A's part offline");
    check(!bothOn, "and never were both pins on");
    checkEq(bus.errors(), 0U, "no errors");
}

// -- a bridge used as if it were a switch -------------------------------------------------------

struct Ports {};

struct PortA : Pin<PortA> {
    using ExclusiveGroup           = Ports;
    static constexpr auto Mode     = Kvasir::I2C::BridgeMode::switched;
    static constexpr auto WhileOff = Kvasir::I2C::WhileOff::disconnected;
    static constexpr auto Settle   = 2ms;
};

struct PortB : Pin<PortB> {
    using ExclusiveGroup           = Ports;
    static constexpr auto Mode     = Kvasir::I2C::BridgeMode::switched;
    static constexpr auto WhileOff = Kvasir::I2C::WhileOff::disconnected;
    static constexpr auto Settle   = 2ms;
};

using OnA = BehindBridge<PortA, Dev<Chips::Bh1750>>;
using OnB = BehindBridge<PortB, Dev<Chips::Bh1750>>;
/// In front of both, and not to be heard behind them: it talks while both are off. At an
/// address of its own -- it is on the wire all the time, whatever its gate.
using Before
  = Device<FakeBus, FakeClock, Chips::Bh1750, At<0x5C>, NoReset, BridgeFrontGate<PortA, PortB>>;
using BeforeAt23  = Device<FakeBus,
                           FakeClock,
                           Chips::Bh1750,
                           DefaultConfig,
                           NoReset,
                           BridgeFrontGate<PortA, PortB>>;
using SwitchedBus = Kvasir::I2C::Bus<FakeBus, FakeClock, Before, OnA, OnB>;

static_assert(detail::addressesDistinct<Before,
                                        OnA,
                                        OnB>(),
              "one address behind each of two switched bridges of a group");
static_assert(!detail::addressesDistinct<Front,
                                         OnA>()
                && !detail::addressesDistinct<BeforeAt23,
                                              OnA>(),
              "never in front as well, whatever gate that part has: it is on the wire all the "
              "time and would answer with the part behind the bridge");

/// The application's word instead (BridgeAddresses::separate).
struct Trusted : Pin<Trusted> {
    static constexpr auto WhileOff  = Kvasir::I2C::WhileOff::unpowered;
    static constexpr auto Addresses = Kvasir::I2C::BridgeAddresses::separate;
};

using TrustedLight = BehindBridge<Trusted, Dev<Chips::Bh1750>>;

static_assert(!BridgeLine<FakeClock,
                          Trusted>::Switched
              && BridgeLine<FakeClock,
                            PortA>::Switched);

static_assert(detail::addressesDistinct<CutLight,
                                        TrustedLight>(),
              "behind it and behind another");
static_assert(!detail::addressesDistinct<Front,
                                         TrustedLight>(),
              "but not behind it and behind nothing: that part is always on the wire");
static_assert(!detail::addressesDistinct<TrustedLight,
                                         TrustedLight>(),
              "but two behind it are two on one wire");

void asASwitch() {
    SwitchedBus bus{};
    std::size_t bothOn    = 0;
    std::size_t overheard = 0;

    fresh();
    PortA::clear();
    PortB::clear();
    // Two parts at 0x23: which one answers is where the bridges are. Each part says who it is.
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const>, std::span<std::byte> r) {
        if(PortA::isOn && PortB::isOn) { ++bothOn; }
        if(a == Before::Address && (PortA::isOn || PortB::isOn)) { ++overheard; }
        if(a != Before::Address && !PortA::isOn && !PortB::isOn) {
            return FakeBusResult::notAcknowledged;   // nobody there with both bridges off
        }
        std::uint8_t const who = a == Before::Address ? 0xF : PortA::isOn ? 0xA : 0xB;
        for(auto& b : r) { b = std::byte{who}; }
        return FakeBusResult::succeeded;
    };
    auto watched = [&](auto done, std::chrono::milliseconds within) {
        auto const until = FakeClock::now() + within;
        while(FakeClock::now() < until) {
            turn(bus);
            if(PortA::isOn && PortB::isOn) { ++bothOn; }
            if(done()) { return true; }
        }
        return false;
    };
    testCase("Switched bridges: nobody calls on(), every part delivers, each its own reading");
    check(watched(
            [&] {
                return bus.get<Before>().valid() && bus.get<OnA>().valid()
                    && bus.get<OnB>().valid();
            },
            5s),
          "all three deliver");
    watched([] { return false; }, 5s);
    checkEq(bus.get<Before>().latest().raw, 0x0F0FU, "the part in front delivers");
    checkEq(overheard, std::size_t{0}, "and never talked with a bridge on");
    checkEq(bus.get<OnA>().latest().raw, 0x0A0AU, "the part behind A through A");
    checkEq(bus.get<OnB>().latest().raw, 0x0B0BU, "the part behind B through B");
    checkEq(bothOn, std::size_t{0}, "and never were both bridges on");
    checkEq(bus.offlineCount(), std::size_t{0}, "switched away is a wait, not offline");
    checkEq(bus.get<OnA>().bringUps(), 1U, "one bring-up each, whatever the switching");
    checkEq(bus.get<OnB>().bringUps(), 1U, "");
    checkEq(bus.errors(), 0U, "no errors");
    check(bus.bridge<PortA>().switchOns() > 10 && bus.bridge<PortB>().switchOns() > 10,
          "the bridges took turns");
    std::printf("    samples in front %u, behind A %u, behind B %u; switch-ons A %u, B %u\n",
                static_cast<unsigned>(bus.get<Before>().samples()),
                static_cast<unsigned>(bus.get<OnA>().samples()),
                static_cast<unsigned>(bus.get<OnB>().samples()),
                static_cast<unsigned>(bus.bridge<PortA>().switchOns()),
                static_cast<unsigned>(bus.bridge<PortB>().switchOns()));
    auto const rate = [&](auto const& d) { return d.samples(); };
    check(rate(bus.get<OnA>()) >= 24 && rate(bus.get<OnB>()) >= 24 && rate(bus.get<Before>()) >= 24,
          "and nobody starved: 200 ms parts over five seconds, all at their nominal rate");

    testCase("Switched bridges: off() disables one, and its part is offline");
    bus.bridge<PortA>().off();
    check(watched([&] { return bus.get<OnA>().offline() && !PortA::isOn; }, 1s),
          "offline, pin off");
    auto const samplesA = bus.get<OnA>().samples();
    auto const samplesB = bus.get<OnB>().samples();
    watched([] { return false; }, 2s);
    checkEq(bus.get<OnA>().samples(), samplesA, "nothing read through a disabled bridge");
    check(bus.get<OnB>().samples() > samplesB, "the other goes on");
    bus.bridge<PortA>().on();
    check(watched([&] { return bus.get<OnA>().valid(); }, 2s), "on() enables it again");
    checkEq(bus.get<OnA>().bringUps(), 1U, "disconnected: no new bring-up");
    checkEq(bothOn, std::size_t{0}, "still never both");
}

}   // namespace

int main() {
    driven<BridgeLine<FakeClock, Cut>>(true);
    driven<BridgeLine<FakeClock, Split>>(false);
    sensed();
    owed();
    unbound();
    inABus();
    behindAChannel();
    switchBehindABridge();
    throughAPart();
    oneAtATime();
    asASwitch();
    return finish();
}
