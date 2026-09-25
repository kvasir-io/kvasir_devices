/// The PowerRail: a supply the parts of a Bus share -- off before anything is said, cycled for
/// a part that stays absent, every device started over with it, and a part that never comes
/// back not taking the others down at a fixed rate.
#include "Harness.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Mux.hpp>
#include <kvasir/Devices/I2C/PowerRail.hpp>
#include <kvasir/Devices/I2C/chips/Bh1750.hpp>
#include <kvasir/Devices/I2C/chips/Sht3x.hpp>
#include <kvasir/Devices/I2C/chips/Tca9548a.hpp>
#include <span>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

using Near = Dev<Chips::Bh1750>;
using Far  = Dev<Chips::Bh1750, At<0x5C>>;
using Pair = Kvasir::I2C::Bus<FakeBus, FakeClock, Near, Far>;

/// The switch, and what was done to it. `abandonBus` is the hook a firmware resets its bus
/// behavior in; here it drops what the fake bus still holds, and says where it came.
struct Supply {
    static inline std::vector<bool>     switched{};
    static inline bool                  isOn{false};
    static inline std::size_t           abandoned{};
    static inline bool                  abandonedWhileOn{true};
    static inline std::function<void()> onOff{};

    static void abandonBus() {
        ++abandoned;
        abandonedWhileOn = abandonedWhileOn && isOn;
        FakeBus::pending.clear();
    }

    static void off() {
        isOn = false;
        switched.push_back(false);
        if(onOff) { onOff(); }
    }

    static void on() {
        isOn = true;
        switched.push_back(true);
    }

    static void clear() {
        switched.clear();
        abandoned        = 0;
        abandonedWhileOn = true;
        onOff            = {};
    }
};

using Rail = PowerRail<FakeClock, Supply>;

/// Every value redeclared, none the default's: a lambda of PowerRail's that read the wrong
/// one would show.
struct FastConfig {
    static constexpr auto OffTime           = std::chrono::milliseconds{20};
    static constexpr auto SettleTime        = std::chrono::milliseconds{7};
    static constexpr auto AbsentBeforeCycle = std::chrono::milliseconds{1500};
    static constexpr auto CycleIntervalMax  = std::chrono::milliseconds{4000};
};

using FastRail = PowerRail<FakeClock, Supply, FastConfig>;

static_assert(FastRail::OffTime == 20ms && FastRail::SettleTime == 7ms
              && FastRail::AbsentBeforeCycle == 1500ms && FastRail::CycleIntervalMax == 4000ms);
static_assert(Rail::OffTime == PowerRailDefaults::OffTime
              && Rail::CycleIntervalMax == PowerRailDefaults::CycleIntervalMax);

/// A switch that claims its pin hands the claim to the rail; one that does not still is one.
struct ClaimingSupply : Supply {
    struct Claims {};
};

static_assert(std::is_same_v<PowerRail<FakeClock,
                                       ClaimingSupply>::Claims,
                             ClaimingSupply::Claims>);
template<typename T>
concept HasClaims = requires { typename T::Claims; };
static_assert(HasClaims<PowerRail<FakeClock,
                                  ClaimingSupply>>
              && !HasClaims<Rail>);

/// A Bus run through its rail, for the harness's turn().
template<typename R, typename B>
struct Powered {
    R& rail;
    B& bus;

    void handler() { rail.handler(bus); }
};

/// off, on, off, on ...: never the same edge twice.
bool alternates(std::vector<bool> const& edges) {
    for(std::size_t i = 0; i < edges.size(); ++i) {
        if(edges[i] != (i % 2 == 1)) { return false; }
    }
    return true;
}

std::size_t busEntries() {
    std::size_t n = 0;
    for(auto const& t : FakeBus::log) {
        if(t.isBus()) { ++n; }
    }
    return n;
}

void powerRail() {
    Pair    bus{};
    Rail    rail{};
    Powered powered{rail, bus};
    bool    farDead        = false;
    bool    talkedWhileOff = false;

    fresh();
    Supply::clear();
    Supply::isOn     = true;   // as a warm reset of the controller leaves it
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(!Supply::isOn) { talkedWhileOff = true; }
        if(farDead && a == Far::Address) { return FakeBusResult::notAcknowledged; }
        return zeros(a, s, r);
    };

    testCase("PowerRail: off first, then on, and nothing on the bus before it settled");
    runFor(powered, Rail::OffTime);
    check(Supply::switched == std::vector<bool>{false}, "the first thing it does is switch off");
    checkEq(busEntries(), std::size_t{0}, "and nothing is said while it is off");
    runFor(powered, Rail::SettleTime + 5ms);
    check((Supply::switched == std::vector<bool>{false, true}), "then on");
    check(runUntil(
            powered,
            [&] { return bus.answeringCount() == 2; },
            2s),
          "both parts come up behind it");
    check(rail.powered(), "the rail says so");
    checkEq(rail.cycles(), 0U, "the first power-on is not a cycle");

    testCase("PowerRail: a part that stays absent gets the rail cycled, and every device restarts");
    auto const nearBringUps = bus.get<Near>().bringUps();
    farDead                 = true;
    check(runUntil(powered, [&] { return bus.absentCount() == 1; }, 10s), "the far part is parked");
    checkEq(rail.cycles(), 0U, "which alone cycles nothing");
    auto const parkedAt = FakeClock::now();
    check(runUntil(powered, [&] { return rail.cycles() == 1; }, 10s), "the rail is cycled for it");
    check(FakeClock::now() - parkedAt >= Rail::AbsentBeforeCycle, "not before AbsentBeforeCycle");
    auto const firstCycleAt = FakeClock::now();
    check(!Supply::isOn && !rail.powered(), "off");
    check(!bus.get<Near>().answering(), "and the part that was fine starts over too");
    runFor(powered, Rail::OffTime + Rail::SettleTime + 5ms);
    check(Supply::isOn, "on again");
    check(runUntil(
            powered,
            [&] { return bus.get<Near>().answering(); },
            2s),
          "the near part is back");
    check(bus.get<Near>().bringUps() > nearBringUps, "through a bring-up of its own");

    testCase("PowerRail: a part that never comes back is cycled for at a growing interval");
    check(runUntil(powered, [&] { return rail.cycles() == 2; }, 60s), "a second cycle comes");
    check(FakeClock::now() - firstCycleAt >= 2 * Rail::AbsentBeforeCycle, "after twice the wait");
    auto const secondCycleAt = FakeClock::now();
    check(runUntil(powered, [&] { return rail.cycles() == 3; }, 120s), "and a third");
    check(FakeClock::now() - secondCycleAt >= 4 * Rail::AbsentBeforeCycle, "after four times");

    testCase("PowerRail: the part is back, the cycling stops and the wait is the first one again");
    farDead = false;
    check(runUntil(powered, [&] { return bus.answeringCount() == 2; }, 60s), "both answer");
    auto const cycles = rail.cycles();
    runFor(powered, 30s);
    checkEq(rail.cycles(), cycles, "no cycle while every part answers");

    testCase("PowerRail: cycle() cycles it whatever the parts say");
    rail.cycle();
    check(runUntil(powered, [&] { return rail.cycles() == cycles + 1; }, 1s), "on request");
    check(runUntil(powered, [&] { return bus.answeringCount() == 2; }, 5s), "and back");

    testCase("PowerRail: what was in flight is dropped before the rail goes");
    check(!talkedWhileOff, "no transaction ever went out while the rail was off");
    checkEq(Supply::abandoned, std::size_t{rail.cycles()}, "the bus is abandoned once per cycle");
    check(Supply::abandonedWhileOn, "while the rail is still on");
    check(alternates(Supply::switched), "and the switch only ever alternates");
    checkEq(Supply::switched.size(), std::size_t{2} + 2 * rail.cycles(), "one off and on a cycle");
}

void configured() {
    Pair     bus{};
    FastRail rail{};
    Powered  powered{rail, bus};
    bool     farDead = false;

    fresh();
    Supply::clear();
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(farDead && a == Far::Address) { return FakeBusResult::notAcknowledged; }
        return zeros(a, s, r);
    };

    testCase("PowerRail: a Config's own times");
    check(rail.state() == FastRail::State::off, "off from the start");
    runFor(powered, FastConfig::OffTime + 2ms);
    check(rail.state() == FastRail::State::settling, "OffTime, then settling");
    runFor(powered, FastConfig::SettleTime + 2ms);
    check(rail.state() == FastRail::State::on, "SettleTime, then on");
    check(runUntil(powered, [&] { return bus.answeringCount() == 2; }, 2s), "both parts up");

    testCase("PowerRail: the wait grows to CycleIntervalMax and no further");
    farDead = true;
    check(runUntil(powered, [&] { return rail.cycles() == 1; }, 20s), "first cycle");
    checkEq(rail.cycleInterval(), 3000ms, "doubled");
    check(runUntil(powered, [&] { return rail.cycles() == 2; }, 20s), "second");
    checkEq(rail.cycleInterval(), FastConfig::CycleIntervalMax, "clamped, not 6 s");
    check(runUntil(powered, [&] { return rail.cycles() == 3; }, 20s), "third");
    checkEq(rail.cycleInterval(), FastConfig::CycleIntervalMax, "and stays there");

    testCase("PowerRail: cycle() while the rail is off is that cycle, not one more");
    check(rail.state() == FastRail::State::off, "just cycled");
    rail.cycle();
    farDead = false;
    check(runUntil(powered, [&] { return bus.answeringCount() == 2; }, 20s), "both answer");
    checkEq(rail.cycles(), 3U, "no fourth cycle came of it");
    checkEq(rail.cycleInterval(), FastConfig::AbsentBeforeCycle, "and the wait is the first again");

    testCase(
      "PowerRail: after a recovery the next absence waits AbsentBeforeCycle, not the maximum");
    runFor(powered, 2 * FastConfig::AbsentBeforeCycle);
    farDead = true;
    check(runUntil(powered, [&] { return bus.absentCount() == 1; }, 10s), "parked again");
    auto const parkedAt = FakeClock::now();
    check(runUntil(powered, [&] { return rail.cycles() == 4; }, 20s), "cycled");
    check(FakeClock::now() - parkedAt < FastConfig::AbsentBeforeCycle + 100ms,
          "after the first wait");
}

// -- a switch on the rail -----------------------------------------------------------------

using MuxDev   = Dev<Chips::Tca9548a>;
using Gated    = Device<FakeBus,
                        FakeClock,
                        Chips::Sht3x,
                        DefaultConfig,
                        NoReset,
                        Kvasir::I2C::MuxGate<MuxDev, 0>>;
using Switched = Kvasir::I2C::Bus<FakeBus, FakeClock, MuxDev, Gated>;

void muxOnTheRail() {
    Switched bus{};
    FastRail rail{};
    Powered  powered{rail, bus};

    fresh();
    Supply::clear();
    SwitchWire wire{};
    bool       dead         = false;
    bool       behindClosed = false;
    wire.behind = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(dead) { return FakeBusResult::notAcknowledged; }
        // A part behind a closed channel is not on the wire.
        if(wire.control[0x70] != 0x01) {
            behindClosed = true;
            return FakeBusResult::notAcknowledged;
        }
        return crcZeros(a, s, r);
    };
    FakeBus::respond = std::ref(wire);
    Supply::onOff
      = [&] { wire.control.clear(); };   // the switch powers up with every channel closed

    testCase("PowerRail: a switch on the rail is written again before the part behind it talks");
    check(runUntil(powered, [&] { return bus.answeringCount() == 2; }, 5s), "switch and part up");
    dead = true;
    check(runUntil(
            powered,
            [&] { return rail.cycles() == 1; },
            20s),
          "the part dies, the rail cycles");
    dead         = false;
    behindClosed = false;
    check(runUntil(powered, [&] { return bus.answeringCount() == 2; }, 10s), "both back");
    check(!behindClosed, "and the part was never addressed through a closed channel");
}

}   // namespace

int main() {
    powerRail();
    configured();
    muxOnTheRail();
    return finish();
}
