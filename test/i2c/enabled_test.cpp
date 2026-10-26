/// Config::enabled() on its own: a part the application switches off (a module that is not fitted) is offline and
/// left alone, none of it counts against it, and it comes back by itself when enabled again - brought up from the
/// start unless `WhileDisabled = disconnected` says its registers survived. The resting enabled() cases are in
/// engine_test, enabled() together with a bridge in bridge_test.
#include "EnabledFixtures.hpp"
#include "Harness.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Bh1750.hpp>
#include <kvasir/Devices/I2C/chips/Tca9548a.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

using EnabledFixtures::Enabled;
using EnabledFixtures::EnabledKeeping;
using EnabledFixtures::Fitted;

struct EnabledSwitch {
    static bool enabled() { return Fitted::yes; }
};

using ModuleLight  = Dev<Chips::Bh1750, Enabled>;
using KeptLight    = Dev<Chips::Bh1750, EnabledKeeping>;
using ModuleSwitch = Dev<Chips::Tca9548a, EnabledSwitch>;

static_assert(ModuleLight::Enableable && ModuleLight::Switchable);
static_assert(!Dev<Chips::Bh1750>::Switchable);

std::size_t transactionsTo(std::uint8_t address,
                           std::size_t  from = 0) {
    std::size_t n = 0;
    for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
        if(FakeBus::log[i].isBus() && FakeBus::log[i].address == address) { ++n; }
    }
    return n;
}

void disabled() {
    I2cCase testcase{"enabled(): disabled from the start, the part is offline and left alone"};
    Fitted::yes      = false;
    FakeBus::respond = zeros;
    {
        ModuleLight light{};
        runFor(light, 2s);
        check(light.link() == Kvasir::Link::offline, "offline");
        checkEq(transactionsTo(ModuleLight::Address), std::size_t{0}, "silent");
        checkEq(light.errors(), 0U, "nothing counted");

        testcase.step("enabled(): enabled, the part comes up");
        Fitted::yes = true;
        check(runUntil(light, [&] { return light.valid(); }, 2s), "delivers");
        checkEq(light.bringUps(), 1U, "one bring-up");

        testcase.step(
          "enabled(): disabled mid-transaction, the transaction is waited for, not cut");
        bool pendingSeen = false;
        for(int i = 0; i < 1000 && !pendingSeen; ++i) {
            light.turn();
            pendingSeen = !FakeBus::pending.empty();
            if(!pendingSeen) {
                FakeBus::complete();
                FakeClock::current += 1ms;
            }
        }
        check(pendingSeen, "a transaction is on the wire");
        Fitted::yes = false;
        light.turn();
        check(!FakeBus::pending.empty(), "still on the wire: not abandoned");
        runFor(light, 1s);
        check(light.link() == Kvasir::Link::offline, "offline");
        checkEq(light.errors(), 0U, "nothing counted against the part");
        check(!light.valid(), "and not valid");

        testcase.step("enabled(): enabled again, brought up from the start (unpowered)");
        Fitted::yes = true;
        check(runUntil(light, [&] { return light.valid(); }, 2s), "back");
        checkEq(light.bringUps(), 2U, "through a second bring-up");
    }

    testcase.next("enabled(): WhileDisabled = disconnected keeps the bring-up");
    Fitted::yes      = true;
    FakeBus::respond = zeros;
    {
        KeptLight light{};
        check(runUntil(light, [&] { return light.valid(); }, 2s), "up");
        Fitted::yes = false;
        runFor(light, 1s);
        check(light.offline(), "offline");
        Fitted::yes = true;
        check(runUntil(light, [&] { return light.valid(); }, 2s), "back");
        checkEq(light.bringUps(), 1U, "no second bring-up");
    }

    testcase.next("enabled(): what is set while disabled is sent when enabled");
    Fitted::yes      = false;
    FakeBus::respond = zeros;
    {
        ModuleSwitch mux{};
        mux.set<Chips::Tca9548a::Channels>(0x04);
        runFor(mux, 1s);
        checkEq(transactionsTo(ModuleSwitch::Address), std::size_t{0}, "nothing goes out");
        Fitted::yes = true;
        check(runUntil(mux, [&] { return !mux.pending<Chips::Tca9548a::Channels>(); }, 1s), "sent");
        check(hasWrite({0x04}), "the control byte is on the wire");
    }

    testcase.next("enabled(): its transitions are logged, once each");
    Fitted::yes      = true;
    FakeBus::respond = zeros;
    {
        KeptLight light{};
        check(runUntil(light, [&] { return light.valid(); }, 2s), "up");
        auto       before = Kvasir::Test::Log::debugs;
        auto const lines  = [&] {
            auto const n = Kvasir::Test::Log::debugs - before;
            before       = Kvasir::Test::Log::debugs;
            return n;
        };
        Fitted::yes = false;
        runFor(light, 1s);
        checkEq(lines(), 1, "disabled: exactly one line");
        runFor(light, 1s);
        checkEq(lines(), 0, "and nothing more while it stays disabled");
        Fitted::yes = true;
        check(runUntil(light, [&] { return light.valid(); }, 2s), "back");
        check(lines() >= 1, "enabled: its line, then the bring-up's own");
    }
}

}   // namespace

int main() {
    disabled();
    return finish();
}
