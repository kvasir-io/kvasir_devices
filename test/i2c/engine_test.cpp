/// The engine on its own -- presence, the link state, tickets and writes, the read-back
/// verify, sized reads, multi-step writes -- against chips that exist only to exercise it,
/// and a few real descriptions where they show an engine behaviour best.
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
#include <kvasir/Devices/I2C/Watch.hpp>
#include <kvasir/Devices/I2C/chips/All.hpp>
#include <map>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

// -- presence: parked after NAKs, probed, back at once; the nets under a script ------------

namespace PresenceTest {

    /// The smallest chip: one register read at bring-up, one cyclic read.
    struct Probe {
        static constexpr std::string_view Name          = "PROBE";
        static constexpr Address7         Address       = 0x21;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        using Reads = List<Value>;
    };

    struct NeverPark {
        static constexpr std::uint8_t AbsentAfterNaks = 0;
    };

    /// Probe without an Init: nothing is sent at bring-up, so its first read is what answers.
    struct NoInit {
        static constexpr std::string_view Name          = "NOINIT";
        static constexpr Address7         Address       = 0x22;
        static constexpr std::size_t      RegisterBytes = 1;

        using Value = Probe::Value;
        using Reads = List<Value>;
    };

    struct ShortNet {
        static constexpr auto InFlightTimeout = std::chrono::milliseconds{50};
    };

    /// Transactions to `address` whose first sent byte is `reg`: how often a register was
    /// asked for.
    std::size_t readsOf(std::uint8_t address,
                        std::uint8_t reg) {
        std::size_t n = 0;
        for(auto const& t : FakeBus::log) {
            if(t.isBus() && t.address == address && !t.sent.empty() && t.sent[0] == reg) { ++n; }
        }
        return n;
    }

}   // namespace PresenceTest

void presence() {
    using namespace PresenceTest;

    testCase("presence: three NAKs park the device, and nothing goes out while parked");
    fresh();
    FakeBus::respond = alwaysNak;
    Dev<Probe> d{};
    check(runUntil(d, [&] { return d.absent(); }, 100ms), "parked");
    checkEq(d.consecutiveNaks(), 3U, "after three NAKs");
    checkEq(Log::warnings, 1, "said once");
    {
        auto const sent = FakeBus::log.size();
        runFor(d, 500ms);   // half a second: before the first probe
        checkEq(FakeBus::log.size(), sent, "no transaction while parked");
        check(!d.answering() && !d.valid(), "and not up");
    }

    testCase("presence: probes at 1, 2, 4, 8, 16, then every 30 s");
    {
        // From here on the clock steps 100 ms a turn; a probe is one transaction, NAKed.
        std::vector<std::size_t> probeTurns;
        auto                     seen = FakeBus::log.size();
        for(std::size_t t = 0; t < 1200; ++t) {   // 120 s
            d.handler();
            FakeBus::complete();
            FakeClock::current += 100ms;
            if(FakeBus::log.size() != seen) {
                checkEq(FakeBus::log.size() - seen, std::size_t{1}, "one transaction per probe");
                seen = FakeBus::log.size();
                probeTurns.push_back(t);
            }
        }
        std::vector<std::size_t> gaps;
        for(std::size_t i = 1; i < probeTurns.size(); ++i) {
            gaps.push_back(probeTurns[i] - probeTurns[i - 1]);
        }
        check(gaps == std::vector<std::size_t>{10, 20, 40, 80, 160, 300, 300},
              "1, 2, 4, 8, 16, 32, 62, 92 s: the interval doubles to the 30 s cap");
        check(d.absent(), "still parked");
        checkEq(Log::warnings, 1, "and still said once");
    }

    testCase("presence: an ACK unparks at once, and the next turns are ordinary");
    {
        FakeBus::respond = zeros;
        auto const infos = Log::infos;
        // The next probe is up to 30 s away: step to it.
        check(runUntil(d, [&] { return !d.absent(); }, 31s), "present again");
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "and sampling");
        checkEq(d.consecutiveNaks(), 0U, "streak cleared");
        check(Log::infos >= infos + 2, "'present again' and 'up' were logged");
        checkEq(Log::warnings, 1, "no new warning");
    }

    testCase("presence: bus faults do not park; a NAK streak survives a fault between");
    fresh();
    FakeBus::respond = alwaysFault;
    Dev<Probe> f{};
    runFor(f, 300ms);
    check(!f.absent(), "still present after nothing but bus faults");
    check(f.errors() > 5, "which were all counted");
    checkEq(Log::warnings, 0, "and none of them was 'not responding'");
    {
        int  n      = 0;
        auto script = [&](std::uint8_t, std::span<std::byte const>, std::span<std::byte>) {
            ++n;
            return n == 2 ? FakeBus::Result::failed : FakeBus::Result::notAcknowledged;
        };
        FakeBus::respond = script;
        Dev<Probe> e{};
        check(runUntil(e, [&] { return e.absent(); }, 100ms), "parked: NAK, fault, NAK, NAK");
        checkEq(n, 4, "in four transactions");
    }

    testCase("presence: AbsentAfterNaks = 0 never parks");
    fresh();
    FakeBus::respond = alwaysNak;
    Dev<Probe, NeverPark> np{};
    runFor(np, 300ms);
    check(!np.absent(), "present through 300 NAKs");
    checkEq(Log::warnings, 0, "nothing said");

    testCase("presence: a parked chip with no Init is up again only once it has answered");
    fresh();
    FakeBus::respond = alwaysNak;
    {
        Dev<NoInit> n{};
        check(runUntil(n, [&] { return n.absent(); }, 1s), "parked");
        auto const infos = Log::infos;   // power-up said "up": nothing had failed yet
        runFor(n, 70s);                  // 70 s: seven probes
        check(!n.answering() && !n.valid(), "never up while nothing answers");
        checkEq(Log::infos, infos, "and never said 'up'");
        FakeBus::respond = zeros;
        check(runUntil(n, [&] { return n.answering(); }, 31s), "up at the first ACK");
        check(!n.absent(), "and present");
        check(runUntil(n, [&] { return n.valid(); }, 500ms), "and sampling");
    }

    testCase("presence: a flapping device does not flood the log");
    fresh();
    {
        // Answers exactly one probe, then NAKs again -- twenty times.
        bool answer = false;
        FakeBus::respond
          = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
                return answer ? zeros(a, s, r) : alwaysNak(a, s, r);
            };
        Dev<Probe> flap{};
        for(int i = 0; i < 20; ++i) {
            answer = false;
            // three cyclic reads at 100 ms, NAKed: parked within half a second
            check(runUntil(flap, [&] { return flap.absent(); }, 1s), "parked");
            answer = true;
            check(runUntil(flap, [&] { return !flap.absent(); }, 31s), "probed and back");
        }
        // Twenty flaps in some 25 s. Every return logs "up" (one info each); what the
        // presence limiter lets through on top -- "not responding" and "present again" share
        // one backoff schedule -- is a handful, not forty.
        auto const presenceLines = Log::warnings + Log::infos - 20;
        check(Log::warnings >= 1, "the parking line was said");
        check(presenceLines >= 2 && presenceLines <= 8, "and the presence lines are rate-limited");
    }

    testCase("engine: a request the bus never answers is given up on after InFlightTimeout");
    fresh();
    FakeBus::respond = zeros;
    Dev<Probe, ShortNet> lost{};
    check(runUntil(lost, [&] { return lost.valid(); }, 500ms), "up");
    {
        // Wait for a turn that submits, then drop the request: no callback, ever.
        bool dropped = false;
        for(int i = 0; i < 200 && !dropped; ++i) {
            lost.handler();
            if(!FakeBus::pending.empty()) {
                FakeBus::pending.clear();
                dropped = true;
            }
            FakeClock::current += 1ms;
        }
        check(dropped, "a request was dropped");
        auto const errors  = lost.errors();
        auto const samples = lost.samples();
        runFor(lost, 40ms);
        checkEq(lost.errors(), errors, "nothing happened before the timeout");
        runFor(lost, 20ms);
        checkEq(lost.errors(), errors + 1, "given up on after 50 ms, as one error");
        checkEq(Log::warnings, 1, "and said");
        check(runUntil(
                lost,
                [&] { return lost.samples() > samples; },
                500ms),
              "the device carries on");
    }

    testCase("engine: five faults in a row past the bring-up run the Init script again");
    fresh();
    FakeBus::respond = zeros;
    Dev<Probe> re{};
    check(runUntil(re, [&] { return re.valid(); }, 500ms), "up");
    checkEq(readsOf(Probe::Address, 0x00), std::size_t{1}, "one bring-up so far");
    FakeBus::respond = alwaysFault;
    check(runUntil(re, [&] { return !re.answering(); }, 2s), "no longer up after the faults");
    checkEq(readsOf(Probe::Address, 0x00), std::size_t{1}, "nothing re-read while the bus faults");
    check(!re.absent(), "faults did not park it");
    FakeBus::respond = zeros;
    check(runUntil(re, [&] { return re.valid(); }, 500ms), "up again once the bus answers");
    checkEq(readsOf(Probe::Address, 0x00), std::size_t{2}, "through the Init script");
    checkEq(Log::warnings, 0, "with nothing to warn about");
    if(failures != 0) { dump(); }
}

// -- Link: where a device is with its part ---------------------------------------------------

namespace LinkTest {

    /// Two cyclic read groups and no Init: nothing is sent to come up, so only an acknowledged
    /// read makes the device answering.
    struct TwoGroups {
        static constexpr std::string_view Name          = "LINK";
        static constexpr Address7         Address       = 0x2A;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Fast {
            static constexpr auto       Period = std::chrono::milliseconds{10};
            static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        struct Slow {
            static constexpr auto       Period = std::chrono::milliseconds{50};
            static constexpr std::array Steps{Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        using Reads   = List<Fast, Slow>;
        using Primary = Fast;
    };

}   // namespace LinkTest

void linkState() {
    using LinkTest::TwoGroups;

    testCase("Link: a chip without Init is starting until its part acknowledges something");
    fresh();
    RegisterModel<1, 1> model{TwoGroups::Address};
    model.set(0x00, {7, 9});
    bool answer = false;
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            return answer ? model(a, sent, recv) : FakeBus::Result::notAcknowledged;
        };
    Dev<TwoGroups> d{};
    check(d.link() == Link::starting, "starting before anything happened");
    turn(d);
    turn(d);
    check(!d.answering() && !d.absent(), "brought up, but nothing acknowledged: starting");
    check(runUntil(d, [&] { return d.absent(); }, 200ms), "NAKs in a row park it");
    answer = true;
    check(runUntil(d, [&] { return d.answering(); }, 3s), "answering with the probe's ACK");
    check(runUntil(d, [&] { return d.valid(); }, 200ms), "valid once both cyclic groups delivered");
    checkEq(d.latest(), std::uint8_t{7}, "latest() without a group is the Primary's");
    checkEq(d.seq(), d.seq<TwoGroups::Fast>(), "and so is seq()");

    testCase("Link: a bring-up forgets the readings from before it, and bringUps() counts it");
    auto const ups = d.bringUps();
    answer         = false;
    check(runUntil(d, [&] { return d.absent(); }, 200ms), "parked again");
    check(!d.valid() && !d.valid<TwoGroups::Fast>(), "a parked part has no valid reading");
    answer = true;
    check(runUntil(d, [&] { return d.answering(); }, 5s), "back");
    check(d.bringUps() > ups, "brought up again");
    check(d.samples<TwoGroups::Slow>() > 0, "the lifetime counts stay");
    check(runUntil(
            d,
            [&] { return d.valid(); },
            200ms),
          "and valid again on samples from after it");

    testCase("Link: fresh(seen) gives every reader every sample");
    std::uint32_t first  = 0;
    std::uint32_t second = 0;
    check(d.fresh(first) && d.fresh(second), "both readers see the newest sample");
    check(!d.fresh(first) && !d.fresh(second), "and neither sees it twice");
    auto const seq = d.seq();
    check(runUntil(d, [&] { return d.seq() != seq; }, 50ms), "a new sample");
    check(d.fresh(second) && d.fresh(first), "each reader sees it, in either order");
    check(d.template fresh<TwoGroups::Slow>(first), "a cursor per group: Slow's is its own");
}

// -- request tickets, and writes that skip what the chip already holds --------------------------

namespace TicketTest {

    /// An on-demand read with a check step whose flag the model controls, and two write groups:
    /// a plain state, and one whose every set is a command.
    struct OnDemand {
        static constexpr std::string_view Name          = "TICKET";
        static constexpr Address7         Address       = 0x2B;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Page {
            static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0}),
                                              Step::check(2ms),
                                              Step::read({.reg = 0x01, .count = 1, .offset = 1})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr bool ready(Bytes data) { return (data.u8(0) & 1U) != 0; }

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(1); }
        };

        struct Level {
            using Value                        = std::uint8_t;
            static constexpr std::size_t Bytes = 1;

            [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                       std::span<std::byte> buffer) {
                buffer[0] = std::byte{value};
                return Step::writeBuffer({.reg = 0x10, .offset = 0, .count = 1});
            }
        };

        struct Clear {
            using Value                              = std::uint8_t;
            static constexpr std::size_t Bytes       = 1;
            static constexpr bool        AlwaysWrite = true;

            [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                       std::span<std::byte> buffer) {
                buffer[0] = std::byte{value};
                return Step::writeBuffer({.reg = 0x11, .offset = 0, .count = 1});
            }
        };

        using Reads  = List<Page>;
        using Writes = List<Level, Clear>;
    };

}   // namespace TicketTest

void ticketsAndWrites() {
    using TicketTest::OnDemand;
    using P = OnDemand::Page;
    using L = OnDemand::Level;
    using C = OnDemand::Clear;

    testCase("request: a ticket is pending until its run ends, then says how it went");
    fresh();
    RegisterModel<1, 1> model{OnDemand::Address};
    model.set(0x00, {0x01, 42});
    bool nak = false;
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            return nak ? FakeBus::Result::notAcknowledged : model(a, sent, recv);
        };
    Dev<OnDemand> d{};
    auto const    t1 = d.request<P>();
    check(d.answer<P>(t1) == Answer::pending, "pending before the run");
    check(runUntil(d, [&] { return d.answer<P>(t1) != Answer::pending; }, 50ms), "answered");
    check(d.answer<P>(t1) == Answer::ok && d.latest() == 42, "ok, with the sample");
    check(d.answer<P>(Ticket{}) == Answer::pending, "a ticket nobody was given is never answered");

    testCase("request: two requests made before the run are served by that one run");
    auto const reads = model.reads;
    auto const t2    = d.request<P>();
    auto const t3    = d.request<P>();
    check(runUntil(d, [&] { return d.answer<P>(t3) != Answer::pending; }, 50ms), "answered");
    check(d.answer<P>(t2) == Answer::ok && d.answer<P>(t3) == Answer::ok, "both ok");
    checkEq(model.reads - reads, 2, "one run: its two reads");

    testCase("request: a run whose check never passes answers rejected");
    model.set(0x00, {0x00});
    auto const t4 = d.request<P>();
    check(runUntil(d, [&] { return d.answer<P>(t4) != Answer::pending; }, 300ms), "answered");
    check(d.answer<P>(t4) == Answer::rejected, "rejected once the retries ran out");
    model.set(0x00, {0x01});

    testCase("request: a NAK answers failed, and one made while the part is away waits for it");
    nak = true;
    for(int k = 0; k < 3; ++k) {
        auto const t = d.request<P>();
        check(runUntil(d, [&] { return d.answer<P>(t) != Answer::pending; }, 50ms), "answered");
        check(d.answer<P>(t) == Answer::failed, "failed");
    }
    check(runUntil(d, [&] { return d.absent(); }, 50ms), "three NAKs park it");
    auto const t5 = d.request<P>();
    runFor(d, 100ms);
    check(d.answer<P>(t5) == Answer::pending, "still pending while the part is away");
    nak = false;
    check(runUntil(
            d,
            [&] { return d.answer<P>(t5) != Answer::pending; },
            5s),
          "answered once it is back");
    check(d.answer<P>(t5) == Answer::ok && d.answering(),
          "ok: the probe was the request's own run");

    testCase("set: the value the chip holds is not sent again; rewrite() and modify() behave");
    auto const w0 = model.writes;
    check(d.set<L>(5), "a new value is set");
    check(d.pending<L>(), "and owed");
    check(runUntil(d, [&] { return !d.pending<L>(); }, 50ms), "then written");
    checkEq(model.writes - w0, 1, "one write");
    check(!d.set<L>(5), "the same value again: nothing to send");
    check(!d.pending<L>(), "and nothing owed");
    d.rewrite<L>(5);
    check(d.pending<L>(), "rewrite() owes it anyway");
    check(runUntil(d, [&] { return !d.pending<L>(); }, 50ms), "written");
    checkEq(model.writes - w0, 2, "a second write");
    check(!d.modify<L>([](auto& v) { v = 5; }), "modify() to the value it holds sends nothing");
    check(d.modify<L>([](auto& v) { v = 6; }), "modify() to another value does");
    check(runUntil(d, [&] { return !d.pending<L>(); }, 50ms), "written");
    checkEq(model.writes - w0, 3, "a third write");

    testCase("set: an AlwaysWrite group sends every value, the one it holds too");
    auto const w1 = model.writes;
    check(d.set<C>(1) && runUntil(d, [&] { return !d.pending<C>(); }, 50ms), "set and written");
    check(d.set<C>(1)
            && runUntil(
              d,
              [&] { return !d.pending<C>(); },
              50ms),
          "the same value, set and written again");
    checkEq(model.writes - w1, 2, "two writes");

    testCase(
      "StallWatch: a part that answers and delivers nothing new is caught, and not every turn");
    fresh();
    RegisterModel<1, 1> quietModel{OnDemand::Address};
    quietModel.set(0x00, {0x01, 7});
    FakeBus::respond = std::ref(quietModel);
    Dev<OnDemand> quiet{};
    auto const    first = quiet.request<P>();
    check(runUntil(quiet, [&] { return quiet.answer<P>(first) == Answer::ok; }, 50ms), "answering");
    StallWatch<Dev<OnDemand>, P> watch{std::chrono::seconds{3}, std::chrono::seconds{10}};
    int                          fired = 0;
    auto const                   run   = [&](int ms) {
        for(int i = 0; i < ms; ++i) {
            if(watch.handler(quiet, FakeClock::now())) { ++fired; }
            turn(quiet);
        }
    };
    run(2900);
    checkEq(fired, 0, "nothing before 3 s without a sample");
    run(200);
    checkEq(fired, 1, "once at 3 s");
    run(9000);
    checkEq(fired, 1, "not again within 10 s");
    run(1500);
    checkEq(fired, 2, "and again after them");
    checkEq(watch.stalls(), 2U, "counted");
    static_cast<void>(quiet.request<P>());
    run(50);
    auto const before = fired;
    run(2900);
    checkEq(fired, before, "a new sample starts the count over");

    testCase(
      "AbsentWatch: a part that stops answering is reported after a while, and not every turn");
    fresh();
    FakeBus::respond = std::ref(quietModel);
    Dev<OnDemand> there{};
    auto const    asked = there.request<P>();
    check(runUntil(there, [&] { return there.answer<P>(asked) == Answer::ok; }, 50ms), "answering");
    AbsentWatch<Dev<OnDemand>> lost{std::chrono::seconds{2}, std::chrono::seconds{10}};
    int                        gone   = 0;
    auto const                 watch2 = [&](int ms) {
        for(int i = 0; i < ms; ++i) {
            if(lost.handler(there, FakeClock::now())) { ++gone; }
            static_cast<void>(there.request<P>());
            turn(there);
        }
    };
    watch2(3000);
    checkEq(gone, 0, "nothing while it answers");
    FakeBus::respond = nullptr;   // nobody home: every transaction is a NAK
    watch2(1500);
    checkEq(gone, 0, "nor in the first two seconds of silence");
    watch2(1500);
    checkEq(gone, 1, "once after them");
    watch2(8000);
    checkEq(gone, 1, "not again within 10 s");
    watch2(3000);
    checkEq(gone, 2, "and again after them");
    checkEq(lost.count(), 2U, "counted");
    FakeBus::respond = std::ref(quietModel);
    watch2(31000);   // the presence probe backs off to 30 s
    auto const soFar = gone;
    watch2(5000);
    checkEq(gone, soFar, "and an answering part ends it");
}

// -- the engine on its own -----------------------------------------------------------------

/// A chip that exists only to exercise the engine's read-back verify: one status register
/// it reads and one config register it writes and checks. Nothing of a real chip's
/// character is in the way.
struct VerifyChip {
    static constexpr std::string_view Name          = "VERIFY";
    static constexpr Address7         Address       = 0x40;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array Init{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

    struct Status {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        struct Sample {
            std::uint8_t v{};
        };

        static constexpr Sample decode(Bytes b) { return {b.u8(0)}; }
    };

    struct Config {
        using Value                                 = std::uint8_t;
        static constexpr std::size_t Bytes          = 1;
        static constexpr auto        VerifyDelay    = std::chrono::milliseconds{10};
        static constexpr auto        VerifyInterval = std::chrono::milliseconds{500};

        static constexpr Step encode(Value const&         v,
                                     std::span<std::byte> buf) {
            buf[0] = static_cast<std::byte>(v);
            return Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 1});
        }
    };

    using Reads  = List<Status>;
    using Writes = List<Config>;
};

/// The same, but the top bit of the register is a status flag the chip owns, so a
/// read-back that differs only there is not a mismatch.
struct VerifyMaskChip {
    static constexpr std::string_view Name          = "VERIFYMASK";
    static constexpr Address7         Address       = 0x41;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array Init{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

    struct Config {
        using Value                              = std::uint8_t;
        static constexpr std::size_t Bytes       = 1;
        static constexpr auto        VerifyDelay = std::chrono::milliseconds{10};

        static constexpr Step encode(Value const&         v,
                                     std::span<std::byte> buf) {
            buf[0] = static_cast<std::byte>(v);
            return Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 1});
        }

        // Bytes the type has to be spelt out: a write group's own Bytes member shadows it.
        static constexpr bool verify(Kvasir::I2C::Bytes want,
                                     Kvasir::I2C::Bytes got) {
            return (want.u8(0) & 0x7FU) == (got.u8(0) & 0x7FU);
        }
    };

    using Writes = List<Config>;
};

/// A memory whose read length is part of the request: one description serves a four-byte
/// and a sixty-four-byte read.
struct MemChip {
    static constexpr std::string_view Name          = "MEM";
    static constexpr Address7         Address       = 0x50;
    static constexpr std::size_t      RegisterBytes = 1;

    struct Page {
        struct Request {
            std::uint8_t address{};
            std::uint8_t length{4};
        };

        static constexpr std::array Steps{
          Step::readIndirect({.regOffset = 0, .count = 64, .offset = 1})};

        static constexpr std::size_t prepare(Request const&       r,
                                             std::span<std::byte> buf) {
            buf[0] = static_cast<std::byte>(r.address);
            return r.length;
        }

        struct Sample {
            std::uint8_t                 length{};
            std::array<std::uint8_t, 64> data{};
        };

        static constexpr Sample decode(Kvasir::I2C::Bytes b) {
            Sample s{};
            s.length = static_cast<std::uint8_t>(b.size() - 1);
            for(std::size_t i = 0; i + 1 < b.size(); ++i) { s.data[i] = b.u8(1 + i); }
            return s;
        }
    };

    using Reads = List<Page>;
};

/// The page's gap in the test that asks for one.
struct FiveMsGap {
    static constexpr auto Gap = 5ms;
};

/// A display page written as two transactions: the window command, then the RAM data
/// behind it (an SSD1306 in miniature). The chip has no register address, so both are bare
/// commands. `Timing::Gap` is the delay the description asks for between them. `Contrast` is an
/// ordinary single-step group on the same chip and `Status` a cyclic read, so the tests can
/// show that nothing gets between the two transactions of a page.
template<typename Timing = DefaultTiming>
struct PageChip {
    static constexpr std::chrono::milliseconds Gap = [] {
        if constexpr(requires { Timing::Gap; }) {
            return Kvasir::asDuration(Timing::Gap);
        } else {
            return std::chrono::milliseconds::zero();
        }
    }();

    static constexpr std::string_view Name          = "PAGE";
    static constexpr Address7         Address       = 0x3C;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array Init{Step::command({.payload = {0xAE}}),
                                     Step::command({.payload = {0xAF}})};

    struct Status {
        static constexpr auto       Period = std::chrono::milliseconds{2};
        static constexpr std::array Steps{Step::receive({.count = 1})};

        struct Sample {
            std::uint8_t v{};
        };

        static constexpr Sample decode(Kvasir::I2C::Bytes b) { return {b.u8(0)}; }
    };

    struct Page {
        using Value                        = std::array<std::uint8_t, 8>;
        static constexpr std::size_t Items = 2;
        static constexpr std::size_t Bytes = 4 + 1 + 8;

        static constexpr std::array<Step,
                                    2>
        encode(Value const&         v,
               std::size_t          page,
               std::span<std::byte> buf) {
            buf[0] = std::byte{0x00};                        // the command stream
            buf[1] = static_cast<std::byte>(0xB0U | page);   // page address
            buf[2] = std::byte{0x00};                        // column, low nibble
            buf[3] = std::byte{0x10};                        // column, high nibble
            buf[4] = std::byte{0x40};                        // the data stream
            for(std::size_t i = 0; i < 8; ++i) { buf[5 + i] = std::byte{v[i]}; }
            return {Step::commandBuffer({.offset = 0, .count = 4, .delay = Gap}),
                    Step::commandBuffer({.offset = 4, .count = 9})};
        }
    };

    struct Contrast {
        using Value                        = std::uint8_t;
        static constexpr std::size_t Bytes = 3;

        static constexpr Step encode(Value const&         v,
                                     std::span<std::byte> buf) {
            buf[0] = std::byte{0x00};
            buf[1] = std::byte{0x81};
            buf[2] = static_cast<std::byte>(v);
            return Step::commandBuffer({.offset = 0, .count = 3});
        }
    };

    using Reads  = List<Status>;
    using Writes = List<Page, Contrast>;
};

// A chip whose writes are one transaction each carries no script store at all: the empty
// [[no_unique_address]] member, the same trick verify_ uses on a chip that does not verify.
static_assert(std::is_empty_v<Kvasir::I2C::detail::WriteScript<false,
                                                               1>>);
static_assert(Dev<Chips::Ht16k33>::MaxWriteSteps == 1);
static_assert(Dev<PageChip<>>::MaxWriteSteps == 2);

/// Acks anything at 0x3C and answers a bare read with 0x5A.
FakeBus::Result pageOk(std::uint8_t               addr,
                       std::span<std::byte const> sent,
                       std::span<std::byte>       recv) {
    static_cast<void>(sent);
    if(addr != 0x3C) { return FakeBus::Result::notAcknowledged; }
    for(auto& b : recv) { b = std::byte{0x5A}; }
    return FakeBus::Result::succeeded;
}

void engine() {
    testCase("engine: absent device parked, probe is the first bring-up step");
    fresh();
    FakeBus::respond = {};
    Dev<Chips::Bme280> d{};
    check(runUntil(d, [&] { return d.absent(); }, 100ms), "parked after three NAKs");
    auto n = FakeBus::submitted;
    runFor(d, 500ms);
    check(FakeBus::submitted <= n + 1, "at most one probe in half a second");
    // The BME280 names its oracle, so its bring-up opens with the identity read of 0xD0 -- and
    // that, not the soft reset the Init script starts with, is what an absent part is probed with.
    check(FakeBus::log.back().isRead()
            && FakeBus::log.back().sent == std::vector<std::uint8_t>{0xD0},
          "the probe is the identity read, which changes nothing on whatever answers");

    testCase("engine: a NAK in a read group loses that sample, the period continues");
    fresh();
    RegisterModel<1> m{0x20};
    bool             nak = false;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(nak) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                return addr == 0x20 ? FakeBus::Result::succeeded : FakeBus::Result::notAcknowledged;
            }
            recv[0] = std::byte{0x55};
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Pcf8574> p{};
    check(runUntil(p, [&] { return p.samples() == 2; }, 200ms), "two samples");
    nak = true;
    runFor(p, 60ms);
    nak             = false;
    auto const errs = p.errors();
    check(errs >= 1 && errs <= 2, "one or two errors in 60 ms");
    check(!p.absent(), "not parked by one lost read");
    check(runUntil(p, [&] { return p.samples() == 4; }, 200ms), "and back to sampling");

    testCase("engine: a failed write stays owed");
    fresh();
    nak               = false;
    std::uint8_t port = 0;
    FakeBus::respond
      = [&](std::uint8_t, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(nak) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                port = static_cast<std::uint8_t>(sent[0]);
                return FakeBus::Result::succeeded;
            }
            recv[0] = std::byte{0};
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Pcf8574> q{};
    check(runUntil(q, [&] { return q.samples() == 1; }, 100ms), "up");
    nak = true;
    q.set<Chips::Pcf8574::Port>(0x3C);
    runFor(q, 10ms);
    check(q.pending(), "still pending while the chip NAKs");
    nak = false;
    check(runUntil(q, [&] { return port == 0x3C; }, 100ms), "written once it answers");

    testCase("engine: request() runs a group at once");
    fresh();
    RegisterModel<1> rtc{0x68};
    FakeBus::respond = std::ref(rtc);
    rtc.set(0x00, {0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00});
    Dev<Chips::Ds3231> r{};
    check(runUntil(r, [&] { return r.samples<Chips::Ds3231::Clock>() == 1; }, 100ms), "clock read");
    check(runUntil(
            r,
            [&] { return r.samples<Chips::Ds3231::Temperature>() == 1; },
            100ms),
          "temperature read once at start");
    r.request<Chips::Ds3231::Temperature>();
    check(runUntil(
            r,
            [&] { return r.samples<Chips::Ds3231::Temperature>() == 2; },
            20ms),
          "requested read within 20 ms");
    runFor(r, 2s);
    checkEq(r.samples<Chips::Ds3231::Temperature>(), 2U, "and the 10 s period is untouched by it");
    check(r.samples<Chips::Ds3231::Clock>() >= 3, "the clock kept its second");

    // A reset puts the chip back at its defaults. Anything the application has set since is
    // owed again, so what value<W>() reports stays what is on the chip.
    testCase("engine: a write group without Initial is re-sent after a reset");
    fresh();
    RegisterModel<1, 2> lim{0x18};
    lim.set(0x06, {0x00, 0x54, 0x04, 0x00});
    lim.set(0x05, {0x01, 0x90});
    lim.readOnly = {0x05, 0x06, 0x07};
    bool limNak  = false;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(limNak) { return FakeBus::Result::notAcknowledged; }
            if(addr == 0x18 && sent.size() == 2 && recv.empty()
               && static_cast<std::uint8_t>(sent[0]) == 0x08)
            {
                return FakeBus::Result::succeeded;   // the one 8-bit register
            }
            return lim(addr, sent, recv);
        };
    Dev<Chips::Mcp9808> t{};
    check(runUntil(t, [&] { return t.valid(); }, 500ms), "up");
    t.set<Chips::Mcp9808::Limits>(Chips::Mcp9808::High,
                                  Units::centiDegC(3000));   // 30.00 degC -> 0x01E0
    check(runUntil(t, [&] { return lim.word(0x02) == 0x01E0; }, 200ms), "upper limit written");

    lim.set(0x02, {0x00, 0x00});   // the chip loses it, as a power glitch would
    limNak = true;
    check(runUntil(t, [&] { return t.absent(); }, 3s), "parked after three NAKs");
    limNak = false;
    check(runUntil(t, [&] { return !t.absent() && t.valid(); }, 3s), "back after a probe");
    check(runUntil(
            t,
            [&] { return lim.word(0x02) == 0x01E0; },
            200ms),
          "and the limit the application set is on the chip again");
    checkEq(centiOf(t.value<Chips::Mcp9808::Limits>(Chips::Mcp9808::High)),
            3000,
            "value<W>() agrees");

    testCase("engine: a Transient write group is not replayed");
    fresh();
    RegisterModel<1> rtc2{0x68};
    rtc2.set(0x00, {0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00});
    bool rtcNak = false;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(rtcNak) { return FakeBus::Result::notAcknowledged; }
            return rtc2(addr, sent, recv);
        };
    Dev<Chips::Ds1307> c2{};
    check(runUntil(c2, [&] { return c2.valid(); }, 500ms), "up");
    c2.set<Chips::Ds1307::SetTime>(Chips::Ds1307::Time{0, 30, 13, 4, 9, 9, 26, false});
    check(runUntil(
            c2,
            [&] { return c2.writes<Chips::Ds1307::SetTime>() == 1; },
            200ms),
          "time set");
    rtcNak = true;
    check(runUntil(c2, [&] { return c2.absent(); }, 8s), "parked");
    rtcNak = false;
    check(runUntil(c2, [&] { return !c2.absent() && c2.valid(); }, 8s), "back after a probe");
    runFor(c2, 200ms);
    checkEq(c2.writes<Chips::Ds1307::SetTime>(), 1U, "the stale time was not written again");

    testCase("engine: a Transient write survives the base's error-threshold reset");
    fresh();
    RegisterModel<1> rtc4{0x68};
    rtc4.set(0x00, {0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00});
    bool wireBroken = false;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(wireBroken) { return FakeBus::Result::failed; }
            return rtc4(addr, sent, recv);
        };
    Dev<Chips::Ds1307> c4{};
    check(runUntil(c4, [&] { return c4.valid(); }, 500ms), "up");
    // The wire breaks, then the time is set: every attempt is a bus fault, and the sixth
    // trips the base's error threshold with the write still on the wire. A Transient group
    // is not replayed by the bring-up, so the only way it survives is being owed again.
    wireBroken = true;
    c4.set<Chips::Ds1307::SetTime>(Chips::Ds1307::Time{0, 30, 13, 4, 9, 9, 26, false});
    runFor(c4, 300ms);
    checkEq(c4.writes<Chips::Ds1307::SetTime>(), 0U, "nothing landed while the wire was broken");
    check(c4.errors() > 5, "well past the base's threshold, so it did reset the device");
    check(!c4.absent(), "a bus fault is not a NAK: still present");
    wireBroken = false;
    check(runUntil(
            c4,
            [&] { return c4.writes<Chips::Ds1307::SetTime>() == 1; },
            1s),
          "written once the wire is back");
    checkEq(rtc4.word(0x02), 0x13U, "and 13:00 reached the chip");

    testCase("engine: modify<W>() edits one field and leaves the rest");
    fresh();
    RegisterModel<1> rtc3{0x68};
    rtc3.set(0x00, {0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00});
    FakeBus::respond = std::ref(rtc3);
    Dev<Chips::Ds1307> c3{};
    check(runUntil(c3, [&] { return c3.valid(); }, 500ms), "up");
    c3.set<Chips::Ds1307::SetTime>(Chips::Ds1307::Time{0, 30, 13, 4, 9, 9, 26, false});
    check(runUntil(
            c3,
            [&] { return c3.writes<Chips::Ds1307::SetTime>() == 1; },
            200ms),
          "time set");
    checkEq(rtc3.word(0x02), 0x13U, "13:00 as BCD in the hour register");

    c3.modify<Chips::Ds1307::SetTime>([](auto& v) { v.hour = 7; });
    check(runUntil(
            c3,
            [&] { return c3.writes<Chips::Ds1307::SetTime>() == 2; },
            200ms),
          "rewritten");
    checkEq(rtc3.word(0x02), 0x07U, "the hour changed");
    checkEq(rtc3.word(0x01), 0x30U, "the minute the application never touched is still 30");
    checkEq(rtc3.word(0x06), 0x26U, "and so is the year");
    checkEq(c3.value<Chips::Ds1307::SetTime>().minute, 30U, "value<W>() kept the rest too");

    testCase("engine: a written register is read back and matches");
    fresh();
    RegisterModel<1> vm{0x40};
    vm.set(0x00, {0x00});
    FakeBus::respond = std::ref(vm);
    Dev<VerifyChip> v{};
    check(runUntil(v, [&] { return v.valid(); }, 500ms), "up");
    auto const readsBefore = vm.reads;
    v.set<VerifyChip::Config>(0x5A);
    check(runUntil(v, [&] { return v.writes<VerifyChip::Config>() == 1; }, 200ms), "written");
    checkEq(vm.word(0x01), 0x5AU, "the chip holds it");
    check(runUntil(v, [&] { return vm.reads > readsBefore + 1; }, 200ms), "and it is read back");
    runFor(v, 200ms);
    checkEq(v.mismatches<VerifyChip::Config>(), 0U, "no mismatch");
    checkEq(v.writes<VerifyChip::Config>(), 1U, "and it was written exactly once");

    testCase("engine: a register that lost its value is written again");
    fresh();
    RegisterModel<1> vc{0x40};
    vc.set(0x00, {0x00});
    bool corrupt = true;
    vc.onWrite   = [&](std::uint32_t reg, std::array<std::uint8_t, 1> const&) {
        if(corrupt && reg == 0x01) {
            corrupt = false;   // it drops the first write only
            vc.set(0x01, {0x00});
        }
    };
    FakeBus::respond = std::ref(vc);
    Dev<VerifyChip> w{};
    check(runUntil(w, [&] { return w.valid(); }, 500ms), "up");
    w.set<VerifyChip::Config>(0x33);
    check(runUntil(
            w,
            [&] { return w.mismatches<VerifyChip::Config>() == 1; },
            300ms),
          "the read-back did not match");
    check(runUntil(w, [&] { return vc.word(0x01) == 0x33U; }, 300ms), "so it was written again");
    runFor(w, 300ms);
    checkEq(w.mismatches<VerifyChip::Config>(), 1U, "and it matched from then on");

    testCase("engine: a register that never holds its value is not rewritten for ever");
    fresh();
    RegisterModel<1> vs{0x40};
    vs.set(0x00, {0x00});
    vs.onWrite = [&](std::uint32_t reg, std::array<std::uint8_t, 1> const&) {
        if(reg == 0x01) { vs.set(0x01, {0x00}); }   // stuck low, always
    };
    FakeBus::respond = std::ref(vs);
    Dev<VerifyChip> x{};
    check(runUntil(x, [&] { return x.valid(); }, 500ms), "up");
    x.set<VerifyChip::Config>(0x77);
    // the first write plus MaxRetries (8) rewrites, then it gives up until VerifyInterval
    check(runUntil(x, [&] { return x.writes<VerifyChip::Config>() == 9; }, 400ms), "nine attempts");
    auto const at = FakeClock::current;
    while(FakeClock::current - at < 300ms) { turn(x); }
    checkEq(x.writes<VerifyChip::Config>(), 9U, "and then it stops");
    check(runUntil(
            x,
            [&] { return x.writes<VerifyChip::Config>() > 9; },
            600ms),
          "until the 500 ms verify interval comes round");

    testCase("engine: verify() decides what counts as a match");
    fresh();
    RegisterModel<1> vk{0x41};
    vk.set(0x00, {0x00});
    vk.onWrite = [&](std::uint32_t reg, std::array<std::uint8_t, 1> const& val) {
        if(reg == 0x01) { vk.set(0x01, {static_cast<std::uint8_t>(val[0] | 0x80U)}); }
    };
    FakeBus::respond = std::ref(vk);
    Dev<VerifyMaskChip> y{};
    check(runUntil(y, [&] { return y.answering(); }, 500ms), "up");
    y.set<VerifyMaskChip::Config>(0x12);
    check(runUntil(y, [&] { return y.writes<VerifyMaskChip::Config>() == 1; }, 200ms), "written");
    checkEq(vk.word(0x01), 0x92U, "the chip set its own status bit");
    runFor(y, 300ms);
    checkEq(y.mismatches<VerifyMaskChip::Config>(), 0U, "the status bit is not a mismatch");
    checkEq(y.writes<VerifyMaskChip::Config>(), 1U, "so nothing was rewritten");

    testCase("engine: an on-demand group does not hold valid() down");
    fresh();
    RegisterModel<1> od{0x5A};
    od.set(0x00, {0x00, 0x00});
    od.set(0x5C, {0x10, 0x24});   // CDC and CDT as a reset leaves them
    FakeBus::respond = std::ref(od);
    Dev<Chips::Mpr121<>> ov{};
    // Touch is cyclic, Filtered only answers request<Filtered>(); valid() is about the
    // cyclic readings, so it comes up without anyone asking for the filtered data.
    check(runUntil(ov, [&] { return ov.valid(); }, 500ms), "valid on the cyclic group alone");
    checkEq(ov.samples<Chips::Mpr121<>::Filtered>(), 0U, "the on-demand group has not run");
    ov.request<Chips::Mpr121<>::Filtered>();
    check(runUntil(
            ov,
            [&] { return ov.samples<Chips::Mpr121<>::Filtered>() == 1; },
            200ms),
          "and runs when asked");

    testCase("engine: prepare() sizes the read");
    fresh();
    RegisterModel<1> mem{0x50};
    for(std::uint32_t i = 0; i < 64; ++i) { mem.set(i, {static_cast<std::uint8_t>(0xA0 + i)}); }
    FakeBus::respond = std::ref(mem);
    Dev<MemChip> z{};
    z.request<MemChip::Page>({0x10, 4});
    check(runUntil(z, [&] { return z.samples<MemChip::Page>() == 1; }, 200ms), "four bytes");
    checkEq(FakeBus::log.back().recvLen, std::size_t{4}, "four bytes on the wire");
    checkEq(z.latest<MemChip::Page>().length, 4U, "and four decoded");
    checkEq(z.latest<MemChip::Page>().data[0], 0xB0U, "from address 0x10");

    z.request<MemChip::Page>({0x00, 64});
    check(runUntil(z, [&] { return z.samples<MemChip::Page>() == 2; }, 200ms), "sixty-four bytes");
    checkEq(FakeBus::log.back().recvLen, std::size_t{64}, "sixty-four bytes on the wire");
    checkEq(z.latest<MemChip::Page>().length, 64U, "and sixty-four decoded");
    checkEq(z.latest<MemChip::Page>().data[63], 0xDFU, "the last byte");

    z.request<MemChip::Page>({0x00, 200});   // more than the group holds
    check(runUntil(z, [&] { return z.samples<MemChip::Page>() == 3; }, 200ms), "clamped, not lost");
    checkEq(FakeBus::log.back().recvLen, std::size_t{64}, "clamped to what the group holds");

    // -- a write group whose item is several transactions ----------------------------------

    using P0   = PageChip<>;
    using Wire = std::vector<std::vector<std::uint8_t>>;

    testCase("engine: a write group that encodes to two Steps goes out as two transactions");
    fresh();
    FakeBus::respond = pageOk;
    Dev<P0> pg{};
    check(runUntil(pg, [&] { return pg.answering(); }, 200ms), "up");
    check(hasWrite({0xAE}) && hasWrite({0xAF}), "the Init commands");
    auto base = FakeBus::log.size();
    pg.set<P0::Page>(0, {1, 2, 3, 4, 5, 6, 7, 8});
    check(runUntil(pg, [&] { return !pg.pending(); }, 200ms), "page written");
    check(writes(base) == Wire{{0x00, 0xB0, 0x00, 0x10}, {0x40, 1, 2, 3, 4, 5, 6, 7, 8}},
          "the window command, then the RAM data, in that order and nothing else");
    checkEq(pg.writes<P0::Page>(), 1U, "one item written once, counted after the last step");
    checkEq(pg.errors(), 0U, "no errors");

    testCase("engine: each dirty item is a whole sequence before any other group runs");
    base = FakeBus::log.size();
    // Page 0 holds these bytes already, which set() would skip: rewrite() sends them anyway.
    pg.rewrite<P0::Page>(0, {1, 2, 3, 4, 5, 6, 7, 8});
    pg.set<P0::Page>(1, {9, 10, 11, 12, 13, 14, 15, 16});
    pg.set<P0::Contrast>(0x7F);
    check(runUntil(pg, [&] { return !pg.pending(); }, 400ms), "everything written");
    check(writes(base)
            == Wire{{0x00, 0xB0, 0x00, 0x10},
                    {0x40, 1, 2, 3, 4, 5, 6, 7, 8},
                    {0x00, 0xB1, 0x00, 0x10},
                    {0x40, 9, 10, 11, 12, 13, 14, 15, 16},
                    {0x00, 0x81, 0x7F}},
          "page 0 whole, then page 1 whole, then the other group -- never interleaved");
    checkEq(pg.writes<P0::Page>(), 3U, "three items written in all");
    checkEq(pg.writes<P0::Contrast>(), 1U, "and the single-step group once");

    testCase("engine: a single-step group on the same chip is unaffected");
    base = FakeBus::log.size();
    pg.set<P0::Contrast>(0x20);
    check(runUntil(pg, [&] { return !pg.pending(); }, 200ms), "contrast written");
    check(writes(base) == Wire{{0x00, 0x81, 0x20}}, "exactly one transaction");

    testCase("engine: a failure on the second step retries the item from the first");
    fresh();
    int nakData = 1;   // NAK the data transaction once
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr == 0x3C && !sent.empty() && static_cast<std::uint8_t>(sent[0]) == 0x40
               && nakData > 0)
            {
                --nakData;
                return FakeBus::Result::notAcknowledged;
            }
            return pageOk(addr, sent, recv);
        };
    Dev<P0> pf{};
    check(runUntil(pf, [&] { return pf.answering(); }, 200ms), "up");
    auto const failBase = FakeBus::log.size();
    pf.set<P0::Page>(0, {1, 2, 3, 4, 5, 6, 7, 8});
    check(runUntil(
            pf,
            [&] { return pf.writes<P0::Page>() == 1; },
            400ms),
          "written once the chip answers");
    check(writes(failBase)
            == Wire{{0x00, 0xB0, 0x00, 0x10},
                    {0x40, 1, 2, 3, 4, 5, 6, 7, 8},   // NAKed
                    {0x00, 0xB0, 0x00, 0x10},         // the window again, not just the data
                    {0x40, 1, 2, 3, 4, 5, 6, 7, 8}},
          "the whole sequence again from its first step");
    checkEq(pf.errors(), 1U, "one error");
    check(!pf.pending(), "and nothing left owed");

    testCase("engine: a delay between the two steps is honoured, and nothing gets between");
    fresh();
    FakeBus::respond = pageOk;
    using P5         = PageChip<FiveMsGap>;
    Dev<P5> pd{};
    check(runUntil(pd, [&] { return pd.answering(); }, 200ms), "up");
    check(runUntil(
            pd,
            [&] { return pd.samples<P5::Status>() >= 2; },
            200ms),
          "the 2 ms read group is running");
    auto const delayBase = FakeBus::log.size();
    pd.set<P5::Page>(0, {1, 2, 3, 4, 5, 6, 7, 8});
    check(runUntil(pd, [&] { return !pd.pending(); }, 200ms), "page written");
    std::size_t windowAt = delayBase;
    while(windowAt < FakeBus::log.size()
          && FakeBus::log[windowAt].sent != std::vector<std::uint8_t>{0x00, 0xB0, 0x00, 0x10})
    {
        ++windowAt;
    }
    check(windowAt + 1 < FakeBus::log.size(), "the window went out");
    check(!FakeBus::log[windowAt + 1].sent.empty() && FakeBus::log[windowAt + 1].sent[0] == 0x40,
          "the RAM data is the very next transaction on the bus: no read gets between");
    auto const gap = FakeBus::log[windowAt + 1].at - FakeBus::log[windowAt].at;
    check(gap >= 5ms, "the step's 5 ms was waited out");
    check(gap <= 7ms, "and not much more (one turn of slack)");
    if(failures != 0) { dump(); }
}

// -- the nets under a script, and the knobs a run-time caller has ----------------------------

namespace NetTest {

    using PresenceTest::Probe;
    using PresenceTest::readsOf;
    using PresenceTest::ShortNet;

    /// Three Init steps, so a NAK on a later one can be told from one on the first.
    struct ThreeStep {
        static constexpr std::string_view Name          = "THREE";
        static constexpr Address7         Address       = 0x2D;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{Step::read({.reg = 0x00, .count = 1, .offset = 0}),
                                               Step::write({.reg = 0x01, .payload = {0x11}}),
                                               Step::write({.reg = 0x02, .payload = {0x22}})};

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{Step::read({.reg = 0x03, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        using Reads = List<Value>;
    };

    /// A decode that always asks to be run again: a busy bit that never clears.
    struct Retrying {
        static constexpr std::string_view Name          = "RETRY";
        static constexpr Address7         Address       = 0x2E;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Data {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes) {
                return Outcome<Sample>::retry(1ms);
            }
        };

        using Reads = List<Data>;
    };

    struct TwoRetries {
        static constexpr std::uint8_t MaxRetries = 2;
    };

    /// Four registers the application may set one at a time, with no Initial.
    struct Levels {
        static constexpr std::string_view Name          = "LEVELS";
        static constexpr Address7         Address       = 0x2F;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        using Value = Probe::Value;

        struct Level {
            using Value                        = std::uint8_t;
            static constexpr std::size_t Items = 4;
            static constexpr std::size_t Bytes = 1;

            [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                       std::size_t          item,
                                                       std::span<std::byte> buffer) {
                buffer[0] = std::byte{value};
                return Step::writeBuffer(
                  {.reg = static_cast<std::uint16_t>(0x10 + item), .offset = 0, .count = 1});
            }
        };

        using Reads  = List<Value>;
        using Writes = List<Level>;
    };

    /// The same four registers with an Initial every item starts from.
    struct InitialLevels {
        static constexpr std::string_view Name          = "INITLEVELS";
        static constexpr Address7         Address       = 0x2C;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        using Value = Probe::Value;

        struct Level {
            using Value                        = std::uint8_t;
            static constexpr std::size_t Items = 4;
            static constexpr std::size_t Bytes = 1;
            static constexpr Value       Initial{0x55};

            [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                       std::size_t          item,
                                                       std::span<std::byte> buffer) {
                buffer[0] = std::byte{value};
                return Step::writeBuffer(
                  {.reg = static_cast<std::uint16_t>(0x10 + item), .offset = 0, .count = 1});
            }
        };

        using Reads  = List<Value>;
        using Writes = List<Level>;
    };

    /// A bus type that says what its callback slot holds, so the engine's check of it is on.
    struct SizedBus : FakeBus {
        static constexpr std::size_t CallbackSize = 32;
    };

    static_assert(Dev<Probe>::CallbackBytes >= sizeof(void*) + sizeof(std::uint32_t),
                  "the completion lambda carries the device and a generation");
    static_assert(Device<SizedBus,
                         FakeClock,
                         Probe>::CallbackBytes
                    <= SizedBus::CallbackSize,
                  "and fits a bus that says what it holds");

}   // namespace NetTest

void nets() {
    using namespace NetTest;

    testCase("engine: a NAK on a later Init step starts the bring-up over, after FaultBackoff");
    fresh();
    RegisterModel<1> three{ThreeStep::Address};
    int              nakLeft = 1;
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(nakLeft > 0 && recv.empty() && !sent.empty()
               && static_cast<std::uint8_t>(sent[0]) == 0x02)
            {
                --nakLeft;
                return FakeBus::Result::notAcknowledged;
            }
            return three(a, sent, recv);
        };
    Dev<ThreeStep> d{};
    check(runUntil(d, [&] { return d.answering(); }, 500ms), "up despite the NAK");
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0x01, 0x11}, {0x02, 0x22}, {0x01, 0x11}, {0x02, 0x22}},
          "the script again from its first step, not from the step that failed");
    checkEq(readsOf(ThreeStep::Address, 0x00),
            std::size_t{2},
            "the first step's read went out twice");
    {
        // The NAKed write is the second transaction; the restart's read is the third.
        check(FakeBus::log.size() >= 4 && FakeBus::log[3].at - FakeBus::log[2].at >= 5ms,
              "and not on the very next turn: FaultBackoff first");
    }
    checkEq(d.errors(), 1U, "one error");
    check(!d.absent(), "one NAK does not park");
    checkEq(d.consecutiveNaks(), 0U, "and the ACKs after it cleared the streak");

    testCase("engine: a decode that keeps asking for a retry is rejected at MaxRetries");
    fresh();
    FakeBus::respond = zeros;
    Dev<Retrying> r{};
    auto const    ticket = r.request<Retrying::Data>();
    check(runUntil(
            r,
            [&] { return r.answer<Retrying::Data>(ticket) != Answer::pending; },
            500ms),
          "answered");
    check(r.answer<Retrying::Data>(ticket) == Answer::rejected, "rejected");
    checkEq(r.rejected(), 1U, "counted once");
    checkEq(r.samples(), 0U, "no sample");
    checkEq(readsOf(Retrying::Address, 0x00),
            std::size_t{EngineDefaults::MaxRetries} + 1,
            "the run, then MaxRetries runs again, a millisecond apart");
    runFor(r, 100ms);
    check(r.rejected() >= 2, "and the period goes on: the next run is rejected the same way");
    {
        fresh();
        FakeBus::respond = zeros;
        Dev<Retrying, TwoRetries> two{};
        check(runUntil(two, [&] { return two.rejected() == 1; }, 200ms), "rejected");
        checkEq(readsOf(Retrying::Address, 0x00),
                std::size_t{3},
                "the run and two retries: Config::MaxRetries");
    }

    testCase("engine: the late answer to a request given up on is not taken for the next one");
    fresh();
    FakeBus::respond = zeros;
    Dev<Probe, ShortNet> lost{};
    check(runUntil(lost, [&] { return lost.valid(); }, 500ms), "up");
    {
        for(int i = 0; i < 200 && FakeBus::pending.empty(); ++i) {
            lost.handler();
            FakeClock::current += 1ms;
        }
        check(!FakeBus::pending.empty(), "a read is on the wire");
        auto       late    = FakeBus::lose();   // the bus never answers it
        auto const errors  = lost.errors();
        auto const samples = lost.samples();
        runFor(lost, 60ms);
        checkEq(lost.errors(), errors + 1, "given up on after InFlightTimeout");
        for(int i = 0; i < 200 && FakeBus::pending.empty(); ++i) {
            lost.handler();
            FakeClock::current += 1ms;
        }
        check(!FakeBus::pending.empty(), "the next read is on the wire");
        late.callback(FakeBus::Result::succeeded);   // and now the lost one answers
        lost.handler();
        FakeClock::current += 1ms;
        checkEq(lost.samples(), samples, "which is not taken for the read on the wire");
        FakeBus::complete();
        lost.handler();
        checkEq(lost.samples(), samples + 1, "that one is answered when the bus answers it");
        checkEq(lost.errors(), errors + 1, "and the one error stands");
    }

    testCase("engine: a full bus queue is not a fault: the request goes next turn");
    fresh();
    FakeBus::respond    = zeros;
    FakeBus::refuseNext = 3;
    Dev<Probe> full{};
    check(runUntil(full, [&] { return full.valid(); }, 500ms), "up");
    checkEq(FakeBus::refused, 3, "three submits were refused");
    checkEq(readsOf(Probe::Address, 0x00),
            std::size_t{1},
            "the bring-up's read went out once, on the fourth try");
    checkEq(full.errors(), 0U, "none of the refusals was an error");
    checkEq(Log::warnings, 0, "nor a line");

    testCase("engine: a bring-up again writes the items the application set, and only those");
    fresh();
    RegisterModel<1> levels{Levels::Address};
    bool             away = false;
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            return away ? FakeBus::Result::notAcknowledged : levels(a, sent, recv);
        };
    Dev<Levels> l{};
    check(runUntil(l, [&] { return l.valid(); }, 500ms), "up");
    l.set<Levels::Level>(1, 0xA1);
    l.set<Levels::Level>(3, 0xA3);
    check(runUntil(l, [&] { return !l.pending(); }, 100ms), "two items written");
    check(levels.word(0x11) == 0xA1 && levels.word(0x13) == 0xA3, "to their registers");
    check(levels.word(0x10) == 0 && levels.word(0x12) == 0, "and the other two never touched");
    auto const from = FakeBus::log.size();
    away            = true;
    check(runUntil(l, [&] { return l.absent(); }, 3s), "parked");
    away = false;
    check(runUntil(l, [&] { return l.valid() && !l.pending(); }, 5s), "back, brought up, written");
    check(writes(from) == std::vector<std::vector<std::uint8_t>>{{0x11, 0xA1}, {0x13, 0xA3}},
          "items 1 and 3 again, and neither of the two the application never set");

    testCase("engine: one item set before the bring-up leaves the others their Initial");
    fresh();
    RegisterModel<1> initial{InitialLevels::Address};
    FakeBus::respond = std::ref(initial);
    Dev<InitialLevels> il{};
    il.set<InitialLevels::Level>(2, 0x2C);
    check(runUntil(il, [&] { return il.valid() && !il.pending(); }, 500ms), "up and written");
    check(initial.word(0x12) == 0x2C, "the item the application set");
    check(initial.word(0x10) == 0x55 && initial.word(0x11) == 0x55 && initial.word(0x13) == 0x55,
          "the other three their Initial, not a default-constructed Value");
    check(il.value<InitialLevels::Level>(0) == 0x55 && il.value<InitialLevels::Level>(2) == 0x2C,
          "value<W>() agrees");
    if(failures != 0) { dump(); }
}

void knobs() {
    using namespace NetTest;
    using G = Probe::Value;

    testCase("Ticket: none() is false and never answered; a real one is true");
    fresh();
    FakeBus::respond = zeros;
    Dev<Probe> d{};
    Ticket     none{};
    check(!none && none == Ticket::none(), "a value-initialised ticket is none()");
    check(d.answer<G>(none) == Answer::pending, "and stays pending");
    auto const t1 = d.request<G>();
    check(static_cast<bool>(t1) && t1 != Ticket::none(), "request() hands out a real one");
    check(runUntil(d, [&] { return d.answer<G>(t1) == Answer::ok; }, 500ms), "answered");

    testCase("Device: the group as a tag, for generic code");
    check(d.valid(G{}) && d.latest(G{}) == d.latest<G>(), "valid() and latest()");
    check(d.seq(G{}) == d.seq<G>(), "seq()");
    std::uint32_t seen = 0;
    check(d.fresh(G{}, seen) && !d.fresh(G{}, seen), "fresh(seen)");
    check(d.fresh(G{}) && !d.fresh(G{}), "fresh()");
    auto const t2 = d.request(G{});
    check(runUntil(d, [&] { return d.answer<G>(t2) == Answer::ok; }, 500ms), "request()");
    {
        RegisterModel<1> model{Levels::Address};
        FakeBus::respond = std::ref(model);
        Dev<Levels> l{};
        check(l.set(Levels::Level{}, 2, 0x22), "set(W{}, item, value)");
        check(l.set(Levels::Level{}, 0x20), "set(W{}, value) is item 0");
        check(runUntil(l, [&] { return l.valid() && !l.pending(); }, 500ms), "written");
        check(model.word(0x10) == 0x20 && model.word(0x12) == 0x22, "both landed");
    }

    testCase("Device: period<G>() is the description's until period<G>(ms) says otherwise");
    fresh();
    FakeBus::respond = zeros;
    Dev<Probe> p{};
    check(p.period<G>() == 100ms, "the description's Period");
    check(runUntil(p, [&] { return p.valid(); }, 500ms), "up");
    p.period<G>(20ms);
    check(p.period<G>() == 20ms, "set");
    {
        auto const n = p.samples();
        runFor(p, 1s);
        auto const perSecond = p.samples() - n;
        check(perSecond >= 49 && perSecond <= 51, "fifty a second from then on");
    }
    p.period<G>(0ms);
    {
        auto const n = p.samples();
        runFor(p, 500ms);
        checkEq(p.samples(), n, "0 parks the group: nothing runs");
        check(p.valid(), "and the last sample still stands");
        auto const t = p.request<G>();
        check(runUntil(
                p,
                [&] { return p.answer<G>(t) == Answer::ok; },
                100ms),
              "until request<G>() asks");
        runFor(p, 200ms);
        checkEq(p.samples(), n + 1, "and only that once");
    }
    p.period<G>(100ms);
    {
        auto const n = p.samples();
        runFor(p, 1s);
        auto const perSecond = p.samples() - n;
        check(perSecond >= 9 && perSecond <= 11, "and ten a second once it is set again");
    }

    testCase("Device: broughtUp(seen) is true once per bring-up, and logHealth() is one line");
    fresh();
    bool away        = false;
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        return away ? alwaysNak(a, s, r) : zeros(a, s, r);
    };
    Dev<Probe>    b{};
    std::uint16_t up = 0;
    check(!b.broughtUp(up), "nothing yet");
    check(runUntil(b, [&] { return b.valid(); }, 500ms), "up");
    check(b.broughtUp(up) && !b.broughtUp(up), "once");
    away = true;
    check(runUntil(b, [&] { return b.absent(); }, 3s), "parked");
    away = false;
    check(runUntil(b, [&] { return b.valid(); }, 5s), "back");
    check(b.broughtUp(up) && !b.broughtUp(up),
          "and once again after the bring-up that brought it back");
    checkEq(up, b.bringUps(), "the cursor is the count");
    {
        auto const infos = Log::infos;
        b.logHealth();
        checkEq(Log::infos, infos + 1, "one info line");
    }
    if(failures != 0) { dump(); }
}

// -- counted reads: a length an earlier step of the run read -------------------------------

namespace CountedTest {

    /// One-byte count at 0x10, then at most four bytes of 0x20; a two-byte count at 0x30, then
    /// a bare read of at most 200.
    struct Counted {
        static constexpr std::string_view Name          = "COUNTED";
        static constexpr Address7         Address       = 0x33;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Short {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Step::read({.reg = 0x10, .count = 1, .offset = 0}),
              Step::readCounted(
                {.reg = 0x20, .countOffset = 0, .countBytes = 1, .maxCount = 4, .offset = 1})};

            struct Sample {
                std::size_t  size{};   ///< what decode was handed
                std::uint8_t last{};
            };

            [[nodiscard]] static constexpr Sample decode(Bytes data) {
                return {data.size(), data.u8(data.size() - 1)};
            }
        };

        struct Long {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Step::read({.reg = 0x30, .count = 2, .offset = 0}),
              Step::receiveCounted(
                {.countOffset = 0, .countBytes = 2, .maxCount = 200, .offset = 2})};

            struct Sample {
                std::size_t size{};
            };

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.size()}; }
        };

        using Reads = List<Short, Long>;
    };

    static_assert(bufferBytes(std::span<Step const>{Counted::Short::Steps}) == 5,
                  "a counted read's buffer is reckoned at its most");
    static_assert(detail::stepBits(Counted::Long::Steps[1],
                                   1)
                    == (1 + 200) * 9 + 2,
                  "and so is its bus load");

    // What the script checks turn down.
    static constexpr std::array WideCount{
      Step::read({.reg = 0x10, .count = 3, .offset = 0}),
      Step::readCounted(
        {.reg = 0x20, .countOffset = 0, .countBytes = 3, .maxCount = 4, .offset = 3})};
    static constexpr std::array CountNotRead{
      Step::read({.reg = 0x10, .count = 1, .offset = 0}),
      Step::readCounted(
        {.reg = 0x20, .countOffset = 1, .countBytes = 1, .maxCount = 4, .offset = 2})};
    static constexpr std::array CountAfter{
      Step::readCounted(
        {.reg = 0x20, .countOffset = 0, .countBytes = 1, .maxCount = 4, .offset = 1}),
      Step::read({.reg = 0x10, .count = 1, .offset = 0})};
    static constexpr std::array CountFromCounted{
      Step::read({.reg = 0x10, .count = 1, .offset = 0}),
      Step::readCounted(
        {.reg = 0x20, .countOffset = 0, .countBytes = 1, .maxCount = 4, .offset = 1}),
      Step::readCounted(
        {.reg = 0x20, .countOffset = 1, .countBytes = 1, .maxCount = 4, .offset = 5})};
    static constexpr std::array PastBuffer{
      Step::read({.reg = 0x10, .count = 1, .offset = 0}),
      Step::readCounted(
        {.reg = 0x20, .countOffset = 0, .countBytes = 1, .maxCount = 250, .offset = 10})};
    static_assert(scriptFault(WideCount,
                              1)
                  == ScriptFault::countWidth);
    static_assert(scriptFault(CountNotRead,
                              1)
                  == ScriptFault::countNotReadBefore);
    static_assert(scriptFault(CountAfter,
                              1)
                  == ScriptFault::countNotReadBefore);
    static_assert(scriptFault(CountFromCounted,
                              1)
                    == ScriptFault::countNotReadBefore,
                  "a counted read may not have filled the bytes a later count names");
    static_assert(scriptFault(PastBuffer,
                              1)
                  == ScriptFault::countedPastBuffer);
    static_assert(wellFormed(Counted::Short::Steps,
                             1)
                  && wellFormed(Counted::Long::Steps,
                                1));

}   // namespace CountedTest

void countedReads() {
    using namespace CountedTest;
    using Short = Counted::Short;
    using Long  = Counted::Long;

    fresh();
    std::uint8_t  shortCount = 0;
    std::uint16_t longCount  = 0;
    FakeBus::respond =
      [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(a != Counted::Address) { return FakeBus::Result::notAcknowledged; }
          auto const reg = sent.empty() ? -1 : static_cast<int>(static_cast<std::uint8_t>(sent[0]));
          if(reg == 0x10) { recv[0] = std::byte{shortCount}; }
          if(reg == 0x30) {
              recv[0] = std::byte{static_cast<std::uint8_t>(longCount >> 8U)};
              recv[1] = std::byte{static_cast<std::uint8_t>(longCount & 0xFFU)};
          }
          if(reg == 0x20 || reg == -1) {
              for(std::size_t i = 0; i < recv.size(); ++i) {
                  recv[i] = std::byte{static_cast<std::uint8_t>(0xA0 + i)};
              }
          }
          return FakeBus::Result::succeeded;
      };
    auto const readsOf = [](int reg) {
        std::vector<std::size_t> lengths;
        for(auto const& t : FakeBus::log) {
            if(!t.isRead()) { continue; }
            if(reg < 0 ? t.sent.empty() : (!t.sent.empty() && t.sent[0] == reg)) {
                lengths.push_back(t.recvLen);
            }
        }
        return lengths;
    };

    testCase(
      "counted read: a count of 0 sends nothing after the count, decode sees the count alone");
    Dev<Counted> d{};
    check(runUntil(
            d,
            [&] { return d.samples<Short>() >= 3 && d.samples<Long>() >= 3; },
            1s),
          "sampling");
    check(readsOf(0x20).empty() && readsOf(-1).empty(), "no counted transaction");
    check(readsOf(0x10).size() >= 3 && readsOf(0x30).size() >= 3, "only the counts");
    checkEq(d.latest<Short>().size, std::size_t{1}, "Short's decode sees the one count byte");
    checkEq(d.latest<Long>().size, std::size_t{2}, "Long's the two");
    checkEq(d.errors(), 0U, "and nothing failed");

    testCase("counted read: the count in range is what is read, and what decode sees");
    shortCount = 3;
    FakeBus::log.clear();
    {
        auto const n = d.samples<Short>();
        check(runUntil(d, [&] { return d.samples<Short>() > n + 1; }, 1s), "sampled");
    }
    check(!readsOf(0x20).empty() && readsOf(0x20).back() == 3, "three bytes of 0x20");
    checkEq(d.latest<Short>().size, std::size_t{4}, "offset 1 + 3");
    checkEq(d.latest<Short>().last, 0xA2U, "the third byte read last");

    testCase("counted read: a count past the step's most is clamped to it");
    shortCount = 9;
    FakeBus::log.clear();
    {
        auto const n = d.samples<Short>();
        check(runUntil(d, [&] { return d.samples<Short>() > n + 1; }, 1s), "sampled");
    }
    check(!readsOf(0x20).empty() && readsOf(0x20).back() == 4, "four, not nine");
    checkEq(d.latest<Short>().size, std::size_t{5}, "offset 1 + 4");

    testCase("counted read: a two-byte count is big-endian");
    longCount = 0x0081;   // 129; little-endian it would be 0x8100
    FakeBus::log.clear();
    {
        auto const n = d.samples<Long>();
        check(runUntil(d, [&] { return d.samples<Long>() > n + 1; }, 1s), "sampled");
    }
    check(!readsOf(-1).empty() && readsOf(-1).back() == 129, "a bare read of 129");
    checkEq(d.latest<Long>().size, std::size_t{131}, "decode sees offset 2 + 129");
    longCount = 0x0102;   // 258, past the 200
    FakeBus::log.clear();
    {
        auto const n = d.samples<Long>();
        check(runUntil(d, [&] { return d.samples<Long>() > n + 1; }, 1s), "sampled");
    }
    check(!readsOf(-1).empty() && readsOf(-1).back() == 200, "clamped to 200");
    checkEq(d.latest<Long>().size, std::size_t{202}, "offset 2 + 200");

    testCase("counted read: back to 0, decode sees the count alone again");
    shortCount = 0;
    FakeBus::log.clear();
    {
        auto const n = d.samples<Short>();
        check(runUntil(d, [&] { return d.samples<Short>() > n + 1; }, 1s), "sampled");
    }
    check(readsOf(0x20).empty(), "nothing read after the count");
    checkEq(d.latest<Short>().size, std::size_t{1}, "not what the last run read");
    if(failures != 0) { dump(); }
}

// -- wake retries: a part that NAKs until it is awake --------------------------------------

namespace WakeTest {

    struct Sleepy {
        static constexpr std::string_view Name           = "SLEEPY";
        static constexpr Address7         Address        = 0x37;
        static constexpr std::size_t      RegisterBytes  = 1;
        static constexpr std::uint8_t     WakeRetries    = 3;
        static constexpr auto             WakeRetryDelay = std::chrono::milliseconds{4};
        static constexpr std::array       Init{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        using Reads = List<Value>;
    };

    static_assert(Dev<Sleepy>::WakeRetries == 3 && Dev<PresenceTest::Probe>::WakeRetries == 0);

}   // namespace WakeTest

void wakeRetries() {
    using namespace WakeTest;

    int        naksLeft = 0;
    auto const sleepy = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(naksLeft > 0) {
            --naksLeft;
            return FakeBus::Result::notAcknowledged;
        }
        return zeros(a, s, r);
    };

    testCase("wake retries: N NAKs, then the ACK: no error, N retries, WakeRetryDelay apart");
    fresh();
    naksLeft         = 3;
    FakeBus::respond = sleepy;
    {
        Dev<Sleepy> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.errors(), 0U, "no error");
        checkEq(d.wakeRetries(), 3U, "three retries");
        checkEq(d.consecutiveNaks(), 0U, "no NAK towards parking");
        checkEq(d.bringUps(), 1U, "one bring-up");
        check(FakeBus::log.size() >= 4, "the Init read four times");
        for(std::size_t i = 1; i < 4 && i < FakeBus::log.size(); ++i) {
            check(FakeBus::log[i].at - FakeBus::log[i - 1].at >= 4ms, "each after WakeRetryDelay");
        }
    }

    testCase("wake retries: N + 1 NAKs are one NAK");
    fresh();
    naksLeft         = 4;
    FakeBus::respond = sleepy;
    {
        Dev<Sleepy> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up after the bring-up again");
        checkEq(d.errors(), 1U, "one error");
        checkEq(d.wakeRetries(), 3U, "after three retries");
        checkEq(d.bringUps(), 1U, "and one bring-up that finished");
    }

    testCase("wake retries: a parked part's probe gets them too");
    fresh();
    FakeBus::respond = alwaysNak;
    {
        Dev<Sleepy> d{};
        check(runUntil(d, [&] { return d.absent(); }, 500ms), "parked");
        checkEq(d.errors(), 3U, "after three NAKs");
        checkEq(d.wakeRetries(), 9U, "each after its three retries");
        auto const sent  = FakeBus::log.size();
        naksLeft         = 2;
        FakeBus::respond = sleepy;
        check(runUntil(d, [&] { return !d.absent(); }, 2s), "the probe brought it back");
        checkEq(d.wakeRetries(), 11U, "through two retries");
        checkEq(FakeBus::log.size() - sent, std::size_t{3}, "the probe: two NAKs and the ACK");
        checkEq(d.errors(), 3U, "and no new error");
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "sampling");
    }

    testCase("wake retries: a chip without them counts none");
    fresh();
    FakeBus::respond = alwaysNak;
    {
        Dev<PresenceTest::Probe> d{};
        check(runUntil(d, [&] { return d.absent(); }, 500ms), "parked");
        checkEq(d.wakeRetries(), 0U, "none");
        checkEq(d.errors(), 3U, "three NAKs, three errors");
    }
    if(failures != 0) { dump(); }
}

namespace MayNakTest {

    /// A part whose soft reset (0x00 = 0x10) is never acknowledged: the probe read, the reset
    /// that may NAK, then a configuration write.
    struct Resets {
        static constexpr std::string_view Name          = "RESETS";
        static constexpr Address7         Address       = 0x24;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{Step::read({.reg = 0x06, .count = 1, .offset = 0}),
                                               Step::write({.reg     = 0x00,
                                                            .payload = {0x10},
                                                            .delay   = std::chrono::milliseconds{5},
                                                            .mayNak  = true}),
                                               Step::write({.reg = 0x04, .payload = {0x22}})};

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        using Reads = List<Value>;
    };

    static constexpr std::array ProbeMayNak{
      Step::write({.reg = 0x00, .payload = {0x10}, .mayNak = true}),
      Step::read({.reg = 0x06, .count = 1})};
    static_assert(scriptFault(ProbeMayNak,
                              1,
                              false)
                    == ScriptFault::mayNakProbe,
                  "Init's first transaction is the probe and may not be one that NAKs");
    static_assert(scriptFault(ProbeMayNak,
                              1,
                              true)
                    == ScriptFault::none,
                  "a read group has no probe to protect");

    /// A two-step Init with a 1 ms gap, for the rounding of a delay to the clock's tick.
    struct OneMs {
        static constexpr std::string_view Name          = "ONEMS";
        static constexpr Address7         Address       = 0x25;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{
          Step::write({.reg = 0x00, .payload = {0x01}, .delay = std::chrono::milliseconds{1}}),
          Step::write({.reg = 0x01, .payload = {0x02}})};
    };

    /// A WHO_AM_I read, identify, then configuration writes.
    struct Identifies {
        static constexpr std::string_view Name          = "IDENT";
        static constexpr Address7         Address       = 0x26;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{Step::read({.reg = 0x0F, .count = 1, .offset = 0}),
                                               Step::identify(),
                                               Step::write({.reg = 0x20, .payload = {0x57}})};

        struct State {
            std::uint8_t id{};
        };

        [[nodiscard]] static constexpr bool setup(Bytes  data,
                                                  State& state) {
            state.id = data.u8(0);
            return state.id == 0x33;
        }
    };

    static constexpr std::array IdentifyInRead{Step::read({.reg = 0x06, .count = 1}),
                                               Step::identify()};
    static_assert(scriptFault(IdentifyInRead,
                              1,
                              true)
                    == ScriptFault::identifyOutsideInit,
                  "identify belongs to Init, whose setup() it runs");

}   // namespace MayNakTest

void mayNak() {
    using namespace MayNakTest;

    testCase("mayNak: the NAK of a write the part never acknowledges is not an error");
    fresh();
    FakeBus::respond = [](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(a != 0x24) { return FakeBus::Result::notAcknowledged; }
        if(s.size() == 2 && s[0] == std::byte{0x00}) { return FakeBus::Result::notAcknowledged; }
        return zeros(a, s, r);
    };
    {
        Dev<Resets> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.errors(), 0U, "no error");
        checkEq(d.consecutiveNaks(), 0U, "no NAK towards parking");
        checkEq(d.bringUps(), 1U, "one bring-up, not a start over");
        check(hasWrite({0x04, 0x22}), "the script went on past it");
        auto const reset = findWrite({0x00, 0x10});
        auto const conf  = findWrite({0x04, 0x22});
        check(reset < conf && conf < FakeBus::log.size()
                && FakeBus::log[conf].at - FakeBus::log[reset].at >= 5ms,
              "after the step's delay");
    }

    testCase("mayNak: a bus fault on that write is no error either");
    fresh();
    FakeBus::respond = [](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(a != 0x24) { return FakeBus::Result::notAcknowledged; }
        // the part resets inside the byte and lets go of SDA: a lost arbitration, not a NAK
        if(s.size() == 2 && s[0] == std::byte{0x00}) { return FakeBus::Result::failed; }
        return zeros(a, s, r);
    };
    {
        Dev<Resets> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.errors(), 0U, "no error");
        checkEq(d.bringUps(), 1U, "one bring-up, not a start over for every reset");
        check(hasWrite({0x04, 0x22}), "the script went on past it");
    }

    testCase("mayNak: the same write acknowledged is just as good");
    fresh();
    FakeBus::respond = zeros;
    {
        Dev<Resets> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.errors(), 0U, "no error");
    }

    testCase("identify: a part that is not this chip is not written to");
    fresh();
    {
        RegisterModel<1> other{0x26};
        other.set(0x0F, {0x44});
        FakeBus::respond = std::ref(other);
        Dev<Identifies> d{};
        runFor(d, 300ms);
        check(!d.identified() && d.unidentified() >= 1, "turned down");
        check(!hasWrite({0x20, 0x57}), "and nothing after the identify step went to it");
    }
    fresh();
    {
        RegisterModel<1> part{0x26};
        part.set(0x0F, {0x33});
        FakeBus::respond = std::ref(part);
        Dev<Identifies> d{};
        check(runUntil(d, [&] { return d.identified(); }, 300ms), "the right chip comes up");
        check(hasWrite({0x20, 0x57}), "and is configured");
    }

    testCase("delay: a 1 ms step delay is more than one tick of a millisecond clock");
    fresh();
    FakeBus::respond = zeros;
    {
        Dev<OneMs> d{};
        check(runUntil(d, [&] { return hasWrite({0x01, 0x02}); }, 200ms), "both written");
        auto const first  = findWrite({0x00, 0x01});
        auto const second = findWrite({0x01, 0x02});
        check(first < second && second < FakeBus::log.size()
                && FakeBus::log[second].at - FakeBus::log[first].at >= 2ms,
              "the completion's tick is not counted as the whole millisecond");
    }
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    presence();
    linkState();
    ticketsAndWrites();
    engine();
    nets();
    knobs();
    countedReads();
    wakeRetries();
    mayNak();
    return finish();
}
