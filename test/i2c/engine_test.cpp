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
#include <kvasir/Devices/I2C/Adapters/Watch.hpp>
#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
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
namespace Units = Kvasir::Units;

namespace {

// -- presence: parked after NAKs, probed, back at once; the nets under a script ------------

namespace PresenceTest {

    /// The smallest chip: one register read at bring-up, one cyclic read.
    struct Probe {
        static constexpr std::string_view Name          = "PROBE";
        static constexpr Address7         Address       = 0x21;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{
          Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        using Reads = Kvasir::Engine::List<Value>;
    };

    struct NeverPark {
        static constexpr std::uint8_t absentAfterNaks = 0;
    };

    /// Probe without an Init: nothing is sent at bring-up, so its first read is what answers.
    struct NoInit {
        static constexpr std::string_view Name          = "NOINIT";
        static constexpr Address7         Address       = 0x22;
        static constexpr std::size_t      RegisterBytes = 1;

        using Value = Probe::Value;
        using Reads = Kvasir::Engine::List<Value>;
    };

    struct ShortNet {
        static constexpr auto inFlightTimeout = std::chrono::milliseconds{50};
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

    I2cCase testcase{"presence: three NAKs park the device, and nothing goes out while parked"};
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

    testcase.step("presence: probes at 1, 2, 4, 8, 16, then every 30 s");
    {
        // From here on the clock steps 100 ms a turn; a probe is one transaction, NAKed.
        std::vector<std::size_t> probeTurns;
        auto                     seen = FakeBus::log.size();
        for(std::size_t t = 0; t < 1200; ++t) {   // 120 s
            d.turn();
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

    testcase.step("presence: an ACK unparks at once, and the next turns are ordinary");
    {
        FakeBus::respond  = zeros;
        auto const infos  = Log::infos;
        auto const debugs = Log::debugs;
        // The next probe is up to 30 s away: step to it.
        check(runUntil(d, [&] { return !d.absent(); }, 31s), "present again");
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "and sampling");
        checkEq(d.consecutiveNaks(), 0U, "streak cleared");
        // "present again" is info; "up", an ordinary bring-up, is debug (EngineLog.hpp)
        check(Log::infos >= infos + 1 && Log::debugs >= debugs + 1,
              "'present again' and 'up' were logged");
        checkEq(Log::warnings, 1, "no new warning");
    }

    testcase.next("presence: bus faults do not park; a NAK streak survives a fault between");
    FakeBus::respond = alwaysFault;
    Dev<Probe> f{};
    runFor(f, 300ms);
    check(!f.absent(), "still present after nothing but bus faults");
    check(f.errors() > 5, "which were all counted");
    checkEq(Log::warnings, 0, "and none of them was 'not responding'");
    {
        int        n      = 0;
        auto const script = [&](std::uint8_t, std::span<std::byte const>, std::span<std::byte>) {
            ++n;
            return n == 2 ? FakeBus::Result::failed : FakeBus::Result::nack;
        };
        FakeBus::respond = script;
        Dev<Probe> e{};
        check(runUntil(e, [&] { return e.absent(); }, 100ms), "parked: NAK, fault, NAK, NAK");
        checkEq(n, 4, "in four transactions");
    }

    testcase.next("presence: AbsentAfterNaks = 0 never parks");
    FakeBus::respond = alwaysNak;
    Dev<Probe, NeverPark> np{};
    runFor(np, 300ms);
    check(!np.absent(), "present through 300 NAKs");
    checkEq(Log::warnings, 0, "nothing said");

    testcase.next("presence: a parked chip with no Init is up again only once it has answered");
    FakeBus::respond = alwaysNak;
    {
        Dev<NoInit> n{};
        check(runUntil(n, [&] { return n.absent(); }, 1s), "parked");
        auto const debugs = Log::debugs;   // power-up said "up" (debug): nothing had failed yet
        runFor(n, 70s);                    // 70 s: seven probes
        check(!n.answering() && !n.valid(), "never up while nothing answers");
        checkEq(Log::debugs, debugs, "and never said 'up'");
        FakeBus::respond = zeros;
        check(runUntil(n, [&] { return n.answering(); }, 31s), "up at the first ACK");
        check(!n.absent(), "and present");
        check(runUntil(n, [&] { return n.valid(); }, 500ms), "and sampling");
    }

    testcase.next("presence: a flapping device does not flood the log");
    {
        // Answers exactly one probe, then NAKs again -- twenty times.
        bool             answer = false;
        ScopedHook const answering{
          FakeBus::respond,
          [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
              return answer ? zeros(a, s, r) : alwaysNak(a, s, r);
          }};
        Dev<Probe> flap{};
        for(int i = 0; i < 20; ++i) {
            answer = false;
            // three cyclic reads at 100 ms, NAKed: parked within half a second
            check(runUntil(flap, [&] { return flap.absent(); }, 1s), "parked");
            answer = true;
            check(runUntil(flap, [&] { return !flap.absent(); }, 31s), "probed and back");
        }
        // Twenty flaps in some 25 s. Every return logs "up" (one debug line each); what the
        // presence limiter lets through -- "not responding" and "present again" share one
        // backoff schedule -- is a handful, not forty.
        auto const presenceLines = Log::warnings + Log::infos;
        check(Log::debugs >= 20, "every return is brought up again");
        check(Log::warnings >= 1, "the parking line was said");
        check(presenceLines >= 2 && presenceLines <= 8, "and the presence lines are rate-limited");
    }

    testcase.next("engine: a request the bus never answers is given up on after InFlightTimeout");
    FakeBus::respond = zeros;
    Dev<Probe, ShortNet> lost{};
    check(runUntil(lost, [&] { return lost.valid(); }, 500ms), "up");
    {
        // Wait for a turn that submits, then drop the request: no callback, ever.
        bool dropped = false;
        for(int i = 0; i < 200 && !dropped; ++i) {
            lost.turn();
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

    testcase.next("engine: five faults in a row past the bring-up run the Init script again");
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
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        struct Slow {
            static constexpr auto       Period = std::chrono::milliseconds{50};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        using Reads   = Kvasir::Engine::List<Fast, Slow>;
        using Primary = Fast;
    };

}   // namespace LinkTest

namespace FeaturesTest {
    using Kvasir::Engine::EngineFeatures;
    using Kvasir::Engine::WithFeatures;
    namespace D = Kvasir::Engine::detail;

    /// The fake with every feature off, and one with only the in-flight net off.
    using MinBus = Kvasir::Engine::WithFeatures<
      FakeBus,
      Kvasir::Engine::EngineFeatures{.switchable = false, .presence = false, .inFlightNet = false}>;
    using NoNetBus = Kvasir::Engine::WithFeatures<
      FakeBus,
      Kvasir::Engine::EngineFeatures{.switchable = true, .presence = true, .inFlightNet = false}>;

    struct NoFeaturesMember {
        using Request = FakeBus::Request;
        using Result  = FakeBus::Result;
    };

    static_assert(D::featuresOf<NoFeaturesMember>() == Kvasir::Engine::EngineFeatures{},
                  "a bus that says nothing gets the defaults");
    static_assert(!Kvasir::Engine::EngineFeatures{}.switchable
                  && Kvasir::Engine::EngineFeatures{}.presence
                  && Kvasir::Engine::EngineFeatures{}.inFlightNet);
    static_assert(!std::is_same_v<D::PortOf<MinBus>,
                                  D::PortOf<FakeBus>>,
                  "other features, another port");
    static_assert(
      std::is_same_v<D::PortOf<Kvasir::Engine::WithFeatures<FakeBus,
                                                            Kvasir::Engine::AllEngineFeatures>>,
                     D::PortOf<FakeBus>>,
      "the same features, the same port: one engine");
    // What a feature that is off leaves out of every device's state.
    static_assert(sizeof(D::EngineState<D::PortOf<MinBus>,
                                        FakeClock>)
                    + sizeof(Kvasir::Engine::Presence<FakeClock>) + sizeof(D::OfflineReasons)
                  <= sizeof(D::EngineState<D::PortOf<FakeBus>,
                                           FakeClock>));

    /// Everything but the statistics.
    using NoStatsBus
      = Kvasir::Engine::WithFeatures<FakeBus,
                                     Kvasir::Engine::EngineFeatures{.switchable  = true,
                                                                    .presence    = true,
                                                                    .inFlightNet = true,
                                                                    .timestamps  = true,
                                                                    .stats       = false}>;
    template<typename Dv>
    concept HasCounters = requires(Dv const& d) {
        d.samples();
        d.errors();
        d.rejected();
        d.turns();
    };
    using CountedProbe   = Kvasir::I2C::Device<FakeBus, FakeClock, PresenceTest::Probe>;
    using UncountedProbe = Kvasir::I2C::Device<NoStatsBus, FakeClock, PresenceTest::Probe>;
    // The tables (flash) lose what the features that are off need: the shared knobs the bridge's
    // generation hook, the presence knobs and the in-flight timeout; the per-device part the
    // enable, its argument and generation, and the bridge's offline hook.
    static_assert(sizeof(D::DeviceKnobs<D::PortOf<MinBus>,
                                        FakeClock>)
                      + sizeof(void*) + sizeof(Kvasir::Engine::PresenceKnobs)
                      + sizeof(std::uint32_t)
                    <= sizeof(D::DeviceKnobs<D::PortOf<FakeBus>,
                                             FakeClock>),
                  "the knobs have no fields for the features that are off");
    static_assert(sizeof(D::DeviceOps<D::PortOf<MinBus>,
                                      FakeClock>)
                      + 3 * sizeof(void*) + sizeof(std::uint32_t)
                    <= sizeof(D::DeviceOps<D::PortOf<FakeBus>,
                                           FakeClock>),
                  "the per-device table has no switch fields when nothing is switchable");
    // The layout of the three tables, pinned: the chip table is generated from one list (Hooks.hpp) and its
    // order is the order of that list; the per-device tables are written by hand in a measured order with their
    // feature tags. Host (64-bit) values. A change here is a change of every device's flash table.
#if defined(__x86_64__) || defined(__aarch64__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Winvalid-offsetof"
    using OpsT   = D::Ops<D::PortOf<FakeBus>, FakeClock>;
    using KnobsT = D::DeviceKnobs<D::PortOf<FakeBus>, FakeClock>;
    using DevT   = D::DeviceOps<D::PortOf<FakeBus>, FakeClock>;
    static_assert(sizeof(OpsT) == 296
                    && offsetof(OpsT,
                                script)
                         == 0
                    && offsetof(OpsT,
                                startupDelayMs)
                         == 128
                    && offsetof(OpsT,
                                name)
                         == 272
                    && offsetof(OpsT,
                                registerBytes)
                         == 288
                    && offsetof(OpsT,
                                initEmpty)
                         == 289,
                  "the chip table's layout");
    static_assert(sizeof(KnobsT) == 80
                    && offsetof(KnobsT,
                                gateGeneration)
                         == 16
                    && offsetof(KnobsT,
                                bridgeUnpowered)
                         == 70,
                  "the shared knobs' layout, the bridge fields included");
    static_assert(sizeof(DevT) == 56
                    && offsetof(DevT,
                                enabled)
                         == 16
                    && offsetof(DevT,
                                gateOffline)
                         == 40
                    && offsetof(DevT,
                                address)
                         == 48,
                  "the per-device table's layout, the enable and bridge fields included");
    #pragma GCC diagnostic pop
#endif
    /// Without a run-time period: no period<G>(ms) and no check of a stored one per reschedule.
    /// ReadSlotBase alone does not shrink (the 8-byte time point's padding takes the 5 bytes),
    /// but a device's slot does: its own members move into the base's freed tail - a
    /// feature matrix measured -32 B of Bus RAM for 8 devices.
    using FixedBus
      = Kvasir::Engine::WithFeatures<FakeBus,
                                     Kvasir::Engine::EngineFeatures{.switchable    = true,
                                                                    .presence      = true,
                                                                    .inFlightNet   = true,
                                                                    .timestamps    = true,
                                                                    .stats         = true,
                                                                    .runtimePeriod = false}>;
    template<typename Dv>
    concept HasPeriodSetter = requires(Dv& d) {
        d.template period<PresenceTest::Probe::Value>(std::chrono::milliseconds{5});
    };
    static_assert(HasPeriodSetter<CountedProbe>
                    && !HasPeriodSetter<Kvasir::I2C::Device<FixedBus,
                                                            FakeClock,
                                                            PresenceTest::Probe>>,
                  "period<G>(ms) exists only with the run-time period");
    static_assert(HasCounters<CountedProbe> && !HasCounters<UncountedProbe>,
                  "the counters exist only with the stats feature");
    static_assert(sizeof(UncountedProbe) < sizeof(CountedProbe),
                  "and take no RAM without it");
}   // namespace FeaturesTest

void features() {
    using namespace PresenceTest;
    using namespace FeaturesTest;

    I2cCase testcase{
      "features: without presence, a part that NAKs is never parked and is asked on"};
    FakeBus::respond = alwaysNak;
    Kvasir::I2C::Device<MinBus, FakeClock, Probe> d{};
    runFor(d, 500ms);
    check(!d.absent(), "not parked");
    checkEq(d.consecutiveNaks(), 0U, "no streak is kept");
    {
        auto const sent = FakeBus::log.size();
        runFor(d, 500ms);   // a parked part would send nothing here (first probe after 1 s)
        check(FakeBus::log.size() > sent, "still asked");
    }

    testcase.next(
      "features: without the in-flight net, a request the bus never answers is waited for");
    FakeBus::respond = zeros;
    Kvasir::I2C::Device<NoNetBus, FakeClock, Probe, ShortNet> lost{};
    check(runUntil(lost, [&] { return lost.valid(); }, 500ms), "up");
    {
        bool dropped = false;
        for(int i = 0; i < 200 && !dropped; ++i) {
            lost.turn();
            if(!FakeBus::pending.empty()) {
                FakeBus::pending.clear();
                dropped = true;
            }
            FakeClock::current += 1ms;
        }
        check(dropped, "a request was dropped");
        auto const errors = lost.errors();
        runFor(lost, 200ms);   // four times ShortNet's 50 ms
        checkEq(lost.errors(), errors, "not given up on");
        checkEq(Log::warnings, 0, "and nothing said");
    }

    testcase.step(
      "features: a bus with request deadlines replaces the in-flight net: the engine stamps the "
      "deadline");
    {
        struct DeadlineTag {};

        using DeadlineBus = FakeBusFor<DeadlineTag, FakeDeadlineRequest>;
        fresh();
        DeadlineBus::reset();
        DeadlineBus::respond = zeros;
        Kvasir::I2C::Device<DeadlineBus, FakeClock, Probe, ShortNet> dd{};
        for(int i = 0; i < 500 && !dd.valid();
            ++i) {   // the harness's turn() knows tagged buses only
            dd.turn();
            DeadlineBus::complete();
            FakeClock::current += 1ms;
        }
        check(dd.valid(), "up");
        for(int i = 0; i < 200 && DeadlineBus::pending.empty(); ++i) {
            dd.turn();
            FakeClock::current += 1ms;
        }
        check(!DeadlineBus::pending.empty(), "a read is on the wire");
        auto const left = DeadlineBus::pending.front().deadline - FakeClock::now();
        check(left > 0ms && left <= ShortNet::inFlightTimeout,
              "deadline = submit + the in-flight timeout");
        auto const late
          = DeadlineBus::lose();   // the bus answers it at its deadline, not the engine
        auto const errors = dd.errors();
        for(int i = 0; i < 200; ++i) {
            dd.turn();
            FakeClock::current += 1ms;
        }
        checkEq(dd.errors(), errors, "the engine's own net does not give up on it");
        late.callback(DeadlineBus::Result::failed);   // the bus's timedOut, as the engine sees it
        dd.turn();
        checkEq(dd.errors(), errors + 1, "the bus's answer is the one failure");
        DeadlineBus::reset();
    }

    testcase.step("features: without resting turns every loop turn is a full turn");
    {
        using NoRestBus = Kvasir::Engine::
          WithFeatures<FakeBus, Kvasir::Engine::EngineFeatures{.switchable = true, .rest = false}>;
        fresh();
        FakeBus::respond = zeros;
        Kvasir::I2C::Device<NoRestBus, FakeClock, Probe> busy{};
        Dev<Probe>                                       calm{};
        check(runUntil(
                all(busy, calm),
                [&] { return busy.valid() && calm.valid(); },
                500ms),
              "both up");
        auto const busyTurns = busy.turns();
        auto const calmTurns = calm.turns();
        runFor(all(busy, calm), 50ms);   // half a period of nothing to do
        checkEq(busy.turns() - busyTurns, 50U, "without resting: one full turn a millisecond");
        check(calm.turns() - calmTurns < 10U, "with it: the turns with nothing due are skipped");
    }

    testcase.step("features: without fault back-off a failing bring-up is tried again at once");
    {
        using NoBackoffBus
          = Kvasir::Engine::WithFeatures<FakeBus,
                                         Kvasir::Engine::EngineFeatures{.switchable   = true,
                                                                        .faultBackoff = false}>;
        fresh();
        FakeBus::respond = alwaysFault;   // bus faults: presence does not park for them
        Kvasir::I2C::Device<NoBackoffBus, FakeClock, Probe> eager{};
        runFor(eager, 100ms);
        auto const eagerTries = FakeBus::log.size();
        fresh();
        FakeBus::respond = alwaysFault;
        Dev<Probe> patient{};
        runFor(patient, 100ms);
        // A try is three turns (reset, settle, the Init read): every ~3 ms without the back-off,
        // every ~8 ms with the default 5 ms (measured: 33 against 13 in 100 ms).
        check(eagerTries > 2 * FakeBus::log.size(),
              "more than twice the tries without the back-off");
    }

    testcase.next("features: without statistics a device comes up, reads, and logs its health");
    FakeBus::respond = zeros;
    UncountedProbe quiet{};
    check(runUntil(quiet, [&] { return quiet.valid(); }, 500ms), "up and reading");
    auto const debugs = Log::debugs;
    quiet.logHealth();
    checkEq(Log::debugs, debugs + 1, "one health line, without counters");
}

void linkState() {
    using LinkTest::TwoGroups;

    I2cCase const testcase{
      "Link: a chip without Init is starting until its part acknowledges something"};
    RegisterModel<1, 1> model{TwoGroups::Address};
    model.set(0x00, {7, 9});
    bool             answer = false;
    ScopedHook const answering{
      FakeBus::respond,
      [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
          return answer ? model(a, sent, recv) : FakeBus::Result::nack;
      }};
    Dev<TwoGroups> d{};
    check(d.link() == Kvasir::Link::starting, "starting before anything happened");
    turn(d);
    turn(d);
    check(!d.answering() && !d.absent(), "brought up, but nothing acknowledged: starting");
    check(runUntil(d, [&] { return d.absent(); }, 200ms), "NAKs in a row park it");
    answer = true;
    check(runUntil(d, [&] { return d.answering(); }, 3s), "answering with the probe's ACK");
    check(runUntil(d, [&] { return d.valid(); }, 200ms), "valid once both cyclic groups delivered");
    checkEq(d.latest(), std::uint8_t{7}, "latest() without a group is the Primary's");
    checkEq(d.seq(), d.seq<TwoGroups::Fast>(), "and so is seq()");

    testcase.step("Link: a bring-up forgets the readings from before it, and bringUps() counts it");
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

    testcase.step("Link: fresh(seen) gives every reader every sample");
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
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0}),
              Kvasir::Engine::Step::check(2ms),
              Kvasir::Engine::Step::read({.reg = 0x01, .count = 1, .offset = 1})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr bool ready(Kvasir::Bytes data) {
                return (data.u8(0) & 1U) != 0;
            }

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(1); }
        };

        struct Level {
            using Value                        = std::uint8_t;
            static constexpr std::size_t Bytes = 1;

            [[nodiscard]] static constexpr Kvasir::Engine::Step
            encode(Value const&         value,
                   std::span<std::byte> buffer) {
                buffer[0] = std::byte{value};
                return Kvasir::Engine::Step::writeBuffer({.reg = 0x10, .offset = 0, .count = 1});
            }
        };

        struct Clear {
            using Value                              = std::uint8_t;
            static constexpr std::size_t Bytes       = 1;
            static constexpr bool        AlwaysWrite = true;

            [[nodiscard]] static constexpr Kvasir::Engine::Step
            encode(Value const&         value,
                   std::span<std::byte> buffer) {
                buffer[0] = std::byte{value};
                return Kvasir::Engine::Step::writeBuffer({.reg = 0x11, .offset = 0, .count = 1});
            }
        };

        using Reads  = Kvasir::Engine::List<Page>;
        using Writes = Kvasir::Engine::List<Level, Clear>;
    };

}   // namespace TicketTest

void ticketsAndWrites() {
    using TicketTest::OnDemand;
    using P = OnDemand::Page;
    using L = OnDemand::Level;
    using C = OnDemand::Clear;

    I2cCase testcase{"request: a ticket is pending until its run ends, then says how it went"};
    RegisterModel<1, 1> model{OnDemand::Address};
    model.set(0x00, {0x01, 42});
    bool             nak = false;
    ScopedHook const answering{
      FakeBus::respond,
      [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
          return nak ? FakeBus::Result::nack : model(a, sent, recv);
      }};
    Dev<OnDemand> d{};
    auto const    t1 = d.request<P>();
    check(d.answer<P>(t1) == Kvasir::Engine::Answer::pending, "pending before the run");
    check(runUntil(
            d,
            [&] { return d.answer<P>(t1) != Kvasir::Engine::Answer::pending; },
            50ms),
          "answered");
    check(d.answer<P>(t1) == Kvasir::Engine::Answer::ok && d.latest() == 42, "ok, with the sample");
    check(d.answer<P>(Kvasir::Engine::Ticket{}) == Kvasir::Engine::Answer::pending,
          "a ticket nobody was given is never answered");

    testcase.step("request: two requests made before the run are served by that one run");
    auto const reads = model.reads;
    auto const t2    = d.request<P>();
    auto const t3    = d.request<P>();
    check(runUntil(
            d,
            [&] { return d.answer<P>(t3) != Kvasir::Engine::Answer::pending; },
            50ms),
          "answered");
    check(d.answer<P>(t2) == Kvasir::Engine::Answer::ok
            && d.answer<P>(t3) == Kvasir::Engine::Answer::ok,
          "both ok");
    checkEq(model.reads - reads, 2, "one run: its two reads");

    testcase.step("request: a run whose check never passes answers rejected");
    model.set(0x00, {0x00});
    auto const t4 = d.request<P>();
    check(runUntil(
            d,
            [&] { return d.answer<P>(t4) != Kvasir::Engine::Answer::pending; },
            300ms),
          "answered");
    check(d.answer<P>(t4) == Kvasir::Engine::Answer::rejected, "rejected once the retries ran out");
    model.set(0x00, {0x01});

    testcase.step(
      "request: a NAK answers failed, and one made while the part is away waits for it");
    nak = true;
    for(int k = 0; k < 3; ++k) {
        auto const t = d.request<P>();
        check(runUntil(
                d,
                [&] { return d.answer<P>(t) != Kvasir::Engine::Answer::pending; },
                50ms),
              "answered");
        check(d.answer<P>(t) == Kvasir::Engine::Answer::failed, "failed");
    }
    check(runUntil(d, [&] { return d.absent(); }, 50ms), "three NAKs park it");
    auto const t5 = d.request<P>();
    runFor(d, 100ms);
    check(d.answer<P>(t5) == Kvasir::Engine::Answer::pending,
          "still pending while the part is away");
    nak = false;
    check(runUntil(
            d,
            [&] { return d.answer<P>(t5) != Kvasir::Engine::Answer::pending; },
            5s),
          "answered once it is back");
    check(d.answer<P>(t5) == Kvasir::Engine::Answer::ok && d.answering(),
          "ok: the probe was the request's own run");

    testcase.step("set: the value the chip holds is not sent again; rewrite() and modify() behave");
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

    testcase.step("set: an AlwaysWrite group sends every value, the one it holds too");
    auto const w1 = model.writes;
    check(d.set<C>(1) && runUntil(d, [&] { return !d.pending<C>(); }, 50ms), "set and written");
    check(d.set<C>(1)
            && runUntil(
              d,
              [&] { return !d.pending<C>(); },
              50ms),
          "the same value, set and written again");
    checkEq(model.writes - w1, 2, "two writes");

    testcase.next(
      "StallWatch: a part that answers and delivers nothing new is caught, and not every turn");
    RegisterModel<1, 1> quietModel{OnDemand::Address};
    quietModel.set(0x00, {0x01, 7});
    FakeBus::respond = std::ref(quietModel);
    Dev<OnDemand> quiet{};
    auto const    first = quiet.request<P>();
    check(runUntil(
            quiet,
            [&] { return quiet.answer<P>(first) == Kvasir::Engine::Answer::ok; },
            50ms),
          "answering");
    StallWatch<Dev<OnDemand>, P> watch{std::chrono::seconds{3}, std::chrono::seconds{10}};
    int                          fired = 0;
    auto const                   run   = [&](int ms) {
        for(int i = 0; i < ms; ++i) {
            if(watch.turn(quiet, FakeClock::now())) { ++fired; }
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

    testcase.next(
      "AbsentWatch: a part that stops answering is reported after a while, and not every turn");
    FakeBus::respond = std::ref(quietModel);
    Dev<OnDemand> there{};
    auto const    asked = there.request<P>();
    check(runUntil(
            there,
            [&] { return there.answer<P>(asked) == Kvasir::Engine::Answer::ok; },
            50ms),
          "answering");
    AbsentWatch<Dev<OnDemand>> lost{std::chrono::seconds{2}, std::chrono::seconds{10}};
    int                        gone   = 0;
    auto const                 watch2 = [&](int ms) {
        for(int i = 0; i < ms; ++i) {
            if(lost.turn(there, FakeClock::now())) { ++gone; }
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

    static constexpr std::array Init{
      Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};

    struct Status {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{
          Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        struct Sample {
            std::uint8_t v{};
        };

        static constexpr Sample decode(Kvasir::Bytes b) { return {b.u8(0)}; }
    };

    struct Config {
        using Value                                 = std::uint8_t;
        static constexpr std::size_t Bytes          = 1;
        static constexpr auto        VerifyDelay    = std::chrono::milliseconds{10};
        static constexpr auto        VerifyInterval = std::chrono::milliseconds{500};

        static constexpr Kvasir::Engine::Step encode(Value const&         v,
                                                     std::span<std::byte> buf) {
            buf[0] = static_cast<std::byte>(v);
            return Kvasir::Engine::Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 1});
        }
    };

    using Reads  = Kvasir::Engine::List<Status>;
    using Writes = Kvasir::Engine::List<Config>;
};

/// The same, but the top bit of the register is a status flag the chip owns, so a
/// read-back that differs only there is not a mismatch.
struct VerifyMaskChip {
    static constexpr std::string_view Name          = "VERIFYMASK";
    static constexpr Address7         Address       = 0x41;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array Init{
      Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};

    struct Config {
        using Value                              = std::uint8_t;
        static constexpr std::size_t Bytes       = 1;
        static constexpr auto        VerifyDelay = std::chrono::milliseconds{10};

        static constexpr Kvasir::Engine::Step encode(Value const&         v,
                                                     std::span<std::byte> buf) {
            buf[0] = static_cast<std::byte>(v);
            return Kvasir::Engine::Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 1});
        }

        // Bytes the type has to be spelt out: a write group's own Bytes member shadows it.
        static constexpr bool verify(Kvasir::Bytes want,
                                     Kvasir::Bytes got) {
            return (want.u8(0) & 0x7FU) == (got.u8(0) & 0x7FU);
        }
    };

    using Writes = Kvasir::Engine::List<Config>;
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
          Kvasir::Engine::Step::readIndirect({.regOffset = 0, .count = 64, .offset = 1})};

        static constexpr std::size_t prepare(Request const&       r,
                                             std::span<std::byte> buf) {
            buf[0] = static_cast<std::byte>(r.address);
            return r.length;
        }

        struct Sample {
            std::uint8_t                 length{};
            std::array<std::uint8_t, 64> data{};
        };

        static constexpr Sample decode(Kvasir::Bytes b) {
            Sample s{};
            s.length = static_cast<std::uint8_t>(b.size() - 1);
            for(std::size_t i = 0; i + 1 < b.size(); ++i) { s.data[i] = b.u8(1 + i); }
            return s;
        }
    };

    using Reads = Kvasir::Engine::List<Page>;
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
template<typename Timing = Kvasir::Engine::DefaultTiming>
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

    static constexpr std::array Init{Kvasir::Engine::Step::command({.payload = {0xAE}}),
                                     Kvasir::Engine::Step::command({.payload = {0xAF}})};

    struct Status {
        static constexpr auto       Period = std::chrono::milliseconds{2};
        static constexpr std::array Steps{Kvasir::Engine::Step::receive({.count = 1})};

        struct Sample {
            std::uint8_t v{};
        };

        static constexpr Sample decode(Kvasir::Bytes b) { return {b.u8(0)}; }
    };

    struct Page {
        using Value                        = std::array<std::uint8_t, 8>;
        static constexpr std::size_t Items = 2;
        static constexpr std::size_t Bytes = 4 + 1 + 8;

        static constexpr std::array<Kvasir::Engine::Step,
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
            return {Kvasir::Engine::Step::commandBuffer({.offset = 0, .count = 4, .delay = Gap}),
                    Kvasir::Engine::Step::commandBuffer({.offset = 4, .count = 9})};
        }
    };

    struct Contrast {
        using Value                        = std::uint8_t;
        static constexpr std::size_t Bytes = 3;

        static constexpr Kvasir::Engine::Step encode(Value const&         v,
                                                     std::span<std::byte> buf) {
            buf[0] = std::byte{0x00};
            buf[1] = std::byte{0x81};
            buf[2] = static_cast<std::byte>(v);
            return Kvasir::Engine::Step::commandBuffer({.offset = 0, .count = 3});
        }
    };

    using Reads  = Kvasir::Engine::List<Status>;
    using Writes = Kvasir::Engine::List<Page, Contrast>;
};

// A chip whose writes are one transaction each carries no script store at all: the empty
// [[no_unique_address]] member, the same trick verify_ uses on a chip that does not verify.
static_assert(std::is_empty_v<Kvasir::Engine::detail::WriteScript<false,
                                                                  1>>);
static_assert(Dev<Chips::Ht16k33>::MaxWriteSteps == 1);
static_assert(Dev<PageChip<>>::MaxWriteSteps == 2);

/// Acks anything at 0x3C and answers a bare read with 0x5A.
FakeBus::Result pageOk(std::uint8_t               addr,
                       std::span<std::byte const> sent,
                       std::span<std::byte>       recv) {
    static_cast<void>(sent);
    if(addr != 0x3C) { return FakeBus::Result::nack; }
    for(auto& b : recv) { b = std::byte{0x5A}; }
    return FakeBus::Result::ok;
}

void engine() {
    I2cCase testcase{"engine: absent device parked, probe is the first bring-up step"};
    FakeBus::respond = {};
    Dev<Chips::Bme280> d{};
    check(runUntil(d, [&] { return d.absent(); }, 100ms), "parked after three NAKs");
    auto const n = FakeBus::submitted;
    runFor(d, 500ms);
    check(FakeBus::submitted <= n + 1, "at most one probe in half a second");
    // The BME280 names its oracle, so its bring-up opens with the identity read of 0xD0 -- and
    // that, not the soft reset the Init script starts with, is what an absent part is probed with.
    check(FakeBus::log.back().isRead()
            && FakeBus::log.back().sent == std::vector<std::uint8_t>{0xD0},
          "the probe is the identity read, which changes nothing on whatever answers");

    testcase.next("engine: a NAK in a read group loses that sample, the period continues");
    RegisterModel<1> const m{0x20};
    bool                   nak = false;
    ScopedHook const       answering{
      FakeBus::respond,
      [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(nak) { return FakeBus::Result::nack; }
          if(!sent.empty()) { return addr == 0x20 ? FakeBus::Result::ok : FakeBus::Result::nack; }
          recv[0] = std::byte{0x55};
          return FakeBus::Result::ok;
      }};
    Dev<Chips::Pcf8574> p{};
    check(runUntil(p, [&] { return p.samples() == 2; }, 200ms), "two samples");
    nak = true;
    runFor(p, 60ms);
    nak             = false;
    auto const errs = p.errors();
    check(errs >= 1 && errs <= 2, "one or two errors in 60 ms");
    check(!p.absent(), "not parked by one lost read");
    check(runUntil(p, [&] { return p.samples() == 4; }, 200ms), "and back to sampling");

    testcase.next("engine: a failed write stays owed");
    nak                   = false;
    std::uint8_t     port = 0;
    ScopedHook const answeringAgain{
      FakeBus::respond,
      [&](std::uint8_t, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(nak) { return FakeBus::Result::nack; }
          if(!sent.empty()) {
              port = static_cast<std::uint8_t>(sent[0]);
              return FakeBus::Result::ok;
          }
          recv[0] = std::byte{0};
          return FakeBus::Result::ok;
      }};
    Dev<Chips::Pcf8574> q{};
    check(runUntil(q, [&] { return q.samples() == 1; }, 100ms), "up");
    nak = true;
    q.set<Chips::Pcf8574::Port>(0x3C);
    runFor(q, 10ms);
    check(q.pending(), "still pending while the chip NAKs");
    nak = false;
    check(runUntil(q, [&] { return port == 0x3C; }, 100ms), "written once it answers");

    testcase.next("engine: request() runs a group at once");
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
    testcase.next("engine: a write group without Initial is re-sent after a reset");
    RegisterModel<1, 2> lim{0x18};
    lim.set(0x06, {0x00, 0x54, 0x04, 0x00});
    lim.set(0x05, {0x01, 0x90});
    lim.readOnly            = {0x05, 0x06, 0x07};
    bool             limNak = false;
    ScopedHook const answeringThird{
      FakeBus::respond,
      [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(limNak) { return FakeBus::Result::nack; }
          if(addr == 0x18 && sent.size() == 2 && recv.empty()
             && static_cast<std::uint8_t>(sent[0]) == 0x08)
          {
              return FakeBus::Result::ok;   // the one 8-bit register
          }
          return lim(addr, sent, recv);
      }};
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

    testcase.next("engine: a Transient write group is not replayed");
    RegisterModel<1> rtc2{0x68};
    rtc2.set(0x00, {0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00});
    bool             rtcNak = false;
    ScopedHook const answeringFourth{
      FakeBus::respond,
      [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(rtcNak) { return FakeBus::Result::nack; }
          return rtc2(addr, sent, recv);
      }};
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

    testcase.next("engine: a Transient write survives the base's error-threshold reset");
    RegisterModel<1> rtc4{0x68};
    rtc4.set(0x00, {0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00});
    bool             wireBroken = false;
    ScopedHook const answeringFifth{
      FakeBus::respond,
      [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(wireBroken) { return FakeBus::Result::failed; }
          return rtc4(addr, sent, recv);
      }};
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

    testcase.next("engine: modify<W>() edits one field and leaves the rest");
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

    testcase.next("engine: a written register is read back and matches");
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

    testcase.next("engine: a register that lost its value is written again");
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

    testcase.next("engine: a register that never holds its value is not rewritten for ever");
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

    testcase.next("engine: verify() decides what counts as a match");
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

    testcase.next("engine: an on-demand group does not hold valid() down");
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

    testcase.next("engine: prepare() sizes the read");
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

    testcase.next("engine: a write group that encodes to two Steps goes out as two transactions");
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

    testcase.step("engine: each dirty item is a whole sequence before any other group runs");
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

    testcase.step("engine: a single-step group on the same chip is unaffected");
    base = FakeBus::log.size();
    pg.set<P0::Contrast>(0x20);
    check(runUntil(pg, [&] { return !pg.pending(); }, 200ms), "contrast written");
    check(writes(base) == Wire{{0x00, 0x81, 0x20}}, "exactly one transaction");

    testcase.next("engine: a failure on the second step retries the item from the first");
    int              nakData = 1;   // NAK the data transaction once
    ScopedHook const answeringSixth{
      FakeBus::respond,
      [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(addr == 0x3C && !sent.empty() && static_cast<std::uint8_t>(sent[0]) == 0x40
             && nakData > 0)
          {
              --nakData;
              return FakeBus::Result::nack;
          }
          return pageOk(addr, sent, recv);
      }};
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

    testcase.next("engine: a delay between the two steps is honoured, and nothing gets between");
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
        static constexpr std::array       Init{
          Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0}),
          Kvasir::Engine::Step::write({.reg = 0x01, .payload = {0x11}}),
          Kvasir::Engine::Step::write({.reg = 0x02, .payload = {0x22}})};

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x03, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        using Reads = Kvasir::Engine::List<Value>;
    };

    /// A decode that always asks to be run again: a busy bit that never clears.
    struct Retrying {
        static constexpr std::string_view Name          = "RETRY";
        static constexpr Address7         Address       = 0x2E;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Data {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Kvasir::Engine::Outcome<Sample> decode(Kvasir::Bytes) {
                return Kvasir::Engine::Outcome<Sample>::retry(1ms);
            }
        };

        using Reads = Kvasir::Engine::List<Data>;
    };

    struct TwoRetries {
        static constexpr std::uint8_t maxRetries = 2;
    };

    /// Four registers the application may set one at a time, with no Initial.
    struct Levels {
        static constexpr std::string_view Name          = "LEVELS";
        static constexpr Address7         Address       = 0x2F;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{
          Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        using Value = Probe::Value;

        struct Level {
            using Value                        = std::uint8_t;
            static constexpr std::size_t Items = 4;
            static constexpr std::size_t Bytes = 1;

            [[nodiscard]] static constexpr Kvasir::Engine::Step
            encode(Value const&         value,
                   std::size_t          item,
                   std::span<std::byte> buffer) {
                buffer[0] = std::byte{value};
                return Kvasir::Engine::Step::writeBuffer(
                  {.reg = static_cast<std::uint16_t>(0x10 + item), .offset = 0, .count = 1});
            }
        };

        using Reads  = Kvasir::Engine::List<Value>;
        using Writes = Kvasir::Engine::List<Level>;
    };

    /// The same four registers with an Initial every item starts from.
    struct InitialLevels {
        static constexpr std::string_view Name          = "INITLEVELS";
        static constexpr Address7         Address       = 0x2C;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{
          Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        using Value = Probe::Value;

        struct Level {
            using Value                        = std::uint8_t;
            static constexpr std::size_t Items = 4;
            static constexpr std::size_t Bytes = 1;
            static constexpr Value       Initial{0x55};

            [[nodiscard]] static constexpr Kvasir::Engine::Step
            encode(Value const&         value,
                   std::size_t          item,
                   std::span<std::byte> buffer) {
                buffer[0] = std::byte{value};
                return Kvasir::Engine::Step::writeBuffer(
                  {.reg = static_cast<std::uint16_t>(0x10 + item), .offset = 0, .count = 1});
            }
        };

        using Reads  = Kvasir::Engine::List<Value>;
        using Writes = Kvasir::Engine::List<Level>;
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

    /// The fault backoff knobs (EngineDefaults::faultBackoff, faultBackoffLongest, faultBackoffResetOn).
    struct FirstPause2ms {
        static constexpr auto faultBackoff = std::chrono::milliseconds{2};
    };

    struct Ceiling15ms {
        static constexpr auto faultBackoffLongest = std::chrono::milliseconds{15};
    };

    struct ResetOnAck {
        static constexpr Kvasir::Engine::BackoffReset faultBackoffResetOn
          = Kvasir::Engine::BackoffReset::anyAck;
    };

    /// A ThreeStep whose last Init step is NAKed @p naks times: the pause between each NAKed write and the read
    /// that starts the bring-up over (each bring-up ACKs its first two steps before the NAK).
    template<typename Config = Kvasir::Engine::DefaultConfig>
    std::vector<std::chrono::milliseconds> bringUpPauses(int naks);

    /// A measured gap is the pause plus the turns that see the NAK and leave the reset (3 ms here).
    constexpr bool pauseOf(std::chrono::milliseconds gap,
                           std::chrono::milliseconds pause) {
        return gap >= pause && gap < pause + 4ms;
    }

    template<typename Config>
    std::vector<std::chrono::milliseconds> bringUpPauses(int naks) {
        RegisterModel<1> model{ThreeStep::Address};
        ScopedHook const failing{
          FakeBus::respond,
          [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
              if(naks > 0 && recv.empty() && !sent.empty()
                 && static_cast<std::uint8_t>(sent[0]) == 0x02)
              {
                  --naks;
                  return FakeBus::Result::nack;
              }
              return model(a, sent, recv);
          }};
        FakeBus::log.clear();
        Dev<ThreeStep, Config> d{};
        check(runUntil(d, [&] { return d.answering(); }, 2s), "up after the NAKed bring-ups");
        std::vector<std::chrono::milliseconds> gaps{};
        for(std::size_t i = 0; i + 1 < FakeBus::log.size(); ++i) {
            auto const& w = FakeBus::log[i];
            if(w.recvLen == 0 && !w.sent.empty() && w.sent[0] == 0x02
               && FakeBus::log[i + 1].recvLen != 0 && !FakeBus::log[i + 1].sent.empty()
               && FakeBus::log[i + 1].sent[0]
                    == 0x00) {   // a NAKed last step, then the restart's first read
                gaps.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
                  FakeBus::log[i + 1].at - w.at));
            }
        }
        return gaps;
    }

}   // namespace NetTest

/// Found by a feature matrix on hardware, not by these tests: a verified write group decided
/// by its `writes` counter, which the statistics feature had taken away.
void statsOffVerify() {
    I2cCase const testcase{
      "features: without statistics a verified write is written once, read back, and kept"};
    RegisterModel<1> vs{0x40};
    vs.set(0x00, {0x00});
    FakeBus::respond = std::ref(vs);
    Kvasir::I2C::Device<FeaturesTest::NoStatsBus, FakeClock, VerifyChip> x{};
    check(runUntil(x, [&] { return x.valid(); }, 500ms), "up");
    x.set<VerifyChip::Config>(0x77);
    check(runUntil(x, [&] { return x.writes<VerifyChip::Config>() == 1; }, 400ms), "written");
    auto const at = FakeClock::current;
    while(FakeClock::current - at < 1200ms) { turn(x); }   // past two verify intervals
    checkEq(x.writes<VerifyChip::Config>(), 1U, "the read-back matched: never rewritten");
}

void nets() {
    using namespace NetTest;

    I2cCase testcase{
      "engine: a NAK on a later Init step starts the bring-up over, after FaultBackoff"};
    RegisterModel<1> three{ThreeStep::Address};
    int              nakLeft = 1;
    ScopedHook const answering{
      FakeBus::respond,
      [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(nakLeft > 0 && recv.empty() && !sent.empty()
             && static_cast<std::uint8_t>(sent[0]) == 0x02)
          {
              --nakLeft;
              return FakeBus::Result::nack;
          }
          return three(a, sent, recv);
      }};
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

    testcase.next(
      "engine: a bring-up that keeps failing waits twice as long each time, a completed one starts "
      "over");
    {
        auto const gaps = bringUpPauses(3);
        check(gaps.size() == 3, "three NAKed bring-ups");
        if(gaps.size() == 3) {
            check(pauseOf(gaps[0], 5ms), "the first pause: FaultBackoff");
            check(pauseOf(gaps[1], 10ms),
                  "the second: twice that, the ACKs before the NAK do not reset it");
            check(pauseOf(gaps[2], 20ms), "the third: twice again");
        }
    }

    testcase.next("engine: Config::faultBackoff sets the first pause the doubling starts from");
    {
        auto const gaps = bringUpPauses<FirstPause2ms>(3);
        check(gaps.size() == 3, "three NAKed bring-ups");
        if(gaps.size() == 3) {
            check(pauseOf(gaps[0], 2ms), "2 ms");
            check(pauseOf(gaps[1], 4ms), "4 ms");
            check(pauseOf(gaps[2], 8ms), "8 ms");
        }
    }

    testcase.next("engine: Config::faultBackoffLongest is the ceiling the pause doubles up to");
    {
        auto const gaps = bringUpPauses<Ceiling15ms>(4);
        check(gaps.size() == 4, "four NAKed bring-ups");
        if(gaps.size() == 4) {
            check(pauseOf(gaps[0], 5ms) && pauseOf(gaps[1], 10ms), "5, then 10 ms");
            check(pauseOf(gaps[2], 15ms), "then 15 ms, not 20");
            check(pauseOf(gaps[3], 15ms), "and 15 ms from there on");
        }
    }

    testcase.next(
      "engine: Config::faultBackoffResetOn = anyAck starts the pause over at every ACK");
    {
        auto const gaps = bringUpPauses<ResetOnAck>(3);
        check(gaps.size() == 3, "three NAKed bring-ups");
        for(auto const g : gaps) {
            check(pauseOf(g, 5ms), "each bring-up ACKs its first steps: 5 ms every time");
        }
    }

    testcase.next("engine: a decode that keeps asking for a retry is rejected at MaxRetries");
    FakeBus::respond = zeros;
    Dev<Retrying> r{};
    auto const    ticket = r.request<Retrying::Data>();
    check(runUntil(
            r,
            [&] { return r.answer<Retrying::Data>(ticket) != Kvasir::Engine::Answer::pending; },
            500ms),
          "answered");
    check(r.answer<Retrying::Data>(ticket) == Kvasir::Engine::Answer::rejected, "rejected");
    checkEq(r.rejected(), 1U, "counted once");
    checkEq(r.samples(), 0U, "no sample");
    checkEq(readsOf(Retrying::Address, 0x00),
            std::size_t{Kvasir::Engine::EngineDefaults::maxRetries} + 1,
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

    testcase.next("engine: the late answer to a request given up on is not taken for the next one");
    FakeBus::respond = zeros;
    Dev<Probe, ShortNet> lost{};
    check(runUntil(lost, [&] { return lost.valid(); }, 500ms), "up");
    {
        for(int i = 0; i < 200 && FakeBus::pending.empty(); ++i) {
            lost.turn();
            FakeClock::current += 1ms;
        }
        check(!FakeBus::pending.empty(), "a read is on the wire");
        auto const late    = FakeBus::lose();   // the bus never answers it
        auto const errors  = lost.errors();
        auto const samples = lost.samples();
        runFor(lost, 60ms);
        checkEq(lost.errors(), errors + 1, "given up on after InFlightTimeout");
        for(int i = 0; i < 200 && FakeBus::pending.empty(); ++i) {
            lost.turn();
            FakeClock::current += 1ms;
        }
        check(!FakeBus::pending.empty(), "the next read is on the wire");
        late.callback(FakeBus::Result::ok);   // and now the lost one answers
        lost.turn();
        FakeClock::current += 1ms;
        checkEq(lost.samples(), samples, "which is not taken for the read on the wire");
        FakeBus::complete();
        lost.turn();
        checkEq(lost.samples(), samples + 1, "that one is answered when the bus answers it");
        checkEq(lost.errors(), errors + 1, "and the one error stands");
    }

    testcase.next("engine: a full bus queue is not a fault: the request goes next turn");
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

    testcase.next("engine: a bring-up again writes the items the application set, and only those");
    RegisterModel<1> levels{Levels::Address};
    bool             away = false;
    ScopedHook const answeringAgain{
      FakeBus::respond,
      [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
          return away ? FakeBus::Result::nack : levels(a, sent, recv);
      }};
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

    testcase.next("engine: one item set before the bring-up leaves the others their Initial");
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
}

void knobs() {
    using namespace NetTest;
    using G = Probe::Value;

    I2cCase testcase{"Ticket: none() is false and never answered; a real one is true"};
    FakeBus::respond = zeros;
    Dev<Probe>                   d{};
    Kvasir::Engine::Ticket const none{};
    check(!none && none == Kvasir::Engine::Ticket::none(), "a value-initialised ticket is none()");
    check(d.answer<G>(none) == Kvasir::Engine::Answer::pending, "and stays pending");
    auto const t1 = d.request<G>();
    check(static_cast<bool>(t1) && t1 != Kvasir::Engine::Ticket::none(),
          "request() hands out a real one");
    check(runUntil(
            d,
            [&] { return d.answer<G>(t1) == Kvasir::Engine::Answer::ok; },
            500ms),
          "answered");

    testcase.step("Device: the group as a tag, for generic code");
    check(d.valid(G{}) && d.latest(G{}) == d.latest<G>(), "valid() and latest()");
    check(d.seq(G{}) == d.seq<G>(), "seq()");
    std::uint32_t seen = 0;
    check(d.fresh(G{}, seen) && !d.fresh(G{}, seen), "fresh(seen)");
    check(d.fresh(G{}) && !d.fresh(G{}), "fresh()");
    auto const t2 = d.request(G{});
    check(runUntil(
            d,
            [&] { return d.answer<G>(t2) == Kvasir::Engine::Answer::ok; },
            500ms),
          "request()");
    {
        RegisterModel<1> model{Levels::Address};
        FakeBus::respond = std::ref(model);
        Dev<Levels> l{};
        check(l.set(Levels::Level{}, 2, 0x22), "set(W{}, item, value)");
        check(l.set(Levels::Level{}, 0x20), "set(W{}, value) is item 0");
        check(runUntil(l, [&] { return l.valid() && !l.pending(); }, 500ms), "written");
        check(model.word(0x10) == 0x20 && model.word(0x12) == 0x22, "both landed");
    }

    testcase.next("Device: period<G>() is the description's until period<G>(ms) says otherwise");
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
                [&] { return p.answer<G>(t) == Kvasir::Engine::Answer::ok; },
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

    testcase.next("Device: broughtUp(seen) is true once per bring-up, and logHealth() is one line");
    bool             away = false;
    ScopedHook const answering{
      FakeBus::respond,
      [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
          return away ? alwaysNak(a, s, r) : zeros(a, s, r);
      }};
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
        auto const debugs = Log::debugs;
        b.logHealth();
        checkEq(Log::debugs, debugs + 1, "one debug line");
    }
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
              Kvasir::Engine::Step::read({.reg = 0x10, .count = 1, .offset = 0}),
              Kvasir::Engine::Step::readCounted(
                {.reg = 0x20, .countOffset = 0, .countBytes = 1, .maxCount = 4, .offset = 1})};

            struct Sample {
                std::size_t  size{};   ///< what decode was handed
                std::uint8_t last{};
            };

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) {
                return {data.size(), data.u8(data.size() - 1)};
            }
        };

        struct Long {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x30, .count = 2, .offset = 0}),
              Kvasir::Engine::Step::receiveCounted(
                {.countOffset = 0, .countBytes = 2, .maxCount = 200, .offset = 2})};

            struct Sample {
                std::size_t size{};
            };

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) {
                return {data.size()};
            }
        };

        using Reads = Kvasir::Engine::List<Short, Long>;
    };

    static_assert(Kvasir::Engine::bufferBytes(std::span<Kvasir::Engine::Step const>{
                    Counted::Short::Steps})
                    == 5,
                  "a counted read's buffer is reckoned at its most");
    static_assert(detail::stepBits(Counted::Long::Steps[1],
                                   1)
                    == (1 + 200) * 9 + 2,
                  "and so is its bus load");

    // What the script checks turn down.
    constexpr std::array WideCount{
      Kvasir::Engine::Step::read({.reg = 0x10, .count = 3, .offset = 0}),
      Kvasir::Engine::Step::readCounted(
        {.reg = 0x20, .countOffset = 0, .countBytes = 3, .maxCount = 4, .offset = 3})};
    constexpr std::array CountNotRead{
      Kvasir::Engine::Step::read({.reg = 0x10, .count = 1, .offset = 0}),
      Kvasir::Engine::Step::readCounted(
        {.reg = 0x20, .countOffset = 1, .countBytes = 1, .maxCount = 4, .offset = 2})};
    constexpr std::array CountAfter{
      Kvasir::Engine::Step::readCounted(
        {.reg = 0x20, .countOffset = 0, .countBytes = 1, .maxCount = 4, .offset = 1}),
      Kvasir::Engine::Step::read({.reg = 0x10, .count = 1, .offset = 0})};
    constexpr std::array CountFromCounted{
      Kvasir::Engine::Step::read({.reg = 0x10, .count = 1, .offset = 0}),
      Kvasir::Engine::Step::readCounted(
        {.reg = 0x20, .countOffset = 0, .countBytes = 1, .maxCount = 4, .offset = 1}),
      Kvasir::Engine::Step::readCounted(
        {.reg = 0x20, .countOffset = 1, .countBytes = 1, .maxCount = 4, .offset = 5})};
    constexpr std::array PastBuffer{
      Kvasir::Engine::Step::read({.reg = 0x10, .count = 1, .offset = 0}),
      Kvasir::Engine::Step::readCounted(
        {.reg = 0x20, .countOffset = 0, .countBytes = 1, .maxCount = 250, .offset = 10})};
    static_assert(Kvasir::Engine::scriptFault(WideCount,
                                              1)
                  == Kvasir::Engine::ScriptFault::countWidth);
    static_assert(Kvasir::Engine::scriptFault(CountNotRead,
                                              1)
                  == Kvasir::Engine::ScriptFault::countNotReadBefore);
    static_assert(Kvasir::Engine::scriptFault(CountAfter,
                                              1)
                  == Kvasir::Engine::ScriptFault::countNotReadBefore);
    static_assert(Kvasir::Engine::scriptFault(CountFromCounted,
                                              1)
                    == Kvasir::Engine::ScriptFault::countNotReadBefore,
                  "a counted read may not have filled the bytes a later count names");
    static_assert(Kvasir::Engine::scriptFault(PastBuffer,
                                              1)
                  == Kvasir::Engine::ScriptFault::countedPastBuffer);
    static_assert(Kvasir::Engine::wellFormed(Counted::Short::Steps,
                                             1)
                  && Kvasir::Engine::wellFormed(Counted::Long::Steps,
                                                1));

}   // namespace CountedTest

void countedReads() {
    using namespace CountedTest;
    using Short = Counted::Short;
    using Long  = Counted::Long;

    I2cCase const testcase{
      "counted read: a count of 0 sends nothing after the count, decode sees the count alone"};
    std::uint8_t     shortCount = 0;
    std::uint16_t    longCount  = 0;
    ScopedHook const answering{
      FakeBus::respond,
      [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(a != Counted::Address) { return FakeBus::Result::nack; }
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
          return FakeBus::Result::ok;
      }};
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

    testcase.step("counted read: the count in range is what is read, and what decode sees");
    shortCount = 3;
    FakeBus::log.clear();
    {
        auto const n = d.samples<Short>();
        check(runUntil(d, [&] { return d.samples<Short>() > n + 1; }, 1s), "sampled");
    }
    check(!readsOf(0x20).empty() && readsOf(0x20).back() == 3, "three bytes of 0x20");
    checkEq(d.latest<Short>().size, std::size_t{4}, "offset 1 + 3");
    checkEq(d.latest<Short>().last, 0xA2U, "the third byte read last");

    testcase.step("counted read: a count past the step's most is clamped to it");
    shortCount = 9;
    FakeBus::log.clear();
    {
        auto const n = d.samples<Short>();
        check(runUntil(d, [&] { return d.samples<Short>() > n + 1; }, 1s), "sampled");
    }
    check(!readsOf(0x20).empty() && readsOf(0x20).back() == 4, "four, not nine");
    checkEq(d.latest<Short>().size, std::size_t{5}, "offset 1 + 4");

    testcase.step("counted read: a two-byte count is big-endian");
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

    testcase.step("counted read: back to 0, decode sees the count alone again");
    shortCount = 0;
    FakeBus::log.clear();
    {
        auto const n = d.samples<Short>();
        check(runUntil(d, [&] { return d.samples<Short>() > n + 1; }, 1s), "sampled");
    }
    check(readsOf(0x20).empty(), "nothing read after the count");
    checkEq(d.latest<Short>().size, std::size_t{1}, "not what the last run read");
}

// -- wake retries: a part that NAKs until it is awake --------------------------------------

namespace WakeTest {

    struct Sleepy {
        static constexpr std::string_view Name           = "SLEEPY";
        static constexpr Address7         Address        = 0x37;
        static constexpr std::size_t      RegisterBytes  = 1;
        static constexpr std::uint8_t     WakeRetries    = 3;
        static constexpr auto             WakeRetryDelay = std::chrono::milliseconds{4};
        static constexpr std::array       Init{
          Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        using Reads = Kvasir::Engine::List<Value>;
    };

    static_assert(Dev<Sleepy>::WakeRetries == 3 && Dev<PresenceTest::Probe>::WakeRetries == 0);

}   // namespace WakeTest

void wakeRetries() {
    using namespace WakeTest;

    int        naksLeft = 0;
    auto const sleepy = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(naksLeft > 0) {
            --naksLeft;
            return FakeBus::Result::nack;
        }
        return zeros(a, s, r);
    };

    I2cCase testcase{
      "wake retries: N NAKs, then the ACK: no error, N retries, WakeRetryDelay apart"};
    naksLeft         = 3;
    FakeBus::respond = sleepy;
    {
        Dev<Sleepy> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.errors(), 0U, "no error");
        checkEq(d.wakeRetryCount(), 3U, "three retries");
        checkEq(d.consecutiveNaks(), 0U, "no NAK towards parking");
        checkEq(d.bringUps(), 1U, "one bring-up");
        check(FakeBus::log.size() >= 4, "the Init read four times");
        for(std::size_t i = 1; i < 4 && i < FakeBus::log.size(); ++i) {
            check(FakeBus::log[i].at - FakeBus::log[i - 1].at >= 4ms, "each after WakeRetryDelay");
        }
    }

    testcase.next("wake retries: N + 1 NAKs are one NAK");
    naksLeft         = 4;
    FakeBus::respond = sleepy;
    {
        Dev<Sleepy> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up after the bring-up again");
        checkEq(d.errors(), 1U, "one error");
        checkEq(d.wakeRetryCount(), 3U, "after three retries");
        checkEq(d.bringUps(), 1U, "and one bring-up that finished");
    }

    testcase.next("wake retries: a parked part's probe gets them too");
    FakeBus::respond = alwaysNak;
    {
        Dev<Sleepy> d{};
        check(runUntil(d, [&] { return d.absent(); }, 500ms), "parked");
        checkEq(d.errors(), 3U, "after three NAKs");
        checkEq(d.wakeRetryCount(), 9U, "each after its three retries");
        auto const sent  = FakeBus::log.size();
        naksLeft         = 2;
        FakeBus::respond = sleepy;
        check(runUntil(d, [&] { return !d.absent(); }, 2s), "the probe brought it back");
        checkEq(d.wakeRetryCount(), 11U, "through two retries");
        checkEq(FakeBus::log.size() - sent, std::size_t{3}, "the probe: two NAKs and the ACK");
        checkEq(d.errors(), 3U, "and no new error");
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "sampling");
    }

    testcase.next("wake retries: a chip without them counts none");
    FakeBus::respond = alwaysNak;
    {
        Dev<PresenceTest::Probe> d{};
        check(runUntil(d, [&] { return d.absent(); }, 500ms), "parked");
        checkEq(d.wakeRetryCount(), 0U, "none");
        checkEq(d.errors(), 3U, "three NAKs, three errors");
    }
}

namespace MayNakTest {

    /// A part whose soft reset (0x00 = 0x10) is never acknowledged: the probe read, the reset
    /// that may NAK, then a configuration write.
    struct Resets {
        static constexpr std::string_view Name          = "RESETS";
        static constexpr Address7         Address       = 0x24;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{
          Kvasir::Engine::Step::read({.reg = 0x06, .count = 1, .offset = 0}),
          Kvasir::Engine::Step::write({.reg     = 0x00,
                                       .payload = {0x10},
                                       .delay   = std::chrono::milliseconds{5},
                                       .mayNak  = true}),
          Kvasir::Engine::Step::write({.reg = 0x04, .payload = {0x22}})};

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        using Reads = Kvasir::Engine::List<Value>;
    };

    constexpr std::array ProbeMayNak{
      Kvasir::Engine::Step::write({.reg = 0x00, .payload = {0x10}, .mayNak = true}),
      Kvasir::Engine::Step::read({.reg = 0x06, .count = 1})};
    static_assert(Kvasir::Engine::scriptFault(ProbeMayNak,
                                              1,
                                              false)
                    == Kvasir::Engine::ScriptFault::mayNakProbe,
                  "Init's first transaction is the probe and may not be one that NAKs");
    static_assert(Kvasir::Engine::scriptFault(ProbeMayNak,
                                              1,
                                              true)
                    == Kvasir::Engine::ScriptFault::none,
                  "a read group has no probe to protect");

    /// A two-step Init with a 1 ms gap, for the rounding of a delay to the clock's tick.
    struct OneMs {
        static constexpr std::string_view Name          = "ONEMS";
        static constexpr Address7         Address       = 0x25;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{
          Kvasir::Engine::Step::write(
            {.reg = 0x00, .payload = {0x01}, .delay = std::chrono::milliseconds{1}}),
          Kvasir::Engine::Step::write({.reg = 0x01, .payload = {0x02}})};
    };

    /// A WHO_AM_I read, identify, then configuration writes.
    struct Identifies {
        static constexpr std::string_view Name          = "IDENT";
        static constexpr Address7         Address       = 0x26;
        static constexpr std::size_t      RegisterBytes = 1;
        static constexpr std::array       Init{
          Kvasir::Engine::Step::read({.reg = 0x0F, .count = 1, .offset = 0}),
          Kvasir::Engine::Step::identify(),
          Kvasir::Engine::Step::write({.reg = 0x20, .payload = {0x57}})};

        struct State {
            std::uint8_t id{};
        };

        [[nodiscard]] static constexpr bool setup(Kvasir::Bytes data,
                                                  State&        state) {
            state.id = data.u8(0);
            return state.id == 0x33;
        }
    };

    constexpr std::array IdentifyInRead{Kvasir::Engine::Step::read({.reg = 0x06, .count = 1}),
                                        Kvasir::Engine::Step::identify()};
    static_assert(Kvasir::Engine::scriptFault(IdentifyInRead,
                                              1,
                                              true)
                    == Kvasir::Engine::ScriptFault::identifyOutsideInit,
                  "identify belongs to Init, whose setup() it runs");

}   // namespace MayNakTest

void mayNak() {
    using namespace MayNakTest;

    I2cCase testcase{"mayNak: the NAK of a write the part never acknowledges is not an error"};
    FakeBus::respond = [](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(a != 0x24) { return FakeBus::Result::nack; }
        if(s.size() == 2 && s[0] == std::byte{0x00}) { return FakeBus::Result::nack; }
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

    testcase.next("mayNak: a bus fault on that write is no error either");
    FakeBus::respond = [](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(a != 0x24) { return FakeBus::Result::nack; }
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

    testcase.next("mayNak: the same write acknowledged is just as good");
    FakeBus::respond = zeros;
    {
        Dev<Resets> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.errors(), 0U, "no error");
    }

    testcase.next("identify: a part that is not this chip is not written to");
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

    testcase.next("delay: a 1 ms step delay is more than one tick of a millisecond clock");
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
}

}   // namespace

// -- ConfirmChange: a changed reading is taken when the next one agrees ----------------------

namespace {

namespace ConfirmTest {

    /// An input port: one byte, read every 100 ms, a changed value confirmed by the next read.
    struct Port {
        static constexpr std::string_view Name          = "PORT";
        static constexpr Address7         Address       = 0x24;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Pins {
            static constexpr auto         Period        = std::chrono::milliseconds{100};
            static constexpr std::uint8_t ConfirmChange = 3;
            static constexpr std::array   Steps{
              Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        using Reads = Kvasir::Engine::List<Pins>;
    };

    /// One scripted value per read, then the last again; negative: fails on the wire.
    struct Script {
        std::vector<int> values;
        std::size_t      reads{};

        FakeBus::Result operator()(std::uint8_t,
                                   std::span<std::byte const>,
                                   std::span<std::byte> r) {
            if(r.empty()) { return FakeBus::Result::ok; }   // a write: acked
            int const v = values.empty() ? 0 : values[std::min(reads, values.size() - 1)];
            ++reads;
            if(v < 0) { return FakeBus::Result::failed; }
            std::ranges::fill(r, static_cast<std::byte>(v));
            return FakeBus::Result::ok;
        }
    };

    struct Confirmed {
        using Confirm                               = Kvasir::Engine::List<Chips::Tca9555::Pins>;
        static constexpr std::uint8_t confirmChange = 3;
    };

    /// Two read groups: a level register worth confirming and a count the Config leaves alone.
    struct TwoGroups {
        static constexpr std::string_view Name          = "TWO";
        static constexpr Address7         Address       = 0x25;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Levels {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        struct Count {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x01, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) { return data.u8(0); }
        };

        using Reads = Kvasir::Engine::List<Levels, Count>;
    };

    struct LevelsOnly {
        using Confirm                               = Kvasir::Engine::List<TwoGroups::Levels>;
        static constexpr std::uint8_t confirmChange = 2;
    };

    // What a Config may say, as the Device's static_asserts see it.
    using Kvasir::Engine::detail::checkConfirmConfig;
    using Kvasir::Engine::detail::ConfirmConfig;

    struct Unnamed {
        static constexpr std::uint8_t confirmChange = 3;
    };

    struct Uncounted {
        using Confirm = Kvasir::Engine::List<TwoGroups::Levels>;
    };

    struct Foreign {
        using Confirm                               = Kvasir::Engine::List<Chips::Tca9555::Pins>;
        static constexpr std::uint8_t confirmChange = 3;
    };

    struct Stream {
        using Confirm                               = Kvasir::Engine::List<Chips::SamM8q<>::Stream>;
        static constexpr std::uint8_t confirmChange = 3;
    };

    static_assert(checkConfirmConfig<LevelsOnly,
                                     TwoGroups::Reads>()
                  == ConfirmConfig::ok);
    static_assert(checkConfirmConfig<Kvasir::Engine::DefaultConfig,
                                     TwoGroups::Reads>()
                  == ConfirmConfig::ok);
    static_assert(checkConfirmConfig<Unnamed,
                                     TwoGroups::Reads>()
                  == ConfirmConfig::unnamed);
    static_assert(checkConfirmConfig<Uncounted,
                                     TwoGroups::Reads>()
                  == ConfirmConfig::uncounted);
    static_assert(checkConfirmConfig<Foreign,
                                     TwoGroups::Reads>()
                  == ConfirmConfig::foreign);
    static_assert(checkConfirmConfig<Stream,
                                     Chips::SamM8q<>::Reads>()
                    == ConfirmConfig::counted,
                  "a GNSS stream is a counted read: never read twice");
    static_assert(Dev<TwoGroups,
                      LevelsOnly>::ConfirmChange<TwoGroups::Levels>
                  == 2);
    static_assert(Dev<TwoGroups,
                      LevelsOnly>::ConfirmChange<TwoGroups::Count>
                    == 0,
                  "a group the Config does not name is read once");

}   // namespace ConfirmTest

void confirmChange() {
    using namespace ConfirmTest;

    auto const run = [](Script& script, std::vector<int> values) {
        script.values    = std::move(values);
        script.reads     = 0;
        FakeBus::respond = std::ref(script);
    };

    I2cCase testcase{"confirm: the first reading after a bring-up is read twice"};
    {
        Script script;
        run(script, {0x0F});
        Dev<Port> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.latest(), std::uint8_t{0x0F}, "the value");
        checkEq(script.reads, std::size_t{2}, "read, then confirmed");
        checkEq(d.disagreements<Port::Pins>(), 0U, "nothing contradicted");
    }

    testcase.next("confirm: a glitch between two good reads is never taken");
    {
        Script script;
        run(script, {0x0F, 0x0F, 0xFF, 0x0F});
        Dev<Port> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        std::vector<std::uint8_t> seen;
        auto const                until = FakeClock::now() + 1s;
        while(FakeClock::now() < until) {
            turn(d);
            seen.push_back(d.latest());
        }
        check(std::ranges::none_of(seen, [](auto v) { return v == 0xFF; }), "0xFF never served");
        checkEq(d.latest(), std::uint8_t{0x0F}, "still the good value");
        checkEq(d.disagreements<Port::Pins>(), 1U, "the glitch counted");
    }

    testcase.next("confirm: a change that the next read repeats is taken");
    {
        Script script;
        run(script, {0x0F, 0x0F, 0x0E});
        Dev<Port> d{};
        check(runUntil(d, [&] { return d.latest() == 0x0E; }, 1s), "the new value");
        checkEq(script.reads, std::size_t{4}, "two reads for it, one more to confirm");
        checkEq(d.disagreements<Port::Pins>(), 0U, "nothing contradicted");
    }

    testcase.next("confirm: an input moving faster than two reads is taken after ConfirmChange");
    {
        Script script;
        run(script, {1, 2, 3, 4, 5});
        Dev<Port> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.latest(), std::uint8_t{4}, "the fourth read, after three contradicted ones");
        checkEq(d.disagreements<Port::Pins>(), 3U, "three contradicted");
    }

    testcase.next("confirm: a failed transfer drops the reading waiting for its confirmation");
    {
        Script script;
        run(script, {0x0F, 0x0F, 0x0E, -1, 0x0E, 0x0E});
        Dev<Port> d{};
        check(runUntil(d, [&] { return d.latest() == 0x0E; }, 2s), "the new value in the end");
        checkEq(script.reads, std::size_t{6}, "0x0E read twice again after the fault");
    }

    testcase.next("confirm: a Config turns it on for a description that does not say it");
    {
        // TCA9555: the Init read, then the input pair every 100 ms; one glitch to 0xFFFF.
        Script script;
        run(script, {0x34, 0x34, 0x34, 0xFF, 0x34});
        Dev<Chips::Tca9555, Confirmed> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        bool       glitched = false;
        auto const until    = FakeClock::now() + 1s;
        while(FakeClock::now() < until) {
            turn(d);
            glitched = glitched || d.latest().port == 0xFFFF;
        }
        check(!glitched, "the glitch never served");
        checkEq(d.latest().port, std::uint16_t{0x3434}, "the pins");
        checkEq(d.disagreements<Chips::Tca9555::Pins>(), 1U, "one contradicted reading");
    }

    testcase.next("confirm: only the groups the Config names");
    {
        // both registers read 0x10, and once 0xFF each (the third read of each)
        std::array<std::size_t, 2> reads{};
        ScopedHook const           answering{
          FakeBus::respond,
          [&](std::uint8_t, std::span<std::byte const> sent, std::span<std::byte> r) {
              if(r.empty()) { return FakeBus::Result::ok; }
              auto const reg = static_cast<std::size_t>(std::to_integer<unsigned>(sent[0]));
              auto const n   = reads[reg]++;
              std::ranges::fill(r, n == 2 ? std::byte{0xFF} : std::byte{0x10});
              return FakeBus::Result::ok;
          }};
        Dev<TwoGroups, LevelsOnly> d{};
        bool                       levelsGlitched = false;
        bool                       countGlitched  = false;
        auto const                 until          = FakeClock::now() + 1s;
        while(FakeClock::now() < until) {
            turn(d);
            levelsGlitched = levelsGlitched || d.latest<TwoGroups::Levels>() == 0xFF;
            countGlitched  = countGlitched || d.latest<TwoGroups::Count>() == 0xFF;
        }
        check(!levelsGlitched, "the named group never serves the glitch");
        check(countGlitched, "the other one is read once and serves what it read");
        checkEq(d.disagreements<TwoGroups::Levels>(), 1U, "one contradicted reading");
    }

    testcase.next("confirm: after a restart the value is confirmed again");
    {
        Script script;
        run(script, {0x0F});
        Dev<Port> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        d.restart();
        auto const before = script.reads;
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up again");
        checkEq(script.reads - before, std::size_t{2}, "read twice again");
    }
}

// -- withdraw<W>(): a write the application no longer wants --------------------------------------

void withdrawWrites() {
    using Eeprom = Chips::Eeprom24aa025e48<>;
    using Page   = Eeprom::Page;

    /// Acks everything, and answers the EUI-48 read with a real one (not all 0x00 / 0xFF).
    auto const eeprom = [](std::uint8_t, std::span<std::byte const>, std::span<std::byte> r) {
        std::ranges::fill(r, std::byte{0x42});
        return FakeBus::Result::ok;
    };
    auto const pageWrites = [](std::size_t from) {
        std::size_t n = 0;
        for(auto const& w : writes(from)) {
            if(w.size() > 1 && w[0] == 0x10) { ++n; }
        }
        return n;
    };
    Eeprom::PageData const data{
      .address = 0x10,
      .data    = {1, 2},
      .length  = 2
    };

    I2cCase testcase{"withdraw: a page owed while the part is away is never written"};
    FakeBus::respond = eeprom;
    {
        Dev<Eeprom> d{};
        check(runUntil(d, [&] { return d.answering(); }, 500ms), "up");
        FakeBus::respond = alwaysNak;
        d.set<Page>(data);   // NAKed, owed again, NAKed: the part is parked
        check(runUntil(d, [&] { return d.absent(); }, 1s), "gone");
        check(d.pending<Page>(), "owed");
        check(d.withdraw<Page>(), "something was owed");
        check(!d.pending<Page>(), "no longer owed");
        auto const from  = FakeBus::log.size();
        FakeBus::respond = eeprom;
        runFor(d, 40s);   // probed back meanwhile
        check(d.answering(), "back");
        checkEq(pageWrites(from), std::size_t{0}, "the page never went out");
        checkEq(d.writes<Page>(), 0U, "nothing written");
        check(!d.withdraw<Page>(), "and nothing owed to withdraw");
    }

    testcase.next("withdraw: a page on the wire that then fails is not written again");
    FakeBus::respond = eeprom;
    {
        Dev<Eeprom> d{};
        check(runUntil(d, [&] { return d.answering(); }, 500ms), "up");
        d.set<Page>(data);
        d.turn();   // the page goes on the wire
        check(!FakeBus::pending.empty(), "on the wire");
        check(d.withdraw<Page>(), "the one on the wire counts");
        FakeBus::respond = [](std::uint8_t, std::span<std::byte const>, std::span<std::byte>) {
            return FakeBus::Result::failed;
        };
        FakeBus::complete();
        FakeBus::respond = eeprom;
        auto const from  = FakeBus::log.size();
        runFor(d, 1s);
        checkEq(pageWrites(from), std::size_t{0}, "not written again");
        check(!d.pending<Page>(), "not owed");
    }

    testcase.next("withdraw: a later set() of the same group is written as usual");
    FakeBus::respond = eeprom;
    {
        Dev<Eeprom> d{};
        check(runUntil(d, [&] { return d.answering(); }, 500ms), "up");
        d.set<Page>(data);
        static_cast<void>(d.withdraw<Page>());
        d.set<Page>(data);
        check(runUntil(d, [&] { return !d.pending<Page>(); }, 500ms), "written");
        checkEq(d.writes<Page>(), 1U, "once");
    }
}

}   // namespace

// -- resting: the turns that would change nothing are skipped, and nothing is missed ----------

namespace {

namespace RestTest {
    inline bool enabled = true;

    struct Switched {
        static bool enabled() { return RestTest::enabled; }
    };

    /// The shared form: one function for every device, and this one's flag as its argument.
    inline bool flags[2] = {true, true};

    inline bool flagSet(std::uint32_t i) { return flags[i]; }

    struct SharedSwitched {
        static constexpr bool (*enabledBy)(std::uint32_t) = &flagSet;
        static constexpr std::uint32_t enabledArg         = 1;
    };

    /// With a generation: enabled() is asked only after the counter moved. `stepWhileAsked`
    /// plays an interrupt that changes the answer and steps the counter while enabled() runs,
    /// after it has read the old answer.
    inline bool                       genEnabled = true;
    inline std::atomic<std::uint32_t> generation{};
    inline int                        asked{};
    inline bool                       stepWhileAsked{};

    struct GenSwitched {
        static bool enabled() {
            ++asked;
            bool const answer = genEnabled;
            if(stepWhileAsked) {
                stepWhileAsked = false;
                genEnabled     = !genEnabled;
                ++generation;
            }
            return answer;
        }

        static constexpr std::atomic<std::uint32_t> const* enabledGeneration = &generation;
    };
}   // namespace RestTest

void resting() {
    using PresenceTest::Probe;
    using PresenceTest::readsOf;
    using TicketTest::OnDemand;

    I2cCase testcase{"resting: between two periods a device runs no turn, and still reads on time"};
    FakeBus::respond = zeros;
    Dev<Probe> d{};
    check(runUntil(d, [&] { return d.valid(); }, 50ms), "up");
    {
        auto const turns = d.turns();
        auto const reads = readsOf(Probe::Address, 0x01);
        runFor(d, 1000ms);   // a turn a millisecond, a read every 100 ms
        checkEq(readsOf(Probe::Address, 0x01) - reads, std::size_t{10}, "ten reads in a second");
        check(d.turns() - turns <= 30, "a few full turns per read, not a thousand");
    }

    testcase.step("resting: a request from outside the turn is served on the very next turn");
    {
        auto const reads = readsOf(Probe::Address, 0x01);
        runFor(d, 20ms);   // well inside the period
        auto const t = d.request<Probe::Value>();
        turn(d);
        checkEq(readsOf(Probe::Address, 0x01) - reads, std::size_t{1}, "on the wire at once");
        check(runUntil(
                d,
                [&] { return d.answer<Probe::Value>(t) == Kvasir::Engine::Answer::ok; },
                10ms),
              "and answered");
    }

    testcase.step("resting: a shorter period takes effect at once");
    {
        runFor(d, 5ms);
        d.period<Probe::Value>(10ms);
        auto const reads = readsOf(Probe::Address, 0x01);
        runFor(d, 100ms);
        check(readsOf(Probe::Address, 0x01) - reads >= 9, "every 10 ms from now on");
        d.period<Probe::Value>(100ms);
    }

    testcase.step(
      "resting: a clock that jumps past several periods gets one read, then the period");
    {
        runFor(d, 5ms);
        auto const reads = readsOf(Probe::Address, 0x01);
        FakeClock::current += 1s;
        turn(d);
        checkEq(readsOf(Probe::Address, 0x01) - reads, std::size_t{1}, "one read");
        runFor(d, 50ms);
        checkEq(readsOf(Probe::Address, 0x01) - reads, std::size_t{1}, "not a burst");
    }

    testcase.next("resting: set() on a resting device is written on the next turn");
    {
        RegisterModel<1, 1> model{OnDemand::Address};
        model.set(0x00, {0x01, 7});
        FakeBus::respond = std::ref(model);
        Dev<OnDemand> w{};
        runFor(w, 50ms);   // up, nothing cyclic: resting for good
        auto const turns  = w.turns();
        auto const writes = model.writes;
        runFor(w, 50ms);
        checkEq(w.turns(), turns, "no turn at all while nothing is owed");
        check(w.set<OnDemand::Level>(3), "set");
        turn(w);
        checkEq(model.writes - writes, 1, "written on the next turn");
    }

    testcase.next(
      "resting: Config::enabled() going false takes the part offline at once, and back");
    {
        FakeBus::respond  = zeros;
        RestTest::enabled = true;
        Dev<Probe, RestTest::Switched> s{};
        check(runUntil(s, [&] { return s.valid(); }, 50ms), "up");
        runFor(s, 20ms);
        RestTest::enabled = false;
        turn(s);
        check(s.offline(), "offline on the next turn");
        auto const sent = FakeBus::log.size();
        runFor(s, 500ms);
        checkEq(FakeBus::log.size(), sent, "nothing said to it while disabled");
        RestTest::enabled = true;
        check(runUntil(s, [&] { return s.valid(); }, 50ms), "back up once enabled");
    }

    testcase.next("resting: an enable given as a shared function and an argument works the same");
    {
        FakeBus::respond = zeros;
        Dev<Probe, RestTest::SharedSwitched> s{};
        check(runUntil(s, [&] { return s.valid(); }, 50ms), "up");
        RestTest::flags[1] = false;
        turn(s);
        check(s.offline(), "offline on the next turn");
        RestTest::flags[0] = false;
        RestTest::flags[1] = true;
        check(runUntil(s, [&] { return s.valid(); }, 50ms), "its own flag, not another's");
    }

    testcase.next("resting: with an enabled generation, enabled() is asked only after it moved");
    {
        FakeBus::respond     = zeros;
        RestTest::genEnabled = true;
        RestTest::asked      = 0;
        Dev<Probe, RestTest::GenSwitched> s{};
        check(runUntil(s, [&] { return s.valid(); }, 50ms), "up");
        auto const askedUp = RestTest::asked;
        check(askedUp >= 1 && askedUp <= 2, "asked once or twice on the way up");
        runFor(s, 500ms);
        checkEq(RestTest::asked, askedUp, "not asked again while the generation stands");

        RestTest::genEnabled = false;   // without a step the engine keeps the old answer
        runFor(s, 50ms);
        check(!s.offline(), "an unannounced change is not seen: the contract");
        ++RestTest::generation;
        turn(s);
        check(s.offline(), "offline on the next turn after the step");
        auto const sent = FakeBus::log.size();
        runFor(s, 500ms);
        checkEq(FakeBus::log.size(), sent, "nothing said to it while disabled");

        auto const askedOff = RestTest::asked;
        ++RestTest::generation;   // a step whose answer did not change
        runFor(s, 50ms);
        checkEq(RestTest::asked, askedOff + 1, "asked once for the step, then not again");
        check(s.offline(), "still offline");

        RestTest::genEnabled = true;
        ++RestTest::generation;
        check(runUntil(s, [&] { return s.valid(); }, 50ms), "back up after the next step");
    }

    testcase.next("resting: a generation step while enabled() runs is not taken for its answer");
    {
        FakeBus::respond     = zeros;
        RestTest::genEnabled = true;
        Dev<Probe, RestTest::GenSwitched> s{};
        check(runUntil(s, [&] { return s.valid(); }, 50ms), "up");
        runFor(s, 20ms);
        // enabled() answers "enabled", and meanwhile the part is disabled and the step made.
        RestTest::stepWhileAsked = true;
        ++RestTest::generation;
        turn(s);
        check(!RestTest::stepWhileAsked, "asked, and stepped while asked");
        check(runUntil(
                s,
                [&] { return s.offline(); },
                50ms),
              "the step made during the question is asked about again: offline");
        auto const sent = FakeBus::log.size();
        runFor(s, 200ms);
        checkEq(FakeBus::log.size(), sent, "and stays quiet");
        RestTest::genEnabled = true;
        ++RestTest::generation;
        check(runUntil(s, [&] { return s.valid(); }, 50ms), "back");
    }
}

}   // namespace

namespace {

// -- a device on its own: through its table, or as a device set of one ---------------------------

namespace OpsKindsTest {
    /// A bus whose devices call their hooks directly (EngineFeatures::directOps); the untagged
    /// FakeBus drives them through their table, the default.
    struct DirectTag {};

    using DirectBus = Kvasir::Engine::WithFeatures<FakeBusFor<DirectTag>, [] {
        auto f      = Kvasir::Engine::AllEngineFeatures;
        f.directOps = true;
        return f;
    }()>;

    /// Limits read back 100 ms after a write and every second: the verify path on both kinds.
    struct VerifiedLimits {
        static constexpr std::chrono::seconds LimitsVerifyInterval{1};
    };

    using Tmp = Chips::Tmp1075<VerifiedLimits>;

    struct Run {
        std::vector<Transaction> log;
        std::size_t              restartAt{};
        std::uint32_t            samples{};
        std::uint32_t            writes{};
        std::uint32_t            mismatches{};
    };

    /// Bring-up, a sample, two limits and a config change written, a restart, and everything
    /// again after it: what one device does with its part, recorded.
    template<typename Tag,
             typename Bus>
    Run scenario() {
        fresh<Tag>();
        RegisterModel<1, 2> t75{0x48};
        t75.set(0x0F, {0x75, 0x00});   // DIEID
        t75.set(0x00, {0x19, 0x00});
        t75.readOnly = {0x00, 0x0F};
        // the first TLOW write does not stick: the read-back has to catch it, on either path
        bool dropTlow = true;
        t75.onWrite   = [&](std::uint32_t reg, auto const&) {
            if(dropTlow && reg == 0x02) {
                dropTlow = false;
                t75.set(0x02, {0x00, 0x00});
            }
        };
        FakeBusFor<Tag>::respond = std::ref(t75);

        Kvasir::I2C::Device<Bus, FakeClock, Tmp> d{};
        check(runUntil<Tag>(d, [&] { return d.samples() == 1; }, 500ms), "first sample");
        d.template set<Tmp::Limits>(Tmp::Low, Units::centiDegC(7500));
        d.template set<Tmp::Limits>(Tmp::High, Units::centiDegC(8000));
        d.template modify<Tmp::Config>([](auto& c) { c.rate = Tmp::Rate::ms220; });
        check(runUntil<Tag>(
                d,
                [&] { return d.template mismatches<Tmp::Limits>() == 1; },
                2000ms),
              "the read-back caught the dropped TLOW");
        check(runUntil<Tag>(d, [&] { return t75.word(0x02) == 0x4B00U; }, 1000ms), "TLOW written");

        Run r{};
        r.restartAt = FakeBusFor<Tag>::log.size();
        t75.set(0x02, {0x00, 0x00});   // the part lost it
        d.restart();
        runFor<Tag>(d, 600ms);
        checkEq(t75.word(0x02), 0x4B00U, "TLOW written again after the restart");
        r.log        = FakeBusFor<Tag>::log;
        r.samples    = d.samples();
        r.writes     = d.template writes<Tmp::Limits>();
        r.mismatches = d.template mismatches<Tmp::Limits>();
        return r;
    }
}   // namespace OpsKindsTest

void opsKinds() {
    using namespace OpsKindsTest;
    // scenario() runs each side on its own clean bus; the transcripts of both on failure
    Case const testcase{"ops: a device through its table does on the bus what a set of one does",
                        [] {
                            dump();
                            dump<DirectTag>();
                        }};
    auto const table  = scenario<void, FakeBus>();
    auto const direct = scenario<DirectTag, DirectBus>();
    checkEq(table.log.size(), direct.log.size(), "as many transactions");
    bool same = table.log.size() == direct.log.size();
    for(std::size_t i = 0; same && i < table.log.size(); ++i) {
        auto const& a = table.log[i];
        auto const& b = direct.log[i];
        same          = a.kind == b.kind && a.address == b.address && a.sent == b.sent
                     && a.recvLen == b.recvLen && a.received == b.received && a.at == b.at;
        if(!same) { std::printf("    first difference at transaction %zu\n", i); }
    }
    check(same, "the same transactions, bytes and times");
    checkEq(table.restartAt, direct.restartAt, "the restart at the same point");
    checkEq(table.samples, direct.samples, "as many samples");
    checkEq(table.writes, direct.writes, "as many limit writes");
    checkEq(table.mismatches, direct.mismatches, "the same read-back mismatches");
    check(table.writes >= 4, "the limits re-sent after the restart");
}

// -- the faults a model cannot cause, injected on the request on the wire -----------------

namespace InjectedFaults {

    /// One four-byte cyclic read: a partly landed one can be told from a whole one.
    struct Wide {
        static constexpr std::string_view Name          = "WIDE";
        static constexpr Address7         Address       = 0x23;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Value {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{
              Kvasir::Engine::Step::read({.reg = 0x02, .count = 4, .offset = 0})};
            using Sample = std::uint32_t;

            [[nodiscard]] static constexpr Sample decode(Kvasir::Bytes data) {
                return data.be32(0);
            }
        };

        using Reads = Kvasir::Engine::List<Value>;
    };

    /// The part's answer: 0x11223344 to every read, ACK to every write.
    FakeBus::Result answer(std::uint8_t a,
                           std::span<std::byte const>,
                           std::span<std::byte> recv) {
        if(a != Wide::Address && a != PresenceTest::Probe::Address) {
            return FakeBus::Result::nack;
        }
        std::array<std::byte, 4> const v{std::byte{0x11},
                                         std::byte{0x22},
                                         std::byte{0x33},
                                         std::byte{0x44}};
        for(std::size_t i = 0; i < recv.size() && i < v.size(); ++i) { recv[i] = v[i]; }
        return FakeBus::Result::ok;
    }

    /// Turns until the device has a request on the wire; whether it has.
    template<typename D>
    bool onTheWire(D& d) {
        for(int i = 0; i < 500 && FakeBus::pending.empty(); ++i) {
            d.turn();
            FakeClock::current += 1ms;
        }
        return !FakeBus::pending.empty();
    }

}   // namespace InjectedFaults

void injectedFaults() {
    using namespace InjectedFaults;
    using PresenceTest::Probe;
    using PresenceTest::ShortNet;

    I2cCase testcase{"faults: a data NAK is a fault, not a missing part, and sampling goes on"};
    FakeBus::respond = answer;
    {
        Dev<Probe> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        auto const errors  = d.errors();
        auto const samples = d.samples();
        // more in a row than the NAKs that park a part (AbsentAfterNaks, 3 by default)
        for(int i = 0; i < 4; ++i) {
            check(onTheWire(d), "a read is on the wire");
            check(FakeBus::dataNak(), "its data byte is not acknowledged");
            d.turn();
        }
        checkEq(d.errors(), errors + 4, "each one counted as an error");
        checkEq(d.samples(), samples, "no sample from them");
        check(!d.absent(), "and not parked: the address was acknowledged every time");
        check(runUntil(d, [&] { return d.samples() > samples; }, 500ms), "the next read is taken");
    }

    testcase.next(
      "faults: a transfer the bus times out fails once, and the in-flight net does not fail it "
      "again");
    FakeBus::respond = answer;
    {
        Dev<Probe, ShortNet> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        check(onTheWire(d), "a read is on the wire");
        auto const errors  = d.errors();
        auto const samples = d.samples();
        auto const before  = FakeClock::now();
        check(FakeBus::timeout(), "the bus's transfer timeout");
        checkEq(FakeClock::now() - before,
                FakeClock::duration{FakeBus::TransferTimeout},
                "after 25 ms");
        d.turn();
        checkEq(d.errors(), errors + 1, "one error");
        // past the device's own 50 ms net, measured from when the read went out
        check(runUntil(d, [&] { return d.samples() > samples; }, 500ms), "read again");
        runFor(d, 200ms);
        checkEq(d.errors(),
                errors + 1,
                "and still the one error: the net had nothing left to give up on");
    }

    testcase.next(
      "faults: a read that failed after two of its four bytes is not taken for an answer");
    FakeBus::respond = answer;
    {
        Dev<Wide> d{};
        check(runUntil(d, [&] { return d.valid(); }, 500ms), "up");
        checkEq(d.latest(), std::uint32_t{0x11223344}, "the whole answer");
        check(onTheWire(d), "a read is on the wire");
        auto const errors  = d.errors();
        auto const samples = d.samples();
        check(FakeBus::partialRead(2), "two bytes land, then the transfer fails");
        d.turn();
        checkEq(d.samples(), samples, "no sample from the half-filled buffer");
        checkEq(d.latest(), std::uint32_t{0x11223344}, "the last whole answer stands");
        checkEq(d.errors(), errors + 1, "counted as an error");
        check(runUntil(
                d,
                [&] { return d.samples() > samples; },
                500ms),
              "the next whole read is taken");
    }
}

}   // namespace

int main() {
    presence();
    features();
    statsOffVerify();
    linkState();
    ticketsAndWrites();
    engine();
    nets();
    knobs();
    countedReads();
    wakeRetries();
    mayNak();
    confirmChange();
    withdrawWrites();
    resting();
    opsKinds();
    injectedFaults();
    return finish();
}
