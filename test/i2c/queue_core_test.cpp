/// The queued I2C master's chip-independent half (I2C/Controller/QueueCore.hpp) over a fake hardware policy: what the chip
/// drivers (chip_rp_common I2CQueued.hpp, chip_atsam_common Sercom_I2CQueued.hpp) leave to it - one callback per
/// request, the depth-counted mask, cancel, deadlines, the timeout, the dead-bus watchdog, the post-failure settle
/// and the callback order. And the parts of a master the chips share above the core (I2C/Controller/QueuedFront.hpp): the front's
/// members reach the core, the shape's timing() and request type, the timing cache.
#include "FakeQueueHw.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/I2C/Controller/QueueCore.hpp>
#include <kvasir/Devices/I2C/Controller/QueuedFront.hpp>
#include <kvasir/Devices/I2C/Controller/Request.hpp>
#include <ratio>
#include <span>
#include <support/FakeClock.hpp>
#include <type_traits>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::I2C::QueueState;
using namespace Kvasir::Test::I2cQueue;

namespace {

static_assert(Kvasir::I2C::QueueHw<FakeHw<true>,
                                   FakeClock>);
static_assert(Kvasir::I2C::QueueHw<FakeHw<false>,
                                   FakeClock>);

struct NoStartHw : FakeHw<true> {
    static void start(Request const&) = delete;
};

static_assert(!Kvasir::I2C::QueueHw<NoStartHw,
                                    FakeClock>);

struct NoLogTimeoutHw : FakeHw<true> {
    static void logTimeout(FakeClock::time_point) = delete;
};

static_assert(!Kvasir::I2C::QueueHw<NoLogTimeoutHw,
                                    FakeClock>,
              "logTimeout is part of the contract");

using Hw      = FakeHw<true>;
using Core    = CoreOf<Hw>;
using Request = Core::Request;
using Result  = Core::Result;

using PlainHw   = FakeHw<false>;
using PlainCore = CoreOf<PlainHw>;

std::vector<std::pair<int, Result>> results{};

std::array<std::byte, 4> txBuf{};
std::array<std::byte, 4> rxBuf{};

Request read(int tag) {
    Request r{};
    r.address     = static_cast<std::uint8_t>(0x40 + tag);
    r.receiveData = std::span{rxBuf}.first(2);
    r.callback    = [tag](Result x) { results.emplace_back(tag, x); };
    return r;
}

int count(Result r) {
    int n = 0;
    for(auto const& e : results) { n += e.second == r ? 1 : 0; }
    return n;
}

void fresh() {
    Core::reset();
    PlainCore::reset();
    Hw::clear();
    PlainHw::clear();
    FakeRecovery::reset();
    FakeClock::reset();
    results.clear();
    Log::reset();
    static_cast<void>(Core::takeLatency());
    // the dead-bus streak: one success clears it
    if(Core::consecutiveFailures() != 0) {
        Core::submit(read(99));
        Hw::raise(Wire::ok);
        results.clear();
    }
    Hw::clear();
}

void oneCallbackEach() {
    testCase("every request one callback, in submit order; the line unmasked afterwards");
    fresh();
    check(Core::submit(read(1)), "first accepted");
    check(Core::submit(read(2)), "second accepted");
    checkEq(Hw::startCount(), 1, "only the first is on the wire");
    Hw::raise(Wire::ok);
    checkEq(Hw::startCount(), 2, "the next one started from the completion");
    Hw::raise(Wire::nack);
    checkEq(results.size(), 2U, "two callbacks");
    check(results[0] == std::pair{1, Result::ok} && results[1] == std::pair{2, Result::nack},
          "in order, each with its own result");
    checkEq(Core::transfers(), 2U, "two transfers counted");
    checkEq(Core::mask_.depth(), std::uint8_t{0}, "no masked section left open");
    check(!Hw::lineMasked, "the line is unmasked");
    check(!Hw::startedUnmasked, "every start ran with the line masked (or in the interrupt)");
    check(!Hw::nested(), "never unmasked inside a nested section");
}

// QueueDepth (4 here) is what the queue holds: a request on the wire is not in it.
static_assert(Core::QueueDepth == 4);
static_assert(decltype(Core::queue_)::capacity() == Core::QueueDepth);

void queueFull() {
    testCase("a full queue refuses, no callback");
    fresh();
    Core::submit(read(0));   // on the wire
    for(int i = 1; i <= 4; ++i) { check(Core::submit(read(i)), "fits"); }
    check(!Core::submit(read(5)), "the fifth queued request is refused");
    auto const drainedBefore = Core::drainedRequests();
    Core::reset();
    checkEq(results.size(), 5U, "the one on the wire and the four queued failed by reset()");
    checkEq(count(Result::failed), 5, "all as failed");
    checkEq(Core::drainedRequests() - drainedBefore, 4U, "four drained");
}

void queueHoldsQueueDepth() {
    testCase("the queue holds exactly QueueDepth requests, and is empty after they ran");
    fresh();
    FakeRecovery::sdaHigh = false;   // nothing starts: every accepted request waits in the queue
    for(int i = 1; i <= static_cast<int>(Core::QueueDepth); ++i) {
        check(Core::submit(read(i)), "fits");
    }
    checkEq(Hw::startCount(), 0, "none on the wire");
    check(!Core::submit(read(9)), "request QueueDepth + 1 is refused");
    checkEq(results.size(), 0U, "a refused request gets no callback");
    FakeRecovery::sdaHigh = true;
    Core::turn();
    for(std::size_t i = 0; i != Core::QueueDepth; ++i) { Hw::raise(Wire::ok); }
    checkEq(Hw::startCount(), static_cast<int>(Core::QueueDepth), "all QueueDepth started");
    checkEq(count(Result::ok), static_cast<int>(Core::QueueDepth), "and completed, in order");
    for(int i = 1; i <= static_cast<int>(Core::QueueDepth); ++i) {
        check(Core::submit(read(i)), "the emptied queue takes QueueDepth again");
    }
    Core::reset();
}

void nestedSections() {
    testCase(
      "a callback run inside a masked section may submit; only the outermost unmask reaches the "
      "line");
    fresh();
    Core::submit(read(1));
    Core::submit(read(2));
    // reset() fails both; a callback's submit is queued only, the block is coming back
    results.clear();
    int     resubmits = 0;
    Request a         = read(3);
    a.callback        = [&](Result x) {
        results.emplace_back(3, x);
        check(Hw::lineMasked, "the line stays masked in reset()'s callbacks");
        if(resubmits++ == 0) {
            check(Core::submit(read(4)), "a submit from reset()'s callback is queued");
        }
    };
    Core::reset();   // fails 1 and 2
    Core::submit(a);
    auto const startsBefore = Hw::startCount();
    results.clear();
    Core::reset();   // fails 3 (on the wire); its callback queues 4, which waits for the block
    check(results.size() == 1 && results[0] == std::pair{3, Result::failed},
          "only what was there when reset() was called failed");
    checkEq(Hw::startCount(),
            startsBefore + 1,
            "the callback's request started once the block was back");
    check(!Hw::lineMasked && !Hw::nested(), "unmasked once, at the end");
    Hw::raise(Wire::ok);
    check(results.back() == std::pair{4, Result::ok}, "and ran");

    // a callback run by the deadline sweep (masked) submits
    fresh();
    Core::submit(read(1));
    auto q     = read(2);
    q.deadline = FakeClock::now() + 1ms;
    q.callback = [](Result x) {
        results.emplace_back(2, x);
        check(Hw::lineMasked, "the sweep's callback runs masked");
        Core::submit(read(5));
    };
    Core::submit(q);
    FakeClock::advance(2ms);
    Core::turn();
    check(results.size() == 1 && results[0] == std::pair{2, Result::timedOut},
          "the queued one timed out");
    check(!Hw::lineMasked && !Hw::nested(), "the submit's unmask did not open the sweep's section");
    Hw::raise(Wire::ok);
    Hw::raise(Wire::ok);
    check(results.back() == std::pair{5, Result::ok}, "the submitted one ran after the first");
}

void cancel() {
    testCase("cancel: queued (removed), on the wire (stopping, then cancelled), too late, invalid");
    fresh();
    auto const wire   = Core::submitTracked(read(1));
    auto const queued = Core::submitTracked(read(2));
    check(wire.valid() && queued.valid(), "tickets");
    check(Core::cancel(queued) == Kvasir::Bus::Cancel::removed, "queued: removed");
    check(results.size() == 1 && results[0] == std::pair{2, Result::cancelled},
          "its callback ran before cancel() returned");
    check(Core::cancel(wire) == Kvasir::Bus::Cancel::stopping, "on the wire: stopping");
    check(Core::cancel(wire) == Kvasir::Bus::Cancel::stopping, "asked twice: still stopping");
    checkEq(Hw::stops, 1, "the block was asked to stop once");
    Hw::raise(Wire::stopped);
    check(results.back() == std::pair{1, Result::cancelled}, "the stop completes it as cancelled");
    check(Core::cancel(wire) == Kvasir::Bus::Cancel::tooLate, "completed: tooLate");
    check(Core::cancel(Kvasir::Bus::Ticket{}) == Kvasir::Bus::Cancel::tooLate,
          "invalid ticket: tooLate");
    checkEq(Hw::startCount(), 1, "the tombstone never went out");
    Core::turn();
    checkEq(Hw::startCount(), 1, "and is dropped, not started, later");
}

void deadlines() {
    testCase(
      "deadlines: queued expiry, on-wire expiry (stopped as timedOut), dropped at the start");
    fresh();
    auto a     = read(1);
    a.deadline = FakeClock::now() + 5ms;
    auto b     = read(2);
    b.deadline = FakeClock::now() + 1ms;
    Core::submit(a);
    Core::submit(b);
    FakeClock::advance(2ms);
    Core::turn();
    check(results.size() == 1 && results[0] == std::pair{2, Result::timedOut},
          "the queued one expired");
    checkEq(Hw::stops, 0, "the one on the wire is still in time");
    FakeClock::advance(4ms);
    Core::turn();
    checkEq(Hw::stops, 1, "past its deadline on the wire: stopped");
    Hw::raise(Wire::stopped);
    check(results.back() == std::pair{1, Result::timedOut}, "and completed as timedOut");

    fresh();
    Core::submit(read(1));
    auto c     = read(2);
    c.deadline = FakeClock::now() + 1ms;
    Core::submit(c);
    FakeClock::advance(2ms);
    Hw::raise(Wire::ok);   // completes 1, startNext finds 2 out of time
    check(results.size() == 2 && results[1] == std::pair{2, Result::timedOut},
          "dropped at its start");
    checkEq(Hw::startCount(), 1, "never on the wire");
}

void timeout() {
    testCase(
      "timeout: failed, the block aborted, the snapshot taken; a pending stop keeps its outcome");
    fresh();
    Core::submit(read(1));
    Core::submit(read(2));
    FakeClock::advance(9ms);
    Core::turn();
    checkEq(results.size(), 0U, "within the transfer time nothing happens");
    FakeClock::advance(5ms);
    Core::turn();
    check(results.size() == 1 && results[0] == std::pair{1, Result::failed},
          "the request on the wire failed");
    checkEq(Hw::timeoutAborts, 1, "the block let go");
    checkEq(Core::timeouts(), 1U, "counted");
    checkEq(Core::lastTimeout().address, 0x41, "the snapshot is of that request");
    checkEq(Core::lastTimeout().hw.marker, 42U, "with the block's registers as the chip read them");
    checkEq(Core::lastTimeout().state, std::uint8_t{2}, "the core's half: receiving");
    checkEq(Core::lastTimeout().received, std::uint16_t{0}, "nothing of it received");
    checkEq(Core::lastTimeout().toReceive, std::uint16_t{2}, "of the two bytes asked");
    checkEq(Core::lastTimeout().usAge, 14'000U, "timed out 14 ms after its start");
    check(Core::lastTimeout().sdaHigh && Core::lastTimeout().sclHigh,
          "the lines, from the recovery");
    checkEq(Hw::startCount(), 2, "the next one started");

    fresh();
    auto const t = Core::submitTracked(read(1));
    Core::cancel(t);
    FakeClock::advance(20ms);
    Core::turn();
    check(results.size() == 1 && results[0] == std::pair{1, Result::cancelled},
          "a stop that never completed still ends as the stop's outcome");
}

void deadBus() {
    testCase("dead bus: 40 faults re-initialise, 40 more run a recovery; NAKs never count");
    fresh();
    for(int i = 0; i < 60; ++i) {
        Core::submit(read(1));
        Hw::raise(Wire::nack);
    }
    checkEq(Core::consecutiveFailures(), 0U, "NAKs are answers");
    auto const fail40 = [] {
        for(std::uint32_t i = 0; i < Core::DeadBusFailures; ++i) {
            Core::submit(read(1));
            Hw::raise(Wire::fault);
        }
    };
    fail40();
    checkEq(Core::consecutiveFailures(), 40U, "forty faults in a row");
    Core::turn();
    checkEq(Hw::reinits, 1, "the cheap fix first: the block re-initialised");
    checkEq(Core::resuscitations(), 1U, "once");
    fail40();
    Core::turn();
    checkEq(FakeRecovery::begins, 1, "then the expensive one: a bus recovery");
    checkEq(Core::resuscitations(), 2U, "twice");
}

void settleAfterFailure() {
    testCase("after a failure: the master still busy or SDA low defers the next start");
    fresh();
    Core::submit(read(1));
    Core::submit(read(2));
    Hw::busy = true;
    Hw::raise(Wire::fault);
    checkEq(Hw::startCount(), 1, "busy: the next one waits");
    checkEq(FakeRecovery::defers, 1, "for a settle");
    Hw::busy = false;
    Core::turn();
    checkEq(Hw::startCount(), 1, "not before the settle is over");
    FakeClock::advance(2ms);
    Core::turn();
    checkEq(Hw::startCount(), 2, "then it starts");
    FakeRecovery::sdaHigh = false;
    Core::submit(read(3));
    Hw::raise(Wire::nack);
    checkEq(Core::sdaLowAfterAbort(), 1U, "SDA low after the failure counted");
    FakeClock::advance(2ms);
    Core::turn();
    checkEq(Hw::startCount(), 2, "nothing starts while SDA is low");
    FakeRecovery::sdaHigh = true;
    Core::turn();
    checkEq(Hw::startCount(), 3, "SDA back: it starts");
}

void stuckBusWaitsForTheRecovery() {
    testCase(
      "a stuck SDA on an idle bus: the recovery fails what was queued when it began; what is "
      "queued while it "
      "runs waits and starts after its re-init");
    fresh();
    FakeRecovery::sdaHigh = false;
    auto const drained    = Core::drainedRequests();
    Core::submit(read(1));
    Core::turn();
    checkEq(results.size(), 0U, "SDA low: queued, not started");
    checkEq(Hw::startCount(), 0, "not started");
    FakeRecovery::stuck = true;   // the idle look begins a recovery
    Core::turn();
    check(results.size() == 1 && results[0] == std::pair{1, Result::failed},
          "what was queued when the recovery began failed");
    checkEq(Core::drainedRequests() - drained, 1U, "drained, never on the wire");
    FakeRecovery::stuck  = false;
    FakeRecovery::active = true;   // the recovery runs
    Core::submit(read(2));
    checkEq(Hw::startCount(), 0, "queued while the recovery runs: waits");
    FakeRecovery::active   = false;
    FakeRecovery::nextTick = FakeRecovery::TickResult::needsReinit;
    FakeRecovery::sdaHigh  = true;   // the recovery freed the line
    Core::turn();
    checkEq(Hw::reinits, 1, "the recovery's re-init");
    checkEq(results.size(), 1U, "failed nothing that waited for it");
    checkEq(Hw::startCount(), 1, "it started once the block was back");
    Hw::raise(Wire::ok);
    check(results.size() == 2 && results[1] == std::pair{2, Result::ok}, "and ran");
}

void callbackSubmitStartsAfterIt() {
    testCase("what a callback submits starts after the callback has returned");
    fresh();
    auto a     = read(1);
    a.callback = [](Result x) {
        results.emplace_back(1, x);
        Hw::inCallback = true;
        check(Core::submit(read(2)), "accepted from the callback");
        checkEq(Core::state_, QueueState::idle, "the request is over while its callback runs");
        Hw::inCallback = false;
    };
    Core::submit(a);
    Hw::raise(Wire::ok);
    check(!Hw::startedInCallback, "not started inside the callback");
    checkEq(Hw::startCount(), 2, "started once the callback returned");

    fresh();
    auto b     = read(1);
    b.callback = [](Result x) {
        results.emplace_back(1, x);
        Core::submit(read(2));
    };
    Core::submit(b);
    Hw::busy = true;
    Hw::raise(Wire::fault);
    checkEq(Hw::startCount(), 1, "after a failure the resubmit waits for the settle checks");
    Hw::busy = false;
    FakeClock::advance(2ms);
    Core::turn();
    checkEq(Hw::startCount(), 2, "then it starts");
}

void emptyRequest() {
    testCase("a request with nothing to send or receive completes failed, never on the wire");
    fresh();
    auto e        = read(1);
    e.receiveData = {};
    Core::submit(e);
    check(results.size() == 1 && results[0] == std::pair{1, Result::failed}, "failed at once");
    checkEq(Hw::startCount(), 0, "the block never saw it");
    check(!Core::submit(read(2)) || Hw::startCount() == 1, "the bus goes on");
}

void plainBus() {
    testCase("a bus without cancel and deadlines: the plain request and result types");
    static_assert(std::is_same_v<PlainCore::Request, Kvasir::I2C::Queue::I2CRequest<16>>);
    static_assert(std::is_same_v<PlainCore::Result, Kvasir::Bus::Outcome>);
    fresh();
    std::vector<Kvasir::Bus::Outcome> seen{};
    for(int i = 0; i < 3; ++i) {
        Kvasir::I2C::Queue::I2CRequest<16> r{};
        r.address  = 0x10;
        r.sendData = std::span<std::byte const>{txBuf};
        r.callback = [&seen](Kvasir::Bus::Outcome x) { seen.push_back(x); };
        PlainCore::submit(r);
    }
    PlainHw::raise(Wire::ok);
    PlainHw::raise(Wire::ok);
    PlainHw::raise(Wire::ok);
    checkEq(seen.size(), 3U, "three callbacks");
    checkEq(PlainHw::startCount(), 3, "each on the wire");
    check(!PlainCore::active_, "idle afterwards");
    PlainHw::raise(Wire::ok);
    checkEq(PlainCore::spuriousIsr(), 1U, "an entry with nothing running is spurious");
    check(!PlainHw::nested(), "never unmasked inside a nested section");
}

// -- above the core: the front, the shape, the timing cache ------------------------------------------------------

using Front      = Kvasir::I2C::QueuedFront<Core>;
using PlainFront = Kvasir::I2C::QueuedFront<PlainCore>;

static_assert(std::is_empty_v<Front>,
              "the front holds no state");
static_assert(std::is_same_v<Front::Result,
                             Core::Result>
              && std::is_same_v<Front::Latency,
                                Core::Latency>);
static_assert(std::is_same_v<Front::State,
                             QueueState>);
static_assert(Front::QueueDepth == Core::QueueDepth && Front::CallbackSize == Core::CallbackSize);
static_assert(std::is_same_v<Front::Extends,
                             Kvasir::Startup::Extend<Kvasir::Hook::MainLoop,
                                                     &Front::turn>>,
              "the main-loop hook is the front's turn()");

template<typename F, typename R>
concept HasTickets = requires(R const& r, Kvasir::Bus::Ticket t) {
    F::submitTracked(r);
    F::cancel(t);
};

static_assert(HasTickets<Front,
                         Request>,
              "a tracked core: tickets and cancel()");
static_assert(!HasTickets<PlainFront,
                          PlainCore::Request>,
              "a plain core: neither");

/// A bus on which every device has its own clock: its Hw's TimingCache counts the switches.
struct TimedHw : FakeHw<false> {
    static constexpr bool       perDeviceClock = true;
    static inline std::uint32_t switches{};

    static std::uint32_t clockSwitches() { return switches; }
};

static_assert(Kvasir::I2C::QueueHw<TimedHw,
                                   FakeClock>);
using TimedFront = Kvasir::I2C::QueuedFront<CoreOf<TimedHw>>;

void frontReachesTheCore() {
    testCase("every member of the front reaches the core");
    fresh();
    check(Front::submit(read(1)), "submit");
    checkEq(Hw::startCount(), 1, "on the wire");
    Hw::raise(Wire::ok);
    check(results.size() == 1 && results[0] == std::pair{1, Result::ok},
          "its callback, through the core");
    checkEq(Front::transfers(), Core::transfers(), "transfers()");

    auto const queuedBehind = [] {
        Front::submit(read(2));
        return Front::submitTracked(read(3));
    }();
    check(queuedBehind.valid(), "submitTracked: a ticket");
    check(Front::cancel(queuedBehind) == Kvasir::Bus::Cancel::removed,
          "cancel(): removed from the queue");

    auto const timeouts = Front::timeouts();
    FakeClock::advance(20ms);
    Front::turn();
    checkEq(Front::timeouts(), timeouts + 1, "turn(): the timeout, timeouts()");
    checkEq(Front::lastTimeout().address, 0x42, "lastTimeout()");
    Front::logLastTimeout();
    checkEq(Hw::lastTimeoutLogs, 1, "logLastTimeout(): the chip's line");

    Hw::exhausted = 5;
    checkEq(Front::disableWaitsExhausted(),
            5U,
            "disableWaitsExhausted(): the chip's waitsExhausted()");
    PlainHw::exhausted = 2;
    checkEq(PlainFront::disableWaitsExhausted(),
            2U,
            "disableWaitsExhausted(): each bus its own chip's count");
    checkEq(PlainFront::clockSwitches(), 0U, "clockSwitches(): 0 without perDeviceClock");
    TimedHw::switches = 3;
    checkEq(TimedFront::clockSwitches(),
            3U,
            "clockSwitches(): the chip's count with perDeviceClock");

    auto const spurious = Front::spuriousIsr();
    Front::onIsr();
    checkEq(Front::spuriousIsr(), spurious + 1, "onIsr() with nothing running: spuriousIsr()");

    Front::requestRecovery();
    check(Front::isRecovering(), "requestRecovery(), isRecovering()");
    checkEq(FakeRecovery::begins, 1, "the recovery began");
    FakeRecovery::active = false;

    auto const reinits = Hw::reinits;
    Front::reset();
    checkEq(Hw::reinits, reinits + 1, "reset(): the block re-initialised");
    checkEq(Front::consecutiveFailures(), Core::consecutiveFailures(), "consecutiveFailures()");
    checkEq(Front::resuscitations(), Core::resuscitations(), "resuscitations()");
    checkEq(Front::sdaLowAfterAbort(), Core::sdaLowAfterAbort(), "sdaLowAfterAbort()");
    checkEq(Front::drainedRequests(), Core::drainedRequests(), "drainedRequests()");
    [[maybe_unused]] Front::Latency const l = Front::takeLatency();
}

/// A chip's per-device timing, as QueuedShape takes it: one number, and a refusal above 1 MHz.
struct FakeTiming {
    struct ClockTiming {
        std::uint32_t hz{};

        constexpr bool operator==(ClockTiming const&) const = default;
    };

    template<std::intmax_t N,
             std::intmax_t D>
    static consteval ClockTiming clockTiming(std::uint32_t,
                                             std::uint32_t hz,
                                             std::ratio<N,
                                                        D>) {
        return ClockTiming{hz};
    }
};

template<bool PerDevice>
struct ShapeCfg {
    static constexpr std::uint32_t              baudRate   = 400'000;
    static constexpr std::uint32_t              clockSpeed = 48'000'000;
    static constexpr Kvasir::Bus::QueueFeatures queueFeatures{.cancel = true};
    static constexpr bool                       perDeviceClock   = PerDevice;
    static constexpr auto                       maxBaudRateError = std::ratio<1, 100>{};
    static constexpr std::uint32_t              minBaudRate      = 100'000;
};

using TimedShape = Kvasir::I2C::QueuedShape<ShapeCfg<true>, FakeClock, 16, FakeTiming>;
using OneShape   = Kvasir::I2C::QueuedShape<ShapeCfg<false>, FakeClock, 16, FakeTiming>;

static_assert(TimedShape::Cancellable && !TimedShape::Deadlines && TimedShape::Tracked);
static_assert(TimedShape::DefaultTiming == FakeTiming::ClockTiming{400'000},
              "a request's default: baudRate's");
static_assert(TimedShape::timing(100'000) == FakeTiming::ClockTiming{100'000},
              "a slower device: its own clock");
static_assert(TimedShape::timing(1'000'000) == FakeTiming::ClockTiming{400'000},
              "never above baudRate");
static_assert(std::is_same_v<TimedShape::Request,
                             Kvasir::I2C::Queue::QueuedRequestFor<16,
                                                                  FakeClock::time_point,
                                                                  true,
                                                                  false,
                                                                  true,
                                                                  FakeTiming::ClockTiming,
                                                                  TimedShape::DefaultTiming>>);
static_assert(std::is_same_v<decltype(OneShape::timing(400'000)),
                             std::false_type>,
              "one clock for every device: no timing in a request");
static_assert(std::is_same_v<OneShape::Request,
                             Kvasir::I2C::Queue::I2CTrackedRequest<16,
                                                                   FakeClock::time_point,
                                                                   false>>);

void timingCache() {
    testCase(
      "the timing cache: a write only on a change, forgotten on a reset, each write counted");
    Kvasir::I2C::TimingCache<FakeTiming::ClockTiming> c{};
    check(c.needs({100'000}), "nothing known: write");
    c.took({100'000});
    check(!c.needs({100'000}), "the same timing again: no write");
    checkEq(c.switches(), 1U, "one switch");
    check(c.needs({400'000}), "another device's clock: write");
    c.took({400'000});
    checkEq(c.switches(), 2U, "two switches");
    c.invalidate();
    check(c.needs({400'000}), "after a reset the block is at baudRate's: write again");
    checkEq(c.switches(), 2U, "a reset is no switch");
}

}   // namespace

/// Every optional part off (queueFeatures): what is left must still move requests, time a lost one out and watch for
/// a dead bus; the accessors of what is off do not compile (here: are absent to a requires-expression; the
/// i2c_queue_gate cases below make the error itself the test).
namespace {
struct LeanTag {};

constexpr Kvasir::Bus::QueueFeatures LeanFeatures{.latency         = false,
                                                  .counters        = false,
                                                  .timeoutSnapshot = false};

using LeanHw    = FakeHw<false, LeanTag, LeanFeatures>;
using LeanCore  = CoreOf<LeanHw>;
using LeanFront = Kvasir::I2C::QueuedFront<LeanCore>;

template<typename C>
concept HasCounters = requires {
    C::timeouts();
    C::transfers();
    C::sdaLowAfterAbort();
    C::drainedRequests();
    C::resuscitations();
};
template<typename C>
concept HasLatency = requires { C::takeLatency(); };
template<typename C>
concept HasSnapshot = requires {
    C::lastTimeout();
    C::logLastTimeout();
};

static_assert(HasCounters<Core> && HasLatency<Core> && HasSnapshot<Core>,
              "all on by default");
static_assert(!HasCounters<LeanCore> && !HasLatency<LeanCore> && !HasSnapshot<LeanCore>,
              "and gone when off");
static_assert(!HasCounters<LeanFront> && !HasLatency<LeanFront> && !HasSnapshot<LeanFront>,
              "from the front too");

void leanBus() {
    testCase(
      "without latency, counters and snapshot: requests run, a lost one times out, a dead bus is "
      "seen");
    LeanCore::reset();
    LeanHw::clear();
    FakeRecoveryOf<LeanTag>::reset();
    FakeClock::reset();
    std::vector<Kvasir::Bus::Outcome> seen{};
    auto const                        lean = [&seen] {
        Kvasir::I2C::Queue::I2CRequest<16> r{};
        r.address     = 0x20;
        r.receiveData = std::span{rxBuf}.first(2);
        r.callback    = [&seen](Kvasir::Bus::Outcome x) { seen.push_back(x); };
        return r;
    };
    LeanCore::submit(lean());
    LeanCore::submit(lean());
    LeanHw::raise(Wire::ok);
    checkEq(seen.size(), 1U, "the first completed");
    checkEq(LeanHw::startCount(), 2, "the second started from the completion");
    FakeClock::advance(20ms);
    LeanCore::turn();
    check(seen.size() == 2 && seen[1] == Kvasir::Bus::Outcome::failed,
          "the lost one failed at its timeout");
    checkEq(LeanHw::timeoutAborts, 1, "the block let go");
    checkEq(LeanCore::consecutiveFailures(), 1U, "the dead-bus streak still counts");
    for(std::uint32_t i = 1; i < LeanCore::DeadBusFailures; ++i) {
        LeanCore::submit(lean());
        LeanHw::raise(Wire::fault);
    }
    LeanCore::turn();
    checkEq(LeanHw::reinits, 1, "forty faults: the block re-initialised");
    checkEq(LeanCore::consecutiveFailures(), 0U, "and the streak starts over");
    checkEq(LeanCore::mask_.depth(), std::uint8_t{0}, "no masked section left open");
}

#ifdef I2C_QUEUE_GATE_COUNTERS
[[maybe_unused]] auto const gateCounters = LeanFront::timeouts();
#endif
#ifdef I2C_QUEUE_GATE_LATENCY
[[maybe_unused]] auto const gateLatency = LeanFront::takeLatency();
#endif
#ifdef I2C_QUEUE_GATE_SNAPSHOT
[[maybe_unused]] auto const& gateSnapshot = LeanFront::lastTimeout();
#endif
}   // namespace

int main() {
    oneCallbackEach();
    queueFull();
    queueHoldsQueueDepth();
    nestedSections();
    cancel();
    deadlines();
    timeout();
    deadBus();
    settleAfterFailure();
    stuckBusWaitsForTheRecovery();
    callbackSubmitStartsAfterIt();
    emptyRequest();
    plainBus();
    frontReachesTheCore();
    timingCache();
    leanBus();
    return finish();
}
