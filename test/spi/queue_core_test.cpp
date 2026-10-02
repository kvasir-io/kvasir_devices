/// The queued SPI master's chip-independent half (SPI/QueueCore.hpp) over a fake hardware policy.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/SPI/QueueCore.hpp>
#include <span>
#include <string>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::SPI::Lines;
using Kvasir::SPI::Transfer;
using Kvasir::SPI::TransferResult;

namespace {

/// Everything the core asked of the hardware, in order.
inline std::vector<std::string> events{};

struct FakeHw {
    struct Setup {
        std::uint8_t  mode{};
        std::uint32_t hz{1'000'000};
        std::uint32_t usPerBitQ10{Kvasir::SPI::usPerBitQ10(1'000'000)};

        constexpr bool operator==(Setup const&) const = default;
    };

    struct Snapshot {
        std::uint32_t marker{};
    };

    static constexpr unsigned Instance = 0;

    static inline bool          masked{};
    static inline Transfer      last{};
    static inline std::uint32_t gen{};
    static inline void (*done)(std::uint32_t,
                               bool){};
    static inline int                aborts{};
    static inline int                reinits{};
    static inline std::vector<Setup> configured{};

    static void mask() { masked = true; }

    static void unmask() { masked = false; }

    static void configure(Setup const& s) {
        configured.push_back(s);
        events.push_back("configure " + std::to_string(s.mode));
    }

    static void start(Transfer const& t,
                      std::uint32_t   g,
                      void (*d)(std::uint32_t,
                                bool)) {
        last = t;
        gen  = g;
        done = d;
        events.push_back("start " + std::to_string(t.frames));
    }

    static void abort() {
        ++aborts;
        events.push_back("abort");
    }

    static void reinit() {
        ++reinits;
        events.push_back("reinit");
    }

    static Snapshot snapshot() { return {42}; }

    static void log(Snapshot const&) {}

    /// The DMA finishing what was started last, as the RX completion interrupt would.
    static void finish(bool overrun = false) { done(gen, overrun); }

    static void reset() {
        masked = false;
        last   = {};
        gen    = 0;
        done   = nullptr;
        aborts = reinits = 0;
        configured.clear();
    }
};

using Core    = Kvasir::SPI::QueueCore<FakeHw, FakeClock, 4, 16>;
using Request = Core::RequestT;

template<int Id>
struct Device {
    static inline bool low_{};

    /// Read through a function: clang 23 counts a read of a class template's static from outside the class
    /// (`Device<1>::low_`) as no use and warns -Wunused-but-set-global.
    static bool low() { return low_; }

    static void select() {
        low_ = true;
        events.push_back("select " + std::to_string(Id));
    }

    static void deselect() {
        low_ = false;
        events.push_back("deselect " + std::to_string(Id));
    }

    static constexpr Lines lines{&select, &deselect, nullptr};
};

/// What the callbacks saw, per request tag.
inline std::vector<std::pair<int, TransferResult>> results{};

auto recordAs(int tag) {
    return [tag](TransferResult r) { results.emplace_back(tag, r); };
}

std::array<std::byte, 8> txBuf{};
std::array<std::byte, 8> rxBuf{};

Request write(Lines       l,
              int         tag,
              std::size_t n = 4) {
    return Request{.setup    = {},
                   .lines    = l,
                   .tx       = std::span{txBuf}.first(n),
                   .callback = recordAs(tag)};
}

void fresh() {
    Core::reset();
    events.clear();
    results.clear();
    FakeHw::reset();
    FakeClock::reset();
    Device<1>::low_ = Device<2>::low_ = false;
    Log::reset();
    static_cast<void>(Core::takeLatency());
}

int count(TransferResult r) {
    int n = 0;
    for(auto const& e : results) { n += e.second == r ? 1 : 0; }
    return n;
}

void oneCallbackEach() {
    testCase("every request one callback, CS up in the completion, FIFO order");
    fresh();
    check(Core::submit(write(Device<1>::lines, 1)), "first accepted");
    check(Core::submit(write(Device<2>::lines, 2)), "second accepted");
    check(Device<1>::low() && !Device<2>::low(), "only the first is selected while it runs");
    FakeHw::finish();
    check(!Device<1>::low(), "CS released in the completion, not a loop turn later");
    check(Device<2>::low(), "the next one started from the completion");
    FakeHw::finish();
    checkEq(results.size(), 2U, "two callbacks");
    check(results[0].first == 1 && results[1].first == 2, "in submit order");
    checkEq(count(TransferResult::succeeded), 2, "both succeeded");
    check(!Core::busy(), "idle afterwards");
    checkEq(Core::transfers(), 2U, "two transfers counted");
    check(!FakeHw::masked, "the interrupt is unmasked again");
}

void setupOnlyWhenItChanges() {
    testCase("the set-up is written only when it changes, before CS drops");
    fresh();
    auto a       = write(Device<1>::lines, 1);
    a.setup.mode = 3;
    auto b       = write(Device<1>::lines, 2);
    b.setup.mode = 3;
    auto c       = write(Device<2>::lines, 3);
    c.setup.mode = 0;
    Core::submit(a);
    Core::submit(b);
    Core::submit(c);
    FakeHw::finish();
    FakeHw::finish();
    FakeHw::finish();
    checkEq(FakeHw::configured.size(), 2U, "mode 3 once, mode 0 once");
    std::vector<std::string> const expect{"configure 3",
                                          "select 1",
                                          "start 4",
                                          "deselect 1",
                                          "select 1",
                                          "start 4",
                                          "deselect 1",
                                          "configure 0",
                                          "select 2",
                                          "start 4",
                                          "deselect 2"};
    check(events == expect, "configure, select, start, deselect in that order");
}

void holdKeepsTheBus() {
    testCase("a hold keeps CS low and runs its continuation before the queue");
    fresh();
    auto cmd = write(Device<1>::lines, 1, 1);
    cmd.hold = true;
    Core::submit(cmd);
    Core::submit(write(Device<2>::lines, 2));   // queued behind the hold
    FakeHw::finish();
    check(Device<1>::low(), "CS stays low after the held part");
    check(!Device<2>::low(), "the other device waits");
    auto data = write(Device<1>::lines, 3, 8);
    check(Core::submit(data), "the continuation is taken");
    checkEq(FakeHw::last.frames, 8U, "and started at once, ahead of the queue");
    FakeHw::finish();
    check(!Device<1>::low(), "the hold ends with a request without hold");
    check(Device<2>::low(), "then the queue moves on");
    FakeHw::finish();
    checkEq(results.size(), 3U, "three callbacks");
    check(results[1].first == 3 && results[2].first == 2, "continuation before the queued one");
    int selects = 0;
    for(auto const& e : events) { selects += e == "select 1" ? 1 : 0; }
    checkEq(selects, 1, "one CS edge for the held frame");
}

void holdTimeout() {
    testCase("a hold nobody continues is released after HoldTimeout");
    fresh();
    auto cmd = write(Device<1>::lines, 1, 1);
    cmd.hold = true;
    Core::submit(cmd);
    Core::submit(write(Device<2>::lines, 2));
    FakeHw::finish();
    FakeClock::advance(49ms);
    Core::handler();
    check(Device<1>::low(), "still held before the timeout");
    FakeClock::advance(2ms);
    Core::handler();
    check(!Device<1>::low(), "released after it");
    checkEq(Core::holdTimeouts(), 1U, "counted");
    check(Device<2>::low(), "and the queue moves on");
    FakeHw::finish();
}

void releaseHold() {
    testCase("releaseHold() ends a hold without a transfer");
    fresh();
    auto cmd = write(Device<1>::lines, 1, 1);
    cmd.hold = true;
    Core::submit(cmd);
    FakeHw::finish();
    Core::releaseHold(Device<2>::lines);
    check(Device<1>::low(), "another device cannot end it");
    Core::releaseHold(Device<1>::lines);
    check(!Device<1>::low(), "its own can");
    check(!Core::busy(), "idle");
}

void timeoutAndLateCompletion() {
    testCase("a lost transfer times out, is aborted, and its late completion is dropped");
    fresh();
    Core::submit(write(Device<1>::lines, 1, 8));   // 64 bits at 1 MHz: 64 us, deadline 2128 us
    Core::submit(write(Device<2>::lines, 2));
    auto const lostGen = FakeHw::gen;
    FakeClock::advance(2000us);
    Core::handler();
    checkEq(Core::timeouts(), 0U, "not before twice the wire time plus the margin");
    FakeClock::advance(200us);
    Core::handler();
    checkEq(Core::timeouts(), 1U, "then it is lost");
    checkEq(FakeHw::aborts, 1, "the DMA aborted");
    check(!Device<1>::low(), "CS released");
    check(results.size() == 1 && results[0].second == TransferResult::failed, "failed once");
    check(Device<2>::low(), "the next one started");
    FakeHw::done(lostGen, false);   // the old DMA's completion, arriving late
    checkEq(Core::staleCompletions(), 1U, "a late completion is counted");
    check(Device<2>::low(), "and does not end the transfer that runs now");
    checkEq(results.size(), 1U, "nor gives anyone a second callback");
    FakeHw::finish();
    checkEq(count(TransferResult::succeeded), 1, "the second completes normally");
    checkEq(Core::lastTimeout().frames, 8U, "the snapshot has the lost transfer");
    checkEq(Core::lastTimeout().hw.marker, 42U, "and the hardware's own part");
    checkEq(Core::lastTimeout().queued, 1U, "and what waited behind it");
    Core::logLastTimeout();
}

void overrun() {
    testCase("a receive overrun fails the frame and is counted");
    fresh();
    Core::submit(write(Device<1>::lines, 1));
    FakeHw::finish(true);
    checkEq(Core::overruns(), 1U, "counted");
    check(results.size() == 1 && results[0].second == TransferResult::failed, "failed");
    check(!Device<1>::low(), "CS released");
}

void failureInAHold() {
    testCase("a failure inside a hold fails the continuation too and releases CS");
    fresh();
    auto cmd = write(Device<1>::lines, 1, 1);
    cmd.hold = true;
    Core::submit(cmd);
    FakeHw::finish();
    auto next = write(Device<1>::lines, 2);
    next.hold = true;
    Core::submit(next);
    // the continuation is on the wire now; submit one more continuation behind it
    auto third = write(Device<1>::lines, 3);
    check(Core::submit(third), "a continuation behind the running one is taken");
    FakeHw::finish(true);   // the running one fails
    check(!Device<1>::low(), "CS released");
    checkEq(count(TransferResult::failed), 2, "the failed one and the waiting continuation");
    checkEq(results.size(), 3U, "every request one callback");
    check(!Core::busy(), "idle");
}

void resetDrains() {
    testCase("reset() fails what is on the wire and everything queued, once each");
    fresh();
    for(int i = 0; i < 5; ++i) { check(Core::submit(write(Device<1>::lines, i)), "accepted"); }
    check(!Core::submit(write(Device<1>::lines, 9)), "a full queue refuses");
    auto const drainedBefore = Core::drainedRequests();
    Core::reset();
    checkEq(results.size(), 5U, "five callbacks, the refused one none");
    checkEq(count(TransferResult::failed), 5, "all failed");
    checkEq(Core::drainedRequests() - drainedBefore, 4U, "four never went on the wire");
    checkEq(FakeHw::reinits, 1, "the block re-initialised");
    check(!Device<1>::low(), "CS released");
}

void submitFromACallbackDuringReset() {
    testCase("a callback that submits during reset() only queues");
    fresh();
    static bool resubmitted{};
    resubmitted = false;
    Request r   = write(Device<1>::lines, 1);
    r.callback  = [](TransferResult) {
        if(!resubmitted) {
            resubmitted = true;
            Core::submit(write(Device<2>::lines, 2));
            check(FakeHw::masked, "still masked inside reset()");
        }
    };
    Core::submit(r);
    Core::reset();
    check(Device<2>::low(), "the resubmitted request starts once the reset is over");
    FakeHw::finish();
}

void deadBus() {
    testCase("40 failures in a row re-initialise the block");
    fresh();
    for(std::uint32_t i = 0; i < Core::DeadBusFailures; ++i) {
        Core::submit(write(Device<1>::lines, 1));
        FakeHw::finish(true);
    }
    Core::handler();
    checkEq(Core::resuscitations(), 1U, "watchdog stepped in");
    checkEq(FakeHw::reinits, 1, "re-initialised");
    checkEq(Core::consecutiveFailures(), 0U, "streak restarted");
    Core::submit(write(Device<1>::lines, 1));
    FakeHw::finish();
    Core::handler();
    checkEq(Core::resuscitations(), 1U, "a success keeps it quiet");
}

void transferResolution() {
    testCase("what a request becomes on the DMA");
    fresh();
    Core::submit(Request{.lines = Device<1>::lines, .rx = std::span{rxBuf}.first(5)});
    auto const read = FakeHw::last;
    check(read.tx != nullptr && std::to_integer<int>(read.tx[0]) == 0xFF, "a read clocks out 0xFF");
    check(!read.txIncrement && read.rxIncrement && read.frames == 5, "fill fixed, rx incrementing");
    FakeHw::finish();

    Core::submit(write(Device<1>::lines, 1, 6));
    auto const w = FakeHw::last;
    check(w.txIncrement && !w.rxIncrement && w.rx != nullptr, "a write drops what comes back");
    FakeHw::finish();

    alignas(2) static std::array<std::byte, 2> pixel{std::byte{0x00}, std::byte{0xF8}};
    Core::submit(Request{.lines = Device<1>::lines, .tx = pixel, .repeat = 1000, .wide = true});
    auto const fill = FakeHw::last;
    check(fill.wide && fill.frames == 1000 && !fill.txIncrement, "a repeat of one 16-bit frame");
    FakeHw::finish();

    Core::submit(Request{.lines = Device<1>::lines, .tx = std::span{txBuf}, .wide = true});
    checkEq(FakeHw::last.frames, 4U, "8 bytes are 4 wide frames");
    FakeHw::finish();

    auto const before = Core::refused();
    check(!Core::submit(Request{.lines = Device<1>::lines}), "nothing to send is refused");
    check(!Core::submit(
            Request{.lines = Device<1>::lines, .tx = std::span{txBuf}.first(3), .wide = true}),
          "half a 16-bit frame is refused");
    check(!Core::submit(Request{.lines = Device<1>::lines,
                                .tx    = std::span{txBuf}.first(3),
                                .rx    = std::span{rxBuf}.first(4)}),
          "tx and rx of different lengths are refused");
    check(!Core::submit(Request{.lines = Device<1>::lines, .tx = std::span{txBuf}, .repeat = 3}),
          "a repeat of more than one frame is refused");
    checkEq(Core::refused() - before, 4U, "each counted");
    check(!Core::busy(), "none of them queued");
}

void wireTime() {
    testCase("wire time");
    using Kvasir::SPI::usPerBitQ10;
    checkEq(Core::wireTime(1, false, usPerBitQ10(1'000'000)).count(), 8, "a byte at 1 MHz");
    checkEq(Core::wireTime(3, false, usPerBitQ10(8'000'000)).count(),
            3,
            "24 bits at 8 MHz, rounded up");
    // 36 864 us exactly; the Q10 bit time rounds up (20.48 -> 21 units), never down
    checkEq(Core::wireTime(115'200, true, usPerBitQ10(50'000'000)).count(),
            37'800,
            "a 240x480 strip at 50 MHz, over-estimated by 2.5 %");
    checkEq(usPerBitQ10(1'000'000), 1024U, "1 us a bit is 1024 in Q10");
    checkEq(usPerBitQ10(75'000'000), 14U, "13.65 rounds up to 14");
}

void latency() {
    testCase("latency: queue wait and late completions, taken and cleared");
    fresh();
    Core::submit(write(Device<1>::lines, 1, 1));
    Core::submit(write(Device<2>::lines, 2, 1));
    FakeClock::advance(300us);
    FakeHw::finish();   // 8 us of wire, 292 late; the second waited 300 us
    FakeHw::finish();
    auto const l = Core::takeLatency();
    checkEq(l.queueWaitUs, 300U, "queue wait");
    checkEq(l.lateUs, 292U, "late completion");
    auto const again = Core::takeLatency();
    check(again.queueWaitUs == 0 && again.lateUs == 0, "cleared");
}

void commandPhase() {
    testCase("a command phase: one request, one CS edge, between() once, data into the buffer");
    fresh();
    static int betweens{};
    betweens  = 0;
    Lines l   = Device<1>::lines;
    l.between = [] {
        ++betweens;
        events.push_back("between");
    };
    std::array<std::byte, 1> cmd{std::byte{0x80}};
    Core::submit(Request{.lines    = l,
                         .command  = cmd,
                         .rx       = std::span{rxBuf}.first(3),
                         .callback = recordAs(1)});
    checkEq(FakeHw::last.frames, 1U, "the command goes first");
    check(FakeHw::last.txIncrement && !FakeHw::last.rxIncrement,
          "what comes back with it is dropped");
    FakeHw::finish();
    checkEq(betweens, 1, "between() after the command");
    check(Device<1>::low(), "CS still low");
    check(FakeHw::last.rx == rxBuf.data() && FakeHw::last.frames == 3, "the data into the buffer");
    check(results.empty(), "no callback yet");
    FakeHw::finish();
    check(results.size() == 1 && results[0].second == TransferResult::succeeded, "one callback");
    check(!Device<1>::low(), "CS released at the end");
    int selects = 0;
    for(auto const& e : events) { selects += e == "select 1" ? 1 : 0; }
    checkEq(selects, 1, "one CS edge");

    Core::submit(Request{.lines = Device<1>::lines, .command = cmd, .callback = recordAs(2)});
    checkEq(FakeHw::last.frames, 1U, "a command alone");
    FakeHw::finish();
    check(results.size() == 2 && results[1].second == TransferResult::succeeded
            && !Device<1>::low(),
          "is the whole frame");

    Core::submit(Request{.lines    = l,
                         .command  = cmd,
                         .rx       = std::span{rxBuf}.first(3),
                         .callback = recordAs(3)});
    auto const transfersBefore = FakeHw::gen;
    FakeHw::finish(true);   // the command phase fails
    check(results.size() == 3 && results[2].second == TransferResult::failed, "fails the request");
    checkEq(FakeHw::gen, transfersBefore, "and no data phase goes out");
    checkEq(betweens, 1, "nor between()");
}

/// A block with no completion interrupt (the SAM DMAC): completions found by Hw::poll() in handler().
struct PolledHw : FakeHw {
    static constexpr bool          SupportsWide = false;
    static constexpr std::uint32_t MaxFrames    = 16;
    static inline bool             doneOnWire{};

    static void poll() {
        if(doneOnWire) {
            doneOnWire = false;
            finish();
        }
    }
};

using Polled = Kvasir::SPI::QueueCore<PolledHw, FakeClock, 4, 16>;

void polledCompletion() {
    testCase("a polled block: completion found in handler(), its limits refused");
    fresh();
    Polled::reset();
    results.clear();
    Polled::submit(
      Polled::RequestT{.lines = Device<1>::lines, .tx = std::span{txBuf}, .callback = recordAs(1)});
    Polled::handler();
    check(Device<1>::low() && results.empty(), "nothing done until the wire is");
    PolledHw::doneOnWire = true;
    Polled::handler();
    check(!Device<1>::low() && results.size() == 1
            && results[0].second == TransferResult::succeeded,
          "handler() finds the completion and releases CS");
    std::array<std::byte, 20> big{};
    check(!Polled::submit(
            Polled::RequestT{.lines = Device<1>::lines, .tx = std::span{txBuf}, .wide = true}),
          "no 16-bit frames");
    check(!Polled::submit(Polled::RequestT{.lines = Device<1>::lines, .tx = big}),
          "no more frames than the DMA counts");
    std::array<std::byte, 1> one{};
    check(!Polled::submit(Polled::RequestT{.lines = Device<1>::lines, .tx = one, .repeat = 17}),
          "nor a repeat past it");
    checkEq(Polled::refused(), 3U, "each counted");
}

void callbacksMaySubmit() {
    testCase("a callback may submit: it queues, keeps the section masked, starts after");
    fresh();
    static bool startedInside{};
    static bool maskedInside{};
    static int  captured{};
    Request     r = write(Device<1>::lines, 1);
    r.callback    = [tag = 1](TransferResult) {
        Core::submit(write(Device<2>::lines, 2));
        startedInside = Device<2>::low();
        maskedInside  = FakeHw::masked;
        // the lambda's own storage is not reused underneath it by what it submitted
        captured = tag;
    };
    Core::submit(r);
    // from the completion (the DMA interrupt on the RP)
    FakeHw::finish();
    check(!startedInside, "what the callback submitted did not start inside it");
    check(maskedInside, "the section stays masked through a nested submit");
    checkEq(captured, 1, "the callback still reads its own captures");
    check(Device<2>::low(), "and it started right after the callback");
    check(!FakeHw::masked, "unmasked once the completion is over");
    FakeHw::finish();

    // from handler()'s timeout path (the main loop, under the mask)
    fresh();
    startedInside = maskedInside = false;
    auto const timeoutsBefore    = Core::timeouts();
    Core::submit(r);
    FakeClock::advance(3000us);
    Core::handler();
    checkEq(Core::timeouts() - timeoutsBefore, 1U, "timed out");
    check(!startedInside && maskedInside, "same from the timeout path");
    check(Device<2>::low(), "the queued one started once the callback was over");
    check(!FakeHw::masked, "and handler() unmasked at its end, not the nested submit");
    FakeHw::finish();
}

void resetAndReleaseHoldFromACallback() {
    testCase("reset() and releaseHold() from a callback");
    fresh();
    Request r  = write(Device<1>::lines, 1);
    r.callback = [](TransferResult res) {
        results.emplace_back(1, res);
        Core::reset();
    };
    Core::submit(r);
    Core::submit(write(Device<2>::lines, 2));
    FakeHw::finish();
    checkEq(results.size(), 2U, "the finished request once, the queued one drained once");
    check(results[0].first == 1 && results[0].second == TransferResult::succeeded,
          "the first succeeded and was not failed again by its own reset");
    check(results[1].first == 2 && results[1].second == TransferResult::failed,
          "the queued one was drained");
    checkEq(FakeHw::reinits, 1, "re-initialised once");
    check(!FakeHw::masked && !Core::busy(), "idle and unmasked afterwards");

    fresh();
    Request held  = write(Device<1>::lines, 1);
    held.hold     = true;
    held.callback = [](TransferResult) { Core::releaseHold(Device<1>::lines); };
    Core::submit(held);
    Core::submit(write(Device<2>::lines, 2));
    FakeHw::finish();
    check(!Device<1>::low(), "the hold ended inside its own callback");
    check(Device<2>::low(), "and the next device started");
    FakeHw::finish();
}

void holdNeedsASelect() {
    testCase("a hold without a select is refused: it would adopt any next request");
    fresh();
    Request r          = write(Lines{}, 1);
    r.hold             = true;
    auto const refused = Core::refused();
    check(!Core::submit(r), "refused");
    checkEq(Core::refused() - refused, 1U, "counted");
    check(!Core::busy(), "nothing queued");
}

}   // namespace

namespace {

/// Every optional part off (QueueCoreFeatures): what is left must still move frames and time
/// a lost one out.
struct BareTiming : Kvasir::SPI::QueueCoreDefaults {
    static constexpr Kvasir::SPI::QueueCoreFeatures QueueFeatures{.latency         = false,
                                                                  .counters        = false,
                                                                  .timeoutSnapshot = false};
};

using Bare = Kvasir::SPI::QueueCore<FakeHw, FakeClock, 4, 16, BareTiming>;

template<typename C>
concept HasCounters = requires { C::timeouts(); };
template<typename C>
concept HasLatency = requires { C::takeLatency(); };
template<typename C>
concept HasSnapshot = requires { C::lastTimeout(); };
static_assert(HasCounters<Core> && HasLatency<Core> && HasSnapshot<Core>,
              "all on by default");
static_assert(!HasCounters<Bare> && !HasLatency<Bare> && !HasSnapshot<Bare>,
              "and gone when off");

void bareQueue() {
    testCase("without latency, counters and snapshot a lost transfer still times out");
    Bare::reset();
    events.clear();
    results.clear();
    FakeHw::reset();
    FakeClock::reset();
    Device<1>::low_ = Device<2>::low_ = false;
    check(Bare::submit(write(Device<1>::lines, 1, 8)), "accepted");
    check(Bare::submit(write(Device<2>::lines, 2)), "second accepted");
    FakeClock::advance(2200us);
    Bare::handler();
    checkEq(FakeHw::aborts, 1, "aborted after twice the wire time plus the margin");
    check(results.size() == 1 && results[0].second == TransferResult::failed, "failed once");
    check(Device<2>::low(), "the next one started");
    FakeHw::finish();
    checkEq(count(TransferResult::succeeded), 1, "and completed");
    Bare::reset();
}

}   // namespace

int main() {
    oneCallbackEach();
    setupOnlyWhenItChanges();
    holdKeepsTheBus();
    holdTimeout();
    releaseHold();
    timeoutAndLateCompletion();
    overrun();
    failureInAHold();
    resetDrains();
    submitFromACallbackDuringReset();
    deadBus();
    transferResolution();
    wireTime();
    latency();
    polledCompletion();
    commandPhase();
    callbacksMaySubmit();
    resetAndReleaseHoldFromACallback();
    holdNeedsASelect();
    bareQueue();
    return finish();
}
