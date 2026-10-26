/// SPI/Client.hpp: ChipSelect's lines and hold identity, and OneFrame's one frame in flight - start, the second
/// start refused while running, a refused submit leaving it idle, refuse(), a completion inside submit(), the
/// tracked results, one callback per accepted frame - on a stub master and on the queued master's fake.
#include "Harness.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/SPI/Client.hpp>
#include <kvasir/Util/StaticFunction.hpp>
#include <span>
#include <support/FakeClock.hpp>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;

namespace {

using Kvasir::Bus::Outcome;
using Kvasir::Bus::TrackedOutcome;
using Kvasir::SPI::ChipSelect;
using Kvasir::SPI::OneFrame;

// ChipSelect

using CsA = Spi::Cs;
using CsB = Spi::Rst;

static_assert(ChipSelect<CsA>::lines.deselect == ChipSelect<CsA>::deselect,
              "deselects by default");
static_assert(ChipSelect<CsA,
                         false>::lines.deselect
                == nullptr,
              "CS stays low after the frame");
static_assert(ChipSelect<CsA>::lines.prepare == nullptr
                && ChipSelect<CsA>::lines.between == nullptr,
              "no prepare, no between");

void chipSelect() {
    testCase("ChipSelect: select low, deselect and release high, one identity per pin");
    Pins::reset();
    ChipSelect<CsA>::lines.select();
    check(!Pins::level[CsA::id], "select drives CS low");
    ChipSelect<CsA>::lines.deselect();
    check(Pins::level[CsA::id], "deselect drives it high");
    ChipSelect<CsA>::lines.select();
    ChipSelect<CsA>::release();
    check(Pins::level[CsA::id], "release drives it high");
    check(ChipSelect<CsA>::lines.select != ChipSelect<CsB>::lines.select, "two pins, two holders");
    check(ChipSelect<CsA>::lines.select == ChipSelect<CsA, false>::lines.select,
          "one pin, one holder, whatever the deselect");
}

// OneFrame on a stub master: the callback of the last accepted request is held until the test delivers it.

template<typename Result>
struct Stub {
    struct Request {
        int                                      tag{};
        Kvasir::StaticFunction<void(Result), 16> callback{};
    };

    static inline Request held{};
    static inline bool    waiting{};
    static inline bool    refuseNext{};
    /// Complete inside submit(), before it returns, as the RP's DMA interrupt may.
    static inline bool   completeInside{};
    static inline Result insideResult{Result::ok};
    static inline int    accepted{};
    static inline int    callbacks{};

    static int acceptedCount() { return accepted; }

    static void reset() {
        held           = {};
        waiting        = false;
        refuseNext     = false;
        completeInside = false;
        insideResult   = Result::ok;
        accepted = callbacks = 0;
    }

    static bool submit(Request const& r) {
        if(refuseNext) {
            refuseNext = false;
            return false;
        }
        ++accepted;
        held    = r;
        waiting = true;
        if(completeInside) { deliver(insideResult); }
        return true;
    }

    static void deliver(Result res) {
        if(!waiting) { return; }
        waiting = false;
        ++callbacks;
        auto const cb = held.callback;
        cb(res);
    }
};

using Plain   = Stub<Outcome>;
using Tracked = Stub<TrackedOutcome>;
using Poll    = OneFrame<Plain>::Poll;
using TPoll   = OneFrame<Tracked>::Poll;

void startTakeIdle() {
    testCase("OneFrame: idle -> start -> running -> ok, taken once, then idle");
    Plain::reset();
    OneFrame<Plain> f{};
    check(f.take() == Poll::idle, "idle at first");
    check(f.start(Plain::Request{.tag = 1}), "accepted");
    check(f.running() && f.take() == Poll::running, "running while on the wire");
    Plain::deliver(Outcome::ok);
    check(!f.running(), "not running after the callback");
    check(f.take() == Poll::ok, "ok");
    check(f.take() == Poll::idle, "then idle");
    check(f.start(Plain::Request{.tag = 2}), "the next frame goes");
    Plain::deliver(Outcome::failed);
    check(f.take() == Poll::failed, "failed");
    check(f.take() == Poll::idle, "then idle");
    checkEq(Plain::callbacks, Plain::acceptedCount(), "one callback per accepted frame");
}

void secondStartRefused() {
    testCase("OneFrame: a second start while running is refused, no second request goes out");
    Plain::reset();
    OneFrame<Plain> f{};
    check(f.start(Plain::Request{.tag = 1}), "first accepted");
    check(!f.start(Plain::Request{.tag = 2}), "second refused");
    checkEq(Plain::acceptedCount(), 1, "only one submit");
    checkEq(Plain::held.tag, 1, "the first is the one out");
    Plain::deliver(Outcome::ok);
    check(f.take() == Poll::ok, "the first's result");
}

void refusedSubmit() {
    testCase("OneFrame: a refused submit leaves it idle; refuse() makes it a failed frame");
    Plain::reset();
    OneFrame<Plain> f{};
    Plain::refuseNext = true;
    check(!f.start(Plain::Request{}), "refused by the master");
    check(!f.running() && f.take() == Poll::idle, "idle, nothing to wait for");
    check(f.start(Plain::Request{}), "the next start goes");
    Plain::deliver(Outcome::ok);
    check(f.take() == Poll::ok, "and completes");
    Plain::refuseNext = true;
    if(!f.start(Plain::Request{})) { f.refuse(); }
    check(f.take() == Poll::failed, "refuse(): failed once");
    check(f.take() == Poll::idle, "then idle");
    check(f.begin(), "begin");
    f.abandon();
    check(f.take() == Poll::idle, "abandon(): back to idle");
}

void completionInsideSubmit() {
    testCase("OneFrame: a completion before submit() returns still ends ok");
    Plain::reset();
    OneFrame<Plain> f{};
    Plain::completeInside = true;
    check(f.start(Plain::Request{}), "accepted");
    check(f.take() == Poll::ok, "ok, not overwritten by running");
    Plain::insideResult = Outcome::failed;
    check(f.start(Plain::Request{}), "accepted");
    check(f.take() == Poll::failed, "failed");
}

void trackedResults() {
    testCase("OneFrame: TrackedOutcome cancelled / timedOut count as failed");
    Tracked::reset();
    OneFrame<Tracked> f{};
    for(auto const r :
        {TrackedOutcome::cancelled, TrackedOutcome::timedOut, TrackedOutcome::failed})
    {
        check(f.start(Tracked::Request{}), "accepted");
        Tracked::deliver(r);
        check(f.take() == TPoll::failed, "failed");
    }
    check(f.start(Tracked::Request{}), "accepted");
    Tracked::deliver(TrackedOutcome::ok);
    check(f.take() == TPoll::ok, "ok is ok");
    checkEq(Tracked::callbacks, 4, "four callbacks for four frames");
    checkEq(Tracked::acceptedCount(), 4, "four accepted");
}

// On the queued master's fake: the real request type, ChipSelect's lines around the frame.

using Bus = Spi::DefaultBus;

struct Driver {
    OneFrame<Bus>            frame{};
    std::array<std::byte, 2> tx{std::byte{0x12}, std::byte{0x34}};
    std::array<std::byte, 2> rx{};
    using Select = ChipSelect<CsA>;

    Driver() { Select::release(); }

    bool go() {
        return frame.start(Bus::Request{
          .setup = Bus::setup(Kvasir::Hal::ClockMode::mode0, Kvasir::Units::hertz(1'000'000)),
          .lines = Select::lines,
          .tx    = tx,
          .rx    = rx});
    }
};

void onTheQueuedMaster() {
    testCase(
      "OneFrame + ChipSelect on the queued master: CS low on the wire, high after, one result");
    Spi::Harness::fresh<Bus>([](std::uint8_t b) { return static_cast<std::uint8_t>(b ^ 0xFFU); });
    Driver                   d{};
    Spi::BusScope<Bus> const scope{};
    check(Pins::level[CsA::id], "released by the constructor");
    check(d.go(), "accepted");
    check(!d.go(), "one frame in flight");
    Bus::turn();
    check(d.frame.running(), "running until the completion");
    Bus::tick();
    check(Pins::level[CsA::id], "CS up after the frame");
    check(d.frame.take() == OneFrame<Bus>::Poll::ok, "ok");
    checkEq(Bus::frames.size(), std::size_t{1}, "one frame on the wire");
    check(!Bus::frames.empty() && Bus::frames[0].selected, "the part was selected during it");
    checkEq(std::to_integer<int>(d.rx[0]), 0xED, "the part's answer is in");
    Bus::overrunNext = true;
    check(d.go(), "the next frame");
    Bus::turn();
    Bus::tick();
    check(d.frame.take() == OneFrame<Bus>::Poll::failed, "an overrun is a failed frame");
    check(d.go(), "and the frame after a failure goes again");
    Bus::turn();
    Bus::tick();
    check(d.frame.take() == OneFrame<Bus>::Poll::ok, "ok again");
}

}   // namespace

int main() {
    chipSelect();
    startTakeIdle();
    secondStartRefused();
    refusedSubmit();
    completionInsideSubmit();
    trackedResults();
    onTheQueuedMaster();
    return finish();
}
