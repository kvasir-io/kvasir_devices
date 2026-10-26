/// "Every accepted request gets exactly one callback" and "what a callback submits starts after it has returned"
/// (both cores' headers), against callbacks that submit, cancel or reset from inside the core: the queued walks
/// (cancel, deadline expiry), the expired-at-start loop, a completion. One script per case, run on both cores
/// through QueueBuses.hpp.
#include "QueueBuses.hpp"

#include <chrono>
#include <kd/Check.hpp>
#include <string>
#include <support/FakeClock.hpp>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::Test::QueueBuses;
using Kvasir::Bus::Cancel;

namespace {

template<typename B>
std::string named(char const* what) {
    return std::string{B::name} + ": " + what;
}

template<typename B>
void endAll(int n) {
    for(int i = 0; i != n; ++i) { B::ok(); }
}

template<typename B>
void eachOnce(std::initializer_list<int> tags) {
    for(int tag : tags) {
        checkEq(B::callbacksOf(tag),
                1,
                "request " + std::to_string(tag) + ": exactly one callback");
    }
}

/// The queued expiry walks the queue between the head and tail it read first; a timedOut callback that submits on an
/// idle bus must not start (and expire) a request the walk has still to reach.
template<typename B>
void expiryWalkAndASubmit() {
    testCase(named<B>("a timedOut callback of the expiry walk submits on an idle bus"));
    B::fresh();
    B::holdIdle();
    B::submit(1, FakeClock::now() + 1ms);
    B::submit(2, FakeClock::now() + 1ms);
    B::after = [](int tag, R) {
        if(tag == 1) { B::submit(3); }
    };
    FakeClock::advance(1500us);
    B::releaseIdle();
    B::turn();
    B::after = {};
    endAll<B>(3);
    eachOnce<B>({1, 2, 3});
    check(B::saw(1, R::timedOut) && B::saw(2, R::timedOut) && B::saw(3, R::ok),
          "the two timed out, the new one ran");
}

/// The queued expiry walk again, its first timedOut callback resets the bus: the reset fails (and pops) the second
/// expired request, and the walk, which read head and tail before, must not reach its stale slot and call it again.
template<typename B>
void expiryWalkAndAReset() {
    testCase(named<B>("a timedOut callback of the expiry walk resets the bus"));
    B::fresh();
    B::holdIdle();
    B::submit(0);   // SPI: on the wire; I2C: queued, no deadline
    B::submit(1, FakeClock::now() + 1ms);
    B::submit(2, FakeClock::now() + 1ms);
    static bool once{};
    once     = false;
    B::after = [](int tag, R) {
        if(tag == 1 && !once) {
            once = true;
            B::reset();
        }
    };
    FakeClock::advance(1500us);
    B::releaseIdle();
    B::turn();
    B::after = {};
    endAll<B>(2);
    eachOnce<B>({0, 1, 2});
    check(B::saw(1, R::timedOut) && B::saw(2, R::failed), "1 timed out, 2 failed by the reset");
}

/// A request found expired at its start: its timedOut callback resubmits. The start loop must not run again inside
/// it and start a second request over the first.
template<typename B>
void expiredAtStartResubmits() {
    testCase(
      named<B>("an expired request's timedOut callback resubmits while the bus starts the next"));
    B::fresh();
    B::holdIdle();
    B::submit(1, FakeClock::now() + 1ms);
    FakeClock::advance(1500us);
    B::releaseIdle();
    B::after = [](int tag, R) {
        if(tag == 1) { B::submit(3); }
    };
    B::submit(2);
    checkEq(B::starts(), 1, "one request on the wire");
    endAll<B>(3);
    B::after = {};
    eachOnce<B>({2, 3});
    check(B::callbacksOf(1) <= 1, "the expired one at most once");
}

/// A completion callback that resets the bus: the request it belongs to is over and gets no second callback.
template<typename B>
void resetFromACompletionCallback() {
    testCase(named<B>("reset() from a completion callback"));
    B::fresh();
    static bool once{};
    once     = false;
    B::after = [](int tag, R) {
        if(tag == 1 && !once) {
            once = true;
            B::reset();
        }
    };
    B::submit(1);
    B::submit(2);
    B::ok();
    B::after = {};
    eachOnce<B>({1, 2});
    check(B::saw(1, R::ok) && B::saw(2, R::failed),
          "the completed one ok, the queued one failed by the reset");
}

/// A cancelled request's callback resets the bus and fills the queue again: the pushes land in the slot of the
/// request whose callback is running.
template<typename B>
void cancelCallbackRefillsItsOwnSlot() {
    testCase(named<B>("a cancel callback that resets and refills the queue"));
    B::fresh();
    B::submit(0);
    auto const t = B::Core::submitTracked(B::makeRecordingLast(1));
    B::submit(2);
    static bool once{};
    once     = false;
    B::after = [](int tag, R) {
        if(tag != 1 || once) { return; }
        once = true;
        B::reset();
        // four: the I2C queue's depth, all of them waiting while the callback runs; the last lands in the slot of
        // the request whose callback this is
        for(int i = 10; i != 14; ++i) { check(B::submit(i), "accepted from the callback"); }
    };
    check(B::cancel(t) == Cancel::removed, "removed");
    B::after = {};
    B::turn();   // SPI starts what a queued cancel's callback submitted at the next turn()
    endAll<B>(4);
    eachOnce<B>({0, 1, 2, 10, 11, 12, 13});
    check(B::saw(1, R::cancelled), "the cancelled one saw cancelled");
}

/// A cancel callback's submit waits for the callback, then starts at once - not at the next turn().
template<typename B>
void cancelCallbackSubmitStartsAtOnce() {
    testCase(named<B>("what a cancel callback submits starts when cancel() returns"));
    B::fresh();
    B::holdIdle();
    auto const t = B::submitTracked(1);
    B::releaseIdle();
    auto const starts = B::starts();   // I2C: 1 waits in the queue; SPI: 1 is on the wire
    B::after          = [](int tag, R) {
        if(tag == 1) { B::submit(2); }
    };
    check(B::cancel(t) == Cancel::removed, "removed");
    B::after = {};
    checkEq(B::starts(), starts + 1, "the callback's request is on the wire");
    B::ok();
    eachOnce<B>({1, 2});
    check(B::saw(1, R::cancelled) && B::saw(2, R::ok), "cancelled, then the new one ran");
}

/// I2C: a reset() while a stop of ours is out (a cancel on the wire) fails the request; the next request on the wire
/// must take a cancel again - a stop still marked pending would make cancel() answer `stopping` with no STOP asked
/// for, and let a timeout end it as `cancelled`.
void i2cResetWhileStopping() {
    using B = I2cBus<TagA>;
    testCase("i2c: reset() while a cancel on the wire is stopping");
    B::fresh();
    auto const t1 = B::submitTracked(1);
    check(B::cancel(t1) == Cancel::stopping, "1 on the wire: stopping");
    B::reset();
    auto const t2    = B::submitTracked(2);
    auto const stops = B::Hw::stops;
    check(B::cancel(t2) == Cancel::stopping, "2 on the wire: stopping");
    checkEq(B::Hw::stops, stops + 1, "a STOP asked for 2");
    B::Hw::raise(Kvasir::Test::I2cQueue::Wire::stopped);
    eachOnce<B>({1, 2});
    check(B::saw(1, R::failed) && B::saw(2, R::cancelled), "1 failed by the reset, 2 cancelled");
}

}   // namespace

int main() {
    using I = I2cBus<TagA>;
    using S = SpiBus<TagA>;
    expiryWalkAndASubmit<I>();
    expiryWalkAndASubmit<S>();
    expiryWalkAndAReset<I>();
    expiryWalkAndAReset<S>();
    expiredAtStartResubmits<I>();
    expiredAtStartResubmits<S>();
    resetFromACompletionCallback<I>();
    resetFromACompletionCallback<S>();
    cancelCallbackRefillsItsOwnSlot<I>();
    cancelCallbackRefillsItsOwnSlot<S>();
    cancelCallbackSubmitStartsAtOnce<I>();
    cancelCallbackSubmitStartsAtOnce<S>();
    i2cResetWhileStopping();
    return finish();
}
