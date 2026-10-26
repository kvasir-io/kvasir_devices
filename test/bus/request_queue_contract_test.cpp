/// One scenario list against both queued-bus cores (I2C/Controller/QueueCore.hpp, SPI/QueueCore.hpp, through
/// QueueBuses.hpp): what they share - tickets, tombstones, cancel and deadlines of queued requests, the drain, one
/// callback per request, two buses of one shape that share nothing, the mask calls of a nested section, the dead-bus
/// streak after each way a request ends (one rule since LG3 P3).
#include "QueueBuses.hpp"

#include <chrono>
#include <cstdint>
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
void ticketWrapSkipsZero() {
    testCase(named<B>("a ticket is never 0, also when the counter wraps"));
    B::fresh();
    auto prev = B::submitTracked(1);
    check(prev.valid(), "the first ticket is valid");
    B::ok();
    bool allValid = true;
    bool inOrder  = true;
    for(int i = 0; i != 65'536; ++i) {
        B::seen.clear();
        auto const t = B::submitTracked(1);
        allValid     = allValid && t.valid();
        auto const expected
          = prev.id == 0xFFFF ? std::uint16_t{1} : static_cast<std::uint16_t>(prev.id + 1U);
        inOrder = inOrder && t.id == expected;
        prev    = t;
        B::ok();
    }
    check(allValid, "65 536 tickets, none of them 0");
    check(inOrder, "one after the other, 65 535 followed by 1");
}

template<typename B>
void tombstoneKeepsItsSlot() {
    testCase(
      named<B>("a cancelled queued request calls back before cancel() returns, keeps its slot, and "
               "is dropped at the head"));
    B::fresh();
    check(B::submit(0), "on the wire");
    Kvasir::Bus::Ticket first{};
    for(int i = 1; i <= static_cast<int>(B::Depth); ++i) {
        auto const t = B::submitTracked(i);
        check(t.valid(), "queued");
        if(i == 1) { first = t; }
    }
    check(B::cancel(first) == Cancel::removed, "the head of the queue: removed");
    check(B::seen.size() == 1 && B::seen[0] == std::pair{1, R::cancelled},
          "its callback ran inside cancel()");
    check(!B::submit(99), "the tombstone still takes its slot");
    B::ok();
    checkEq(B::starts(), 2, "the completion started the next live request, not the tombstone");
    check(B::submit(100), "the tombstone dropped at the head: its slot is back");
    for(int i = 0; i != static_cast<int>(B::Depth) + 1; ++i) { B::ok(); }
    checkEq(B::callbacksOf(1), 1, "the cancelled one: one callback");
    checkEq(B::callbacksOf(100), 1, "the late one ran");
    checkEq(B::starts(), static_cast<int>(B::Depth) + 1, "every live request once on the wire");
}

template<typename B>
void queuedExpiry() {
    testCase(named<B>(
      "a queued request past its deadline: one timedOut from turn(), drained, never started"));
    B::fresh();
    B::submit(1);
    B::submit(2, FakeClock::now() + 1ms);
    auto const drained = B::drained();
    FakeClock::advance(1500us);
    B::turn();
    check(B::seen.size() == 1 && B::seen[0] == std::pair{2, R::timedOut}, "timed out in the queue");
    checkEq(B::drained() - drained, 1U, "counted as drained");
    B::ok();
    checkEq(B::callbacksOf(2), 1, "no second callback when the tombstone reaches the head");
    checkEq(B::starts(), 1, "it never went out");
}

template<typename B>
void expiredAtStart() {
    testCase(named<B>(
      "a request whose deadline passed while it waited: one timedOut at its start, drained"));
    B::fresh();
    B::submit(1);
    B::submit(2, FakeClock::now() + 1ms);
    auto const drained = B::drained();
    FakeClock::advance(1500us);
    B::ok();
    check(B::seen.size() == 2 && B::seen[1] == std::pair{2, R::timedOut},
          "dropped at its start, timedOut");
    checkEq(B::drained() - drained, 1U, "counted as drained");
    checkEq(B::starts(), 1, "never on the wire");
}

template<typename B>
void drainSkipsTombstones() {
    testCase(named<B>(
      "reset() fails the request on the wire and the queued ones, but not a tombstone again"));
    B::fresh();
    B::submit(1);
    auto const t = B::submitTracked(2);
    B::submit(3);
    B::cancel(t);
    auto const drained = B::drained();
    B::reset();
    check(B::saw(1, R::failed) && B::saw(3, R::failed),
          "the one on the wire and the queued one failed");
    checkEq(B::callbacksOf(2), 1, "the tombstone's callback ran once, at its cancel");
    checkEq(B::drained() - drained, 1U, "one drained: the queued live one");
}

template<typename B>
void resetFailsWhatWasQueued() {
    testCase(
      named<B>("reset() fails what was there when it was called; a failure callback's resubmit "
               "waits for the "
               "block and starts after it"));
    B::fresh();
    B::submit(1);
    B::submit(2);
    static int resubmits{};
    resubmits = 0;
    B::after  = [](int tag, R r) {
        if(tag == 2 && r == R::failed && resubmits++ == 0) {
            check(B::submit(3), "accepted from the callback");
        }
    };
    B::reset();
    B::after = {};
    check(B::saw(1, R::failed) && B::saw(2, R::failed),
          "the one on the wire and the queued one failed");
    checkEq(B::callbacksOf(3), 0, "the resubmit was not failed");
    checkEq(B::starts(), 2, "it started once the block was back");
    B::ok();
    check(B::saw(3, R::ok), "and ran");
}

template<typename B>
void twelveForTwelve() {
    testCase(named<B>("12 requests, every way to end, 12 callbacks"));
    B::fresh();
    B::submit(1);
    B::submit(2);
    auto const t3 = B::submitTracked(3);
    B::submit(4, FakeClock::now() + 1ms);
    B::cancel(t3);   // 3 cancelled in the queue
    B::ok();         // 1 ok, 2 on the wire
    FakeClock::advance(1500us);
    B::turn();   // 4 timed out in the queue
    B::fail();   // 2 failed
    B::submit(5);
    auto const t6 = B::submitTracked(6);
    B::submit(7);
    B::ok();               // 5 ok, 6 on the wire
    B::cancelOnWire(t6);   // 6 cancelled, 7 on the wire
    B::submit(8);
    B::submit(9);
    B::reset();   // 7, 8, 9 failed
    B::submit(10);
    B::submit(11, FakeClock::now() + 1ms);
    FakeClock::advance(1500us);
    B::ok();   // 10 ok, 11 timed out at its start
    B::submit(12);
    B::ok();   // 12 ok
    constexpr std::array<R, 12> expected{R::ok,
                                         R::failed,
                                         R::cancelled,
                                         R::timedOut,
                                         R::ok,
                                         R::cancelled,
                                         R::failed,
                                         R::failed,
                                         R::failed,
                                         R::ok,
                                         R::timedOut,
                                         R::ok};
    checkEq(B::seen.size(), 12U, "12 callbacks");
    for(int tag = 1; tag <= 12; ++tag) {
        check(B::callbacksOf(tag) == 1 && B::saw(tag, expected[static_cast<std::size_t>(tag - 1)]),
              "request " + std::to_string(tag) + ": one callback with its result");
    }
}

/// Two buses of one shape: what one does leaves the other's queue, tickets, streak and mask alone.
template<template<typename> class Bus>
void twoBusesShareNothing() {
    using A = Bus<TagA>;
    using B = Bus<TagB>;
    testCase(named<A>("two buses of the same shape in one binary share no state"));
    A::fresh();
    B::fresh();
    auto const tb0 = B::submitTracked(1);
    B::ok();
    auto const starts = B::starts();
    auto const streak = B::streak();
    auto const masks  = B::maskCalls();
    auto const seen   = B::seen.size();

    A::submit(1);
    auto const ta = A::submitTracked(2);
    A::cancel(ta);
    A::fail();
    A::submit(3);
    A::fail();
    checkEq(A::streak(), 2U, "the busy bus counted its faults");
    A::submit(4);
    A::submit(5);
    A::reset();

    checkEq(B::starts(), starts, "the other bus started nothing");
    checkEq(B::streak(), streak, "its streak is its own");
    checkEq(B::maskCalls(), masks, "its line was never masked");
    checkEq(B::seen.size(), seen, "it saw no callback");
    auto const tb1 = B::submitTracked(2);
    checkEq(tb1.id, static_cast<std::uint16_t>(tb0.id + 1U), "its ticket counter is its own");
    for(int i = 0; i != static_cast<int>(B::Depth); ++i) {
        check(B::submit(10 + i), "its queue is its own: empty");
    }
    check(!B::submit(50), "and holds its own depth");

    // the callback guard is per bus too: a callback of one bus that submits on the other, idle one starts there at
    // once (only its own bus waits until the callback has returned)
    A::fresh();
    B::fresh();
    A::submit(1);
    auto const ta2 = A::submitTracked(2);
    A::after       = [](int tag, R) {
        if(tag == 2) { B::submit(7); }
    };
    auto const bStarts = B::starts();
    A::cancel(ta2);
    checkEq(B::starts(),
            bStarts + 1,
            "the other bus started the submit of this bus's callback at once");
    A::after = {};
    A::fresh();
    B::fresh();
}

/// The mask: one Hw::mask() per masked section. A callback run inside one (here a cancel's) that submits opens a
/// nested section.
template<typename B>
void maskCallsOfANestedSection(int expected) {
    testCase(named<B>("Hw::mask() calls of a section with a nested one"));
    B::fresh();
    B::submit(1);
    auto const t = B::submitTracked(2);
    B::after     = [](int tag, R) {
        if(tag == 2) { B::submit(3); }
    };
    auto const before = B::maskCalls();
    B::cancel(t);
    checkEq(B::maskCalls() - before,
            expected,
            "mask() calls for cancel() and the callback's submit()");
    check(!B::masked(), "unmasked afterwards");
    B::after = {};
}

/// The dead-bus streak after two faults and then one end of `e`.
template<typename B>
void streakAfter(End           e,
                 std::uint32_t expected,
                 char const*   what) {
    testCase(named<B>(what));
    B::fresh();
    for(int i = 0; i != 2; ++i) {
        B::submit(1);
        B::fail();
    }
    checkEq(B::streak(), 2U, "two faults");
    if(e == End::cancelled) {
        B::cancelOnWire(B::submitTracked(2));
    } else {
        B::submit(2, e == End::deadlineCut ? FakeClock::now() + 1ms : tp::max());
        B::end(e);
    }
    checkEq(B::callbacksOf(2), 1, "the request ended once");
    checkEq(B::streak(), expected, "the streak after it");
}

}   // namespace

int main() {
    using I = I2cBus<TagA>;
    using S = SpiBus<TagA>;

    ticketWrapSkipsZero<I>();
    ticketWrapSkipsZero<S>();
    tombstoneKeepsItsSlot<I>();
    tombstoneKeepsItsSlot<S>();
    queuedExpiry<I>();
    queuedExpiry<S>();
    expiredAtStart<I>();
    expiredAtStart<S>();
    drainSkipsTombstones<I>();
    drainSkipsTombstones<S>();
    resetFailsWhatWasQueued<I>();
    resetFailsWhatWasQueued<S>();
    twelveForTwelve<I>();
    twelveForTwelve<S>();
    twoBusesShareNothing<I2cBus>();
    twoBusesShareNothing<SpiBus>();

    // Only the outermost section masks (Bus/MaskDepth.hpp).
    maskCallsOfANestedSection<I>(1);
    maskCallsOfANestedSection<S>(1);

    // One streak rule on both buses (Bus/DeadBus.hpp): a fault counts, a working transfer (ok, NAK)
    // resets, a cancel, a deadline cut and our own reset()'s failure leave it. Before it, a cancel and a deadline cut
    // reset the I2C streak, and a deadline cut and our own reset counted on SPI.
    streakAfter<I>(End::ok, 0, "streak: ok resets");
    streakAfter<I>(End::nack, 0, "streak: a NAK resets");
    streakAfter<I>(End::fault, 3, "streak: a fault counts");
    streakAfter<I>(End::lost, 3, "streak: a transfer timeout counts");
    streakAfter<I>(End::cancelled, 2, "streak: a cancel on the wire leaves it (reset it before)");
    streakAfter<I>(End::deadlineCut,
                   2,
                   "streak: a deadline cut on the wire leaves it (reset it before)");
    streakAfter<I>(End::ownReset, 2, "streak: our own reset() leaves it");
    streakAfter<S>(End::ok, 0, "streak: ok resets");
    streakAfter<S>(End::fault, 3, "streak: a fault counts");
    streakAfter<S>(End::lost, 3, "streak: a transfer timeout counts");
    streakAfter<S>(End::cancelled, 2, "streak: a cancel on the wire leaves it");
    streakAfter<S>(End::deadlineCut,
                   2,
                   "streak: a deadline cut on the wire leaves it (counted before)");
    streakAfter<S>(End::ownReset, 2, "streak: our own reset() leaves it (counted before)");
    return finish();
}
