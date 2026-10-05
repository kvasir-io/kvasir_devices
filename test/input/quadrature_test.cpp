/// QuadratureDecoder: clean turns both ways, contact bounce on every edge, a knob turned half way
/// and let go, a lost edge, and a random walk of a bouncing knob against the detents it really
/// went through.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include <cstdint>
#include <initializer_list>
#include <kvasir/Devices/QuadratureDecoder.hpp>
#include <random>

using namespace Kvasir::Test;

namespace {

/// The states of one forward detent of a whole-cycle encoder, (A << 1) | B, from rest to rest.
constexpr unsigned Forward[4] = {0b10, 0b11, 0b01, 0b00};

template<typename D>
constexpr int feed(D&                              d,
                   std::initializer_list<unsigned> states) {
    int detents{};
    for(auto s : states) { detents += d.update((s & 2U) != 0, (s & 1U) != 0); }
    return detents;
}

static_assert(
  [] {
      Kvasir::QuadratureDecoder<> d{};
      return feed(d, {0b10, 0b11, 0b01, 0b00}) == 1 && feed(d, {0b01, 0b11, 0b10, 0b00}) == -1;
  }(),
  "one detent forward, one back");

void cleanTurns() {
    testCase("clean turns");
    Kvasir::QuadratureDecoder<> d{};
    int                         total{};
    for(int i = 0; i < 24; ++i) {
        int const got = feed(d, {Forward[0], Forward[1], Forward[2], Forward[3]});
        checkEq(got, 1, "each forward detent counts one");
        total += got;
    }
    for(int i = 0; i < 30; ++i) { total += feed(d, {0b01, 0b11, 0b10, 0b00}); }
    checkEq(total, -6, "24 forward, 30 back");
    checkEq(d.skipped(), 0, "nothing skipped");
}

void noCountBeforeRest() {
    testCase("counts on arrival in the detent");
    Kvasir::QuadratureDecoder<> d{};
    checkEq(feed(d, {0b10, 0b11, 0b01}), 0, "three quarters: nothing yet");
    checkEq(feed(d, {0b00}), 1, "the detent");
}

void bounce() {
    testCase("bounce on every edge");
    Kvasir::QuadratureDecoder<> d{};
    // every change chatters three times before it settles
    int  total{};
    auto prev = 0U;
    for(int detent = 0; detent < 10; ++detent) {
        for(auto s : Forward) {
            for(int k = 0; k < 3; ++k) { total += feed(d, {s, prev}); }
            total += feed(d, {s});
            prev = s;
        }
    }
    checkEq(total, 10, "10 detents through the bounce");
    checkEq(d.skipped(), 0, "bounce of one line never looks like a skip");
}

void halfwayAndBack() {
    testCase("half way and let go");
    Kvasir::QuadratureDecoder<> d{};
    checkEq(feed(d, {0b10, 0b11, 0b10, 0b00}), 0, "half a detent forward and back");
    checkEq(feed(d, {0b01, 0b00, 0b01, 0b00}), 0, "rocking on the detent");
    checkEq(feed(d, {0b10, 0b11, 0b01, 0b11, 0b10, 0b00}), 0, "three quarters and back");
}

void lostEdge() {
    testCase("a lost edge");
    Kvasir::QuadratureDecoder<> d{};
    // 11 never seen: 10 -> 01 changes both lines
    checkEq(feed(d, {0b10, 0b01, 0b00}), 1, "the click still counts");
    checkEq(d.skipped(), 1, "and the skip is counted");
    checkEq(feed(d, {0b11, 0b00}),
            0,
            "two lost edges in one detent: direction unknown, not counted");
}

void halfCycle() {
    testCase("half a cycle per detent");
    Kvasir::QuadratureDecoder<2> d{};
    checkEq(feed(d, {0b10, 0b11}), 1, "00 -> 11 is one detent");
    checkEq(feed(d, {0b01, 0b00}), 1, "11 -> 00 the next");
    checkEq(feed(d, {0b01, 0b11}), -1, "and back");
}

void startsWhereTheLinesAre() {
    testCase("starts from the lines' levels");
    Kvasir::QuadratureDecoder<> d{true, false};   // between detents at power-up
    checkEq(feed(d, {0b11, 0b01, 0b00}), 1, "finishes the detent it was in");
}

/// A knob walked at random over the detents; each change of a line bounces 0..3 times. The
/// decoder sees every level (the interrupt is never late), so its sum is the walk's.
void randomWalk() {
    testCase("random walk with bounce");
    std::mt19937                       rng{12345};
    std::uniform_int_distribution<int> dir{0, 2};
    std::uniform_int_distribution<int> chatter{0, 3};

    Kvasir::QuadratureDecoder<> d{};
    unsigned                    pos{};   // quarter steps, mod 4 is the index into the Gray cycle
    int                         counted{};
    int                         quarter{};   // quarter steps from the start
    constexpr unsigned          Cycle[4] = {0b00, 0b10, 0b11, 0b01};

    for(int i = 0; i < 20000; ++i) {
        int const      step = dir(rng) == 0 ? -1 : 1;
        unsigned const from = Cycle[pos % 4];
        pos                 = static_cast<unsigned>(static_cast<int>(pos) + step);
        quarter += step;
        unsigned const to = Cycle[pos % 4];
        for(int k = chatter(rng); k > 0; --k) { counted += feed(d, {to, from}); }
        counted += feed(d, {to});
    }
    // run forward to the next detent so the walk ends in one
    while(pos % 4 != 0) {
        ++pos;
        ++quarter;
        counted += feed(d, {Cycle[pos % 4]});
    }
    checkEq(counted, quarter / 4, "the detents the walk went through");
    checkEq(d.skipped(), 0, "nothing skipped");
}

}   // namespace

int main() {
    cleanTurns();
    noCountBeforeRest();
    bounce();
    halfwayAndBack();
    lostEdge();
    halfCycle();
    startsWhereTheLinesAre();
    randomWalk();
    return finish();
}
