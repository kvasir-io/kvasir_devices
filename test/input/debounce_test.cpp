/// Debounce over fake clocks: a clean change both ways (one settle time and two), bounces during
/// the wait (the wait restarts at every bounce, chatter that never settles is no change, a bounce
/// that settles on the old level is none either), pulses shorter than the wait and one exactly as
/// long, a zero settle time, start-up (the initial level, a change at the clock's epoch, a clock
/// that starts far from it), the wraps the clock types allow (unsigned 32- and 16-bit reps across
/// zero, a signed 64-bit one at the top of its range) and a seeded random walk against a model
/// that recomputes every step from the whole sample history.
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <kd/Check.hpp>
#include <kvasir/Devices/Debounce.hpp>
#include <limits>
#include <random>
#include <source_location>
#include <string>
#include <string_view>
#include <support/FakeClock.hpp>
#include <vector>

using namespace Kvasir::Test;
using namespace std::chrono_literals;

namespace {
using Us  = FakeClockT<std::chrono::microseconds>;                          // signed 64 bits
using U32 = FakeClockT<std::chrono::duration<std::uint32_t, std::micro>>;   // wraps after 71.6 min
using U16 = FakeClockT<std::chrono::duration<std::uint16_t, std::milli>>;   // wraps after 65.5 s

template<typename Clock>
constexpr typename Clock::time_point at(typename Clock::duration since) {
    return typename Clock::time_point{since};
}

/// Samples the input `level(i)` once per `step` from `start` (sample i at start + i * step) and
/// lists value()'s changes by sample index: "rise@15 fall@50".
template<typename Clock,
         typename Level>
std::string run(Kvasir::Debounce<Clock>&   d,
                typename Clock::time_point start,
                typename Clock::duration   step,
                int                        samples,
                Level                      level) {
    std::string out;
    auto        now = start;
    for(int i = 0; i < samples; ++i) {
        bool const before  = d.value();
        bool const changed = d.update(level(i), now);
        check(changed == (d.value() != before), "update() says true exactly when value() changed");
        if(changed) {
            out += std::format("{}{}@{}", out.empty() ? "" : " ", d.value() ? "rise" : "fall", i);
        }
        now += step;
    }
    return out;
}

/// The common case below: 1 ms samples from the epoch of the microsecond clock.
template<typename Level>
std::string runMs(Kvasir::Debounce<Us>& d,
                  int                   samples,
                  Level                 level) {
    return run(d, at<Us>({}), Us::duration{1ms}, samples, level);
}

/// The changes run() listed against the expected ones; the FAIL line names the caller's line.
void expect(std::string const&   got,
            std::string const&   want,
            std::string_view     what,
            std::source_location where = std::source_location::current()) {
    check(got == want, std::format("{}: got [{}], want [{}]", what, got, want), where);
}

void cleanChange() {
    testCase("a clean change both ways: value() follows settle after it, once");
    Kvasir::Debounce<Us> d{10ms};
    checkEq(d.value(), false, "starts at the default initial level");
    expect(runMs(d, 80, [](int t) { return t >= 5 && t < 40; }),
           "rise@15 fall@50",
           "high from 5 ms to 40 ms, settle 10 ms");
}

void twoSettleTimes() {
    testCase("two settle times: the rise waits rise, the fall waits fall");
    Kvasir::Debounce<Us> d{5ms, 20ms};
    expect(runMs(d, 100, [](int t) { return t >= 10 && t < 60; }),
           "rise@15 fall@80",
           "rise 5 ms, fall 20 ms");
    Kvasir::Debounce<Us> e{20ms, 5ms, true};
    expect(runMs(e, 100, [](int t) { return t < 10 || t >= 60; }),
           "fall@15 rise@80",
           "the same, mirrored");
    // a drop-out longer than rise but shorter than fall: the release guard holds the level
    Kvasir::Debounce<Us> f{5ms, 20ms, true};
    expect(runMs(f, 100, [](int t) { return t < 10 || t >= 25; }),
           "",
           "a 15 ms drop-out under a 20 ms fall time");
}

void bouncesDuringTheWait() {
    testCase("bounces during the wait restart it");
    // A->B->A->B within the settle time, the last bounce at 14 ms: the 10 ms count from 15 ms
    Kvasir::Debounce<Us> d{10ms};
    expect(runMs(d,
                 60,
                 [](int t) {
                     if(t < 5) { return false; }
                     if(t >= 15) { return true; }
                     return t <= 7 || (t >= 11 && t <= 13);
                 }),
           "rise@25",
           "the change counts from the last bounce");

    // chatter that never settles: every sample the other level
    Kvasir::Debounce<Us> e{10ms};
    expect(runMs(e, 200, [](int t) { return t % 2 == 1; }), "", "1 ms chatter for 200 ms");
    // nor chatter slower than the samples but faster than the settle time
    Kvasir::Debounce<Us> f{10ms};
    expect(runMs(f, 200, [](int t) { return (t / 9) % 2 == 1; }), "", "9 ms on, 9 ms off");

    // a bounce that settles on the old level is no change, and leaves no wait behind: the
    // later real change waits its own full settle time
    Kvasir::Debounce<Us> g{10ms};
    expect(runMs(g, 80, [](int t) { return t == 5 || t == 6 || t == 8 || t >= 30; }),
           "rise@40",
           "a burst at 5 ms back to low, then high from 30 ms");

    // the same in the other direction
    Kvasir::Debounce<Us> h{10ms, true};
    expect(runMs(h, 80, [](int t) { return !(t == 5 || t == 6 || t == 8 || t >= 30); }),
           "fall@40",
           "a burst at 5 ms back to high, then low from 30 ms");
}

void shortPulses() {
    testCase("a pulse shorter than the wait is no change, one exactly as long is");
    Kvasir::Debounce<Us> d{10ms};
    expect(runMs(d, 60, [](int t) { return t >= 5 && t <= 14; }),
           "",
           "high for samples 5..14: 9 ms");
    Kvasir::Debounce<Us> e{10ms};
    expect(runMs(e, 60, [](int t) { return t >= 5 && t <= 15; }),
           "rise@15 fall@26",
           "high for samples 5..15: 10 ms");

    // two one-sample spikes a settle time and more apart: nothing to hold
    Kvasir::Debounce<Us> f{10ms};
    expect(runMs(f, 60, [](int t) { return t == 5 || t == 17; }), "", "spikes at 5 and 17 ms");
    Kvasir::Debounce<Us> g{10ms};
    expect(runMs(g, 200, [](int t) { return t % 12 == 5; }), "", "a spike every 12 ms");
    Kvasir::Debounce<Us> h{10ms, true};
    expect(runMs(h, 200, [](int t) { return t % 12 != 5; }), "", "a drop-out every 12 ms");

    // to the tick: the wait is `>= settle`
    Kvasir::Debounce<Us> k{10ms};
    check(!k.update(true, at<Us>(1s)), "the first sample starts the wait");
    check(!k.update(true, at<Us>(1s + 10ms - 1us)), "one tick short: no change");
    check(k.update(true, at<Us>(1s + 10ms)), "settle after the first sample: the change");
    check(!k.update(true, at<Us>(1s + 11ms)), "and no second report");
}

void zeroSettle() {
    testCase("a zero settle time passes every change at once");
    Kvasir::Debounce<Us> d{Us::duration::zero()};
    expect(runMs(d, 6, [](int t) { return t % 2 == 1; }),
           "rise@1 fall@2 rise@3 fall@4 rise@5",
           "every sample");
    Kvasir::Debounce<Us> e{Us::duration::zero(), 10ms};
    expect(runMs(e, 40, [](int t) { return t >= 5 && t != 8; }),
           "rise@5",
           "rise 0: the rise at once; a 1 ms drop-out under the 10 ms fall time is none");
}

void startUp() {
    testCase("start-up: the initial level, a change at the epoch, a clock that starts late");
    Kvasir::Debounce<Us> high{10ms, true};
    checkEq(high.value(), true, "starts at the initial level");
    expect(runMs(high, 30, [](int) { return true; }),
           "",
           "an input at the initial level is no change");

    // a change at the clock's epoch: the start time 0 is a time like any other
    Kvasir::Debounce<Us> d{10ms};
    check(!d.update(true, at<Us>({})), "a change at time zero starts the wait");
    check(!d.update(true, at<Us>(10ms - 1us)), "and waits");
    check(d.update(true, at<Us>(10ms)), "settle after time zero");

    // a clock that starts far from its epoch (a SysTick clock synced to an outside time)
    Kvasir::Debounce<Us> late{10ms};
    check(!late.update(true, at<Us>(1000s)), "the first sample at 1000 s is no change at once");
    check(!late.update(true, at<Us>(1000s + 9ms)), "nor 9 ms later");
    check(late.update(true, at<Us>(1000s + 10ms)), "10 ms later it is");
}

/// The unsigned clocks: a change pending across the rep's wrap to zero, both ways, and a
/// bounce right at it.
template<typename Clock>
void unsignedWrap(char const* name) {
    testCase(name);
    using D = typename Clock::duration;
    // sample 0 is 3 ms before the rep's top wraps to 0, sample 3 is at 0
    constexpr D top{std::numeric_limits<typename Clock::rep>::max()};
    auto const  before = at<Clock>(top - std::chrono::duration_cast<D>(3ms) + D{1});
    auto const  step   = std::chrono::duration_cast<D>(1ms);

    Kvasir::Debounce<Clock> d{std::chrono::duration_cast<D>(10ms)};
    expect(run(d, before, step, 40, [](int t) { return t >= 1 && t < 20; }),
           "rise@11 fall@30",
           "high from 2 ms before the wrap: the rise after it, the fall too");

    Kvasir::Debounce<Clock> e{std::chrono::duration_cast<D>(10ms), true};
    expect(run(e, before, step, 40, [](int t) { return t < 1 || t >= 20; }),
           "fall@11 rise@30",
           "low from 2 ms before the wrap");

    // a bounce on both sides of the wrap: the wait starts at its last sample, after the wrap
    Kvasir::Debounce<Clock> f{std::chrono::duration_cast<D>(10ms)};
    expect(run(f, before, step, 40, [](int t) { return t == 1 || t >= 5; }),
           "rise@15",
           "a spike 2 ms before the wrap, the level from 2 ms after it");
}

void signedTop() {
    testCase("a signed 64-bit clock at the top of its range: no deadline that overflows");
    auto const           top = Us::duration::max();
    Kvasir::Debounce<Us> d{10ms};
    expect(run(d, at<Us>(top - 20ms), Us::duration{1ms}, 20, [](int t) { return t >= 2; }),
           "rise@12",
           "a change 18 ms before the top");
    // a settle time longer than the rest of the range: a deadline `start + settle` overflows
    Kvasir::Debounce<Us> e{1h};
    check(!e.update(true, at<Us>(top - 30min)), "a change half an hour before the top");
    check(!e.update(true, at<Us>(top)), "the top itself: 30 min of 1 h");
}

void readsTheClock() {
    testCase("update(raw) reads the clock");
    Us::set(5s);
    Kvasir::Debounce<Us> d{10ms};
    check(!d.update(true), "the change at 5 s");
    Us::advance(9ms);
    check(!d.update(true), "9 ms later");
    Us::advance(1ms);
    check(d.update(true), "10 ms later");
    Us::reset();
}

static_assert(
  [] {
      Kvasir::Debounce<Us> d{10ms};
      int                  changes = 0;
      for(int t = 0; t < 18; ++t) {   // bounces at 1, 4 and 7 ms, high from 8 ms
          changes += d.update(t > 7 || t % 3 != 1, at<Us>(std::chrono::milliseconds{t})) ? 1 : 0;
      }
      return changes == 0 && !d.value() && d.update(true, at<Us>(18ms)) && d.value();
  }(),
  "constexpr: a bouncing rise is seen settle after its last bounce");

/// Random sample gaps and an input that holds for a while and chatters for a while, against a
/// model that recomputes from the whole history after each sample: the output turns to the
/// input's level once the input's trailing run of that level spans the settle time of the
/// direction.
void randomAgainstModel() {
    testCase("random input, random sample times: the same as the model from the history");
    std::mt19937                       rng{0x5eed'0001U};   // fixed: every run the same
    std::uniform_int_distribution<int> gap{1, 3'000};       // µs between samples
    std::uniform_int_distribution<int> hold{0, 40};         // samples a level holds, 0 = chatter
    std::bernoulli_distribution        coin{0.5};

    for(int round = 0; round < 8; ++round) {
        auto const rise = std::chrono::microseconds{round * 1'500};   // 0 (passing) up to 10.5 ms
        auto const fall = std::chrono::microseconds{(7 - round) * 900 + 250};
        bool const initial = (round % 2) == 1;

        Kvasir::Debounce<Us>        d{rise, fall, initial};
        std::vector<Us::time_point> times;
        std::vector<bool>           levels;
        bool                        model   = initial;
        bool                        level   = initial;
        int                         left    = 0;
        int                         changes = 0;
        auto                        now     = at<Us>(std::chrono::microseconds{round} * 1'000'000);

        for(int i = 0; i < 20'000; ++i) {
            if(left-- <= 0) {
                left  = hold(rng);
                level = coin(rng);
            }
            now += std::chrono::microseconds{gap(rng)};
            times.push_back(now);
            levels.push_back(level);

            bool const changed = d.update(level, now);

            // the model: the trailing run of samples at this level, from its first sample
            bool modelChanged = false;
            if(level != model) {
                std::size_t first = levels.size() - 1;
                while(first > 0 && levels[first - 1] == level) { --first; }
                if(now - times[first] >= (level ? rise : fall)) {
                    model        = level;
                    modelChanged = true;
                }
            }
            if(changed != modelChanged || d.value() != model) {
                failf(std::source_location::current(),
                      "round {} sample {}: debounce {} (changed {}), model {} (changed {})",
                      round,
                      i,
                      d.value(),
                      changed,
                      model,
                      modelChanged);
                return;
            }
            changes += changed ? 1 : 0;
        }
        check(changes > 20,
              std::format("round {}: the walk changed the output ({} times)", round, changes));
    }
}

}   // namespace

int main() {
    cleanChange();
    twoSettleTimes();
    bouncesDuringTheWait();
    shortPulses();
    zeroSettle();
    startUp();
    unsignedWrap<U32>("an unsigned 32-bit microsecond clock across its wrap");
    unsignedWrap<U16>("an unsigned 16-bit millisecond clock across its wrap");
    signedTop();
    readsTheClock();
    randomAgainstModel();
    return finish();
}
