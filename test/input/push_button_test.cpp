/// PushButton in both modes over a fake pin and the fake clock: the edge mode (every edge an event,
/// queued in the interrupt, the long press found by handler()), every subset of the four events, and
/// the debounced mode (handler() polls the pin; bounce shorter than the debounce times is no event,
/// an event carries the time its level was first read).
#include <chrono>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/PushButton.hpp>
#include <string>
#include <support/FakeClock.hpp>
#include <type_traits>
#include <utility>
#include <vector>

using namespace Kvasir::Test;
using namespace std::chrono_literals;

namespace {
using Clock = FakeClockT<std::chrono::microseconds>;

/// The button's line as PushButton reads it (`apply(read(Pin{}))`, found by ADL): high = pressed.
namespace Line {
    inline bool level = false;

    struct Pin {};

    struct ReadPin {};

    constexpr ReadPin read(Pin) { return {}; }

    inline bool apply(ReadPin) { return level; }
}   // namespace Line

/// What handler() delivered, with the time where the config asks for it.
struct Seen {
    int                    type;
    Clock::time_point::rep us;
};

std::vector<Seen> seen{};

template<typename B>
void poll() {
    if constexpr(B::Config::useTime) {
        B::handler([](typename B::Event::Type t, Clock::time_point at) {
            seen.push_back({static_cast<int>(t), at.time_since_epoch().count()});
        });
    } else {
        B::handler([](typename B::Event::Type t) { seen.push_back({static_cast<int>(t), -1}); });
    }
}

struct AllTimed {
    static constexpr bool useHit          = true;
    static constexpr bool useShortRelease = true;
    static constexpr bool useLong         = true;
    static constexpr bool useLongRelease  = true;
    static constexpr bool useTime         = true;
    static constexpr auto longPressTime   = std::chrono::milliseconds{500};
};

using T = Kvasir::PushButton<Clock, Line::Pin, 8, AllTimed>::Type;

constexpr int hit          = static_cast<int>(T::hit);
constexpr int releaseShort = static_cast<int>(T::releaseShort);
constexpr int longPress    = static_cast<int>(T::longPress);
constexpr int releaseLong  = static_cast<int>(T::releaseLong);

std::string names(std::vector<Seen> const& s) {
    static constexpr char const* name[]{"hit", "releaseShort", "longPress", "releaseLong"};
    std::string                  out;
    for(auto const& e : s) {
        out += out.empty() ? "" : " ";
        out += name[e.type];
    }
    return out;
}

/// Edge mode: the interrupt on each edge, the loop polling every millisecond.
template<typename B>
void edgeTo(bool level) {
    Line::level = level;
    B::edgeCallback();
}

template<typename B>
void loopFor(std::chrono::microseconds span) {
    auto const end = Clock::now() + span;
    while(Clock::now() < end) {
        Clock::advance(1ms);
        poll<B>();
    }
}

void edgeMode() {
    testCase("edge mode: hit and short release, hit, long press once, release after long");
    using B = Kvasir::PushButton<Clock, Line::Pin, 8, AllTimed>;
    Clock::set(1s);
    seen.clear();
    static_assert(!B::Config::debounce);
    edgeTo<B>(true);
    loopFor<B>(100ms);
    edgeTo<B>(false);
    loopFor<B>(10ms);
    check(names(seen) == "hit releaseShort", "a short press: " + names(seen));
    checkEq(seen.size() == 2 ? seen[0].us : 0, 1'000'000, "the hit at the edge's time");
    checkEq(seen.size() == 2 ? seen[1].us : 0, 1'100'000, "the release at its edge's time");

    seen.clear();
    edgeTo<B>(true);
    loopFor<B>(499ms);
    check(names(seen) == "hit", "no long press before longPressTime: " + names(seen));
    loopFor<B>(2ms);
    check(names(seen) == "hit longPress", "the long press once longPressTime has passed");
    loopFor<B>(1s);
    check(names(seen) == "hit longPress", "and only once");
    edgeTo<B>(false);
    loopFor<B>(1ms);
    check(names(seen) == "hit longPress releaseLong", "the release after it is releaseLong");
}

/// A config of the events in `M` (bit 0 hit, 1 releaseShort, 2 longPress, 3 releaseLong), with
/// longPressTime exactly when it has a long event.
struct LongTime {
    static constexpr auto longPressTime = std::chrono::milliseconds{300};
};

struct NoLongTime {};

template<unsigned M>
struct Subset : std::conditional_t<(M & 12U) != 0, LongTime, NoLongTime> {
    static constexpr bool useHit          = (M & 1U) != 0;
    static constexpr bool useShortRelease = (M & 2U) != 0;
    static constexpr bool useLong         = (M & 4U) != 0;
    static constexpr bool useLongRelease  = (M & 8U) != 0;
};

template<unsigned M>
void subset() {
    using B = Kvasir::PushButton<Clock, Line::Pin, 8, Subset<M>>;
    seen.clear();
    // a short press, then a long one
    edgeTo<B>(true);
    loopFor<B>(50ms);
    edgeTo<B>(false);
    loopFor<B>(50ms);
    edgeTo<B>(true);
    loopFor<B>(400ms);
    edgeTo<B>(false);
    loopFor<B>(10ms);
    std::vector<int> want;
    if((M & 1U) != 0) { want.push_back(hit); }
    if((M & 2U) != 0) { want.push_back(releaseShort); }
    if((M & 1U) != 0) { want.push_back(hit); }
    if((M & 4U) != 0) { want.push_back(longPress); }
    if((M & 8U) != 0) {
        want.push_back(releaseLong);
    } else if((M & 2U) != 0) {
        want.push_back(releaseShort);   // without releaseLong a long press ends in a short release
    }
    std::vector<int> got;
    got.reserve(seen.size());
    for(auto const& e : seen) { got.push_back(e.type); }
    check(got == want, "events " + std::to_string(M) + ": " + names(seen));
}

void everySubset() {
    testCase("every subset of the four events: only the enabled ones, in order");
    Clock::set(1s);
    []<unsigned... M>(std::integer_sequence<unsigned, M...>) {
        (subset<M + 1U>(), ...);
    }(std::make_integer_sequence<unsigned, 15>{});
}

struct Debounced : AllTimed {
    static constexpr auto debouncePress   = std::chrono::milliseconds{3};
    static constexpr auto debounceRelease = std::chrono::milliseconds{20};
};

/// Debounced mode: the level at each loop turn, one turn a millisecond.
template<typename B>
void hold(bool                      level,
          std::chrono::milliseconds span) {
    Line::level    = level;
    auto const end = Clock::now() + span;
    while(Clock::now() < end) {
        poll<B>();
        Clock::advance(1ms);
    }
}

void debouncedMode() {
    testCase(
      "debounced: bounce shorter than the debounce times is no event; the times are the first "
      "reads");
    using B = Kvasir::PushButton<Clock, Line::Pin, 8, Debounced>;
    static_assert(B::Config::debounce);
    Clock::set(2s);
    seen.clear();
    Line::level = true;
    B::edgeCallback();   // the edge path does nothing in this mode
    hold<B>(false, 10ms);
    check(seen.empty(), "an edge is no event in the debounced mode");

    // a 2 ms spike: shorter than debouncePress
    hold<B>(true, 2ms);
    hold<B>(false, 30ms);
    check(seen.empty(), "a spike shorter than debouncePress is no press");

    auto const pressAt = Clock::now();
    hold<B>(true, 50ms);
    check(names(seen) == "hit", "a press held past debouncePress is a hit");
    checkEq(seen.empty() ? 0 : seen[0].us, pressAt.time_since_epoch().count(), "at its first read");

    // drop-outs of 8 ms in the press: shorter than debounceRelease
    for(int i = 0; i < 5; ++i) {
        hold<B>(false, 8ms);
        hold<B>(true, 12ms);
    }
    check(names(seen) == "hit", "contact drop-outs shorter than debounceRelease are no release");

    auto const releaseAt = Clock::now();
    hold<B>(false, 40ms);
    check(names(seen) == "hit releaseShort", "the release, short: " + names(seen));
    checkEq(seen.size() == 2 ? seen[1].us : 0,
            releaseAt.time_since_epoch().count(),
            "at its first read");

    seen.clear();
    auto const longAt = Clock::now();
    hold<B>(true, 600ms);
    check(names(seen) == "hit longPress", "a long press once longPressTime has passed");
    auto const lp = seen.size() == 2 ? seen[1].us - longAt.time_since_epoch().count() : 0;
    check(lp > 500'000 && lp <= 502'000,
          "the long press 500 ms after the press (" + std::to_string(lp) + " us)");
    hold<B>(false, 40ms);
    check(names(seen) == "hit longPress releaseLong", "the release after it is releaseLong");
}

struct Queued {
    static constexpr bool useHit          = true;
    static constexpr bool useShortRelease = true;
};

void queueOrder() {
    testCase("edge mode: the queue holds what came between two polls, in order");
    using B = Kvasir::PushButton<Clock, Line::Pin, 4, Queued>;   // three events fit
    seen.clear();
    edgeTo<B>(true);
    edgeTo<B>(false);
    edgeTo<B>(true);
    poll<B>();
    poll<B>();
    poll<B>();
    poll<B>();
    check(names(seen) == "hit releaseShort hit", "one event per poll, in order: " + names(seen));
}
}   // namespace

int main() {
    edgeMode();
    everySubset();
    debouncedMode();
    queueOrder();
    return finish();
}
