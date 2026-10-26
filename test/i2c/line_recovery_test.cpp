/// Kvasir::I2C::LineRecovery against a model of the two lines and of the slave that holds SDA: the stuck threshold
/// per bus rate; the idle look that finds SDA held and the transfer start that restarts its timer; the sequence for a
/// slave that lets go after a few clocks, one that never lets go until it sees a STOP, one that takes SDA again after
/// it let go, and one that holds SCL too; the backoff between automatic recoveries; and that the pad is pushed high
/// nowhere but in the forced STOP.
///
/// The model: a line reads high unless the pad drives it low (an output with its latch low) or the slave holds it.
/// An output with its latch high is a push-pull high and is counted. The pin actions are tags in this file's namespace,
/// found by argument-dependent lookup as the chip's are; the clock is the SDK's FakeClock (microseconds), wound by the
/// test.
#include <array>
#include <chrono>
#include <cstdint>
#include <kd/Case.hpp>
#include <kd/Check.hpp>
#include <kvasir/Devices/I2C/Controller/LineRecovery.hpp>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;

namespace lrtest {

/// One line as the pad and the slave leave it.
struct Line {
    bool output{};   // the pad is an output
    bool latch{};    // its OUT latch
    bool held{};     // the slave pulls it low

    [[nodiscard]] bool drivenLow() const { return output && !latch; }

    [[nodiscard]] bool high() const { return !drivenLow() && !held; }
};

/// What the slave does with SDA.
enum class Slave : std::uint8_t {
    none,           // never holds it
    releaseAfter,   // lets go after `clocks` rising SCL edges
    untilStop,      // holds it until it sees a STOP (SDA rising while SCL is high)
    retake,   // lets go at an SCL rising edge, takes it again when the master next pulls SDA low
    dead,     // holds it whatever happens
};

enum class Kind : std::uint8_t { abort, pinsBack, makeOutput, makeInput, clear, outputHigh };

struct Event {
    Kind         kind;
    int          pin;   // 0 SDA, 1 SCL, -1 for the block's actions
    std::int64_t us;    // fake time
};

struct Model {
    Line               sda{};
    Line               scl{};
    bool               sclHeld{};   // the slave holds SCL low as well
    Slave              slave{Slave::none};
    int                clocks{};   // releaseAfter: rising edges still to go
    bool               retakeArmed{};
    int                pushedHighs{};
    int                risingEdges{};
    int                stopsSeen{};
    std::vector<Event> events;
};

inline Model model{};

inline std::int64_t nowUs() { return FakeClock::now().time_since_epoch().count(); }

inline void reset(Slave slave,
                  int   clocks = 0) {
    model        = Model{};
    model.slave  = slave;
    model.clocks = clocks;
    model.sda.held
      = slave != Slave::none;   // the slave holds SDA from the start (a transfer it was cut off in)
}

inline Line& line(int pin) { return pin == 0 ? model.sda : model.scl; }

inline bool reads(int pin) {
    if(pin == 1) { return model.scl.high() && !model.sclHeld; }
    return model.sda.high();
}

inline void record(Kind k,
                   int  pin) {
    model.events.push_back(Event{k, pin, nowUs()});
}

/// The slave's view after the pad changed: an SCL rising edge clocks it, SDA rising while SCL is high is a STOP.
inline void changed(int  pin,
                    bool wasHigh) {
    bool const isHigh = reads(pin);
    if(pin == 1 && !wasHigh && isHigh) {
        ++model.risingEdges;
        switch(model.slave) {
        case Slave::releaseAfter:
            if(model.clocks > 0) {
                --model.clocks;
                if(model.clocks == 0) { model.sda.held = false; }
            }
            break;
        case Slave::retake:
            if(model.sda.held) {
                model.sda.held    = false;
                model.retakeArmed = true;
            }
            break;
        default: break;
        }
    }
    if(pin == 0 && !wasHigh && isHigh && reads(1)) { ++model.stopsSeen; }
}

template<int N>
struct Pin {};

template<int N>
struct MakeOutput {};

template<int N>
struct MakeInput {};

template<int N>
struct Clear {};

template<int N>
struct OutputHigh {};

template<int N>
struct Read {};

struct AbortTag {};

struct PinsBackTag {};

template<int N>
MakeOutput<N> makeOutput(Pin<N>) {
    return {};
}

template<int N>
MakeInput<N> makeInput(Pin<N>) {
    return {};
}

template<int N>
Clear<N> clear(Pin<N>) {
    return {};
}

template<int N>
OutputHigh<N> makeOutputInitHigh(Pin<N>) {
    return {};
}

template<int N>
Read<N> read(Pin<N>) {
    return {};
}

template<int N>
void apply(MakeOutput<N>) {
    record(Kind::makeOutput, N);
    bool const was = reads(N);
    line(N).output = true;
    if(line(N).latch) { ++model.pushedHighs; }
    if(N == 0 && model.slave == Slave::retake && model.retakeArmed) {
        model.sda.held    = true;   // takes SDA again as soon as the master pulls it low
        model.retakeArmed = false;
    }
    changed(N, was);
}

template<int N>
void apply(MakeInput<N>) {
    record(Kind::makeInput, N);
    bool const was = reads(N);
    line(N).output = false;
    changed(N, was);
}

template<int N>
void apply(Clear<N>) {
    record(Kind::clear, N);
    bool const was = reads(N);
    line(N).latch  = false;
    changed(N, was);
}

template<int N>
void apply(OutputHigh<N>) {
    record(Kind::outputHigh, N);
    ++model.pushedHighs;
    bool const was = reads(N);
    line(N).output = true;
    line(N).latch  = true;
    // A push-pull high against an open-drain pull-down wins: the slave sees SDA rise with SCL high, a STOP.
    if(N == 0 && reads(1) && model.slave != Slave::dead) {
        model.sda.held    = false;
        model.retakeArmed = false;
    }
    changed(N, was);
}

template<int N>
std::array<std::uint32_t,
           1>
apply(Read<N>) {
    return {reads(N) ? 1U : 0U};
}

inline void apply(AbortTag) { record(Kind::abort, -1); }

inline void apply(PinsBackTag) { record(Kind::pinsBack, -1); }

template<std::uint32_t Baud>
struct Config {
    static constexpr Pin<0>        sdaPinLocation{};
    static constexpr Pin<1>        sclPinLocation{};
    static constexpr std::uint32_t baudRate = Baud;
};

template<std::uint32_t Baud, std::uint32_t MinBaud>
struct SlowestConfig : Config<Baud> {
    static constexpr std::uint32_t minBaudRate = MinBaud;
};

/// One per case (Id): the recovery's statics are per instantiation, so each case starts from a fresh one.
template<int Id, typename Cfg = Config<400'000>>
struct Base {
    using I2CConfig                       = Cfg;
    static constexpr int         instance = 0;
    static constexpr AbortTag    softAbortRequest{};
    static constexpr PinsBackTag initStepPinConfig{};
};

template<int Id, typename Cfg = Config<400'000>>
using Recovery = Kvasir::I2C::LineRecovery<Base<Id, Cfg>, FakeClock>;

/// Not at the clock's epoch: the recovery takes a zero time point for "not seen low yet".
inline void startClock() { FakeClock::set(1s); }

/// Ticks every microsecond until the recovery is idle again (or `limitUs` passed); counts the results.
struct Run {
    int          busy{};
    int          needsReinit{};
    int          idleAfter{};
    std::int64_t endUs{};
};

template<typename R>
Run runToIdle(std::int64_t limitUs = 10'000) {
    Run        run{};
    auto const start = nowUs();
    while(nowUs() - start < limitUs) {
        auto const r = R::tick(FakeClock::now());
        if(r == R::TickResult::busy) {
            ++run.busy;
        } else if(r == R::TickResult::needsReinit) {
            ++run.needsReinit;
            run.endUs = nowUs();
        } else {
            ++run.idleAfter;
            if(run.needsReinit != 0) { break; }
        }
        FakeClock::advance(1us);
    }
    return run;
}

/// The pin events of one kind and pin, in order.
inline std::vector<Event> eventsOf(Kind k,
                                   int  pin) {
    std::vector<Event> out{};
    for(auto const& e : model.events) {
        if(e.kind == k && e.pin == pin) { out.push_back(e); }
    }
    return out;
}

/// The index of the first event of a kind and pin, or -1.
inline int firstIndex(Kind k,
                      int  pin) {
    for(std::size_t i = 0; i != model.events.size(); ++i) {
        if(model.events[i].kind == k && model.events[i].pin == pin) { return static_cast<int>(i); }
    }
    return -1;
}

inline int lastIndex(Kind k,
                     int  pin) {
    int found = -1;
    for(std::size_t i = 0; i != model.events.size(); ++i) {
        if(model.events[i].kind == k && model.events[i].pin == pin) { found = static_cast<int>(i); }
    }
    return found;
}

}   // namespace lrtest

namespace {

using lrtest::Kind;
using lrtest::model;
using lrtest::Slave;

/// 1. max(1 ms, 500 bit periods) of the slowest device's rate.
void stuckThreshold() {
    Case const testcase{"stuck threshold: 500 bit periods, at least 1 ms, of the slowest rate"};
    checkEq(lrtest::Recovery<100, lrtest::Config<400'000>>::kStuckThreshold.count(),
            1250,
            "400 kHz: 1.25 ms");
    checkEq(lrtest::Recovery<101, lrtest::Config<100'000>>::kStuckThreshold.count(),
            5000,
            "100 kHz: 5 ms");
    checkEq(lrtest::Recovery<102, lrtest::Config<1'000'000>>::kStuckThreshold.count(),
            1000,
            "1 MHz: the 1 ms floor");
    checkEq(lrtest::Recovery<103, lrtest::SlowestConfig<400'000, 100'000>>::kStuckThreshold.count(),
            5000,
            "minBaudRate wins over baudRate");
}

/// 2. SDA held on an idle bus: not stuck before the threshold, stuck at it, one recovery begun.
void idleStuckAtTheThreshold() {
    Case const testcase{"SDA held on an idle bus: stuck exactly at the threshold"};
    using R = lrtest::Recovery<2>;
    lrtest::reset(Slave::dead);
    Log::reset();
    lrtest::startClock();
    check(!R::checkBusStuck(FakeClock::now()), "first low look starts the timer");
    FakeClock::advance(R::kStuckThreshold - 1us);
    check(!R::checkBusStuck(FakeClock::now()), "not stuck 1 us before the threshold");
    checkEq(R::idleStuck(), 0, "no idle stuck yet");
    FakeClock::advance(1us);
    check(R::checkBusStuck(FakeClock::now()), "stuck at the threshold: recovery begun");
    checkEq(R::idleStuck(), 1, "idleStuck() 1");
    checkEq(R::recoveries(), 1, "recoveries() 1");
    checkEq(Log::warnings, 1, "one warning line");
    check(R::isActive(), "the sequence runs");
    check(!R::checkBusStuck(FakeClock::now()), "the timer starts again after a detection");
    checkEq(R::idleStuck(), 1, "no second detection on the next look");
    R::resetState();

    // A high SDA in between restarts it too.
    using H = lrtest::Recovery<3>;
    lrtest::reset(Slave::dead);
    check(!H::checkBusStuck(FakeClock::now()), "low");
    FakeClock::advance(H::kStuckThreshold / 2);
    model.sda.held = false;
    check(!H::checkBusStuck(FakeClock::now()), "high: timer cleared");
    model.sda.held = true;
    check(!H::checkBusStuck(FakeClock::now()), "low again: timer restarts");
    FakeClock::advance(H::kStuckThreshold - 1us);
    check(!H::checkBusStuck(FakeClock::now()), "not a threshold since it went low again");
    checkEq(H::recoveries(), 0, "no recovery");
}

/// 3. A transfer start (noteBusFree) between two low looks restarts the stuck timer.
void noteBusFreeRestartsTheTimer() {
    Case const testcase{"noteBusFree() between two idle looks restarts the stuck timer"};
    using R = lrtest::Recovery<4>;
    lrtest::reset(Slave::dead);
    lrtest::startClock();
    check(!R::checkBusStuck(FakeClock::now()), "low look in a transfer's tail");
    R::noteBusFree();
    FakeClock::advance(R::kStuckThreshold);
    check(!R::checkBusStuck(FakeClock::now()),
          "a threshold later, but a transfer started in between");
    checkEq(R::recoveries(), 0, "no recovery");
    checkEq(R::idleStuck(), 0, "no idle stuck");
    FakeClock::advance(R::kStuckThreshold);
    check(R::checkBusStuck(FakeClock::now()), "a full threshold low with no transfer: stuck");
    checkEq(R::recoveries(), 1, "then one recovery");
    R::resetState();
}

/// 4. The sequence on a slave that lets go after three clocks.
void releasedAfterThreeClocks() {
    Case const testcase{"slave lets go after 3 clocks: abort, takeover, 3 pulses, STOP, reinit"};
    using R = lrtest::Recovery<5>;
    lrtest::reset(Slave::releaseAfter, 3);
    model.sda.latch = true;   // OUT latches the I2C block left high
    model.scl.latch = true;
    lrtest::startClock();
    auto const t0 = lrtest::nowUs();
    R::begin();
    checkEq(R::recoveries(), 1, "recoveries() 1 after begin()");
    check(!model.events.empty() && model.events.front().kind == Kind::abort,
          "softAbortRequest applied first");
    check(R::tick(FakeClock::now()) == R::TickResult::busy, "busy while the abort settles");
    auto const run = lrtest::runToIdle<R>();

    auto const clearSda = lrtest::firstIndex(Kind::clear, 0);
    auto const clearScl = lrtest::firstIndex(Kind::clear, 1);
    auto const inSda    = lrtest::firstIndex(Kind::makeInput, 0);
    auto const inScl    = lrtest::firstIndex(Kind::makeInput, 1);
    check(clearSda >= 0 && clearScl >= 0 && inSda >= 0 && inScl >= 0,
          "takeover clears and releases both pins");
    check(clearSda < inSda && clearScl < inSda && clearSda < inScl && clearScl < inScl,
          "latches cleared before the pins are released");
    if(clearSda >= 0) {
        checkEq(model.events[static_cast<std::size_t>(clearSda)].us - t0,
                100,
                "takeover 100 us after the abort");
    }

    auto const lows  = lrtest::eventsOf(Kind::makeOutput, 1);
    auto const highs = lrtest::eventsOf(Kind::makeInput, 1);
    checkEq(lows.size(), 3U, "three SCL pulses");
    checkEq(model.risingEdges, 3, "three rising edges");
    checkEq(R::clocksLeft(), 6, "clocksLeft() 6 of 9");
    for(std::size_t i = 0; i < lows.size() && i + 1 < highs.size(); ++i) {
        // highs[0] is the takeover's release
        checkEq(highs[i + 1].us - lows[i].us, 20, "SCL low for 20 us");
        if(i + 1 < lows.size()) {
            checkEq(lows[i + 1].us - highs[i + 1].us, 20, "SCL high for 20 us");
        }
    }

    auto const sdaLow  = lrtest::eventsOf(Kind::makeOutput, 0);
    auto const sdaRels = lrtest::eventsOf(Kind::makeInput, 0);
    checkEq(sdaLow.size(), 1U, "SDA taken low once for the STOP");
    checkEq(sdaRels.size(), 2U, "SDA released at takeover and for the STOP");
    if(!sdaLow.empty() && !highs.empty()) {
        checkEq(sdaLow[0].us - highs.back().us, 20, "the STOP 20 us after the last clock");
    }
    if(!sdaLow.empty() && sdaRels.size() == 2) {
        checkEq(sdaRels[1].us - sdaLow[0].us, 20, "SDA low for 20 us, then released");
    }
    checkEq(model.stopsSeen, 1, "the slave saw one STOP");
    checkEq(R::forcedStops(), 0, "no forced STOP");
    checkEq(run.needsReinit, 1, "needsReinit once");
    check(run.idleAfter >= 1, "then idle");
    auto const back = lrtest::lastIndex(Kind::pinsBack, -1);
    check(back == static_cast<int>(model.events.size()) - 1, "pins given back to the block last");
    if(!sdaRels.empty()) {
        check(run.endUs - sdaRels.back().us >= 20, "reinit after the STOP's 20 us");
    }
    check(model.sda.high() && lrtest::reads(1), "both lines high at the end");
    checkEq(model.pushedHighs, 0, "the pad never pushed high (case 8)");
    check(!R::isActive(), "idle");
    // The busy ticks: 100 us abort + (3 pulses x 2 + STOP 2) x 20 us of waits, ticked every 1 us.
    check(run.busy >= 99 + 8 * 20, "busy while every wait is armed");
}

/// 5. A slave that never lets go of SDA until a STOP: 9 pulses, the STOP fails, then the forced STOP.
void neverReleasedGetsTheForcedStop() {
    Case const testcase{"slave never lets go: 9 clocks, STOP, forced STOP, latch low again"};
    using R = lrtest::Recovery<6>;
    lrtest::reset(Slave::untilStop);
    lrtest::startClock();
    R::begin();
    auto const run = lrtest::runToIdle<R>();
    checkEq(lrtest::eventsOf(Kind::makeOutput, 1).size(), 9U, "nine SCL pulses");
    checkEq(R::clocksLeft(), 0, "clocksLeft() 0: SDA never came back");
    checkEq(R::forcedStops(), 1, "forcedStops() 1");
    auto const pushes = lrtest::eventsOf(Kind::outputHigh, 0);
    checkEq(pushes.size(), 1U, "SDA pushed high once");
    auto const stopLow = lrtest::eventsOf(Kind::makeOutput, 0);
    check(!stopLow.empty() && !pushes.empty() && pushes[0].us - stopLow[0].us == 40,
          "forced 20 us after the STOP's release (which was 20 us after SDA low)");
    auto const clears = lrtest::eventsOf(Kind::clear, 0);
    checkEq(clears.size(), 2U, "SDA latch cleared at takeover and after the forced STOP");
    if(!clears.empty() && !pushes.empty()) {
        checkEq(clears.back().us - pushes[0].us, 20, "pushed high for 20 us");
    }
    check(!model.sda.latch, "SDA latch low again");
    check(!model.sda.output, "SDA released");
    check(model.sda.high(), "SDA free");
    checkEq(model.pushedHighs, 1, "pushed high only in the forced STOP (case 8)");
    checkEq(run.needsReinit, 1, "needsReinit once");
}

/// 5b. The re-taking slave: lets go at the first clock, takes SDA again when the master pulls it low for the STOP.
void retakingSlaveGetsTheForcedStop() {
    Case const testcase{
      "slave lets go for one clock and takes SDA again: the forced STOP frees it"};
    using R = lrtest::Recovery<7>;
    lrtest::reset(Slave::retake);
    lrtest::startClock();
    R::begin();
    auto const run = lrtest::runToIdle<R>();
    checkEq(lrtest::eventsOf(Kind::makeOutput, 1).size(),
            1U,
            "one SCL pulse: SDA was high after it");
    checkEq(R::clocksLeft(), 8, "clocksLeft() 8");
    checkEq(R::forcedStops(), 1, "the plain STOP did not free it: forced");
    check(model.sda.high(), "SDA free at the end");
    checkEq(model.pushedHighs, 1, "pushed high only in the forced STOP");
    checkEq(run.needsReinit, 1, "needsReinit once");
}

/// 6. SCL held as well: nothing a master can do; no forced STOP into a held clock.
void sclHeldGetsNoForcedStop() {
    Case const testcase{"SCL held too: no forced STOP"};
    using R = lrtest::Recovery<8>;
    lrtest::reset(Slave::untilStop);
    model.sclHeld = true;
    lrtest::startClock();
    R::begin();
    auto const run = lrtest::runToIdle<R>();
    checkEq(model.risingEdges, 0, "no clock got through");
    checkEq(R::forcedStops(), 0, "no forced STOP");
    checkEq(lrtest::eventsOf(Kind::outputHigh, 0).size(), 0U, "SDA never pushed high");
    checkEq(model.pushedHighs, 0, "the pad never pushed high (case 8)");
    checkEq(run.needsReinit, 1, "still ends in needsReinit");
}

/// 7. Backoff between automatic recoveries of a bus nothing frees: 100 ms doubling to 2 s; back to 100 ms once SDA
/// reads high.
void backoffDoublesAndResets() {
    Case const testcase{"backoff: 100 ms, doubling, capped at 2 s, reset once SDA reads high"};
    using R = lrtest::Recovery<9>;
    lrtest::reset(Slave::dead);
    lrtest::startClock();
    std::vector<std::int64_t> began{};
    std::vector<std::int64_t> backoffs{};
    auto const                start = lrtest::nowUs();
    while(lrtest::nowUs() - start < 12'000'000 && began.size() < 10) {
        if(R::checkBusStuck(FakeClock::now())) {
            began.push_back(lrtest::nowUs());
            backoffs.push_back(R::backoff().count());
            R::resetState();   // as if the sequence ran (a dead slave: nothing changes)
        }
        FakeClock::advance(100us);
    }
    checkEq(began.size(), 10U, "ten recoveries in 12 s");
    std::array<std::int64_t, 9> const holdMs{100, 200, 400, 800, 1600, 2000, 2000, 2000, 2000};
    std::array<std::int64_t, 10> const
      nextMs{200, 400, 800, 1600, 2000, 2000, 2000, 2000, 2000, 2000};
    for(std::size_t i = 0; i < backoffs.size(); ++i) {
        checkEq(backoffs[i], nextMs[i], "backoff() after it");
    }
    for(std::size_t i = 0; i + 1 < began.size(); ++i) {
        auto const gap  = began[i + 1] - began[i];
        auto const hold = holdMs[i] * 1000;
        // held off exactly `hold`, then the next detection: within one threshold plus the polling step
        check(gap >= hold && gap <= hold + R::kStuckThreshold.count() + 200,
              "recovery held off by the backoff");
    }
    check(R::idleStuck() > R::recoveries(), "detections held off are counted in idleStuck() only");
    model.sda.held = false;
    check(!R::checkBusStuck(FakeClock::now()), "SDA high");
    checkEq(R::backoff().count(), 100, "backoff back to 100 ms");
}

}   // namespace

int main() {
    stuckThreshold();
    idleStuckAtTheThreshold();
    noteBusFreeRestartsTheTimer();
    releasedAfterThreeClocks();
    neverReleasedGetsTheForcedStop();
    retakingSlaveGetsTheForcedStop();
    sclHeldGetsNoForcedStop();
    backoffDoublesAndResets();
    return finish();
}
