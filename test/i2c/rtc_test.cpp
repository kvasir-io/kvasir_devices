/// Real-time clocks with more than a time register: the RV-8803's bring-up, flags, set
/// sequence, offset, CLKOUT and event time stamp, against a model of the part whose flag
/// register clears on a written 0.
#include "Harness.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Rv8803.hpp>
#include <span>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

using Rv    = Chips::Rv8803<>;
using Cs    = Rv::Centiseconds;
using Frame = std::array<std::byte, 16>;

/// A register file at 0x32 whose flag register (1Eh) keeps a bit only where the write has a
/// 1 (3.7: a flag is cleared by writing 0 to it, a 1 leaves it).
struct Rv8803Model {
    RegisterModel<1> reg{0x32};

    FakeBusResult operator()(std::uint8_t               addr,
                             std::span<std::byte const> sent,
                             std::span<std::byte>       recv) {
        if(addr == 0x32 && sent.size() == 2 && static_cast<std::uint8_t>(sent[0]) == 0x1E) {
            auto const kept = static_cast<std::uint8_t>(
              reg.word(0x1E) & static_cast<std::uint8_t>(sent[1]) & 0x3FU);
            std::array<std::byte, 2> const masked{sent[0], std::byte{kept}};
            return reg(addr, masked, recv);
        }
        return reg(addr, sent, recv);
    }
};

/// The Clock burst: 10h..1Eh (15 bytes), then the re-read of 11h.
constexpr Frame clockFrame(std::uint8_t cc,
                           std::uint8_t ss,
                           std::uint8_t mm,
                           std::uint8_t hh,
                           std::uint8_t weekday,
                           std::uint8_t date,
                           std::uint8_t month,
                           std::uint8_t year,
                           std::uint8_t flags) {
    Frame f{};
    f[0]  = std::byte{cc};
    f[1]  = std::byte{ss};
    f[2]  = std::byte{mm};
    f[3]  = std::byte{hh};
    f[4]  = std::byte{weekday};
    f[5]  = std::byte{date};
    f[6]  = std::byte{month};
    f[7]  = std::byte{year};
    f[14] = std::byte{flags};
    f[15] = std::byte{ss};
    return f;
}

constexpr Cs at(int h,
                int m,
                int s,
                int cc) {
    return Cs{((h * 60 + m) * 60 + s) * 100 + cc};
}

// -- the decode, at compile time -------------------------------------------------------------

// Wed 2026-09-16 23:59:30.42 (weekday bit 3), V1F and V2F set by a power-on reset
static_assert([] {
    using namespace std::chrono;
    auto const got
      = Rv::Clock::decode(Bytes{clockFrame(0x42, 0x30, 0x59, 0x23, 0x08, 0x16, 0x09, 0x26, 0x03)});
    auto const& t = got.value;
    return isOk(got) && t.date == 2026y / September / 16d && t.weekday == Wednesday
        && t.weekday == weekday{local_days{2026y / September / 16d}} && t.time == at(23, 59, 30, 42)
        && t.hms().hours() == 23h && t.hms().subseconds() == Cs{42} && !t.valid && !t.compensated;
}());

// the same with the flags clear: valid, compensated; Sunday is bit 0, Saturday bit 6
static_assert([] {
    using namespace std::chrono;
    auto const sun
      = Rv::Clock::decode(Bytes{clockFrame(0x00, 0x00, 0x00, 0x00, 0x01, 0x20, 0x09, 0x26, 0x00)});
    auto const sat
      = Rv::Clock::decode(Bytes{clockFrame(0x99, 0x59, 0x59, 0x23, 0x40, 0x31, 0x12, 0x99, 0x00)});
    return isOk(sun) && sun.value.valid && sun.value.compensated && sun.value.weekday == Sunday
        && sun.value.time == Cs{0} && isOk(sat) && sat.value.weekday == Saturday
        && sat.value.date == 2099y / December / 31d && sat.value.time == at(23, 59, 59, 99);
}());

// V2F alone: the time is not valid; V1F alone: valid but not compensated
static_assert([] {
    auto const v2
      = Rv::Clock::decode(Bytes{clockFrame(0x00, 0x10, 0x00, 0x12, 0x02, 0x01, 0x01, 0x25, 0x02)});
    auto const v1
      = Rv::Clock::decode(Bytes{clockFrame(0x00, 0x10, 0x00, 0x12, 0x02, 0x01, 0x01, 0x25, 0x01)});
    return isOk(v2) && !v2.value.valid && v2.value.compensated && isOk(v1) && v1.value.valid
        && !v1.value.compensated;
}());

// not BCD, out of range, a date that does not exist, a weekday that is not one-hot: rejected
static_assert([] {
    auto const reject = [](Frame const& f) { return isReject(Rv::Clock::decode(Bytes{f})); };
    return reject(clockFrame(0x00, 0x5A, 0x00, 0x12, 0x02, 0x01, 0x01, 0x25, 0x00))   // seconds 5A
        && reject(
             clockFrame(0xFF, 0x7F, 0x7F, 0x3F, 0x7F, 0x3F, 0x1F, 0xFF, 0x00))   // an absent part
        && reject(clockFrame(0x00, 0x00, 0x60, 0x12, 0x02, 0x01, 0x01, 0x25, 0x00))   // minute 60
        && reject(clockFrame(0x00, 0x00, 0x00, 0x24, 0x02, 0x01, 0x01, 0x25, 0x00))   // hour 24
        && reject(clockFrame(0x00, 0x00, 0x00, 0x12, 0x02, 0x31, 0x02, 0x25, 0x00))   // 31 February
        && reject(clockFrame(0x00, 0x00, 0x00, 0x12, 0x02, 0x00, 0x01, 0x25, 0x00))   // day 0
        && reject(
             clockFrame(0x00, 0x00, 0x00, 0x12, 0x03, 0x01, 0x01, 0x25, 0x00))   // two weekdays
        && reject(clockFrame(0x00, 0x00, 0x00, 0x12, 0x00, 0x01, 0x01, 0x25, 0x00))    // no weekday
        && reject(clockFrame(0x00, 0x00, 0x00, 0x12, 0x80, 0x01, 0x01, 0x25, 0x00));   // bit 7
}());

// 4.12.2: the seconds re-read after the burst changed, so the burst may be torn: read again
static_assert([] {
    auto f = clockFrame(0x99, 0x59, 0x59, 0x12, 0x02, 0x01, 0x01, 0x25, 0x00);
    f[15]  = std::byte{0x00};
    return isRetry(Rv::Clock::decode(Bytes{f}));
}());

// the SetTime sequence: RESET, the seven registers from 11h, the voltage flags, RESET released
static_assert([] {
    using namespace std::chrono;
    std::array<std::byte, 7> buf{};
    auto const t     = Rv::Time::at(local_days{2025y / February / 1d}, at(12, 30, 15, 99));
    auto const steps = Rv::SetTime::encode(t, buf);
    return steps[0].reg == 0x1F && steps[0].count == 1 && steps[0].bytes[0] == 0x01
        && steps[1].reg == 0x11 && steps[1].fromBuffer && steps[1].count == 7
        && steps[2].reg == 0x1E && steps[2].bytes[0] == 0x3C && steps[3].reg == 0x1F
        && steps[3].bytes[0] == 0x00 && buf[0] == std::byte{0x15} && buf[1] == std::byte{0x30}
        && buf[2] == std::byte{0x12} && buf[3] == std::byte{0x40}   // 2025-02-01 was a Saturday
        && buf[4] == std::byte{0x01} && buf[5] == std::byte{0x02} && buf[6] == std::byte{0x25};
}());

// EIE asked for: control keeps it through the set sequence
static_assert([] {
    std::array<std::byte, 7> buf{};
    auto const               steps
      = Chips::Rv8803<Chips::Rv8803Detail::ClockOut::khz32_768,
                      Chips::Rv8803Detail::EventInterrupt::on>::SetTime::encode({}, buf);
    return steps[0].bytes[0] == 0x05 && steps[3].bytes[0] == 0x04;
}());

// OFFSET: 5.4's examples, +5 is 000101 and -21 is 101011; beyond -32 .. +31 clamped
static_assert([] {
    std::array<std::byte, 1> b{};
    auto const               put = [&](std::int8_t v) {
        static_cast<void>(Rv::Offset::encode(v, b));
        return static_cast<std::uint8_t>(b[0]);
    };
    return put(5) == 0x05 && put(-21) == 0x2B && put(31) == 0x1F && put(-32) == 0x20
        && put(40) == 0x1F && put(-100) == 0x20;
}());

// FD and the event control byte
static_assert([] {
    std::array<std::byte, 1> b{};
    static_cast<void>(Rv::ClockOut::encode(Rv::ClockOutRate::hz1, b));
    auto const fd = static_cast<std::uint8_t>(b[0]);
    static_cast<void>(Rv::EventControl::encode({.capture = Rv::Capture::on,
                                                .level   = Rv::EventLevel::high,
                                                .filter  = Rv::EventFilter::ms15_6},
                                               b));
    return fd == 0x08 && static_cast<std::uint8_t>(b[0]) == 0xE0;
}());

// the event stamp: EVF set is an event; a capture that moved is one whose flag the clear raced;
// neither is unchanged; a first request without EVF has nothing to compare with
static_assert([] {
    using Stamp        = Rv::Event::Sample;
    auto const flagged = Rv::Event::decode(Bytes{frame(0x04, 0x25, 0x12)}, Stamp{});
    auto const moved   = Rv::Event::decode(Bytes{frame(0x00, 0x50, 0x13)}, Stamp{Cs{1225}});
    auto const same    = Rv::Event::decode(Bytes{frame(0x00, 0x25, 0x12)}, Stamp{Cs{1225}});
    auto const first   = Rv::Event::decode(Bytes{frame(0x00, 0x25, 0x12)}, Stamp{});
    auto const bad     = Rv::Event::decode(Bytes{frame(0x04, 0x2A, 0x12)}, Stamp{});
    return isOk(flagged) && flagged.value.stamp == Cs{1225} && isOk(moved)
        && moved.value.stamp == Cs{1350} && isUnchanged(same) && isUnchanged(first)
        && isReject(bad);
}());

// -- the part on the wire ----------------------------------------------------------------------

void rv8803() {
    testCase("RV-8803: bring-up");
    fresh();
    Rv8803Model m{};
    m.reg.set(0x10,
              {0x42, 0x30, 0x59, 0x23, 0x08, 0x16, 0x09, 0x26});   // Wed 2026-09-16 23:59:30.42
    m.reg.set(0x1E, {0x07, 0x00});                                 // EVF, V2F, V1F; control 0
    m.reg.set(0x2C, {0x05});
    m.reg.readOnly   = {0x10};
    FakeBus::respond = std::ref(m);
    Dev<Rv> d{};
    check(runUntil(d, [&] { return d.samples<Rv::Clock>() == 1; }, 200ms), "first time read");
    check(d.identified(), "the read-as-zero bits are zero");
    check(d.state().timeLost && d.state().compensationStopped && !d.state().wasStopped,
          "State recorded V2F and V1F before anything was cleared");
    checkEq(d.state().offset, 5, "and the offset the part holds");
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0x1E, 0x03}, {0x1F, 0x00}, {0x1D, 0x00}},
          "UF TF AF EVF cleared with V1F V2F written 1, RESET released, then ClockOut's Initial");
    checkEq(m.reg.word(0x1E), 0x03U, "EVF cleared, the voltage flags kept");
    {
        std::vector<std::vector<std::uint8_t>> reads;
        std::vector<std::size_t>               lengths;
        for(auto const& t : FakeBus::log) {
            if(t.isRead()) {
                reads.push_back(t.sent);
                lengths.push_back(t.recvLen);
            }
        }
        check(reads.size() >= 4 && reads[0] == std::vector<std::uint8_t>{0x1E} && lengths[0] == 2
                && reads[1] == std::vector<std::uint8_t>{0x2C} && lengths[1] == 1
                && reads[2] == std::vector<std::uint8_t>{0x10} && lengths[2] == 15
                && reads[3] == std::vector<std::uint8_t>{0x11} && lengths[3] == 1,
              "flags and control, offset; then the 15-byte burst and the seconds re-read");
    }
    {
        using namespace std::chrono;
        auto const& t = d.latest<Rv::Clock>();
        check(t.date == 2026y / September / 16d && t.weekday == Wednesday, "date and weekday");
        checkEq(t.time.count(), at(23, 59, 30, 42).count(), "time of day to the hundredth");
        check(!t.valid, "V2F still set: the time is not valid");
    }

    testCase("RV-8803: V2F survives a second bring-up");
    {
        fresh();
        FakeBus::respond = std::ref(m);
        Dev<Rv> again{};
        check(runUntil(again, [&] { return again.samples<Rv::Clock>() == 1; }, 200ms), "read");
        check(again.state().timeLost && !again.latest<Rv::Clock>().valid,
              "a bring-up does not make an undefined time valid");
    }

    testCase("RV-8803: SetTime");
    fresh();
    FakeBus::respond = std::ref(m);
    Dev<Rv> c{};
    check(runUntil(c, [&] { return c.samples<Rv::Clock>() == 1; }, 200ms), "up");
    auto const from = FakeBus::log.size();
    {
        using namespace std::chrono;
        c.set<Rv::SetTime>(Rv::Time::at(local_days{2025y / February / 1d}, at(12, 30, 15, 0)));
    }
    check(runUntil(c, [&] { return c.writes<Rv::SetTime>() == 1; }, 200ms), "written");
    check(writes(from)
            == std::vector<std::vector<std::uint8_t>>{{0x1F, 0x01}, {0x11, 0x15, 0x30, 0x12, 0x40, 0x01, 0x02, 0x25}, {0x1E, 0x3C}, {0x1F, 0x00}},
          "RESET, seconds..year from 11h, V1F and V2F cleared, RESET released");
    checkEq(m.reg.word(0x1E), 0x00U, "the voltage flags are clear");
    check(!c.state().timeLost, "applied(): the time is no longer lost");
    auto const seq = c.seq<Rv::Clock>();
    check(runUntil(c, [&] { return c.seq<Rv::Clock>() > seq; }, 2s), "read after the set");
    {
        using namespace std::chrono;
        auto const& t = c.latest<Rv::Clock>();
        check(t.valid && t.compensated, "valid now");
        check(t.date == 2025y / February / 1d && t.weekday == Saturday, "the date that was set");
    }
    runFor(c, 3s);
    checkEq(c.writes<Rv::SetTime>(), 1U, "a set time is not replayed");

    testCase("RV-8803: offset, CLKOUT and the event stamp");
    auto const mark = FakeBus::log.size();
    c.set<Rv::Offset>(std::int8_t{-21});
    check(runUntil(c, [&] { return c.writes<Rv::Offset>() == 1; }, 200ms), "offset written");
    check(hasWrite({0x2C, 0x2B}, mark), "-21 steps as six-bit two's complement");
    checkEq(c.state().offset, -21, "applied()");
    c.set<Rv::ClockOut>(Rv::ClockOutRate::hz1);
    check(runUntil(c, [&] { return c.writes<Rv::ClockOut>() == 2; }, 200ms), "FD written");
    check(hasWrite({0x1D, 0x08}, mark) && c.state().clockOut == Rv::ClockOutRate::hz1,
          "1 Hz on CLKOUT");

    m.reg.set(0x20, {0x25, 0x12});   // 12.25 s captured
    m.reg.set(0x1E, {0x04});         // EVF
    auto const ticket = c.request<Rv::Event>();
    check(runUntil(
            c,
            [&] { return c.answer<Rv::Event>(ticket) != Answer::pending; },
            200ms),
          "answered");
    check(c.answer<Rv::Event>(ticket) == Answer::ok, "an event");
    checkEq(c.latest<Rv::Event>().stamp.count(), 1225, "its stamp");
    checkEq(m.reg.word(0x1E), 0x00U, "EVF cleared by the request");
    auto const quiet = c.request<Rv::Event>();
    check(runUntil(
            c,
            [&] { return c.answer<Rv::Event>(quiet) != Answer::pending; },
            200ms),
          "answered");
    check(c.answer<Rv::Event>(quiet) == Answer::unchanged, "no new event");

    testCase("RV-8803: a NAK in the update window is retried, not a fault");
    {
        fresh();
        int  nakEvery = 0;
        auto inner    = std::ref(m);
        FakeBus::respond
          = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
                // every fifth transaction lands in the 61 us window once
                if(++nakEvery % 5 == 0) { return FakeBus::Result::notAcknowledged; }
                return inner(addr, sent, recv);
            };
        Dev<Rv> w{};
        check(runUntil(w, [&] { return w.samples<Rv::Clock>() >= 3; }, 4s), "reads go on");
        checkEq(w.errors(), 0U, "no NAK counted as an error");
        check(w.wakeRetries() >= 1, "they were retried instead");
    }

    testCase("RV-8803: not the part");
    fresh();
    Rv8803Model other{};
    other.reg.set(0x1E, {0xC0, 0x00});   // flag bits 7:6 read as 1
    FakeBus::respond = std::ref(other);
    Dev<Rv> o{};
    runFor(o, 100ms);
    check(!o.identified() && o.unidentified() >= 1, "refused");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    rv8803();
    return finish();
}
