/// Delta-sigma ADCs with a register file of their own: the NAU7802 against a model of the part
/// that powers up, calibrates and converts.
#include "Harness.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/All.hpp>
#include <span>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

using Nau      = Chips::Nau7802<>;
namespace NauD = Chips::Nau7802Detail;

/// The NAU7802 on the wire: RR resets the file, PUD raises PUR, CALS completes after
/// `calBusyReads` reads of CTRL2 (and sets CAL_ERR when `calError`), a read of PU_CTRL shows CR
/// while `converting`, and addressing ADCO takes the result and clears it.
struct Nau7802Model {
    RegisterModel<1> reg{0x2A};
    bool             converting{true};
    bool             calError{false};
    int              calBusyReads{0};
    int              calibrations{0};
    int              busy{0};
    std::uint8_t     revision{0x0F};

    Nau7802Model() {
        reg.readOnly = {0x12, 0x13, 0x14, 0x1F};
        reset();
        reg.onWrite = [this](std::uint32_t r, RegisterModel<1>::Reg const& v) { written(r, v[0]); };
    }

    void reset() {
        auto const adco = std::array{reg.get(0x12), reg.get(0x13), reg.get(0x14)};
        reg.mem.clear();
        reg.mem[0x12] = adco[0];
        reg.mem[0x13] = adco[1];
        reg.mem[0x14] = adco[2];
        reg.set(0x06, {0x00, 0x80, 0x00, 0x00});
        reg.set(0x0D, {0x00, 0x80, 0x00, 0x00});
        reg.set(0x1F, {revision});
    }

    void code(std::uint32_t c) {
        reg.set(0x12,
                {static_cast<std::uint8_t>(c >> 16),
                 static_cast<std::uint8_t>((c >> 8) & 0xFF),
                 static_cast<std::uint8_t>(c & 0xFF)});
    }

    void written(std::uint32_t r,
                 std::uint8_t  v) {
        if(r == 0x00) {
            if((v & 0x01U) != 0) {
                reset();
                reg.mem[0x00][0] = 0x01;
                return;
            }
            // PUR follows PUD; CR is the model's
            reg.mem[0x00][0]
              = static_cast<std::uint8_t>((v & 0xD7U) | ((v & 0x02U) != 0 ? 0x08U : 0U));
        }
        if(r == 0x02 && (v & 0x04U) != 0) {
            if(busy > 0) {
                // 11.3: a write to CALS while one runs is ignored
                reg.mem[0x02][0] = static_cast<std::uint8_t>((v & 0xF3U) | 0x04U);
                return;
            }
            ++calibrations;
            busy = calBusyReads;
            // an internal offset calibration of the selected channel
            reg.set((v & 0x80U) != 0 ? 0x0A : 0x03, {0x00, 0x01, 0x23});
            reg.mem[0x02][0] = static_cast<std::uint8_t>((v & 0xF3U) | (busy > 0 ? 0x04U : 0U)
                                                         | (busy == 0 && calError ? 0x08U : 0U));
        }
    }

    FakeBusResult operator()(std::uint8_t               addr,
                             std::span<std::byte const> sent,
                             std::span<std::byte>       recv) {
        auto const ptr = sent.empty() ? reg.pointer : static_cast<std::uint8_t>(sent[0]);
        if(!recv.empty() && ptr == 0x00) {
            auto& pu = reg.mem[0x00][0];
            pu       = static_cast<std::uint8_t>(converting ? (pu | 0x20U) : (pu & ~0x20U));
        }
        if(!recv.empty() && ptr == 0x02 && busy > 0 && --busy == 0) {
            reg.mem[0x02][0]
              = static_cast<std::uint8_t>((reg.mem[0x02][0] & ~0x04U) | (calError ? 0x08U : 0U));
        }
        auto const res = reg(addr, sent, recv);
        if(!recv.empty() && ptr == 0x12) { reg.mem[0x00][0] &= static_cast<std::uint8_t>(~0x20U); }
        return res;
    }
};

bool readOf(std::uint8_t reg,
            std::size_t  n) {
    for(auto const& t : FakeBus::log) {
        if(t.isRead() && t.sent == std::vector<std::uint8_t>{reg} && t.recvLen == n) {
            return true;
        }
    }
    return false;
}

// -- pure decode ------------------------------------------------------------------------------

static_assert([] {
    Nau::State st{};   // 3.3 V LDO, x128, channel 1
    auto const pos = Nau::Conversion::decode(Bytes{frame(0x20, 0x40, 0x00, 0x00)}, st);
    auto const neg = Nau::Conversion::decode(Bytes{frame(0x20, 0xFF, 0xFF, 0xFF)}, st);
    auto const top = Nau::Conversion::decode(Bytes{frame(0x20, 0x7F, 0xFF, 0xFF)}, st);
    auto const low = Nau::Conversion::decode(Bytes{frame(0x20, 0x80, 0x00, 0x00)}, st);
    return pos.code == 4'194'304 && neg.code == -1 && top.code == 8'388'607
        && low.code == -8'388'608 && equal(pos.voltage(), 6445)   // quarter scale: 3.3 V / 4 / 128
        && equal(top.voltage(), 12890)    // +0.5 x 3.3 V / 128, one code short
        && equal(low.voltage(), -12890)   // -12.890625 mV, truncated towards zero
        && pos.channel == NauD::Channel::ch1 && equal(pos.reference, 3300);
}());

static_assert([] {
    Nau::State st{};
    st.config.gain    = NauD::Gain::x1;
    st.config.ldo     = NauD::Ldo::v4_5;
    st.config.channel = NauD::Channel::ch2;
    auto const s      = Nau::Conversion::decode(Bytes{frame(0x20, 0x40, 0x00, 0x00)}, st);
    // quarter scale at x1 against 4.5 V: 4.5 V / 4 = 1.125 V
    return equal(s.voltage(), 1'125'000) && s.channel == NauD::Channel::ch2
        && equal(s.reference, 4500);
}());

static_assert([] {
    using Ext = Chips::Nau7802<NauD::Ldo::external,
                               NauD::Gain::x2,
                               NauD::Rate::sps10,
                               NauD::Channel::ch1,
                               NauD::FilterCap::absent,
                               Units::milliVolt(5000)>;
    Ext::State st{};
    auto const s = Ext::Conversion::decode(Bytes{frame(0x20, 0xC0, 0x00, 0x00)}, st);
    // -2^22 at x2 against an external 5 V: -5 V / 8
    return equal(s.voltage(), -625'000) && Ext::Conversion::Period == 105ms
        && NauD::puCtrl(NauD::Ldo::external, NauD::Cycle::running) == 0x16
        && NauD::pgaPwr(NauD::FilterCap::absent, NauD::Channel::ch1) == 0;
}());

static_assert([] {
    auto const s = Nau::Calibration::decode(Bytes{frame(0x08,
                                                        0xFF,
                                                        0xFF,
                                                        0xFB,
                                                        0x00,
                                                        0x81,
                                                        0x23,
                                                        0x45,
                                                        0x00,
                                                        0x01,
                                                        0x23,
                                                        0x00,
                                                        0x80,
                                                        0x00,
                                                        0x00)});
    return s.calError && !s.calibrating && s.channels[0].offset == -5
        && s.channels[0].gain == 0x0081'2345U && s.channels[1].offset == 0x123
        && s.channels[1].gain == 0x0080'0000U;
}());

// the register codes against 11.1 .. 11.3, and the periods the rates ask for
static_assert(NauD::ctrl1(NauD::Ldo::v3_3,
                          NauD::Gain::x128)
              == 0x27);
static_assert(NauD::ctrl1(NauD::Ldo::v2_4,
                          NauD::Gain::x1)
              == 0x38);
static_assert(NauD::ctrl2(NauD::Channel::ch2,
                          NauD::Rate::sps320,
                          NauD::CalMode::systemGain,
                          NauD::Calibration::start)
              == 0xF7);
static_assert(NauD::puCtrl(NauD::Ldo::v3_3,
                           NauD::Cycle::running)
              == 0x96);
static_assert(NauD::readPeriod(NauD::Rate::sps10) == 105ms
              && NauD::readPeriod(NauD::Rate::sps80) == 14ms
              && NauD::readPeriod(NauD::Rate::sps320) == 4ms);
static_assert(Nau::CalibrationWait == 204ms);

void bringUp() {
    testCase("NAU7802 bring-up");
    fresh();
    Nau7802Model m{};
    m.code(0x400000);
    FakeBus::respond = std::ref(m);
    Dev<Nau> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first conversion");
    check(d.identified(), "revision 1111");
    check(writes() == std::vector<std::vector<std::uint8_t>>{
                        {0x00, 0x01},   // RR
                        {0x00, 0x02},   // PUD
                        {0x01, 0x27},   // LDO 3.3 V, x128
                        {0x02, 0x30},   // channel 1, 80 SPS
                        {0x15, 0x30},   // chopper clock off
                        {0x1C, 0x80},   // PGA_CAP_EN
                        {0x00, 0x96},   // AVDDS, CS, PUA, PUD
                        {0x02, 0x34},   // CALS, internal offset
                      },
          "9.1's sequence and an internal calibration");
    check(readOf(0x1F, 1) && readOf(0x00, 1) && readOf(0x02, 1),
          "revision, PU_CTRL and CTRL2 read");
    checkEq(m.calibrations, 1, "one calibration");
    // the calibration is given its time before CTRL2 is read back
    {
        FakeClock::time_point calAt{}, readAt{};
        for(auto const& t : FakeBus::log) {
            if(t.sent == std::vector<std::uint8_t>{0x02, 0x34}) { calAt = t.at; }
            if(t.isRead() && t.sent == std::vector<std::uint8_t>{0x02}
               && readAt == FakeClock::time_point{})
            {
                readAt = t.at;
            }
        }
        check(readAt - calAt >= 204ms, "CalibrationWait before the read-back");
        FakeClock::time_point puaAt{};
        for(auto const& t : FakeBus::log) {
            if(t.sent == std::vector<std::uint8_t>{0x00, 0x96}) { puaAt = t.at; }
        }
        check(calAt - puaAt >= Nau::AnalogSettle && Nau::AnalogSettle >= 160ms,
              "7.3's TRDY -- five conversions and 100 ms -- between PUA and the calibration");
    }
    check(!d.state().calError && !d.state().calibrating, "CAL_ERR and CALS clear");
    check(readOf(0x12, 3), "ADCO in one burst");
    checkEq(d.latest().code, 4'194'304, "the model's code reached the decode");
    checkEq(d.latest().voltage(), 6445, "quarter scale at x128 against 3.3 V");
    if(failures != 0) { dump(); }
}

void rejects() {
    testCase("NAU7802 rejects");
    fresh();
    Nau7802Model m{};
    m.revision = 0x0E;
    m.reset();
    FakeBus::respond = std::ref(m);
    Dev<Nau> d{};
    runFor(d, 500ms);
    check(!d.identified() && !d.valid(), "a revision nibble other than 1111 is not this part");
    checkEq(d.state().revision, 0x0E, "the revision it read");
    check(!readOf(0x12, 3), "and nothing is converted");

    fresh();
    Nau7802Model e{};
    e.calError       = true;
    FakeBus::respond = std::ref(e);
    Dev<Nau> c{};
    runFor(c, 500ms);
    check(!c.identified() && c.state().calError, "a calibration error rejects the bring-up");
    runFor(c, 1500ms);
    check(e.calibrations >= 2, "and the next bring-up calibrates again");
    e.calError = false;
    check(runUntil(c, [&] { return c.valid(); }, 3s), "until one succeeds");
    if(failures != 0) { dump(); }
}

void runtime() {
    testCase("NAU7802 settings");
    fresh();
    Nau7802Model m{};
    m.code(0x400000);
    FakeBus::respond = std::ref(m);
    Dev<Nau> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first conversion");
    checkEq(d.period<Nau::Conversion>().count(), 14, "80 SPS");

    d.modify<Nau::Settings>([](auto& s) { s.gain = NauD::Gain::x1; });
    auto const from = FakeBus::log.size();
    check(runUntil(d, [&] { return !d.pending(); }, 200ms), "written");
    {
        // six conversions at the new setting go by before the part is read again
        FakeClock::time_point edge{}, next{};
        for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
            auto const& t = FakeBus::log[i];
            if(t.sent == std::vector<std::uint8_t>{0x00, 0x96}) { edge = t.at; }
        }
        runFor(d, 150ms);
        for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
            if(FakeBus::log[i].at > edge && next == FakeClock::time_point{}) {
                next = FakeBus::log[i].at;
            }
        }
        check(next - edge >= 6 * 12ms, "six conversions at 80 SPS before the next read");
    }
    check(writes(from) == std::vector<std::vector<std::uint8_t>>{
                            {0x01, 0x20}, {0x02, 0x30}, {0x1C, 0x80}, {0x00, 0x86}, {0x00, 0x96}},
          "CTRL1, CTRL2, PGA_PWR and a CS edge");
    auto const seq = d.seq();
    check(runUntil(d, [&] { return d.seq() > seq; }, 100ms), "sampled after it");
    checkEq(d.latest().voltage(), 825'000, "the same code at x1 is 128 times the voltage");
    check(d.latest().gain == NauD::Gain::x1, "and says so");

    {
        auto const n = d.samples<Nau::Conversion>();
        runFor(d, 1s);
        auto const perSecond = d.samples<Nau::Conversion>() - n;
        check(perSecond >= 68 && perSecond <= 72, "about 71 a second at 80 SPS");
    }
    d.modify<Nau::Settings>([](auto& s) {
        s.rate    = NauD::Rate::sps10;
        s.channel = NauD::Channel::ch2;
    });
    check(runUntil(d, [&] { return !d.pending(); }, 1s), "written, and six conversions at 10 SPS");
    check(m.reg.word(0x02) == 0x80 && m.reg.word(0x1C) == 0x00,
          "channel 2 at 10 SPS, the Cfilter off");
    checkEq(d.period<Nau::Conversion>().count(), 105, "the period follows the rate");
    runFor(d, 150ms);
    {
        auto const n = d.samples<Nau::Conversion>();
        runFor(d, 2s);
        auto const perTwo = d.samples<Nau::Conversion>() - n;
        check(perTwo >= 18 && perTwo <= 20, "about 19 in two seconds at 10 SPS");
    }
    check(d.latest().channel == NauD::Channel::ch2, "samples from channel 2");
    if(failures != 0) { dump(); }
}

void notReady() {
    testCase("NAU7802 not ready");
    fresh();
    Nau7802Model m{};
    m.code(0x000100);
    FakeBus::respond = std::ref(m);
    Dev<Nau> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first conversion");
    m.converting    = false;
    auto const seq  = d.seq();
    auto const rej  = d.rejected<Nau::Conversion>();
    auto const adco = [&] {
        std::size_t n = 0;
        for(auto const& t : FakeBus::log) {
            if(t.isRead() && t.sent == std::vector<std::uint8_t>{0x12}) { ++n; }
        }
        return n;
    };
    auto const reads = adco();
    runFor(d, 200ms);
    checkEq(d.seq(), seq, "no sample while CR is clear");
    check(d.rejected<Nau::Conversion>() > rej, "a run that never saw CR is rejected");
    checkEq(adco(), reads, "and ADCO is never addressed");
    checkEq(d.latest().code, 256, "the last sample stands");
    m.converting = true;
    m.code(0x000200);
    check(runUntil(d, [&] { return d.seq() > seq; }, 100ms), "CR again: samples again");
    checkEq(d.latest().code, 512, "the new code");
    if(failures != 0) { dump(); }
}

void calibrate() {
    testCase("NAU7802 calibration");
    fresh();
    Nau7802Model m{};
    FakeBus::respond = std::ref(m);
    Dev<Nau> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first conversion");

    m.calBusyReads  = 2;   // the first read-back still sees CALS
    auto const from = FakeBus::log.size();
    auto const ran  = m.calibrations;   // the bring-up's
    d.set<Nau::Calibrate>(Nau::calibration(d.value<Nau::Settings>(), NauD::CalMode::systemOffset));
    check(runUntil(
            d,
            [&] { return !d.pending() && m.calibrations > ran && (m.reg.word(0x02) & 0x04U) == 0; },
            2s),
          "calibrated");
    runFor(d, 1s);
    check(hasWrite({0x02, 0x36}, from), "CALS with CALMOD 10 on channel 1 at 80 SPS");
    checkEq(d.mismatches<Nau::Calibrate>(), 1U, "one read-back while it was still running");
    {
        int cals = 0;
        for(auto const& w : writes(from)) {
            cals += w == std::vector<std::uint8_t>{0x02, 0x36} ? 1 : 0;
        }
        checkEq(cals,
                2,
                "CALS written again after the mismatch (the part ignores it while one runs)");
    }
    checkEq(m.calibrations - ran, 1, "one calibration ran");
    {
        // the read-back comes CalibrationWait after the write
        FakeClock::time_point wrote{}, readBack{};
        for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
            auto const& t = FakeBus::log[i];
            if(wrote == FakeClock::time_point{} && t.sent == std::vector<std::uint8_t>{0x02, 0x36})
            {
                wrote = t.at;
            }
            if(wrote != FakeClock::time_point{} && readBack == FakeClock::time_point{} && t.isRead()
               && t.sent == std::vector<std::uint8_t>{0x02})
            {
                readBack = t.at;
            }
        }
        check(readBack - wrote >= 204ms, "read back after CalibrationWait");
    }

    // save: CTRL2 and both channels' OCAL/GCAL
    auto const ticket = d.request<Nau::Calibration>();
    check(runUntil(
            d,
            [&] { return d.answer<Nau::Calibration>(ticket) != Answer::pending; },
            100ms),
          "answered");
    check(d.answer<Nau::Calibration>(ticket) == Answer::ok, "ok");
    check(readOf(0x02, 15), "0x02..0x10 in one burst");
    auto const& saved = d.latest<Nau::Calibration>();
    checkEq(saved.channels[0].offset, 0x123, "channel 1's offset");
    checkEq(saved.channels[0].gain, 0x0080'0000U, "channel 1's gain at 1.0");
    check(!saved.calError && !saved.calibrating, "done, no error");

    // restore
    auto const at = FakeBus::log.size();
    d.set<Nau::RestoreCalibration>(1, {.offset = -5, .gain = 0x0081'2345U});
    check(runUntil(d, [&] { return !d.pending(); }, 100ms), "written");
    check(writes(at) == std::vector<std::vector<std::uint8_t>>{{0x0A, 0xFF, 0xFF, 0xFB, 0x00, 0x81, 0x23, 0x45}},
          "OCAL2 and GCAL2 in one burst");
    check(m.reg.word(0x0A) == 0xFF && m.reg.word(0x0C) == 0xFB && m.reg.word(0x0E) == 0x81,
          "in the part's registers");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    bringUp();
    rejects();
    runtime();
    notReady();
    calibrate();
    return finish();
}
