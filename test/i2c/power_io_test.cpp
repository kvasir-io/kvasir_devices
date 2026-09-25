/// Power monitors, ADCs, DACs, potentiometers, port expanders, memories, clocks and the
/// GPS: each description against a model of the part on the wire.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/All.hpp>
#include <map>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

void ina219() {
    testCase("INA219");
    fresh();
    RegisterModel<1, 2> m{0x40};
    m.set(0x01, {0x0F, 0xA0, 0xFA, 0x00, 0x00, 0x64, 0xFF, 0x9C});
    m.readOnly       = {0x01, 0x02, 0x03, 0x04};
    FakeBus::respond = std::ref(m);
    Dev<Chips::Ina219<>> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 1s), "first sample");
    check(hasWrite({0x00, 0x39, 0x9F}), "configuration 399Fh");
    check(hasWrite({0x05, 0x10, 0x00}), "calibration 4096");
    checkEq(d.latest().shuntVoltage, 40000, "the model's frame reached the decode");
    checkEq(d.latest().busVoltage, 32000U, "bus voltage from the read at offset 2");
    checkEq(d.latest().power, 200U, "power from the read at offset 4");
    checkEq(d.latest().current, -10000, "current from the read at offset 6");
    // a brown-out puts the calibration back to 0; the read-back finds it and writes it again
    m.set(0x05, {0x00, 0x00});
    check(runUntil(
            d,
            [&] { return m.word(0x05) == 0x1000U; },
            1500ms),
          "a lost calibration is written again");
    check(d.mismatches<Chips::Ina219<>::ShuntCal>() >= 1, "and counted as a mismatch");
    // shunt 0x0FA0, bus 0xFA00, power 0x0064, current 0xFF9C at the default 100 mOhm, 100 uA
    static_assert([] {
        auto const f   = frame(0x0F, 0xA0, 0xFA, 0x00, 0x00, 0x64, 0xFF, 0x9C);
        auto const got = Chips::Ina219<>::Power::decode(Bytes{f});
        return equal(got.shuntVoltage, 40000) && equal(got.busVoltage, 32000U)
            && equal(got.power, 200U) && equal(got.current, -10000);
    }());
    if(failures != 0) { dump(); }
}

void ads1115() {
    testCase("ADS1115");
    fresh();
    RegisterModel<1, 2> m{0x48};
    m.onWrite = [&](std::uint32_t reg, RegisterModel<1, 2>::Reg const& r) {
        if(reg != 0x01 || (r[0] & 0x80U) == 0) { return; }
        auto const ch   = (r[0] >> 4) & 0x03U;
        auto const code = static_cast<std::uint16_t>(1000 * (ch + 1));
        m.mem[0x00]
          = {static_cast<std::uint8_t>(code >> 8), static_cast<std::uint8_t>(code & 0xFF)};
    };
    FakeBus::respond = std::ref(m);
    Dev<Chips::Ads1115<>> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 1s), "first sweep");
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0x01, 0xC5, 0x83}, {0x01, 0xD5, 0x83}, {0x01, 0xE5, 0x83}, {0x01, 0xF5, 0x83}},
          "four single-shot configs");
    check(d.latest().code == std::array<std::int16_t, 4>{1000, 2000, 3000, 4000},
          "codes per channel");
    // each read 10 ms after its config write
    check(FakeBus::log[1].at - FakeBus::log[0].at >= 10ms, "conversion wait");
    // codes 1000, 2000, 3000, 4000 at the 2.048 V full scale
    static_assert([] {
        auto const f   = frame(0x03, 0xE8, 0x07, 0xD0, 0x0B, 0xB8, 0x0F, 0xA0);
        auto const got = Chips::Ads1115<>::Sweep::decode(Bytes{f});
        return got.code == std::array<std::int16_t, 4>{1000, 2000, 3000, 4000}
            && equal(Chips::Ads1115<>::Sweep::Sample::toVoltage(16384), 1024000)
            && equal(got.voltage(1), 125000);
    }());
    if(failures != 0) { dump(); }
}

void ds1307() {
    testCase("DS1307");
    fresh();
    RegisterModel<1> m{0x68};
    m.set(0x00, {0x45, 0x59, 0x23, 0x02, 0x31, 0x12, 0x24});
    FakeBus::respond = std::ref(m);
    Dev<Chips::Ds1307> d{};
    check(runUntil(d, [&] { return d.samples<Chips::Ds1307::Clock>() == 1; }, 100ms), "first read");
    checkEq(d.latest<Chips::Ds1307::Clock>().hour, 23, "the model's frame reached the decode");
    // BCD 45 59 23 02 31 12 24: Tue 2024-12-31 23:59:45, CH clear
    static_assert([] {
        auto const  f   = frame(0x45, 0x59, 0x23, 0x02, 0x31, 0x12, 0x24);
        auto const  got = Chips::Ds1307::Clock::decode(Bytes{f});
        auto const& t   = got.value;
        return isOk(got) && t.second == 45 && t.minute == 59 && t.hour == 23 && t.weekday == 2
            && t.day == 31 && t.month == 12 && t.year == 24 && !t.halted;
    }());
    // CH set, as a part comes new: no time to report
    static_assert([] {
        auto const f = frame(0x80, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00);
        return isReject(Chips::Ds1307::Clock::decode(Bytes{f}))
            && isOk(Chips::Ds3231::Clock::decode(
              Bytes{frame(0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00)}));
    }());
    d.set<Chips::Ds1307::SetTime>({0, 30, 12, 5, 1, 2, 25});
    check(d.pending(), "pending");
    check(runUntil(
            d,
            [&] { return !d.pending() && d.writes<Chips::Ds1307::SetTime>() == 1; },
            100ms),
          "written");
    check(m.word(0x00) == 0x00 && m.word(0x01) == 0x30 && m.word(0x02) == 0x12
            && m.word(0x06) == 0x25,
          "BCD, CH clear");
    check(runUntil(
            d,
            [&] { return d.latest<Chips::Ds1307::Clock>().hour == 12; },
            2s),
          "read back");
    if(failures != 0) { dump(); }
}

void pca9685() {
    testCase("PCA9685");
    fresh();
    RegisterModel<1> m{0x40};
    FakeBus::respond = std::ref(m);
    Dev<Chips::Pca9685<>> d{};
    check(runUntil(d, [&] { return d.answering(); }, 100ms), "bring-up");
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0x00, 0x10}, {0xFE, 121}, {0x01, 0x04}, {0x00, 0x20}, {0x00, 0xA0}},
          "sleep, prescale 121, MODE2, wake, restart");
    check(d.identified() && d.state().prescale == 121, "PRE_SCALE and MODE2 read back as written");
    check(runUntil(
            d,
            [&] { return !d.pending(); },
            100ms),
          "the initial value (off) goes to all 16 channels");
    checkEq(d.writes<Chips::Pca9685<>::Channel>(), 16U, "16 writes");
    check(m.word(0x09) == 0x10 && m.word(0x45) == 0x10, "full-off bit on channel 0 and 15");
    auto const from = FakeBus::log.size();
    d.set<Chips::Pca9685<>::Channel>(3, Chips::Pca9685<>::Pwm::duty(2047));
    check(runUntil(d, [&] { return !d.pending(); }, 100ms), "dirty channel written");
    check(writes(from) == std::vector<std::vector<std::uint8_t>>{{0x12, 0x00, 0x00, 0xFF, 0x07}}, "LED3: on 0, off 2047, one transaction");
    if(failures != 0) { dump(); }
}

void aled7709() {
    testCase("ALED7709");
    using Led = Chips::Aled7709<>;
    fresh();
    RegisterModel<1> m{0x28};
    m.set(0x00, {0x13});   // DEVID: cut 1.3
    FakeBus::respond = std::ref(m);
    Dev<Led> d{};
    check(runUntil(d, [&] { return d.answering(); }, 200ms), "bring-up");
    check(d.identityMatched() && d.identity(0) == 0x13, "DEVID read once, by the engine");
    check(FakeBus::log[0].isRead() && FakeBus::log[0].sent == std::vector<std::uint8_t>{0x00},
          "and before anything is written");
    // DIMCFG: REG_PWMI | FDIM 001 (200 Hz) | linear | local = 1000 1001b
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0x01, 0x00}, {0x14, 0x89}, {0x11, 0x0F}, {0x01, 0x81}},
          "standby, dimming, the four channels, then DEN with CLRF");
    static_assert(Led::DimCfg == 0x89);
    static_assert(Led::Address == 0x28
                  && Chips::Aled7709<Chips::Aled7709Detail::Variant::b>::Address == 0x29);
    // the data sheet's identity: version 1, any revision
    static_assert(Led::Identity[0].matches(0x13) && Led::Identity[0].matches(0x14)
                  && !Led::Identity[0].matches(0x23));
    // RISET 10 k: 1022 V / 10 k = 102.2 mA at gain 255
    static_assert(equal(Led::MaxCurrent, Units::microAmp(102'200)));
    static_assert(
      Led::gainFor(Units::microAmp(102'200)) == 255 && Led::gainFor(Units::microAmp(51'100)) == 127
      && Led::gainFor(Units::microAmp(0)) == 0 && Led::gainFor(Units::microAmp(500'000)) == 255);

    auto const from = FakeBus::log.size();
    d.set<Led::Level>(2, {.duty = 0x1234, .gain = 0x80});
    check(runUntil(d, [&] { return !d.pending(); }, 100ms), "a level written");
    check(writes(from) == std::vector<std::vector<std::uint8_t>>{{0x08, 0x12, 0x34, 0x80}},
          "channel 3: PWM3H, PWM3L and GAIN3 in one transaction, so they change on one STOP");

    // DEVSTA LEDF, CHSTA SH2 | OP1, INITSTA CH3GND
    m.set(0x17, {0x08});
    m.set(0x18, {0x21});
    m.set(0x19, {0x04});
    auto const before = d.samples();
    check(runUntil(d, [&] { return d.samples() >= before + 2; }, 500ms), "status read");
    auto const& st = d.latest();
    check(st.ledFault() && st.open(0) && st.shorted(1) && !st.open(1) && st.shortedToGround(2)
            && st.any() && !st.thermalShutdown(),
          "the three status registers, as one sample");

    auto const clr = FakeBus::log.size();
    d.set<Led::ClearFaults>({});
    check(runUntil(d, [&] { return !d.pending(); }, 100ms), "clear faults");
    check(writes(clr) == std::vector<std::vector<std::uint8_t>>{{0x01, 0x81}}, "CLRF with DEN kept");
    if(failures != 0) { dump(); }
}

void pcf8574() {
    testCase("PCF8574");
    fresh();
    std::uint8_t port = 0x00;
    std::uint8_t pins = 0xA5;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x20) { return FakeBus::Result::notAcknowledged; }
            if(sent.size() == 1 && recv.empty()) {
                port = static_cast<std::uint8_t>(sent[0]);
                return FakeBus::Result::succeeded;
            }
            if(sent.empty() && recv.size() == 1) {
                recv[0] = static_cast<std::byte>(pins & port);
                return FakeBus::Result::succeeded;
            }
            return FakeBus::Result::failed;
        };
    Dev<Chips::Pcf8574> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 100ms), "first read");
    checkEq(port, 0xFF, "all released first (Initial)");
    checkEq(d.latest().port, 0xA5, "pins");
    d.set<Chips::Pcf8574::Port>(0x0F);
    check(runUntil(d, [&] { return port == 0x0F; }, 100ms), "output written");
    check(runUntil(d, [&] { return d.samples() == 2; }, 100ms), "read again");
    checkEq(d.latest().port, 0x05, "driven-low pins read low");
    if(failures != 0) { dump(); }
}

void easyC() {
    testCase("MCP47A1 and the two easyC boards");
    fresh();
    std::vector<std::uint8_t> dacW;
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte>) {
        if(a != 0x2E) { return FakeBus::Result::notAcknowledged; }
        dacW.clear();
        for(auto const b : sent) { dacW.push_back(static_cast<std::uint8_t>(b)); }
        return FakeBus::Result::succeeded;
    };
    Dev<Chips::Mcp47a1<Units::milliVolt(3300)>> m47{};
    check(runUntil(m47, [&] { return !m47.absent(); }, 200ms), "not parked");
    check(m47.link() == Link::starting, "no Init and nothing asked of it: starting, not answering");
    checkEq(Chips::Mcp47a1<Units::milliVolt(3300)>::codeFor(Units::milliVolt(1650)),
            std::uint8_t{32},
            "half of VDD is code 32");
    checkEq(Chips::Mcp47a1<Units::milliVolt(3300)>::codeFor(Units::milliVolt(3300)),
            std::uint8_t{64},
            "all of it is code 64, full scale");
    m47.set<Chips::Mcp47a1<Units::milliVolt(3300)>::Level>(32);
    check(runUntil(
            m47,
            [&] { return dacW == std::vector<std::uint8_t>{0x00, 32}; },
            200ms),
          "command byte 0 then the 6-bit level");
    m47.set<Chips::Mcp47a1<Units::milliVolt(3300)>::Level>(200);
    check(runUntil(
            m47,
            [&] { return dacW == std::vector<std::uint8_t>{0x00, 64}; },
            200ms),
          "and it is clamped to full scale, 0x40");

    fresh();
    RegisterModel<1> sl{0x30};
    sl.set(0x00, {0xFF, 0x01});   // 511
    sl.readOnly      = {0x00, 0x01};
    FakeBus::respond = std::ref(sl);
    Dev<Chips::SolderedSlider<>> sp{};
    check(runUntil(sp, [&] { return sp.valid(); }, 300ms), "slider read");
    checkEq(sp.latest().raw, 511U, "the model's frame reached the decode");
    // 0x01FF little endian: 511 of 1023, about half travel
    static_assert([] {
        auto const f   = frame(0xFF, 0x01);
        auto const got = Chips::SolderedSlider<>::Position::decode(Bytes{f});
        return equal(got.raw, 511U) && equal(got.percent(), Units::percent(49));
    }());

    fresh();
    std::array<std::uint8_t, 5> rot{0x0A, 0x00, 0x00, 0x00, 0x06};   // +10, clockwise
    std::vector<std::uint8_t>   rotW;
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(a != 0x30) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                rotW.clear();
                for(auto const b : sent) { rotW.push_back(static_cast<std::uint8_t>(b)); }
                return FakeBus::Result::succeeded;
            }
            for(std::size_t i = 0; i < recv.size() && i < 5; ++i) {
                recv[i] = static_cast<std::byte>(rot[i]);
            }
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::SolderedRotary<>> re{};
    check(runUntil(re, [&] { return re.valid(); }, 300ms), "encoder read");
    checkEq(re.latest().count, 10, "the model's frame reached the decode");
    // a bare five-byte read: int32 counter +10 first, then event 6 (clockwise)
    static_assert([] {
        auto const f   = frame(0x0A, 0x00, 0x00, 0x00, 0x06);
        auto const got = Chips::SolderedRotary<>::Motion::decode(Bytes{f});
        return got.count == 10 && got.event == Chips::SolderedRotary<>::Event::clockwise
            && got.turned() && !got.pressed();
    }());
    rot = {0xFB, 0xFF, 0xFF, 0xFF, 0x01};   // -5, click
    check(runUntil(re, [&] { return re.latest().count == -5; }, 300ms), "negative counts");
    // -5 and event 1: a button click
    static_assert([] {
        auto const f   = frame(0xFB, 0xFF, 0xFF, 0xFF, 0x01);
        auto const got = Chips::SolderedRotary<>::Motion::decode(Bytes{f});
        return got.count == -5 && got.pressed() && !got.turned();
    }());
    re.set<Chips::SolderedRotary<>::Reset>(0);
    check(runUntil(
            re,
            [&] { return rotW == std::vector<std::uint8_t>{0xAA}; },
            200ms),
          "0xAA zeroes the counter");
    if(failures != 0) { dump(); }
}

void ds3502() {
    testCase("DS3502");
    fresh();
    RegisterModel<1> ds{0x28};
    ds.set(0x00, {0x40});   // the IVR brought it up at mid scale
    FakeBus::respond = std::ref(ds);
    Dev<Chips::Ds3502<Units::ohm(10000)>> d5{};
    check(runUntil(d5, [&] { return d5.valid(); }, 500ms), "first sample");
    check(hasWrite({0x02, 0x80}), "MODE 0x80 at bring-up: wiper writes stay volatile");
    checkEq(d5.state().powerUpTap, std::uint8_t{0x40}, "the power-up tap is recorded");
    checkEq(d5.latest().tap, std::uint8_t{0x40}, "and read back");
    // tap 0x40 of 127 on a 10 k part
    static_assert([] {
        auto const f   = frame(0x40);
        auto const got = Chips::Ds3502<Units::ohm(10000)>::Position::decode(Bytes{f});
        return equal(got.tap, 0x40) && equal(got.resistance(), 5039);
    }());

    auto const modeWrites = [] {
        std::size_t n = 0;
        for(auto const& tr : FakeBus::log) {
            if(tr.isWrite() && !tr.sent.empty() && tr.sent[0] == 0x02) { ++n; }
        }
        return n;
    };
    auto const modesBefore = modeWrites();
    d5.set<Chips::Ds3502<Units::ohm(10000)>::Wiper>(0x7F);
    check(runUntil(d5, [&] { return ds.word(0x00) == 0x7F; }, 200ms), "full scale written");
    checkEq(modeWrites(), modesBefore, "an ordinary wiper write never touches MODE");
    checkEq(d5.writes<Chips::Ds3502<Units::ohm(10000)>::Persist>(),
            0U,
            "and does not program the IVR");

    // Programming the IVR is a one-shot: a device reset must not spend a second write cycle
    // on it, while the volatile wiper the application set is owed again.
    auto const persistFrom = FakeBus::log.size();
    d5.set<Chips::Ds3502<Units::ohm(10000)>::Persist>(0x20);
    check(runUntil(
            d5,
            [&] { return d5.writes<Chips::Ds3502<Units::ohm(10000)>::Persist>() == 1; },
            300ms),
          "the IVR write goes out once");
    check(writes(persistFrom) == std::vector<std::vector<std::uint8_t>>{{0x02, 0x00}, {0x00, 0x20}, {0x02, 0x80}},
          "MODE 0x00, the wiper into the IVR, MODE back to 0x80: one item, three transactions");
    bool       dsNak = false;
    auto const resp  = FakeBus::respond;
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(dsNak) { return FakeBus::Result::notAcknowledged; }
            return resp(a, sent, recv);
        };
    ds.set(0x00, {0x00});   // the part comes back at its default
    dsNak = true;
    check(runUntil(d5, [&] { return d5.absent(); }, 3s), "parked after three NAKs");
    dsNak = false;
    check(runUntil(d5, [&] { return !d5.absent() && d5.valid(); }, 5s), "back after a probe");
    check(runUntil(
            d5,
            [&] { return ds.word(0x00) == 0x7F; },
            500ms),
          "the volatile wiper the application set is written again");
    checkEq(d5.writes<Chips::Ds3502<Units::ohm(10000)>::Persist>(),
            1U,
            "but the IVR write was not replayed");
    if(failures != 0) { dump(); }
}

void ads1219() {
    testCase("ADS1219");
    fresh();
    // a code per channel, answered by the MUX (bits 7:5, 011 + channel) of the last WREG:
    // +2^22 (half of the 2^23 full scale), 0x123456, 0x000100, 0xFEDCBA
    std::array<std::array<std::uint8_t, 3>, 4> code{
      {{0x40, 0x00, 0x00}, {0x12, 0x34, 0x56}, {0x00, 0x01, 0x00}, {0xFE, 0xDC, 0xBA}}
    };
    std::size_t               channel = 0;
    std::vector<std::uint8_t> lastCmd;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x40) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                lastCmd.assign(sent.size(), 0);
                for(std::size_t i = 0; i < sent.size(); ++i) {
                    lastCmd[i] = static_cast<std::uint8_t>(sent[i]);
                }
                if(lastCmd.size() == 2 && lastCmd[0] == 0x40) {
                    channel = ((lastCmd[1] >> 5) - 3U) & 0x03U;
                }
                return FakeBus::Result::succeeded;
            }
            if(lastCmd == std::vector<std::uint8_t>{0x20} && recv.size() == 1) {
                recv[0] = std::byte{0x00};   // the configuration register after RESET
                return FakeBus::Result::succeeded;
            }
            for(std::size_t i = 0; i < recv.size() && i < 3; ++i) {
                recv[i] = static_cast<std::byte>(code[channel][i]);
            }
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Ads1219<>> ad19{};
    check(runUntil(ad19, [&] { return ad19.valid(); }, 800ms), "a sweep");
    check(hasWrite({0x06}) && hasWrite({0x20}), "RESET at bring-up, then RREG of the config");
    check(ad19.identified(), "the configuration reads 00h after the reset");
    static_assert(Chips::Ads1219<Chips::Ads1219Detail::Gain::x1,
                                 Chips::Ads1219Detail::DataRate::sps20>::Sweep::Period
                    == 280ms,
                  "at 20 SPS four conversions take longer than 200 ms");
    check(hasWrite({0x40, 0x64}) && hasWrite({0x40, 0xC4}),
          "WREG selects AIN0 (MUX 011) then AIN3 (MUX 110), 90 SPS");
    check(hasWrite({0x08}) && hasWrite({0x10}), "START/SYNC and RDATA");
    checkEq(ad19.latest().code[0], 4194304, "the model's frame reached the decode");
    checkEq(ad19.latest().code[1], 0x123456, "AIN1 from the receive at offset 3");
    checkEq(ad19.latest().code[2], 0x100, "AIN2 from the receive at offset 6");
    checkEq(ad19.latest().code[3], -74566, "AIN3 from the receive at offset 9");
    // 0x400000 (+2^22, half scale), 0x123456, 0x000100, 0xFEDCBA
    static_assert([] {
        auto const f   = frame(0x40, 0, 0, 0x12, 0x34, 0x56, 0x00, 0x01, 0x00, 0xFE, 0xDC, 0xBA);
        auto const got = Chips::Ads1219<>::Sweep::decode(Bytes{f});
        return got.code[0] == 4194304 && got.code[1] == 0x123456 && got.code[2] == 0x100
            && got.code[3] == -74566
            && equal(got.voltage(0), 1024000);   // against the internal 2.048 V
    }());
    code.fill({0xC0, 0x00, 0x00});   // -2^22
    check(runUntil(ad19, [&] { return ad19.samples() == 2; }, 800ms), "second sweep");
    // 0xC00000: negative codes sign extend
    static_assert([] {
        auto const f   = frame(0xC0, 0, 0, 0xC0, 0, 0, 0xC0, 0, 0, 0xC0, 0, 0);
        auto const got = Chips::Ads1219<>::Sweep::decode(Bytes{f});
        return got.code[1] == -4194304;
    }());
    if(failures != 0) { dump(); }
}

void mpr121() {
    testCase("MPR121");
    fresh();
    RegisterModel<1> mpr{0x5A};
    mpr.set(0x00, {0x05, 0x00});   // electrodes 0 and 2 touched
    mpr.readOnly = {0x00, 0x01};
    // the soft reset leaves every register 0 but CDC (10h) and CDT (24h)
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(s.size() == 2 && s[0] == std::byte{0x80} && s[1] == std::byte{0x63}) {
            mpr.set(0x5C, {0x10, 0x24});
        }
        return mpr(a, s, r);
    };
    Dev<Chips::Mpr121<>> mp{};
    check(runUntil(mp, [&] { return mp.valid(); }, 500ms), "first sample");
    check(hasWrite({0x80, 0x63}), "soft reset first");
    check(mp.identified() && mp.state().cdt == 0x24, "CDC and CDT read back at their reset values");
    check(hasWrite({0x5E, 0x00}), "then stop mode, so the rest is writable");
    check(hasWrite({0x41, 0x0C}) && hasWrite({0x42, 0x06}),
          "electrode 0 touch and release thresholds");
    check(hasWrite({0x57, 0x0C}) && hasWrite({0x58, 0x06}), "and electrode 11");
    checkEq(mpr.word(0x5E), 0x8CU, "the ECR that starts it: baseline tracking, 12 electrodes");
    using MprTouch = Chips::Mpr121<>::Touch;
    check(mp.latest<MprTouch>().electrode(0) && mp.latest<MprTouch>().electrode(2),
          "the model's frame reached the decode");
    // touch status 0x0005 little endian: electrodes 0 and 2, OVCF (bit 15) clear
    static_assert([] {
        auto const f   = frame(0x05, 0x00);
        auto const got = MprTouch::decode(Bytes{f});
        return got.electrode(0) && got.electrode(2) && !got.electrode(1) && !got.overCurrent;
    }());
    // the ECR must be written after the thresholds, or they would be ignored
    {
        std::size_t thr = 0, ecr = 0;
        for(std::size_t i = 0; i < FakeBus::log.size(); ++i) {
            auto const& tx = FakeBus::log[i];
            if(tx.isWrite() && !tx.sent.empty() && tx.sent[0] == 0x41) { thr = i; }
            if(tx.isWrite() && tx.sent == std::vector<std::uint8_t>{0x5E, 0x8C}) { ecr = i; }
        }
        check(thr != 0 && ecr > thr, "the run command comes after the configuration");
    }
    testCase("MPR121: not an MPR121 at 0x5A");
    {
        fresh();
        RegisterModel<1> mlx{0x5A};   // an MLX90614 answers, but not with 10h 24h after a reset
        FakeBus::respond = std::ref(mlx);
        Dev<Chips::Mpr121<>> no{};
        runFor(no, 300ms);
        check(!no.identified(), "turned down");
        check(!hasWrite({0x5E, 0x00}), "and nothing past the check was written");
    }
    if(failures != 0) { dump(); }
}

void mcp4018() {
    testCase("MCP4018");
    fresh();
    std::uint8_t tap = 0x40;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x2F) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                tap = static_cast<std::uint8_t>(sent[0]);
                return FakeBus::Result::succeeded;
            }
            recv[0] = std::byte{tap};
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Mcp4018<Units::ohm(5000)>> mc{};
    check(runUntil(mc, [&] { return mc.valid(); }, 500ms), "first sample");
    checkEq(mc.latest().tap, std::uint8_t{0x40}, "the power-up wiper, mid scale");
    // 0x40 of 127 is about half of 5 k, 0x7F the full end-to-end resistance
    static_assert([] {
        using Wiper    = Chips::Mcp4018<Units::ohm(5000)>::Wiper;
        auto const mid = frame(0x40);
        auto const top = frame(0x7F);
        return equal(Wiper::decode(Bytes{mid}).resistance(), 2519)
            && equal(Wiper::decode(Bytes{top}).resistance(), 5000);
    }());
    mc.set<Chips::Mcp4018<Units::ohm(5000)>::Tap>(0x7F);
    check(runUntil(mc, [&] { return tap == 0x7F; }, 200ms), "full scale written as one byte");
    check(runUntil(mc, [&] { return mc.latest().tap == 0x7F; }, 500ms), "and read back");
    if(failures != 0) { dump(); }
}

void ina228() {
    testCase("INA228");
    fresh();
    using I228 = Chips::Ina228<Units::microOhm(2000), Units::milliAmp(10000)>;
    checkEq(I228::Calibration, std::uint16_t{500}, "SHUNT_CAL = 13107.2e6 x LSB x R");
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x40 || sent.empty()) { return FakeBus::Result::notAcknowledged; }
            auto const reg = static_cast<std::uint8_t>(sent[0]);
            if(recv.empty()) { return FakeBus::Result::succeeded; }
            if(reg == 0x3E) {
                recv[0] = std::byte{0x54};
                recv[1] = std::byte{0x49};
            } else if(reg == 0x3F) {
                recv[0] = std::byte{0x22};
                recv[1] = std::byte{0x81};
            } else if(reg == 0x06) {
                recv[0] = std::byte{0x0C};
                recv[1] = std::byte{0x80};
            } else if(reg == 0x0B) {   // DIAG_ALRT: MATHOF clear, TMPOL and MEMSTAT set
                recv[0] = std::byte{0x00};
                recv[1] = std::byte{0x81};
            } else {   // the 24-bit measurements, each its own: a wrong offset reads another
                auto const hi = reg == 0x04 ? 0x10 : reg == 0x05 ? 0x20 : reg == 0x07 ? 0x08 : 0x03;
                recv[0]       = std::byte{static_cast<std::uint8_t>(hi)};
                recv[1]       = std::byte{0x00};
                recv[2]       = std::byte{0x00};
            }
            return FakeBus::Result::succeeded;
        };
    Dev<I228> i228{};
    check(runUntil(i228, [&] { return i228.valid(); }, 800ms), "first sample");
    check(i228.identified(), "manufacturer 0x5449, device 0x228x");
    checkEq(i228.latest().shuntVoltage, 20480000, "the model's frame reached the decode");
    checkEq(i228.latest().busVoltage, 25600000U, "bus voltage from the read at offset 3");
    checkEq(i228.latest().dieTemperature, 25000, "die temperature from the read at offset 6");
    checkEq(i228.latest().current, 624984, "current from the read at offset 8");
    checkEq(i228.latest().power, 11999694U, "power from the read at offset 11");
    checkEq(i228.latest().diagnostics, 0x0081U, "DIAG_ALRT from the read at offset 14");
    // shunt 0x100000, bus 0x200000, temperature 0x0C80, current 0x080000, power 0x030000,
    // DIAG_ALRT 0x0081: TMPOL and MEMSTAT, MATHOF clear
    static_assert([] {
        auto const f
          = frame(0x10, 0, 0, 0x20, 0, 0, 0x0C, 0x80, 0x08, 0, 0, 0x03, 0, 0, 0x00, 0x81);
        auto const  got = I228::Power::decode(Bytes{f});
        auto const& v   = got.value;
        return isOk(got) && equal(v.shuntVoltage, 20480000)   // 20.48 mV at 312.5 nV/LSB
            && equal(v.busVoltage, 25600000U)                 // 25.6 V at 195.3125 uV/LSB
            && equal(v.dieTemperature, 25000)                 // 25.000 degC at 7.8125 m degC/LSB
            && equal(v.current, 624984)                       // 0.625 A at CURRENT_LSB
            && equal(v.power, 11999694U)                      // 12 W at 3.2 x CURRENT_LSB
            && v.diagnostics == 0x0081 && v.temperatureOverLimit() && !v.busOverLimit()
            && !v.busUnderLimit() && !v.powerOverLimit();
    }());
    // BUSOL 4, BUSUL 3, POL 2 of DIAG_ALRT, each on its own
    static_assert([] {
        auto const flags = [](std::uint16_t d) {
            I228::Power::Sample s{};
            s.diagnostics = d;
            return s;
        };
        return flags(0x0010).busOverLimit() && !flags(0x0010).temperatureOverLimit()
            && flags(0x0008).busUnderLimit() && !flags(0x0008).powerOverLimit()
            && flags(0x0004).powerOverLimit() && !flags(0x0004).busUnderLimit()
            && !flags(0x0100).temperatureOverLimit();
    }());
    check(Chips::Ina237<>::Name == std::string_view{"INA237"}, "the INA237 is the INA238 map");
    if(failures != 0) { dump(); }
}

void ads1015() {
    testCase("ADS1015");
    fresh();
    // a left-justified code per channel, answered by the MUX of the last config write:
    // +FS (2047), 0x123, 0x456 and 1
    std::array<std::array<std::uint8_t, 2>, 4> conv{
      {{0x7F, 0xF0}, {0x12, 0x30}, {0x45, 0x60}, {0x00, 0x10}}
    };
    std::size_t mux = 0;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x48) { return FakeBus::Result::notAcknowledged; }
            if(recv.empty()) {
                if(sent.size() == 3 && static_cast<std::uint8_t>(sent[0]) == 0x01) {
                    mux = (static_cast<std::uint8_t>(sent[1]) >> 4) & 0x03U;
                }
                return FakeBus::Result::succeeded;
            }
            recv[0] = static_cast<std::byte>(conv[mux][0]);
            recv[1] = static_cast<std::byte>(conv[mux][1]);
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Ads1015<>> a15{};
    check(runUntil(a15, [&] { return a15.valid(); }, 500ms), "a sweep");
    check(hasWrite({0x01, 0xC5, 0x83}) && hasWrite({0x01, 0xF5, 0x83}),
          "single-shot config for AIN0 and AIN3");
    checkEq(a15.latest().code[0], std::int16_t{2047}, "the model's frame reached the decode");
    checkEq(a15.latest().code[1], std::int16_t{0x123}, "AIN1 from the read at offset 2");
    checkEq(a15.latest().code[2], std::int16_t{0x456}, "AIN2 from the read at offset 4");
    checkEq(a15.latest().code[3], std::int16_t{1}, "AIN3 from the read at offset 6");
    // 0x7FF0 (+FS, 2047 left-justified in 15:4), 0x1230, 0x4560, 0x0010
    static_assert([] {
        auto const f   = frame(0x7F, 0xF0, 0x12, 0x30, 0x45, 0x60, 0x00, 0x10);
        auto const got = Chips::Ads1015<>::Sweep::decode(Bytes{f});
        return got.code[0] == 2047 && got.code[1] == 0x123 && got.code[2] == 0x456
            && got.code[3] == 1 && equal(Chips::Ads1015<>::Sweep::Sample::toVoltage(2047), 2047000)
            && equal(got.voltage(0), 2047000);
    }());
    conv.fill({0x80, 0x00});   // -FS
    check(runUntil(a15, [&] { return a15.samples() == 2; }, 500ms), "second sweep");
    // 0x8000: -FS, and the sign with it
    static_assert([] {
        auto const f   = frame(0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00);
        auto const got = Chips::Ads1015<>::Sweep::decode(Bytes{f});
        return got.code[0] == -2048;
    }());
    if(failures != 0) { dump(); }
}

void ina3221() {
    testCase("INA3221");
    fresh();
    RegisterModel<1, 2> i32{0x40};
    i32.set(0xFE, {0x54, 0x49});
    i32.set(0xFF, {0x32, 0x20});
    i32.set(0x01, {0x03, 0x20});   // 100 counts x 40 uV = 4 mV
    i32.set(0x02, {0x2E, 0xE0});   // 1500 counts x 8 mV = 12 V
    i32.set(0x03, {0x06, 0x40});   // channel 2: 200 counts x 40 uV = 8 mV
    i32.set(0x04, {0x17, 0x70});   // 750 counts x 8 mV = 6 V
    i32.set(0x05, {0x09, 0x60});   // channel 3: 300 counts x 40 uV = 12 mV
    i32.set(0x06, {0x0B, 0xB8});   // 375 counts x 8 mV = 3 V
    i32.readOnly     = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0xFE, 0xFF};
    FakeBus::respond = std::ref(i32);
    Dev<Chips::Ina3221<>> i3{};
    check(runUntil(i3, [&] { return i3.valid(); }, 800ms), "first sample");
    check(i3.identified(), "manufacturer 0x5449, die 0x3220");
    checkEq(i32.word(0x00), 0x7127U, "the reset configuration at bring-up");
    checkEq(i3.latest().shuntVoltage[0], 4000000, "the model's frame reached the decode");
    checkEq(i3.latest().busVoltage[0], 12000, "channel 1 bus from the read at offset 2");
    checkEq(i3.latest().shuntVoltage[1], 8000000, "channel 2 shunt from the read at offset 4");
    checkEq(i3.latest().busVoltage[1], 6000, "channel 2 bus from the read at offset 6");
    checkEq(i3.latest().shuntVoltage[2], 12000000, "channel 3 shunt from the read at offset 8");
    checkEq(i3.latest().busVoltage[2], 3000, "channel 3 bus from the read at offset 10");
    // shunt and bus per channel, each value its own: 0x0320 0x2EE0 0x0640 0x1770 0x0960 0x0BB8
    static_assert([] {
        auto const f
          = frame(0x03, 0x20, 0x2E, 0xE0, 0x06, 0x40, 0x17, 0x70, 0x09, 0x60, 0x0B, 0xB8);
        auto const got = Chips::Ina3221<>::Power::decode(Bytes{f});
        return equal(got.shuntVoltage[0], 4000000) && equal(got.busVoltage[0], 12000)
            && equal(got.current[0], 80000)   // 4 mV through 50 mOhm
            && equal(got.shuntVoltage[1], 8000000) && equal(got.busVoltage[1], 6000)
            && equal(got.current[2], 240000)   // 12 mV through 50 mOhm
            && equal(got.busVoltage[2], 3000);
    }());
    if(failures != 0) { dump(); }
}

void cy15b064j() {
    testCase("CY15B064J");
    fresh();
    RegisterModel<2> fram{0x50};
    for(std::uint32_t i = 0; i < 8; ++i) { fram.set(0x0100 + i, {static_cast<std::uint8_t>(i)}); }
    FakeBus::respond = std::ref(fram);
    Dev<Chips::Cy15b064j<>> fr{};
    check(runUntil(fr, [&] { return fr.answering(); }, 300ms), "up");
    check(FakeBus::log[0].sent == std::vector<std::uint8_t>{0x00, 0x00},
          "the probe is a two-byte address read from zero");
    fr.request<Chips::Cy15b064j<>::Block>({0x0100, 8});
    check(runUntil(
            fr,
            [&] { return fr.samples<Chips::Cy15b064j<>::Block>() == 1; },
            200ms),
          "eight bytes read");
    checkEq(FakeBus::log.back().recvLen, std::size_t{8}, "eight on the wire, not the chunk");
    checkEq(fr.latest<Chips::Cy15b064j<>::Block>().data[5],
            std::uint8_t{5},
            "the model's bytes reached the decode");
    // the address in front, eight bytes behind it: the length is what was read
    static_assert([] {
        auto const f   = frame(0x01, 0x00, 0, 1, 2, 3, 4, 5, 6, 7);
        auto const got = Chips::Cy15b064j<>::Block::decode(Bytes{f});
        return got.address == 0x0100 && got.length == 8 && got.data[5] == 5;
    }());

    Chips::Cy15b064j<>::Store::Value sv{};
    sv.address = 0x0200;
    sv.length  = 3;
    sv.data[0] = 0xAA;
    sv.data[1] = 0xBB;
    sv.data[2] = 0xCC;
    fr.set<Chips::Cy15b064j<>::Store>(sv);
    check(runUntil(fr, [&] { return fram.word(0x0202) == 0xCC; }, 200ms), "three bytes written");
    check(hasWrite({0x02, 0x00, 0xAA, 0xBB, 0xCC}), "address then payload, in one transaction");
    if(failures != 0) { dump(); }
}

void eeprom24aa025e48() {
    testCase("24AA025E48");
    fresh();
    RegisterModel<1> ee48{0x50};
    ee48.set(0xFA, {0x00, 0x04, 0xA3, 0x0B, 0x00, 0x01});
    FakeBus::respond = std::ref(ee48);
    Dev<Chips::Eeprom24aa025e48<>> ep{};
    check(runUntil(ep, [&] { return ep.answering(); }, 300ms), "up");
    check(ep.identified(), "the EUI-48 is neither all zeroes nor all ones");
    checkEq(ep.state().eui48[1], std::uint8_t{0x04}, "Microchip's OUI, kept in State");
    Chips::Eeprom24aa025e48<>::Page::Value pv{};
    pv.address    = 0x10;
    pv.length     = 4;
    pv.data[0]    = 0xDE;
    pv.data[3]    = 0xEF;
    auto const tw = FakeClock::current;
    ep.set<Chips::Eeprom24aa025e48<>::Page>(pv);
    check(runUntil(ep, [&] { return ee48.word(0x13) == 0xEF; }, 200ms), "four bytes written");
    ep.request<Chips::Eeprom24aa025e48<>::Block>({0x10, 4});
    check(runUntil(
            ep,
            [&] { return ep.samples<Chips::Eeprom24aa025e48<>::Block>() == 1; },
            200ms),
          "and read back");
    check(FakeClock::current - tw >= 5ms, "the 5 ms write cycle was waited out");
    checkEq(ep.latest<Chips::Eeprom24aa025e48<>::Block>().data[0], std::uint8_t{0xDE}, "first");
    checkEq(ep.latest<Chips::Eeprom24aa025e48<>::Block>().length, std::uint8_t{4}, "four bytes");
    if(failures != 0) { dump(); }
}

void ad7291() {
    testCase("AD7291");
    fresh();
    using Adc = Chips::Ad7291<0x20>;
    checkEq(Adc::mirrored(0x01), std::uint8_t{0x80}, "channel 0 is D15 in the command register");
    std::array<std::uint8_t, 16> burst{};
    for(std::size_t i = 0; i < 8; ++i) {
        auto const w     = static_cast<std::uint16_t>((i << 12) | (0x100 + i));
        burst[2 * i]     = static_cast<std::uint8_t>(w >> 8);
        burst[2 * i + 1] = static_cast<std::uint8_t>(w & 0xFF);
    }
    std::vector<std::vector<std::uint8_t>> cmdWrites;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x20) { return FakeBus::Result::notAcknowledged; }
            if(recv.empty()) {
                std::vector<std::uint8_t> w;
                for(auto const b : sent) { w.push_back(static_cast<std::uint8_t>(b)); }
                cmdWrites.push_back(w);
                return FakeBus::Result::succeeded;
            }
            auto const reg = static_cast<std::uint8_t>(sent[0]);
            if(reg == 0x01) {
                for(std::size_t i = 0; i < recv.size(); ++i) {
                    recv[i] = static_cast<std::byte>(burst[i]);
                }
            } else {   // TSENSE: channel 8, -40.00 degC is -160 counts = 0xF60
                recv[0] = std::byte{0x8F};
                recv[1] = std::byte{0x60};
            }
            return FakeBus::Result::succeeded;
        };
    Dev<Adc> ad7{};
    check(runUntil(ad7, [&] { return ad7.valid(); }, 800ms), "first burst");
    check(cmdWrites.size() >= 1 && cmdWrites[0] == std::vector<std::uint8_t>{0x00, 0xFF, 0xA0},
          "the expected command register value: FF A0");
    check(ad7.latest<Adc::Voltages>().valid(3), "channel 3 was in the burst");
    checkEq(ad7.latest<Adc::Voltages>().code[3],
            std::uint16_t{0x103},
            "the model's burst reached the decode");
    checkEq(ad7.latest<Adc::Temperature>().temperature, -4000, "and the TSENSE word its own");
    // a burst of eight words, channel address in 15:12 and 0x100 + channel below it
    static_assert([] {
        std::array<std::byte, 16> f{};
        for(std::size_t i = 0; i < 8; ++i) {
            f[2 * i]     = static_cast<std::byte>(i << 4 | 0x01);
            f[2 * i + 1] = static_cast<std::byte>(i);
        }
        auto const got = Adc::Voltages::decode(Bytes{f});
        return got.valid(3) && got.code[3] == 0x103
            && equal(got.voltage(0), 156250);   // against 2.5 V / 4096
    }());
    // TSENSE: channel 8, -160 counts (0xF60) is -40.00 degC, sign from bit 11
    static_assert([] {
        auto const f   = frame(0x8F, 0x60);
        auto const got = Adc::Temperature::decode(Bytes{f});
        return isOk(got) && equal(got.value.temperature, -4000);
    }());
    // the command register cannot be read back, so it is rewritten on its period
    auto const cmds = cmdWrites.size();
    auto const at7  = FakeClock::current;
    while(FakeClock::current - at7 < 6s) { turn(ad7); }
    check(cmdWrites.size() > cmds, "and it is rewritten every five seconds");
    if(failures != 0) { dump(); }
}

void ina238() {
    testCase("INA238");
    fresh();
    // 2 mOhm shunt, 10 A full scale: CURRENT_LSB 305175 nA, SHUNT_CAL 500
    using Ina = Chips::Ina238<Units::microOhm(2000), Units::milliAmp(10000)>;
    checkEq(Ina::Calibration, std::uint16_t{500}, "SHUNT_CAL from the datasheet formula");
    RegisterModel<1, 2> in{0x40};
    in.set(0x04, {0x10, 0x00});   // 4096 x 5 uV = 20.48 mV
    in.set(0x05, {0x20, 0x00});   // 8192 x 3.125 mV = 25.6 V
    in.set(0x06, {0x0C, 0x80});   // 0x0C80 >> 4 = 200 x 125 m degC = 25 degC
    in.set(0x07, {0x08, 0x00});   // 2048 x 305175 nA = 0.625 A
    in.set(0x0B, {0x00, 0x81});   // DIAG_ALRT: TMPOL and MEMSTAT, MATHOF clear
    in.set(0x3E, {0x54, 0x49});   // manufacturer TI
    in.set(0x3F, {0x23, 0x81});   // device id INA238
    in.readOnly = {0x04, 0x05, 0x06, 0x07, 0x08, 0x0B, 0x3E, 0x3F};
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr == 0x40 && sent.size() == 1 && recv.size() == 3
               && static_cast<std::uint8_t>(sent[0]) == 0x08)
            {
                recv[0] = std::byte{0x00};   // 24-bit power: 0x003000 = 12288
                recv[1] = std::byte{0x30};
                recv[2] = std::byte{0x00};
                return FakeBus::Result::succeeded;
            }
            return in(addr, sent, recv);
        };
    Dev<Ina> ii{};
    check(runUntil(ii, [&] { return ii.valid(); }, 800ms), "first sample");
    check(ii.identified(), "manufacturer 0x5449, device 0x2381");
    checkEq(in.word(0x02), 500U, "the calibration register is written at bring-up");
    checkEq(in.word(0x01), 0xFB68U, "and the ADC configuration reset value");
    checkEq(ii.latest().shuntVoltage, 20480000, "the model's frame reached the decode");
    checkEq(ii.latest().busVoltage, 25600000U, "bus voltage from the read at offset 2");
    checkEq(ii.latest().dieTemperature, 25000, "die temperature from the read at offset 4");
    checkEq(ii.latest().current, 624998, "current from the read at offset 6");
    checkEq(ii.latest().power, 749998U, "power from the read at offset 8");
    checkEq(ii.latest().diagnostics, 0x0081U, "DIAG_ALRT from the read at offset 11");
    // shunt 0x1000, bus 0x2000, temperature 0x0C80, current 0x0800, power 0x003000,
    // DIAG_ALRT 0x0081: TMPOL and MEMSTAT, MATHOF clear
    static_assert([] {
        auto const  f   = frame(0x10, 0, 0x20, 0, 0x0C, 0x80, 0x08, 0, 0, 0x30, 0, 0x00, 0x81);
        auto const  got = Ina::Power::decode(Bytes{f});
        auto const& v   = got.value;
        return isOk(got) && equal(v.shuntVoltage, 20480000)   // 20.48 mV at 5 uV/LSB
            && equal(v.busVoltage, 25600000U)                 // 25.6 V at 3.125 mV/LSB
            && equal(v.dieTemperature, 25000)                 // 25.000 degC
            && equal(v.current, 624998)                       // 0.625 A at CURRENT_LSB
            && equal(v.power, 749998U)                        // 0.75 W at 0.2 x CURRENT_LSB
            && v.diagnostics == 0x0081 && v.temperatureOverLimit() && !v.busOverLimit()
            && !v.busUnderLimit() && !v.powerOverLimit();
    }());
    // BUSOL 4, BUSUL 3, POL 2 of DIAG_ALRT, each on its own
    static_assert([] {
        auto const flags = [](std::uint16_t d) {
            Ina::Power::Sample s{};
            s.diagnostics = d;
            return s;
        };
        return flags(0x0010).busOverLimit() && !flags(0x0010).temperatureOverLimit()
            && flags(0x0008).busUnderLimit() && !flags(0x0008).powerOverLimit()
            && flags(0x0004).powerOverLimit() && !flags(0x0004).busUnderLimit()
            && !flags(0x0100).temperatureOverLimit();
    }());
    if(failures != 0) { dump(); }
}

void ad5665() {
    testCase("AD5665R");
    fresh();
    std::vector<std::vector<std::uint8_t>> dacWrites;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte>) {
            if(addr != 0x0F) { return FakeBus::Result::notAcknowledged; }
            std::vector<std::uint8_t> w;
            for(auto const b : sent) { w.push_back(static_cast<std::uint8_t>(b)); }
            dacWrites.push_back(w);
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Ad5665> dd{};
    check(runUntil(
            dd,
            [&] { return dd.writes<Chips::Ad5665::Level>() == 4; },
            200ms),
          "all four channels written at bring-up");
    check(dacWrites.size() == 5 && dacWrites[0] == std::vector<std::uint8_t>{0x28, 0x00, 0x01},
          "the software reset first, DB0 = 1: LDAC, power-down and reference too");
    check(dacWrites.size() == 5 && dacWrites[1] == std::vector<std::uint8_t>{0x18, 0, 0}
            && dacWrites[4] == std::vector<std::uint8_t>{0x1B, 0, 0},
          "each zeroed with its own write-and-update command");
    dd.set<Chips::Ad5665::Level>(Chips::Ad5665::C, 0x8000);
    check(runUntil(
            dd,
            [&] { return hasWrite({0x1A, 0x80, 0x00}); },
            200ms),
          "write-and-update channel C at half scale");
    dd.set<Chips::Ad5665::InternalReference>(Chips::Ad5665::Reference::on);
    check(runUntil(
            dd,
            [&] { return dd.writes<Chips::Ad5665::InternalReference>() == 1; },
            200ms),
          "reference command");
    check(hasWrite({0x38, 0x00, 0x01}), "internal reference on");
    if(failures != 0) { dump(); }
}

void lmk1d1208i() {
    testCase("LMK1D1208I (read-back verify on a real description)");
    fresh();
    RegisterModel<1> lk{0x65};
    lk.set(0x85, {0x20});   // R5: REV_ID 2, DEV_ID 0 (Table 9-15)
    lk.readOnly = {0x85};
    bool lkDrop = false;
    lk.onWrite  = [&](std::uint32_t reg, std::array<std::uint8_t, 1> const&) {
        if(lkDrop && reg == 0x80) {
            lkDrop = false;
            lk.set(0x80, {0x00});   // the output enable byte does not stick, once
        }
    };
    FakeBus::respond = std::ref(lk);
    Dev<Chips::Lmk1d1208i<0x65>> ll{};
    check(runUntil(ll, [&] { return ll.valid(); }, 1500ms), "up");
    check(ll.identified(), "DEV_ID 0 accepted");
    checkEq(ll.latest().revision, 2U, "the model's R5 reached the decode");
    // R5 0x20: REV_ID 2 in 7:4, DEV_ID 0 in 3:0
    static_assert([] {
        auto const f   = frame(0x20);
        auto const got = Chips::Lmk1d1208i<0x65>::Identity::decode(Bytes{f});
        return got.deviceId == 0 && got.revision == 2;
    }());
    check(lk.word(0x80) == 0x00 && lk.word(0x82) == 0xF1,
          "the reset configuration is written at bring-up");
    lkDrop = true;
    ll.set<Chips::Lmk1d1208i<0x65>::OutputEnable>(0x0F);
    check(runUntil(
            ll,
            [&] { return ll.mismatches<Chips::Lmk1d1208i<0x65>::OutputEnable>() == 1; },
            500ms),
          "the read-back caught the dropped write");
    check(runUntil(ll, [&] { return lk.word(0x80) == 0x0F; }, 500ms), "and it was written again");
    if(failures != 0) { dump(); }
}

void pca9557() {
    testCase("PCA9557");
    fresh();
    RegisterModel<1> pe{0x18};
    pe.set(0x00, {0x5A});
    pe.readOnly      = {0x00};
    FakeBus::respond = std::ref(pe);
    Dev<Chips::Pca9557> pp{};
    check(runUntil(pp, [&] { return pp.valid(); }, 500ms), "first sample");
    checkEq(pp.latest().port, 0x5AU, "the model's input port reached the decode");
    // 0x5A: bit 1 set, bit 0 clear
    static_assert([] {
        auto const f   = frame(0x5A);
        auto const got = Chips::Pca9557::Pins::decode(Bytes{f});
        return got.port == 0x5A && got.pin(1) && !got.pin(0);
    }());
    check(
      pe.word(0x03) == 0xFF && pe.word(0x01) == 0x00 && pe.word(0x02) == 0xF0,
      "the reset configuration (0xFF), output (0x00) and polarity (0xF0) are written at bring-up");
    pp.set<Chips::Pca9557::Direction>(0xF0);
    pp.set<Chips::Pca9557::Output>(0x0C);
    check(runUntil(
            pp,
            [&] { return pe.word(0x03) == 0xF0 && pe.word(0x01) == 0x0C; },
            200ms),
          "low nibble as outputs, driven");
    if(failures != 0) { dump(); }
}

void tca9555() {
    testCase("TCA9555");
    fresh();
    RegisterModel<1> te{0x20};
    te.set(0x00, {0x34, 0x12});
    te.readOnly      = {0x00, 0x01};
    FakeBus::respond = std::ref(te);
    Dev<Chips::Tca9555> tc{};
    check(runUntil(tc, [&] { return tc.valid(); }, 500ms), "first sample");
    checkEq(tc.latest().port, 0x1234U, "the model's frame reached the decode");
    // 34 12: port 0 low byte, port 1 high byte
    static_assert([] {
        auto const f = frame(0x34, 0x12);
        return Chips::Tca9555::Pins::decode(Bytes{f}).port == 0x1234;
    }());
    check(te.word(0x06) == 0xFF && te.word(0x07) == 0xFF, "configuration reset both ports");
    auto const from = FakeBus::log.size();
    tc.set<Chips::Tca9555::Direction>(0x00FF);
    tc.set<Chips::Tca9555::Output>(0xA55A);
    check(runUntil(tc, [&] { return !tc.pending(); }, 200ms), "both written");
    check(te.word(0x02) == 0x5A && te.word(0x03) == 0xA5,
          "output written little-endian across the pair");
    check(te.word(0x06) == 0xFF && te.word(0x07) == 0x00, "port 1 is now all outputs");
    check(writes(from) == std::vector<std::vector<std::uint8_t>>{{0x02, 0x5A, 0xA5}, {0x06, 0xFF, 0x00}},
          "the output latches first, then the direction: a pin never drives its old value");
    if(failures != 0) { dump(); }
}

void power() {
    testCase("INA226 (datasheet Table 6-1)");
    fresh();
    RegisterModel<1, 2> m{0x40};
    m.set(0xFE, {0x54, 0x49, 0x22, 0x60});
    m.set(0x01, {0x1F, 0x40, 0x25, 0x70, 0x12, 0xB8, 0x27, 0x10});
    m.readOnly       = {0x01, 0x02, 0x03, 0x04, 0xFE, 0xFF};
    FakeBus::respond = std::ref(m);
    Dev<Chips::Ina226<Units::milliOhm(2), Units::microAmp(1000)>> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 500ms), "first sample");
    check(d.identified(), "TI, die 0x226x");
    check(m.word(0x00) == 0x4127 && m.word(0x05) == 0x0A00, "config, calibration 2560");
    checkEq(d.latest().shuntVoltage, 20000, "the model's frame reached the decode");
    checkEq(d.latest().busVoltage, 11980U, "bus voltage from the read at offset 2");
    checkEq(d.latest().power, 119800U, "power from the read at offset 4");
    checkEq(d.latest().current, 10000000, "current from the read at offset 6");
    // Table 6-1: shunt 0x1F40, bus 0x2570, power 0x12B8, current 0x2710 at 2 mOhm, 1 mA/LSB
    static_assert([] {
        auto const f = frame(0x1F, 0x40, 0x25, 0x70, 0x12, 0xB8, 0x27, 0x10);
        auto const got
          = Chips::Ina226<Units::milliOhm(2), Units::microAmp(1000)>::Power::decode(Bytes{f});
        return equal(got.shuntVoltage, 20000)   // 20 mV
            && equal(got.busVoltage, 11980U)    // 11.98 V
            && equal(got.current, 10000000)     // 10 A
            && equal(got.power, 119800U);       // 119.8 W
    }());

    testCase("MCP3421");
    fresh();
    int reads = 0;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x68) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) { return FakeBus::Result::succeeded; }
            ++reads;
            std::array<std::uint8_t, 4> f{0x01,
                                          0x00,
                                          0x00,
                                          static_cast<std::uint8_t>(reads == 1 ? 0x9C : 0x1C)};
            for(std::size_t i = 0; i < 4; ++i) { recv[i] = static_cast<std::byte>(f[i]); }
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Mcp3421> a{};
    check(runUntil(a, [&] { return a.samples() == 1; }, 1s), "first sample");
    check(writes()[0] == std::vector<std::uint8_t>{0x9C}, "config 0x9C");
    checkEq(reads, 2, "one not-ready read, then the sample");
    checkEq(a.latest().code, 65536, "the model's frame reached the decode");
    // RDY (bit 7 of the config byte) set: not converted yet, ask again in 50 ms
    static_assert([] {
        auto const f   = frame(0x01, 0x00, 0x00, 0x9C);
        auto const got = Chips::Mcp3421::Conversion::decode(Bytes{f});
        return isRetry(got) && got.retryAfter == 50ms;
    }());
    // RDY clear: the 18-bit code 0x10000, 1.024 V at 15.625 uV a count
    static_assert([] {
        auto const f   = frame(0x01, 0x00, 0x00, 0x1C);
        auto const got = Chips::Mcp3421::Conversion::decode(Bytes{f});
        return isOk(got) && got.value.code == 65536 && equal(got.value.voltage(), 1024000);
    }());
    if(failures != 0) { dump(); }
}

void mcp3221() {
    testCase("MCP3221");
    fresh();
    int reads = 0;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x4D) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty() || recv.size() != 2) { return FakeBus::Result::failed; }
            ++reads;
            recv[0] = std::byte{0xF7};   // the upper nibble is don't-care
            recv[1] = std::byte{0xFF};
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Mcp3221<>> d{};
    check(runUntil(d, [&] { return d.valid(); }, 100ms), "first sample");
    checkEq(writes().size(), std::size_t{0}, "nothing is ever written");
    checkEq(d.latest().code, 0x7FF, "the model's frame reached the decode");
    // 0xF7FF: 12-bit code 0x7FF with the don't-care nibble masked, 2047 of 4096 at 3.3 V
    static_assert([] {
        auto const f   = frame(0xF7, 0xFF);
        auto const got = Chips::Mcp3221<>::Conversion::decode(Bytes{f});
        return got.code == 0x7FF && equal(got.voltage(Units::milliVolt(3300)), 1649);
    }());
    auto const n = d.samples();
    runFor(d, 1s);
    checkEq(d.samples() - n, 50U, "50 samples a second");
    checkEq(reads, static_cast<int>(d.samples()), "one read a sample");
    if(failures != 0) { dump(); }
}

void io() {
    testCase("MCP23017");
    fresh();
    RegisterModel<1> m{0x20};
    m.set(0x12, {0xA5, 0x5A});
    m.readOnly       = {0x12, 0x13};
    FakeBus::respond = std::ref(m);
    Dev<Chips::Mcp23017> d{};
    check(runUntil(
            d,
            [&] { return d.samples() == 1 && !d.pending(); },
            500ms),
          "up, initial values written");
    check(m.word(0x0A) == 0x00, "IOCON");
    check(m.word(0x00) == 0xFF && m.word(0x01) == 0xFF && m.word(0x0C) == 0 && m.word(0x14) == 0,
          "IODIR all input, no pull-ups, outputs low");
    checkEq(d.latest().port, 0x5AA5, "the model's frame reached the decode");
    // GPIOA 0xA5, GPIOB 0x5A: pins A | B << 8
    static_assert([] {
        auto const f = frame(0xA5, 0x5A);
        return Chips::Mcp23017::Pins::decode(Bytes{f}).port == 0x5AA5;
    }());
    d.set<Chips::Mcp23017::Output>(0x1234);
    check(runUntil(d, [&] { return !d.pending(); }, 100ms), "output written");
    check(m.word(0x14) == 0x34 && m.word(0x15) == 0x12, "OLATA 0x34, OLATB 0x12");

    testCase("a write group set before the bring-up beats its Initial");
    // An application sets configuration next to the rest of its configuration, before the
    // device has ever been on the wire. A value set that early counts as set: the bring-up
    // writes it in place of Initial, and value<W>() reports what is on the chip.
    fresh();
    RegisterModel<1> early{0x20};
    early.set(0x12, {0xA5});
    early.set(0x13, {0x5A});
    FakeBus::respond = std::ref(early);
    Dev<Chips::Mcp23017> e{};
    e.set<Chips::Mcp23017::PullUp>(0xFFFF);
    check(runUntil(
            e,
            [&] { return e.samples() == 1 && !e.pending(); },
            500ms),
          "up, values written");
    check(early.word(0x0C) == 0xFF && early.word(0x0D) == 0xFF,
          "GPPU is the 0xFFFF that was set, not the group's 0x0000 Initial");
    checkEq(e.value<Chips::Mcp23017::PullUp>(),
            std::uint16_t{0xFFFF},
            "and that is what it reports");
    check(early.word(0x00) == 0xFF && early.word(0x01) == 0xFF,
          "the group that was not set still gets its Initial");

    testCase("MCP4725");
    fresh();
    std::vector<std::uint8_t> lastWrite;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x60) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                lastWrite.clear();
                for(auto const b : sent) { lastWrite.push_back(static_cast<std::uint8_t>(b)); }
                return FakeBus::Result::succeeded;
            }
            std::array<std::uint8_t, 5> f{0xC0, 0xAB, 0xC0, 0x0A, 0xBC};
            for(std::size_t i = 0; i < 5; ++i) { recv[i] = static_cast<std::byte>(f[i]); }
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Mcp4725<>> dac{};
    runFor(dac, 100ms);
    check(dac.link() == Link::starting && lastWrite.empty(),
          "no Init and no Initial: nothing is sent, the part keeps the level its EEPROM loaded");
    dac.set<Chips::Mcp4725<>::Level>(0x0ABC);
    check(runUntil(
            dac,
            [&] { return dac.answering() && !dac.pending(); },
            100ms),
          "level written, and its ACK is what makes the part answering");
    check(lastWrite == std::vector<std::uint8_t>{0x0A, 0xBC}, "fast mode bytes");
    auto const t0 = FakeClock::now();
    dac.set<Chips::Mcp4725<>::Persist>(0x0ABC);
    check(runUntil(dac, [&] { return !dac.pending(); }, 100ms), "persist written");
    check(lastWrite == std::vector<std::uint8_t>{0x60, 0xAB, 0xC0}, "write DAC and EEPROM command");
    dac.request<Chips::Mcp4725<>::Status>();
    check(runUntil(
            dac,
            [&] { return dac.samples<Chips::Mcp4725<>::Status>() == 1; },
            200ms),
          "status read");
    check(FakeClock::now() - t0 >= 50ms,
          "the EEPROM write time passed before the next transaction");
    auto const& st = dac.latest<Chips::Mcp4725<>::Status>();
    checkEq(st.dac, 0xABC, "the model's status frame reached the decode");
    // C0 AB C0 0A BC: RDY and POR, DAC 0xABC, EEPROM 0xABC, both in normal mode
    static_assert([] {
        auto const f   = frame(0xC0, 0xAB, 0xC0, 0x0A, 0xBC);
        auto const got = Chips::Mcp4725<>::Status::decode(Bytes{f});
        return got.ready && got.poweredOn && got.dac == 0xABC && got.eeprom == 0xABC
            && got.dacPowerDown == Chips::Mcp4725<>::PowerDown::normal
            && got.eepromPowerDown == Chips::Mcp4725<>::PowerDown::normal;
    }());

    // the power-down bits
    dac.set<Chips::Mcp4725<>::Level>({0x123, Chips::Mcp4725<>::PowerDown::res100k});
    check(runUntil(
            dac,
            [&] { return hasWrite({0x21, 0x23}); },
            200ms),
          "PD1 PD0 = 10 in bits 5:4 of the fast-mode write");
    dac.set<Chips::Mcp4725<>::Persist>({0xABC, Chips::Mcp4725<>::PowerDown::res1k});
    check(runUntil(
            dac,
            [&] { return hasWrite({0x62, 0xAB, 0xC0}); },
            200ms),
          "and in bits 2:1 of the EEPROM command");

    testCase("TCA9548A");
    fresh();
    std::uint8_t ctrl = 0xFF;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x70) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                ctrl = static_cast<std::uint8_t>(sent[0]);
                return FakeBus::Result::succeeded;
            }
            recv[0] = std::byte{ctrl};
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Tca9548a> mux{};
    check(runUntil(mux, [&] { return mux.answering() && !mux.pending(); }, 100ms), "up");
    checkEq(ctrl, 0x00, "all channels closed at start");
    mux.set<Chips::Tca9548a::Channels>(0x05);
    check(runUntil(mux, [&] { return ctrl == 0x05; }, 100ms), "channels 0 and 2 open");
    mux.request<Chips::Tca9548a::Selected>();
    check(runUntil(mux, [&] { return mux.samples() == 1; }, 100ms), "read back");
    checkEq(mux.latest(), 0x05, "control byte");

    testCase("PCF8575");
    fresh();
    std::uint16_t port = 0;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x20) { return FakeBus::Result::notAcknowledged; }
            if(sent.size() == 2) {
                port = static_cast<std::uint16_t>(static_cast<std::uint8_t>(sent[0])
                                                  | (static_cast<std::uint8_t>(sent[1]) << 8));
                return FakeBus::Result::succeeded;
            }
            if(recv.size() == 2) {
                recv[0] = std::byte{0xA5};
                recv[1] = std::byte{0x5A};
                return FakeBus::Result::succeeded;
            }
            return FakeBus::Result::failed;
        };
    Dev<Chips::Pcf8575> p{};
    check(runUntil(p, [&] { return p.samples() == 1; }, 100ms), "first read");
    checkEq(port, 0xFFFF, "released");
    checkEq(p.latest().port, 0x5AA5, "the model's frame reached the decode");
    // A5 5A: P0 low byte, P1 high byte
    static_assert([] {
        auto const f = frame(0xA5, 0x5A);
        return Chips::Pcf8575::Pins::decode(Bytes{f}).port == 0x5AA5;
    }());
    if(failures != 0) { dump(); }
}

void eeprom() {
    testCase("AT24C32");
    fresh();
    RegisterModel<2> m{0x50};
    for(std::uint32_t i = 0; i < 32; ++i) { m.mem[0x0120 + i] = {static_cast<std::uint8_t>(i)}; }
    FakeBus::respond = std::ref(m);
    Dev<Chips::At24c32> d{};
    check(d.link() == Link::starting, "a memory has no Init: starting until it is asked something");
    checkEq(FakeBus::log.size(), 0U, "no bring-up transactions, nothing cyclic");
    d.request<Chips::At24c32::ReadPage>({0x0120});
    check(runUntil(d, [&] { return d.samples() == 1; }, 100ms), "page read");
    check(d.answering(), "and answering once the read was acknowledged");
    check(FakeBus::log[0].sent == std::vector<std::uint8_t>{0x01, 0x20}
            && FakeBus::log[0].recvLen == 32,
          "two address bytes, 32 data bytes");
    checkEq(d.latest().data[31], 31, "the model's page reached the decode");
    // the two address bytes, then 32 data bytes 0..31
    static_assert([] {
        std::array<std::byte, 34> f{std::byte{0x01}, std::byte{0x20}};
        for(std::size_t i = 0; i < 32; ++i) { f[2 + i] = static_cast<std::byte>(i); }
        auto const got = Chips::At24c32::ReadPage::decode(Bytes{f});
        return got.address == 0x0120 && got.data[5] == 5 && got.data[31] == 31;
    }());
    Chips::At24c32::WritePage::Value w{};
    w.address = 0x0200;
    w.length  = 4;
    w.data[0] = 0xDE;
    w.data[1] = 0xAD;
    w.data[2] = 0xBE;
    w.data[3] = 0xEF;
    d.set<Chips::At24c32::WritePage>(w);
    check(runUntil(d, [&] { return !d.pending(); }, 100ms), "page written");
    check(FakeBus::log.back().sent == std::vector<std::uint8_t>{0x02, 0x00, 0xDE, 0xAD, 0xBE, 0xEF},
          "address then four bytes");
    check(m.mem[0x0203][0] == 0xEF, "landed");

    testCase("AT24C32: a page write stops at the end of its page and of the memory");
    Chips::At24c32::WritePage::Value across{};
    across.address = 0x001C;
    across.length  = 8;
    for(std::size_t i = 0; i < 8; ++i) { across.data[i] = static_cast<std::uint8_t>(0xA0 + i); }
    checkEq(Chips::At24c32::WritePage::fits(across), 4, "four bytes left in the page from 0x1C");
    across.address = 0x0FFE;
    checkEq(Chips::At24c32::WritePage::fits(across), 2, "two before the end of 4096 bytes");
    across.address = 0x1000;
    checkEq(Chips::At24c32::WritePage::fits(across), 0, "none past the end");
    across.address = 0x001C;
    d.set<Chips::At24c32::WritePage>(across);
    check(runUntil(d, [&] { return !d.pending(); }, 100ms), "written");
    check(FakeBus::log.back().sent == std::vector<std::uint8_t>{0x00, 0x1C, 0xA0, 0xA1, 0xA2, 0xA3},
          "the address and the four bytes that fit, not across into the next page");

    testCase("CAT24C512");
    fresh();
    RegisterModel<2> big{0x50};
    for(std::uint32_t i = 0; i < 128; ++i) {
        big.mem[0xFF80 + i] = {static_cast<std::uint8_t>(0x80 + i)};
    }
    FakeBus::respond = std::ref(big);
    Dev<Chips::At24c512> e{};
    check(e.link() == Link::starting, "starting until it is asked something");
    // 128 is what a Step's count -- one byte -- still holds, and the last page of the part
    // is the far end of both the 16-bit address and that count.
    e.request<Chips::At24c512::ReadPage>({0xFF80});
    check(runUntil(e, [&] { return e.samples() == 1; }, 100ms), "page read");
    check(e.answering(), "and answering once the read was acknowledged");
    check(FakeBus::log[0].sent == std::vector<std::uint8_t>{0xFF, 0x80}
            && FakeBus::log[0].recvLen == 128,
          "the last page: two address bytes, 128 data bytes");
    checkEq(e.latest().data[127], 0xFF, "the model's page reached the decode");
    // the last page: FF 80, then 128 bytes 0x80..0xFF
    static_assert([] {
        std::array<std::byte, 130> f{std::byte{0xFF}, std::byte{0x80}};
        for(std::size_t i = 0; i < 128; ++i) { f[2 + i] = static_cast<std::byte>(0x80 + i); }
        auto const got = Chips::At24c512::ReadPage::decode(Bytes{f});
        return got.address == 0xFF80 && got.data[0] == 0x80 && got.data[127] == 0xFF;
    }());
    Chips::At24c512::WritePage::Value wide{};
    wide.address = 0xFF80;
    wide.length  = 128;
    for(std::size_t i = 0; i < 128; ++i) { wide.data[i] = static_cast<std::uint8_t>(i); }
    e.set<Chips::At24c512::WritePage>(wide);
    check(runUntil(e, [&] { return !e.pending(); }, 100ms), "page written");
    check(FakeBus::log.back().sent.size() == 130 && FakeBus::log.back().sent[0] == 0xFF
            && FakeBus::log.back().sent[1] == 0x80 && FakeBus::log.back().sent[129] == 127,
          "two address bytes and a full page behind them");
    check(big.mem[0xFFFF][0] == 127, "the last byte of the part landed");
    if(failures != 0) { dump(); }
}

void pa1010d() {
    testCase("PA1010D");
    fresh();
    // the module pads with 0x0A when its buffer runs dry, also in the middle of a sentence it is
    // still writing: those are not line ends
    std::string stream
      = "\n\n$GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,\n\n46.9,M,,*77\r\n$GNRMC,"
        "123519."
        "00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*5A\r\n";
    std::size_t               pos = 0;
    std::vector<std::uint8_t> command;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> s, std::span<std::byte> recv) {
            if(addr != 0x10) { return FakeBus::Result::notAcknowledged; }
            if(!s.empty()) {
                command.clear();
                for(auto const b : s) { command.push_back(static_cast<std::uint8_t>(b)); }
                return FakeBus::Result::succeeded;
            }
            for(auto& b : recv) {
                b = static_cast<std::byte>(pos < stream.size() ? stream[pos++] : '\n');
            }
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Pa1010d<>> d{};
    check(runUntil(
            d,
            [&] { return d.answering() && !d.pending(); },
            1500ms),
          "bring-up and the initial command");
    check(command.size() == 51 && command[0] == '$'
            && std::string(command.begin(), command.begin() + 8) == "$PMTK314",
          "PMTK314 sent with checksum");
    check(std::string(command.end() - 5, command.end()) == "*28\r\n", "checksum *28");

    Chips::Nmea nmea{};
    int         gga = 0, rmc = 0;
    std::string lat, sats;
    for(int i = 0; i < 400 && (gga == 0 || rmc == 0); ++i) {
        if(d.fresh()) {
            nmea.feed(d.latest().chars, [&](Chips::Nmea const& n) {
                if(n.is("GGA")) {
                    ++gga;
                    lat  = std::string(n.field(2));
                    sats = std::string(n.field(7));
                }
                if(n.is("RMC")) { ++rmc; }
            });
        }
        turn(d);
    }
    checkEq(gga, 1, "one GGA");
    checkEq(rmc, 1, "one RMC");
    check(lat == "4807.038" && sats == "08", "GGA fields");
    checkEq(nmea.badChecksum, 0U, "checksums fine");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    ina219();
    ads1115();
    ds1307();
    pca9685();
    aled7709();
    pcf8574();
    easyC();
    ds3502();
    ads1219();
    mpr121();
    mcp4018();
    ina228();
    ads1015();
    ina3221();
    cy15b064j();
    eeprom24aa025e48();
    ad7291();
    ina238();
    ad5665();
    lmk1d1208i();
    pca9557();
    tca9555();
    power();
    mcp3221();
    io();
    eeprom();
    pa1010d();
    return finish();
}
