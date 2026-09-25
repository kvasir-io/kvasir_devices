/// The displays and the AMOLED board's parts: SSD1306 family, HD44780 behind a PCF8574,
/// HT16K33, the QMI8658 / AXP2101 / PCF85063A trio and the ES8311 codec.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/MonoPanel.hpp>
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

void ht16k33() {
    testCase("HT16K33");
    fresh();
    std::vector<std::vector<std::uint8_t>> sent;
    FakeBus::respond = [&](std::uint8_t addr, std::span<std::byte const> s, std::span<std::byte>) {
        if(addr != 0x70) { return FakeBus::Result::notAcknowledged; }
        std::vector<std::uint8_t> v;
        for(auto const b : s) { v.push_back(static_cast<std::uint8_t>(b)); }
        sent.push_back(v);
        return FakeBus::Result::succeeded;
    };
    Dev<Chips::Ht16k33> d{};
    check(runUntil(
            d,
            [&] { return d.answering() && !d.pending(); },
            500ms),
          "bring-up and the initial writes");
    check(sent.size() >= 4 && sent[0] == std::vector<std::uint8_t>{0x21}
            && sent[2] == std::vector<std::uint8_t>{0x81},
          "oscillator on, display on");
    check(hasWrite({0xEF}) && hasWrite({0x81}), "brightness 15 and blink off (Initial)");
    d.set<Chips::Ht16k33::Display>(Chips::Ht16k33::fourDigits({1, 2, 3, 4}, true));
    check(runUntil(d, [&] { return !d.pending(); }, 200ms), "display written");
    auto const& last = sent.back();
    check(last.size() == 17 && last[0] == 0x00 && last[1] == 0x06 && last[3] == 0x5B
            && last[5] == 0x02 && last[7] == 0x4F && last[9] == 0x66,
          "RAM address 0, digits 1 2 : 3 4 in rows 0 1 2 3 4");
    d.set<Chips::Ht16k33::Brightness>(3);
    check(runUntil(d, [&] { return !d.pending(); }, 200ms), "brightness written");
    check(sent.back() == std::vector<std::uint8_t>{0xE3}, "dimming command");
    if(failures != 0) { dump(); }
}

void lcd1602() {
    testCase("LCD1602 (HD44780 behind a PCF8574)");
    fresh();
    std::vector<std::vector<std::uint8_t>> sent;
    FakeBus::respond = [&](std::uint8_t addr, std::span<std::byte const> s, std::span<std::byte>) {
        if(addr != 0x27) { return FakeBus::Result::notAcknowledged; }
        std::vector<std::uint8_t> v;
        for(auto const b : s) { v.push_back(static_cast<std::uint8_t>(b)); }
        sent.push_back(v);
        return FakeBus::Result::succeeded;
    };
    Dev<Chips::Lcd1602> d{};
    check(runUntil(d, [&] { return d.answering() && !d.pending(); }, 1s), "bring-up completes");
    check(sent.size() >= 9 && sent[0] == std::vector<std::uint8_t>{0x3C, 0x38}
            && sent[3] == std::vector<std::uint8_t>{0x2C, 0x28},
          "three 0x3 nibbles then 0x2 (4-bit mode), E pulsed, backlight on");
    // function set 0x28: high nibble 0x2 -> 0x2C 0x28, low nibble 0x8 -> 0x8C 0x88
    check(sent[4] == std::vector<std::uint8_t>{0x2C, 0x28, 0x8C, 0x88},
          "function set as four port bytes");
    d.set<Chips::Lcd1602::Line>(1, Chips::Lcd1602::Text::of("Hi"));
    check(runUntil(d, [&] { return !d.pending(); }, 200ms), "line written");
    auto const& l = sent.back();
    check(l.size() == 68, "address + 16 characters, four bytes each");
    // DDRAM 0x40 | 0x80 = 0xC0: nibbles 0xC then 0x0, RS low
    check(l[0] == 0xCC && l[1] == 0xC8 && l[2] == 0x0C && l[3] == 0x08, "set DDRAM address 0x40");
    // 'H' = 0x48 with RS: 0x4D 0x49 0x8D 0x89
    check(l[4] == 0x4D && l[5] == 0x49 && l[6] == 0x8D && l[7] == 0x89, "'H' as data nibbles");
    check(l[12] == 0x2D && l[14] == 0x0D, "padding spaces");
    if(failures != 0) { dump(); }

    testCase("LCD2004: twenty columns, rows 2 and 3 at DDRAM 0x14 and 0x54");
    fresh();
    sent.clear();
    FakeBus::respond = [&](std::uint8_t addr, std::span<std::byte const> s, std::span<std::byte>) {
        if(addr != 0x27) { return FakeBus::Result::notAcknowledged; }
        std::vector<std::uint8_t> v;
        for(auto const b : s) { v.push_back(static_cast<std::uint8_t>(b)); }
        sent.push_back(v);
        return FakeBus::Result::succeeded;
    };
    Dev<Chips::Lcd2004> w{};
    check(runUntil(w, [&] { return w.answering() && !w.pending(); }, 1s), "bring-up completes");
    w.set<Chips::Lcd2004::Line>(3, Chips::Lcd2004::Text::of("20x4"));
    check(runUntil(w, [&] { return !w.pending(); }, 200ms), "line written");
    auto const& r = sent.back();
    checkEq(r.size(), std::size_t{84}, "address + 20 characters, four bytes each");
    // 0x80 | 0x54 = 0xD4: nibbles 0xD then 0x4, RS low
    check(r[0] == 0xDC && r[1] == 0xD8 && r[2] == 0x4C && r[3] == 0x48, "set DDRAM address 0x54");
    // '2' = 0x32 with RS: 0x3D 0x39 0x2D 0x29
    check(r[4] == 0x3D && r[5] == 0x39 && r[6] == 0x2D && r[7] == 0x29, "'2' as data nibbles");
    check(r[80] == 0x2D && r[82] == 0x0D, "padded to the last column");
    if(failures != 0) { dump(); }
}

// -- the SSD1306 family ---------------------------------------------------------------------

namespace Oled {
    struct Wide : Chips::Ssd1306Detail::PanelDefaults {
        static constexpr int      Width   = 128;
        static constexpr int      Height  = 32;
        static constexpr Address7 Address = 0x3C;
    };

    /// The 0.66" module: 64 x 48 on RAM columns 32..95.
    struct Small : Chips::Ssd1306Detail::PanelDefaults {
        static constexpr int          Width        = 64;
        static constexpr int          Height       = 48;
        static constexpr Address7     Address      = 0x3D;
        static constexpr int          ColumnOffset = 32;
        static constexpr std::uint8_t ComPins      = 0x12;
    };
}   // namespace Oled

// MonoPanel names its device, so that device must have static storage. MonoPanelRef holds
// one by reference instead and may point at a local; both have to satisfy what a frame
// renderer asks of a panel, which is spelled out here as a concept.
template<typename P>
concept PanelContract = requires(P& p, std::uint32_t pages) {
    { P::Width } -> std::convertible_to<int>;
    { P::Height } -> std::convertible_to<int>;
    { p.ready() } -> std::same_as<bool>;
    { p.failed() } -> std::same_as<bool>;
    { p.handler() };
    { p.pageRam() };
    { p.pagesChanged(pages) };
};

using PanelDev = Dev<Chips::Ssd1315<Oled::Wide>>;
using Panel    = Kvasir::I2C::MonoPanelRef<PanelDev>;
static_assert(PanelContract<Panel>,
              "the panel is what a frame renderer wants");

void ssd1306() {
    using Wire = std::vector<std::vector<std::uint8_t>>;
    using W    = Chips::Ssd1315<Oled::Wide>;

    testCase("SSD1315 128 x 32: bring-up is the command batch, a blank frame, then the panel on");
    fresh();
    std::uint8_t seen = 0;
    FakeBus::respond  = [&](std::uint8_t addr, std::span<std::byte const>, std::span<std::byte>) {
        if(addr != 0x3C) { return FakeBus::Result::notAcknowledged; }
        ++seen;
        return FakeBus::Result::succeeded;
    };
    Dev<W> d{};
    check(runUntil(
            d,
            [&] { return d.answering() && !d.pending(); },
            500ms),
          "up and everything written");

    auto const all = writes();
    // The data sheet's bring-up sequence for this panel, split on
    // command boundaries into transactions of a control byte and up to seven command bytes.
    check(all.size() >= 4
            && all[0] == std::vector<std::uint8_t>{0x00, 0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00}
            && all[1] == std::vector<std::uint8_t>{0x00, 0x40, 0x8D, 0x14, 0x20, 0x02, 0xA0, 0xC0}
            && all[2] == std::vector<std::uint8_t>{0x00, 0xDA, 0x02, 0x81, 0x7F, 0xD9, 0xF1}
            && all[3] == std::vector<std::uint8_t>{0x00, 0xDB, 0x40, 0xA4, 0xA6},
          "the bring-up commands, and no DISPLAYON among them");

    checkEq(all.size(), std::size_t{4 + 2 * 4 + 1}, "four command batches, four pages, the on");
    for(std::size_t p = 0; p < 4; ++p) {
        auto const& win = all[4 + 2 * p];
        auto const& dat = all[5 + 2 * p];
        check(win
                == std::vector<std::uint8_t>{0x00, static_cast<std::uint8_t>(0xB0 + p), 0x00, 0x10},
              "the page and column window");
        check(dat.size() == 129 && dat[0] == 0x40, "the data stream and 128 bytes");
        check(std::all_of(std::next(dat.begin()), dat.end(), [](auto b) { return b == 0; }),
              "blank");
    }
    check(all.back() == std::vector<std::uint8_t>{0x00, 0xAF},
          "DISPLAYON last, so the random RAM a cold controller holds is never shown");

    testCase("SSD1315: only a page that changed goes on the wire");
    auto                    base = FakeBus::log.size();
    typename W::Page::Value row{};
    row.fill(0xF0);
    d.set<W::Page>(2, row);
    check(runUntil(d, [&] { return !d.pending(); }, 200ms), "written");
    auto const one = writes(base);
    checkEq(one.size(), std::size_t{2}, "one page: two transactions, not eight");
    check(one[0] == std::vector<std::uint8_t>{0x00, 0xB2, 0x00, 0x10}, "page 2");
    check(one[1].size() == 129 && one[1][1] == 0xF0, "and its bytes");

    testCase("SSD1315: contrast and inversion");
    base = FakeBus::log.size();
    d.set<W::Contrast>(0x40);
    d.set<W::Invert>(W::Invert::Value::inverted);
    check(runUntil(d, [&] { return !d.pending(); }, 200ms), "written");
    check(writes(base) == Wire{{0x00, 0x81, 0x40}, {0x00, 0xA7}}, "81h then A7h");

    testCase("SSD1315 64 x 48 at 0x3D: the column offset addresses RAM column 32");
    fresh();
    using S          = Chips::Ssd1315<Oled::Small>;
    FakeBus::respond = [&](std::uint8_t addr, std::span<std::byte const>, std::span<std::byte>) {
        return addr == 0x3D ? FakeBus::Result::succeeded : FakeBus::Result::notAcknowledged;
    };
    Dev<S> sm{};
    check(runUntil(sm, [&] { return sm.answering() && !sm.pending(); }, 500ms), "up");
    checkEq(std::size_t{S::Pages}, std::size_t{6}, "six pages");
    check(hasWrite({0x00, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1}), "ComPins 12h, as the panel says");
    check(hasWrite({0x00, 0xB0, 0x00, 0x12}), "page 0 at column 32: low nibble 0, high 2");
    check(hasWrite({0x00, 0xB5, 0x00, 0x12}), "and page 5, the last");

    testCase("SSD1315: MonoPanel draws into the frame in place");
    fresh();
    FakeBus::respond = [&](std::uint8_t addr, std::span<std::byte const>, std::span<std::byte>) {
        return addr == 0x3C ? FakeBus::Result::succeeded : FakeBus::Result::notAcknowledged;
    };
    // The device is a local and the panel points at it: nothing here has static storage.
    PanelDev panelDev{};
    Panel    panel{panelDev};
    check(runUntil(panelDev, [&] { return panel.ready(); }, 500ms), "ready once brought up");
    checkEq(std::size_t{Panel::Pages}, std::size_t{4}, "four pages");
    base                  = FakeBus::log.size();
    panel.pageRam()[1][7] = 0x5A;   // a canvas would do this, one pixel at a time
    panel.pagesChanged(1U << 1);
    check(runUntil(panelDev, [&] { return panel.ready(); }, 200ms), "sent");
    auto const drawn = writes(base);
    checkEq(drawn.size(), std::size_t{2}, "the one page that was drawn into");
    check(drawn[0] == std::vector<std::uint8_t>{0x00, 0xB1, 0x00, 0x10}, "page 1");
    check(drawn[1].size() == 129 && drawn[1][8] == 0x5A,
          "the byte the caller wrote went out with no copy in between");

    testCase("SSD1315: MonoPanel sends a page drawn into only when its bytes changed");
    // A frame that clears and redraws touches every page; what goes on the wire is only the
    // pages that now read differently from what was sent last.
    base                  = FakeBus::log.size();
    panel.pageRam()[1][7] = 0x5A;   // drawn again, the same as before
    panel.pageRam()[2][0] = 0x00;   // drawn into, still blank
    panel.pagesChanged((1U << 1) | (1U << 2));
    runFor(panelDev, 50ms);
    // Page 2 has never been sent through the panel, so nothing is known about what the glass
    // shows there: it goes out once. Page 1 is exactly what was sent, and does not.
    auto const redrawn = writes(base);
    checkEq(redrawn.size(), std::size_t{2}, "one page, not two");
    check(redrawn.size() == 2 && redrawn[0] == std::vector<std::uint8_t>{0x00, 0xB2, 0x00, 0x10},
          "page 2, the one never sent; page 1 as it was sent stays off the wire");
    base                  = FakeBus::log.size();
    panel.pageRam()[1][7] = 0xA5;
    panel.pagesChanged(1U << 1);
    check(runUntil(panelDev, [&] { return panel.ready(); }, 200ms), "sent");
    auto const changed = writes(base);
    checkEq(changed.size(), std::size_t{2}, "a byte that changed sends its page");
    check(changed.size() == 2 && changed[1][8] == 0xA5, "with the new byte");

    testCase("SSD1315: a failure mid-page retries the page from its window command");
    fresh();
    int nak = 1;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte>) {
            if(addr != 0x3C) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty() && static_cast<std::uint8_t>(sent[0]) == 0x40 && nak > 0) {
                --nak;
                return FakeBus::Result::notAcknowledged;
            }
            return FakeBus::Result::succeeded;
        };
    Dev<W> f{};
    check(runUntil(
            f,
            [&] { return f.answering() && !f.pending(); },
            500ms),
          "up despite the one NAK");
    auto const  w       = writes();
    std::size_t windows = 0;
    for(auto const& t : w) {
        if(t.size() == 4 && t[0] == 0x00 && t[1] == 0xB0) { ++windows; }
    }
    checkEq(windows, std::size_t{2}, "page 0's window went out twice: the whole page again");
    checkEq(f.errors(), 1U, "one error");
    check(hasWrite({0x00, 0xAF}), "and the panel still came on");
    if(failures != 0) { dump(); }
}

// -- the Waveshare RP2350-Touch-AMOLED-1.75's other three I2C parts ------------------------

void amoledBoard() {
    testCase("QMI8658: bring-up, and twelve data bytes out of one read");
    fresh();
    RegisterModel<1> imu{0x6A};
    imu.set(0x00, {0x05});   // WHO_AM_I
    imu.set(0x2E, {0x03});   // STATUS0: accelerometer and gyroscope data available
    // +1 g on Z at the 8g range (4096 counts/g), -0.5 g on X, and 100 dps on Y at 1024 dps
    // (32 counts/dps). Little-endian pairs from 0x35.
    imu.set(0x35, {0x00, 0xF8, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x80, 0x0C, 0x00, 0x00});
    FakeBus::respond = std::ref(imu);
    Dev<Chips::Qmi8658<>> q{};
    check(runUntil(q, [&] { return q.samples() == 1; }, 500ms), "first sample");
    check(q.identified(), "WHO_AM_I 0x05");
    checkEq(imu.word(0x02),
            0x40U,
            "CTRL1: auto-increment on for the 12-byte read, big-endian off for the decode");
    checkEq(imu.word(0x03), 0x26U, "CTRL2: 8g at 125 Hz");
    checkEq(imu.word(0x04), 0x66U, "CTRL3: 1024 dps at 125 Hz, per the datasheet's gFS");
    checkEq(imu.word(0x08), 0x03U, "CTRL7: accelerometer and gyroscope on");
    checkEq(q.latest().accel[2], 1'000'000, "the model's frame reached the decode");
    // 0x80 0x0C at buffer 9..10, both bytes set and unequal: a data read landed anywhere but
    // buffer offset 1 (after STATUS0) gives something other than 100 dps here
    checkEq(q.latest().gyro[1], 100'000, "the 12-byte read at offset 1, late in its frame");
    // STATUS0 aDA | gDA, then -0.5 g on X, +1 g on Z at 8g, 100 dps on Y at 1024 dps
    static_assert([] {
        namespace QD = Chips::Qmi8658Detail;
        auto const f
          = frame(0x03, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x80, 0x0C, 0x00, 0x00);
        constexpr Chips::Qmi8658<>::State st{
          .deviceId   = 0x05,
          .accelRange = static_cast<std::uint8_t>(QD::AccelRange::g8),
          .gyroRange  = static_cast<std::uint8_t>(QD::GyroRange::dps1024)};
        auto const got = Chips::Qmi8658<>::Motion::decode(Bytes{f}, st);
        return isOk(got) && equal(got.value.accel[0], -500'000) && equal(got.value.accel[1], 0)
            && equal(got.value.accel[2], 1'000'000) && equal(got.value.gyro[0], 0)
            && equal(got.value.gyro[1], 100'000) && equal(got.value.gyro[2], 0);
    }());

    testCase("QMI8658: a different range rescales without touching the decode");
    fresh();
    RegisterModel<1> imu2{0x6A};
    imu2.set(0x00, {0x05});
    imu2.set(0x2E, {0x03});
    imu2.set(0x35, {0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    FakeBus::respond = std::ref(imu2);
    Dev<Chips::Qmi8658<Chips::Qmi8658Detail::AccelRange::g2>> q2{};
    check(runUntil(q2, [&] { return q2.samples() == 1; }, 500ms), "first sample");
    checkEq(imu2.word(0x03), 0x06U, "CTRL2: 2g at 125 Hz");
    checkEq(q2.latest().accel[0], 1'000'000, "16384 counts is 1 g at the 2g range");
    // the same 16384 counts on X, decoded at the 2g range the State holds
    static_assert([] {
        namespace QD = Chips::Qmi8658Detail;
        auto const f
          = frame(0x03, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
        constexpr Chips::Qmi8658<QD::AccelRange::g2>::State st{
          .deviceId   = 0x05,
          .accelRange = static_cast<std::uint8_t>(QD::AccelRange::g2),
          .gyroRange  = static_cast<std::uint8_t>(QD::GyroRange::dps1024)};
        auto const got = Chips::Qmi8658<QD::AccelRange::g2>::Motion::decode(Bytes{f}, st);
        return isOk(got) && equal(got.value.accel[0], 1'000'000);
    }());

    testCase("AXP2101: status bits, the ADC block, and the fuel gauge");
    fresh();
    RegisterModel<1> pmu{0x34};
    pmu.set(0x03, {0x4A});   // IC type
    // STATUS1 bit 3 battery present, bit 5 VBUS good; STATUS2 bits 7..5 = 001 charging,
    // bits 2..0 = 011 constant voltage, bit 3 clear so VBUS is usable.
    pmu.set(0x00, {0x28, 0x23});
    // 0x34..0x3D: battery 3900 (H5), TS, VBUS 5050, system 4200, die raw 7074 -> 32.00 degC
    pmu.set(0x34, {0x0F, 0x3C, 0x00, 0x00, 0x13, 0xBA, 0x10, 0x68, 0x1B, 0xA2});
    pmu.set(0xA4, {77});
    FakeBus::respond = std::ref(pmu);
    Dev<Chips::Axp2101> a{};
    check(runUntil(
            a,
            [&] { return a.samples<Chips::Axp2101::Power>() == 1; },
            500ms),
          "first read");
    check(a.identified(), "IC type 0x4A");
    checkEq(pmu.word(0x30), 0x1FU, "the four ADC channels are on, and TS keeps its reset 1");
    checkEq(pmu.word(0x68), 0x01U, "and the battery detector");
    checkEq(a.latest<Chips::Axp2101::Power>().battery,
            3900,
            "the model's frame reached the decode");
    // 0x13 0xBA at buffer 6..7: the ADC read at offset 2, past the two status bytes
    checkEq(a.latest<Chips::Axp2101::Power>().vbus, 5050, "the 0x34 read at offset 2");
    // STATUS1/2, then 0x34..0x3D: battery 3900 mV, VBUS 5050, system 4200, die raw 7074
    static_assert([] {
        auto const f
          = frame(0x28, 0x23, 0x0F, 0x3C, 0x00, 0x00, 0x13, 0xBA, 0x10, 0x68, 0x1B, 0xA2);
        auto const p = Chips::Axp2101::Power::decode(Bytes{f});
        return equal(p.battery, 3900) && equal(p.vbus, 5050) && equal(p.system, 4200)
            && equal(p.dieTemperature, Units::centiDegC(3200))          // 32.00 degC
            && p.batteryPresent() && p.vbusPresent() && !p.inVindpm()   // not current-limited
            && p.charging() && !p.discharging() && p.charge() == Chips::Axp2101::Charge::constantV;
    }());
    check(runUntil(a, [&] { return a.samples<Chips::Axp2101::Gauge>() == 1; }, 3s), "gauge");
    checkEq(a.latest<Chips::Axp2101::Gauge>(), 77, "the model's gauge byte reached the decode");
    // 77 is a percentage; 0xFF is the gauge's no-estimate reading, rejected
    static_assert([] {
        auto const good = frame(77);
        auto const none = frame(0xFF);
        auto const g    = Chips::Axp2101::Gauge::decode(Bytes{good});
        return isOk(g) && equal(g.value, 77)
            && isReject(Chips::Axp2101::Gauge::decode(Bytes{none}));
    }());

    testCase("AXP2101: the gauge rejects its own no-estimate reading");
    auto const rejectedBefore = a.rejected<Chips::Axp2101::Gauge>();
    auto const goodBefore     = a.samples<Chips::Axp2101::Gauge>();
    pmu.set(0xA4, {0xFF});
    check(runUntil(
            a,
            [&] { return a.rejected<Chips::Axp2101::Gauge>() > rejectedBefore; },
            4s),
          "0xFF is counted as a rejection");
    checkEq(a.samples<Chips::Axp2101::Gauge>(), goodBefore, "and is not a sample");
    checkEq(a.latest<Chips::Axp2101::Gauge>(), 77, "the last good percentage stands");

    testCase("PCF85063A: BCD time, the stop flag, and setting the clock");
    fresh();
    RegisterModel<1> rtc{0x51};
    rtc.set(0x04, {0x45, 0x59, 0x23, 0x31, 0x02, 0x12, 0x24});
    FakeBus::respond = std::ref(rtc);
    Dev<Chips::Pcf85063a<>> r{};
    check(runUntil(r, [&] { return r.samples<Chips::Pcf85063a<>::Time>() == 1; }, 200ms), "first");
    checkEq(r.latest<Chips::Pcf85063a<>::Time>().year,
            2024,
            "the model's frame reached the decode");
    // Tue 2024-12-31 23:59:45 in BCD, the oscillator running
    static_assert([] {
        auto const f = frame(0x45, 0x59, 0x23, 0x31, 0x02, 0x12, 0x24);
        auto const t = Chips::Pcf85063a<>::Time::decode(Bytes{f});
        return t.second == 45 && t.minute == 59 && t.hour == 23 && t.day == 31 && t.weekday == 2
            && t.month == 12 && t.year == 2024 && t.integrity;
    }());
    checkEq(rtc.word(0x00),
            0x01U,
            "Control_1 at bring-up: running, 24 hour, and the 12.5 pF crystal this board fits");

    rtc.set(0x04, {0xC5, 0x59, 0x23, 0x31, 0x02, 0x12, 0x24});   // seconds bit 7
    check(runUntil(
            r,
            [&] { return !r.latest<Chips::Pcf85063a<>::Time>().integrity; },
            3s),
          "the stop flag is reported, and the seconds still decode");
    // seconds bit 7 set: the stop flag, and 0xC5 masked to 45
    static_assert([] {
        auto const f = frame(0xC5, 0x59, 0x23, 0x31, 0x02, 0x12, 0x24);
        auto const t = Chips::Pcf85063a<>::Time::decode(Bytes{f});
        return !t.integrity && t.second == 45;
    }());

    r.set<Chips::Pcf85063a<>::SetTime>({0, 30, 12, 1, 5, 2, 2025, true});
    check(runUntil(
            r,
            [&] { return !r.pending() && r.writes<Chips::Pcf85063a<>::SetTime>() == 1; },
            200ms),
          "written");
    check(rtc.word(0x04) == 0x00 && rtc.word(0x05) == 0x30 && rtc.word(0x06) == 0x12
            && rtc.word(0x07) == 0x01 && rtc.word(0x08) == 0x05 && rtc.word(0x09) == 0x02
            && rtc.word(0x0A) == 0x25,
          "Wed 2025-02-01 12:30:00 as BCD, and the year back to two digits");
    {
        auto const stop  = findWrite({0x00, 0x21});
        auto const time  = findWrite({0x04, 0x00, 0x30, 0x12, 0x01, 0x05, 0x02, 0x25}, stop);
        auto const start = findWrite({0x00, 0x01}, time);
        check(stop < time && time < start && start < FakeBus::log.size(),
              "STOP set, the time, STOP released");
    }
    if(failures != 0) { dump(); }
}

// -- the ES8311 codec, the board's fourth I2C part -----------------------------------------

namespace Es8311Test {
    namespace ED = Kvasir::I2C::Chips::Es8311Detail;

    // The clock coefficients are a compile-time lookup: a pair with no row does not build. These
    // pin the two the AMOLED board can reach from a 200 MHz PIO.
    static_assert(ED::coefficientsFor(Units::hertz(6'144'000),
                                      Units::hertz(48'000))
                      .preMulti
                    == 0x01,
                  "6.144 MHz at 48 kHz");
    static_assert(ED::coefficientsFor(Units::hertz(6'144'000),
                                      Units::hertz(24'000))
                      .preDiv
                    == 0x01,
                  "and at 24 kHz");
    static_assert(ED::coefficientsFor(Units::hertz(12'288'000),
                                      Units::hertz(48'000))
                      .preDiv
                    == 0x01,
                  "12.288 MHz at 48 kHz");

    // 0xBF is 0 dB and the ladder is half a decibel a step (datasheet REGISTER 0X32).
    static_assert(ED::volumeForDb(0.0) == 0xBF,
                  "unity");
    static_assert(ED::volumeForDb(-0.5) == 0xBE,
                  "half a dB down");
    static_assert(ED::volumeForDb(-30.0) == 0xBF - 60,
                  "the quiet default this description ships");
    static_assert(ED::volumeForDb(+32.0) == 0xFF,
                  "the top of the range");

    // Master and slave differ by one bit of REG00, which is the whole of the topology choice.
    using Master = Chips::Es8311<Units::hertz(6'144'000), Units::hertz(24'000)>;
    using Slave
      = Chips::Es8311<Units::hertz(6'144'000), Units::hertz(24'000), ED::Role::codecSlave>;
    static_assert(Master::Reg00Run == 0xC0,
                  "CSM_ON | MSC");
    static_assert(Slave::Reg00Run == 0x80,
                  "CSM_ON alone");
    static_assert(Master::Reg09 == 0x0C,
                  "16-bit I2S into the DAC");

    // The master-mode dividers follow MCLK / fs, not the vendor table's 256 fs for every row.
    using Master16k = Chips::Es8311<Units::hertz(12'288'000), Units::hertz(16'000)>;
    static_assert(Master16k::LrckDiv == 767 && Master16k::BclkCode == 21,
                  "768 fs: LRCK / 768, BCLK / 24 is 32 bits a frame (code 21 is / 24)");
    using Master8k24 = Chips::Es8311<Units::hertz(12'288'000),
                                     Units::hertz(8'000),
                                     ED::Role::codecMaster,
                                     ED::WordLength::bits24>;
    static_assert(Master8k24::LrckDiv == 1535 && Master8k24::BclkCode == 21,
                  "1536 fs with 24-bit words: 64 bits a frame, BCLK / 24");
}   // namespace Es8311Test

void es8311() {
    using namespace Es8311Test;

    testCase("ES8311: identified, and brought up in the vendor's order");
    fresh();
    RegisterModel<1> codec{0x18};
    codec.set(0xFD, {0x83, 0x11});
    FakeBus::respond = std::ref(codec);
    Dev<Master> c{};
    check(runUntil(c, [&] { return c.answering(); }, 500ms), "up");
    check(c.identified(), "chip ID 0x83 0x11");
    // Two single-register reads: the part's pointer does not auto-increment. The fake
    // bus's RegisterModel does increment, so only the transaction shape can catch a regression
    // here.
    checkEq(std::ranges::count_if(FakeBus::log,
                                  [](auto const& t) {
                                      return t.isBus() && t.isRead() && !t.sent.empty()
                                          && (t.sent[0] == 0xFD || t.sent[0] == 0xFE);
                                  }),
            2,
            "the two chip-ID registers are read one at a time");

    // The clock manager, from the 6.144 MHz / 48 kHz row.
    checkEq(codec.word(0x01), 0x3FU, "every clock domain on, MCLK from the MCLK pin");
    checkEq(codec.word(0x02), 0x00U, "pre_div 1, and no clock doubler");
    checkEq(codec.word(0x03), 0x10U, "ADC OSR 0x10, fs_mode normal");
    checkEq(codec.word(0x04), 0x10U, "DAC OSR");
    checkEq(codec.word(0x05), 0x00U, "adc_div and dac_div both 1");
    checkEq(codec.word(0x06), 0x07U, "bclk_div 8, which is 32 BCLK a frame");
    checkEq(codec.word(0x07), 0x00U, "LRCK divider high bits");
    checkEq(codec.word(0x08), 0xFFU, "LRCK divider low byte: MCLK / 256");
    checkEq(codec.word(0x00), 0xC0U, "CSM on and the codec is the serial-port master");
    checkEq(codec.word(0x09), 0x0CU, "16-bit I2S, left slot to the DAC");
    checkEq(codec.word(0x0D), 0x01U, "analogue reference and bias up, VMID charging");
    checkEq(codec.word(0x13), 0x10U, "HPSW: the headphone driver, not the line-out default");

    testCase("ES8311: it comes up muted and quiet, not at the vendor's -3 dB");
    checkEq(codec.word(0x31), 0x60U, "DSM and DEM mutes both set");
    checkEq(codec.word(0x32), 0x83U, "-30 dB");

    testCase("ES8311: volume, mute and fade are write groups");
    c.set<Master::Volume>(ED::volumeForDb(-12.0));
    c.set<Master::Mute>(Master::Mute::Value::playing);
    c.set<Master::Fade>(4);
    check(runUntil(c, [&] { return !c.pending(); }, 200ms), "written");
    checkEq(codec.word(0x32), 0xA7U, "-12 dB");
    checkEq(codec.word(0x31), 0x00U, "unmuted");
    checkEq(codec.word(0x37), 0x48U, "fade 4, DAC equaliser still bypassed");

    testCase("ES8311: slave mode differs by exactly one bit");
    fresh();
    RegisterModel<1> slaveCodec{0x18};
    slaveCodec.set(0xFD, {0x83, 0x11});
    FakeBus::respond = std::ref(slaveCodec);
    Dev<Slave> sc{};
    check(runUntil(sc, [&] { return sc.answering(); }, 500ms), "up");
    checkEq(slaveCodec.word(0x00), 0x80U, "CSM on, MSC clear: the host drives BCLK and LRCK");
    checkEq(slaveCodec.word(0x09), 0x0CU, "and the serial port is configured the same either way");

    testCase("ES8311: the Status read-back, ten one-byte reads each at its own buffer offset");
    {
        // A distinct byte in every register the group reads, fixed against writes, so a read
        // step that landed at another offset shows up as another register's value.
        std::uint8_t v = 0xA0;
        for(std::uint32_t const reg :
            {0x00U, 0x01U, 0x09U, 0x0DU, 0x0EU, 0x12U, 0x13U, 0x31U, 0x32U, 0x37U})
        {
            slaveCodec.set(reg, {v++});
            slaveCodec.readOnly.push_back(reg);
        }
        auto const before = sc.samples();
        check(runUntil(
                sc,
                [&] { return sc.samples() > before + 1; },
                3s),
              "a Status read of the new bytes");
        auto const& st = sc.latest();
        checkEq(st.reg00, std::uint8_t{0xA0}, "0x00 at offset 0");
        checkEq(st.reg01, std::uint8_t{0xA1}, "0x01 at offset 1");
        checkEq(st.reg09, std::uint8_t{0xA2}, "0x09 at offset 2");
        checkEq(st.reg0D, std::uint8_t{0xA3}, "0x0D at offset 3");
        checkEq(st.reg0E, std::uint8_t{0xA4}, "0x0E at offset 4");
        checkEq(st.reg12, std::uint8_t{0xA5}, "0x12 at offset 5");
        checkEq(st.reg13, std::uint8_t{0xA6}, "0x13 at offset 6");
        checkEq(st.reg31, std::uint8_t{0xA7}, "0x31 at offset 7");
        checkEq(st.reg32, std::uint8_t{0xA8}, "0x32 at offset 8");
        checkEq(st.reg37, std::uint8_t{0xA9}, "0x37 at offset 9");
    }

    testCase("ES8311: a part that is not one is not adopted, and asked again a second later");
    fresh();
    RegisterModel<1> wrong{0x18};
    wrong.set(0xFD, {0x83, 0x22});   // ES8388's second ID byte
    FakeBus::respond = std::ref(wrong);
    Dev<Master> w{};
    check(runUntil(
            w,
            [&] { return w.unidentified() == 1; },
            2s),
          "the bring-up read the id and turned it down");
    check(!w.identified() && w.link() == Link::starting, "not an ES8311: starting, not answering");
    checkEq(Log::warnings, 1, "said once");
    {
        // Nothing else goes to it: no read group, and none of the write groups' Initial.
        auto const reads  = wrong.reads;
        auto const writes = wrong.writes;
        runFor(w, 900ms);
        check(wrong.reads == reads && wrong.writes == writes,
              "and nothing is sent to it meanwhile");
        check(runUntil(
                w,
                [&] { return wrong.reads > reads; },
                300ms),
              "the bring-up is tried again a second after the last");
        check(runUntil(w, [&] { return w.unidentified() == 2; }, 2s), "and turned down again");
        checkEq(Log::warnings, 1, "without a second line");
    }
    // A part that gets its id right on a later bring-up -- it was still booting -- comes up.
    wrong.set(0xFD, {0x83, 0x11});
    check(runUntil(w, [&] { return w.answering(); }, 3s), "up once the id is right");
    check(w.identified(), "identified");
    checkEq(wrong.word(0x00), 0xC0U, "and configured: CSM on, master");
    checkEq(w.unidentified(), 2U, "after two bring-ups that were turned down");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    ht16k33();
    lcd1602();
    ssd1306();
    amoledBoard();
    es8311();
    return finish();
}
