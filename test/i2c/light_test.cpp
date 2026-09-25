/// Light, colour, proximity and the LED drivers: each description against a model of the
/// part on the wire.
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

void bh1750() {
    testCase("BH1750");
    fresh();
    std::vector<std::uint8_t> reading{0x04, 0xB0};   // 1200 -> 1000 lx
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x23) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) { return FakeBus::Result::succeeded; }
            for(std::size_t i = 0; i < recv.size(); ++i) {
                recv[i] = static_cast<std::byte>(reading[i]);
            }
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Bh1750> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first sample");
    check(writes()
            == std::vector<std::vector<std::uint8_t>>{{0x01}, {0x07}, {0x42}, {0x65}, {0x10}},
          "power on, reset, MTreg 69 in its two halves, continuous H");
    // the first read waits the 180 ms measurement
    check(FakeBus::log[5].at - FakeBus::log[4].at >= 180ms, "180 ms before the first read");
    checkEq(d.latest().raw, 1200U, "the model's frame reached the decode");
    // 0x04B0 is 1200 counts, 1000 lx at the H resolution
    static_assert([] {
        auto const f = frame(0x04, 0xB0);
        auto const s = Chips::Bh1750::Light::decode(Bytes{f});
        return equal(s.raw, 1200U) && equal(s.lux(), 1000000U);
    }());
    check(d.fresh(), "fresh once");
    check(!d.fresh(), "then not");
    auto const n = d.samples();
    runFor(d, 1s);
    checkEq(d.samples() - n, 5U, "five samples a second");
    if(failures != 0) { dump(); }
}

void veml6030() {
    testCase("VEML6030");
    fresh();
    RegisterModel<1, 2> m{0x48};
    // wire order is LSB first: register 0x07 holds bytes {0x81, 0xD4}
    m.set(0x07, {0x81, 0xD4});
    m.set(0x04, {0xE8, 0x03});   // 1000 counts
    m.set(0x05, {0xD0, 0x07});   // 2000 counts
    m.readOnly       = {0x04, 0x05, 0x07};
    FakeBus::respond = std::ref(m);
    Dev<Chips::Veml6030<>, At<0x48>> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first sample");
    check(d.identified(), "ID low byte 0x81 accepted");
    checkEq(d.state().deviceId, 0xD481U, "ID kept");
    check(m.word(0x00) == 0x0010, "ALS_CONF: gain 1/8 (10b << 11) = 0x1000 as LE bytes 00 10");
    checkEq(d.latest().white, 2000U, "WHITE, read into offset 2, reached the decode");
    // ALS 1000 and WHITE 2000, little endian, at gain 1/8 and 100 ms: 0.5376 lx a count
    static_assert([] {
        auto const f   = frame(0xE8, 0x03, 0xD0, 0x07);
        auto const got = Chips::Veml6030<>::Light::decode(Bytes{f}, Chips::Veml6030<>::State{});
        return equal(got.als, 1000U) && equal(got.white, 2000U) && equal(got.lux(), 537600);
    }());
    if(failures != 0) { dump(); }
}

void opt3001() {
    testCase("OPT3001");
    fresh();
    RegisterModel<1, 2> opt{0x44};
    opt.set(0x7E, {0x54, 0x49});   // manufacturer "TI"
    opt.set(0x7F, {0x30, 0x01});   // device id
    opt.set(0x00, {0x51, 0x23});   // exponent 5, mantissa 0x123: 0.01 x 32 x 291 = 93.12 lx
    opt.readOnly     = {0x00, 0x7E, 0x7F};
    FakeBus::respond = std::ref(opt);
    Dev<Chips::Opt3001<>> d{};
    check(runUntil(d, [&] { return d.valid(); }, 3s), "first sample");
    check(d.identified(), "5449h / 3001h accepted");
    checkEq(d.state().manufacturerId, 0x5449U, "manufacturer id kept");
    checkEq(opt.word(0x01), 0xCE00U, "configuration: auto range, 800 ms, continuous");
    checkEq(d.latest().raw, 0x5123U, "the model's frame reached the decode");
    checkEq(Units::value(d.latest().lux()), 93120U, "93.12 lx");
    static_assert([] {
        auto const f = frame(0x51, 0x23);
        auto const s = Chips::Opt3001<>::Light::decode(Bytes{f});
        return equal(s.exponent(), 5) && equal(s.mantissa(), 0x123) && equal(s.lux(), 93120U)
            && !s.saturated();
    }());
    // E 0 is the 10 mlx LSB; the top of the automatic range is 83 865.6 lx and marks itself.
    static_assert([] {
        auto const one = frame(0x00, 0x01);
        auto const top = frame(0xBF, 0xFF);
        return equal(Chips::Opt3001<>::Light::decode(Bytes{one}).lux(), 10U)
            && equal(Chips::Opt3001<>::Light::decode(Bytes{top}).lux(), 83865600U)
            && Chips::Opt3001<>::Light::decode(Bytes{top}).saturated();
    }());
    // CT picks the conversion time, the read period trails it by 50 ms.
    static_assert(Chips::Opt3001<>::Light::Period == 850ms
                  && Chips::Opt3001<Chips::Opt3001Detail::ConversionTime::ms100>::Light::Period
                       == 150ms
                  && Chips::Opt3001<Chips::Opt3001Detail::ConversionTime::ms100>::Conf == 0xC600);
    if(failures != 0) { dump(); }
}

void opt4048() {
    testCase("OPT4048");
    fresh();
    RegisterModel<1, 2> opt{0x44};
    opt.set(0x11, {0x08, 0x21});   // DEVICE_ID: DIDL 0x2, DIDH 0x21
    // CH1 (Y): exponent 2, mantissa 0x12345 -> code 0x48D14
    opt.set(0x02, {0x21, 0x23});   // exp 2, RESULT_MSB 0x123
    opt.set(0x03, {0x45, 0x30});   // RESULT_LSB 0x45, counter 3
    opt.readOnly     = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x11};
    FakeBus::respond = std::ref(opt);
    Dev<Chips::Opt4048<>> op{};
    check(runUntil(op, [&] { return op.valid(); }, 3s), "first sample");
    check(op.identified(), "device id 0x821");
    checkEq(opt.word(0x0A), 0x3230U, "auto-range, 100 ms, continuous");
    checkEq(op.latest().y(), 0x48D14U, "the model's frame reached the decode");
    // the 16-byte burst with only CH1 set: exponent 2, mantissa 0x12345, counter 3, CRC 0
    static_assert([] {
        auto const f = frame(0x00,
                             0x00,
                             0x00,
                             0x00,
                             0x21,
                             0x23,
                             0x45,
                             0x30,
                             0x00,
                             0x00,
                             0x00,
                             0x00,
                             0x00,
                             0x00,
                             0x00,
                             0x00);
        auto const got
          = Chips::Opt4048<>::Colour::decode(Bytes{f}, Chips::Opt4048<>::Colour::Sample{});
        return isOk(got) && equal(got.value.exponent[1], 2) && equal(got.value.counter[1], 3)
            && equal(got.value.y(), 0x48D14U)
            && equal(got.value.lux(), 641259U);   // code x 2.15e-3 = 641.259 lux
    }());
    // Counters wrap at 16: a frame on the same counters is the same conversion only when its
    // readings match too. The all-zero frame (counters 0, CRC 0) against a sample that read
    // light on CH1 under the same counters is new; against itself it is unchanged.
    static_assert([] {
        using S      = Chips::Opt4048<>::Colour::Sample;
        auto const f = frame(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        S          lit{};
        lit.code[1] = 0x48D14U;
        return isOk(Chips::Opt4048<>::Colour::decode(Bytes{f}, lit))
            && isUnchanged(Chips::Opt4048<>::Colour::decode(Bytes{f}, S{}));
    }());
    if(failures != 0) { dump(); }
}

void ltr390() {
    testCase("LTR390");
    fresh();
    RegisterModel<1> lt{0x53};
    lt.set(0x06, {0xB2});               // PART_ID, upper nibble 0xB
    lt.set(0x07, {0x08});               // MAIN_STATUS: ALS/UVS data ready
    lt.set(0x0D, {0xE8, 0x03, 0x00});   // ALS = 1000
    lt.set(0x10, {0xBE, 0x03, 0x00});   // UVS = 958
    lt.readOnly = {0x06, 0x07, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12};
    // The part resets before it acknowledges SW_RESET (0x00 = 0x10): that write NAKs.
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(s.size() == 2 && s[0] == std::byte{0x00} && s[1] == std::byte{0x10}) {
            return FakeBus::Result::notAcknowledged;
        }
        return lt(a, s, r);
    };
    Dev<Chips::Ltr390<>> lr{};
    check(runUntil(lr, [&] { return lr.valid(); }, 2s), "first sample");
    check(lr.identified(), "PART_ID upper nibble 0xB");
    checkEq(lr.errors(), 0U, "the NAK of the software reset is not an error");
    check(hasWrite({0x00, 0x02}) && hasWrite({0x00, 0x0A}),
          "one pass in ALS mode and one in UVS mode");
    checkEq(lr.latest().als, 1000U, "ALSDATA, read into offset 1, reached the decode");
    checkEq(lr.latest().uvs, 958U, "UVSDATA, read into offset 5, reached the decode");
    // MAIN_STATUS and ALSDATA = 1000 from the ALS pass, then MAIN_STATUS and UVSDATA = 958;
    // lux = 1000 x 0.6 / (3 x 1.0) = 200; UVI = 958 / ((3/18) x (2^18/2^20) x 2300)
    static_assert([] {
        auto const f   = frame(0x08, 0xE8, 0x03, 0x00, 0x08, 0xBE, 0x03, 0x00);
        auto const got = Chips::Ltr390<>::Light::decode(Bytes{f}, Chips::Ltr390<>::State{});
        return isOk(got) && equal(got.value.als, 1000U) && equal(got.value.uvs, 958U)
            && equal(got.value.lux(), 200000U) && equal(got.value.uvi(), 9996U);
    }());
    if(failures != 0) { dump(); }
}

void ltr507() {
    testCase("LTR-507ALS");
    fresh();
    RegisterModel<1> l507{0x3A};
    l507.set(0x86, {0x91});         // PART_ID
    l507.set(0x87, {0x05});         // MANUFAC_ID
    l507.set(0x8A, {0x05});         // ALS valid (bit 2) and PS valid (bit 0)
    l507.set(0x88, {0x34, 0x12});   // ALS = 0x1234
    l507.set(0x8B, {0xFF, 0x07});   // PS = 0x7FF, the 11-bit maximum
    l507.readOnly    = {0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C};
    FakeBus::respond = std::ref(l507);
    Dev<Chips::Ltr507<>> l5{};
    check(runUntil(l5, [&] { return l5.valid(); }, 800ms), "first sample");
    check(l5.identified(), "PART_ID 0x91, MANUFAC_ID 0x05");
    // Mode is bit 1 of both control registers; bit 0 of ALS_CONTR is reserved and leaves the
    // ALS in standby.
    check(hasWrite({0x80, 0x02}) && hasWrite({0x81, 0x0E}),
          "ALS and PS both put in active mode, PS gain at the 11 the part requires");
    check(hasWrite({0x82, 0x6C}), "PS_LED with the duty field at the 01 the part requires");
    check(hasWrite({0x85, 0x80}), "ALS_MEAS_RATE 16 bit, 75 ms, every 100 ms");
    checkEq(l5.latest().light, 0x1234U, "ALS_DATA, read into offset 1, reached the decode");
    checkEq(l5.latest().proximity, 0x07FFU, "PS_DATA, read into offset 3, reached the decode");
    // ALS_PS_STATUS with ALS and PS valid, ALS_DATA 0x1234, PS_DATA 0x7FF
    static_assert([] {
        auto const f   = frame(0x05, 0x34, 0x12, 0xFF, 0x07);
        auto const got = Chips::Ltr507<>::Light::decode(Bytes{f}, Chips::Ltr507<>::State{});
        return isOk(got) && equal(got.value.light, 0x1234U)
            && equal(got.value.proximity, 0x07FFU)   // masked to eleven bits
            && got.value.proximityValid() && got.value.proximityNew();
    }());
    // lux per count by range: 1, 0.5, 0.01 and 0.005 lux (datasheet 6.1)
    static_assert([] {
        Chips::Ltr507<>::Light::Sample s{};
        s.light       = 1000;
        s.gain        = 2;
        auto const r3 = s.lux();
        s.gain        = 3;
        return equal(r3, Units::milliLux(10000U)) && equal(s.lux(), Units::milliLux(5000U));
    }());
    // the same frame with the upper PS_DATA bits set: still eleven bits, and bit 4 of
    // PS_DATA_1 is the overflow flag (6.11); PS status bit 0 clear is data already read
    static_assert([] {
        auto const f   = frame(0x05, 0x34, 0x12, 0xFF, 0xFF);
        auto const got = Chips::Ltr507<>::Light::decode(Bytes{f}, Chips::Ltr507<>::State{});
        auto const old = frame(0x04, 0x34, 0x12, 0xFF, 0x07);
        auto const was = Chips::Ltr507<>::Light::decode(Bytes{old}, Chips::Ltr507<>::State{});
        return isOk(got) && equal(got.value.proximity, 0x07FFU) && !got.value.proximityValid()
            && isOk(was) && was.value.proximityValid() && !was.value.proximityNew();
    }());
    // the ALS status bit clear is old data: nothing new, whatever ALS_DATA says
    static_assert([] {
        auto const f = frame(0x00, 0xFF, 0xFF, 0xFF, 0x07);
        return isUnchanged(Chips::Ltr507<>::Light::decode(Bytes{f}, Chips::Ltr507<>::State{}));
    }());
    // the status bit gates the reading: with it clear the run reports nothing, and is no fault
    auto const before = l5.samples();
    l507.set(0x8A, {0x00});
    l507.set(0x88, {0xFF, 0xFF});
    runFor(l5, 400ms);
    checkEq(l5.samples(), before, "no new sample while the ALS status bit is clear");
    checkEq(l5.rejected(), 0U, "and old data is not a rejection");
    checkEq(l5.latest().light, 0x1234U, "and the previous reading is still the one on offer");
    if(failures != 0) { dump(); }
}

void apds9960() {
    testCase("APDS-9960");
    fresh();
    RegisterModel<1> apd{0x39};
    apd.set(0x92, {0xAB});
    apd.set(0x93, {0x03});   // AVALID | PVALID
    apd.set(0x94, {0xE8, 0x03, 0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01});
    apd.set(0x9C, {0x7F});
    apd.readOnly     = {0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C};
    FakeBus::respond = std::ref(apd);
    Dev<Chips::Apds9960<>> ap{};
    check(runUntil(ap, [&] { return ap.valid(); }, 500ms), "first sample");
    check(ap.identified(), "ID 0xAB");
    check(hasWrite({0x81, 0xF6}) && hasWrite({0x80, 0x07}),
          "ATIME = 256 - 10, then PON | AEN | PEN");
    checkEq(ap.latest().clear, 1000U, "the colour block, read into offset 1, reached the decode");
    checkEq(ap.latest().blue, 300U, "all eight bytes of it");
    checkEq(ap.latest().proximity,
            std::uint8_t{0x7F},
            "PDATA, read into offset 9, reached the decode");
    // STATUS AVALID | PVALID, clear red green blue little endian, then proximity
    static_assert([] {
        auto const f   = frame(0x03, 0xE8, 0x03, 0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01, 0x7F);
        auto const got = Chips::Apds9960<>::Colour::decode(Bytes{f});
        return equal(got.clear, 1000U) && equal(got.red, 100U) && equal(got.green, 200U)
            && equal(got.blue, 300U) && equal(got.proximity, 0x7F) && got.colourValid()
            && got.proximityValid() && !got.saturated();
    }());
    // CPSAT latches until CICLEAR is addressed: every read ends with it
    {
        auto const from = FakeBus::log.size();
        apd.set(0x93, {0x83});   // saturated
        check(runUntil(ap, [&] { return ap.latest().saturated(); }, 500ms), "CPSAT seen");
        bool cleared = false;
        for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
            auto const& t = FakeBus::log[i];
            cleared       = cleared || (t.isWrite() && t.sent == std::vector<std::uint8_t>{0xE6});
        }
        check(cleared, "CICLEAR (0xE6) addressed with nothing written");
    }
    if(failures != 0) { dump(); }
}

// Integration time in, register codes out, rounded to the nearest step.
struct ApdsSlow {
    static constexpr auto IntegrationTime = 100ms;
};

struct As7341Datasheet {
    static constexpr unsigned Atime = 29, Astep = 599;
};

struct As7341Long {
    static constexpr auto IntegrationTime = 2s;
};

static_assert(Chips::Apds9960<>::Cycles == 10 && Chips::Apds9960<>::Atime == 0xF6
                && Chips::Apds9960<>::IntegrationTime == 27'800us
                && Chips::Apds9960<>::IntegrationWait == 28ms,
              "the default: ten cycles, as before");
static_assert(Chips::Apds9960<Chips::Apds9960Detail::Gain::x4,
                              ApdsSlow>::Cycles
                  == 36
                && Chips::Apds9960<Chips::Apds9960Detail::Gain::x4,
                                   ApdsSlow>::IntegrationTime
                     == 100'080us,
              "100 ms is 35.97 cycles: 36");
static_assert(Chips::As7341<>::Atime == 0 && Chips::As7341<>::Astep == 17'985
                && Chips::As7341<>::IntegrationTime == 50'002us,
              "50 ms with the smallest ATIME: 17986 x 2.78 us");
static_assert(Chips::As7341<Chips::As7341Detail::Gain::x256,
                            As7341Datasheet>::Atime
                  == 29
                && Chips::As7341<Chips::As7341Detail::Gain::x256,
                                 As7341Datasheet>::Astep
                     == 599
                && Chips::As7341<Chips::As7341Detail::Gain::x256,
                                 As7341Datasheet>::IntegrationTime
                     == 50'040us,
              "the codes as given");
static_assert(Chips::As7341<Chips::As7341Detail::Gain::x256,
                            As7341Long>::Atime
                  == 10
                && Chips::As7341<Chips::As7341Detail::Gain::x256,
                                 As7341Long>::Astep
                     == 65'401,
              "2 s: ATIME 9 would need 71942 steps, ATIME 10 needs 65402");

void as7341() {
    testCase("AS7341");
    fresh();
    RegisterModel<1> as{0x39};
    as.set(0x92, {0x09 << 2});
    // six ADCs; the SMUX decides which photodiode each one is. The first byte of the SMUX
    // table says which pass is running (0x30 for F1F4_Clear_NIR, 0x00 for F5F8_Clear_NIR), and
    // each pass reads its own counts: 1..6 in the first, 11..16 in the second.
    as.set(0x95, {0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00});
    as.onWrite = [&](std::uint32_t reg, RegisterModel<1>::Reg const& value) {
        if(reg != 0x00) { return; }
        if(value[0] == 0x30) {
            as.set(0x95, {0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00});
        } else {
            as.set(0x95, {0x0B, 0x00, 0x0C, 0x00, 0x0D, 0x00, 0x0E, 0x00, 0x0F, 0x00, 0x10, 0x00});
        }
    };
    as.set(0xA3, {0x00});   // STATUS2: AVALID once the integration has run
    as.readOnly = {0x92, 0xA3};
    // SMUXEN clears itself when the SMUX command is done; AVALID sets when SP_EN has run
    int busyReadBacks = 1;   // the very first SMUX read-back finds it still running
    FakeBus::respond  = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        auto const res = as(a, s, r);
        if(s.size() == 2 && s[0] == std::byte{0x80}) {
            as.set(0xA3, {s[1] == std::byte{0x03} ? std::uint8_t{0x40} : std::uint8_t{0x00}});
        }
        if(s.size() == 1 && s[0] == std::byte{0x80} && r.size() == 1) {
            r[0] = busyReadBacks-- > 0 ? std::byte{0x11} : std::byte{0x01};
        }
        return res;
    };
    Dev<Chips::As7341<>> a7{};
    check(runUntil(a7, [&] { return a7.valid(); }, 1s), "a full spectrum");
    check(a7.identified(), "the chip id in bits 7:2 is 0x09");
    check(hasWrite({0xAF, 0x10}), "CFG6 says a SMUX configuration follows");
    check(hasWrite({0x00, 0x30, 0x01, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00}),
          "the F1F4_Clear_NIR mapping, first eight bytes");
    check(hasWrite({0x00, 0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x10, 0x03}),
          "then the F5F8_Clear_NIR one");
    check(hasWrite({0x80, 0x11}), "SMUXEN latches each mapping");
    check(hasWrite({0x80, 0x03}), "and SP_EN runs the integration");
    check(hasWrite({0xA9, 0x00}), "CFG0 at bring-up: REG_BANK 0");
    check(a7.samples() >= 1 && a7.rejected() == 0 && busyReadBacks < 0,
          "a SMUX still busy at its read-back ran the pass again rather than being decoded");
    checkEq(a7.latest().f[0], 1U, "the first pass, read into offset 0, reached the decode");
    checkEq(a7.latest().f[4], 11U, "the second pass, read into offset 12, reached the decode");
    checkEq(a7.latest().nir, 16U, "and its last ADC");
    // the six ADCs, 1..6 from the first pass at offset 0 and 11..16 from the second at 12
    static_assert([] {
        auto const f    = frame(0x01,
                                0x00,
                                0x02,
                                0x00,
                                0x03,
                                0x00,
                                0x04,
                                0x00,
                                0x05,
                                0x00,
                                0x06,
                                0x00,
                                0x0B,
                                0x00,
                                0x0C,
                                0x00,
                                0x0D,
                                0x00,
                                0x0E,
                                0x00,
                                0x0F,
                                0x00,
                                0x10,
                                0x00,
                                0x01,    // ENABLE after the first SMUX: SMUXEN clear
                                0x01,    // and after the second
                                0x40,    // STATUS2 AVALID, first pass
                                0x40);   // second pass
        auto const out  = Chips::As7341<>::Spectrum::decode(Bytes{f});
        auto const got  = out.value;
        auto       busy = f;
        busy[25]        = std::byte{0x11};   // the second SMUX not done
        auto nodata     = f;
        nodata[26]      = std::byte{0x00};        // the first pass without AVALID
        return isOk(out) && equal(got.f[0], 1U)   // F1 is ADC0 of the first pass
            && equal(got.f[3], 4U)                // F4 is ADC3 of the first pass
            && equal(got.f[4], 11U)               // F5 is ADC0 of the second
            && equal(got.f[7], 14U)               // F8 is ADC3 of the second
            && equal(got.clear, 15U)              // clear is ADC4, from the fresher second pass
            && equal(got.nir, 16U)                // NIR is ADC5
            && isRetry(Chips::As7341<>::Spectrum::decode(Bytes{busy}))
            && isRetry(Chips::As7341<>::Spectrum::decode(Bytes{nodata}));
    }());
    if(failures != 0) { dump(); }
}

void ktd2026() {
    testCase("KTD2026");
    fresh();
    RegisterModel<1> ktd{0x30};
    // "Reset Complete Chip" is not acknowledged (datasheet, the note under "Write").
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(s.size() == 2 && s[0] == std::byte{0x00} && s[1] == std::byte{0x07}) {
            return FakeBus::Result::notAcknowledged;
        }
        return ktd(a, s, r);
    };
    Dev<Chips::Ktd2026<>> kt{};
    check(runUntil(kt, [&] { return kt.answering() && hasWrite({0x00, 0x1C}); }, 800ms), "up");
    check(writes()
            == std::vector<std::vector<std::uint8_t>>{{0x00, 0x00}, {0x00, 0x07}, {0x00, 0x1C}},
          "probe, the complete reset, then always on");
    checkEq(kt.errors(), 0U, "the reset's NAK is not an error");
    for(auto const& w : writes()) { check(w.size() == 2, "one register per transaction"); }
    kt.set<Chips::Ktd2026<>::Colour>({255, 0, 128});
    check(runUntil(
            kt,
            [&] { return kt.writes<Chips::Ktd2026<>::Colour>() >= 1; },
            200ms),
          "written");
    // red on, blue on, green off; a current register counts from zero and tops out at 191
    // (24 mA), so 255 is clamped: 0xBF, 128 is 0x7F, in the chip's R B G order
    {
        auto const ramp = findWrite({0x05, 0x00});
        auto const d1   = findWrite({0x06, 0xBF}, ramp);
        auto const d2   = findWrite({0x07, 0x7F}, d1);
        auto const d3   = findWrite({0x08, 0x00}, d2);
        auto const ctrl = findWrite({0x04, 0x05}, d3);
        check(ctrl < FakeBus::log.size(),
              "no ramp, red (clamped to 191), blue and green, then channel control 0x05");
    }
    kt.set<Chips::Ktd2026<>::Colour>({0, 0, 0});
    check(runUntil(
            kt,
            [&] { return hasWrite({0x04, 0x00}); },
            200ms),
          "all zero turns every channel off");
    if(failures != 0) { dump(); }
}

void ktd2061() {
    testCase("KTD2061");
    fresh();
    RegisterModel<1> ktd{0x68};
    ktd.set(0x00, {0xA4});   // ID: VENDOR 101, DIE_ID 00100
    ktd.set(0x01, {0x00});   // Monitor: nothing to report
    ktd.readOnly     = {0x00, 0x01};
    FakeBus::respond = std::ref(ktd);
    using Chip       = Chips::Ktd2061<>;
    Dev<Chip> kt{};
    check(runUntil(kt, [&] { return kt.answering(); }, 1s), "up");
    // Control: night mode (01), BrightExtend off, CoolExtend 90 C (11), fade 1 s (101) = 0x5D,
    // which is what the previous driver wrote.
    checkEq(ktd.word(0x02), 0x5DU, "control");
    static_assert(Chip::ControlValue == 0x5D);

    // Twelve LEDs, two to a select register, the first of a pair in the high nibble (ISELA12:
    // ENA1 is bit 7). On plus all three channels from slot 0 is 0x8; on with red from slot 1 is
    // 0xC.
    static_assert([] {
        Chip::Select::Value b{};
        b = Chip::with(b, 1, {.on = true});
        b = Chip::with(b, 2, {.on = true, .red = true});
        b = Chip::with(b, 12, {.on = true, .green = true, .blue = true});
        return equal(b[0], 0x8C) && equal(b[5], 0x0B);
    }());
    static_assert(Chip::with({}, Chip::Module::C4, {.on = true})
                  == Chip::with({}, 12, {.on = true}));
    // and changing one LED leaves the other in its register alone
    static_assert([] {
        Chip::Select::Value b{};
        b = Chip::with(b, 3, {.on = true, .blue = true});
        b = Chip::with(b, 4, {.on = true});
        b = Chip::with(b, 3, Chip::Off);
        return equal(b[1], 0x08);
    }());

    kt.set<Chip::Colour>({0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF});
    kt.set<Chip::Select>(Chip::with(Chip::Select::Value{}, 1, {.on = true, .red = true}));
    check(runUntil(
            kt,
            [&] { return kt.writes<Chip::Colour>() >= 1 && kt.writes<Chip::Select>() >= 1; },
            500ms),
          "both blocks written");
    // six colour bytes as one transaction at 0x03, six select bytes as one at 0x09
    checkEq(ktd.word(0x03), 0xFFU, "R0");
    checkEq(ktd.word(0x08), 0xFFU, "B1");
    checkEq(ktd.word(0x09), 0xC0U, "A1 on, red from slot 1");
    if(failures != 0) { dump(); }
}

void pca9956b() {
    testCase("PCA9956B");
    fresh();
    RegisterModel<1> pca{0x3F};
    pca.set(0x81, {0x00});
    pca.readOnly     = {0x81};
    FakeBus::respond = std::ref(pca);
    using Led        = Chips::Pca9956b<0x3F, Units::ohm(1000)>;
    checkEq(Led::iref(Units::milliAmp(10)), std::uint8_t{44}, "IREF = 10 mA x 4 x 1000 / 900");
    checkEq(Led::iref(Units::milliAmp(100)),
            std::uint8_t{255},
            "100 mA is past the part's 57.4 mA: 255");
    Dev<Led> pc{};
    check(runUntil(pc, [&] { return pc.valid(); }, 1500ms), "up and polling MODE2");
    check(!pc.latest().error(), "no LED error flagged");
    // MODE2 as Table 9 has it: 1 is the fault in both flags; bits 2 and 0 are reserved and
    // read back 1
    static_assert([] {
        auto const clean = Led::Status::decode(Bytes{frame(0x00)});
        auto const err   = Led::Status::decode(Bytes{frame(0x40)});
        auto const res   = Led::Status::decode(Bytes{frame(0x45)});
        auto const hot   = Led::Status::decode(Bytes{frame(0x85)});
        return !clean.error() && !clean.overTemperature()   //
            && err.error() && !err.overTemperature()        // 0x40: ERROR, not OVERTEMP
            && equal(res.mode2, 0x45) && res.error() && !res.overTemperature()
            && hot.overTemperature() && !hot.error();   // bit 7 set is the fault
    }());
    check(hasWrite({0x80, 0x00}), "MODE1: All Call and sub-address 1 off, written at bring-up");
    check(hasWrite({0x82, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}),
          "LEDOUT: every channel in PWM mode, written at bring-up");
    Led::Pwm::Value pwm{};
    pwm[0]  = 0xFF;
    pwm[23] = 0x40;
    pc.set<Led::Pwm>(pwm);
    check(runUntil(pc, [&] { return pc.writes<Led::Pwm>() == 1; }, 300ms), "PWM block written");
    {
        bool found = false;
        for(auto const& w : writes()) {
            if(w.size() == 25 && w[0] == 0x8A && w[1] == 0xFF && w[24] == 0x40) { found = true; }
        }
        check(found, "all twenty-four channels in one auto-incrementing transaction");
    }
    pca.set(0x81, {0x40});
    check(runUntil(pc, [&] { return pc.latest().error(); }, 1500ms), "the ERROR flag is decoded");

    // Table 9 of the data sheet (rev 1.2): 1 is the fault in both flags, and bits 2 and 0
    // are reserved and read back 1 -- so 0x45 is a part with a latched LED error and no
    // over-temperature.
    check(!pc.latest().overTemperature(), "0x40: ERROR set, OVERTEMP clear");
    pca.set(0x81, {0x45});
    check(runUntil(
            pc,
            [&] { return pc.latest().mode2 == 0x45; },
            1500ms),
          "the reserved bits read 1");
    check(pc.latest().error() && !pc.latest().overTemperature(),
          "0x45 is an LED error on a part that is not too hot");
    pca.set(0x81, {0x85});
    check(runUntil(
            pc,
            [&] { return pc.latest().overTemperature(); },
            1500ms),
          "OVERTEMP is bit 7 set, not bit 7 clear");
    check(!pc.latest().error(), "and that alone is not an LED error");

    // MODE2 has an Initial of its own, written at bring-up, so the count to wait on is the
    // one after that.
    auto const mode2Writes = pc.writes<Led::Mode2>();
    pc.set<Led::Mode2>(Led::ClrErr);
    check(runUntil(
            pc,
            [&] { return pc.writes<Led::Mode2>() != mode2Writes; },
            300ms),
          "CLRERR written");
    check(hasWrite({0x81, 0x10}), "MODE2 bit 4, which self-clears");
    if(failures != 0) { dump(); }
}

void tsl2591() {
    testCase("TSL2591");
    fresh();
    RegisterModel<1> tsl{0x29};
    tsl.set(0xB2, {0x50});                     // device id, behind the command byte
    tsl.set(0xB3, {0x01});                     // STATUS: AVALID
    tsl.set(0xB4, {0x10, 0x27, 0xE8, 0x03});   // CH0 = 10000, CH1 = 1000
    tsl.readOnly     = {0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7};
    FakeBus::respond = std::ref(tsl);
    Dev<Chips::Tsl2591<>> ts{};
    check(runUntil(ts, [&] { return ts.valid(); }, 1s), "first sample");
    check(ts.identified(), "device id 0x50");
    check(hasWrite({0xA1, 0x10}), "CONFIG: medium gain, 100 ms, behind the command byte");
    check(hasWrite({0xA0, 0x03}), "ENABLE: PON | AEN");
    checkEq(ts.latest().infrared, 1000U, "the model's frame reached the decode");
    // STATUS AVALID, CH0 = 10000 and CH1 = 1000 little endian, at medium gain and 100 ms;
    // CPL = 100 ms x 25 / 408; lux = (10000 - 1000) x (1 - 0.1) / CPL = 1321.920 lx
    static_assert([] {
        auto const f   = frame(0x01, 0x10, 0x27, 0xE8, 0x03);
        auto const got = Chips::Tsl2591<>::Light::decode(Bytes{f}, Chips::Tsl2591<>::State{});
        return isOk(got) && equal(got.value.full, 10000U) && equal(got.value.infrared, 1000U)
            && equal(got.value.visible(), 9000U) && !got.value.saturated()
            && equal(got.value.lux(), 1321920);
    }());
    if(failures != 0) { dump(); }
}

// A run-time gain or integration change holds the part's other groups off for two integrations
// at the new setting, so a result integrated under the old one is never scaled with the new.
static_assert([] {
    std::array<std::byte, 2> b{};
    return Chips::Tsl2561::TimingSetting::encode(0x01, b).delay == 212ms      // 101 ms
        && Chips::Tsl2591<>::Configuration::encode(0x05, b).delay == 1210ms   // 600 ms
        && Chips::Veml6030<>::Config::encode(0x0040, b).delay == 410ms;       // 200 ms
}());

void light() {
    testCase("TCS34725");
    fresh();
    RegisterModel<1> m{0x29};
    m.set(0x92, {0x44});
    m.set(0xB3, {0x01});   // STATUS: AVALID
    m.set(0xB4, {0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04});
    FakeBus::respond = std::ref(m);
    Dev<Chips::Tcs34725> c{};
    check(runUntil(c, [&] { return c.samples() == 1; }, 500ms), "first sample");
    check(c.identified(), "ID 0x44");
    check(writes()
            == std::vector<std::vector<std::uint8_t>>{{0x80, 0x01}, {0x81, 0xD5}, {0x8F, 0x01}, {0x80, 0x03}, {0x81, 0xD5}, {0x8F, 0x01}, {0x80, 0x01}, {0x80, 0x03}},
          "PON, ATIME, gain, PON|AEN, then the integration and gain groups' Initial, the gain's "
          "with the RGBC cycle restarted");
    {
        // ATIME: two integrations at the new time before anything else of the part runs
        auto const from = FakeBus::log.size();
        c.set<Chips::Tcs34725::IntegrationSetting>(Chips::Tcs34725Detail::Integration::ms24);
        check(runUntil(
                c,
                [&] { return findWrite({0x81, 0xF6}, from) < FakeBus::log.size(); },
                1s),
              "ATIME written");
        auto const  at   = findWrite({0x81, 0xF6}, from);
        std::size_t next = at + 1;
        runFor(c, 300ms);
        check(next < FakeBus::log.size() && FakeBus::log[next].at - FakeBus::log[at].at >= 48ms,
              "nothing else of the part within two 24 ms integrations");
    }
    checkEq(c.latest().blue, 1024U, "the model's frame reached the decode");
    // STATUS AVALID, then C R G B as big-endian-looking bytes read little endian
    static_assert([] {
        auto const f   = frame(0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04);
        auto const got = Chips::Tcs34725::Colour::decode(Bytes{f}, Chips::Tcs34725::State{});
        return isOk(got) && equal(got.value.clear, 256U) && equal(got.value.red, 512U)
            && equal(got.value.green, 768U) && equal(got.value.blue, 1024U);
    }());

    testCase("VEML7700");
    fresh();
    RegisterModel<1, 2> v{0x10};
    v.set(0x07, {0x81, 0xC4});
    v.set(0x04, {0xE8, 0x03});
    v.set(0x05, {0x39, 0x05});   // WHITE = 1337
    v.readOnly       = {0x04, 0x05, 0x07};
    FakeBus::respond = std::ref(v);
    Dev<Chips::Veml7700<>> l{};
    check(runUntil(l, [&] { return l.samples() == 1; }, 500ms), "first sample");
    check(l.identified() && l.latest().als == 1000, "ID, and the model's frame reached the decode");
    checkEq(l.latest().white, 1337U, "WHITE, read into offset 2, reached the decode");
    // ALS 1000 and WHITE 1337, little endian
    static_assert([] {
        auto const f   = frame(0xE8, 0x03, 0x39, 0x05);
        auto const got = Chips::Veml7700<>::Light::decode(Bytes{f}, Chips::Veml7700<>::State{});
        return equal(got.als, 1000U) && equal(got.white, 1337U);
    }());

    testCase("VCNL4040");
    fresh();
    RegisterModel<1, 2> p{0x60};
    p.set(0x0C, {0x86, 0x01});
    p.set(0x08, {0x10, 0x00, 0xE8, 0x03, 0xD0, 0x07});
    p.readOnly       = {0x08, 0x09, 0x0A, 0x0C};
    FakeBus::respond = std::ref(p);
    Dev<Chips::Vcnl4040> n{};
    check(runUntil(n, [&] { return n.samples() == 1; }, 500ms), "first sample");
    check(n.identified(), "ID 0x86");
    check(hasWrite({0x00, 0x00, 0x00}) && hasWrite({0x03, 0x00, 0x00})
            && hasWrite({0x04, 0x00, 0x00}),
          "ALS_CONF, PS_CONF1/2, PS_CONF3/MS");
    checkEq(n.latest().als, 1000U, "ALS_Data, read into offset 2, reached the decode");
    checkEq(n.latest().white, 2000U, "WHITE_Data, read into offset 4, reached the decode");
    // PS 16, ALS 1000, WHITE 2000, little endian; 0.1 lx a count
    static_assert([] {
        auto const f   = frame(0x10, 0x00, 0xE8, 0x03, 0xD0, 0x07);
        auto const got = Chips::Vcnl4040::Light::decode(Bytes{f});
        return equal(got.proximity, 16U) && equal(got.als, 1000U) && equal(got.white, 2000U)
            && equal(got.lux(), 100000U);
    }());

    testCase("TSL2561");
    fresh();
    RegisterModel<1> t{0x39};
    t.set(0x8A, {0x50});
    t.set(0xAC, {0x00, 0x04, 0xFF, 0xFF});
    t.set(0xAE, {0x00, 0x01});
    FakeBus::respond = std::ref(t);
    Dev<Chips::Tsl2561> s{};
    check(runUntil(s, [&] { return s.samples() == 1; }, 2s), "first sample");
    check(s.identified(), "part number 5");
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0x80, 0x03}, {0x81, 0x02}, {0x81, 0x02}},
          "power up, timing, then the Timing group's Initial (the same byte again)");
    checkEq(s.latest().infrared, 256U, "channel 1, read into offset 2, reached the decode");
    // channel 0 = 1024 and channel 1 = 256, at 1x gain and 402 ms, T package (part 5): ratio
    // 0.25, the float formula gives 22.0 lx at 16x, so 352 lx here
    static_assert([] {
        auto const f   = frame(0x00, 0x04, 0x00, 0x01);
        auto const got = Chips::Tsl2561::Light::decode(Bytes{f}, Chips::Tsl2561::State{});
        return equal(got.full, 1024U) && equal(got.infrared, 256U)
            && equal(got.lux(), 351750);   // the T package integer calculation
    }());
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    bh1750();
    veml6030();
    opt3001();
    opt4048();
    ltr390();
    ltr507();
    apds9960();
    as7341();
    ktd2026();
    ktd2061();
    pca9956b();
    tsl2591();
    light();
    return finish();
}
