/// Temperature, humidity, pressure and gas: the Bosch, Sensirion and TI parts and their
/// kin, each against a model of the part on the wire.
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

// Not exercised against the fake bus, but parsed here so a change to a shared type (the
// BME280 compensation lives in the I2C description) cannot break them unnoticed.
#include <kvasir/Devices/Max31865.hpp>
#include <kvasir/Devices/SPI/Bme280.hpp>
#include <kvasir/Devices/SPI/Max7219.hpp>
#include <kvasir/Devices/SPI/Mpu9250.hpp>
#include <kvasir/Devices/SPI/NorFlash.hpp>
#include <kvasir/Devices/SPIDeviceBase.hpp>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

// -- compile-time decode helpers ----------------------------------------------------------

/// checkNear's comparison, for a static_assert: a quantity as the number in its own unit.
template<typename Q>
constexpr bool within(Q const& q,
                      double   want,
                      double   tol) {
    auto const x = static_cast<double>(CheckImpl::plain(q));
    return (x > want ? x - want : want - x) <= tol;
}

/// Sensirion words, each followed by its CRC, as a decode sees them.
template<typename... W>
    requires(std::integral<W> && ...)
constexpr std::array<std::byte,
                     3 * sizeof...(W)>
crcWords(W... ws) {
    std::array<std::byte, 3 * sizeof...(W)> out{};
    std::size_t                             i = 0;
    for(auto const w : {static_cast<std::uint16_t>(ws)...}) {
        for(auto const b : Sensirion::framed(w)) { out[i++] = static_cast<std::byte>(b); }
    }
    return out;
}

/// Two frames back to back.
template<std::size_t N,
         std::size_t M>
constexpr std::array<std::byte,
                     N + M>
join(std::array<std::byte,
                N> const& a,
     std::array<std::byte,
                M> const& b) {
    std::array<std::byte, N + M> out{};
    for(std::size_t i = 0; i < N; ++i) { out[i] = a[i]; }
    for(std::size_t i = 0; i < M; ++i) { out[N + i] = b[i]; }
    return out;
}

/// The BME280 trimming the bme280() model serves, as setup() leaves it.
constexpr Chips::Bme280::State bmeTrim() {
    Chips::Bme280::State t{};
    t.t1 = 27504;
    t.t2 = 26435;
    t.t3 = -1000;
    t.p  = {36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000};
    t.h1 = 75;
    t.h2 = 369;
    t.h3 = 0;
    t.h4 = 311;
    t.h5 = 50;
    t.h6 = 30;
    return t;
}

// -- the common parts ---------------------------------------------------------------------

void bme280() {
    testCase("BME280");
    fresh();
    RegisterModel<1> m{0x76};
    m.set(0xD0, {0x60});
    auto const le = [&](std::uint32_t reg, std::int32_t v) {
        m.set(reg,
              {static_cast<std::uint8_t>(v & 0xFF), static_cast<std::uint8_t>((v >> 8) & 0xFF)});
    };
    le(0x88, 27504);
    le(0x8A, 26435);
    le(0x8C, -1000);
    le(0x8E, 36477);
    le(0x90, -10685);
    le(0x92, 3024);
    le(0x94, 2855);
    le(0x96, 140);
    le(0x98, -7);
    le(0x9A, 15500);
    le(0x9C, -14600);
    le(0x9E, 6000);
    m.set(0xA1, {75});
    le(0xE1, 369);
    m.set(0xE3, {0});
    m.set(0xE4, {0x13, 0x27, 0x03});
    m.set(0xE7, {30});
    m.set(0xF7, {0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00, 0x60, 0x00});
    m.readOnly       = {0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xD0};
    FakeBus::respond = std::ref(m);

    Dev<Chips::Bme280> d{};
    check(runUntil(d, [&] { return d.answering(); }, 500ms), "bring-up completes");
    check(d.identified() && d.state().deviceId == 0x60, "chip id 0x60");
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0xE0, 0xB6}, {0xF2, 0x01}, {0xF5, 0xA0}, {0xF4, 0x27}},
          "reset, ctrl_hum, config while still asleep, then ctrl_meas into normal mode");
    checkEq(d.state().t1, 27504U, "trimming decoded");
    checkEq(d.state().h4, 311, "H4 nibbles");
    checkEq(d.state().h5, 50, "H5 nibbles");
    check(runUntil(d, [&] { return d.samples() == 1; }, 2s), "first sample");
    checkEq(d.latest().temperature, 2508, "the model's frame reached the decode");
    // press 0x655AC, temp 0x7EED0, hum 0x6000 against the trimming above; 0x80000 and 0x8000
    // are the skipped values the registers hold before the first conversion: rejected
    static_assert([] {
        auto const f   = frame(0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00, 0x60, 0x00);
        auto const got = Chips::Bme280::Measurement::decode(Bytes{f}, bmeTrim());
        auto const noT = frame(0x65, 0x5A, 0xC0, 0x80, 0x00, 0x00, 0x60, 0x00);
        auto const noP = frame(0x80, 0x00, 0x00, 0x7E, 0xED, 0x00, 0x60, 0x00);
        auto const noH = frame(0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00, 0x80, 0x00);
        return isOk(got) && equal(got.value.temperature, 2508)
            && within(got.value.pressure, 100653, 2)
            && got.value.humidity <= Units::milliPercent(100000)
            && isReject(Chips::Bme280::Measurement::decode(Bytes{noT}, bmeTrim()))
            && isReject(Chips::Bme280::Measurement::decode(Bytes{noP}, bmeTrim()))
            && isReject(Chips::Bme280::Measurement::decode(Bytes{noH}, bmeTrim()));
    }());
    auto const t0 = FakeClock::now();
    check(runUntil(d, [&] { return d.samples() == 2; }, 3s), "second sample");
    check(FakeClock::now() - t0 >= 990ms, "one a second");
    checkEq(Log::warnings, 0, "no warnings");
    if(failures != 0) { dump(); }

    testCase("BMP280");
    fresh();
    m.set(0xD0, {0x58});
    FakeBus::respond = std::ref(m);
    Dev<Chips::Bmp280> p{};
    check(runUntil(p, [&] { return p.samples() == 1; }, 2s), "first sample");
    check(!hasWrite({0xF2, 0x01}), "no ctrl_hum on a BMP280");
    checkEq(p.latest().temperature, 2508, "the model's frame reached the decode");
    // the same six bytes without the humidity word
    static_assert([] {
        auto const f   = frame(0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00);
        auto const got = Chips::Bmp280::Measurement::decode(Bytes{f}, bmeTrim());
        return isOk(got) && equal(got.value.temperature, 2508)
            && within(got.value.pressure, 100653, 2) && equal(got.value.humidity, 0U);
    }());
}

void sht3x() {
    testCase("SHT3x");
    fresh();
    std::vector<std::uint8_t> reply;
    bool                      corrupt = false;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x44) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                auto const cmd = static_cast<std::uint16_t>(
                  (static_cast<std::uint8_t>(sent[0]) << 8) | static_cast<std::uint8_t>(sent[1]));
                if(cmd == 0xF32D) { reply = withCrc(0x8010); }
                if(cmd == 0x2400) {
                    reply         = withCrc(0x6666);
                    auto const rh = withCrc(0x8000);
                    reply.insert(reply.end(), rh.begin(), rh.end());
                    if(corrupt) { reply[2] ^= 0xFF; }
                }
                return FakeBus::Result::succeeded;
            }
            if(recv.size() != reply.size()) { return FakeBus::Result::failed; }
            for(std::size_t i = 0; i < recv.size(); ++i) {
                recv[i] = static_cast<std::byte>(reply[i]);
            }
            return FakeBus::Result::succeeded;
        };
    Dev<Chips::Sht3x> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 2s), "first sample");
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0x30, 0x93}, {0x30, 0xA2}, {0xF3, 0x2D}, {0x24, 0x00}}, "break, reset, status, measure");
    check(d.identified(), "status CRC ok");
    checkEq(d.state().status, 0x8010U, "status");
    checkNear(d.latest().temperature, 2500, 1, "the model's frame reached the decode");
    // 0x6666 is 25.00 degC, 0x8000 50.00 %RH; a corrupt first CRC is rejected
    static_assert([] {
        auto const f   = crcWords(0x6666, 0x8000);
        auto const got = Chips::Sht3x::Measurement::decode(Bytes{f});
        auto       bad = f;
        bad[2] ^= std::byte{0xFF};
        return isOk(got) && within(got.value.temperature, 2500, 1)
            && within(got.value.humidity, 5000, 1)
            && isReject(Chips::Sht3x::Measurement::decode(Bytes{bad}));
    }());
    check(FakeBus::log[5].at - FakeBus::log[4].at >= 15ms, "conversion wait");
    corrupt = true;
    check(runUntil(d, [&] { return d.rejected() == 1; }, 3s), "a corrupt CRC is rejected");
    checkEq(d.samples(), 1U, "and not a sample");
    // the heater: a command each way, nothing until it is asked for
    using Heater = Chips::Sht3x::Heater;
    check(!hasWrite({0x30, 0x6D}) && !hasWrite({0x30, 0x66}), "no heater command by itself");
    static_cast<void>(d.set<Heater>(Chips::Sht3xDetail::Heater::on));
    check(runUntil(d, [&] { return hasWrite({0x30, 0x6D}); }, 2s), "heater enable 0x306D");
    static_cast<void>(d.set<Heater>(Chips::Sht3xDetail::Heater::off));
    check(runUntil(d, [&] { return hasWrite({0x30, 0x66}); }, 2s), "heater disable 0x3066");
    if(failures != 0) { dump(); }
}

void aht20() {
    testCase("AHT20");
    fresh();
    bool triggered = false;
    int  busyLeft  = 2;   // the first two frames read say busy; a retry re-triggers
    int  triggers  = 0;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x38) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                if(static_cast<std::uint8_t>(sent[0]) == 0xAC) {
                    triggered = true;
                    ++triggers;
                }
                return FakeBus::Result::succeeded;
            }
            if(recv.size() == 1) {
                recv[0] = std::byte{0x18};
                return FakeBus::Result::succeeded;
            }
            if(recv.size() == 7 && triggered) {
                if(busyLeft > 0) {
                    --busyLeft;
                    recv[0] = std::byte{0x9C};
                    return FakeBus::Result::succeeded;
                }
                std::array<std::byte, 6> f{std::byte{0x1C},
                                           std::byte{0x80},
                                           std::byte{0x00},
                                           std::byte{0x06},
                                           std::byte{0x66},
                                           std::byte{0x66}};
                for(std::size_t i = 0; i < 6; ++i) { recv[i] = f[i]; }
                recv[6] = static_cast<std::byte>(Sensirion::crc8(Bytes{f}));
                return FakeBus::Result::succeeded;
            }
            return FakeBus::Result::failed;
        };
    Dev<Chips::Aht20> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 3s), "first sample");
    check(FakeBus::log[0].at >= FakeClock::time_point{} + 1s + 100ms, "100 ms power-up");
    check(writes()[0] == std::vector<std::uint8_t>{0xBA}
            && writes()[1] == std::vector<std::uint8_t>{0xBE, 0x08, 0x00}
            && writes()[2] == std::vector<std::uint8_t>{0xAC, 0x33, 0x00},
          "reset, init, trigger");
    check(d.identified(), "calibrated");
    std::size_t reads7 = 0;
    for(auto const& t : FakeBus::log) { reads7 += t.isRead() && t.recvLen == 7; }
    checkEq(reads7, 3U, "two busy frames, each answered by a fresh trigger, then the frame");
    checkEq(triggers, 3, "three triggers");
    checkNear(d.latest().humidity, 5000, 1, "the model's frame reached the decode");
    // RH 0x80000 is 50.00 %, T 0x66666 30.00 degC, CRC over the six; busy bit 7 is a retry
    static_assert([] {
        auto const body = frame(0x1C, 0x80, 0x00, 0x06, 0x66, 0x66);
        auto const f    = join(body, frame(Sensirion::crc8(Bytes{body})));
        auto const got  = Chips::Aht20::Measurement::decode(Bytes{f});
        auto const busy = frame(0x9C, 0, 0, 0, 0, 0, 0);
        return isOk(got) && within(got.value.humidity, 5000, 1)
            && within(got.value.temperature, 3000, 1)
            && isRetry(Chips::Aht20::Measurement::decode(Bytes{busy}));
    }());
    checkEq(d.rejected(), 0U, "nothing rejected");
    if(failures != 0) { dump(); }
}

void sht4x() {
    testCase("SHT4x");
    fresh();
    CommandModel m{0x44};
    m.replies[0x89]  = words({0x1234, 0x5678});
    m.replies[0xFD]  = words({0x6666, 0x72B0});   // 25.0 degC, -6 + 125 * 29360 / 65535 = 50.0 %
    FakeBus::respond = std::ref(m);
    Dev<Chips::Sht4x> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 2s), "first sample");
    check(d.identified() && d.state().serial == 0x12345678U, "serial");
    check(m.commands[0] == 0x94 && m.commands[1] == 0x89 && m.commands[2] == 0xFD,
          "reset, serial, measure");
    checkNear(d.latest().temperature, 2500, 1, "the model's frame reached the decode");
    // 0x6666 is 25.0 degC, -6 + 125 x 0x72B0 / 65535 is 50.0 %RH
    static_assert([] {
        auto const f   = crcWords(0x6666, 0x72B0);
        auto const got = Chips::Sht4x::Measurement::decode(Bytes{f});
        return isOk(got) && within(got.value.temperature, 2500, 1)
            && within(got.value.humidity, 5000, 1);
    }());
    // the heated measurement: on demand only, its own command, the same frame
    using Heated       = Chips::Sht4x::Heated;
    m.replies[0x24]    = words({0x7333, 0x6000});   // 33.75 degC: warmer, and drier
    auto const before  = m.commands.size();
    auto const samples = d.samples<Heated>();
    runFor(d, 3s);
    check(
      std::find(m.commands.begin() + static_cast<std::ptrdiff_t>(before), m.commands.end(), 0x24)
        == m.commands.end(),
      "never by itself");
    auto const ticket = d.request<Heated>();
    check(runUntil(
            d,
            [&] { return d.answer<Heated>(ticket) == Answer::ok; },
            2s),
          "asked for, answered");
    checkEq(d.samples<Heated>(), samples + 1, "one heated sample");
    check(d.latest<Heated>().temperature > d.latest().temperature,
          "and latest() is still the unheated one");
    if(failures != 0) { dump(); }
}

void shtc3() {
    testCase("SHTC3");
    fresh();
    CommandModel m{0x70};
    m.replies[0xEFC8] = words({0x0807});
    m.replies[0x7866] = words({0x6666, 0x8000});
    FakeBus::respond  = std::ref(m);
    Dev<Chips::Shtc3> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 2s), "first sample");
    check(d.identified(), "ID 0x0807 pattern");
    check(m.commands
            == std::vector<
              std::uint16_t>{0x3517, 0x805D, 0x3517, 0xEFC8, 0xB098, 0x3517, 0x7866, 0xB098},
          "wake, reset, wake, id, sleep; wake, measure, sleep");
    checkNear(d.latest().temperature, 2500, 1, "the model's frame reached the decode");
    // 0x6666 is 25.00 degC, 0x8000 50.00 %RH
    static_assert([] {
        auto const f   = crcWords(0x6666, 0x8000);
        auto const got = Chips::Shtc3::Measurement::decode(Bytes{f});
        return isOk(got) && within(got.value.temperature, 2500, 1)
            && within(got.value.humidity, 5000, 1);
    }());
    if(failures != 0) { dump(); }
}

void htu21d() {
    testCase("HTU21D");
    fresh();
    CommandModel m{0x40};
    m.replies[0xF3]  = {0x4E, 0x85, 0x6B};   // datasheet: 7.04 degC, CRC 0x6B
    m.replies[0xF5]  = {0x68, 0x3A, 0x7C};   // datasheet: 44.8 %RH, CRC 0x7C
    FakeBus::respond = std::ref(m);
    Dev<Chips::Htu21d> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 2s), "first sample");
    check(m.commands == std::vector<std::uint16_t>{0xFE, 0xF3, 0xF5}, "reset, T, RH");
    checkNear(d.latest().temperature, 704, 1, "the model's frames reached the decode");
    checkEq(d.latest().humidity, 4488U, "and the humidity word from the receive at offset 3");
    // the datasheet's words: 0x4E85 (CRC 0x6B) is 7.04 degC, 0x683A (CRC 0x7C) 44.8 %RH
    static_assert([] {
        auto const f   = frame(0x4E, 0x85, 0x6B, 0x68, 0x3A, 0x7C);
        auto const got = Chips::Htu21d::Measurement::decode(Bytes{f});
        return isOk(got) && within(got.value.temperature, 704, 1)
            && within(got.value.humidity, 4488, 10);
    }());
    if(failures != 0) { dump(); }

    testCase("Si7021");
    fresh();
    m.commands.clear();
    m.replies[0xFCC9] = {0x15, 0xFF, 0xB5, 0xFF, 0xFF, 0x2D};   // SNB_3..0, each pair with its CRC
    FakeBus::respond  = std::ref(m);
    Dev<Chips::Si7021> si{};
    check(runUntil(si, [&] { return si.samples() == 1; }, 2s), "first sample");
    check(si.identified() && si.state().deviceId == 0x15, "device id 0x15");
}

void hdc1080() {
    testCase("HDC1080");
    fresh();
    RegisterModel<1, 2> m{0x40};
    m.set(0xFE, {0x54, 0x49, 0x10, 0x50});
    std::vector<std::uint8_t> result{0x66, 0x66, 0x80, 0x00};
    bool                      pointerOnly = false;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x40) { return FakeBus::Result::notAcknowledged; }
            if(sent.size() == 1 && recv.empty()) {
                pointerOnly = static_cast<std::uint8_t>(sent[0]) == 0x00;
                return FakeBus::Result::succeeded;
            }
            if(sent.empty() && recv.size() == 4 && pointerOnly) {
                for(std::size_t i = 0; i < 4; ++i) { recv[i] = static_cast<std::byte>(result[i]); }
                return FakeBus::Result::succeeded;
            }
            return m(addr, sent, recv);
        };
    Dev<Chips::Hdc1080> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 2s), "first sample");
    check(d.identified() && d.state().deviceId == 0x1050, "TI, HDC1080");
    check(m.word(0x02) == 0x1000, "configuration: both, 14 bit");
    checkNear(d.latest().temperature, 2600, 1, "the model's frame reached the decode");
    // 0x6666 is 26.00 degC, 0x8000 50.00 %RH
    static_assert([] {
        auto const f   = frame(0x66, 0x66, 0x80, 0x00);
        auto const got = Chips::Hdc1080::Measurement::decode(Bytes{f});
        return isOk(got) && within(got.value.temperature, 2600, 1)
            && within(got.value.humidity, 5000, 1);
    }());
    if(failures != 0) { dump(); }
}

void tmp102() {
    testCase("TMP102");
    fresh();
    RegisterModel<1, 2> m{0x48};
    m.set(0x00, {0x00, 0x00});   // 0 degC until the first conversion is done
    m.set(0x01, {0x60, 0xA0});
    m.readOnly                    = {0x00};
    bool                  written = false;
    FakeClock::time_point configured{};
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(s.size() == 3 && s[0] == std::byte{0x01}) {
            written    = true;
            configured = FakeClock::now();
        }
        if(written && FakeClock::now() >= configured + 15ms) {
            m.set(0x00, {0x19, 0x00});
            written = false;
        }
        return m(a, s, r);
    };
    Dev<Chips::Tmp102> t{};
    check(runUntil(t, [&] { return t.samples() == 1; }, 500ms), "first sample");
    check(t.identified(), "R1:R0 read 11, the low nibble 0");
    check(FakeBus::log.front().isRead()
            && FakeBus::log.front().sent == std::vector<std::uint8_t>{0x01},
          "the configuration read before anything is written");
    checkEq(centiOf(t.latest().temperature), 2500, "the model's frame reached the decode");
    check(m.word(0x01) == 0x60A0, "configuration written");
    m.set(0x00, {0xE7, 0x00});
    check(runUntil(t, [&] { return t.samples() == 2; }, 500ms), "second");
    // 0x1900 is 25.00 degC, 0xE700 -25.00 degC
    static_assert([] {
        auto const f = frame(0x19, 0x00);
        auto const g = frame(0xE7, 0x00);
        using T      = Chips::Tmp102::Temperature;
        return equal(centiOf(T::decode(Bytes{f}).temperature), 2500)
            && equal(centiOf(T::decode(Bytes{g}).temperature), -2500);
    }());
    // Not a TMP102: an ADS1115's config register (8583h) has bit 13 clear
    static_assert([] {
        Chips::Tmp102::State st{};
        return Chips::Tmp102::setup(Bytes{frame(0x60, 0xA0)}, st)
            && !Chips::Tmp102::setup(Bytes{frame(0x85, 0x83)}, st)
            && !Chips::Tmp102::setup(Bytes{frame(0x60, 0xA1)}, st);
    }());
    if(failures != 0) { dump(); }
}

void lm75() {
    RegisterModel<1, 2> m{0x48};
    m.readOnly = {0x00};
    testCase("LM75");
    fresh();
    // the configuration register is one byte on a chip whose others are two: the model
    // takes that write itself
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr == 0x48 && sent.size() == 2 && recv.empty()
               && static_cast<std::uint8_t>(sent[0]) == 0x01)
            {
                return FakeBus::Result::succeeded;
            }
            return m(addr, sent, recv);
        };
    m.set(0x00, {0x19, 0x00});
    Dev<Chips::Lm75> l{};
    check(runUntil(l, [&] { return l.samples() == 1; }, 500ms), "first sample");
    checkEq(centiOf(l.latest().temperature), 2500, "the model's frame reached the decode");
    // 0x1900 is 25.00 degC
    static_assert([] {
        auto const f = frame(0x19, 0x00);
        return equal(Chips::Lm75::Temperature::decode(Bytes{f}).temperature, 2500);
    }());
    if(failures != 0) { dump(); }
}

void dps310() {
    testCase("DPS310 (worked example)");
    fresh();
    RegisterModel<1> dps{0x77};
    dps.set(0x0D, {0x10});   // product id
    dps.set(0x28, {0x80});   // TMP_COEF_SRCE = external
    // c0 = 50, c1 = 20, c00 = 100000, c10 = 4000, c01 = 100, the rest zero
    dps.set(0x10,
            {0x03,
             0x20,
             0x14,
             0x18,
             0x6A,
             0x00,
             0x0F,
             0xA0,
             0x00,
             0x64,
             0x00,
             0x00,
             0x00,
             0x00,
             0x00,
             0x00,
             0x00,
             0x00});
    dps.set(0x00, {0x03, 0xF8, 0x00, 0x04, 0x00, 0x00});   // Praw_sc = 0.25, Traw_sc = 0.5
    dps.readOnly = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0D, 0x28};
    // MEAS_CFG as the part has it: COEF_RDY and SENSOR_RDY set, PRS_RDY and TMP_RDY set once a
    // conversion has finished since continuous mode began, cleared by a read of the result.
    FakeClock::time_point startedAt{};
    bool                  running     = false;
    bool                  resultRead  = false;
    int                   resultReads = 0;
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(s.size() == 2 && s[0] == std::byte{0x08}) {
            running    = s[1] == std::byte{0x07};
            startedAt  = FakeClock::now();
            resultRead = false;
        }
        auto const res       = dps(a, s, r);
        auto const converted = running && FakeClock::now() >= startedAt + 110ms;
        if(s.size() == 1 && s[0] == std::byte{0x08} && r.size() == 1) {
            r[0] |= converted && !resultRead ? std::byte{0xF0} : std::byte{0xC0};
        }
        if(s.size() == 1 && s[0] == std::byte{0x00} && r.size() == 6) {
            if(converted) { resultRead = true; }
            ++resultReads;
        }
        return res;
    };
    Dev<Chips::Dps310<>> dp{};
    check(runUntil(dp, [&] { return dp.valid(); }, 2s), "first sample");
    check(resultReads >= 2 && dp.samples() == 1,
          "the read before the first conversion was not a sample");
    check(dp.identified(), "product id 0x10 and the coefficient source agrees");
    checkEq(dp.state().c0, 50, "c0 out of the packed 12 bits");
    checkEq(dp.state().c1, 20, "c1 out of the other packed 12");
    checkEq(dp.state().c00, 100000, "c00 out of 20 bits");
    checkEq(dp.state().c10, 4000, "c10 out of the next 20");
    check(dps.word(0x06) == 0x06 && dps.word(0x07) == 0x80 && dps.word(0x09) == 0x04,
          "64x pressure with its result shift, single external temperature");
    checkEq(dps.word(0x08), 0x07U, "continuous pressure and temperature");
    checkEq(dp.latest().temperature, 35000, "the model's frame reached the decode");
    // Praw 0x03F800 and Traw 0x040000 against c0 = 50, c1 = 20, c00 = 100000, c10 = 4000,
    // c01 = 100: 35.000 degC = c0/2 + c1 x 0.5, and the datasheet's polynomial
    static_assert([] {
        Chips::Dps310<>::State st{};
        st.c0          = 50;
        st.c1          = 20;
        st.c00         = 100000;
        st.c10         = 4000;
        st.c01         = 100;
        auto const f   = frame(0x03, 0xF8, 0x00, 0x04, 0x00, 0x00, 0xF7);
        auto const got = Chips::Dps310<>::Measurement::decode(Bytes{f}, st);
        // PRS_RDY and TMP_RDY clear: nothing new, whatever the result registers hold
        auto const old = frame(0x03, 0xF8, 0x00, 0x04, 0x00, 0x00, 0xC7);
        return isOk(got) && equal(got.value.temperature, 35000) && equal(got.value.pressure, 101050)
            && isUnchanged(Chips::Dps310<>::Measurement::decode(Bytes{old}, st));
    }());

    // Every coefficient in play, against the datasheet's polynomial evaluated in double.
    static_assert([] {
        using D = Chips::Dps310<>;
        D::State st{};
        st.c0                 = 209;
        st.c1                 = -259;
        st.c00                = 80413;
        st.c10                = -56046;
        st.c01                = -2958;
        st.c11                = 1318;
        st.c20                = -11155;
        st.c30                = -1269;
        st.c21                = 71;
        constexpr double praw = -1'513'241;   // 0xE8E8E7 as 24 bits; raw / kP about -0.72
        constexpr double traw = 305'000;      // 0x04A768; raw / kT about 0.58
        auto const       f    = frame(0xE8, 0xE8, 0xE7, 0x04, 0xA7, 0x68, 0x30);
        auto const       got  = D::Measurement::decode(Bytes{f}, st).value;
        double const     prs  = praw / D::Kp;
        double const     tmp  = traw / D::Kt;
        double const wantP = st.c00 + prs * (st.c10 + prs * (st.c20 + prs * st.c30)) + tmp * st.c01
                           + tmp * prs * (st.c11 + prs * st.c21);
        double const wantT = st.c0 * 0.5 + st.c1 * tmp;
        return within(got.pressure, wantP, 1.0)                // within a pascal
            && within(got.temperature, wantT * 1000.0, 1.0);   // and a millidegree
    }());
    if(failures != 0) { dump(); }
}

void tmp117() {
    testCase("TMP117 / TMP119");
    fresh();
    RegisterModel<1, 2> t17{0x48};
    t17.set(0x0F, {0x01, 0x17});
    t17.set(0x00, {0x0C, 0x80});   // 3200 x 7.8125 m degC = 25.000 degC
    t17.readOnly = {0x00, 0x0F};
    // Data_Ready (bit 13) is the part's: set when a conversion is done, cleared by a read of
    // the temperature or of the configuration, so a group that reads the temperature first
    // never sees it. HIGH_Alert (bit 15) is set on every configuration read, so the flag can
    // only have come from the configuration step's bytes.
    bool ready       = true;
    bool bringUpRead = true;   // the bring-up's configuration read; the conversion after it
                               // is what the first Temperature read finds ready
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            auto const r = t17(addr, sent, recv);
            if(r == FakeBus::Result::succeeded && sent.size() == 1 && recv.size() == 2) {
                auto const reg = static_cast<std::uint8_t>(sent[0]);
                if(reg == 0x01) { recv[0] |= ready ? std::byte{0xA0} : std::byte{0x80}; }
                if(reg == 0x01 && bringUpRead) {
                    bringUpRead = false;
                } else if(reg == 0x00 || reg == 0x01) {
                    ready = false;
                }
            }
            return r;
        };
    Dev<Chips::Tmp117> t1{};
    check(runUntil(t1, [&] { return t1.valid(); }, 2s), "first sample");
    check(t1.identified(), "device id 0x0117");
    checkEq(t1.identity(0), 0x0117U, "DEVICE_ID as the engine read it, once");
    checkEq(t1.state().deviceId, 0x0117U, "and published in the State");
    // The identity is the description's Identity, under its mask: DID 117h whatever the revision
    // in 15:12; the TMP119's is its whole word, 2117h.
    static_assert(Chips::Tmp117::Identity[0].matches(0x0117)
                  && Chips::Tmp117::Identity[0].matches(0x3117)
                  && !Chips::Tmp117::Identity[0].matches(0x0118));
    static_assert(Chips::Tmp119::Identity[0].matches(0x2117)
                  && !Chips::Tmp119::Identity[0].matches(0x0117));
    // what is left to setup(): a part still loading its EEPROM is not brought up yet
    static_assert([] {
        Chips::Tmp117::State st{};
        auto const           loaded  = frame(0x02, 0x20);
        auto const           loading = frame(0x12, 0x20);   // EEPROM_Busy
        return Chips::Tmp117::setup(Bytes{loaded}, st) && !st.eepromBusy
            && !Chips::Tmp117::setup(Bytes{loading}, st) && st.eepromBusy;
    }());
    checkEq(t1.latest().temperature, 25000, "the model's frame reached the decode");
    checkEq(t1.latest().highAlert, true, "HIGH_Alert from the configuration read at offset 0");
    checkEq(t1.latest().lowAlert, false, "and LOW_Alert clear");
    checkEq(t17.word(0x01), 0x0220U, "the reset configuration at bring-up");
    t1.set<Chips::Tmp117::Limits>(Chips::Tmp117::High, Units::milliDegC(60000));
    check(runUntil(t1, [&] { return t17.word(0x02) == 0x1E00U; }, 200ms), "THIGH 60 degC");
    t1.modify<Chips::Tmp117::Config>(
      [](auto& c) { c.averaging = Chips::Tmp117::Averaging::sixtyFour; });
    check(runUntil(
            t1,
            [&] { return t17.word(0x01) == 0x0260U; },
            200ms),
          "AVG 11, the 1 s cycle untouched");
    t17.set(0x00, {0xF3, 0x80});   // -3200 counts
    ready = true;
    check(runUntil(t1, [&] { return t1.samples() == 2; }, 2s), "second sample");
    // config 0xA220 (HIGH_Alert, Data_Ready), then 3200 and -3200 counts: +-25.000 degC;
    // without Data_Ready the previous sample stands
    static_assert([] {
        using T       = Chips::Tmp117::Temperature;
        auto const f  = frame(0xA2, 0x20, 0x0C, 0x80);
        auto const g  = frame(0xA2, 0x20, 0xF3, 0x80);
        auto const h  = frame(0x02, 0x20, 0xF3, 0x80);
        auto const up = T::decode(Bytes{f}, {});
        auto const dn = T::decode(Bytes{g}, up.value);
        return isOk(up) && equal(up.value.temperature, 25000) && up.value.highAlert
            && !up.value.lowAlert && isOk(dn) && equal(dn.value.temperature, -25000)
            && isUnchanged(T::decode(Bytes{h}, dn.value));
    }());
    check(Chips::Tmp119::Name == std::string_view{"TMP119"}, "the TMP119 shares the map");
    if(failures != 0) { dump(); }
}

void emc2101() {
    testCase("EMC2101");
    fresh();
    RegisterModel<1> em{0x4C};
    em.set(0x00, {0x19});   // internal 25 degC
    em.set(0x01, {0x1E});   // external 30 degC whole
    em.set(0x02, {0x50});   // status: internal and external high, no other byte's value
    em.set(0x10, {0x60});   // external fraction 3/8 = 0.375
    em.set(0x46, {0x10});   // tach low
    em.set(0x47, {0x27});   // tach high -> 0x2710 = 10000 -> 540 rpm
    em.set(0xFD, {0x16});   // product id
    em.set(0xFE, {0x5D});   // manufacturer SMSC
    em.readOnly      = {0x00, 0x01, 0x02, 0x10, 0x46, 0x47, 0xFD, 0xFE};
    FakeBus::respond = std::ref(em);
    Dev<Chips::Emc2101> ee{};
    check(runUntil(ee, [&] { return ee.valid(); }, 800ms), "first sample");
    check(ee.identified(), "manufacturer 0x5D, product 0x16");
    checkEq(ee.latest().tach, 10000U, "the model's frame reached the decode, low byte first");
    checkEq(ee.latest().externalTemperature,
            30375,
            "external whole degrees at offset 1 and the fraction at 2");
    checkEq(ee.latest().status, std::uint8_t{0x50}, "status at offset 5");
    // internal 0x19, external 0x1E + 3/8, tach 0x10 0x27, status 0x50
    static_assert([] {
        auto const f   = frame(0x19, 0x1E, 0x60, 0x10, 0x27, 0x50);
        auto const got = Chips::Emc2101::Sensors::decode(Bytes{f});
        return equal(got.temperature, 25000) && equal(got.externalTemperature, 30375)
            && equal(got.tach, 10000U) && equal(got.fanSpeed(), 540) && equal(got.status, 0x50)
            && !got.externalDiodeOpen() && got.externalValid();
    }());
    check(em.word(0x03) == 0x8C && em.word(0x4A) == 0x20 && em.word(0xBF) == 0x06,
          "the configuration registers are written at bring-up");
    check(hasWrite({0x46}) == false, "and 0x46 is never written");
    // 0x46 must be read before 0x47, which it latches
    {
        std::size_t low = 0, high = 0;
        for(std::size_t i = 0; i < FakeBus::log.size(); ++i) {
            auto const& tr = FakeBus::log[i];
            if(tr.isRead() && tr.sent == std::vector<std::uint8_t>{0x46}) { low = i; }
            if(tr.isRead() && tr.sent == std::vector<std::uint8_t>{0x47} && high == 0) { high = i; }
        }
        check(low != 0 && high > low, "the tachometer low byte is read before the high byte");
    }
    ee.set<Chips::Emc2101::FanSetting>(Chips::Emc2101::fanSetting(Units::percent(100)));
    check(runUntil(ee, [&] { return em.word(0x4C) == 63U; }, 300ms), "full speed is 63");
    // +127.000 degC exactly is an open diode, not a temperature
    static_assert([] {
        auto const f   = frame(0x19, 0x7F, 0x00, 0x10, 0x27, 0x00);
        auto const got = Chips::Emc2101::Sensors::decode(Bytes{f});
        return got.externalDiodeOpen() && !got.externalValid();
    }());
    if(failures != 0) { dump(); }
}

// A limit past what the part measures is clamped to its range, not wrapped to the other sign.
static_assert([] {
    std::array<std::byte, 2> b{};
    static_cast<void>(Chips::Adt7420<>::Limits::encode(Units::centiDegC(30000), 0, b));
    auto const adt = static_cast<std::int16_t>((std::to_integer<unsigned>(b[0]) << 8U)
                                               | std::to_integer<unsigned>(b[1]));
    static_cast<void>(Chips::Mcp9808::Limits::encode(Units::centiDegC(-30000), 1, b));
    auto const mcp = (std::to_integer<unsigned>(b[0]) << 8U) | std::to_integer<unsigned>(b[1]);
    static_cast<void>(Chips::Tmp1075::Limits::encode(Units::centiDegC(20000), 1, b));
    auto const tmp = static_cast<std::int16_t>((std::to_integer<unsigned>(b[0]) << 8U)
                                               | std::to_integer<unsigned>(b[1]));
    return adt == 150 * 16 * 8   // +150 degC in sixteenths << 3
        && mcp
             == (0x1000U
                 | (static_cast<unsigned>(-40 * 4 * 4) & 0x0FFCU))   // -40 degC, sign bit 12
        && tmp == 125 * 16 * 16;                                     // +125 degC in sixteenths << 4
}());

void adt7420() {
    testCase("ADT7420");
    fresh();
    RegisterModel<1, 2> ad{0x48};
    ad.set(0x00, {0x0C, 0x80});   // 13-bit: 0x0C80 >> 3 = 400 sixteenths = 25.00 degC
    ad.readOnly        = {0x00};
    std::uint8_t adCfg = 0xFF, adHyst = 0xFF;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr == 0x48 && sent.size() == 2 && recv.empty()) {
                auto const r = static_cast<std::uint8_t>(sent[0]);
                if(r == 0x03) {
                    adCfg = static_cast<std::uint8_t>(sent[1]);
                    return FakeBus::Result::succeeded;
                }
                if(r == 0x0A) {
                    adHyst = static_cast<std::uint8_t>(sent[1]);
                    return FakeBus::Result::succeeded;
                }
            }
            if(addr == 0x48 && sent.size() == 1 && recv.size() == 1) {
                // the two 8-bit registers: the ID, and the configuration read back
                recv[0]
                  = static_cast<std::uint8_t>(sent[0]) == 0x0B ? std::byte{0xCB} : std::byte{adCfg};
                return FakeBus::Result::succeeded;
            }
            return ad(addr, sent, recv);
        };
    Dev<Chips::Adt7420<>> aa{};
    check(runUntil(aa, [&] { return aa.samples() == 1; }, 800ms), "first sample");
    check(aa.identified(), "ID 0xCB");
    checkEq(aa.latest().temperature, 2500, "the model's frame reached the decode");
    // 13-bit: 0x0C80 >> 3 = 400 sixteenths = 25.00 degC; 0xFFFF is a floating bus
    static_assert([] {
        using A = Chips::Adt7420<>;
        A::State const st{0xCB, A::Resolution::bits13};
        auto const     f   = frame(0x0C, 0x80);
        auto const     got = A::Temperature::decode(Bytes{f}, st);
        auto const     ff  = frame(0xFF, 0xFF);
        return isOk(got) && equal(got.value.temperature, 2500)
            && isReject(A::Temperature::decode(Bytes{ff}, st));
    }());
    // 16-bit: 0xFFFF is -1/128 degC, a reading just below zero, and is kept (-0.78 centi-degC
    // truncates to 0)
    static_assert([] {
        using A = Chips::Adt7420<>;
        A::State const st{0xCB, A::Resolution::bits16};
        auto const     ff  = frame(0xFF, 0xFF);
        auto const     got = A::Temperature::decode(Bytes{ff}, st);
        return isOk(got) && equal(got.value.temperature, 0);
    }());
    checkEq(adCfg, std::uint8_t{0x00}, "the reset configuration at bring-up");
    checkEq(adHyst, std::uint8_t{0x05}, "and 5 degC of hysteresis");
    aa.set<Chips::Adt7420<>::Limits>(Chips::Adt7420<>::High, Units::centiDegC(6400));
    check(runUntil(aa, [&] { return ad.word(0x04) == 0x2000U; }, 200ms), "THIGH 64 degC");
    aa.modify<Chips::Adt7420<>::Config>([](auto& c) { c.mode = Chips::Adt7420<>::Mode::shutdown; });
    check(runUntil(aa, [&] { return adCfg == 0x60; }, 200ms), "shutdown, the rest untouched");
    if(failures != 0) { dump(); }
}

void tmp1075() {
    testCase("TMP1075");
    fresh();
    RegisterModel<1, 2> t75{0x48};
    t75.set(0x0F, {0x75, 0x00});   // DIEID
    t75.set(0x01, {0x00, 0xFF});
    t75.set(0x00, {0x19, 0x00});   // left-aligned: 0x1900 >> 4 = 400 sixteenths = 25.00 degC
    t75.readOnly     = {0x00, 0x0F};
    FakeBus::respond = std::ref(t75);
    Dev<Chips::Tmp1075> tt{};
    check(runUntil(tt, [&] { return tt.samples() == 1; }, 500ms), "first sample");
    check(tt.identified(), "DIEID 0x7500");
    checkEq(tt.latest().temperature, 2500, "the model's frame reached the decode");
    checkEq(t75.word(0x01), 0x0000U, "the reset configuration is written back at bring-up");

    tt.set<Chips::Tmp1075::Limits>(Chips::Tmp1075::Low, Units::centiDegC(7500));
    tt.set<Chips::Tmp1075::Limits>(Chips::Tmp1075::High, Units::centiDegC(8000));
    check(runUntil(tt, [&] { return tt.writes<Chips::Tmp1075::Limits>() == 2; }, 200ms), "limits");
    checkEq(t75.word(0x02), 0x4B00U, "TLOW 75 degC");
    checkEq(t75.word(0x03), 0x5000U, "THIGH 80 degC");

    tt.modify<Chips::Tmp1075::Config>([](auto& c) { c.rate = Chips::Tmp1075::Rate::ms220; });
    check(runUntil(tt, [&] { return t75.word(0x01) == 0x6000U; }, 200ms), "conversion rate 0b11");
    tt.modify<Chips::Tmp1075::Config>([](auto& c) { c.power = Chips::Tmp1075::Power::shutdown; });
    check(runUntil(
            tt,
            [&] { return t75.word(0x01) == 0x6100U; },
            200ms),
          "and shutdown, with the rate untouched");

    t75.set(0x00, {0xF0, 0x00});   // -256 sixteenths = -16.00 degC
    check(runUntil(tt, [&] { return tt.samples() == 2; }, 800ms), "second sample");
    // 0x1900 is 25.00 degC; 0xF000, -256 sixteenths, is -16.00 degC sign extended
    static_assert([] {
        using T       = Chips::Tmp1075::Temperature;
        auto const f  = frame(0x19, 0x00);
        auto const g  = frame(0xF0, 0x00);
        auto const up = T::decode(Bytes{f});
        auto const dn = T::decode(Bytes{g});
        return isOk(up) && equal(up.value.temperature, 2500) && isOk(dn)
            && equal(dn.value.temperature, -1600);
    }());
    if(failures != 0) { dump(); }
}

void mcp9808() {
    testCase("MCP9808");
    fresh();
    RegisterModel<1, 2> c{0x18};
    c.set(0x06, {0x00, 0x54, 0x04, 0x00});
    c.set(0x05, {0x01, 0x90});
    c.readOnly              = {0x05, 0x06, 0x07};
    std::uint8_t resolution = 0xFF;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr == 0x18 && sent.size() == 2 && recv.empty()
               && static_cast<std::uint8_t>(sent[0]) == 0x08)
            {
                resolution = static_cast<std::uint8_t>(sent[1]);   // the one 8-bit register
                return FakeBus::Result::succeeded;
            }
            return c(addr, sent, recv);
        };
    Dev<Chips::Mcp9808> p{};
    check(runUntil(p, [&] { return p.samples() == 1; }, 500ms), "first sample");
    check(p.identified(), "manufacturer 0x0054, device 0x04");
    checkEq(centiOf(p.latest().temperature), 2500, "the model's frame reached the decode");
    check(resolution == 0x03 && c.word(0x01) == 0x0000,
          "resolution 8-bit write, config 16-bit write");
    c.set(0x05, {0x1E, 0x70});
    check(runUntil(p, [&] { return p.samples() == 2; }, 500ms), "second");
    // 0x0190 is 25.00 degC, 0x1E70 -25.00 degC (sign bit 12)
    static_assert([] {
        using T      = Chips::Mcp9808::Temperature;
        auto const f = frame(0x01, 0x90);
        auto const g = frame(0x1E, 0x70);
        return equal(centiOf(T::decode(Bytes{f}).temperature), 2500)
            && equal(centiOf(T::decode(Bytes{g}).temperature), -2500);
    }());
    if(failures != 0) { dump(); }
}

/// The AHT20's frame, as the DHT20 model serves it.
constexpr auto Frame_ = frame(0x1C, 0x80, 0x00, 0x06, 0x66, 0x66);

void dht20() {
    // The AHT20's frame: 50.00 %RH, 30.00 degC.
    constexpr std::array<std::uint8_t, 6> Frame{0x1C, 0x80, 0x00, 0x06, 0x66, 0x66};
    std::uint8_t                          status = 0x18;
    std::array<std::uint8_t, 3>           regBytes{0x11, 0x22, 0x33};
    auto const                            respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x38) { return FakeBus::Result::notAcknowledged; }
            if(!sent.empty()) {
                // The write-back of the restore sets the calibration bits, as a real part does.
                if(static_cast<std::uint8_t>(sent[0]) == 0xBE) { status = 0x1C; }
                return FakeBus::Result::succeeded;
            }
            if(recv.size() == 1) {
                recv[0] = std::byte{status};
            } else if(recv.size() == 3) {
                for(std::size_t i = 0; i < 3; ++i) { recv[i] = std::byte{regBytes[i]}; }
            } else if(recv.size() == 7) {
                for(std::size_t i = 0; i < 6; ++i) { recv[i] = std::byte{Frame[i]}; }
                recv[6] = std::byte{Sensirion::crc8(Bytes{std::as_bytes(std::span{Frame})})};
            } else {
                return FakeBus::Result::failed;
            }
            return FakeBus::Result::succeeded;
        };

    testCase("DHT20: calibrated at power-up -- a status read, then measurements");
    fresh();
    FakeBus::respond = respond;
    Dev<Chips::Dht20> d{};
    check(runUntil(d, [&] { return d.valid(); }, 3s), "first sample");
    check(FakeBus::log[0].at >= FakeClock::time_point{} + 1s + 100ms, "100 ms power-up");
    check(FakeBus::log[0].isRead() && FakeBus::log[0].recvLen == 1, "the status byte first");
    check(d.state().calibrated(), "0x18 set");
    check(writes() == std::vector<std::vector<std::uint8_t>>{{0xAC, 0x33, 0x00}},
          "no reset, no 0xBE init, no restore: only the trigger");
    checkNear(d.latest<Chips::Dht20::Measurement>().humidity,
              5000,
              1,
              "the model's frame reached the decode");
    // the AHT20's frame and CRC: 50.00 %RH, 30.00 degC
    static_assert([] {
        auto const f   = join(Frame_, frame(Sensirion::crc8(Bytes{Frame_})));
        auto const got = Chips::Dht20::Measurement::decode(Bytes{f});
        return isOk(got) && within(got.value.humidity, 5000, 1)
            && within(got.value.temperature, 3000, 1);
    }());

    testCase("DHT20: without the calibration bits, Restore rewrites 0x1B, 0x1C and 0x1E");
    fresh();
    status           = 0x08;
    FakeBus::respond = respond;
    Dev<Chips::Dht20> e{};
    check(runUntil(e, [&] { return e.answering(); }, 3s), "up");
    check(!e.state().calibrated(), "0x08 is not calibrated");
    auto const base = FakeBus::log.size();
    e.request<Chips::Dht20::Restore>();
    check(runUntil(e, [&] { return e.samples<Chips::Dht20::Restore>() == 1; }, 3s), "restored");
    auto const restore = [&] {
        std::vector<std::vector<std::uint8_t>> w;
        for(auto const& v : writes(base)) {
            if(v.size() == 3 && v[0] != 0xAC) { w.push_back(v); }
        }
        return w;
    }();
    check(restore
            == std::vector<std::vector<std::uint8_t>>{{0x1B, 0x00, 0x00},
                                                      {0xBB, 0x22, 0x33},
                                                      {0x1C, 0x00, 0x00},
                                                      {0xBC, 0x22, 0x33},
                                                      {0x1E, 0x00, 0x00},
                                                      {0xBE, 0x22, 0x33}},
          "register, read, the second and third bytes back under 0xB0 | register");
    check(e.latest<Chips::Dht20::Restore>().calibrated(), "the status read after it has 0x18");
    checkEq(e.latest<Chips::Dht20::Restore>().status,
            std::uint8_t{0x1C},
            "the final status byte at offset 9, not a register byte");
    if(failures != 0) { dump(); }
}

void gas() {
    testCase("SCD4x (datasheet example 500 ppm, 25 degC, 37 %)");
    fresh();
    CommandModel          m{0x62};
    int                   readyPolls = 0;
    FakeClock::time_point started{};
    m.replies[0x3682] = words({0x1234, 0x5678, 0x9ABC});
    // Table 11's bytes; its printed CRC for 0x01F4 (0x7B) is wrong, 0x33 is what the
    // polynomial gives (the other two CRCs in the example check out)
    m.replies[0xEC05] = {0x01, 0xF4, 0x33, 0x66, 0x67, 0xA2, 0x5E, 0xB9, 0x3C};
    // Like the part: the first measurement is there 5 s after start_periodic_measurement.
    m.onCommand = [&](std::uint16_t cmd) {
        if(cmd == 0x21B1) { started = FakeClock::now(); }
        if(cmd == 0xE4B8) {
            ++readyPolls;
            m.replies[0xE4B8] = words(
              {static_cast<std::uint16_t>(FakeClock::now() - started < 5s ? 0x8000 : 0x8006)});
        }
    };
    FakeBus::respond = std::ref(m);
    Dev<Chips::Scd4x> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 12s), "first sample");
    check(d.identified() && d.state().serial == 0x123456789ABCULL, "serial");
    check(m.commands[0] == 0x3F86 && m.commands[1] == 0x3682 && m.commands[2] == 0x21B1,
          "stop, serial, start");
    check(readyPolls >= 5 && readyPolls <= 7,
          "not ready for the first 5 s, polled a second apart, then ready in the same run");
    checkEq(d.latest().co2, 500, "the model's frame reached the decode");
    // the ready word, then Table 11: 500 ppm, 25 degC, 37 %RH (0x01F4's CRC is 0x33)
    static_assert([] {
        auto const f
          = join(crcWords(0x8006), frame(0x01, 0xF4, 0x33, 0x66, 0x67, 0xA2, 0x5E, 0xB9, 0x3C));
        auto const got = Chips::Scd4x::Measurement::decode(Bytes{f});
        return isOk(got) && equal(got.value.co2, 500) && within(got.value.temperature, 2500, 1)
            && within(got.value.humidity, 3700, 1);
    }());
    checkEq(d.rejected(), 0U, "nothing rejected");
    if(failures != 0) { dump(); }

    testCase("SGP40");
    fresh();
    CommandModel g{0x59};
    g.replies[0x3682] = words({0x0001, 0x0002, 0x0003});
    g.replies[0x260F] = words({0x1234});
    FakeBus::respond  = std::ref(g);
    Dev<Chips::Sgp40> s{};
    check(runUntil(s, [&] { return s.samples() == 1; }, 2s), "first sample");
    check(s.identified(), "serial CRCs");
    check(FakeBus::log[2].sent
            == std::vector<std::uint8_t>{0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93},
          "measure_raw with default compensation");
    checkEq(s.latest().ticks, 0x1234U, "the model's frame reached the decode");
    // one word with its CRC
    static_assert([] {
        auto const f   = crcWords(0x1234);
        auto const got = Chips::Sgp40::Raw::decode(Bytes{f});
        return isOk(got) && equal(got.value.ticks, 0x1234U);
    }());
    if(failures != 0) { dump(); }
}

void thermalAndBaro() {
    testCase("MLX90614 (datasheet PEC example)");
    fresh();
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(addr != 0x5A) { return FakeBus::Result::notAcknowledged; }
            if(sent.size() != 1 || recv.size() != 3) { return FakeBus::Result::failed; }
            auto const cmd = static_cast<std::uint8_t>(sent[0]);
            if(cmd == 0x07) {
                recv[0] = std::byte{0xD2};
                recv[1] = std::byte{0x3A};
                recv[2] = std::byte{0x30};
                return FakeBus::Result::succeeded;
            }
            if(cmd == 0x06) {
                std::array<std::byte, 5> frame{std::byte{0xB4},
                                               std::byte{0x06},
                                               std::byte{0xB5},
                                               std::byte{0x2E},
                                               std::byte{0x39}};
                recv[0] = std::byte{0x2E};
                recv[1] = std::byte{0x39};
                recv[2] = static_cast<std::byte>(crc8Smbus(Bytes{frame}));
                return FakeBus::Result::succeeded;
            }
            return FakeBus::Result::failed;
        };
    Dev<Chips::Mlx90614<>> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 500ms), "first sample");
    checkEq(d.latest().object, 2801, "the model's frame reached the decode");
    // ambient 0x392E (19.61 degC) with its PEC, then the datasheet's object 0x3AD2, PEC 0x30
    static_assert([] {
        auto const pec = crc8Smbus(Bytes{frame(0xB4, 0x06, 0xB5, 0x2E, 0x39)});
        auto const f   = frame(0x2E, 0x39, pec, 0xD2, 0x3A, 0x30);
        auto const got = Chips::Mlx90614<>::Temperature::decode(Bytes{f});
        return isOk(got) && equal(got.value.object, 2801) && equal(got.value.ambient, 1961);
    }());
    checkEq(d.rejected(), 0U, "PECs accepted");

    testCase("BMP180 (datasheet worked example)");
    fresh();
    RegisterModel<1> m{0x77};
    m.set(0xD0, {0x55});
    m.set(0xAA, {0x01, 0x98, 0xFF, 0xB8, 0xC7, 0xD1, 0x7F, 0xE5, 0x7F, 0xF5, 0x5A,
                 0x71, 0x18, 0x2E, 0x00, 0x04, 0x80, 0x00, 0xDD, 0xF9, 0x0B, 0x34});
    m.onWrite = [&](std::uint32_t reg, RegisterModel<1>::Reg const& r) {
        if(reg != 0xF4) { return; }
        if(r[0] == 0x2E) { m.set(0xF6, {0x6C, 0xFA, 0x00}); }   // UT 27898
        if(r[0] == 0x34) { m.set(0xF6, {0x5D, 0x23, 0x00}); }   // UP 23843, oss 0
    };
    FakeBus::respond = std::ref(m);
    Dev<Chips::Bmp180<Chips::Bmp180Detail::Oversampling::x1>> b{};
    check(runUntil(b, [&] { return b.samples() == 1; }, 2s), "first sample");
    check(b.identified() && b.state().ac1 == 408 && b.state().md == 2868, "id and calibration");
    checkEq(b.latest().temperature, 150, "the model's frame reached the decode");
    checkEq(b.latest().pressure, 69964U, "and UP from the second read, at offset 2");
    // the datasheet's example: UT 27898, UP 23843 at oss 0, 15.0 degC and 69964 Pa
    static_assert([] {
        using B = Chips::Bmp180<Chips::Bmp180Detail::Oversampling::x1>;
        B::State c{};
        c.ac1          = 408;
        c.ac2          = -72;
        c.ac3          = -14383;
        c.ac4          = 32741;
        c.ac5          = 32757;
        c.ac6          = 23153;
        c.b1           = 6190;
        c.b2           = 4;
        c.mb           = -32768;
        c.mc           = -8711;
        c.md           = 2868;
        auto const f   = frame(0x6C, 0xFA, 0x5D, 0x23, 0x00);
        auto const got = B::Measurement::decode(Bytes{f}, c);
        return isOk(got) && equal(got.value.temperature, 150) && equal(got.value.pressure, 69964U);
    }());
    if(failures != 0) { dump(); }
}

void mpl3115a2() {
    testCase("MPL3115A2");
    fresh();
    RegisterModel<1> m{0x60};
    m.set(0x0C, {0xC4});
    m.set(0x00, {0x00});   // not ready yet
    // 101325 Pa = 405300 quarter-Pa = 0x62F34 -> 20 bits left-justified in 3 bytes: 0x62F340;
    // 25.5 degC = 0x19 0x80
    m.set(0x01, {0x62, 0xF3, 0x40, 0x19, 0x80});
    m.readOnly = {0x0C, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
    // RST resets the part's I2C interface too: that write is not acknowledged.
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(s.size() == 2 && s[0] == std::byte{0x26} && s[1] == std::byte{0x04}) {
            return FakeBus::Result::notAcknowledged;
        }
        return m(a, s, r);
    };

    Dev<Chips::Mpl3115a2> d{};
    check(runUntil(d, [&] { return d.answering(); }, 500ms), "bring-up completes");
    check(d.identified() && d.state().deviceId == 0xC4, "WHO_AM_I 0xC4");
    check(writes().size() >= 2 && writes()[0] == std::vector<std::uint8_t>{0x26, 0x04},
          "the software reset first");
    checkEq(d.errors(), 0U, "its NAK is not an error");
    check(hasWrite({0x13, 0x07}) && hasWrite({0x26, 0x38}) && hasWrite({0x26, 0x39}),
          "event flags, OS 128 barometer, active");
    check(!runUntil(d, [&] { return d.samples() == 1; }, 300ms), "no sample while PTDR is clear");
    m.set(0x00, {0x08});
    check(runUntil(d, [&] { return d.samples() == 1; }, 500ms), "sample once ready");
    checkEq(d.latest().pressure, 101325U, "the model's frame reached the decode");
    // status, then 0x62F340 (Q18.2 left-justified: 101325 Pa) and 0x1980 (25.50 degC)
    static_assert([] {
        auto const f   = frame(0x08, 0x62, 0xF3, 0x40, 0x19, 0x80);
        auto const got = Chips::Mpl3115a2::Measurement::decode(Bytes{f});
        return isOk(got) && equal(got.value.pressure, 101325U)
            && equal(got.value.temperature, 2550);
    }());
    if(failures != 0) { dump(); }
}

void max31865() {
    testCase("MAX31865: Callendar-Van Dusen in integers");

    struct None {};

    using Pt500 = Kvasir::Max31865<FakeClock, None, None, None>;
    using Pt100 = Kvasir::Max31865<FakeClock, None, None, None, Units::ohm(100), Units::ohm(430)>;
    checkEq(Pt500::temperatureFor(16384), 0, "R = R0 is 0 degC");
    checkEq(Pt500::resistanceFor(16384), 500000, "half of the 1 k reference");
    // R(100 degC) = 500 (1 + 0.39083 - 0.005775) = 692.5375 ohm, code 22693
    checkNear(Pt500::temperatureFor(22693), 100000.0, 5.0, "100 degC on a PT500");
    // R(-50 degC) without the C term = 80.3141 ohm against 430, code 6120
    checkNear(Pt100::temperatureFor(6120), -50000.0, 30.0, "-50 degC on a PT100");
    // R(850 degC) = 3.9048 R0: 390.48 ohm against 430 is code 29756
    check(Pt100::codeInRange(29756) && !Pt100::codeInRange(29760),
          "the Callendar-Van Dusen range ends at 850 degC");
    check(Pt500::codeInRange(32767), "a 2 x reference never reads past it");
}

}   // namespace

int main() {
    bme280();
    sht3x();
    aht20();
    sht4x();
    shtc3();
    htu21d();
    hdc1080();
    tmp102();
    lm75();
    dps310();
    tmp117();
    emc2101();
    adt7420();
    tmp1075();
    mcp9808();
    dht20();
    gas();
    thermalAndBaro();
    mpl3115a2();
    max31865();
    return finish();
}
