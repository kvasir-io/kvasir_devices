/// The CY8CMBR3102 CapSense controller (the SparkFun soil moisture board) against a model of its
/// register map: identification, the stored-CRC check, provisioning, the sync-guarded debug
/// block and the NAKs the part gives while it wakes.
#include "Harness.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/All.hpp>
#include <span>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

using Soil = Chips::SparkFunSoilMoisture;

// -- compile time ---------------------------------------------------------------------------

// CRC-16/CCITT-FALSE (the header asserts the "123456789" check value 0x29B1): one zero byte
// is 0xE1F0, and an empty message leaves the seed.
static_assert(Chips::Cy8cmbr3102Detail::crc16(std::array<std::uint8_t,
                                                         1>{0x00})
              == 0xE1F0);
static_assert(Chips::Cy8cmbr3102Detail::crc16(std::array<std::uint8_t,
                                                         0>{})
              == 0xFFFF);

// The block: SparkFun's moisture setup on the 3102 factory defaults, the CRC behind it, LE.
static_assert([] {
    auto const& b = Soil::Configuration;
    return b[0x00] == 0x01 && b[0x01] == 0x00   // SENSOR_EN: CS0 only
        && b[0x08] == 0x00                      // 500 counts/pF
        && b[0x0C] == 128 && b[0x0D] == 128     // BASE_THRESHOLD0/1
        && b[0x2A] == 0x00 && b[0x2B] == 0x02   // PROX_TOUCH_TH0 = 512
        && b[0x40] == 0x0D                      // GPO_CFG: host, DC, strong, active high
        && b[0x41] == 0x0F && b[0x4C] == 0x05   // PWM 0x0F, SPO0 = GPO
        && b[0x4D] == 0x03 && b[0x4E] == 0x01   // filters, system diagnostics
        && b[0x4F] == 0x58                      // auto-reset 5 s both, ATH_EN
        && b[0x51] == 0x37 && b[0x52] == 5      // address, 100 ms
        && b[0x55] == 10                        // STATE_TIMEOUT
        && b[0x7E] == (Soil::ConfigCrc & 0xFF) && b[0x7F] == (Soil::ConfigCrc >> 8);
}());

// The template parameters reach the block, and the CRC follows it.
struct SlowRefresh {
    static constexpr auto Refresh = 200ms;
};

using Other = Chips::Cy8cmbr3102<0x38,
                                 1,
                                 Chips::Cy8cmbr3102Detail::Sensitivity::counts125PerPf,
                                 Chips::Cy8cmbr3102Detail::AutoReset::after20s,
                                 Chips::Cy8cmbr3102Detail::Spo0::sensor,
                                 SlowRefresh>;
static_assert(Other::Configuration[0x00] == 0x02 && Other::Configuration[0x08] == 0x0C
              && Other::Configuration[0x40] == 0x00 && Other::Configuration[0x4C] == 0x01
              && Other::Configuration[0x4F] == 0xA8 && Other::Configuration[0x51] == 0x38
              && Other::Configuration[0x52] == 10 && Other::ConfigCrc != Soil::ConfigCrc);

// The debug block: SYNC1 7, sensor 0, 23 pF, difference 0x0012, baseline 0x0456, raw 0x0468,
// average 0, 0xE6, SYNC2 7.
static_assert([] {
    auto const f   = frame(0x07, 0x00, 23, 0x12, 0x00, 0x56, 0x04, 0x68, 0x04, 0, 0, 0, 0x07);
    auto const got = Soil::Moisture::decode(Bytes{f});
    return isOk(got) && equal(got.value.capacitance, 23) && equal(got.value.difference, 0x12)
        && equal(got.value.baseline, 0x456) && equal(got.value.raw, 0x468);
}());
// the reserved high nibble of the counters does not count
static_assert(isOk(Soil::Moisture::decode(Bytes{frame(0xA7,
                                                      0x00,
                                                      23,
                                                      0,
                                                      0,
                                                      0,
                                                      0,
                                                      0,
                                                      0,
                                                      0,
                                                      0,
                                                      0,
                                                      0x57)})));
// the part moved on between the counters: read again, soon
static_assert([] {
    auto const got
      = Soil::Moisture::decode(Bytes{frame(0x07, 0x00, 23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x08)});
    return isRetry(got) && got.retryAfter == 5ms;
}());
// SENSOR_ID not taken yet (255 after a reset): read again after the command latency
static_assert([] {
    auto const got
      = Soil::Moisture::decode(Bytes{frame(0x07, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x07)});
    return isRetry(got) && got.retryAfter == 50ms;
}());

// The moisture percentage: clamped, rounded, either direction.
using Chips::Cy8cmbr3102Detail::moisture;
static_assert(equal(moisture(1000,
                             1000,
                             2000),
                    0));
static_assert(equal(moisture(1500,
                             1000,
                             2000),
                    50));
static_assert(equal(moisture(2000,
                             1000,
                             2000),
                    100));
static_assert(equal(moisture(500,
                             1000,
                             2000),
                    0));
static_assert(equal(moisture(9000,
                             1000,
                             2000),
                    100));
static_assert(equal(moisture(1250,
                             2000,
                             1000),
                    75));   // wet reads lower
static_assert(equal(moisture(1234,
                             1000,
                             1000),
                    0));   // no span
static_assert(equal(Soil::Sample{.raw = 1333}.moisture(1000,
                                                       2000),
                    33));

// -- the model ------------------------------------------------------------------------------

std::array<std::uint8_t,
           13>
debugBlock(std::uint8_t  sync1,
           std::uint8_t  sensor,
           std::uint8_t  cp,
           std::uint16_t raw,
           std::uint8_t  sync2) {
    return {sync1,
            sensor,
            cp,
            0x10,
            0x00,
            0x00,
            0x04,
            static_cast<std::uint8_t>(raw & 0xFF),
            static_cast<std::uint8_t>(raw >> 8),
            0,
            0,
            0,
            sync2};
}

struct Part {
    RegisterModel<1> m{0x37};

    explicit Part(std::uint16_t storedCrc,
                  std::uint16_t deviceId = 0x0A01) {
        m.set(0x89, {0x00, 0x00});
        m.set(0x8F,
              {0x9A,
               static_cast<std::uint8_t>(deviceId & 0xFF),
               static_cast<std::uint8_t>(deviceId >> 8)});
        m.set(
          0x7E,
          {static_cast<std::uint8_t>(storedCrc & 0xFF), static_cast<std::uint8_t>(storedCrc >> 8)});
        block(debugBlock(3, 0, 21, 0x0468, 3));
    }

    void block(std::array<std::uint8_t,
                          13> const& b) {
        for(std::size_t i = 0; i < b.size(); ++i) {
            m.mem[0xDB + static_cast<std::uint32_t>(i)] = {b[i]};
        }
    }
};

std::size_t debugReads() {
    std::size_t n = 0;
    for(auto const& t : FakeBus::log) {
        if(t.isRead() && !t.sent.empty() && t.sent[0] == 0xDB) { ++n; }
    }
    return n;
}

// -- cases ----------------------------------------------------------------------------------

void identified() {
    testCase("CY8CMBR3102 identified, configured, reading");
    fresh();
    Part part{Soil::ConfigCrc};
    FakeBus::respond = std::ref(part.m);
    Dev<Soil> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first sample");
    check(d.identified(), "FAMILY_ID 0x9A, DEVICE_ID 0x0A01");
    checkEq(d.state().deviceId, 0x0A01U, "DEVICE_ID, little endian");
    checkEq(d.state().storedCrc, Soil::ConfigCrc, "CONFIG_CRC as stored");
    check(d.state().configured, "the stored CRC is the description's: nothing to provision");
    check(d.state().lastError == Soil::CommandError::none, "CTRL_CMD_ERR");
    checkEq(d.latest().raw, 0x0468U, "DEBUG_RAW_COUNT0 reached the sample");
    checkEq(d.latest().capacitance, 21U, "DEBUG_CP, pF");
    checkEq(d.latest().baseline, 0x0400U, "DEBUG_BASELINE0");
    checkEq(d.latest().difference, 0x0010U, "DEBUG_DIFFERENCE_COUNT0");
    check(hasWrite({0x82, 0x00}), "SENSOR_ID selects CS0");
    runFor(d, 1s);
    for(auto const& w : writes()) {
        check(w.size() == 2 && w[0] == 0x82, "nothing but SENSOR_ID written: no flash write");
    }
    checkEq(d.errors(), 0U, "no errors");
    checkEq(d.rejected(), 0U, "no rejections");
    if(failures != 0) { dump(); }
}

void rejected() {
    testCase("CY8CMBR3102 rejects another family member");
    fresh();
    Part part{Soil::ConfigCrc, 0x0A06};   // a CY8CMBR3106S
    FakeBus::respond = std::ref(part.m);
    Dev<Soil> d{};
    runFor(d, 500ms);
    check(!d.identified(), "DEVICE_ID 0x0A06 is not a 3102");
    check(!d.valid(), "and nothing is read from it");
    check(writes().empty(), "nor written to it");
    if(failures != 0) { dump(); }
}

void provision() {
    testCase("CY8CMBR3102 unconfigured, provisioned once");
    fresh();
    Part part{0xBEEF};
    FakeBus::respond = std::ref(part.m);
    Dev<Soil> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "reads regardless");
    check(d.identified(), "identified");
    check(!d.state().configured, "a stored CRC of 0xBEEF is not this configuration");
    runFor(d, 300ms);
    for(auto const& w : writes()) {
        check(w.size() == 2 && w[0] == 0x82, "setup wrote nothing to the configuration");
    }

    auto const from = FakeBus::log.size();
    d.set<Soil::Provision>(0);
    runFor(d, 2s);
    std::vector<std::vector<std::uint8_t>> ws;
    std::vector<FakeClock::time_point>     at;
    for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
        auto const& t = FakeBus::log[i];
        if(t.isWrite() && t.sent[0] != 0x82) {
            ws.push_back(t.sent);
            at.push_back(t.at);
        }
    }
    check(ws.size() == 3, "block, save, reset");
    if(ws.size() == 3) {
        check(ws[0].size() == 129 && ws[0][0] == 0x00, "one write of 0x00..0x7F");
        bool same = true;
        for(std::size_t i = 0; i < 128 && ws[0].size() == 129; ++i) {
            same = same && ws[0][i + 1] == Soil::Configuration[i];
        }
        check(same, "the compile-time block, CRC included");
        checkEq(ws[0][0x7F], Soil::ConfigCrc & 0xFFU, "CONFIG_CRC low byte at 0x7E");
        checkEq(ws[0][0x80], Soil::ConfigCrc >> 8U, "high byte at 0x7F");
        check(ws[1] == std::vector<std::uint8_t>{0x86, 0x02}, "then SAVE_CHECK_CRC (TRM 1.5.80)");
        check(ws[2] == std::vector<std::uint8_t>{0x86, 0xFF}, "then the software reset");
        check(at[2] - at[1] >= 500ms, "the save is given 500 ms before the reset");
    }
    check(part.m.word(0x4F) == 0x58, "the model holds the block");
    check(!d.state().configured, "the write alone does not say the save took");
    checkEq(d.writes<Soil::Provision>(), 1U, "written once");
    auto const ticket = d.request<Soil::Stored>();
    check(runUntil(
            d,
            [&] { return d.answer<Soil::Stored>(ticket) != Answer::pending; },
            500ms),
          "Stored read back");
    check(d.latest<Soil::Stored>().configured
            && d.latest<Soil::Stored>().storedCrc == Soil::ConfigCrc,
          "CONFIG_CRC now this configuration's");

    auto const after = FakeBus::log.size();
    runFor(d, 2s);
    for(auto const& w : writes(after)) { check(w.size() == 2 && w[0] == 0x82, "and never again"); }
    if(failures != 0) { dump(); }
}

void setAddress() {
    testCase("CY8CMBR3102 SetAddress and Command");
    fresh();
    Part part{Soil::ConfigCrc};
    FakeBus::respond = std::ref(part.m);
    Dev<Soil> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first sample");

    auto from = FakeBus::log.size();
    d.set<Soil::Command>(Soil::Operation::clearLatched);
    runFor(d, 200ms);
    check(hasWrite({0x86, 0x08}, from), "clear latched status: CTRL_CMD 8");

    from = FakeBus::log.size();
    d.set<Soil::SetAddress>(0x42);
    runFor(d, 2s);
    auto const block = Soil::configurationFor(0x42);
    check(block[0x51] == 0x42, "I2C_ADDR in the block");
    check(Soil::crc(block) != Soil::ConfigCrc, "and a CRC of its own");
    check(block[0x7E] == (Soil::crc(block) & 0xFF), "behind it");
    check(part.m.get(0x51)[0] == 0x42 && part.m.get(0x7E)[0] == block[0x7E],
          "the block reached the part");
    check(hasWrite({0x86, 0x02}, from) && hasWrite({0x86, 0xFF}, from), "saved and reset");
    std::array<std::byte, Soil::BlockBytes> buf{};
    static_cast<void>(Soil::SetAddress::encode(0x05, buf));
    check(buf[0x51] == std::byte{0x37}, "an address outside 8..119 writes Addr");
    if(failures != 0) { dump(); }
}

void syncMismatch() {
    testCase("CY8CMBR3102 sync counters disagree: retried, then read");
    fresh();
    Part part{Soil::ConfigCrc};
    int  torn = 2;
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            auto const r = part.m(addr, sent, recv);
            if(r == FakeBusResult::succeeded && recv.size() == 13 && torn > 0) {
                --torn;
                recv[12] = std::byte{4};   // SYNC_COUNTER2 moved on under the read
            }
            return r;
        };
    Dev<Soil> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "a sample in the end");
    checkEq(torn, 0, "both torn frames were read");
    checkEq(debugReads(), 3U, "two retries, then the frame that holds");
    checkEq(d.rejected(), 0U, "a retry is not a rejection");
    checkEq(d.samples(), 1U, "one sample");
    if(failures != 0) { dump(); }
}

void wakeNaks() {
    testCase("CY8CMBR3102 NAKs while it wakes: put on the wire again");
    fresh();
    Part part{Soil::ConfigCrc};
    int  naks = 3;   // the bring-up's first read
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(naks > 0) {
                --naks;
                return FakeBusResult::notAcknowledged;
            }
            return part.m(addr, sent, recv);
        };
    Dev<Soil> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first sample");
    check(d.identified(), "identified after the wake NAKs");
    checkEq(d.errors(), 0U, "the NAKs did not count as errors");
    checkEq(d.wakeRetries(), 3U, "they counted as wake retries");

    // asleep again later: two NAKs on a cyclic read
    auto const n = d.samples();
    naks         = 2;
    runFor(d, 500ms);
    checkEq(d.errors(), 0U, "still no errors");
    checkEq(d.wakeRetries(), 5U, "two more retries");
    check(d.samples() > n, "and the samples go on");
    check(d.answering(), "the part is answering");

    // a part that stays silent past the retries does count
    naks = 1000;
    runFor(d, 500ms);
    check(d.errors() > 0, "six NAKs in a row are a NAK");
    if(failures != 0) { dump(); }
}

void capacitance() {
    testCase("CY8CMBR3102 Capacitance on demand: SENSOR_ID away and back");
    fresh();
    Part part{Soil::ConfigCrc};
    FakeBus::respond = std::ref(part.m);
    Dev<Soil> d{};
    check(runUntil(d, [&] { return d.valid(); }, 1s), "first sample");
    d.period<Soil::Moisture>(0ms);
    runFor(d, 200ms);
    auto const from = FakeBus::log.size();
    part.block(debugBlock(9, 0, 37, 0x0500, 9));
    auto const ticket = d.request<Soil::Capacitance>();
    check(runUntil(
            d,
            [&] { return d.answer<Soil::Capacitance>(ticket) != Answer::pending; },
            1s),
          "answered");
    check(d.answer<Soil::Capacitance>(ticket) == Answer::ok, "ok");
    check(writes(from) == std::vector<std::vector<std::uint8_t>>{{0x82, 0x01}, {0x82, 0x00}},
          "CS1 selected, then CS0");
    std::vector<FakeClock::time_point> at;
    for(std::size_t i = from; i < FakeBus::log.size(); ++i) { at.push_back(FakeBus::log[i].at); }
    check(at.size() == 3 && at[1] - at[0] >= 50ms && at[2] - at[1] >= 50ms,
          "each given the 50 ms command latency, then the block");
    checkEq(d.latest<Soil::Capacitance>().capacitance, 37U, "DEBUG_CP after the reselect");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    identified();
    rejected();
    provision();
    setAddress();
    syncMismatch();
    capacitance();
    wakeNaks();
    return finish();
}
