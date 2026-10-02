/// SPI parts on the chip-description engine (SPI/Device.hpp, SPI/Transport.hpp) over the real queued
/// master core: the BME280 on SPI, an empty bus and a failing one.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "FakeQueuedSpi.hpp"
#include "FakeSpi.hpp"
#include "SpiRegisters.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <kvasir/Devices/SPI/chips/Bme280.hpp>
#include <kvasir/Devices/SPI/chips/Max7219.hpp>
#include <kvasir/Devices/SPI/chips/Mpu9250.hpp>
#include <support/FakeClock.hpp>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
using Kvasir::Test::Spi::SpiRegisters;

namespace {

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;
using Bme = Kvasir::SPI::Device<Bus, FakeClock, Kvasir::SPI::Chips::Bme280, Spi::Cs>;
using Bmp = Kvasir::SPI::Device<Bus, FakeClock, Kvasir::SPI::Chips::Bmp280, Spi::Cs>;

// Two SPI devices are one engine port, so they share one Engine (Transport.hpp, TransportRequest).
static_assert(std::is_same_v<Kvasir::I2C::detail::PortOf<Bme::TransportT>,
                             Kvasir::I2C::detail::PortOf<Bmp::TransportT>>);

SpiRegisters part{};

void bmeRegisters() {
    part = SpiRegisters{};
    // The BME280's registers are 0x80..0xFF, and SPI carries their low 7 bits: "address 0xF7 is
    // accessed by using SPI register address 0x77" (BME280.md:1483).
    part.decode = [](std::uint8_t c) {
        return std::pair{(c & 0x80U) != 0, static_cast<std::uint8_t>(c | 0x80U)};
    };
    part.reg[0xD0] = 0x60;
    auto const le  = [](std::uint8_t r, std::int32_t v) {
        part.reg[r]      = static_cast<std::uint8_t>(v & 0xFF);
        part.reg[r + 1U] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
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
    part.reg[0xA1] = 75;
    le(0xE1, 369);
    part.reg[0xE3] = 0;
    part.reg[0xE4] = 0x13;
    part.reg[0xE5] = 0x27;
    part.reg[0xE6] = 0x03;
    part.reg[0xE7] = 30;
    // press 0x655AC, temp 0x7EED0, hum 0x6000: 25.08 degC against the trimming (thermal_test)
    std::uint8_t const frame[]{0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00, 0x60, 0x00};
    for(std::size_t i = 0; i < sizeof frame; ++i) { part.reg[0xF7 + i] = frame[i]; }
}

void fresh() {
    Bus::reset();
    Pins::reset();
    FakeClock::reset();
    Log::reset();
    Bus::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
    Bus::exchange     = [](std::uint8_t m) { return part.exchange(m); };
}

/// Declared right after a case's device: resets the bus while the device still lives, so nothing
/// queued completes into a dead frame in the next case.
struct BusScope {
    ~BusScope() { Bus::reset(); }
};

[[maybe_unused]] void dump() {
    for(auto const& f : Bus::frames) {
        std::printf("  %s mode %u %u Hz mosi",
                    f.selected ? "CS " : "   ",
                    f.setup.mode,
                    f.setup.hz);
        for(auto b : f.mosi) { std::printf(" %02x", b); }
        std::printf(" | miso");
        for(auto b : f.miso) { std::printf(" %02x", b); }
        std::printf("\n");
    }
}

template<typename D,
         typename Pred>
bool runUntil(D&                        d,
              Pred                      pred,
              std::chrono::milliseconds limit) {
    auto const end = FakeClock::now() + limit;
    while(FakeClock::now() < end) {
        Bus::handler();
        d.handler();
        Bus::tick();
        FakeClock::advance(100us);
        if(pred()) { return true; }
    }
    return false;
}

void bme280OnSpi() {
    testCase("BME280 on SPI: the I2C description, its commands with R/W in bit 7");
    bmeRegisters();
    fresh();
    Bme      d{};
    BusScope scope{};
    check(Pins::level[Spi::Cs::id], "CS released by the constructor");
    check(runUntil(d, [&] { return d.answering(); }, 500ms), "bring-up completes");
    check(d.identified() && d.state().deviceId == 0x60, "Identity: chip id 0x60");
    std::vector<std::pair<std::uint8_t, std::uint8_t>> const expect{
      {0xE0, 0xB6},
      {0xF2, 0x01},
      {0xF5, 0xA0},
      {0xF4, 0x27}
    };
    check(part.written == expect, "reset, ctrl_hum, config, ctrl_meas");
    check(Bus::frames.size() > 3 && Bus::frames[2].mosi == std::vector<std::uint8_t>{0x60}
            && Bus::frames[3].mosi == std::vector<std::uint8_t>{0xB6},
          "the reset goes out as control 0x60 (bit 7 = 0: write), then 0xB6");
    // every frame of the part went out with CS low, at mode 0 and at most 10 MHz
    bool allSelected = true;
    for(auto const& f : Bus::frames) {
        allSelected = allSelected && f.selected && f.setup.mode == 0 && f.setup.hz <= 10'000'000U;
    }
    check(allSelected, "every frame selected, mode 0, <= 10 MHz");
    // the first read frame is the Identity: control 0xD0 | 0x80 = 0xD0, then one byte
    check(Bus::frames.size() > 2 && Bus::frames[0].mosi == std::vector<std::uint8_t>{0xD0}
            && Bus::frames[1].miso == std::vector<std::uint8_t>{0x60},
          "the Identity read first: command 0xD0, then 0x60 back");
    check(runUntil(d, [&] { return d.samples() == 1; }, 2s), "first sample");
    checkEq(d.latest().temperature, 2508, "the part's frame reached the decode");
    auto const t0 = FakeClock::now();
    check(runUntil(d, [&] { return d.samples() == 2; }, 3s), "second sample");
    check(FakeClock::now() - t0 >= 990ms, "one a second");
    checkEq(Log::warnings, 0, "no warnings");
    if(failures != 0) { dump(); }
}

void bmp280OnSpi() {
    testCase("BMP280 on SPI: its own Identity, no ctrl_hum");
    bmeRegisters();
    part.reg[0xD0] = 0x58;
    fresh();
    Bmp      p{};
    BusScope scope{};
    check(runUntil(p, [&] { return p.samples() == 1; }, 2s), "first sample");
    bool ctrlHum = false;
    for(auto const& w : part.written) { ctrlHum = ctrlHum || w.first == 0xF2; }
    check(!ctrlHum, "no ctrl_hum");
}

void noPart(std::uint8_t floating) {
    testCase(floating == 0xFF ? "no part, MISO high: never answering, nothing written"
                              : "no part, MISO low: never answering, nothing written");
    fresh();
    Bus::exchange = {};
    Bus::floating = floating;
    Bme      d{};
    BusScope scope{};
    runUntil(d, [] { return false; }, 5s);
    check(!d.answering() && !d.identified(), "not answering");
    bool anyWrite = false;
    for(auto const& f : Bus::frames) { anyWrite = anyWrite || (f.mosi.size() > 1); }
    check(!anyWrite, "the Identity failed first: no reset or configuration went to nothing");
    check(Pins::level[Spi::Cs::id], "CS released");
    checkEq(d.bringUps(), 0U, "no bring-up counted");
}

void wrongPart() {
    testCase("a part whose id is not the BME280's is not brought up");
    bmeRegisters();
    part.reg[0xD0] = 0x55;
    fresh();
    Bme      d{};
    BusScope scope{};
    runUntil(d, [] { return false; }, 3s);
    check(!d.answering(), "not answering");
    check(part.written.empty(), "nothing written to it");
    checkEq(Log::warnings >= 1, true, "the mismatch is logged");
}

void busFaults() {
    testCase("a bus that fails transfers: the engine counts and brings the part up again");
    bmeRegisters();
    fresh();
    Bme      d{};
    BusScope scope{};
    runUntil(d, [&] { return d.samples() == 1; }, 2s);
    auto const bringUps = d.bringUps();
    for(int i = 0; i < 400 && d.errors() < 5; ++i) {
        Bus::overrunNext = true;
        Bus::handler();
        d.handler();
        Bus::tick();
        FakeClock::advance(100ms);
    }
    check(d.errors() >= 5, "errors counted");
    check(runUntil(
            d,
            [&] { return d.bringUps() > bringUps && d.samples() >= 1 && d.answering(); },
            5s),
          "brought up again, and sampling");
}

using Mpu = Kvasir::SPI::Device<Bus, FakeClock, Kvasir::SPI::Chips::Mpu9250, Spi::Cs>;

void mpu9250() {
    testCase("MPU-9250 on SPI: WHO_AM_I, the bring-up, a sample, a range changed at run time");
    part           = SpiRegisters{};   // registers 00h..7Fh, bit 7 = read
    part.writes    = SpiRegisters::Writes::increment;
    part.reg[0x75] = 0x71;
    // accel x = 16384 (1 g at +-2 g), temperature 0 (21 degC), gyro x = 131 (1 dps at 250)
    std::uint8_t const frame[]{0x40, 0x00, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0x83, 0, 0, 0, 0};
    for(std::size_t i = 0; i < sizeof frame; ++i) { part.reg[0x3B + i] = frame[i]; }
    fresh();
    Mpu      d{};
    BusScope scope{};
    check(runUntil(d, [&] { return d.samples() >= 1; }, 2s), "a sample");
    std::vector<std::pair<std::uint8_t, std::uint8_t>> const expect{
      {0x6B, 0x80},
      {0x68, 0x07},
      {0x6B, 0x01},
      {0x6A, 0x10},
      {0x19,    9},
      {0x1A,    3},
      {0x1D,    3},
      {0x1C, 0x00},
      {0x1B, 0x00}
    };
    check(part.written == expect,
          "reset, signal paths, PLL, I2C off, rate, both filters, the ranges");
    checkEq(d.latest().accel[0], 1'000'000, "1 g");
    checkEq(d.latest().gyro[0], 1000, "1 dps");
    checkEq(d.latest().temperature, 2100, "21 degC");
    bool slow = true;
    for(auto const& f : Bus::frames) {
        slow = slow && f.setup.hz <= 1'000'000U && f.setup.mode == 0;
    }
    check(slow, "every frame at 1 MHz or less, mode 0");
    d.set<Kvasir::SPI::Chips::Mpu9250::AccelConfig>(static_cast<std::uint8_t>(1U << 3U));   // +-4 g
    auto const before = d.samples();
    check(runUntil(d, [&] { return d.samples() >= before + 2; }, 1s), "sampling on");
    checkEq(part.reg[0x1C], 0x08, "ACCEL_FS_SEL 1 written");
    checkEq(d.latest().accel[0], 2'000'000, "the same counts are 2 g now");

    testCase("MPU-9250 on SPI: a part with another id gets nothing written");
    part           = SpiRegisters{};
    part.reg[0x75] = 0x68;   // an MPU-6050
    fresh();
    Mpu      other{};
    BusScope otherScope{};
    runUntil(other, [] { return false; }, 2s);
    check(!other.answering() && part.written.empty(), "not answering, nothing written");
}

/// A cascade of MAX7219s: each 16-bit word shifts through; on LOAD (CS) rising the last word in
/// each chip is latched (MAX7219.md:236). Words are recorded per chip, chip 0 the one at DIN.
struct Cascade {
    std::size_t                                                     chips{2};
    std::vector<std::uint8_t>                                       shift{};
    std::uint32_t                                                   seenRises{};
    std::vector<std::vector<std::pair<std::uint8_t, std::uint8_t>>> latched{};

    std::uint8_t exchange(std::uint8_t mosi) {
        if(Pins::rises[Spi::Cs::id] != seenRises) {
            seenRises = Pins::rises[Spi::Cs::id];
            latch();
        }
        shift.push_back(mosi);
        return 0x00;
    }

    /// The frame before this one is latched: the last 2 x chips bytes, the first word the far chip's.
    void latch() {
        if(shift.size() < 2 * chips) {
            shift.clear();
            return;
        }
        latched.resize(chips);
        auto const base = shift.size() - 2 * chips;
        for(std::size_t c = 0; c < chips; ++c) {
            // the word sent first ends in the last chip of the chain
            auto const at = base + 2 * (chips - 1 - c);
            latched[c].emplace_back(shift[at], shift[at + 1]);
        }
        shift.clear();
    }
};

Cascade cascade{};

void max7219() {
    testCase("MAX7219 x2 on SPI: shut down, configured, digits, intensity, then on; refreshed");
    cascade = Cascade{};
    fresh();
    Bus::exchange = [](std::uint8_t m) { return cascade.exchange(m); };
    using Display = Kvasir::SPI::Max7219<Bus, FakeClock, Spi::Cs, 2, 0xFF>;
    Display  d{};
    BusScope scope{};
    d.setDigit(1, 3, 7);
    runUntil(d, [&] { return d.answering() && !d.pending(); }, 1s);
    cascade.latch();
    check(d.answering(), "answering once the frames went out");
    auto const& c0 = cascade.latched[0];
    auto const& c1 = cascade.latched[1];
    check(!c0.empty() && c0.front() == std::pair<std::uint8_t, std::uint8_t>{0x0C, 0},
          "shutdown first");
    check(c0.back() == std::pair<std::uint8_t, std::uint8_t>{0x0C, 1}, "normal operation last");
    bool digit7 = false;
    for(auto const& w : c1) {
        digit7 = digit7 || w == std::pair<std::uint8_t, std::uint8_t>{0x04, 7};
    }
    bool blank = false;
    for(auto const& w : c0) {
        blank = blank || w == std::pair<std::uint8_t, std::uint8_t>{0x04, 0x0F};
    }
    check(digit7 && blank, "module 1 digit 3 is 7, module 0's is blank");
    bool test = false, scan = false, decode = false, intensity = false;
    for(auto const& w : c0) {
        test      = test || w == std::pair<std::uint8_t, std::uint8_t>{0x0F, 0};
        scan      = scan || w == std::pair<std::uint8_t, std::uint8_t>{0x0B, 7};
        decode    = decode || w == std::pair<std::uint8_t, std::uint8_t>{0x09, 0xFF};
        intensity = intensity || w == std::pair<std::uint8_t, std::uint8_t>{0x0A, 8};
    }
    check(test && scan && decode && intensity, "test off, scan limit, decode mode, intensity");
    auto const before = c0.size();
    runUntil(d, [] { return false; }, 1100ms);
    cascade.latch();
    check(cascade.latched[0].size() >= before + 12, "everything written again after a second");
    bool on = true;
    for(std::size_t i = before; i < cascade.latched[0].size(); ++i) {
        on = on && cascade.latched[0][i] != std::pair<std::uint8_t, std::uint8_t>{0x0C, 0};
    }
    check(on, "and never shut down by the refresh");
}

}   // namespace

int main() {
    bme280OnSpi();
    bmp280OnSpi();
    noPart(0xFF);
    noPart(0x00);
    wrongPart();
    busFaults();
    mpu9250();
    max7219();
    return finish();
}
