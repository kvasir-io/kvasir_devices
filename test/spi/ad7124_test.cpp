/// The AD7124 (SPI/chips/Ad7124.hpp) against a register model of the part over the queued SPI core.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "FakeQueuedSpi.hpp"
#include "FakeSpi.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <kvasir/Devices/SPI/chips/Ad7124.hpp>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
namespace A = Kvasir::SPI;

namespace {

/// The part: registers by size (Table 37), 64 ones a reset (AD7124-4.md:2714), POR_FLAG cleared by a
/// STATUS read (:4181), a conversion every 10 ms in continuous mode. A calibration keeps RDY high
/// (:4249) and leaves the part idle (:4251); a register written meanwhile is ignored and sets
/// ERROR.SPI_IGNORE_ERR (:3662, :4440).
struct Part {
    std::array<std::uint32_t, 0x40>                     reg{};
    std::uint8_t                                        id{0x07};
    bool                                                calibrating{};
    FakeClock::time_point                               calibrationDone{};
    std::uint32_t                                       seenRises{};
    bool                                                haveComms{};
    std::uint8_t                                        comms{};
    std::vector<std::uint8_t>                           bytes{};
    int                                                 ones{};
    std::vector<std::pair<std::uint8_t, std::uint32_t>> written{};
    FakeClock::time_point                               nextConversion{};
    std::uint8_t                                        channel{};
    std::uint32_t                                       code{0x800100};

    static constexpr std::uint8_t size(std::uint8_t r) {
        if(r == 0x00 || r == 0x05) { return 1; }
        if(r == 0x01 || (r >= 0x09 && r <= 0x20)) { return 2; }
        return 3;
    }

    void powerOn() {
        reg.fill(0);
        reg[0x00] = 0x90;   // RDY high, POR_FLAG
        reg[0x05] = id;
        reg[0x07] = 0x000040;
        haveComms = false;
    }

    bool continuous() const { return (reg[0x01] & 0x3CU) == 0; }

    /// From the data sheet, not the driver: one settling period zero-scale, four full-scale (:2756);
    /// tSETTLE = (4 x 32 x FS + dead time) / fCLK sinc4 (:2850), 3 x 32 sinc3 (:3026), dead time 61/95
    /// (:2882), fCLK by power mode (:2842), at the clock's slowest (-5 %, :2571).
    static std::chrono::microseconds calibrationTime(std::uint32_t adcControl,
                                                     std::uint32_t filter0) {
        auto const          mode    = (adcControl >> 2U) & 0xFU;
        auto const          power   = (adcControl >> 6U) & 0x3U;
        std::uint64_t const fs      = filter0 & 0x7FFU;
        auto const          type    = (filter0 >> 21U) & 0x7U;
        std::uint64_t const fclk    = power == 2 ? 614'400U : power == 1 ? 153'600U : 76'800U;
        std::uint64_t const dead    = fs == 1 ? 61U : 95U;
        std::uint64_t const sinc    = type == 2 ? 3U : 4U;
        std::uint64_t const periods = mode == 6 ? 4U : 1U;
        std::uint64_t const clocks  = periods * (sinc * 32U * fs + dead);
        // clocks / (0.95 fCLK), in microseconds
        return std::chrono::microseconds{
          static_cast<std::int64_t>(clocks * 20'000'000U / (19U * fclk))};
    }

    void tick() {
        if(calibrating && FakeClock::now() >= calibrationDone) {
            calibrating = false;
            reg[0x00] &= ~std::uint32_t{0x80U};                        // RDY low: done
            reg[0x01] = (reg[0x01] & ~std::uint32_t{0x3CU}) | 0x10U;   // idle mode after it
        }
        if(!continuous() || (reg[0x01] & 0x0400U) == 0 || FakeClock::now() < nextConversion) {
            return;
        }
        nextConversion = FakeClock::now() + 10ms;
        // the next enabled channel
        for(int i = 0; i < 16; ++i) {
            channel = static_cast<std::uint8_t>((channel + 1) % 16);
            if((reg[0x09 + channel] & 0x8000U) != 0) { break; }
        }
        reg[0x02] = code + channel;
        reg[0x00] = static_cast<std::uint8_t>((reg[0x00] & 0x70U) | channel);   // RDY low
    }

    std::uint8_t exchange(std::uint8_t mosi) {
        if(Pins::rises[Spi::Cs::id] != seenRises) {
            seenRises = Pins::rises[Spi::Cs::id];
            haveComms = false;
        }
        ones = mosi == 0xFF ? ones + 1 : 0;
        if(ones == 8) {   // 64 ones
            powerOn();
            ones = 0;
            return 0xFF;
        }
        if(!haveComms) {
            haveComms = true;
            comms     = mosi;
            bytes.clear();
            return 0xFF;
        }
        auto const r    = static_cast<std::uint8_t>(comms & 0x3FU);
        bool const read = (comms & 0x40U) != 0;
        auto const n    = size(r);
        if(read) {
            auto const i = bytes.size();
            bytes.push_back(0);
            std::uint32_t v = reg[r];
            if(r == 0x02 && i == n) {   // DATA_STATUS: the status after the data
                return static_cast<std::uint8_t>(reg[0x00]);
            }
            if(i >= n) { return 0xFF; }   // past the register: nothing (the reset's 64 ones)
            if(r == 0x02 && i + 1 == n) { reg[0x00] |= 0x80U; }     // data read: RDY high
            if(r == 0x00) { reg[0x00] &= ~std::uint32_t{0x10U}; }   // POR_FLAG cleared
            if(r == 0x06 && i + 1 == n) {
                reg[0x06] &= ~std::uint32_t{0x40U};
            }   // cleared when read
            return static_cast<std::uint8_t>(v >> (8U * (n - 1U - i)));
        }
        bytes.push_back(mosi);
        if(bytes.size() == n) {
            std::uint32_t v = 0;
            for(auto b : bytes) { v = (v << 8U) | b; }
            if(calibrating) {   // "registers cannot be accessed ... the write operation is ignored"
                reg[0x06] |= 0x40U;
                ignoredWrites.emplace_back(r, v);
                return 0xFF;
            }
            reg[r] = v;
            written.emplace_back(r, v);
            if(r == 0x01 && ((v >> 2U) & 0xFU) >= 5) {
                calibrating     = true;
                calibrationDone = FakeClock::now() + calibrationTime(v, reg[0x21]);
                reg[0x00] |= 0x80U;   // RDY high until it is done
            }
        }
        return 0xFF;
    }

    std::vector<std::pair<std::uint8_t, std::uint32_t>> ignoredWrites{};
};

Part part{};

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;

struct TwoCells : A::Ad7124Defaults {
    static constexpr std::array<A::Ad7124Channel, 2> Channels{
      {{1, 0}, {3, 2}}
    };
    static constexpr A::Ad7124Gain  Gain  = A::Ad7124Gain::x8;
    static constexpr A::Ad7124Power Power = A::Ad7124Power::full;
};

using Adc = A::Ad7124<Bus, FakeClock, Spi::Cs, TwoCells>;

constexpr std::uint32_t ConfigWord = A::Ad7124Detail::configValue(TwoCells::Polarity,
                                                                  TwoCells::ReferenceBuffers,
                                                                  TwoCells::InputBuffers,
                                                                  TwoCells::Reference,
                                                                  TwoCells::Gain);
static_assert(ConfigWord == 0x0863,
              "bipolar, input buffers, REFIN1, gain 8");

void fresh() {
    Bus::reset();
    Pins::reset();
    FakeClock::set(1s);
    Log::reset();
    part = Part{};
    part.powerOn();
    Bus::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
    Bus::exchange     = [](std::uint8_t m) { return part.exchange(m); };
}

template<typename D,
         typename Pred>
bool runUntil(D&                        d,
              Pred                      pred,
              std::chrono::milliseconds limit) {
    auto const end = FakeClock::now() + limit;
    while(FakeClock::now() < end) {
        FakeClock::advance(100us);
        part.tick();
        Bus::handler();
        d.handler();
        Bus::tick();
        if(pred()) { return true; }
    }
    return false;
}

/// TwoCells calibrates at sinc4, FS 384, mid power (full power not allowed full-scale, :2760):
/// 337.5 / 1350 ms against the driver's 339 / 1356 ms. A too short full-scale wait fails at "nothing
/// ignored": the zero-scale write lands inside the full-scale calibration.
void bringUp() {
    testCase("AD7124: reset, ID, setup, channels, calibration, continuous conversion");
    fresh();
    Adc adc{};
    check(runUntil(adc, [&] { return adc.present(); }, 5s), "comes up");
    checkEq(adc.id(), 0x07, "ID 0x07");
    check(part.ignoredWrites.empty(), "nothing ignored: every write came after its calibration");
    static_assert(Adc::Chip::ZeroScaleWait >= 338ms,
                  "one settling period covers the model's 337.5 ms");
    std::vector<std::uint8_t> order{};
    for(auto const& w : part.written) { order.push_back(w.first); }
    std::vector<std::uint8_t> const
      expect{0x07, 0x03, 0x19, 0x21, 0x09, 0x0A, 0x29, 0x01, 0x01, 0x01};
    check(order == expect,
          "ERROR_EN, IO_CONTROL_1, CONFIG_0, FILTER_0, two channels, OFFSET_0, two calibrations, "
          "ADC_CONTROL");
    checkEq(part.reg[0x19], ConfigWord, "CONFIG_0 as configValue() makes it");
    check(((part.written[7].second >> 2U) & 0xFU) == 0x6
            && ((part.written[8].second >> 2U) & 0xFU) == 0x5,
          "full-scale then zero-scale");
    check(((part.written[7].second >> 6U) & 0x3U) == 1, "the full-scale one in mid power");
    checkEq(part.written.back().second, 0x0480U, "continuous, full power, DATA_STATUS");
    bool mode3 = true;
    for(auto const& f : Bus::frames) {
        mode3 = mode3 && f.setup.mode == 3 && f.setup.hz <= 5'000'000U;
    }
    check(mode3, "mode 3, at most 5 MHz");
}

void conversions() {
    testCase("AD7124: the sequence's conversions, by channel index, codes signed");
    fresh();
    Adc adc{};
    runUntil(adc, [&] { return adc.present(); }, 5s);
    std::array<int, 2> seen{};
    runUntil(
      adc,
      [&] {
          if(auto const c = adc.nextConversion()) {
              ++seen[c->channel];
              checkEq(c->code,
                      static_cast<std::int32_t>(0x100 + (c->channel == 0 ? 0 : 1)),
                      "code = model + channel register");
          }
          return seen[0] >= 5 && seen[1] >= 5;
      },
      2s);
    check(seen[0] >= 5 && seen[1] >= 5, "both channels");
    checkEq(adc.missed(), 0U, "none missed at a 5 ms poll");
}

void resetItself() {
    testCase("AD7124: a part that reset itself (POR_FLAG) is configured again");
    fresh();
    Adc adc{};
    runUntil(adc, [&] { return adc.present(); }, 5s);
    part.powerOn();
    check(runUntil(
            adc,
            [&] { return adc.bringUps() == 2 && adc.present(); },
            3s),
          "brought up again");
    checkEq(part.reg[0x19], ConfigWord, "configured again");
}

void errorFlag() {
    testCase("AD7124: ERROR_FLAG makes the driver read ERROR");
    fresh();
    Adc adc{};
    runUntil(adc, [&] { return adc.present(); }, 5s);
    part.reg[0x06] = 0x000800;   // REF_DET_ERR
    part.reg[0x00] |= 0x40U;
    check(runUntil(adc, [&] { return adc.errorReports() >= 1; }, 1s), "read");
    checkEq(adc.errorRegister(), 0x000800U, "REF_DET_ERR");
}

void wrongId() {
    testCase("AD7124: an ID whose DEVICE_ID nibble is not an AD7124's is not configured");
    fresh();
    part.id = 0x27;   // device 2: neither the -4 (0) nor the -8 (1), revision 7
    part.powerOn();
    Adc adc{};
    runUntil(adc, [] { return false; }, 3s);
    check(!adc.present(), "not answering");
    check(part.written.empty(), "nothing written");
    check(Log::warnings >= 1, "logged as unidentified");

    testCase("AD7124: another silicon revision is the part all the same");
    for(auto const id : {std::uint8_t{0x08}, std::uint8_t{0x1A}}) {   // a -4 rev 8, a -8 rev 10
        fresh();
        part.id = id;
        part.powerOn();
        Adc other{};
        check(runUntil(other, [&] { return other.present(); }, 5s), "comes up");
        checkEq(other.id(), id, "the id as read");
        checkEq(part.reg[0x19], ConfigWord, "configured");
    }
}

void noPart() {
    testCase("AD7124: nothing on the bus");
    for(auto const level : {std::uint8_t{0x00}, std::uint8_t{0xFF}}) {
        fresh();
        Bus::exchange = {};
        Bus::floating = level;
        Adc adc{};
        runUntil(adc, [] { return false; }, 3s);
        check(!adc.present() && adc.conversions() == 0, "not answering, nothing converted");
        bool configured = false;
        for(auto const& f : Bus::frames) {
            configured = configured || (f.mosi.size() > 1 && f.mosi[0] == 0x07);
        }
        check(!configured, "no configuration written to nothing");
    }
}

}   // namespace

int main() {
    bringUp();
    conversions();
    resetItself();
    errorFlag();
    wrongId();
    noPart();
    return finish();
}
