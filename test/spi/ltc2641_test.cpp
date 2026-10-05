/// LTC2641: the 16-bit frame for every resolution (LTC2641_LTC2642.md, Serial Interface and Tables
/// 1a-1c), the code-to-voltage line, and the Dac over the queued master - one 16-bit frame per
/// write, CS high after it, the newest code winning while a frame is out.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "FakeQueuedSpi.hpp"
#include "FakeSpi.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/SPI/chips/Ltc2641.hpp>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
namespace D = Kvasir::SPI::Ltc2641;

namespace {

// The frame, at compile time

// 16 bits: the code itself, MSB first (Figure 1a)
static_assert(D::frame(0x0000) == 0x0000);
static_assert(D::frame(0x8000) == 0x8000);
static_assert(D::frame(0xFFFF) == 0xFFFF);
static_assert(D::frame(0x1234) == 0x1234);
static_assert(D::frame<16>(0x1'0000) == 0xFFFF,
              "above full scale saturates, never wraps to 0");
// 14 bits: left-justified, two don't-care bits after the LSB (Table 1b: 1111 1111 1111 11xx)
static_assert(D::frame<14>(0x3FFF) == 0xFFFC);
static_assert(D::frame<14>(0x2000) == 0x8000);
static_assert(D::frame<14>(0x0001) == 0x0004);
static_assert(D::frame<14>(0x4000) == 0xFFFC);
// 12 bits: four don't-care bits (Table 1c: 0000 0000 0001 xxxx is 1 LSB)
static_assert(D::frame<12>(0x0001) == 0x0010);
static_assert(D::frame<12>(0x0ABC) == 0xABC0);
static_assert(D::frame<12>(0x1000) == 0xFFF0);
// on the wire, byte by byte
static_assert(D::frameBytes(0x1234)
              == std::array{std::byte{0x12},
                            std::byte{0x34}});
static_assert(D::frameBytes<12>(0x0ABC)
              == std::array{std::byte{0xAB},
                            std::byte{0xC0}});

// VOUT = VREF x code / 2^N (Table 1a: 8000h is VREF/2, FFFFh VREF x 65535/65536)
static_assert(D::voltage(0x8000,
                         Kvasir::Units::microVolt(2'500'000))
              == Kvasir::Units::microVolt(1'250'000));
static_assert(D::voltage(0xFFFF,
                         Kvasir::Units::microVolt(4'096'000))
              == Kvasir::Units::microVolt(4'095'937));
static_assert(D::voltage<12>(0x800,
                             Kvasir::Units::microVolt(4'096'000))
              == Kvasir::Units::microVolt(2'048'000));
static_assert(D::voltage(0,
                         Kvasir::Units::microVolt(4'096'000))
              == Kvasir::Units::microVolt(0));

// The timing the data sheet gives (md:228-238), and what 25 MHz leaves of it: a 20 ns half period
// against t3/t4 9 ns and the DIN setup t1 10 ns (data changes on the falling edge, half a period
// before the rising one).
static_assert(D::MaxClock == Kvasir::Units::hertz(50'000'000));
static_assert(D::SclkHighMinNs == 9 && D::SclkLowMinNs == 9 && D::DinSetupMinNs == 10);
static_assert(D::CsHighMinNs == 10 && D::LastSclkToCsHighMinNs == 8 && D::CsLowToSclkMinNs == 8);
static_assert(1'000'000'000U / 25'000'000U / 2U >= D::SclkHighMinNs
              && 1'000'000'000U / 25'000'000U / 2U >= D::DinSetupMinNs);

// The Dac over the queued master

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;
using Cs  = Spi::Cs;

/// The part: DIN shifted in on each byte while CS is low, the word latched when CS rises after
/// exactly 16 bits (md:529).
struct Part {
    std::vector<std::uint16_t> latched{};
    std::uint32_t              shift{};
    int                        bits{};
    std::uint32_t              csRisesSeen{};

    void latch() {
        if(Pins::rises[Cs::id] == csRisesSeen) { return; }
        csRisesSeen = Pins::rises[Cs::id];
        if(bits == 16) { latched.push_back(static_cast<std::uint16_t>(shift)); }
        bits  = 0;
        shift = 0;
    }

    std::uint8_t exchange(std::uint8_t mosi) {
        latch();
        shift = ((shift << 8U) | mosi) & 0xFFFFU;
        bits += 8;
        return 0xFF;
    }
};

Part* part{};

std::uint8_t onByte(std::uint8_t mosi) { return part->exchange(mosi); }

void fresh(Part& p) {
    Bus::reset();
    Pins::reset();
    FakeClock::set(1s);
    Log::reset();
    part              = &p;
    Bus::partSelected = [] { return !Pins::level[Cs::id]; };
    Bus::exchange     = onByte;
}

void settle(Part& p) {
    for(int i = 0; i < 4; ++i) {
        Bus::handler();
        Bus::tick();
        p.latch();
    }
}

void writes() {
    testCase("LTC2641: one 16-bit frame a write, latched on CS high");
    Part p{};
    fresh(p);
    D::Dac<Bus, Cs> dac{};
    check(Pins::level[Cs::id], "CS high at construction");
    dac.write(0x1234);
    settle(p);
    checkEq(Bus::frames.size(), std::size_t{1}, "one frame");
    auto const& f = Bus::frames.front();
    check(f.wide && f.mosi == std::vector<std::uint8_t>{0x12, 0x34}, "a 16-bit frame, MSB first");
    check(f.setup.mode == 0 && f.setup.hz == 25'000'000, "mode 0 at 25 MHz");
    check(Pins::level[Cs::id], "CS high after it: the output is updated");
    check(p.latched == std::vector<std::uint16_t>{0x1234}, "the part latched it");
    checkEq(dac.writes(), 1U, "counted");
    checkEq(dac.lastWritten(), std::uint16_t{0x1234}, "last written");
    check(!dac.busy(), "idle");
    Bus::reset();
}

void newestWins() {
    testCase("LTC2641: writes while a frame is out - the newest goes next");
    Part p{};
    fresh(p);
    D::Dac<Bus, Cs, 12> dac{};
    Bus::stall = true;
    dac.write(0x001);
    Bus::handler();
    check(dac.busy(), "the first frame is out");
    dac.write(0x002);
    dac.write(0x003);
    Bus::stall = false;
    settle(p);
    check(p.latched == std::vector<std::uint16_t>{0x0010, 0x0030}, "first, then the newest only");
    checkEq(dac.writes(), 2U, "two frames");
    check(!dac.busy(), "idle");
    Bus::reset();
}

void failed() {
    testCase("LTC2641: a failed frame is counted, the next write goes out");
    Part p{};
    fresh(p);
    D::Dac<Bus, Cs> dac{};
    Bus::overrunNext = true;
    dac.write(0xAAAA);
    settle(p);
    checkEq(dac.failures(), 1U, "failed");
    check(!dac.busy(), "not stuck busy");
    dac.write(0x5555);
    settle(p);
    checkEq(dac.writes(), 1U, "the next one went out");
    checkEq(dac.lastWritten(), std::uint16_t{0x5555}, "last written");
    Bus::reset();
}

}   // namespace

int main() {
    writes();
    newestWins();
    failed();
    return finish();
}
