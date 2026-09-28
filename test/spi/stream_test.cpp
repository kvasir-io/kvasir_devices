/// The SPI drivers that are not chip descriptions - NOR flash, ADS131M0x, ADS8675 - against models of
/// their parts.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "FakeQueuedSpi.hpp"
#include "FakeSpi.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <kvasir/Devices/SPI/chips/Ads131m0X.hpp>
#include <kvasir/Devices/SPI/chips/Ads8675.hpp>
#include <kvasir/Devices/SPI/chips/NorFlash.hpp>
#include <map>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
namespace A = Kvasir::SPI;

namespace {

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;

void fresh() {
    Bus::reset();
    Pins::reset();
    FakeClock::set(1s);
    Log::reset();
    Bus::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
}

template<typename Turn>
void run(std::chrono::milliseconds span,
         Turn                      turn) {
    auto const until = FakeClock::now() + span;
    while(FakeClock::now() < until) {
        FakeClock::advance(100us);
        Bus::handler();
        turn();
        Bus::tick();
    }
}

/// A W25Q-like flash: JEDEC EF 40 18, status BUSY for `busyFor` after a program or erase, WEL gating.
struct Flash {
    std::map<std::uint32_t, std::uint8_t> mem{};
    std::uint32_t                         seenRises{};
    std::vector<std::uint8_t>             frame{};
    bool                                  wel{};
    FakeClock::time_point                 busyUntil{};
    std::chrono::milliseconds             busyFor{2};

    bool busy() const { return FakeClock::now() < busyUntil; }

    std::uint8_t read(std::uint32_t a) const {
        auto const it = mem.find(a);
        return it == mem.end() ? 0xFF : it->second;
    }

    void end() {
        if(frame.empty()) { return; }
        auto const cmd  = frame[0];
        auto const addr = frame.size() >= 4 ? (std::uint32_t{frame[1]} << 16U)
                                                | (std::uint32_t{frame[2]} << 8U) | frame[3]
                                            : 0U;
        if(busy() && cmd != 0x05) {
            frame.clear();
            return;   // everything but Read Status ignored while busy
        }
        switch(cmd) {
        case 0x06: wel = true; break;
        case 0x20:
            if(wel) {
                for(std::uint32_t i = 0; i < 4096; ++i) { mem.erase(addr + i); }
                busyUntil = FakeClock::now() + busyFor;
                wel       = false;
            }
            break;
        case 0x02:
            if(wel) {
                for(std::size_t i = 4; i < frame.size(); ++i) {
                    mem[addr + static_cast<std::uint32_t>(i - 4)] = static_cast<std::uint8_t>(
                      read(addr + static_cast<std::uint32_t>(i - 4)) & frame[i]);
                }
                busyUntil = FakeClock::now() + busyFor;
                wel       = false;
            }
            break;
        default: break;
        }
        frame.clear();
    }

    std::uint8_t exchange(std::uint8_t mosi) {
        if(Pins::rises[Spi::Cs::id] != seenRises) {
            seenRises = Pins::rises[Spi::Cs::id];
            end();
        }
        auto const i = frame.size();
        frame.push_back(mosi);
        if(i == 0) { return 0xFF; }
        switch(frame[0]) {
        case 0x9F:
            return std::array<std::uint8_t, 3>{0xEF, 0x40, 0x18}[std::min<std::size_t>(i - 1, 2)];
        case 0x05: return static_cast<std::uint8_t>((busy() ? 0x01 : 0x00) | (wel ? 0x02 : 0x00));
        case 0x03:
            if(i >= 4) {
                auto const addr
                  = (std::uint32_t{frame[1]} << 16U) | (std::uint32_t{frame[2]} << 8U) | frame[3];
                return read(addr + static_cast<std::uint32_t>(i - 4));
            }
            return 0xFF;
        default: return 0xFF;
        }
    }
};

Flash flash{};

void norFlash() {
    testCase("NOR flash: JEDEC, sector erase, page program, read back");
    fresh();
    flash         = Flash{};
    Bus::exchange = [](std::uint8_t m) { return flash.exchange(m); };
    A::NorFlash<Bus, FakeClock, Spi::Cs> f{};
    auto const                           until = [&](auto pred) {
        run(2s, [&] {
            f.handler();
            if(pred()) { return; }
        });
    };
    check(f.readJedec(), "readJedec started");
    until([&] { return f.finished(); });
    check(f.takeDone() && f.lastOk() && f.present(), "answering");
    check(f.jedec().manufacturer == 0xEF && f.jedec().bytes() == (16U << 20), "Winbond, 16 MB");
    flash.mem[0x1000] = 0x12;
    check(f.eraseSector(0x1000), "erase started");
    run(100ms, [&] { f.handler(); });
    check(f.takeDone() && f.lastOk(), "erased");
    check(flash.read(0x1000) == 0xFF, "the byte is erased");
    std::array<std::byte, 16> page{};
    for(std::size_t i = 0; i < page.size(); ++i) {
        page[i] = std::byte{static_cast<std::uint8_t>(i * 3U)};
    }
    check(f.program(0x1000, page), "program started");
    run(100ms, [&] { f.handler(); });
    check(f.takeDone() && f.lastOk(), "programmed");
    std::array<std::byte, 16> back{};
    check(f.read(0x1000, back), "read started");
    run(20ms, [&] { f.handler(); });
    check(f.takeDone() && f.lastOk() && back == page, "the same bytes back");

    testCase("NOR flash: a part that stays busy past the timeout");
    flash.busyFor = 5s;
    check(f.eraseSector(0x2000), "erase started");
    run(2s, [&] { f.handler(); });
    check(f.takeDone() && f.result() == decltype(f)::Result::timedOut, "timedOut");
    checkEq(f.timeouts(), 1U, "counted");
}

/// An ADS131M02: a RESET answered by FF22h in the next frame's response word; data frames carry
/// channel words 0x123400 and 0xFEDC00.
struct M0x {
    std::uint32_t                seenRises{};
    std::size_t                  at{};
    bool                         resetSeen{};
    std::vector<std::uint8_t>    cmd{};
    std::array<std::uint8_t, 12> out{};

    std::uint8_t exchange(std::uint8_t mosi) {
        if(Pins::rises[Spi::Cs::id] != seenRises) {
            seenRises = Pins::rises[Spi::Cs::id];
            if(cmd.size() >= 2 && cmd[0] == 0x00 && cmd[1] == 0x11) { resetSeen = true; }
            out
              = resetSeen
                ? std::array<std::uint8_t, 12>{0xFF, 0x22, 0, 0x12, 0x34, 0, 0xFE, 0xDC, 0, 0, 0, 0}
                : std::array<std::uint8_t, 12>{};
            if(resetSeen && cmd.size() >= 2 && cmd[0] == 0x00 && cmd[1] == 0x00) {
                out[0] = 0x05;   // a status word after the acknowledge
                out[1] = 0x00;
            }
            cmd.clear();
            at = 0;
        }
        cmd.push_back(mosi);
        return out[at++ % out.size()];
    }
};

M0x m0x{};

struct M02 {
    static constexpr std::size_t Channels     = 2;
    static constexpr auto        StartupDelay = std::chrono::milliseconds{10};
};

void ads131m0x() {
    testCase("ADS131M02: reset acknowledged, two frames a reading, channel codes");
    fresh();
    m0x           = M0x{};
    Bus::exchange = [](std::uint8_t m) { return m0x.exchange(m); };
    A::Ads131m0X<Bus, FakeClock, Spi::Cs, Spi::Drdy, M02> adc{};
    Pins::level[Spi::Drdy::id] = false;
    check(adc.link() == Kvasir::Link::starting, "starting");
    // reset at 10 ms, the acknowledge a millisecond on, then a reading every 5 ms: the first
    // two are the fast-settling filter's (ADS131M04.md:1331-1334) and read off the part unkept
    run(19ms, [&] { adc.handler(); });
    check(adc.present() && adc.link() == Kvasir::Link::answering, "present after the acknowledge");
    checkEq(adc.discarded(), 2U, "the first two readings dropped");
    check(adc.samples() == 0 && !adc.valid(), "nothing published yet");
    run(6ms, [&] { adc.handler(); });
    checkEq(adc.samples(), 1U, "the third reading is the first one kept");
    run(175ms, [&] { adc.handler(); });
    check(adc.valid() && adc.samples() >= 5, "readings");
    checkEq(adc.discarded(), 2U, "and no more dropped");
    checkEq(adc.latest()[0], static_cast<std::int16_t>(0x1234), "channel 0");
    checkEq(adc.latest()[1], static_cast<std::int16_t>(0xFEDC), "channel 1");
    bool mode1 = true;
    for(auto const& f : Bus::frames) { mode1 = mode1 && f.setup.mode == 1 && f.mosi.size() == 12; }
    check(mode1, "mode 1, full frames of 4 words");
}

/// An ADS8675: WRITE_HWORD / READ_HWORD on RANGE_SEL, the register word in the frame after the read,
/// and conversions of code 0x2100 << 2 (offset binary: +256 counts).
struct Sar {
    std::uint32_t               seenRises{};
    std::vector<std::uint8_t>   cmd{};
    std::uint16_t               rangeSel{};
    bool                        readPending{};
    std::array<std::uint8_t, 4> out{};
    std::size_t                 at{};

    std::uint8_t exchange(std::uint8_t mosi) {
        if(cmd.empty() && at == 0) {
            // the frame's output is decided when it starts
            if(readPending) {
                out         = {static_cast<std::uint8_t>(rangeSel >> 8U),
                               static_cast<std::uint8_t>(rangeSel),
                               0,
                               0};
                readPending = false;
            } else {
                out = {static_cast<std::uint8_t>((0x2100U << 2U) >> 8U),
                       static_cast<std::uint8_t>(0x2100U << 2U),
                       0,
                       0};
            }
        }
        cmd.push_back(mosi);
        auto const v = out[at++];
        if(cmd.size() == 4) {
            if(cmd[0] == 0xD0 && cmd[1] == 0x14) {
                rangeSel = static_cast<std::uint16_t>((cmd[2] << 8U) | cmd[3]);
            }
            if(cmd[0] == 0xC8 && cmd[1] == 0x14) { readPending = true; }
            cmd.clear();
            at = 0;
        }
        return v;
    }
};

Sar sar{};

void ads8675() {
    testCase("ADS8675: RANGE_SEL written and read back, samples through the callback");
    fresh();
    sar           = Sar{};
    Bus::exchange = [](std::uint8_t m) { return sar.exchange(m); };
    std::vector<std::int32_t>                               got{};
    A::Ads8675<Bus, FakeClock, Spi::Cs, Spi::Rvs, Spi::Rst> adc{
      [&](Kvasir::Units::MicroVolt v) { got.push_back(Kvasir::Units::value(v)); }};
    Pins::level[Spi::Rvs::id] = true;
    run(1s, [&] { adc.handler(); });
    check(adc.ready(), "ready after the read-back");
    checkEq(sar.rangeSel, 0x0004U, "RANGE_SEL 0x04");
    for(int i = 0; i < 10; ++i) {
        adc.sampleCallback();   // CS high: a conversion
        adc.pinInterrupt();     // RVS: clock it out
        run(1ms, [&] { adc.handler(); });
    }
    checkEq(got.size(), 10U, "ten samples");
    checkEq(got.front(), 256 * 625 / 2, "+256 counts at 312.5 uV");
    checkEq(adc.errors(), 0U, "no errors");
    check(adc.link() == Kvasir::Link::answering, "answering");

    testCase("ADS8675: a sample due while the frame is still on the wire is skipped and logged");
    Log::reset();
    adc.sampleCallback();   // CS high: a conversion
    adc.pinInterrupt();     // RVS: the frame is submitted, on the wire until the next tick
    adc.sampleCallback();   // the next sample already: skipped, CS left alone
    checkEq(adc.skipped(), 1U, "skipped");
    checkEq(adc.errors(), 1U, "in errors() too");
    run(1ms, [&] { adc.handler(); });
    checkEq(got.size(), 11U, "the frame on the wire still delivered its sample");
    check(Log::warnings >= 1, "handler() logged it");

    testCase("ADS8675: a frame the master refuses puts CS back low so the next sample can start");
    static std::array<std::byte, 1> const one{std::byte{0xA5}};
    Bus::stall   = true;   // nothing completes: fill the master's queue
    int accepted = 0;
    for(int i = 0; i < 12; ++i) {
        if(Bus::submit(Bus::Request{.lines = {}, .tx = one})) { ++accepted; }
    }
    check(accepted < 12, "the queue is full");
    adc.sampleCallback();   // CS high
    check(Pins::level[Spi::Cs::id], "CS high for the conversion");
    adc.pinInterrupt();   // RVS: the frame is refused
    check(!Pins::level[Spi::Cs::id], "CS back low: the next sample makes an edge again");
    checkEq(adc.errors(), 2U, "the lost sample counted");
    check(!adc.inFlight(), "nothing in flight");
    Bus::stall = false;
    run(5ms, [&] { adc.handler(); });
}

}   // namespace

int main() {
    norFlash();
    ads131m0x();
    ads8675();
    return finish();
}
