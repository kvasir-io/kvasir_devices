/// The SPI NOR flash driver (SPI/chips/NorFlash.hpp) against a model of a W25Q128JV on the queued master's fake:
/// the bring-up, a read that lands in the caller's buffer and nowhere else, program and erase with their status
/// polls and timeouts, the settle after an operation that did not end well, a failed frame, and the refusals.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/SPI/chips/NorFlash.hpp>
#include <span>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;

namespace {

/// A W25Q128JV (data sheet W25Q128JV.md), only what the driver uses. An instruction is taken when CS goes
/// high after it (8.2.x, e.g. Sector Erase md l.1309), here: on the first tick() after the CS rise. While BUSY,
/// everything but Read Status is ignored (7.1.1, md l.653). Program and erase need WEL (Write Enable 06h, md
/// l.1013) and clear it when done (md l.657). Page Program only clears bits; an erase sets the sector to 0xFF.
/// JEDEC id 9Fh: EFh 40h 18h (8.1.1, md l.897-900, W25Q128JV-IQ/JQ). The default times are the typical tPP 0.4 ms
/// rounded up to 1 ms and tSE 45 ms (AC characteristics, md l.1983-1984).
struct Part {
    std::vector<std::uint8_t> mem = std::vector<std::uint8_t>(64 * 1024, 0xFF);

    std::chrono::milliseconds programTime{1};
    std::chrono::milliseconds eraseTime{45};

    bool                  wel{};
    bool                  busy{};
    FakeClock::time_point busyUntil{};

    // the instruction being shifted in
    std::uint32_t             seenRises{};
    bool                      inFrame{};
    std::vector<std::uint8_t> frame{};
    std::uint32_t             readAt{};

    std::size_t ignored{};    ///< instructions dropped while busy
    std::size_t executed{};   ///< program/erase instructions taken

    [[nodiscard]] std::uint8_t status() const {
        return static_cast<std::uint8_t>((busy ? 0x01U : 0U) | (wel ? 0x02U : 0U));
    }

    static std::uint32_t addressOf(std::vector<std::uint8_t> const& f) {
        return static_cast<std::uint32_t>(f[1]) << 16U | static_cast<std::uint32_t>(f[2]) << 8U
             | f[3];
    }

    /// CS went high: the instruction shifted in is taken now.
    void endOfFrame() {
        inFrame = false;
        if(frame.empty()) { return; }
        auto const cmd = frame[0];
        if(busy && cmd != 0x05) {
            ++ignored;
            return;
        }
        auto const now = FakeClock::now();
        switch(cmd) {
        case 0x06: wel = true; break;
        case 0x02:
            if(wel && frame.size() >= 4) {
                auto const at   = addressOf(frame);
                auto const page = at & ~0xFFU;
                for(std::size_t i = 4; i < frame.size(); ++i) {
                    // past the end of the page the address wraps inside it (8.2.13)
                    auto const to = page | ((at + static_cast<std::uint32_t>(i - 4)) & 0xFFU);
                    mem[to % mem.size()] &= frame[i];
                }
                wel       = false;
                busy      = true;
                busyUntil = now + programTime;
                ++executed;
            }
            break;
        case 0x20:
            if(wel && frame.size() == 4) {
                auto const at = addressOf(frame) & ~0xFFFU;
                std::fill_n(mem.begin() + static_cast<std::ptrdiff_t>(at % mem.size()), 4096, 0xFF);
                wel       = false;
                busy      = true;
                busyUntil = now + eraseTime;
                ++executed;
            }
            break;
        default: break;
        }
    }

    void tick() {
        if(inFrame && Pins::rises[Spi::Cs::id] != seenRises) { endOfFrame(); }
        if(busy && FakeClock::now() >= busyUntil) { busy = false; }
    }

    std::uint8_t exchange(std::uint8_t mosi) {
        if(!inFrame || Pins::rises[Spi::Cs::id] != seenRises) {
            if(inFrame) { endOfFrame(); }
            seenRises = Pins::rises[Spi::Cs::id];
            inFrame   = true;
            frame.clear();
        }
        frame.push_back(mosi);
        auto const pos = frame.size() - 1;
        switch(frame[0]) {
        case 0x05: return pos >= 1 ? status() : 0xFF;
        case 0x9F:
            if(busy) { return 0xFF; }
            if(pos == 1) { return 0xEF; }
            if(pos == 2) { return 0x40; }
            if(pos == 3) { return 0x18; }
            return 0xFF;
        case 0x03:
            if(busy) { return 0xFF; }
            if(pos == 3) { readAt = addressOf(frame); }
            if(pos >= 4) { return mem[readAt++ % mem.size()]; }
            return 0xFF;
        default: return 0xFF;
        }
    }
};

Part part{};

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;

/// A sector erase that the part (45 ms) outlasts.
struct ShortErase : Kvasir::SPI::NorFlashDefaults {
    static constexpr auto sectorEraseTimeout = std::chrono::milliseconds{20};
};

using Flash          = Kvasir::SPI::NorFlash<Bus, FakeClock, Spi::Cs>;
using ImpatientFlash = Kvasir::SPI::NorFlash<Bus, FakeClock, Spi::Cs, ShortErase>;

using BusScope = Spi::BusScope<Bus>;

constexpr Spi::Loop loop{.step = 100us, .order = Spi::ClockOrder::first};

void fresh() {
    Spi::Harness::fresh<Bus>([](std::uint8_t m) { return part.exchange(m); });
    part = {};
}

/// Turns until the operation under way has finished, at most `within`.
template<typename F>
bool untilDone(F&                        f,
               std::chrono::milliseconds within) {
    return Spi::Harness::runUntil<Bus>(
      f,
      [&] { return f.finished(); },
      within,
      loop,
      [] { part.tick(); });
}

template<typename F>
void run(F&                        f,
         std::chrono::milliseconds span) {
    Spi::Harness::runFor<Bus>(f, span, loop, [] { part.tick(); });
}

std::size_t framesStartingWith(std::uint8_t cmd) {
    return static_cast<std::size_t>(std::ranges::count_if(Bus::frames, [&](auto const& fr) {
        return fr.selected && !fr.mosi.empty() && fr.mosi[0] == cmd;
    }));
}

template<typename F>
void bringUp(F& f) {
    check(f.readJedec(), "readJedec() taken");
    check(untilDone(f, 100ms), "the bring-up finishes");
    check(f.takeDone(), "takeDone() once");
}

void jedec() {
    testCase("NorFlash: the bring-up wakes and resets the part, then reads the JEDEC id");
    fresh();
    Flash          f{};
    BusScope const scope{};
    check(!f.answering(), "not answering before the id");
    bringUp(f);
    checkEq(f.result(), Flash::Result::ok, "ok");
    checkEq(f.jedec().manufacturer, 0xEF, "Winbond");
    checkEq(f.jedec().type, 0x40, "memory type");
    checkEq(f.jedec().capacity, 0x18, "capacity 2^24");
    checkEq(f.jedec().bytes(), std::uint64_t{16} << 20, "16 MB");
    check(f.answering(), "answering");
    checkEq(f.bringUps(), 1, "one bring-up");
    check(!f.takeDone(), "takeDone() is true once per completion");
    std::vector<std::uint8_t> firsts{};
    for(auto const& fr : Bus::frames) {
        if(fr.selected && !fr.mosi.empty()) { firsts.push_back(fr.mosi[0]); }
    }
    check(firsts == std::vector<std::uint8_t>{0xAB, 0x66, 0x99, 0x9F},
          "0xAB, 0x66, 0x99, then 0x9F on the wire");
    check(std::ranges::all_of(Bus::frames, [](auto const& fr) { return fr.setup.mode == 0; }),
          "mode 0");
}

void readIntoCaller() {
    testCase("NorFlash: a read lands in the caller's span and nothing around it is written");
    fresh();
    for(std::size_t i = 0; i < 256; ++i) {
        part.mem[0x1200 + i] = static_cast<std::uint8_t>(i ^ 0x5A);
    }
    Flash          f{};
    BusScope const scope{};
    bringUp(f);
    std::array<std::byte, 64> buffer{};
    buffer.fill(std::byte{0xCC});
    auto const into = std::span{buffer}.subspan(8, 32);
    check(f.read(0x1210, into), "read() taken");
    check(f.busy(), "busy while the read is out");
    check(untilDone(f, 50ms), "the read finishes");
    checkEq(f.result(), Flash::Result::ok, "ok");
    bool same = true;
    for(std::size_t i = 0; i < into.size(); ++i) {
        same = same && std::to_integer<std::uint8_t>(into[i]) == part.mem[0x1210 + i];
    }
    check(same, "the 32 bytes read are the part's");
    bool guards = true;
    for(std::size_t i = 0; i < 8; ++i) {
        guards = guards && buffer[i] == std::byte{0xCC} && buffer[40 + i] == std::byte{0xCC};
        guards = guards && buffer[48 + i] == std::byte{0xCC} && buffer[56 + i] == std::byte{0xCC};
    }
    check(guards, "the bytes before and after the span are untouched");
    check(Bus::frames.size() >= 2, "frames on the wire");
    auto const& header = Bus::frames[Bus::frames.size() - 2];
    auto const& data   = Bus::frames.back();
    checkEq(header.mosi.size(), 4U, "the command phase: 4 bytes");
    check(header.mosi[0] == 0x03 && header.mosi[1] == 0x00 && header.mosi[2] == 0x12
            && header.mosi[3] == 0x10,
          "03h and the 24-bit address MSB first");
    checkEq(data.mosi.size(), 32U, "the data phase: the span's 32 bytes, under the same CS");
    check(header.selected && data.selected, "both phases with the part selected");
    check(std::ranges::all_of(data.mosi, [](std::uint8_t b) { return b == 0xFF; }),
          "MOSI 0xFF during the data (rx only: no copy of the page in the driver)");
    checkEq(framesStartingWith(0x05), 1U, "one status read: the settle after the bring-up's reset");
    check(f.read(0x1200, into.first(4)), "a second read");
    check(untilDone(f, 50ms) && f.lastOk(), "ok");
    checkEq(framesStartingWith(0x05), 1U, "no settle after an operation that ended ok");
    checkEq(std::to_integer<int>(into[0]), 0x5A, "its first byte");
}

void programAndErase() {
    testCase("NorFlash: program polls the status until BUSY clears, then the data reads back");
    fresh();
    part.programTime = 3ms;   // tPP max (md l.1983)
    Flash          f{};
    BusScope const scope{};
    bringUp(f);
    std::array<std::byte, 16> data{};
    for(std::size_t i = 0; i < data.size(); ++i) {
        data[i] = std::byte{static_cast<std::uint8_t>(0xA0 + i)};
    }
    auto const from = FakeClock::now();
    check(f.program(0x0300, data), "program() taken");
    data.fill(std::byte{0});   // copied into the frame: the caller's buffer may go
    auto const polls = framesStartingWith(0x05);
    check(untilDone(f, 50ms), "the program finishes");
    auto const took = FakeClock::now() - from;
    checkEq(f.result(), Flash::Result::ok, "ok");
    check(took >= 3ms && took < 6ms, "after the part's 3 ms, within a poll interval or two");
    check(framesStartingWith(0x05) - polls >= 3, "the status polled about every millisecond");
    checkEq(framesStartingWith(0x06), 1U, "one write enable");
    checkEq(part.executed, 1U, "one instruction executed by the part");
    checkEq(part.ignored, 0U, "nothing sent while it was busy but Read Status");
    bool written = true;
    for(std::size_t i = 0; i < 16; ++i) { written = written && part.mem[0x0300 + i] == 0xA0 + i; }
    check(written, "the page holds the data copied at program()");

    testCase("NorFlash: a sector erase sets the sector to 0xFF and leaves the next one alone");
    part.mem[0x1000] = 0x00;
    part.mem[0x0FFF] = 0x11;
    part.mem[0x2000] = 0x22;
    check(!f.eraseSector(0x1001), "an address off a sector boundary is refused");
    check(f.eraseSector(0x1000), "eraseSector() taken");
    check(untilDone(f, 200ms), "the erase finishes");
    checkEq(f.result(), Flash::Result::ok, "ok");
    checkEq(part.mem[0x1000], 0xFF, "erased");
    checkEq(part.mem[0x0FFF], 0x11, "the sector before untouched");
    checkEq(part.mem[0x2000], 0x22, "the sector after untouched");
    checkEq(f.timeouts(), 0U, "no timeout");
    checkEq(f.errors(), 0U, "no failed frame");
}

void eraseTimesOut() {
    testCase(
      "NorFlash: a part busy past the timeout ends timedOut, and the next operation waits for BUSY "
      "first");
    fresh();
    ImpatientFlash f{};
    BusScope const scope{};
    bringUp(f);
    auto const from = FakeClock::now();
    check(f.eraseSector(0), "eraseSector() taken");
    check(untilDone(f, 200ms), "it ends");
    checkEq(f.result(), ImpatientFlash::Result::timedOut, "timedOut");
    checkEq(f.timeouts(), 1U, "one timeout counted");
    check(FakeClock::now() - from < 25ms, "at the 20 ms timeout, not the part's 45 ms");
    check(part.busy, "the part is still erasing");
    std::array<std::byte, 4> into{};
    check(f.read(0x10, into), "a read after it is taken");
    check(untilDone(f, 100ms), "and finishes");
    checkEq(f.result(), ImpatientFlash::Result::ok, "ok: it waited for the erase to end");
    checkEq(part.ignored, 0U, "no instruction went to the busy part");
    checkEq(std::to_integer<int>(into[0]), 0xFF, "the erased byte read");
}

void failedFrame() {
    testCase(
      "NorFlash: a failed frame fails the operation, counts once, and the next one settles first");
    fresh();
    Flash          f{};
    BusScope const scope{};
    bringUp(f);
    part.mem[0x40] = 0x12;
    std::array<std::byte, 8> into{};
    check(f.read(0x40, into), "a first read (it settles after the bring-up)");
    check(untilDone(f, 50ms) && f.lastOk(), "ok");
    into.fill(std::byte{0xCC});
    // The overrun hits the data phase (the second frame of the read): the bytes are already in the span, the
    // read still fails - read()'s contract: on a result other than ok the span's contents are undefined.
    std::array<std::byte, 12> around{};
    around.fill(std::byte{0xCC});
    auto const        span   = std::span{around}.subspan(2, 8);
    std::size_t const before = Bus::frames.size();
    Bus::onFrame             = [&](bool) {
        if(Bus::frames.size() == before + 1) { Bus::overrunNext = true; }
    };
    check(f.read(0x40, span), "read() taken");
    check(untilDone(f, 50ms), "it ends");
    Bus::onFrame = {};
    checkEq(Bus::frames.size(), before + 2, "both phases went out");
    checkEq(f.result(), Flash::Result::failed, "failed");
    checkEq(f.errors(), 1U, "one error");
    check(around[0] == std::byte{0xCC} && around[1] == std::byte{0xCC}
            && around[10] == std::byte{0xCC} && around[11] == std::byte{0xCC},
          "nothing outside the span written");
    into.fill(std::byte{0xCC});
    run(f, 50ms);
    checkEq(f.errors(), 1U, "still one: nothing retried by itself");
    auto const statusBefore = framesStartingWith(0x05);
    check(f.read(0x40, into), "the next read is taken");
    check(untilDone(f, 50ms), "and finishes");
    checkEq(f.result(), Flash::Result::ok, "ok");
    check(framesStartingWith(0x05) > statusBefore, "after a status read (the settle)");
    checkEq(std::to_integer<int>(into[0]), 0x12, "the byte read");
    checkEq(f.errors(), 1U, "the count did not move");
}

void refusals() {
    testCase("NorFlash: what the operations refuse");
    fresh();
    Flash          f{};
    BusScope const scope{};
    bringUp(f);
    std::array<std::byte, Flash::PageSize + 1> big{};
    check(!f.read(0, {}), "an empty read");
    check(!f.read(0, big), "a read of more than a page");
    check(!f.read(Flash::AddressSpace - 2, std::span{big}.first(4)), "a read past 16 MB");
    check(!f.program(0, {}), "an empty program");
    check(!f.program(0, big), "a program of more than a page");
    check(!f.program(0x00F0, std::span{big}.first(0x20)), "a program across a page boundary");
    check(f.read(0, std::span{big}.first(4)), "a read taken");
    check(!f.read(0, std::span{big}.first(4)), "a second operation while the first runs");
    check(!f.eraseChip(), "nor an erase");
    check(untilDone(f, 50ms), "the first one finishes");
    checkEq(f.result(), Flash::Result::ok, "ok");
}

}   // namespace

int main() {
    jedec();
    readIntoCaller();
    programAndErase();
    eraseTimesOut();
    failedFrame();
    refusals();
    return finish();
}
