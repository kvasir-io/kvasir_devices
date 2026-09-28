/// SPI/SdBlockDevice.hpp over the real SdCard driver and the card model (SdModel.hpp); with the fs library
/// next to this checkout (FS_ROOT) also FAT32 formatted, mounted and used on top of it.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "FakeQueuedSpi.hpp"
#include "FakeSpi.hpp"
#include "SdModel.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/SPI/SdBlockDevice.hpp>
#include <span>
#include <string>
#include <vector>
#if __has_include(<fs/fat/Volume.hpp>)
    #include <fs/fat/Format.hpp>
    #include <fs/fat/Volume.hpp>
    #define SD_BLOCK_DEVICE_TEST_HAS_FS 1
#else
    #define SD_BLOCK_DEVICE_TEST_HAS_FS 0
#endif

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;

namespace {

/// A clock that moves 10 us per read, so the adapter's own wait loops time out.
struct TickClock {
    using duration   = std::chrono::microseconds;
    using rep        = duration::rep;
    using period     = duration::period;
    using time_point = std::chrono::time_point<TickClock, duration>;

    static inline time_point current{};

    static time_point now() {
        current += 10us;
        return current;
    }
};

struct Tag {};

using Bus   = QueuedSpi::Bus<Tag>;
using Card  = Kvasir::SPI::SdCard<Bus, TickClock, Spi::Cs>;
using Dev   = Kvasir::SPI::SdBlockDevice<Card, Bus, TickClock>;
using Model = SdModel<Card>;

Model model{};

void fresh(std::uint32_t cSize = 61'047) {
    Bus::reset();
    Pins::reset();
    Log::reset();
    model             = Model{};
    model.cSize       = cSize;
    Bus::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
    Bus::exchange     = [](std::uint8_t m) { return model.exchange(m); };
    Bus::onFrame      = [](bool selected) {
        if(!selected) { model.deselected(); }
    };
    Bus::autoComplete
      = true;   // the adapter runs the master's handler itself: a transfer completes there
}

std::array<std::byte,
           512>
pattern(std::uint32_t lba) {
    std::array<std::byte, 512> b{};
    for(std::size_t i = 0; i < b.size(); ++i) {
        b[i] = std::byte{static_cast<std::uint8_t>(lba * 31U + i * 7U)};
    }
    return b;
}

void blocks() {
    testCase("blocks written and read back through the adapter");
    fresh();
    Card c{};
    Dev  d{c};
    check(d.waitUp(2s), "the card comes up");
    checkEq(d.blockCount(), (61'047U + 1U) * 1024U, "blockCount from the CSD");
    for(std::uint32_t lba : {0U, 1U, 4096U, 62'000'000U}) {
        auto const out = pattern(lba);
        check(d.writeBlock(lba, out), ("write " + std::to_string(lba)).c_str());
    }
    for(std::uint32_t lba : {0U, 1U, 4096U, 62'000'000U}) {
        std::array<std::byte, 512> in{};
        check(d.readBlock(lba, in) && in == pattern(lba),
              ("read back " + std::to_string(lba)).c_str());
    }
    checkEq(d.writes(), 4U, "4 writes counted");
    checkEq(d.reads(), 4U, "4 reads counted");
    checkEq(d.failures(), 0U, "no failures");
}

void faults() {
    testCase("a CRC error is read again; a rejected write and a lost data token are failures");
    fresh();
    Card c{};
    Dev  d{c};
    check(d.waitUp(2s), "up");
    auto const out = pattern(7);
    check(d.writeBlock(7, out), "write");
    std::array<std::byte, 512> in{};
    model.corruptNextRead = true;
    check(d.readBlock(7, in) && in == out, "a CRC error, then the right block");
    checkEq(d.retries(), 1U, "one retry");
    model.rejectNextWrite = true;
    check(!d.writeBlock(8, out), "a rejected write fails");
    model.noTokenNextRead = true;
    check(!d.readBlock(7, in), "a read that never gets its token fails");
    check(d.failures() == 2U, "two failures counted");
    check(d.readBlock(7, in) && in == out,
          "and the next read works again (the driver brought the card up again)");
}

#if SD_BLOCK_DEVICE_TEST_HAS_FS
void fat32() {
    testCase("FAT32 over the adapter, the driver and the card model");
    fresh(
      79);   // (79 + 1) x 512 KB = 40 MB: past fatgen's 66 600-sector floor for FAT32 (fatgen:526) after the 512 KB offset
    Card c{};
    Dev  d{c};
    check(d.waitUp(2s), "up");
    auto const g = fs::fat::format(d, fs::fat::FormatOptions{.label = "HOSTTEST"});
    check(g.has_value(), "format");
    fs::fat::Volume<Dev> v{d};
    check(v.mount().has_value(), "mount");
    check(v.mkdir("/a directory").has_value(), "mkdir");
    std::vector<std::byte> big(70'000);
    for(std::size_t i = 0; i < big.size(); ++i) {
        big[i] = std::byte{static_cast<std::uint8_t>(i * 13U + i / 511U)};
    }
    auto f = v.open("/a directory/a long file name.bin", fs::fat::Open::Write);
    check(f && v.write(*f, big).has_value() && v.close(*f).has_value(), "write 70 KB");
    check(v.unmount().has_value(), "unmount");
    fs::fat::Volume<Dev> w{d};
    check(w.mount().has_value(), "mount again");
    std::vector<std::byte> back(big.size());
    auto                   r = w.open("/A DIRECTORY/A LONG FILE NAME.BIN", fs::fat::Open::Read);
    auto const             n
      = r ? w.read(*r, back) : fs::Result<std::size_t>{std::unexpected(fs::Error::notFound)};
    check(n && *n == big.size() && back == big, "read back, case-insensitive, byte for byte");
    checkEq(d.failures(), 0U, "no block failed");
}
#endif

}   // namespace

int main() {
    blocks();
    faults();
#if SD_BLOCK_DEVICE_TEST_HAS_FS
    fat32();
#else
    std::printf("(fs not found next to this checkout: the FAT32 case is skipped)\n");
#endif
    return finish();
}
