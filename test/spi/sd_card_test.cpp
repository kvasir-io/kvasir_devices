/// The SD card driver (SPI/SdCard.hpp) against a card model (SdModel.hpp): bring-up, blocks, and each
/// fault the spec names.
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
#include <deque>
#include <kvasir/Devices/SPI/SdCard.hpp>
#include <map>
#include <support/FakeClock.hpp>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::SPI::SdResult;
using Kvasir::Test::Spi::Pins;

namespace {

struct Tag {};

using Bus  = QueuedSpi::Bus<Tag>;
using Cs   = Spi::Cs;
using Card = Kvasir::SPI::SdCard<Bus, FakeClock, Cs>;

using Model = Kvasir::Test::SdModel<Card>;

Model model{};

/// CS rises the model has seen: one mid-sequence ends what the card was saying.
std::uint32_t rises{};

void fresh() {
    Bus::reset();
    Pins::reset();
    FakeClock::reset();
    Log::reset();
    model             = Model{};
    rises             = Pins::rises[Cs::id];
    Bus::partSelected = [] { return !Pins::level[Cs::id]; };
    Bus::exchange     = [](std::uint8_t m) {
        if(Pins::rises[Cs::id] != rises) {
            rises = Pins::rises[Cs::id];
            model.deselected();
        }
        return model.exchange(m);
    };
    Bus::onFrame = [](bool selected) {
        if(!selected) { model.deselected(); }
    };
}

/// Turns of the loop: the card driver, the bus, the transfers completing, 100 us each.
template<typename Pred>
bool runUntil(Card& c,
              Pred  pred,
              int   turns = 200'000) {
    for(int i = 0; i < turns; ++i) {
        c.handler();
        Bus::handler();
        Bus::tick();
        FakeClock::advance(100us);
        if(pred()) { return true; }
    }
    return false;
}

void bringUp() {
    testCase("bring-up: clocks with CS high, CMD0..CMD10 at 400 kHz, then 25 MHz");
    fresh();
    Card c{};
    check(runUntil(c, [&] { return c.up(); }), "comes up");
    check(c.highCapacity(), "CCS from the OCR");
    checkEq(c.blocks(), (61'047U + 1U) * 1024U, "capacity from the CSD");
    checkEq(c.cid()[0], 0x03, "CID read");
    check(c.link() == Kvasir::Link::answering, "link answering");
    auto const& first = Bus::frames.front();
    check(!first.selected && first.mosi.size() == 10, "80 clocks with CS high first");
    checkEq(first.setup.hz, 400'000U, "at the identification clock");
    bool allSlow = true;
    for(auto const& f : Bus::frames) { allSlow = allSlow && f.setup.hz == 400'000U; }
    check(allSlow, "the whole bring-up at 400 kHz");
    checkEq(c.bringUps(), 1U, "one bring-up");
}

void writeAndReadBack() {
    testCase("a block written and read back, CRC16 checked, at 25 MHz");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    std::array<std::byte, 512> out{};
    for(std::size_t i = 0; i < out.size(); ++i) {
        out[i] = std::byte{static_cast<std::uint8_t>(i * 7U + 3U)};
    }
    checkEq(static_cast<int>(c.write(1234, out)), static_cast<int>(SdResult::ok), "write started");
    checkEq(static_cast<int>(c.write(1235, out)),
            static_cast<int>(SdResult::busy),
            "one at a time");
    std::optional<SdResult> done{};
    check(runUntil(c, [&] { return (done = c.takeDone()).has_value(); }), "write finishes");
    check(done == SdResult::ok, "write ok");
    check(model.blocks[1234][10] == static_cast<std::uint8_t>(73U),
          "the card has the data at LBA 1234");
    std::array<std::byte, 512> in{};
    checkEq(static_cast<int>(c.read(1234, in)),
            static_cast<int>(SdResult::ok),
            "read starts at once");
    check(runUntil(c, [&] { return (done = c.takeDone()).has_value(); }), "read finishes");
    check(done == SdResult::ok, "read ok");
    check(in == out, "same data back");
    checkEq(Bus::frames.back().setup.hz, 25'000'000U, "data at the data clock");
    check(c.ready(), "ready again");
}

void crcError() {
    testCase("a read whose CRC16 does not match is crcError, and the card stays up");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    std::array<std::byte, 512> in{};
    model.corruptNextRead = true;
    c.read(5, in);
    std::optional<SdResult> done{};
    runUntil(c, [&] { return (done = c.takeDone()).has_value(); });
    check(done == SdResult::crcError, "crcError");
    checkEq(c.crcErrors(), 1U, "counted");
    check(c.up() && c.ready(), "still up");
}

void noCard() {
    testCase("no card: never up, parked, tried again every second, CS released");
    fresh();
    Bus::exchange = {};   // MISO floats high
    Card c{};
    runUntil(c, [] { return false; }, 25'000);   // 2.5 s
    check(!c.up(), "not up");
    check(c.link() == Kvasir::Link::absent, "absent");
    check(Pins::level[Cs::id], "CS high");
    int starts = 0;
    for(auto const& f : Bus::frames) { starts += (!f.selected && f.mosi.size() == 10) ? 1 : 0; }
    check(starts >= 2 && starts <= 3, "a new attempt a second");
}

void busFault() {
    testCase("a transfer the bus fails ends the operation and brings the card up again");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    std::array<std::byte, 512> in{};
    c.read(9, in);   // its command is on the wire at once
    Bus::overrunNext = true;
    std::optional<SdResult> done{};
    runUntil(c, [&] { return (done = c.takeDone()).has_value(); });
    check(done == SdResult::busFault, "busFault");
    check(Pins::level[Cs::id], "CS released");
    check(runUntil(c, [&] { return c.up(); }), "up again");
    checkEq(c.bringUps(), 2U, "second bring-up");
}

void noToken() {
    testCase("no data token within 100 ms is a timeout");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    model.noTokenNextRead = true;
    std::array<std::byte, 512> in{};
    auto const                 t0 = FakeClock::now();
    c.read(9, in);
    std::optional<SdResult> done{};
    runUntil(c, [&] { return (done = c.takeDone()).has_value(); });
    check(done == SdResult::timeout, "timeout");
    check(FakeClock::now() - t0 >= 100ms && FakeClock::now() - t0 < 110ms,
          "after the spec's 100 ms");
}

void writeRejected() {
    testCase("a write the card does not accept is rejected, the card stays up");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    model.rejectNextWrite = true;
    std::array<std::byte, 512> out{};
    c.write(3, out);
    std::optional<SdResult> done{};
    runUntil(c, [&] { return (done = c.takeDone()).has_value(); });
    check(done == SdResult::rejected, "rejected");
    check(c.up(), "still up");
}

void writeBusyTooLong() {
    testCase("busy past the spec's 500 ms is a timeout, and the card is brought up again");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    model.busyBytes = 6000;   // 600 ms of busy at a byte a turn
    std::array<std::byte, 512> out{};
    auto const                 t0 = FakeClock::now();
    c.write(4, out);
    std::optional<SdResult> done{};
    runUntil(c, [&] { return (done = c.takeDone()).has_value(); });
    check(done == SdResult::timeout, "timeout");
    check(FakeClock::now() - t0 >= 500ms && FakeClock::now() - t0 < 520ms,
          "after the spec's 500 ms");
    check(!c.up(), "the card counts as lost");
    check(runUntil(c, [&] { return c.up(); }), "brought up again");
    checkEq(c.bringUps(), 2U, "second bring-up");
}

void rejectedInARowIsLost() {
    testCase("RejectedLost rejections in a row (MISO low, no card) end in a bring-up");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    model.rejectAllReads = true;
    std::array<std::byte, 512> in{};
    for(std::uint8_t i = 0; i < Card::RejectedLost; ++i) {
        checkEq(static_cast<int>(c.read(1, in)), static_cast<int>(SdResult::ok), "read starts");
        std::optional<SdResult> done{};
        runUntil(c, [&] { return (done = c.takeDone()).has_value(); });
        check(done == SdResult::rejected, "rejected");
        checkEq(c.rejectedInRow(), static_cast<std::uint8_t>(i + 1U), "counted");
        if(i + 1U < Card::RejectedLost) { check(c.up(), "still up: a bad block"); }
    }
    check(!c.up() && c.link() == Kvasir::Link::absent, "the last one: lost");
    check(Log::warnings >= 1, "logged");
    model.rejectAllReads = false;
    check(runUntil(c, [&] { return c.up(); }), "brought up again after RetryAfter");
    checkEq(c.bringUps(), 2U, "second bring-up");
    checkEq(c.rejectedInRow(), 0, "the streak starts over");
}

void loopStallCutsTheHold() {
    testCase(
      "a main loop that stalls past the master's HoldTimeout mid-sequence: a timeout, "
      "then a bring-up");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    model.tokenDelayBytes = 1000;   // the card takes its time before the token
    std::array<std::byte, 512> in{};
    c.read(2, in);
    runUntil(c, [] { return false; }, 30);   // the command out, R1 in, polling for the token
    check(!Pins::level[Cs::id], "in the sequence, CS low");
    // the loop stalls 60 ms; the master's handler runs first, as in a firmware
    FakeClock::advance(60ms);
    Bus::handler();
    checkEq(Bus::Core::holdTimeouts(), 1U, "the master released the hold");
    check(Pins::level[Cs::id], "CS went high mid-sequence");
    std::optional<SdResult> done{};
    runUntil(c, [&] { return (done = c.takeDone()).has_value(); });
    check(done == SdResult::timeout, "the read times out: the card dropped the command");
    check(runUntil(c, [&] { return c.up(); }), "brought up again");
    checkEq(c.bringUps(), 2U, "second bring-up");
}

void sharedBus() {
    testCase("another device's frames never land inside the card's CS-low sequence");
    fresh();
    Card c{};
    runUntil(c, [&] { return c.up(); });
    std::array<std::byte, 512> in{};
    c.read(1, in);
    static std::array<std::byte, 3> other{std::byte{0xA5}, std::byte{0xA5}, std::byte{0xA5}};
    static bool                     otherLow{};
    Kvasir::SPI::Lines const otherLines{[] { otherLow = true; }, [] { otherLow = false; }, nullptr};
    int                      submitted = 0;
    std::optional<SdResult>  done{};
    runUntil(c, [&] {
        if(submitted < 50) {
            if(Bus::submit(Bus::Request{.lines = otherLines, .tx = other})) { ++submitted; }
        }
        return (done = c.takeDone()).has_value();
    });
    check(done == SdResult::ok, "the read still works");
    // every frame of the other device was sent with the card's CS high
    bool clean = true;
    for(auto const& f : Bus::frames) {
        if(f.mosi.size() == 3 && f.mosi[0] == 0xA5) { clean = clean && !f.selected; }
    }
    check(clean, "never with the card selected");
    check(submitted > 0, "and it did get turns");
    for(int i = 0; i < 20; ++i) {
        Bus::handler();
        Bus::tick();
    }
    check(!otherLow, "its CS released too");
}

}   // namespace

int main() {
    bringUp();
    writeAndReadBack();
    crcError();
    noCard();
    busFault();
    noToken();
    writeRejected();
    writeBusyTooLong();
    rejectedInARowIsLost();
    loopStallCutsTheHold();
    sharedBus();
    return finish();
}
