/// The SSD1680 over Display::SpiStream and the queued SPI master: bring-up, D/C per byte, and a
/// failed transfer ending in failed(), then restart().
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "../spi/FakeQueuedSpi.hpp"
#include "../spi/FakeSpi.hpp"

#include <chrono>
#include <cstdint>
#include <kvasir/Devices/Display/SpiStream.hpp>
#include <kvasir/Devices/Display/Ssd1680.hpp>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
namespace D = Kvasir::Display;
using Cmd   = D::Ssd1680Cmd;

namespace {

struct Dc {
    static inline bool dataMode{};

    static void command() { dataMode = false; }

    static void data() { dataMode = true; }
};

/// High a few ms after SW reset and for a refresh after MASTER ACTIVATION, as the controller is.
struct Busy {
    static inline FakeClock::time_point until{};

    static bool busy() { return FakeClock::now() < until; }
};

struct Reset {
    static void hold() {}

    static void release() {}
};

/// Every byte the panel takes: a command with D/C low, its parameters with it high.
struct PanelModel {
    struct Command {
        std::uint8_t              cmd{};
        std::vector<std::uint8_t> params{};
    };

    std::vector<Command> commands{};
    std::size_t          orphanData{};

    std::uint8_t exchange(std::uint8_t mosi) {
        if(!Dc::dataMode) {
            commands.push_back({mosi, {}});
            if(mosi == Cmd::SwReset) { Busy::until = FakeClock::now() + 3ms; }
            if(mosi == Cmd::MasterActivation) { Busy::until = FakeClock::now() + 400ms; }
        } else if(commands.empty()) {
            ++orphanData;
        } else {
            commands.back().params.push_back(mosi);
        }
        return 0xFF;
    }

    bool has(std::uint8_t c) const {
        for(auto const& x : commands) {
            if(x.cmd == c) { return true; }
        }
        return false;
    }
};

PanelModel model{};

struct Tag {};

using Bus    = QueuedSpi::Bus<Tag>;
using Stream = D::SpiStream<Bus, Spi::Cs>;
using Panel  = D::Ssd1680<FakeClock, Stream, Reset, Dc, Busy, D::Ssd1680Defaults>;

static_assert(D::SpiBus<Stream>);

void fresh() {
    Bus::reset();
    Pins::reset();
    FakeClock::set(1s);
    Log::reset();
    model             = PanelModel{};
    Busy::until       = {};
    Bus::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
    Bus::exchange     = [](std::uint8_t m) { return model.exchange(m); };
}

template<typename Pred>
bool runUntil(Pred                      pred,
              std::chrono::milliseconds limit) {
    auto const end = FakeClock::now() + limit;
    while(FakeClock::now() < end) {
        FakeClock::advance(1ms);
        Panel::handler();
        Bus::tick();
        if(pred()) { return true; }
    }
    return false;
}

void bringUp() {
    testCase("SSD1680 over the queued master: bring-up and the first refresh");
    fresh();
    Panel::restart();
    check(runUntil([] { return Panel::ready() || Panel::failed(); }, 5s), "settled");
    check(Panel::ready(), "ready");
    checkEq(Panel::refreshes(), 1U, "one refresh");
    check(!model.commands.empty() && model.commands.front().cmd == Cmd::SwReset, "SW reset first");
    check(model.has(Cmd::MasterActivation), "the refresh activated");
    checkEq(model.orphanData, 0U, "no data byte before its command");
    std::size_t ram = 0;
    for(auto const& c : model.commands) {
        if(c.cmd == Cmd::WriteRamBw) { ram += c.params.size(); }
    }
    checkEq(ram, 128U / 8U * 296U, "the whole frame into the RAM");
    bool selected = true;
    bool mode     = true;
    for(auto const& f : Bus::frames) {
        selected = selected && f.selected;
        mode     = mode && f.setup.mode == 0 && f.setup.hz <= 20'000'000U;
    }
    check(selected, "every frame under chip select");
    check(mode, "mode 0, at most 20 MHz");
    check(Pins::level[Spi::Cs::id], "chip select released at the end");
}

void failedTransfer() {
    testCase("SSD1680: a transfer the master fails ends in failed(), restart() brings it back");
    fresh();
    Panel::restart();
    Bus::stall = true;   // the DMA never completes: the master's timeout fails the request
    check(runUntil([] { return Panel::failed(); }, 1s), "failed, not stuck");
    Bus::stall = false;
    Bus::reset();
    Bus::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
    Bus::exchange     = [](std::uint8_t m) { return model.exchange(m); };
    model             = PanelModel{};
    Panel::restart();
    check(runUntil([] { return Panel::ready() || Panel::failed(); }, 5s) && Panel::ready(),
          "ready after restart()");
}

void abandonedSend() {
    testCase(
      "SSD1680: a send abandoned by restart() completing later is not the next one's verdict");
    fresh();
    Panel::restart();
    Bus::stall = true;
    // until a frame is out on the wire, completing nothing
    for(int i = 0; i < 100 && !Bus::pending; ++i) {
        FakeClock::advance(1ms);
        Panel::handler();
    }
    check(Bus::pending, "a send is out");
    Panel::restart();   // -> Stream::restart(): that send is nobody's any more
    checkEq(static_cast<int>(Stream::operationState()),
            static_cast<int>(Stream::OperationState::idle),
            "the stream is idle after the restart");
    Bus::stall = false;
    Bus::tick();   // the abandoned send completes now
    checkEq(static_cast<int>(Stream::operationState()),
            static_cast<int>(Stream::OperationState::idle),
            "its completion changed nothing");
    model = PanelModel{};
    check(runUntil([] { return Panel::ready() || Panel::failed(); }, 5s) && Panel::ready(),
          "and the new bring-up goes through");
}

}   // namespace

int main() {
    bringUp();
    failedTransfer();
    abandonedSend();
    return finish();
}
