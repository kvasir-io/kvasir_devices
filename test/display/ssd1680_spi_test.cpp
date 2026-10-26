/// The SSD1680 over Display::SpiDcsBus and the queued SPI master: bring-up, a command with its
/// parameters under one chip select, the frame in chunks with D/C high, a failed transfer ending in
/// failed() and the panel's own retry, and a send abandoned by restart().
#include <chrono>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/Display/SpiDcsBus.hpp>
#include <kvasir/Devices/Display/Ssd1680.hpp>
#include <spi/Harness.hpp>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
namespace D = Kvasir::Display;
using Cmd   = D::Ssd1680Cmd;

namespace {

using Dc = Spi::Pin<4>;

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
        std::vector<std::uint8_t> params;
    };

    std::vector<Command> commands;
    std::size_t          orphanData{};
    bool                 whileBusy{};

    std::uint8_t exchange(std::uint8_t mosi) {
        if(!Pins::level[Dc::id]) {
            if(Busy::busy()) { whileBusy = true; }
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

    [[nodiscard]] bool has(std::uint8_t c) const {
        for(auto const& x : commands) {
            if(x.cmd == c) { return true; }
        }
        return false;
    }
};

PanelModel model{};

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;

/// SSD1680: mode 0 (6.1.2 Table 6-2), SCL up to 20 MHz writing (12.1 Table 12-1).
struct EpdBusConfig : D::SpiDcsBusDefaults {
    static constexpr Kvasir::Units::Hertz writeClock = Kvasir::Units::hertz(20'000'000);
};

struct EpdConfig : D::Ssd1680Defaults {
    static constexpr std::chrono::milliseconds busyTimeout{2'000};
    static constexpr std::chrono::milliseconds retryAfter{500};
};

using EpdBus = D::SpiDcsBus<Bus, FakeClock, Spi::Cs, Dc, EpdBusConfig>;
using Panel  = D::Ssd1680<FakeClock, EpdBus, Reset, Busy, EpdConfig>;

static_assert(D::DataBus<EpdBus>);

void fresh() {
    Spi::Harness::fresh<Bus>([](std::uint8_t m) { return model.exchange(m); });
    model       = PanelModel{};
    Busy::until = {};
}

/// 1 ms turns, the clock first; the panel runs the master's turn itself (through the bus's).
constexpr Spi::Loop loop{.step = 1ms, .order = Spi::ClockOrder::first, .deviceRunsMaster = true};

template<typename Pred>
bool runUntil(Pred                      pred,
              std::chrono::milliseconds limit) {
    auto panel = [] { Panel::turn(); };
    return Spi::Harness::runUntil<Bus>(panel, pred, limit, loop);
}

void bringUp() {
    testCase("SSD1680 over SpiDcsBus: bring-up and the first refresh");
    fresh();
    Panel::restart();
    auto const rises0 = Pins::rises[Spi::Cs::id];
    check(runUntil([] { return Panel::ready() || Panel::failed(); }, 5s), "settled");
    check(Panel::ready(), "ready");
    checkEq(Panel::refreshes(), 1U, "one refresh");
    check(!model.commands.empty() && model.commands.front().cmd == Cmd::SwReset, "SW reset first");
    check(model.has(Cmd::MasterActivation), "the refresh activated");
    check(!model.whileBusy, "no command while BUSY is high");
    checkEq(model.orphanData, 0U, "no data byte before its command");
    std::size_t ram = 0;
    for(auto const& c : model.commands) {
        if(c.cmd == Cmd::WriteRamBw) { ram += c.params.size(); }
    }
    checkEq(ram, 128U / 8U * 296U, "the whole frame into the RAM");
    // a command and its parameters are one chip-select cycle, each RAM chunk another
    auto const chunks = (296U + 15U) / 16U;
    checkEq(Pins::rises[Spi::Cs::id] - rises0,
            static_cast<std::uint32_t>(model.commands.size() + chunks),
            "one chip-select cycle per command, one per RAM chunk");
    bool selected = true;
    bool mode     = true;
    for(auto const& f : Bus::frames) {
        selected = selected && f.selected;
        mode     = mode && f.setup.mode == 0 && f.setup.hz <= 20'000'000U;
    }
    check(selected, "every frame under chip select");
    check(mode, "mode 0, at most 20 MHz");
    check(Pins::level[Spi::Cs::id], "chip select released at the end");
    check(EpdBus::idle() && !EpdBus::failed(), "the bus idle, no failure");
}

void failedTransfer() {
    testCase("SSD1680: a transfer the master fails ends in failed(), the retry brings it back");
    fresh();
    Panel::restart();
    Bus::stall = true;   // the DMA never completes: the master's timeout fails the request
    check(runUntil([] { return Panel::failed(); }, 1s), "failed, not stuck");
    check(Panel::link() == Kvasir::Link::absent, "link: absent while failed");
    auto const failures = Panel::failures();
    Bus::stall          = false;
    model               = PanelModel{};
    check(runUntil([] { return Panel::ready(); }, 5s), "up again after retryAfter, by itself");
    checkEq(Panel::failures(), failures, "the failure counted once");
    check(!EpdBus::failed(), "the bus's error cleared by the retry");
    check(model.has(Cmd::SwReset) && model.has(Cmd::MasterActivation), "a whole bring-up again");
}

void abandonedSend() {
    testCase("SSD1680: a send abandoned by restart() failing later is not the next one's verdict");
    fresh();
    Panel::restart();
    Bus::stall = true;
    // until a frame is out on the wire, completing nothing
    for(int i = 0; i < 100 && !Bus::pending; ++i) {
        FakeClock::advance(1ms);
        Panel::turn();
    }
    check(Bus::pending, "a send is out");
    Panel::restart();   // -> EpdBus::clearError(): that send is nobody's any more
    check(!EpdBus::idle(), "the bus waits for its completion");
    Bus::stall       = false;
    Bus::overrunNext = true;
    Bus::tick();   // the abandoned send completes now, failed
    EpdBus::turn();
    check(!EpdBus::failed(), "its failure changed nothing");
    check(EpdBus::idle(), "and the bus is idle");
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
