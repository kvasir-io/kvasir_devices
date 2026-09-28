/// St7789::Panel over Display::SpiDcsBus over the queued SPI master, against a panel model that sees
/// every byte with its D/C level; also on a narrow master (the SAM SERCOM) and the failure paths.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "../spi/FakeQueuedSpi.hpp"
#include "../spi/FakeSpi.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <kvasir/Devices/Display/SpiDcsBus.hpp>
#include <kvasir/Devices/Display/St7789.hpp>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
namespace D = Kvasir::Display;

namespace {

using Dc = Spi::Pin<4>;

/// The panel: a command is the byte sent with D/C low; its parameters follow with D/C high.
struct PanelModel {
    struct Command {
        std::uint8_t              cmd{};
        std::vector<std::uint8_t> params{};
    };

    std::vector<Command> commands{};
    std::uint32_t        seenRises{};
    std::size_t          dataWithoutCommand{};
    /// The master stalls once this command byte has gone out: fails its data, nothing before it.
    std::uint8_t stallAfter{};
    bool*        stallFlag{};

    std::uint8_t exchange(std::uint8_t mosi) {
        if(Pins::rises[Spi::Cs::id] != seenRises) { seenRises = Pins::rises[Spi::Cs::id]; }
        if(!Pins::level[Dc::id]) {
            commands.push_back({mosi, {}});
            if(stallFlag != nullptr && mosi == stallAfter) { *stallFlag = true; }
        } else if(commands.empty()) {
            ++dataWithoutCommand;
        } else {
            commands.back().params.push_back(mosi);
        }
        return 0x00;
    }

    std::vector<std::uint8_t> order() const {
        std::vector<std::uint8_t> o{};
        for(auto const& c : commands) { o.push_back(c.cmd); }
        return o;
    }
};

PanelModel model{};

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;

/// The SAM SERCOM's limits over the same fake: 8-bit frames only, a small DMA count.
struct NarrowTag {};

using NarrowBase = QueuedSpi::Bus<NarrowTag>;

struct NarrowBus {
    struct Hw : NarrowBase::Hw {
        static constexpr bool          SupportsWide = false;
        static constexpr std::uint32_t MaxFrames    = 300;
    };

    using Core    = Kvasir::SPI::QueueCore<Hw, FakeClock, 8, 16>;
    using Request = Core::RequestT;
    using Setup   = QueuedSpi::Setup;

    static constexpr Setup setup(Kvasir::SPI::ClockMode mode,
                                 Kvasir::Units::Hertz   maxClock) {
        return NarrowBase::setup(mode, maxClock);
    }

    static bool submit(Request const& r) { return Core::submit(r); }

    static void releaseHold(Kvasir::SPI::Lines const& l) { Core::releaseHold(l); }

    static void tick() {
        if(NarrowBase::pending && !NarrowBase::stall) {
            NarrowBase::pending     = false;
            bool const ovr          = NarrowBase::overrunNext;
            NarrowBase::overrunNext = false;
            Core::complete(NarrowBase::pendingGen, ovr);
        }
    }

    static void handler() {
        if(NarrowBase::autoComplete) { tick(); }
        Core::handler();
    }

    static void reset() {
        Core::reset();
        NarrowBase::reset();
    }
};

/// The Pico Display Pack's 1.14" 240 x 135 in landscape; offsets 40 / 53 from Pimoroni's library.
struct PackConfig : D::DcsDefaults {
    static constexpr int          Width        = 240;
    static constexpr int          Height       = 135;
    static constexpr int          ColumnOffset = 40;
    static constexpr int          RowOffset    = 53;
    static constexpr bool         Invert       = true;
    static constexpr std::uint8_t Madctl = D::Dcs::Madctl::ColumnOrder | D::Dcs::Madctl::Exchange;
};

using Lcd   = D::SpiDcsBus<Bus, FakeClock, Spi::Cs, Dc>;
using Panel = Kvasir::St7789::Panel<FakeClock, Lcd, Kvasir::NoReset, PackConfig>;

using NarrowLcd   = D::SpiDcsBus<NarrowBus, FakeClock, Spi::Cs, Dc>;
using NarrowPanel = Kvasir::St7789::Panel<FakeClock, NarrowLcd, Kvasir::NoReset, PackConfig>;

static_assert(Lcd::Wide && !NarrowLcd::Wide);
static_assert(NarrowLcd::MaxChunk == 300);

/// Where a bus's knobs live: the fake's own, or the narrow wrapper's base's.
template<typename B>
struct KnobsOf {
    using type = B;
};

template<>
struct KnobsOf<NarrowBus> {
    using type = NarrowBase;
};

template<typename B>
void freshOn() {
    using K = typename KnobsOf<B>::type;
    B::reset();
    Pins::reset();
    FakeClock::set(1s);
    Log::reset();
    model           = PanelModel{};
    K::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
    K::exchange     = [](std::uint8_t m) { return model.exchange(m); };
    K::autoComplete = true;   // writeCommand() waits for its frame on the master's handler
}

void fresh() { freshOn<Bus>(); }

template<typename P,
         typename B,
         typename Pred>
bool runUntilOn(Pred                      pred,
                std::chrono::milliseconds limit) {
    auto const end = FakeClock::now() + limit;
    while(FakeClock::now() < end) {
        FakeClock::advance(1ms);
        P::handler();
        B::tick();
        if(pred()) { return true; }
    }
    return false;
}

template<typename Pred>
bool runUntil(Pred                      pred,
              std::chrono::milliseconds limit) {
    return runUntilOn<Panel, Bus>(pred, limit);
}

void bringUp() {
    testCase("ST7789 on 4-wire SPI: SWRESET, the pixel format, SLPOUT, a black screen, DISPON");
    fresh();
    Panel::restart();
    check(runUntil([] { return Panel::ready(); }, 2s), "ready");
    using C                           = D::Dcs::Cmd;
    auto const                      o = model.order();
    std::vector<std::uint8_t> const head{C::Swreset,
                                         C::Colmod,
                                         C::Madctl,
                                         C::Invon,
                                         C::Slpout,
                                         C::Caset,
                                         C::Raset,
                                         C::Ramwr,
                                         C::Dispon};
    check(o == head, "the bring-up in order");
    checkEq(model.commands[1].params.size(), 1U, "COLMOD one parameter");
    checkEq(model.commands[1].params[0], 0x55, "COLMOD 55h: 16 bits a pixel");
    auto const& caset = model.commands[5].params;
    // columns 40 .. 40 + 239 = 279 = 0117h
    check(caset == std::vector<std::uint8_t>{0x00, 40, 0x01, 0x17},
          "CASET: the module's column offset added");
    auto const& ram = model.commands[7].params;
    checkEq(ram.size(), 240U * 135U * 2U, "the whole screen filled");
    bool black = true;
    for(auto b : ram) { black = black && b == 0; }
    check(black, "black");
    checkEq(model.dataWithoutCommand, 0U, "no data byte before its command");
    bool mode = true;
    for(auto const& f : Bus::frames) {
        mode = mode && f.setup.mode == 0 && f.setup.hz <= 15'000'000U;
    }
    check(mode, "mode 0, at most the ST7789V's 15 MHz");
}

void blitAndFill() {
    testCase("ST7789: a blit into its window, a fill as 16-bit repeat frames");
    fresh();
    Panel::restart();
    check(runUntil([] { return Panel::ready(); }, 2s), "ready again");
    model.commands.clear();
    std::array<D::Rgb565, 4> px{
      D::Rgb565{std::byte{0xF8}, std::byte{0x00}},
      D::Rgb565{std::byte{0x07}, std::byte{0xE0}},
      D::Rgb565{std::byte{0x00}, std::byte{0x1F}},
      D::Rgb565{std::byte{0xFF}, std::byte{0xFF}}
    };
    Panel::blit(D::Rect{10, 20, 2, 2}, std::span<D::Rgb565 const>{px});
    runUntil([] { return Panel::ready(); }, 100ms);
    auto const o = model.order();
    check(o.size() == 3 && o[2] == D::Dcs::Cmd::Ramwr, "CASET, RASET, RAMWR");
    auto const& ram = model.commands[2].params;
    check(ram == std::vector<std::uint8_t>{0xF8, 0x00, 0x07, 0xE0, 0x00, 0x1F, 0xFF, 0xFF},
          "the pixels, high byte first");

    model.commands.clear();
    auto const frames = Bus::frames.size();
    Panel::fill(D::Rect{0, 0, 240, 135}, D::Rgb565{std::byte{0x12}, std::byte{0x34}});
    runUntil([] { return Panel::ready(); }, 100ms);
    check(!model.commands.empty(), "the fill sent something");
    if(model.commands.empty()) { return; }
    auto const& f = model.commands.back().params;
    checkEq(f.size(), 240U * 135U * 2U, "every pixel");
    check(f[0] == 0x12 && f[1] == 0x34 && f[f.size() - 2] == 0x12 && f.back() == 0x34,
          "0x1234, high byte first");
    bool wide = false;
    for(std::size_t i = frames; i < Bus::frames.size(); ++i) { wide = wide || Bus::frames[i].wide; }
    check(wide, "sent as 16-bit frames of one repeated pixel");
}

void narrowFill() {
    testCase(
      "ST7789 on a master without 16-bit frames: the fill from a buffer, in chunks under one CS");
    freshOn<NarrowBus>();
    NarrowPanel::restart();
    check(runUntilOn<NarrowPanel, NarrowBus>([] { return NarrowPanel::ready(); }, 2s), "ready");
    model.commands.clear();
    auto const frames = NarrowBase::frames.size();
    auto const rises  = Pins::rises[Spi::Cs::id];
    NarrowPanel::fill(D::Rect{0, 0, 240, 135}, D::Rgb565{std::byte{0x12}, std::byte{0x34}});
    check(runUntilOn<NarrowPanel, NarrowBus>([] { return NarrowPanel::ready(); }, 1s),
          "the fill completes");
    check(
      model.order()
        == std::vector<std::uint8_t>{D::Dcs::Cmd::Caset, D::Dcs::Cmd::Raset, D::Dcs::Cmd::Ramwr},
      "CASET, RASET, one RAMWR");
    auto const& f = model.commands.back().params;
    checkEq(f.size(), 240U * 135U * 2U, "every pixel");
    bool pairs = true;
    for(std::size_t i = 0; i + 1 < f.size(); i += 2) {
        pairs = pairs && f[i] == 0x12 && f[i + 1] == 0x34;
    }
    check(pairs, "0x1234 throughout, high byte first");
    bool wide = false;
    for(std::size_t i = frames; i < NarrowBase::frames.size(); ++i) {
        wide = wide || NarrowBase::frames[i].wide;
    }
    check(!wide, "no 16-bit frame");
    // 32 400 pixels in chunks of the fill buffer (128): 254 data transfers behind the command
    checkEq(NarrowBase::frames.size() - frames,
            2U + 2U + 1U + 254U,
            "CASET, RASET (command + data each), RAMWR, 254 chunks");
    checkEq(Pins::rises[Spi::Cs::id] - rises,
            3U,
            "three chip-select cycles: CASET, RASET, the stream");
    checkEq(model.dataWithoutCommand, 0U, "no chunk outside its RAMWR");
}

void chunkedStream() {
    testCase(
      "ST7789 on a small DMA count: a blit in chunks under one chip select, none from the loop");
    freshOn<NarrowBus>();
    NarrowPanel::restart();
    check(runUntilOn<NarrowPanel, NarrowBus>([] { return NarrowPanel::ready(); }, 2s), "ready");
    model.commands.clear();
    // 20 x 20 pixels = 800 bytes: chunks of 300, 300, 200
    static std::array<D::Rgb565, 400> px{};
    for(std::size_t i = 0; i < px.size(); ++i) {
        px[i] = D::Rgb565{std::byte(i >> 8U), std::byte(i & 0xFFU)};
    }
    auto const frames = NarrowBase::frames.size();
    auto const rises  = Pins::rises[Spi::Cs::id];
    NarrowPanel::blit(D::Rect{0, 0, 20, 20}, std::span<D::Rgb565 const>{px});
    check(runUntilOn<NarrowPanel, NarrowBus>([] { return NarrowPanel::ready(); }, 100ms),
          "the blit completes");
    auto const& ram = model.commands.back().params;
    checkEq(ram.size(), 800U, "every byte");
    bool intact = true;
    for(std::size_t i = 0; i < px.size(); ++i) {
        intact = intact && ram[2 * i] == std::to_integer<std::uint8_t>(px[i].hi)
              && ram[2 * i + 1] == std::to_integer<std::uint8_t>(px[i].lo);
    }
    check(intact, "in order");
    checkEq(NarrowBase::frames.size() - frames, 2U + 2U + 1U + 3U, "three chunks behind the RAMWR");
    checkEq(Pins::rises[Spi::Cs::id] - rises, 3U, "one chip-select cycle for the whole stream");
    checkEq(model.dataWithoutCommand, 0U, "no chunk outside its RAMWR");
}

void refusedStream() {
    testCase("a stream the queue has no room for is failed at once, chip select untouched");
    fresh();
    Bus::autoComplete = false;
    // one request on the wire and the queue full behind it, none of them completing
    static std::array<std::byte, 4> junk{};
    int                             accepted = 0;
    for(int i = 0; i < 12; ++i) {
        if(Bus::submit(Bus::Request{.tx = junk})) { ++accepted; }
    }
    checkEq(accepted, 9, "one on the wire, eight queued");
    static std::array<D::Rgb565, 4> px{};
    auto const                      rises = Pins::rises[Spi::Cs::id];
    Lcd::writePixels(std::as_bytes(std::span<D::Rgb565 const>{px}));
    Lcd::handler();
    check(Lcd::failed(), "failed");
    check(Lcd::idle(), "and idle again");
    check(Pins::level[Spi::Cs::id] && Pins::rises[Spi::Cs::id] == rises,
          "chip select never dropped");
    Lcd::clearError();
    Bus::reset();
}

void failedStream() {
    testCase("a pixel transfer the master fails: the bus reports it, the panel brings itself back");
    fresh();
    Panel::restart();
    check(runUntil([] { return Panel::ready(); }, 2s), "ready");
    // the master stalls once RAMWR has gone out: the pixels behind it never complete
    model.stallAfter = D::Dcs::Cmd::Ramwr;
    model.stallFlag  = &Bus::stall;
    static std::array<D::Rgb565, 16> px{};
    Panel::blit(D::Rect{0, 0, 4, 4}, std::span<D::Rgb565 const>{px});
    check(runUntil([] { return Panel::failed(); }, 100ms),
          "the panel failed within the master's timeout");
    check(Lcd::failed(), "the bus says so too");
    check(Pins::level[Spi::Cs::id], "chip select released by the failure");
    model.stallFlag     = nullptr;
    Bus::stall          = false;
    Bus::pending        = false;
    auto const failures = Panel::failures();
    check(runUntil([] { return Panel::ready(); }, 3s), "up again after RetryAfterMs");
    checkEq(Panel::failures(), failures, "the failure was counted once, before");
    check(!Lcd::failed(), "the bus's error cleared by the retry");
}

}   // namespace

int main() {
    bringUp();
    blitAndFill();
    narrowFill();
    chunkedStream();
    refusedStream();
    failedStream();
    return finish();
}
