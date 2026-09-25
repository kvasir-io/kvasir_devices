/// Seven-segment displays (SegmentDisplay.hpp): the layouts and frames at compile time, then a
/// display on each backend -- a PCA9956B and two port expanders on the fake bus, and pins
/// through a recording policy, driven statically and scanned.
#include "Harness.hpp"

#include <array>
#include <bitset>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/I2C/SegmentBackends.hpp>
#include <kvasir/Devices/I2C/chips/Mcp23017.hpp>
#include <kvasir/Devices/I2C/chips/Pca9956b.hpp>
#include <kvasir/Devices/I2C/chips/Pcf8574.hpp>
#include <kvasir/Devices/SegmentDisplay.hpp>
#include <kvasir/Devices/SegmentDisplay/Gpio.hpp>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;
namespace Sd = Kvasir::SegmentDisplay;

namespace {

using Leds    = Chips::Pca9956b<0x3F, Kvasir::Units::ohm(2200)>;
using LedsDev = Dev<Leds>;
using Pca     = Pca9956bSegments<LedsDev>;

// -- the PCA9956B board wiring: the digit patterns as LEDOUT bytes --
//
// {0xFC, 0xFC} is 0, {0xC0, 0x0C} is 1 and so on, the pair a digit's LEDOUT bytes, and the
// decimal point adds 3 to the second byte. If any of these stops holding, Pca9956bBoard is
// wrong about the wiring.

constexpr Pca::LedOut ledOut(Sd::Digits<3> const& d) {
    return Pca::ledOut(Sd::frame<Pca::Outputs>(Pca9956bBoard, d));
}

constexpr bool pairIs(Sd::Glyph    g,
                      std::uint8_t first,
                      std::uint8_t second) {
    return ledOut({Sd::Blank, Sd::Blank, g}) == Pca::LedOut{first, second, 0, 0, 0, 0};
}

static_assert(pairIs(Sd::HexDigits[0],
                     0xFC,
                     0xFC),
              "0 is LEDOUT 0xFC 0xFC");
static_assert(pairIs(Sd::HexDigits[1],
                     0xC0,
                     0x0C),
              "1 is LEDOUT 0xC0 0x0C");
static_assert(pairIs(Sd::HexDigits[2],
                     0xF3,
                     0xF0),
              "2 is LEDOUT 0xF3 0xF0");
static_assert(pairIs(Sd::HexDigits[3],
                     0xF3,
                     0x3C),
              "3 is LEDOUT 0xF3 0x3C");
static_assert(pairIs(Sd::HexDigits[4],
                     0xCF,
                     0x0C),
              "4 is LEDOUT 0xCF 0x0C");
static_assert(pairIs(Sd::HexDigits[5],
                     0x3F,
                     0x3C),
              "5 is LEDOUT 0x3F 0x3C");
static_assert(pairIs(Sd::HexDigits[6],
                     0x3F,
                     0xFC),
              "6 is LEDOUT 0x3F 0xFC");
static_assert(pairIs(Sd::HexDigits[7],
                     0xF0,
                     0x0C),
              "7 is LEDOUT 0xF0 0x0C");
static_assert(pairIs(Sd::HexDigits[8],
                     0xFF,
                     0xFC),
              "8 is LEDOUT 0xFF 0xFC, the point dark");
static_assert(pairIs(Sd::HexDigits[9],
                     0xFF,
                     0x3C),
              "9 is LEDOUT 0xFF 0x3C");
static_assert(pairIs(Sd::Blank,
                     0x00,
                     0x00),
              "a blank digit is dark");
static_assert(pairIs(Sd::Seg::Dp,
                     0x00,
                     0x03),
              "the point is channel 4, the low field of the digit's second byte: the +3");

// Whole numbers on the wire: the rightmost digit on LEDOUT0/1.
static_assert(ledOut(Sd::number<3>(7))
                == Pca::LedOut{0xF0,
                               0x0C,
                               0,
                               0,
                               0,
                               0},
              "one digit, on the rightmost pair, the leading zeros dark");
static_assert(ledOut(Sd::number<3>(0))
                == Pca::LedOut{0xFC,
                               0xFC,
                               0,
                               0,
                               0,
                               0},
              "0 is drawn");
static_assert(ledOut(Sd::number<3>(205))
                == Pca::LedOut{0x3F,
                               0x3C,
                               0xFC,
                               0xFC,
                               0xF3,
                               0xF0},
              "a zero inside the number is drawn");
static_assert(ledOut(Sd::number<3>(234,
                                   1))
                == Pca::LedOut{0xCF,
                               0x0C,
                               0xF3,
                               0x3F,
                               0xF3,
                               0xF0},
              "\"23.4\": the point on the tens digit");
static_assert(ledOut(Sd::number<3>(1000,
                                   0,
                                   Sd::Overflow::clamp))
                == Pca::LedOut{0xFF,
                               0x3C,
                               0xFF,
                               0x3C,
                               0xFF,
                               0x3C},
              "clamped, past three digits is 999");

// -- layouts --------------------------------------------------------------------------------

constexpr auto LeftFirst = Sd::perDigit<3>({
  .pattern = {2, 3, 5, 6, 7, 1, 0, 4},
  .stride  = 8,
  .first   = Sd::FirstDigit::left,
  .active  = Sd::Active::high,
});
static_assert(Sd::frame<24>(LeftFirst,
                            {Sd::HexDigits[1],
                             Sd::Blank,
                             Sd::Blank})
                == Sd::frame<24>(Pca9956bBoard,
                                 {Sd::Blank,
                                  Sd::Blank,
                                  Sd::HexDigits[1]}),
              "the other digit order puts the leftmost digit on the first eight lines");

/// One digit on a PCF8574, pin n segment n, the port sinking the segment current.
constexpr auto SinkDigit = Sd::perDigit<1>({
  .pattern = {0, 1, 2, 3, 4, 5, 6, 7},
  .first   = Sd::FirstDigit::left,
  .active  = Sd::Active::low,
});
static_assert(Sd::frame<8>(SinkDigit,
                           {Sd::HexDigits[1]})
                  .to_ulong()
                == 0xF9,
              "active low: b and c low, every other line high");

/// Two digits on an MCP23017's ports, a glass without decimal points.
constexpr auto NoPoints = Sd::perDigit<2>({
  .pattern = {0, 1, 2, 3, 4, 5, 6, Sd::NotWired},
  .stride  = 8,
  .first   = Sd::FirstDigit::left,
  .active  = Sd::Active::high,
});
static_assert(Sd::usedLines<16>(NoPoints).to_ulong() == 0x7F7F,
              "no line for the points");
static_assert(Sd::frame<16>(NoPoints,
                            {Sd::Seg::Dp,
                             Sd::Seg::Dp})
                .none(),
              "a point with no line lights nothing");

/// An irregular board, written out: a bar missing on the second digit, lines out of order.
constexpr Sd::StaticLayout<2> Irregular{
  .segment = {{{9, 8, 7, 6, 5, 4, 3, 2}, {0, 1, Sd::NotWired, 10, 11, 12, 13, 14}}},
  .active  = Sd::Active::high,
};
static_assert(Sd::frame<16>(Irregular,
                            {Sd::Seg::A,
                             Sd::Seg::A | Sd::Seg::C})
                  .to_ulong()
                == ((1UL << 9U) | (1UL << 0U)),
              "the table as written, c of the second digit nowhere");

/// Two digits scanned: segments on lines 0..7, digit selects on 8 and 9, pulled low to select.
constexpr Sd::MultiplexedLayout<2> Scanned{
  .segment       = {0, 1, 2, 3, 4, 5, 6, 7},
  .select        = {8, 9},
  .segmentActive = Sd::Active::high,
  .selectActive  = Sd::Active::low,
};
static_assert(Sd::scanFrame<10>(Scanned,
                                {Sd::HexDigits[1],
                                 Sd::HexDigits[2]},
                                1)
                  .to_ulong()
                == (Sd::HexDigits[2] | (1UL << 8U)),
              "digit 1 selected (line 9 low), digit 0 not (line 8 high)");
static_assert(Sd::scanFrame<10>(Scanned,
                                {Sd::HexDigits[8],
                                 Sd::HexDigits[8]},
                                2)
                  .to_ulong()
                == (3UL << 8U),
              "no digit: every select inactive, every segment dark");

struct FastScan {
    static constexpr std::chrono::microseconds DigitPeriod{500};
};

// -- helpers --------------------------------------------------------------------------------

template<typename Display,
         typename Device>
void run(Display&                  display,
         Device&                   device,
         std::chrono::milliseconds span) {
    auto const until = FakeClock::now() + std::chrono::duration_cast<FakeClock::duration>(span);
    while(FakeClock::now() < until) {
        display.update(FakeClock::now());
        turn(device);
    }
}

std::size_t countWrites(std::vector<std::uint8_t> const& bytes) {
    std::size_t n = 0;
    for(auto const& w : writes()) {
        if(w == bytes) { ++n; }
    }
    return n;
}

std::size_t writesTo(std::uint8_t reg) {
    std::size_t n = 0;
    for(auto const& w : writes()) {
        if(!w.empty() && w[0] == reg) { ++n; }
    }
    return n;
}

// -- PCA9956B -------------------------------------------------------------------------------

void pca9956b() {
    testCase("PCA9956B: LEDOUT, IREFALL, PWMALL, blink and CLRERR");
    fresh();
    FakeBus::respond = zeros;
    LedsDev                  leds{};
    Pca9956bDisplay<LedsDev> digits{leds, Kvasir::Units::milliAmp(5), std::uint8_t{200}, 100ms};
    static_assert(Pca9956bDisplay<LedsDev>::Count == 3);

    run(digits, leds, 1s);
    check(leds.answering(), "brought up");
    check(hasWrite({0x40, 49}), "IREFALL for 5 mA at 2.2 kOhm");
    check(hasWrite({0x3F, 200}), "PWMALL the brightness");
    check(hasWrite({0x82, 0, 0, 0, 0, 0, 0}), "LEDOUT dark, not the Initial 0xAA");
    check(hasWrite({0x81, 0x10}), "CLRERR after the bring-up");

    digits.setNumber(234, 1);
    run(digits, leds, 50ms);
    check(hasWrite({0x82, 0xCF, 0x0C, 0xF3, 0x3F, 0xF3, 0xF0}), "\"23.4\"");
    auto const ledOuts = writesTo(0x82);
    run(digits, leds, 50ms);
    checkEq(writesTo(0x82), ledOuts, "an unchanged display sends nothing");

    digits.blink(Sd::Blink::on);
    run(digits, leds, 50ms);
    check(hasWrite({0x3F, 0}), "blinks dark through PWMALL");
    checkEq(writesTo(0x82), ledOuts, "and leaves LEDOUT alone");
    check(digits.blinking(), "blinking");

    digits.brightness(std::uint8_t{80});
    digits.blink(Sd::Blink::off);
    run(digits, leds, 50ms);
    check(hasWrite({0x3F, 80}), "lit again at the new brightness");
    checkEq(digits.brightness(), std::uint8_t{80}, "brightness reads back");

    digits.current(Kvasir::Units::milliAmp(10));
    run(digits, leds, 50ms);
    check(hasWrite({0x40, 98}), "IREFALL for 10 mA");
    if(failures != 0) { dump(); }
}

// -- expanders ------------------------------------------------------------------------------

void pcf8574() {
    testCase("PCF8574: one digit, active low");
    fresh();
    FakeBus::respond = zeros;
    using PortDev    = Dev<Chips::Pcf8574>;
    PortDev                             port{};
    ExpanderDisplay<PortDev, SinkDigit> digit{port};
    static_assert(ExpanderDisplay<PortDev, SinkDigit>::Outputs == 8);

    run(digit, port, 200ms);
    check(port.answering(), "brought up");
    digit.setDigit(0, Sd::HexDigits[1]);
    run(digit, port, 50ms);
    check(hasWrite({0xF9}), "b and c pulled low");
    auto const n = FakeBus::log.size();
    auto const w = countWrites({0xF9});
    run(digit, port, 50ms);
    checkEq(countWrites({0xF9}), w, "an unchanged digit sends nothing");
    check(FakeBus::log.size() >= n, "only the pin reads meanwhile");
    if(failures != 0) { dump(); }
}

void mcp23017() {
    testCase("MCP23017: two digits, no decimal points");
    fresh();
    FakeBus::respond = zeros;
    using PortDev    = Dev<Chips::Mcp23017>;
    PortDev                            port{};
    ExpanderDisplay<PortDev, NoPoints> digits{port};

    run(digits, port, 200ms);
    check(port.answering(), "brought up");
    check(hasWrite({0x00, 0x80, 0x80}), "IODIR: the used lines outputs, the point pins inputs");
    digits.setNumber(12, 1);
    run(digits, port, 50ms);
    check(hasWrite({0x14, 0x06, 0x5B}), "OLAT: \"1\" and \"2\", the point nowhere");
    if(failures != 0) { dump(); }
}

// -- pins -----------------------------------------------------------------------------------

struct PinLog {
    static inline std::vector<std::pair<std::size_t, Sd::Level>> writes{};
    static inline std::array<Sd::Level, 16>                      level{};
    static inline bool                                           initialised{};

    static void record(std::size_t line,
                       Sd::Level   l) {
        writes.emplace_back(line, l);
        level[line] = l;
    }

    static void reset() {
        writes.clear();
        level.fill(Sd::Level::low);
        initialised = false;
    }
};

struct EightPins {
    static constexpr std::size_t Count = 8;

    static void init() { PinLog::initialised = true; }

    static void write(std::size_t line,
                      Sd::Level   l) {
        PinLog::record(line, l);
    }
};

struct TenPins {
    static constexpr std::size_t Count = 10;

    static void write(std::size_t line,
                      Sd::Level   l) {
        PinLog::record(line, l);
    }
};

constexpr auto PinDigit = Sd::perDigit<1>({
  .pattern = {0, 1, 2, 3, 4, 5, 6, 7},
  .first   = Sd::FirstDigit::left,
  .active  = Sd::Active::high,
});

/// The segments on lines 0..7 as a glyph.
Sd::Glyph segmentsLit() {
    unsigned g = 0;
    for(std::size_t s = 0; s < 8; ++s) {
        if(PinLog::level[s] == Sd::Level::high) { g |= 1U << s; }
    }
    return static_cast<Sd::Glyph>(g);
}

void gpioStatic() {
    testCase("GPIO: one digit, static");
    fresh();
    PinLog::reset();
    Sd::GpioDisplay<EightPins, FakeClock, PinDigit> digit{};
    check(PinLog::initialised, "the pins set up");
    checkEq(PinLog::writes.size(), std::size_t{8}, "every line driven dark once");

    PinLog::writes.clear();
    digit.setDigit(0, Sd::HexDigits[1]);
    check(digit.update(FakeClock::now()), "a new frame");
    checkEq(PinLog::writes.size(), std::size_t{2}, "only b and c written");
    checkEq(segmentsLit(), Sd::HexDigits[1], "showing 1");
    PinLog::writes.clear();
    check(!digit.update(FakeClock::now()), "nothing new");
    check(PinLog::writes.empty(), "no pin touched");
}

void gpioScanned() {
    testCase("GPIO: two digits, multiplexed");
    fresh();
    PinLog::reset();
    using Display = Sd::GpioDisplay<TenPins, FakeClock, Scanned>;
    static_assert(Display::Multiplexed && Display::DigitPeriod == 2ms);
    static_assert(Sd::GpioDisplay<TenPins, FakeClock, Scanned, FastScan>::DigitPeriod == 500us);
    Display digits{};
    check(PinLog::level[8] == Sd::Level::high && PinLog::level[9] == Sd::Level::high,
          "no digit selected at first");

    digits.setDigits({Sd::HexDigits[1], Sd::HexDigits[2]});
    PinLog::writes.clear();
    bool ghost = false;
    for(int i = 0; i < 20; ++i) {
        digits.update(FakeClock::now());
        FakeClock::advance(500us);
    }
    // Replay what went out from the dark start: a select only ever goes active over its own
    // digit's segments, and never while the other is active.
    std::array<Sd::Level, 16> lv{};
    lv.fill(Sd::Level::low);
    lv[8]            = Sd::Level::high;
    lv[9]            = Sd::Level::high;
    std::size_t sel0 = 0;
    std::size_t sel1 = 0;
    for(auto const& [line, l] : PinLog::writes) {
        lv[line] = l;
        if(line >= 8 && l == Sd::Level::low) {
            unsigned g = 0;
            for(std::size_t s = 0; s < 8; ++s) {
                if(lv[s] == Sd::Level::high) { g |= 1U << s; }
            }
            std::size_t const pos = line - 8;
            if(g != Sd::HexDigits[pos + 1]) { ghost = true; }
            if(lv[pos == 0 ? 9 : 8] == Sd::Level::low) { ghost = true; }
            ++(pos == 0 ? sel0 : sel1);
        }
    }
    check(!ghost, "each digit selected over its own segments only");
    checkEq(sel0, std::size_t{3}, "digit 0 switched in every other 2 ms period");
    checkEq(sel1, std::size_t{2}, "and digit 1 between");

    digits.blink(Sd::Blink::on);
    FakeClock::advance(2ms);
    digits.update(FakeClock::now());
    checkEq(segmentsLit(), Sd::Blank, "blinks dark by a blank frame");
}

}   // namespace

int main() {
    pca9956b();
    pcf8574();
    mcp23017();
    gpioStatic();
    gpioScanned();
    return finish();
}
