/// LTC2401/LTC2402: the decode of every status combination of LTC2401_LTC2402.md Tables 1 and 2, and
/// the Reader over the queued master against a model of the part - EOC on SDO while CS is low, a
/// 32-bit result clocked out on demand, SDO high again (a new conversion) after the 32nd bit.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "FakeQueuedSpi.hpp"
#include "FakeSpi.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/SPI/chips/Ltc2400.hpp>
#include <support/FakeClock.hpp>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
namespace L = Kvasir::SPI::Ltc2400;

namespace {

// The decode, at compile time

using Kind = L::Decoded::Kind;

/// The word for a row of Table 2: EOC 0, SIG, EXR, a 24-bit result and the sub-LSBs.
constexpr std::uint32_t word(bool          sig,
                             bool          exr,
                             std::uint32_t r24,
                             std::uint32_t sub = 0) {
    return (sig ? L::SigBit : 0U) | (exr ? L::ExrBit : 0U) | (r24 << 4U) | sub;
}

constexpr bool isOk(L::Decoded const& d,
                    std::int32_t      counts,
                    L::Range          range,
                    bool              clamped = false) {
    return d.kind == Kind::ok && d.sample.counts == counts && d.sample.range == range
        && d.sample.clamped == clamped;
}

constexpr std::int32_t FS = L::FullScaleCounts;   // 2^28: VREF in sub-LSB counts

// Table 2, row by row (SIG, EXR, bits 27..4):
// VIN > 9/8 VREF and 9/8 VREF: 1 1 0001 1111...  -> clamped at 9/8 VREF
static_assert(isOk(L::decode(word(true,
                                  true,
                                  0x1F'FFFF,
                                  0xF)),
                   FS + 0x1FF'FFFF,
                   L::Range::aboveVref,
                   true));
// VREF + 1 LSB: 1 1 0000...: 2^24 LSBs
static_assert(isOk(L::decode(word(true,
                                  true,
                                  0)),
                   FS,
                   L::Range::aboveVref));
static_assert(L::decode(word(true,
                             true,
                             0))
                .sample.code24()
              == (1 << 24));
// just below the clamp
static_assert(isOk(L::decode(word(true,
                                  true,
                                  0x1F'FFFE)),
                   FS + 0x1FF'FFE0,
                   L::Range::aboveVref));
// VREF: 1 0 1111...
static_assert(isOk(L::decode(word(true,
                                  false,
                                  0xFF'FFFF,
                                  0xF)),
                   FS - 1,
                   L::Range::normal));
static_assert(L::decode(word(true,
                             false,
                             0xFF'FFFF))
                .sample.code24()
              == 0xFF'FFFF);
// 3/4, 1/2, 1/4 VREF (+ 1 LSB)
static_assert(isOk(L::decode(word(true,
                                  false,
                                  0xC0'0000)),
                   3 * (FS / 4),
                   L::Range::normal));
static_assert(isOk(L::decode(word(true,
                                  false,
                                  0xBF'FFFF)),
                   3 * (FS / 4) - 16,
                   L::Range::normal));
static_assert(isOk(L::decode(word(true,
                                  false,
                                  0x80'0000)),
                   FS / 2,
                   L::Range::normal));
static_assert(isOk(L::decode(word(true,
                                  false,
                                  0x40'0000)),
                   FS / 4,
                   L::Range::normal));
// 0+: SIG 1, EXR 0, all zero; the sub-LSBs count
static_assert(isOk(L::decode(word(true,
                                  false,
                                  0)),
                   0,
                   L::Range::normal));
static_assert(isOk(L::decode(word(true,
                                  false,
                                  0,
                                  0x3)),
                   3,
                   L::Range::normal));
// 0-: SIG 0, EXR 0, all zero (the sub-LSBs are X there and ignored)
static_assert(isOk(L::decode(word(false,
                                  false,
                                  0)),
                   0,
                   L::Range::normal));
static_assert(isOk(L::decode(word(false,
                                  false,
                                  0,
                                  0x7)),
                   0,
                   L::Range::normal));
// -1 LSB: 0 1 1111...
static_assert(isOk(L::decode(word(false,
                                  true,
                                  0xFF'FFFF)),
                   -16,
                   L::Range::belowZero));
static_assert(L::decode(word(false,
                             true,
                             0xFF'FFFF))
                .sample.code24()
              == -1);
static_assert(isOk(L::decode(word(false,
                                  true,
                                  0xFF'FFFF,
                                  0xF)),
                   -1,
                   L::Range::belowZero));
// -1/8 VREF and below: 0 1 1110 0000... -> clamped
static_assert(isOk(L::decode(word(false,
                                  true,
                                  0xE0'0000)),
                   -(FS / 8),
                   L::Range::belowZero,
                   true));
static_assert(L::decode(word(false,
                             true,
                             0xE0'0000))
                .sample.code24()
              == -(1 << 21));
static_assert(isOk(L::decode(word(false,
                                  true,
                                  0xE0'0001)),
                   -(FS / 8) + 16,
                   L::Range::belowZero));

// Not ready: EOC set, whatever follows (a MISO floating high reads all ones)
static_assert(L::decode(0xFFFF'FFFFU).kind == Kind::notReady);
static_assert(L::decode(L::EocBit
                        | word(true,
                               false,
                               0x12'3456))
                .kind
              == Kind::notReady);
// What the part never sends: past either clamp, a result under SIG/EXR 0/0, bit 30 on an LTC2401
static_assert(L::decode(word(true,
                             true,
                             0x20'0000))
                .kind
              == Kind::invalid);
static_assert(L::decode(word(false,
                             true,
                             0xDF'FFFF))
                .kind
              == Kind::invalid);
static_assert(L::decode(word(false,
                             false,
                             0x00'0001))
                .kind
              == Kind::invalid);
static_assert(L::decode(L::Bit30
                        | word(true,
                               false,
                               0x12'3456))
                .kind
              == Kind::invalid);
// ...which is the channel on an LTC2402
static_assert(L::decode(L::Bit30
                          | word(true,
                                 false,
                                 0x12'3456),
                        L::Part::ltc2402)
                .sample.channel
              == 1);
static_assert(L::decode(word(true,
                             false,
                             0x12'3456),
                        L::Part::ltc2402)
                .sample.channel
              == 0);
// A MISO floating low reads as a valid 0- code: only SDO after the frame tells (the Reader's job)
static_assert(isOk(L::decode(0x0000'0000U),
                   0,
                   L::Range::normal));

// The word is offset binary but for 0-: bits 29..0 minus 2^29
static_assert([] {
    for(std::uint32_t r : {0x00'0000U, 0x00'0001U, 0x7F'FFFFU, 0xFF'FFFFU}) {
        auto const w = word(true, false, r, 0x5);
        if(L::decode(w).sample.counts != static_cast<std::int32_t>(w & 0x3FFF'FFFFU) - (1 << 29)) {
            return false;
        }
    }
    return true;
}());

// Volts: 2^28 counts are VREF
static_assert(L::decode(word(true,
                             false,
                             0x80'0000))
                .sample.voltage(Kvasir::Units::microVolt(2'500'000))
              == Kvasir::Units::microVolt(1'250'000));
static_assert(L::decode(word(false,
                             true,
                             0xE0'0000))
                .sample.voltage(Kvasir::Units::microVolt(4'096'000))
              == Kvasir::Units::microVolt(-512'000));

static_assert(L::wordOf(std::array{std::byte{0x2F},
                                   std::byte{0xFF},
                                   std::byte{0xFF},
                                   std::byte{0xF0}})
              == 0x2FFF'FFF0U);

// The data sheet's numbers (md:185, :188)
static_assert(L::conversionTime(L::Rejection::hz50).max == 163'440us);
static_assert(L::conversionTime(L::Rejection::hz60).typical == 133'530us);
static_assert(L::MaxClock == Kvasir::Units::hertz(2'000'000));

// The Reader against a model of the part

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;
using Cs  = Spi::Cs;
using Sdo = Spi::Drdy;

/// EOC on SDO while CS is low; converting for `conversion` after the 32nd bit of a frame. A CS
/// rising edge in the middle of a frame aborts it and starts a conversion (md:574).
struct Part {
    std::uint32_t             result{word(true, false, 0x80'0000)};
    bool                      present{true};
    FakeClock::time_point     doneAt{};
    std::chrono::microseconds conversion{160'230};
    int                       bits{};   ///< of the frame being shifted out, 0 = not in data output
    int                       frames{};
    int                       sckWhileConverting{};
    std::uint32_t             shifting{};
    std::uint32_t             csRisesSeen{};

    [[nodiscard]] bool converting() const { return FakeClock::now() < doneAt; }

    /// What SDO shows: Hi-Z with CS high (left as the floating level), EOC with CS low.
    void drive() {
        if(!present) { return; }
        if(Pins::rises[Cs::id] != csRisesSeen) {
            csRisesSeen = Pins::rises[Cs::id];
            if(bits != 0) {   // aborted data output: a new conversion
                bits   = 0;
                doneAt = FakeClock::now() + conversion;
            }
        }
        if(!Pins::level[Cs::id]) { Pins::level[Sdo::id] = bits == 0 && converting(); }
    }

    std::uint8_t exchange(std::uint8_t) {
        drive();
        if(converting()) {
            ++sckWhileConverting;
            return 0xFF;
        }
        if(bits == 0) { shifting = result; }
        auto const out = static_cast<std::uint8_t>(shifting >> 24U);
        shifting <<= 8U;
        bits += 8;
        if(bits == 32) {   // the 32nd falling edge: SDO high, a new conversion
            bits   = 0;
            doneAt = FakeClock::now() + conversion;
            ++frames;
            Pins::level[Sdo::id] = true;
        }
        return out;
    }
};

Part* part{};

std::uint8_t onByte(std::uint8_t mosi) { return part->exchange(mosi); }

void fresh(Part&        p,
           std::uint8_t floating = 0xFF) {
    Bus::reset();
    Pins::reset();
    FakeClock::set(1s);
    Log::reset();
    part              = &p;
    p.doneAt          = FakeClock::now() + 100ms;   // a conversion running at start
    Bus::floating     = floating;
    Bus::partSelected = [] { return part->present && !Pins::level[Cs::id]; };
    if(p.present) { Bus::exchange = onByte; }
    Pins::level[Sdo::id] = floating != 0;
}

template<typename R>
void run(R&                        r,
         Part&                     p,
         std::chrono::milliseconds span) {
    auto const until = FakeClock::now() + span;
    while(FakeClock::now() < until) {
        FakeClock::advance(1ms);
        p.drive();
        Bus::handler();
        r.handler();
        Bus::tick();
    }
}

void reads() {
    testCase("LTC2401: reads a result when SDO goes low, none while it converts");
    Part p{};
    fresh(p);
    L::Reader<Bus, FakeClock, Cs, Sdo> adc{};
    check(Pins::level[Cs::id], "CS high at construction");
    run(adc, p, 120ms);
    checkEq(adc.samples(), 1U, "the first conversion read");
    check(!Pins::level[Cs::id], "CS held low between frames");
    checkEq(adc.latest().counts, FS / 2, "1/2 VREF");
    check(adc.link() == Kvasir::Link::answering, "answering");
    p.result = word(false, true, 0xFF'FFFF);
    run(adc, p, 170ms);
    checkEq(adc.samples(), 2U, "one conversion later, one more");
    checkEq(adc.latest().counts, -16, "-1 LSB");
    check(adc.latest().range == L::Range::belowZero, "below zero");
    run(adc, p, 2s);
    checkEq(p.sckWhileConverting, 0, "no SCK while it converts");
    checkEq(adc.samples(), static_cast<std::uint32_t>(p.frames), "every frame a sample");
    check(adc.samples() >= 13 && adc.samples() <= 15, "one sample per ~160 ms");
    checkEq(adc.timeouts() + adc.invalid() + adc.stuckLow() + adc.errors(), 0U, "no failures");
    for(auto const& f : Bus::frames) {
        check(f.miso.size() == 4 && f.setup.mode == 0 && f.setup.hz == 2'000'000,
              "4 bytes, mode 0, 2 MHz");
    }
    Bus::reset();
}

struct Ltc2402At60Hz : L::Defaults {
    static constexpr L::Part      part      = L::Part::ltc2402;
    static constexpr L::Rejection rejection = L::Rejection::hz60;
};

void ltc2402Channels() {
    testCase("LTC2402: the channel bit");
    Part p{};
    p.conversion = 133'530us;
    p.result     = L::Bit30 | word(true, false, 0x40'0000);
    fresh(p);
    L::Reader<Bus, FakeClock, Cs, Sdo, Ltc2402At60Hz> adc{};
    static_assert(decltype(adc)::Timeout == 2 * 136'200us);
    run(adc, p, 120ms);
    check(adc.samples() == 1 && adc.latest().channel == 1, "CH1");
    Bus::reset();
}

void invalidFrame() {
    testCase("LTC2401: a frame it cannot send is dropped and CS pulsed high");
    Part p{};
    p.result = L::Bit30 | word(true, false, 0x40'0000);   // bit 30 is always low on the LTC2401
    fresh(p);
    L::Reader<Bus, FakeClock, Cs, Sdo> adc{};
    run(adc, p, 120ms);
    checkEq(adc.samples(), 0U, "not taken");
    checkEq(adc.invalid(), 1U, "counted");
    checkEq(Pins::rises[Cs::id], 1U, "CS taken high once after the bad frame");
    p.result = word(true, false, 0x40'0000);
    run(adc, p, 400ms);
    check(adc.samples() >= 1 && adc.latest().counts == FS / 4, "the next good frame is read");
    check(adc.link() == Kvasir::Link::answering, "answering again");
    Bus::reset();
}

void outOfRange() {
    testCase("LTC2401: above VREF, at the clamp");
    Part p{};
    p.result = word(true, true, 0x1F'FFFF, 0xF);
    fresh(p);
    L::Reader<Bus, FakeClock, Cs, Sdo> adc{};
    run(adc, p, 120ms);
    check(adc.samples() == 1 && adc.latest().range == L::Range::aboveVref && adc.latest().clamped,
          "9/8 VREF, clamped");
    Bus::reset();
}

void noPart(std::uint8_t floating) {
    testCase(floating == 0 ? "LTC2401: no part, MISO low" : "LTC2401: no part, MISO high");
    Part p{};
    p.present = false;
    fresh(p, floating);
    L::Reader<Bus, FakeClock, Cs, Sdo> adc{};
    run(adc, p, 3s);
    checkEq(adc.samples(), 0U, "no sample");
    check(adc.link() == Kvasir::Link::absent, "absent");
    if(floating == 0) {
        check(adc.stuckLow() >= 3, "SDO low after every frame");
    } else {
        check(adc.timeouts() >= 3 && Bus::frames.empty(), "SDO never low: no frame, timeouts");
    }
    Bus::reset();
}

}   // namespace

int main() {
    reads();
    ltc2402Channels();
    invalidFrame();
    outOfRange();
    noPart(0x00);
    noPart(0xFF);
    return finish();
}
