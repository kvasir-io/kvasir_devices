/// The MAX31865: the Callendar-Van Dusen arithmetic, and the driver against a model of the part.
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
#include <kvasir/Devices/SPI/chips/Max31865.hpp>
#include <span>
#include <support/FakeClock.hpp>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;

namespace {

/// The part (datasheet table 1): in auto mode with VBIAS a conversion every 20 ms, D0 set at or
/// past a threshold, DRDY low until 0x01/0x02 is read; an over/undervoltage fault halts conversions
/// and sets D2 again right after a clear (:430, :534). `release` is not the data sheet's: a part whose
/// fault holds itself until one bit of the configuration is worked, for the driver's levers.
struct Part {
    enum class Release : std::uint8_t { outside, vbiasOff, faultCycle };

    std::array<std::uint8_t, 8> reg{};
    std::uint16_t               code{16384};   // R = R0: 0 degC
    bool                        overVoltage{};
    Release                     release{Release::outside};
    FakeClock::time_point       nextConversion{};

    void powerOn() {
        reg                        = {0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00};
        Pins::level[Spi::Drdy::id] = true;
        nextConversion             = FakeClock::now();
    }

    void tick() {
        if((reg[0] & 0xC0) != 0xC0 || FakeClock::now() < nextConversion) { return; }
        nextConversion = FakeClock::now() + 20ms;
        if(overVoltage) {
            reg[7] |= 0x04;
            return;
        }
        auto       word = static_cast<std::uint16_t>(code << 1U);
        auto const high = static_cast<std::uint16_t>(reg[3] << 8U | reg[4]);
        auto const low  = static_cast<std::uint16_t>(reg[5] << 8U | reg[6]);
        if(word >= high) { reg[7] |= 0x80; }
        if(word <= low) { reg[7] |= 0x40; }
        if(reg[7] != 0) { word |= 1U; }
        reg[1]                     = static_cast<std::uint8_t>(word >> 8U);
        reg[2]                     = static_cast<std::uint8_t>(word);
        Pins::level[Spi::Drdy::id] = false;
    }

    /// Register access (MAX31865.md:659): the address byte, then data, auto-incrementing under CS.
    std::uint32_t seenRises{};
    bool          haveAddress{};
    std::uint8_t  command{};
    std::size_t   pos{};

    std::uint8_t exchange(std::uint8_t mosi) {
        if(Pins::rises[Spi::Cs::id] != seenRises) {
            seenRises   = Pins::rises[Spi::Cs::id];
            haveAddress = false;
        }
        if(!haveAddress) {
            haveAddress = true;
            command     = mosi;
            pos         = command & 0x07U;
            return 0xFF;   // SDO stays high impedance during the address (:671)
        }
        std::uint8_t out = 0xFF;
        if((command & 0x80U) != 0) {
            write(pos, mosi);
        } else {
            out = reg[pos];
            if(pos == 1 || pos == 2) { Pins::level[Spi::Drdy::id] = true; }
        }
        pos = (pos + 1) % reg.size();
        return out;
    }

    void write(std::size_t  at,
               std::uint8_t v) {
        if(at == 0) {
            if(release == Release::vbiasOff && (v & 0x80U) == 0) { overVoltage = false; }
            if(release == Release::faultCycle && (v & 0xACU) == 0x84U) { overVoltage = false; }
            // fault status clear; D2 is back at once while the fault is there
            if((v & 0x02U) != 0) { reg[7] = overVoltage ? 0x04 : 0x00; }
            reg[0] = static_cast<std::uint8_t>(v & 0xD1U);   // the command bits self-clear
        } else if(at >= 3 && at <= 6) {
            reg[at] = v;
        }
    }
};

Part part{};

/// The old behaviour: a faulted conversion only cleared, registers never read back.
struct OldBehaviour : Kvasir::SPI::Max31865Defaults {
    static constexpr std::uint16_t RejectedRestart = 0;
    static constexpr auto          RegisterCheck   = std::chrono::seconds{0};
};

struct Tag {};

/// The driver before 2026-10-06: bring-ups only.
struct NoLevers : Kvasir::SPI::Max31865Defaults {
    static constexpr std::uint8_t PlainBringUps = 0;
};

using Lever = Kvasir::SPI::Max31865Lever;

using Bus      = QueuedSpi::Bus<Tag>;
using Rtd      = Kvasir::SPI::Max31865<Bus, FakeClock, Spi::Cs, Spi::Drdy>;
using PlainRtd = Kvasir::SPI::Max31865<Bus,
                                       FakeClock,
                                       Spi::Cs,
                                       Spi::Drdy,
                                       Kvasir::Units::ohm(500),
                                       Kvasir::Units::ohm(1000),
                                       NoLevers>;
using OldRtd   = Kvasir::SPI::Max31865<Bus,
                                       FakeClock,
                                       Spi::Cs,
                                       Spi::Drdy,
                                       Kvasir::Units::ohm(500),
                                       Kvasir::Units::ohm(1000),
                                       OldBehaviour>;

/// Declared right after a case's device: resets the bus while the device still lives.
struct BusScope {
    ~BusScope() { Bus::reset(); }
};

void fresh() {
    Bus::reset();
    FakeClock::set(1s);
    Pins::reset();
    Log::reset();
    part = {};
    part.powerOn();
    Bus::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
    Bus::exchange     = [](std::uint8_t m) { return part.exchange(m); };
}

template<typename D>
void run(D&                        d,
         std::chrono::milliseconds span,
         bool                      withPart = true) {
    auto const until = FakeClock::now() + span;
    while(FakeClock::now() < until) {
        FakeClock::advance(1ms);
        if(withPart) { part.tick(); }
        Bus::handler();
        d.handler();
        Bus::tick();
    }
}

/// Runs until the driver has a reading, at most `span`; how long it took.
template<typename D>
std::chrono::milliseconds untilReading(D&                        d,
                                       std::chrono::milliseconds span) {
    auto const from = FakeClock::now();
    while(!d.temperature() && FakeClock::now() - from < span) { run(d, 1ms); }
    return std::chrono::duration_cast<std::chrono::milliseconds>(FakeClock::now() - from);
}

/// Frames whose only byte is `command`: a register access's address phase.
std::size_t framesOf(std::uint8_t command) {
    std::size_t n = 0;
    for(auto const& f : Bus::frames) {
        n += (f.mosi.size() == 1 && f.mosi[0] == command) ? 1U : 0U;
    }
    return n;
}

/// After every case: CS released between frames, mode 3, at most 5 MHz.
void wire() {
    check(Pins::level[Spi::Cs::id] || Bus::pending, "chip select high between frames");
    bool right = true;
    for(auto const& f : Bus::frames) {
        right = right && f.setup.mode == 3 && f.setup.hz <= 5'000'000U;
    }
    check(right, "mode 3, at most 5 MHz");
}

void bringUp() {
    testCase("MAX31865: bring-up writes the thresholds and reads the registers back");
    fresh();
    part.reg[3] = 0x12;   // left over from before the MCU's reset
    Rtd        rtd{};
    BusScope   scope{};
    auto const took = untilReading(rtd, 2s);
    check(rtd.temperature().has_value(), "a reading after the bring-up");
    check(took < 700ms, "within the startup delay and a conversion");
    checkEq(*rtd.temperature(), 0, "0 degC");
    checkEq(part.reg[0], 0xC1, "VBIAS, auto, 50 Hz");
    checkEq(part.reg[3], 0xFF, "the high threshold back at its POR value");
    checkEq(rtd.bringUps(), 1, "one bring-up");
    auto const rtdReads     = framesOf(0x01);
    auto const statusReads  = framesOf(0x07);
    auto const configWrites = framesOf(0x80);
    run(rtd, 1s);
    check(rtd.samples() >= 45, "a conversion every 20 ms taken");
    check(framesOf(0x01) - rtdReads >= 45, "one RTD read per conversion");
    checkEq(framesOf(0x07) - statusReads, 0U, "the fault status not read for a clean conversion");
    checkEq(framesOf(0x80) - configWrites,
            0U,
            "nor the configuration written again: a clean conversion is one frame");
    wire();
}

void thresholdChanged() {
    testCase("MAX31865: a threshold changed under the driver (every conversion faulted)");
    fresh();
    Rtd      rtd{};
    BusScope scope{};
    untilReading(rtd, 2s);
    part.reg[3]            = 0x00;   // high threshold 0x00FF: 0 degC is past it
    part.reg[4]            = 0xFF;
    auto const statusReads = framesOf(0x07);
    run(rtd, 100ms);
    check(!rtd.temperature(), "no reading while the part flags every conversion");
    checkEq(rtd.fault(), Kvasir::SPI::Max31865Detail::FaultRtdHigh, "the fault status is kept");
    check(framesOf(0x07) > statusReads, "a flagged conversion reads the fault status");
    std::size_t clears = 0;
    for(auto const& f : Bus::frames) {
        clears += (f.mosi.size() == 1 && f.mosi[0] == 0xC3) ? 1U : 0U;
    }
    check(clears >= 1, "and writes the configuration with the fault status cleared");
    auto const took = untilReading(rtd, 5s);
    check(rtd.temperature().has_value(), "the reading is back");
    check(took < 2s, "within RejectedRestart conversions and a bring-up");
    checkEq(part.reg[3], 0xFF, "the threshold written again");
    checkEq(rtd.bringUps(), 2, "a second bring-up");
    wire();

    testCase(
      "MAX31865: ... and with the knobs of the driver before 2026-09-25 it stays without one");
    fresh();
    OldRtd   old{};
    BusScope oldScope{};
    untilReading(old, 2s);
    part.reg[3] = 0x00;
    part.reg[4] = 0xFF;
    run(old, 60s);
    check(!old.temperature(), "a minute later still no reading: what water_mix showed as ---");
}

void configurationChanged() {
    testCase("MAX31865: the configuration changed under the driver, conversions still coming");
    fresh();
    Rtd      rtd{};
    BusScope scope{};
    untilReading(rtd, 2s);
    part.reg[0] = 0xD1;   // 3-wire: still converting, the reading silently wrong
    run(rtd, 11s);
    checkEq(part.reg[0], 0xC1, "the register check wrote it again");
    check(rtd.temperature().has_value(), "and the reading is back");
    wire();
}

void powerLost() {
    testCase("MAX31865: the part reset under the driver (POR: no conversions)");
    fresh();
    Rtd      rtd{};
    BusScope scope{};
    untilReading(rtd, 2s);
    part.powerOn();
    part.reg[0] = 0x00;
    run(rtd, 2s);
    check(rtd.temperature().has_value(), "the reading is back");
    checkEq(rtd.bringUps(), 2, "after ConversionTimeout and one more bring-up");
    checkEq(part.reg[0], 0xC1, "configured again");
    wire();
}

void overVoltage() {
    testCase("MAX31865: an overvoltage that lasts: no reading, then back when it is gone");
    fresh();
    Rtd      rtd{};
    BusScope scope{};
    untilReading(rtd, 2s);
    part.overVoltage = true;
    run(rtd, 30s);
    check(!rtd.temperature(), "no reading while the part halts");
    checkEq(rtd.standingFault(), 0x04, "the bring-up finds D2 set again after its clear");
    check(rtd.barren() > 10, "brought up again and again");
    check(rtd.pulled(Lever::vbias) > 3 && rtd.pulled(Lever::faultCycle) > 3,
          "with each lever in turn");
    part.overVoltage = false;
    auto const took  = untilReading(rtd, 5s);
    check(rtd.temperature().has_value(), "the reading is back");
    check(took < 2s, "within a conversion timeout and a bring-up");
    checkEq(rtd.standingFault(), 0x00, "and that bring-up found no fault standing");
    checkEq(rtd.barren(), 0, "the count starts over");
    checkEq(part.reg[0], 0xC1, "converting as configured");
    wire();
}

/// A fault that holds itself in the part until `by` is worked: what water_mix showed as F04 until
/// its supply was cycled, if a bit of the configuration register can do what the supply did.
void heldUntil(Part::Release by,
               Lever         lever) {
    fresh();
    Rtd      rtd{};
    BusScope scope{};
    untilReading(rtd, 2s);
    part.release     = by;
    part.overVoltage = true;
    run(rtd, 1s);
    check(!rtd.temperature(), "no reading once the conversion timeout has passed");
    auto const took = untilReading(rtd, 30s);
    check(rtd.temperature().has_value(), "the reading is back");
    check(took < 5s, "after the plain bring-ups and the levers before this one");
    checkEq(rtd.recovered(lever), 1, "credited to the lever that did it");
    checkEq(rtd.recovered(Lever::none), 0, "not to a bring-up alone");
    checkEq(rtd.pulled(Lever::none), 2, "which was tried twice first");
    checkEq(part.reg[0], 0xC1, "converting as configured");
    auto const samples = rtd.samples();
    run(rtd, 1s);
    check(rtd.samples() - samples >= 45, "and it stays: a conversion every 20 ms");
    wire();
}

void levers() {
    testCase("MAX31865: a fault that holds until VBIAS was off ends with the VBIAS lever");
    heldUntil(Part::Release::vbiasOff, Lever::vbias);
    testCase("MAX31865: one that holds until a fault-detection cycle ends with that lever");
    heldUntil(Part::Release::faultCycle, Lever::faultCycle);

    testCase("MAX31865: ... and bring-ups alone (the driver before 2026-10-06) never end it");
    fresh();
    PlainRtd plain{};
    BusScope scope{};
    untilReading(plain, 2s);
    part.release     = Part::Release::vbiasOff;
    part.overVoltage = true;
    run(plain, 60s);
    check(!plain.temperature(), "a minute later still no reading");
    check(plain.barren() > 30, "for all its bring-ups");
    checkEq(plain.pulled(Lever::vbias) + plain.pulled(Lever::faultCycle), 0, "no lever used");
}

void absentOnlyAfterAFailedBringUp() {
    testCase("MAX31865: absent() means a bring-up failed since the part last answered");
    fresh();
    Rtd      rtd{};
    BusScope scope{};
    untilReading(rtd, 2s);
    check(rtd.temperature().has_value() && !rtd.absent(), "answering: not absent");
    // the part resets under the driver: a bring-up follows, with nothing unidentified since
    part.powerOn();
    part.reg[0]     = 0x00;
    auto const from = FakeClock::now();
    while(rtd.answering() && FakeClock::now() - from < 2s) { run(rtd, 1ms); }
    check(!rtd.answering(), "the conversion timeout started it over");
    check(!rtd.absent(), "a bring-up under way is not an absent part");
    untilReading(rtd, 3s);
    check(rtd.temperature().has_value() && !rtd.absent(), "back, not absent");
    // the part is gone: the next bring-up reads nothing back
    Bus::exchange = {};
    Bus::floating = 0xFF;
    run(rtd, 3s, false);
    check(!rtd.answering() && rtd.absent(), "absent once a bring-up failed");
    // and back again
    Bus::exchange = [](std::uint8_t m) { return part.exchange(m); };
    part.powerOn();
    untilReading(rtd, 5s);
    check(rtd.temperature().has_value() && !rtd.absent(), "answering again: not absent");
    wire();
}

void noPart() {
    testCase("MAX31865: nothing on the bus is not a part");
    for(auto const level : {std::uint8_t{0x00}, std::uint8_t{0xFF}}) {
        fresh();
        Bus::exchange = {};
        Bus::floating = level;
        Rtd      rtd{};
        BusScope scope{};
        // DRDY floats low as well: the driver would read a conversion whenever it may
        Pins::level[Spi::Drdy::id] = false;
        run(rtd, 10s, false);
        check(!rtd.answering(), "not answering");
        check(rtd.absent(), "absent: the registers did not read back (water_mix's Er1)");
        check(!rtd.temperature(), "no reading");
        checkEq(rtd.bringUps(), 0U, "never brought up");
        bool conversions = false;
        for(auto const& f : Bus::frames) {
            conversions = conversions || (f.mosi.size() == 1 && f.mosi[0] == 0x01);
        }
        check(!conversions, "no conversion read from nothing");
    }
}

void callendarVanDusen() {
    testCase("MAX31865: Callendar-Van Dusen in integers");
    using Pt500 = Rtd;
    using Pt100 = Kvasir::SPI::Max31865<Bus,
                                        FakeClock,
                                        Spi::Cs,
                                        Spi::Drdy,
                                        Kvasir::Units::ohm(100),
                                        Kvasir::Units::ohm(430)>;
    checkEq(Pt500::temperatureFor(16384), 0, "R = R0 is 0 degC");
    checkEq(Pt500::resistanceFor(16384), 500000, "half of the 1 k reference");
    // R(100 degC) = 500 (1 + 0.39083 - 0.005775) = 692.5375 ohm, code 22693
    checkNear(Pt500::temperatureFor(22693), 100000.0, 5.0, "100 degC on a PT500");
    // R(-50 degC) without the C term = 80.3141 ohm against 430, code 6120
    checkNear(Pt100::temperatureFor(6120), -50000.0, 30.0, "-50 degC on a PT100");
    // R(850 degC) = 3.9048 R0: 390.48 ohm against 430 is code 29756
    check(Pt100::codeInRange(29756) && !Pt100::codeInRange(29760),
          "the Callendar-Van Dusen range ends at 850 degC");
    check(Pt500::codeInRange(32767), "a 2 x reference never reads past it");
}

}   // namespace

int main() {
    callendarVanDusen();
    bringUp();
    thresholdChanged();
    configurationChanged();
    powerLost();
    overVoltage();
    levers();
    absentOnlyAfterAFailedBringUp();
    noPart();
    return finish();
}
