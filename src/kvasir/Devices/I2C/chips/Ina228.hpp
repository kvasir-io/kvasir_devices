#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Ina228Detail {
    /// ADCRANGE (CONFIG bit 4): the shunt full scale.
    enum class Range : std::uint8_t {
        mV160 = 0,   ///< +-163.84 mV, 312.5 nV/LSB
        mV40  = 1,   ///< +-40.96 mV, 78.125 nV/LSB
    };

    /// TEMPCOMP (CONFIG bit 5): the shunt temperature compensation.
    enum class Tempco : std::uint8_t { off, on };

    /// The CONFIG register's settable fields. Out here because a default member initializer
    /// may not be used from inside the class that encloses it.
    struct Config {
        std::chrono::milliseconds convDelay{};   ///< CONVDLY, 2 ms steps, bits 13:6
        Tempco                    tempCompensation{Tempco::off};   ///< TEMPCOMP, bit 5
    };
}   // namespace Ina228Detail

/// Texas Instruments INA228 85 V, 20-bit current, voltage, power, energy and charge
/// monitor. One-byte pointer: configuration 0x00, ADC configuration 0x01, shunt
/// calibration 0x02, shunt voltage 0x04 (24 bit), bus voltage 0x05 (24 bit), die
/// temperature 0x06 (16 bit), current 0x07 (24 bit), power 0x08 (24 bit), diagnostics
/// DIAG_ALRT 0x0B (MATHOF bit 9: the current or power computation overflowed, and the
/// frame is rejected), manufacturer id 0x3E (0x5449) and device id 0x3F (0x2281).
///
/// The three 24-bit measurements carry a 20-bit value in bits 23:4, two's complement for
/// shunt and current. Conversion factors: shunt 312.5 nV/LSB (78.125 nV on the +-40.96 mV
/// range), bus 195.3125 uV/LSB, die temperature 7.8125 m degC/LSB, current CURRENT_LSB =
/// MaxCurrent / 2^19 and power 3.2 x CURRENT_LSB.
/// SHUNT_CAL = 13107.2e6 x CURRENT_LSB x R_shunt, and four times that on the smaller range.
///
/// As on the INA226 and INA238, the shunt value, full-scale current and ADC range are
/// template parameters: decode needs the scale, and the calibration register then has a
/// compile-time Initial. `Ina228<>` is a 2 mOhm shunt at 10 A full scale on the wide range,
/// read every `Timing::Period` (500 ms). The Config group writes CONVDLY and the tempco enable
/// with the range the template fixed; `Reset` is the one-shot RST bit, a Transient group
/// that is not replayed after a bring-up.
/// The energy and charge accumulators are 40-bit and deliberately not read cyclically --
/// they are a metering feature with their own reset semantics. 0x40..0x4F by A1 A0.
template<MicroOhm            Shunt      = Units::microOhm(2000),
         MilliAmp            MaxCurrent = Units::milliAmp(10000),
         Ina228Detail::Range Range      = Ina228Detail::Range::mV160,
         typename Timing                = DefaultTiming>
struct Ina228 {
    static constexpr std::string_view Name = "INA228";
    /// TI INA228. INA228.md:1373 (MANUFACTURER_ID 3Eh = 5449h, "TI") and :1387..1398 (DEVICE_ID 3Fh:
    /// DIEID 228h in 15:4, the revision below it).
    /// Configuration: ADC_CONFIG (01h) MODE 15:12 = Fh, continuous bus, shunt and temperature
    /// (INA228.md:1024).
    static constexpr std::array Identity{
      RegisterCheck{"manufacturer-id", 0x3E, 2, true, 0xFFFF, 0x5449},
      RegisterCheck{      "device-id", 0x3F, 2, true, 0xFFF0, 0x2280},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"continuous", 0x01, 2, true, 0xF000, 0xF000},
    };
    static constexpr Address7    Address       = 0x40;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 16> Addresses{0x40,
                                                        0x41,
                                                        0x42,
                                                        0x43,
                                                        0x44,
                                                        0x45,
                                                        0x46,
                                                        0x47,
                                                        0x48,
                                                        0x49,
                                                        0x4A,
                                                        0x4B,
                                                        0x4C,
                                                        0x4D,
                                                        0x4E,
                                                        0x4F};

    static_assert(Shunt > Units::microOhm(0) && MaxCurrent > Units::milliAmp(0),
                  "the shunt value and the full-scale current are what every reading is "
                  "scaled by; neither may be zero");
    static constexpr std::chrono::milliseconds ReadPeriod = [] {
        if constexpr(requires { Timing::Period; }) {
            return Kvasir::asDuration(Timing::Period);
        } else {
            return std::chrono::milliseconds{500};
        }
    }();
    static_assert(ReadPeriod > std::chrono::milliseconds::zero(),
                  "Timing::Period is how often Power is read; 0 never reads it");

    static constexpr bool Range40mV = Range == Ina228Detail::Range::mV40;

    /// CURRENT_LSB: full scale over 2^19.
    static constexpr NanoAmp CurrentLsb = Units::nanoAmp(
      static_cast<std::uint64_t>(Units::value(MaxCurrent)) * 1'000'000ULL / 524288ULL);

    /// SHUNT_CAL = 13107.2e6 x CURRENT_LSB[A] x R[Ohm], rounded, and x4 on the small range.
    static constexpr std::uint64_t CalibrationWanted = [] {
        auto const lsb   = static_cast<std::uint64_t>(Units::value(CurrentLsb));   // nA
        auto const shunt = static_cast<std::uint64_t>(Units::value(Shunt));        // uOhm
        auto       v     = (131072ULL * lsb * shunt + 5'000'000'000ULL) / 10'000'000'000ULL;
        if constexpr(Range40mV) { v *= 4; }
        return v;
    }();
    static_assert(CalibrationWanted <= 0x7FFF,
                  "SHUNT_CAL is 15 bits: lower MaxCurrent or the shunt until 13107.2e6 x "
                  "CURRENT_LSB x R (x4 on the 40 mV range) fits");
    static constexpr std::uint16_t Calibration = static_cast<std::uint16_t>(CalibrationWanted);

    /// decode() scales by CURRENT_LSB while the part divides by the rounded SHUNT_CAL, so a
    /// small SHUNT_CAL is a scale error: at 50 or more it is under 1 %.
    static_assert(Calibration >= 50,
                  "SHUNT_CAL rounds to under 50, which puts current and power more than 1 % off "
                  "the scale decode() uses: raise MaxCurrent or the shunt");
    /// POWER is 24 bits of 3.2 x CURRENT_LSB, reported in microwatts in 32 bits: 85 V at the
    /// full-scale current has to fit.
    static_assert(85ULL * static_cast<std::uint64_t>(Units::value(MaxCurrent)) * 1000ULL
                    <= 0xFFFFFFFFULL,
                  "85 V x MaxCurrent is past the 4294 W a MicroWatt holds");

    struct State {
        std::uint16_t manufacturer{};
        std::uint16_t deviceId{};   ///< Device ID (0x3F)
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.manufacturer = static_cast<std::uint16_t>(ids[0]);
        state.deviceId     = static_cast<std::uint16_t>(ids[1]);
    }

    struct Power {
        static constexpr auto Period = ReadPeriod;

        static constexpr std::array Steps{
          Step::read({.reg = 0x04, .count = 3, .offset = 0}),    // shunt voltage, 20 bit in 23:4
          Step::read({.reg = 0x05, .count = 3, .offset = 3}),    // bus voltage, 20 bit in 23:4
          Step::read({.reg = 0x06, .count = 2, .offset = 6}),    // die temperature, 16 bit
          Step::read({.reg = 0x07, .count = 3, .offset = 8}),    // current, 20 bit in 23:4
          Step::read({.reg = 0x08, .count = 3, .offset = 11}),   // power, 24 bit
          Step::read({.reg = 0x0B, .count = 2, .offset = 14}),   // DIAG_ALRT
        };

        /// The 20-bit two's complement value in bits 23:4 of a 24-bit register.
        [[nodiscard]] static constexpr std::int32_t signed20(std::uint32_t raw24) {
            return Bytes::signExtend(raw24 >> 4, 20);
        }

        struct Sample {
            NanoVolt      shuntVoltage{};
            MicroVolt     busVoltage{};
            MilliDegC     dieTemperature{};
            MicroAmp      current{};
            MicroWatt     power{};
            std::uint16_t diagnostics{};   ///< DIAG_ALRT as read

            /// The alert flags of DIAG_ALRT (Table 7-14).
            [[nodiscard]] constexpr bool busOverLimit() const {
                return (diagnostics & 0x0010U) != 0;   // BUSOL
            }

            [[nodiscard]] constexpr bool busUnderLimit() const {
                return (diagnostics & 0x0008U) != 0;   // BUSUL
            }

            [[nodiscard]] constexpr bool powerOverLimit() const {
                return (diagnostics & 0x0004U) != 0;   // POL
            }

            [[nodiscard]] constexpr bool temperatureOverLimit() const {
                return (diagnostics & 0x0080U) != 0;   // TMPOL
            }
        };

        /// MATHOF (DIAG_ALRT bit 9) set means the current or power register overflowed and is
        /// not a reading: rejected.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            auto const diag = data.be16(14);
            if((diag & 0x0200U) != 0) { return Outcome<Sample>::reject(); }
            Sample     sample{};
            auto const shunt = signed20(data.be24(0));
            // 312.5 nV is 625/2, and 78.125 nV is 625/8: both exact in integers.
            sample.shuntVoltage = Units::nanoVolt(Range40mV ? shunt * 625 / 8 : shunt * 625 / 2);
            // 195.3125 uV is 3125/16.
            sample.busVoltage = Units::microVolt(static_cast<std::uint32_t>(
              static_cast<std::uint64_t>(data.be24(3) >> 4) * 3125ULL / 16ULL));
            sample.dieTemperature
              = Units::milliDegC(static_cast<std::int32_t>(data.s16be(6)) * 125 / 16);
            auto const lsb = static_cast<std::int64_t>(Units::value(CurrentLsb));   // nA
            sample.current
              = Units::microAmp(static_cast<std::int64_t>(signed20(data.be24(8))) * lsb / 1000);
            // POWER_LSB is 3.2 x CURRENT_LSB: x32/10, then nW -> uW.
            sample.power
              = Units::microWatt(static_cast<std::int64_t>(data.be24(11)) * lsb * 32 / 10'000);
            sample.diagnostics = diag;
            return Outcome<Sample>::ok(sample);
        }
    };

    /// CONFIG without its reset bit: the conversion delay and the shunt temperature
    /// compensation, with ADCRANGE from the template parameter, which decode and the
    /// calibration are computed for.
    struct Config {
        using Value                        = Ina228Detail::Config;
        static constexpr std::size_t Bytes = 2;

        static constexpr Value Initial{std::chrono::milliseconds::zero(),
                                       Ina228Detail::Tempco::off};

        [[nodiscard]] static constexpr std::uint16_t raw(Value const& value) {
            std::uint16_t r = 0;
            r               = static_cast<std::uint16_t>(
              r
              | ((static_cast<unsigned>(value.convDelay / std::chrono::milliseconds{2}) & 0xFFU)
                 << 6));
            if(value.tempCompensation == Ina228Detail::Tempco::on) {
                r = static_cast<std::uint16_t>(r | (1U << 5));
            }
            if(Range40mV) { r = static_cast<std::uint16_t>(r | (1U << 4)); }
            return r;
        }

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            putBe16(buffer, 0, raw(value));
            return Step::writeBuffer({.reg = 0x00, .offset = 0, .count = 2});
        }
    };

    /// RST (CONFIG bit 15): a one-shot that puts every register at its reset value, after
    /// which the engine re-sends what the application had set. Transient: never replayed.
    /// `dev.set<Reset>({})`.
    struct Reset {
        struct Value {};

        static constexpr std::size_t Bytes     = 2;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr Step encode(Value const&,
                                                   std::span<std::byte> buffer) {
            putBe16(buffer, 0, 0x8000);
            return Step::writeBuffer(
              {.reg = 0x00, .offset = 0, .count = 2, .delay = std::chrono::milliseconds{2}});
        }
    };

    /// Reset 0xFB68: continuous shunt, bus and temperature, 1052 us conversions, 1 average.
    using AdcConfig = Groups::Word<0x01, 0xFB68>;
    using ShuntCal  = Groups::Word<0x02, Calibration>;

    using Reads  = List<Power>;
    using Writes = List<Config, AdcConfig, ShuntCal, Reset>;
};

}   // namespace Kvasir::I2C::Chips
