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

namespace Ina238Detail {
    /// ADCRANGE (CONFIG bit 4): the shunt full scale.
    enum class Range : std::uint8_t {
        mV160 = 0,   ///< +-163.84 mV, 5 uV/LSB
        mV40  = 1,   ///< +-40.96 mV, 1.25 uV/LSB
    };

    /// The CONFIG register's settable field. Out here because a default member initializer
    /// may not be used from inside the class that encloses it.
    struct Config {
        std::chrono::milliseconds convDelay{};   ///< CONVDLY, 2 ms steps, bits 13:6
    };
}   // namespace Ina238Detail

/// Texas Instruments INA238 current, voltage, power and temperature monitor. One-byte
/// pointer, 16-bit big-endian registers except power, which is 24 bits: configuration
/// 0x00 (reset 0x0000), ADC configuration 0x01 (reset 0xFB68), shunt calibration 0x02,
/// shunt voltage 0x04, bus voltage 0x05, die temperature 0x06, current 0x07, power 0x08,
/// diagnostics DIAG_ALRT 0x0B (MATHOF bit 9: the current or power computation overflowed,
/// and the frame is rejected), manufacturer id 0x3E (0x5449) and device id 0x3F (DIEID 238h in
/// 15:4, REV_ID in 3:0; Table 6-21). Bring-up reads both ids and rejects anything else; any
/// revision of the die is accepted.
///
/// Bus voltage is 3.125 mV/LSB, die temperature 125 m degC/LSB in the top 12 bits, shunt
/// voltage 1.25 uV/LSB on the +-40 mV range and 5 uV/LSB on the +-163.84 mV one, current
/// is CURRENT_LSB = MaxCurrent / 32768 and power 0.2 x CURRENT_LSB.
/// SHUNT_CAL = 819.2e6 x CURRENT_LSB x R_shunt, and four times that on the smaller range
/// (8.1.2).
///
/// The shunt value, the full-scale current and the ADC range are template parameters, as
/// on the INA226: decode needs the scale, and the calibration register then has a
/// compile-time Initial instead of being computed in a constructor. `Ina238<>` is a 2
/// mOhm shunt at 10 A full scale on the wide range, read every `Timing::Period` (500 ms). The
/// Config group writes CONVDLY with the range the template fixed; `Reset` is the one-shot
/// RST bit, a Transient group that is not replayed after a bring-up. 0x40..0x4F by A1 A0.
template<MicroOhm            Shunt      = Units::microOhm(2000),
         MilliAmp            MaxCurrent = Units::milliAmp(10000),
         Ina238Detail::Range Range      = Ina238Detail::Range::mV160,
         typename Timing                = DefaultTiming>
struct Ina238 {
    static constexpr std::string_view Name = "INA238";
    /// TI INA238. INA238.md:1258 (MANUFACTURER_ID 3Eh = 5449h) and :1272..1282 (DEVICE_ID 3Fh: DIEID
    /// 238h in 15:4).
    static constexpr std::array Identity{
      RegisterCheck{"manufacturer-id", 0x3E, 2, true, 0xFFFF, 0x5449},
      RegisterCheck{      "device-id", 0x3F, 2, true, 0xFFF0, 0x2380},
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

    /// Not a datasheet figure (neither the INA238's nor the INA237's gives a start-up time):
    /// a margin for the supply to settle before the first transaction.
    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

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

    static constexpr bool Range40mV = Range == Ina238Detail::Range::mV40;

    /// CURRENT_LSB: full scale over a signed 15-bit range.
    static constexpr NanoAmp CurrentLsb = Units::nanoAmp(
      static_cast<std::uint64_t>(Units::value(MaxCurrent)) * 1'000'000ULL / 32768ULL);

    /// SHUNT_CAL = 819.2e6 x CURRENT_LSB[A] x R[Ohm], in the integer units above and
    /// rounded rather than truncated, and x4 on the small range.
    static constexpr std::uint64_t CalibrationWanted = [] {
        auto const lsb   = static_cast<std::uint64_t>(Units::value(CurrentLsb));   // nA
        auto const shunt = static_cast<std::uint64_t>(Units::value(Shunt));        // uOhm
        auto       v     = (8192ULL * lsb * shunt + 5'000'000'000ULL) / 10'000'000'000ULL;
        if constexpr(Range40mV) { v *= 4; }
        return v;
    }();
    static_assert(CalibrationWanted <= 0x7FFF,
                  "SHUNT_CAL is 15 bits: lower MaxCurrent or the shunt until 819.2e6 x "
                  "CURRENT_LSB x R (x4 on the 40 mV range) fits");
    static constexpr std::uint16_t Calibration = static_cast<std::uint16_t>(CalibrationWanted);

    /// decode() scales by CURRENT_LSB while the part divides by the rounded SHUNT_CAL, so a
    /// small SHUNT_CAL is a scale error: at 50 or more it is under 1 %.
    static_assert(Calibration >= 50,
                  "SHUNT_CAL rounds to under 50, which puts current and power more than 1 % off "
                  "the scale decode() uses: raise MaxCurrent or the shunt");
    /// POWER is 24 bits of 0.2 x CURRENT_LSB, reported in microwatts in 32 bits: 85 V at the
    /// full-scale current has to fit.
    static_assert(85ULL * static_cast<std::uint64_t>(Units::value(MaxCurrent)) * 1000ULL
                    <= 0xFFFFFFFFULL,
                  "85 V x MaxCurrent is past the 4294 W a MicroWatt holds");

    struct State {
        std::uint16_t manufacturer{};
        std::uint16_t deviceId{};   ///< Device ID (0x3F): 0x2381 on the INA238
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.manufacturer = static_cast<std::uint16_t>(ids[0]);
        state.deviceId     = static_cast<std::uint16_t>(ids[1]);
    }

    struct Power {
        static constexpr auto Period = ReadPeriod;

        static constexpr std::array Steps{
          Step::read({.reg = 0x04, .count = 2, .offset = 0}),    // shunt voltage
          Step::read({.reg = 0x05, .count = 2, .offset = 2}),    // bus voltage
          Step::read({.reg = 0x06, .count = 2, .offset = 4}),    // die temperature
          Step::read({.reg = 0x07, .count = 2, .offset = 6}),    // current
          Step::read({.reg = 0x08, .count = 3, .offset = 8}),    // power, 24 bit
          Step::read({.reg = 0x0B, .count = 2, .offset = 11}),   // DIAG_ALRT
        };

        struct Sample {
            NanoVolt      shuntVoltage{};
            MicroVolt     busVoltage{};
            MilliDegC     dieTemperature{};
            MicroAmp      current{};
            MicroWatt     power{};
            std::uint16_t diagnostics{};   ///< DIAG_ALRT as read

            /// The alert flags of DIAG_ALRT (7.6.1.9).
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
            auto const diag = data.be16(11);
            if((diag & 0x0200U) != 0) { return Outcome<Sample>::reject(); }
            Sample sample{};
            sample.shuntVoltage = Units::nanoVolt(static_cast<std::int32_t>(data.s16be(0))
                                                  * (Range40mV ? 1250 : 5000));
            sample.busVoltage = Units::microVolt(static_cast<std::uint32_t>(data.be16(2)) * 3125U);
            sample.dieTemperature
              = Units::milliDegC(static_cast<std::int32_t>(data.s16be(4) >> 4) * 125);
            auto const lsb = static_cast<std::int64_t>(Units::value(CurrentLsb));   // nA
            sample.current = Units::microAmp(static_cast<std::int64_t>(data.s16be(6)) * lsb / 1000);
            // POWER_LSB is 0.2 x CURRENT_LSB, so nanowatts / 5 per count.
            sample.power = Units::microWatt(static_cast<std::int64_t>(data.be24(8)) * lsb / 5000);
            sample.diagnostics = diag;
            return Outcome<Sample>::ok(sample);
        }
    };

    /// CONFIG without its reset bit: the conversion delay, with ADCRANGE from the template
    /// parameter, which decode and the calibration are computed for.
    struct Config {
        using Value                        = Ina238Detail::Config;
        static constexpr std::size_t Bytes = 2;

        static constexpr Value Initial{std::chrono::milliseconds::zero()};

        [[nodiscard]] static constexpr std::uint16_t raw(Value const& value) {
            std::uint16_t r = 0;
            r               = static_cast<std::uint16_t>(
              r
              | ((static_cast<unsigned>(value.convDelay / std::chrono::milliseconds{2}) & 0xFFU)
                 << 6));
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

    using AdcConfig = Groups::Word<0x01, 0xFB68>;
    /// Computed from the template parameters, written after every bring-up.
    using ShuntCal = Groups::Word<0x02, Calibration>;

    using Reads  = List<Power>;
    using Writes = List<Config, AdcConfig, ShuntCal, Reset>;
};

/// The INA237 is the same silicon at a lower accuracy grade: same register map, same
/// conversion factors (5 uV / 1.25 uV shunt, 3.125 mV bus, 125 m degC die,
/// SHUNT_CAL = 819.2e6 x CURRENT_LSB x R), so it is the same description under another name.
/// Its datasheet's register map ends at MANUFACTURER_ID 3Eh and documents no DEVICE_ID, so
/// its bring-up checks the manufacturer alone.
template<MicroOhm            Shunt      = Units::microOhm(2000),
         MilliAmp            MaxCurrent = Units::milliAmp(10000),
         Ina238Detail::Range Range      = Ina238Detail::Range::mV160,
         typename Timing                = DefaultTiming>
struct Ina237 : Ina238<Shunt, MaxCurrent, Range, Timing> {
    using Base = Ina238<Shunt, MaxCurrent, Range, Timing>;

    static constexpr std::string_view Name = "INA237";
    /// TI INA237. INA237.md:1233, MANUFACTURER_ID 3Eh = 5449h. Its data sheet lists no device ID.
    /// Configuration: ADC_CONFIG (01h) MODE 15:12 = Fh, continuous (INA237.md:934).
    static constexpr std::array Identity{
      RegisterCheck{"manufacturer-id", 0x3E, 2, true, 0xFFFF, 0x5449},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"continuous", 0x01, 2, true, 0xF000, 0xF000},
    };

    /// MANUFACTURER_ID alone: 3Fh is not a register the INA237 documents.

    /// The INA237's data sheet lists a manufacturer ID alone: its Identity is one register.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     Base::State&                   state) {
        state.manufacturer = static_cast<std::uint16_t>(ids[0]);
        state.deviceId     = 0;
    }
};

}   // namespace Kvasir::I2C::Chips
