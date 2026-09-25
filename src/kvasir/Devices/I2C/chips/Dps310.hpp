#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {
namespace Dps310Detail {
    /// PM_PRC / TMP_PRC: how many conversions one result averages. Above x8 the result must
    /// be shifted (CFG_REG P_SHIFT / T_SHIFT), which the description does.
    enum class Oversampling : std::uint8_t {
        x1   = 0,
        x2   = 1,
        x4   = 2,
        x8   = 3,
        x16  = 4,
        x32  = 5,
        x64  = 6,   ///< "high precision"
        x128 = 7,
    };

    /// PM_RATE / TMP_RATE: results per second in background (continuous) mode.
    enum class Rate : std::uint8_t {
        hz1   = 0,
        hz2   = 1,
        hz4   = 2,
        hz8   = 3,
        hz16  = 4,
        hz32  = 5,
        hz64  = 6,
        hz128 = 7,
    };

    /// Table 9: the scale factor a raw result is divided by, indexed by the oversampling
    /// exponent (0 = single, 6 = 64 times "high precision").
    constexpr std::array<std::int32_t, 8>
      ScaleFactor{524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

    [[nodiscard]] constexpr std::int32_t scaleFactor(std::uint8_t osr) {
        return ScaleFactor[osr & 0x07U];
    }

    /// The period the faster of the two channels delivers at: 1000 ms at 1 Hz, halved per
    /// step of the rate exponent.
    [[nodiscard]] constexpr std::chrono::milliseconds measurementPeriod(std::uint8_t rateP,
                                                                        std::uint8_t rateT) {
        auto const r = rateP > rateT ? rateP : rateT;
        return std::chrono::milliseconds{1000U >> (r & 0x07U)};
    }

    /// The Config group's value: the two channels' precision and rate. Out here because a
    /// default member initializer may not be used from inside the class that encloses it.
    struct Config {
        Oversampling osrPressure{Oversampling::x64};
        Oversampling osrTemperature{Oversampling::x1};
        Rate         ratePressure{Rate::hz1};
        Rate         rateTemperature{Rate::hz1};
    };
    /// TMP_EXT / TMP_COEF_SRCE: which temperature sensor the coefficients were trimmed on.
    enum class TempSensor : std::uint8_t { internal, external };
}   // namespace Dps310Detail

/// Infineon DPS310 barometric pressure and temperature sensor. One-byte pointer:
/// pressure 0x00..0x02 and temperature 0x03..0x05 (both 24-bit two's complement, and the
/// pointer auto-increments so all six come out of one read), pressure configuration 0x06
/// (PM_RATE 6:4, PM_PRC 3:0), temperature configuration 0x07 (TMP_EXT 7, TMP_RATE 6:4,
/// TMP_PRC 3:0), MEAS_CFG 0x08 (COEF_RDY, SENSOR_RDY, TMP_RDY, PRS_RDY and the measurement
/// mode), CFG_REG 0x09 (bit 3 T_SHIFT, bit 2 P_SHIFT: the result shift each channel needs
/// above 8x oversampling), product id 0x0D (0x10: REV_ID bits 7:4 = 1, PROD_ID bits 3:0 = 0,
/// 8.10), the eighteen calibration bytes 0x10..0x21 and COEF_SRCE 0x28.
///
/// Bring-up reads the id, COEF_SRCE and the coefficients, applies Infineon's temperature
/// erratum workaround -- the undocumented writes 0x0E = 0xA5, 0x0F = 0x96, 0x62 = 0x02,
/// 0x0E = 0x00, 0x0F = 0x00 from Infineon's own Arduino library (`correctTemp()`), without
/// which some parts report a temperature off by tens of degrees; the library then discards
/// one temperature result, which continuous mode does on its own -- then writes the two
/// configurations, CFG_REG and continuous pressure and temperature.
///
/// The compensation is the datasheet's, section 4.9:
///
///     Traw_sc = Traw / kT;  Praw_sc = Praw / kP
///     T = c0/2 + c1 * Traw_sc
///     P = c00 + Praw_sc*(c10 + Praw_sc*(c20 + Praw_sc*c30))
///           + Traw_sc*c01 + Traw_sc*Praw_sc*(c11 + Praw_sc*c21)
///
/// with kP and kT from Table 9 and the nine coefficients read once during bring-up into
/// State, the way the BME280's trimming is. It runs in 64-bit integers: the polynomial is
/// evaluated in Horner form with the scaled raw values kept as the fractions praw / kP and
/// traw / kT, one division per step, at 1/256 Pa so the truncations stay well under a pascal.
///
/// Bring-up reads MEAS_CFG first and turns the part down while COEF_RDY (bit 7) is clear: the
/// coefficients it goes on to read are not loaded yet (8.5, "Coefficients are not available
/// yet"). It writes MEAS_CFG = 0 before the configuration and the start of continuous mode --
/// Linux dps310.c: "MEAS_CFG doesn't update correctly unless first written with 0". A reading
/// takes MEAS_CFG before the result registers, since reading a result clears its RDY bit: a
/// frame with neither PRS_RDY nor TMP_RDY set has no new result -- before the first
/// conversion the result registers hold zeros, which decode to c00 Pa and c0/2 degC -- and
/// is unchanged().
///
/// The template parameters are what bring-up writes; the Config write group changes the
/// precision and rate at run time (the part is put in standby around the change, as
/// Infineon's driver does), and decode's scale factors and the Measurement period follow
/// the State from the write's completion on.
///
/// `Sensor` must match the part's TMP_COEF_SRCE bit (0x28 bit 7): the temperature
/// coefficients were trimmed against one of the two sensors and reading with the other
/// gives a wrong answer. The bit is read during bring-up and a disagreement is reported as
/// "not the chip this description is for" rather than silently mis-compensating. Parts
/// typically read 1 (the external MEMS element). 0x77, or 0x76 with SDO low.
template<Dps310Detail::Oversampling OsrPressure    = Dps310Detail::Oversampling::x64,
         Dps310Detail::Oversampling OsrTemperature = Dps310Detail::Oversampling::x1,
         Dps310Detail::Rate         RatePressure   = Dps310Detail::Rate::hz1,
         Dps310Detail::Rate         RateTemp       = Dps310Detail::Rate::hz1,
         Dps310Detail::TempSensor   Sensor         = Dps310Detail::TempSensor::external>
struct Dps310 {
    static constexpr std::string_view Name = "DPS310";
    /// Infineon DPS310. DPS310.md:966/:1372, Product and Revision ID 0x0D, reset value 0x10: PROD_ID,
    /// bits 3:0, is 0; the revision above it is left to setup(), which asks only that it is not 0.
    /// Configuration: MEAS_CFG (0x08, DPS310.md:1235): COEF_RDY and SENSOR_RDY set, MEAS_CTRL 111b,
    /// continuous pressure and temperature.
    static constexpr std::array Identity{
      RegisterCheck{"product-id", 0x0D, 1, true, 0x0F, 0x00},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"ready-and-continuous", 0x08, 1, true, 0xC7, 0xC7},
    };
    static constexpr Address7    Address       = 0x77;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 2> Addresses{0x76, 0x77};

    using Oversampling = Dps310Detail::Oversampling;
    using Rate         = Dps310Detail::Rate;

    static constexpr std::uint8_t OsrP  = static_cast<std::uint8_t>(OsrPressure);
    static constexpr std::uint8_t OsrT  = static_cast<std::uint8_t>(OsrTemperature);
    static constexpr std::uint8_t RateP = static_cast<std::uint8_t>(RatePressure);
    static constexpr std::uint8_t RateT = static_cast<std::uint8_t>(RateTemp);

    /// The coefficients are ready 40 ms after power-up (TCoef_rdy).
    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

    [[nodiscard]] static constexpr std::uint8_t prsCfg(std::uint8_t osr,
                                                       std::uint8_t rate) {
        return static_cast<std::uint8_t>(((rate & 0x07U) << 4) | (osr & 0x07U));
    }

    [[nodiscard]] static constexpr std::uint8_t tmpCfg(std::uint8_t osr,
                                                       std::uint8_t rate) {
        return static_cast<std::uint8_t>(
          (Sensor == Dps310Detail::TempSensor::external ? 0x80U : 0x00U) | ((rate & 0x07U) << 4)
          | (osr & 0x07U));
    }

    /// Above 8x oversampling the result must be shifted: bit 2 is P_SHIFT, bit 3 T_SHIFT
    /// (CFG_REG, 8.6; Linux dps310.c names bit 4 for the pressure shift, which is INT_PRS).
    [[nodiscard]] static constexpr std::uint8_t cfgReg(std::uint8_t osrP,
                                                       std::uint8_t osrT) {
        return static_cast<std::uint8_t>((osrP > 3 ? 0x04U : 0x00U) | (osrT > 3 ? 0x08U : 0x00U));
    }

    static constexpr std::uint8_t PrsCfg = prsCfg(OsrP, RateP);
    static constexpr std::uint8_t TmpCfg = tmpCfg(OsrT, RateT);
    static constexpr std::uint8_t CfgReg = cfgReg(OsrP, OsrT);

    static constexpr std::int32_t Kp = Dps310Detail::scaleFactor(OsrP);
    static constexpr std::int32_t Kt = Dps310Detail::scaleFactor(OsrT);

    static constexpr std::chrono::milliseconds MeasurementPeriod
      = Dps310Detail::measurementPeriod(RateP, RateT);

    static constexpr std::array Init{
      Step::read({.reg = 0x08, .count = 1, .offset = 19}),   // MEAS_CFG: COEF_RDY
      Step::read({.reg = 0x28, .count = 1, .offset = 0}),    // COEF_SRCE
      Step::read({.reg = 0x10, .count = 18, .offset = 1}),   // the nine calibration coefficients
      Step::identify(),
      Step::write({.reg = 0x08, .payload = {0x00}}),   // standby first
      Step::write({.reg     = 0x0E,
                   .payload = {0xA5}}),   // the temperature erratum workaround (Infineon's driver)
      Step::write({.reg = 0x0F, .payload = {0x96}}),
      Step::write({.reg = 0x62, .payload = {0x02}}),
      Step::write({.reg = 0x0E, .payload = {0x00}}),
      Step::write({.reg = 0x0F, .payload = {0x00}}),
      Step::write({.reg = 0x06, .payload = {PrsCfg}}),
      Step::write({.reg = 0x07, .payload = {TmpCfg}}),
      Step::write({.reg = 0x09, .payload = {CfgReg}}),
      Step::write(
        {.reg     = 0x08,
         .payload = {0x07},
         .delay   = std::chrono::milliseconds{100}}),   // continuous pressure and temperature
    };

    struct State {
        std::uint8_t             deviceId{};   ///< Product ID (0x0D)
        std::uint8_t             measCfg{};    ///< MEAS_CFG as bring-up found it
        Dps310Detail::TempSensor tempSensor{};
        std::int32_t             c0{}, c1{}, c00{}, c10{};
        std::int32_t             c01{}, c11{}, c20{}, c21{}, c30{};
        /// What the configuration registers hold now: the scale factors decode divides by
        /// and the rate Measurement is read at, which a Config write changes (applied()).
        std::uint8_t osrP{OsrP};
        std::uint8_t osrT{OsrT};
        std::uint8_t rateP{RateP};
        std::uint8_t rateT{RateT};
    };

    /// `state().deviceId` is the product and revision ID as the engine read it for the Identity.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint8_t>(ids[0]);
    }

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.measCfg    = data.u8(19);
        state.tempSensor = (data.u8(0) & 0x80U) != 0 ? Dps310Detail::TempSensor::external
                                                     : Dps310Detail::TempSensor::internal;
        state.osrP       = OsrP;   // what the Init script wrote
        state.osrT       = OsrT;
        state.rateP      = RateP;
        state.rateT      = RateT;

        auto const r = [&](std::size_t i) { return static_cast<std::uint32_t>(data.u8(1 + i)); };
        state.c0     = Bytes::signExtend((r(0) << 4) | (r(1) >> 4), 12);
        state.c1     = Bytes::signExtend(((r(1) & 0x0FU) << 8) | r(2), 12);
        state.c00    = Bytes::signExtend((r(3) << 12) | (r(4) << 4) | (r(5) >> 4), 20);
        state.c10    = Bytes::signExtend(((r(5) & 0x0FU) << 16) | (r(6) << 8) | r(7), 20);
        state.c01    = Bytes::signExtend((r(8) << 8) | r(9), 16);
        state.c11    = Bytes::signExtend((r(10) << 8) | r(11), 16);
        state.c20    = Bytes::signExtend((r(12) << 8) | r(13), 16);
        state.c21    = Bytes::signExtend((r(14) << 8) | r(15), 16);
        state.c30    = Bytes::signExtend((r(16) << 8) | r(17), 16);

        // PROD_ID is the Identity's. Here: a revision in 7:4 that is not 0 (an all-zero read is
        // not a DPS310; revision 1 is what the data sheet lists), and the coefficients must have
        // been trimmed against the sensor this description reads
        return (state.deviceId >> 4U) != 0 && state.tempSensor == Sensor
            && (state.measCfg & 0x80U) != 0;
    }

    struct Measurement {
        /// The faster of the two channels' rates, as configured at compile time.
        static constexpr auto       Period = MeasurementPeriod;
        static constexpr std::array Steps{
          Step::read({.reg = 0x08, .count = 1, .offset = 6}),   // MEAS_CFG: PRS_RDY, TMP_RDY
          Step::read({.reg = 0x00, .count = 6, .offset = 0})};

        /// And the rate the part runs at now, after a Config write.
        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            return Dps310Detail::measurementPeriod(state.rateP, state.rateT);
        }

        struct Sample {
            Pascal    pressure{};
            MilliDegC temperature{};   ///< 0.001 degC
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if((data.u8(6) & 0x30U) == 0) { return Outcome<Sample>::unchanged(); }
            std::int64_t const     praw = Bytes::signExtend(data.be24(0), 24);
            std::int64_t const     traw = Bytes::signExtend(data.be24(3), 24);
            std::int64_t const     kp   = Dps310Detail::scaleFactor(state.osrP);
            std::int64_t const     kt   = Dps310Detail::scaleFactor(state.osrT);
            constexpr std::int64_t S    = 256;   // fraction bits the pressure is carried at

            // c00 + ps * (c10 + ps * (c20 + ps * c30))
            auto const x2 = std::int64_t{state.c20} * S + std::int64_t{state.c30} * S * praw / kp;
            auto const x1 = std::int64_t{state.c10} * S + x2 * praw / kp;
            auto const x0 = std::int64_t{state.c00} * S + x1 * praw / kp;
            // + ts * c01 + ts * ps * (c11 + ps * c21)
            auto const y1 = std::int64_t{state.c11} * S + std::int64_t{state.c21} * S * praw / kp;
            auto const y  = y1 * traw / kt * praw / kp;
            auto const z  = std::int64_t{state.c01} * S * traw / kt;

            // T = c0 / 2 + c1 * ts, in millidegrees
            auto const t = std::int64_t{state.c0} * 500 + std::int64_t{state.c1} * 1000 * traw / kt;

            return Outcome<Sample>::ok({Units::pascal((x0 + y + z) / S), Units::milliDegC(t)});
        }
    };

    /// PRS_CFG, TMP_CFG and CFG_REG together, so the precision or rate can be changed at run
    /// time: standby, the three registers, continuous again. The scale factors decode divides
    /// by and the period Measurement is read at follow from the write's completion on. No
    /// Initial: Init writes the template's configuration, and a value set is put back after
    /// a later bring-up.
    struct Config {
        using Value                        = Dps310Detail::Config;
        static constexpr std::size_t Bytes = 3;

        [[nodiscard]] static constexpr std::array<Step,
                                                  5>
        encode(Value const&         value,
               std::span<std::byte> buffer) {
            auto const osrP  = static_cast<std::uint8_t>(value.osrPressure);
            auto const osrT  = static_cast<std::uint8_t>(value.osrTemperature);
            auto const rateP = static_cast<std::uint8_t>(value.ratePressure);
            auto const rateT = static_cast<std::uint8_t>(value.rateTemperature);
            buffer[0]        = static_cast<std::byte>(prsCfg(osrP, rateP));
            buffer[1]        = static_cast<std::byte>(tmpCfg(osrT, rateT));
            buffer[2]        = static_cast<std::byte>(cfgReg(osrP, osrT));
            return {
              Step::write(
                {.reg = 0x08, .payload = {0x00}}),   // idle while the configuration changes
              Step::writeBuffer({.reg = 0x06, .offset = 0, .count = 1}),
              Step::writeBuffer({.reg = 0x07, .offset = 1, .count = 1}),
              Step::writeBuffer({.reg = 0x09, .offset = 2, .count = 1}),
              Step::write(
                {.reg = 0x08, .payload = {0x07}}),   // continuous pressure and temperature
            };
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.osrP  = static_cast<std::uint8_t>(value.osrPressure);
            state.osrT  = static_cast<std::uint8_t>(value.osrTemperature);
            state.rateP = static_cast<std::uint8_t>(value.ratePressure);
            state.rateT = static_cast<std::uint8_t>(value.rateTemperature);
        }
    };

    using Reads  = List<Measurement>;
    using Writes = List<Config>;
};

}   // namespace Kvasir::I2C::Chips
