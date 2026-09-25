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

namespace Nau7802Detail {
    /// AVDD: CTRL1 VLDO (5:3) with PU_CTRL AVDDS (bit 7) set, or `external` -- AVDDS clear, AVDD
    /// from the pin (the power-on default). Each enumerator is the VLDO code of its voltage
    /// (11.2: 111 = 2.4 V .. 000 = 4.5 V). 8.2's prose lists "off, 2.4V, 2.7V, 3.0V 3.3V, 3.6V,
    /// 4.2V, and 4.5V" -- eight levels counting off, without 3.9 V; 11.2's table has 010 = 3.9 V,
    /// which is what this follows (verify on a part).
    enum class Ldo : std::uint8_t {
        v4_5     = 0,
        v4_2     = 1,
        v3_9     = 2,
        v3_6     = 3,
        v3_3     = 4,
        v3_0     = 5,
        v2_7     = 6,
        v2_4     = 7,
        external = 8,
    };

    /// CTRL1 GAINS (2:0): x1 .. x128 (11.2).
    enum class Gain : std::uint8_t {
        x1   = 0,
        x2   = 1,
        x4   = 2,
        x8   = 3,
        x16  = 4,
        x32  = 5,
        x64  = 6,
        x128 = 7,
    };

    /// CTRL2 CRS (6:4): conversions per second (11.3). 100..110 are not in the table.
    enum class Rate : std::uint8_t { sps10 = 0, sps20 = 1, sps40 = 2, sps80 = 3, sps320 = 7 };

    /// CTRL2 CHS (bit 7): VIN1P/VIN1N or VIN2P/VIN2N.
    enum class Channel : std::uint8_t { ch1 = 0, ch2 = 1 };

    /// Whether a Cfilter is fitted across VIN2P/VIN2N (9.4).
    enum class FilterCap : std::uint8_t { absent, fitted };

    /// PU_CTRL CS: conversions stopped or running.
    enum class Cycle : std::uint8_t { stopped, running };

    /// CTRL2 CALS: a write with `start` begins the calibration CALMOD names.
    enum class Calibration : std::uint8_t { idle, start };

    /// CTRL2 CALMOD (1:0), what CALS starts (11.3). 01 is reserved.
    enum class CalMode : std::uint8_t { internalOffset = 0, systemOffset = 2, systemGain = 3 };

    /// The LDO's output in millivolts by VLDO code (11.2).
    inline constexpr std::array<std::int32_t, 8>
      LdoMilliVolt{4500, 4200, 3900, 3600, 3300, 3000, 2700, 2400};

    /// Conversions per second by CRS code; 0 for the codes the table leaves out.
    inline constexpr std::array<std::uint16_t, 8> RateSps{10, 20, 40, 80, 0, 0, 0, 320};

    [[nodiscard]] constexpr unsigned gainFactor(Gain gain) {
        return 1U << static_cast<unsigned>(gain);
    }

    [[nodiscard]] constexpr std::uint16_t sps(Rate rate) {
        return RateSps[static_cast<std::size_t>(rate) & 0x07U];
    }

    /// One conversion, rounded up: 100, 50, 25, 13 and 4 ms.
    [[nodiscard]] constexpr std::chrono::milliseconds conversionTime(Rate rate) {
        auto const s = sps(rate);
        return std::chrono::milliseconds{(1000U + s - 1U) / s};
    }

    /// The period Conversion is read at: a conversion and 5 % more, rounded up (105, 53, 27,
    /// 14 and 4 ms). The internal RC oscillator is +-3 % (7.3), so a part that runs slow still
    /// has a result ready at every read instead of drifting into the check's retries; the cost
    /// is one conversion in about twenty not read at all.
    [[nodiscard]] constexpr std::chrono::milliseconds readPeriod(Rate rate) {
        auto const s = sps(rate);
        return std::chrono::milliseconds{(1050U + s - 1U) / s};
    }

    /// PU_CTRL: PUD and PUA, AVDDS unless AVDD is external, CS when conversions run (11.1).
    [[nodiscard]] constexpr std::uint8_t puCtrl(Ldo   ldo,
                                                Cycle cycle) {
        return static_cast<std::uint8_t>((ldo == Ldo::external ? 0x00U : 0x80U)
                                         | (cycle == Cycle::running ? 0x10U : 0x00U) | 0x06U);
    }

    /// CTRL1: VLDO in 5:3, GAINS in 2:0; CRP and DRDY_SEL at their defaults.
    [[nodiscard]] constexpr std::uint8_t ctrl1(Ldo  ldo,
                                               Gain gain) {
        auto const vldo = ldo == Ldo::external ? 0U : static_cast<unsigned>(ldo);
        return static_cast<std::uint8_t>((vldo << 3) | static_cast<unsigned>(gain));
    }

    /// CTRL2: CHS, CRS, CALS and CALMOD.
    [[nodiscard]] constexpr std::uint8_t ctrl2(Channel     channel,
                                               Rate        rate,
                                               CalMode     mode        = CalMode::internalOffset,
                                               Calibration calibration = Calibration::idle) {
        return static_cast<std::uint8_t>(
          (static_cast<unsigned>(channel) << 7) | (static_cast<unsigned>(rate) << 4)
          | (calibration == Calibration::start ? 0x04U : 0x00U) | static_cast<unsigned>(mode));
    }

    /// REG0x1C PGA_PWR: PGA_CAP_EN (bit 7), the Cfilter across VIN2P/VIN2N (9.4, 11.15) -- only
    /// while channel 1 is measured, since the capacitor sits on channel 2's pins. The bias
    /// currents stay at their 100 % defaults.
    [[nodiscard]] constexpr std::uint8_t pgaPwr(FilterCap filterCap,
                                                Channel   channel) {
        return filterCap == FilterCap::fitted && channel == Channel::ch1 ? 0x80 : 0x00;
    }

    /// The differential input a code stands for. Full scale is (VINxP - VINxN) = +-0.5 x VREF /
    /// gain (9.2, and 7.2's full-scale input range) over 2^23 codes, so one code is
    /// VREF / (gain x 2^24). Linux nau7802.c scales by VREF / 2^23 per code, twice this.
    [[nodiscard]] constexpr MicroVolt toVoltage(std::int32_t code,
                                                MilliVolt    reference,
                                                Gain         gain) {
        return Units::microVolt(static_cast<std::int64_t>(code) * Units::value(reference) * 1000
                                / (static_cast<std::int64_t>(gainFactor(gain)) << 24));
    }

    // -2^23 at gain 1 against 3.3 V is -0.5 x 3.3 V; the top code is one code short of +1.65 V,
    // and at x128 full scale is 12.89 mV.
    static_assert(Units::value(toVoltage(-8'388'608,
                                         Units::milliVolt(3300),
                                         Gain::x1))
                  == -1'650'000);
    static_assert(Units::value(toVoltage(8'388'607,
                                         Units::milliVolt(3300),
                                         Gain::x1))
                  == 1'649'999);
    static_assert(Units::value(toVoltage(8'388'607,
                                         Units::milliVolt(3300),
                                         Gain::x128))
                  == 12'890);

    /// The four knobs the Settings write group holds (its Value), defaulting to the template's.
    template<Ldo L, Gain G, Rate R, Channel C>
    struct Config {
        Ldo     ldo{L};
        Gain    gain{G};
        Rate    rate{R};
        Channel channel{C};

        friend constexpr bool operator==(Config const&,
                                         Config const&) = default;
    };

    /// One channel's calibration registers (11.4 .. 11.7): OCAL 24 bits, GCAL 32 bits with
    /// 0x00800000 = 1.0 (bits 31:23 weigh 2^8 .. 2^0, 22:0 the fraction; verify: the converted
    /// bit table is mangled). The offset is kept sign-extended from 24 bits, which round-trips
    /// the register whatever its encoding (verify: the table marks bit 23 "+/-", two's
    /// complement is assumed).
    struct ChannelCalibration {
        std::int32_t  offset{};
        std::uint32_t gain{0x0080'0000U};

        friend constexpr bool operator==(ChannelCalibration const&,
                                         ChannelCalibration const&) = default;
    };
}   // namespace Nau7802Detail

/// Nuvoton NAU7802 24-bit two-channel ADC for bridge sensors (datasheet V1.7, January 2012). One
/// byte of register pointer that increments on burst reads and writes (8.3.2, 8.3.3); fixed at
/// 0x2A (8.3.2).
///
/// Registers (10, 11): PU_CTRL 0x00 (AVDDS 7, OSCS 6, CR 5 read-only, CS 4, PUR 3 read-only,
/// PUA 2, PUD 1, RR 0), CTRL1 0x01 (CRP 7, DRDY_SEL 6, VLDO 5:3, GAINS 2:0), CTRL2 0x02 (CHS 7,
/// CRS 6:4, CAL_ERR 3 read-only, CALS 2, CALMOD 1:0), channel 1 OCAL 0x03..0x05 and GCAL
/// 0x06..0x09, channel 2 OCAL 0x0A..0x0C and GCAL 0x0D..0x10, I2C control 0x11, ADCO
/// 0x12..0x14, the ADC register 0x15, PGA 0x1B, PGA_PWR 0x1C (PGA_CAP_EN 7) and the revision
/// 0x1F (3:0 read 1111, 11.15.3).
///
/// Bring-up is 9.1's sequence: RR set then cleared with PUD (the part reports PUR "after about
/// 200 microseconds"; there is no check step in an Init script, so it waits 2 ms and setup()
/// looks at PUR afterwards), the revision, CTRL1 (LDO and gain), CTRL2 (channel and rate),
/// REG0x15 = 0x30 (9.1 4b: REG_CHPS 11, the chopper clock off, 11.10), PGA_CAP_EN when channel 1
/// is measured and a Cfilter is fitted (9.4), then PUA, AVDDS and CS in one write. 9.1 4a
/// gives that write as R0x00 = 0xAE, which also sets the read-only CR and PUR and leaves CS
/// clear although step 5 says nothing converts without it (verify: taken as a typo; this writes
/// AVDDS | CS | PUA | PUD, as SparkFun's library ends up with). Then `AnalogSettle` for the
/// analog part and the LDO -- 7.3's TRDY, "5 sample times plus 100 ms", at the template's rate:
/// 600 ms at 10 SPS -- and only then an internal offset calibration (8.6, "recommended after
/// initial power-up"), so it does not run against a front end that is still settling.
///
/// The datasheet gives no calibration time -- CALS reads 1 until it is done (8.6, 11.3). The
/// Init script waits `CalibrationWait` (eight conversions and 100 ms at the template's rate;
/// verify on a part) and reads CTRL2 back. setup() rejects the part when the revision nibble is
/// not 1111, when PUR is clear, or when CAL_ERR is set: 8.6 says that with a calibration error
/// "all data output could be invalid" and 8.6.3 recommends calibrating again, which is what the
/// engine's next bring-up does (UnidentifiedRetry). A calibration still running at the read-back
/// is not rejected -- it is recorded in State::calibrating -- because the wait is a guess and a
/// part that is slow to calibrate is not a broken one.
///
/// Conversion reads CR (PU_CTRL bit 5, 8.3) and, once set, ADCO: the result latches when 0x12 is
/// addressed (11.9). The check retries every quarter conversion, up to the engine's MaxRetries;
/// the first conversions after the bring-up are the "six cycles of data conversion" 8.8 has
/// the part discard itself after a reset, so a few rejected<Conversion>() there are expected.
/// Both channels share the one ADC and its digital filter, so the conversions right after a
/// channel, gain or rate change still carry the old input: a Settings write is followed by
/// six conversions at the new rate with the part's other groups off the wire (Linux
/// nau7802.c discards six after any such change). Whether reading ADCO clears CR is not stated (verify); the period is longer than
/// a conversion, so a read never takes the same result twice either way. The code is 24-bit
/// two's complement (verify: stated only through 8.1's "only positive digital output codes"
/// for a single-ended input).
///
/// `voltage()` assumes REFP is AVDD and REFN AVSS (9.4's circuit), so VREF is the LDO voltage,
/// or `Reference` when AVDD comes from the pin (`Ldo::external`). `Cfilter` says a Cfilter is
/// fitted across VIN2P/VIN2N (9.4); PGA_CAP_EN is set only while channel 1 is measured. The
/// default, `fitted`, is the datasheet's reference circuit and the common load-cell boards
/// (SparkFun Qwiic Scale); a board without the capacitor needs `FilterCap::absent`.
///
/// At run time:
///
/// * `Settings` -- LDO, gain, rate and channel, CTRL1 and CTRL2 plus PGA_PWR and a CS edge.
///   One group rather than four because LDO and gain share CTRL1 and rate and channel share
///   CTRL2, and a write group cannot see the chip State to fill in the other field; change one
///   field with `dev.modify<Settings>([](auto& s) { s.gain = Gain::x64; })`. decode() and the
///   period follow from the write's completion on. 8.6 recommends calibrating again after a
///   gain, rate, channel or supply change.
/// * `Calibrate` -- Transient: CALS with a CALMOD (internal offset, system offset, system gain;
///   the system modes use the inputs as the application has set them up, 8.6.1), on CTRL2 with
///   the rate and channel the application passes (`calibration(dev.value<Settings>())`). The
///   register is read back `CalibrationWait` later and counts as done when CALS is clear, CAL_ERR
///   clear and the rest as written; otherwise CALS is written again (ignored while one is
///   running, 11.3), at most MaxRetries times, each counted by mismatches<Calibrate>().
/// * `Calibration` -- on request only: CTRL2 and both channels' OCAL/GCAL in one burst, for an
///   application that saves a calibration.
/// * `RestoreCalibration` -- item 0 is channel 1, item 1 channel 2: the seven OCAL/GCAL bytes in
///   one burst write. Not Transient, so a restored calibration is written again after every
///   bring-up, after that bring-up's own internal calibration.
///
/// Tare and counts-per-unit are the application's, kept per load cell, so there is no helper
/// for them here.
template<Nau7802Detail::Ldo       LdoV      = Nau7802Detail::Ldo::v3_3,
         Nau7802Detail::Gain      GainV     = Nau7802Detail::Gain::x128,
         Nau7802Detail::Rate      RateV     = Nau7802Detail::Rate::sps80,
         Nau7802Detail::Channel   ChannelV  = Nau7802Detail::Channel::ch1,
         Nau7802Detail::FilterCap Cfilter   = Nau7802Detail::FilterCap::fitted,
         MilliVolt                Reference = Units::milliVolt(3300)>
struct Nau7802 {
    static constexpr std::string_view        Name    = "NAU7802";
    static constexpr Address7                Address = 0x2A;
    static constexpr std::array<Address7, 1> Addresses{0x2A};
    static constexpr std::size_t             RegisterBytes = 1;

    static_assert(Nau7802Detail::sps(RateV) != 0,
                  "a CRS code from the table: 10, 20, 40, 80 or 320 SPS");

    using Ldo       = Nau7802Detail::Ldo;
    using Gain      = Nau7802Detail::Gain;
    using Rate      = Nau7802Detail::Rate;
    using Channel   = Nau7802Detail::Channel;
    using CalMode   = Nau7802Detail::CalMode;
    using FilterCap = Nau7802Detail::FilterCap;
    using Config    = Nau7802Detail::Config<LdoV, GainV, RateV, ChannelV>;

    /// VREF: the LDO's voltage, or `Reference` with the LDO off.
    [[nodiscard]] static constexpr MilliVolt reference(Ldo ldo) {
        return ldo == Ldo::external
               ? Reference
               : Units::milliVolt(
                   Nau7802Detail::LdoMilliVolt[static_cast<std::size_t>(ldo) & 0x07U]);
    }

    /// What an internal calibration is given before CTRL2 is read back (not a datasheet figure).
    static constexpr std::chrono::milliseconds CalibrationWait
      = 8 * Nau7802Detail::conversionTime(RateV) + std::chrono::milliseconds{100};

    /// The analog part and the LDO after PUA (7.3: TRDY is five conversions plus 100 ms).
    static constexpr std::chrono::milliseconds AnalogSettle
      = 5 * Nau7802Detail::conversionTime(RateV) + std::chrono::milliseconds{100};

    static constexpr auto StartupDelay = std::chrono::milliseconds{1};

    static constexpr std::array Init{
      Step::write({.reg     = 0x00,
                   .payload = {0x01},
                   .delay   = std::chrono::milliseconds{1}}),   // RR (9.1 step 1)
      Step::write(
        {.reg     = 0x00,
         .payload = {0x02},
         .delay
         = std::chrono::milliseconds{2}}),   // RR clear, PUD; PUR after about 200 us (steps 2, 3)
      Step::read({.reg = 0x1F, .count = 1, .offset = 0}),   // revision
      Step::write({.reg = 0x01, .payload = {Nau7802Detail::ctrl1(LdoV, GainV)}}),
      Step::write({.reg = 0x02, .payload = {Nau7802Detail::ctrl2(ChannelV, RateV)}}),
      Step::write(
        {.reg = 0x15, .payload = {0x30}}),   // REG_CHPS 11: the chopper clock off (step 4b)
      Step::write({.reg = 0x1C, .payload = {Nau7802Detail::pgaPwr(Cfilter, ChannelV)}}),
      Step::write({.reg     = 0x00,
                   .payload = {Nau7802Detail::puCtrl(LdoV, Nau7802Detail::Cycle::running)},
                   .delay   = AnalogSettle}),
      Step::write({.reg     = 0x02,
                   .payload = {Nau7802Detail::ctrl2(ChannelV,
                                                    RateV,
                                                    CalMode::internalOffset,
                                                    Nau7802Detail::Calibration::start)},
                   .delay   = CalibrationWait}),
      Step::read({.reg = 0x00, .count = 1, .offset = 1}),   // PU_CTRL: PUR
      Step::read({.reg = 0x02, .count = 1, .offset = 2}),   // CTRL2: CALS, CAL_ERR
    };

    struct State {
        std::uint8_t revision{};      ///< REG0x1F
        std::uint8_t puCtrl{};        ///< PU_CTRL at the end of the bring-up
        std::uint8_t ctrl2{};         ///< CTRL2 after the bring-up's calibration
        bool         calError{};      ///< CAL_ERR after it (the part is rejected then)
        bool         calibrating{};   ///< CALS still set when it was read back
        Config       config{};        ///< what CTRL1 and CTRL2 hold: decode and period follow it
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.revision    = data.u8(0);
        state.puCtrl      = data.u8(1);
        state.ctrl2       = data.u8(2);
        state.calError    = (state.ctrl2 & 0x08U) != 0;
        state.calibrating = (state.ctrl2 & 0x04U) != 0;
        state.config      = Config{};
        return (state.revision & 0x0FU) == 0x0FU && (state.puCtrl & 0x08U) != 0 && !state.calError;
    }

    struct Conversion {
        static constexpr auto Period = Nau7802Detail::readPeriod(RateV);

        /// The rate CTRL2 holds now, which a Settings write changes.
        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            return Nau7802Detail::readPeriod(state.config.rate);
        }

        static constexpr std::chrono::milliseconds RetryAfter = [] {
            auto const q = std::chrono::ceil<std::chrono::milliseconds>(
              Nau7802Detail::conversionTime(RateV) / 4.0);
            return q < std::chrono::milliseconds{2} ? std::chrono::milliseconds{2} : q;
        }();

        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0}),
                                          Step::check(RetryAfter),
                                          Step::read({.reg = 0x12, .count = 3, .offset = 1})};

        /// CR, PU_CTRL bit 5.
        [[nodiscard]] static constexpr bool ready(Bytes data) { return (data.u8(0) & 0x20U) != 0; }

        struct Sample {
            std::int32_t code{};        ///< 24-bit two's complement
            Channel      channel{};     ///< CHS when it was converted
            Gain         gain{};        ///< the PGA gain it was converted at
            MilliVolt    reference{};   ///< VREF it was converted against

            /// VINxP - VINxN.
            [[nodiscard]] constexpr MicroVolt voltage() const {
                return Nau7802Detail::toVoltage(code, reference, gain);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes        data,
                                                     State const& state) {
            return {.code      = Bytes::signExtend(data.be24(1), 24),
                    .channel   = state.config.channel,
                    .gain      = state.config.gain,
                    .reference = reference(state.config.ldo)};
        }
    };

    /// CTRL2 and both channels' calibration registers, 0x02..0x10, on request.
    struct Calibration {
        static constexpr std::array Steps{Step::read({.reg = 0x02, .count = 15, .offset = 0})};

        struct Sample {
            bool                                             calibrating{};   ///< CALS
            bool                                             calError{};      ///< CAL_ERR
            std::array<Nau7802Detail::ChannelCalibration, 2> channels{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample s{};
            s.calibrating = (data.u8(0) & 0x04U) != 0;
            s.calError    = (data.u8(0) & 0x08U) != 0;
            s.channels[0] = {.offset = Bytes::signExtend(data.be24(1), 24), .gain = data.be32(4)};
            s.channels[1] = {.offset = Bytes::signExtend(data.be24(8), 24), .gain = data.be32(11)};
            return s;
        }
    };

    struct Settings {
        using Value                        = Config;
        static constexpr std::size_t Bytes = 4;

        /// CTRL1, CTRL2, PGA_PWR, then CS cleared and set: 11.1's CS synchronises conversion to
        /// its rising edge, so the conversions from here on are at the new settings; six of
        /// them go by before anything else of the part runs, for the shared filter to settle.
        [[nodiscard]] static constexpr std::array<Step,
                                                  5>
        encode(Value const&         value,
               std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(Nau7802Detail::ctrl1(value.ldo, value.gain));
            buffer[1] = static_cast<std::byte>(Nau7802Detail::ctrl2(value.channel, value.rate));
            buffer[2] = static_cast<std::byte>(Nau7802Detail::pgaPwr(Cfilter, value.channel));
            buffer[3] = static_cast<std::byte>(
              Nau7802Detail::puCtrl(value.ldo, Nau7802Detail::Cycle::running));
            return {
              Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 1}),
              Step::writeBuffer({.reg = 0x02, .offset = 1, .count = 1}),
              Step::writeBuffer({.reg = 0x1C, .offset = 2, .count = 1}),
              Step::write(
                {.reg     = 0x00,
                 .payload = {Nau7802Detail::puCtrl(value.ldo, Nau7802Detail::Cycle::stopped)}}),
              Step::writeBuffer({.reg    = 0x00,
                                 .offset = 3,
                                 .count  = 1,
                                 .delay  = 6 * Nau7802Detail::conversionTime(value.rate)}),
            };
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.config = value;
        }
    };

    /// What Calibrate writes: the mode, and the rate and channel CTRL2 keeps meanwhile.
    struct CalibrationRun {
        CalMode mode{CalMode::internalOffset};
        Rate    rate{RateV};
        Channel channel{ChannelV};
    };

    [[nodiscard]] static constexpr CalibrationRun calibration(Config const& config,
                                                              CalMode       mode
                                                              = CalMode::internalOffset) {
        return {.mode = mode, .rate = config.rate, .channel = config.channel};
    }

    struct Calibrate {
        using Value                              = CalibrationRun;
        static constexpr std::size_t Bytes       = 1;
        static constexpr bool        Transient   = true;
        static constexpr auto        VerifyDelay = CalibrationWait;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0]
              = static_cast<std::byte>(Nau7802Detail::ctrl2(value.channel,
                                                            value.rate,
                                                            value.mode,
                                                            Nau7802Detail::Calibration::start));
            return Step::writeBuffer({.reg = 0x02, .offset = 0, .count = 1});
        }

        /// Done: CALS clear, CAL_ERR clear, channel, rate and mode as written.
        [[nodiscard]] static constexpr bool verify(Kvasir::I2C::Bytes written,
                                                   Kvasir::I2C::Bytes readBack) {
            auto const w = written.u8(0);
            auto const r = readBack.u8(0);
            return (r & 0x0CU) == 0 && (r & 0xF3U) == (w & 0xF3U);
        }
    };

    /// OCAL and GCAL of one channel (item 0: 0x03..0x09, item 1: 0x0A..0x10) in one burst.
    struct RestoreCalibration {
        using Value                        = Nau7802Detail::ChannelCalibration;
        static constexpr std::size_t Items = 2;
        static constexpr std::size_t Bytes = 7;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            auto const o = static_cast<std::uint32_t>(value.offset);
            buffer[0]    = static_cast<std::byte>((o >> 16) & 0xFFU);
            buffer[1]    = static_cast<std::byte>((o >> 8) & 0xFFU);
            buffer[2]    = static_cast<std::byte>(o & 0xFFU);
            buffer[3]    = static_cast<std::byte>(value.gain >> 24);
            buffer[4]    = static_cast<std::byte>((value.gain >> 16) & 0xFFU);
            buffer[5]    = static_cast<std::byte>((value.gain >> 8) & 0xFFU);
            buffer[6]    = static_cast<std::byte>(value.gain & 0xFFU);
            return Step::writeBuffer({.reg = item == 0 ? 0x03 : 0x0A, .offset = 0, .count = 7});
        }
    };

    using Primary = Conversion;
    using Reads   = List<Conversion, Calibration>;
    using Writes  = List<Settings, Calibrate, RestoreCalibration>;
};

}   // namespace Kvasir::I2C::Chips
