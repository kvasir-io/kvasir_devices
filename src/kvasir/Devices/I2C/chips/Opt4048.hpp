#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Opt4048Detail {
    /// Configuration CONVERSION_TIME (9:6): 600 us up to 800 ms per channel.
    enum class ConversionTime : std::uint8_t {
        us600  = 0,
        ms1    = 1,
        ms1_8  = 2,
        ms3_4  = 3,
        ms6_5  = 4,
        ms12_7 = 5,
        ms25   = 6,
        ms50   = 7,
        ms100  = 8,
        ms200  = 9,
        ms400  = 10,
        ms800  = 11,
    };

    /// Configuration RANGE (13:10): 0..6 the fixed full scales (2.2 klux up to 144 klux), 12
    /// auto-range.
    enum class Range : std::uint8_t {
        klux2_2 = 0,
        klux4_5 = 1,
        klux9   = 2,
        klux18  = 3,
        klux36  = 4,
        klux72  = 5,
        klux144 = 6,
        auto_   = 12,
    };

    /// Per channel, rounded up, so a whole measurement is four of these.
    inline constexpr std::array<std::chrono::milliseconds, 12> ConversionTimes{
      std::chrono::milliseconds{1},
      std::chrono::milliseconds{1},
      std::chrono::milliseconds{2},
      std::chrono::milliseconds{4},
      std::chrono::milliseconds{7},
      std::chrono::milliseconds{13},
      std::chrono::milliseconds{25},
      std::chrono::milliseconds{50},
      std::chrono::milliseconds{100},
      std::chrono::milliseconds{200},
      std::chrono::milliseconds{400},
      std::chrono::milliseconds{800}};

    [[nodiscard]] constexpr std::chrono::milliseconds conversionTime(std::uint8_t code) {
        return code < ConversionTimes.size() ? ConversionTimes[code] : ConversionTimes[8];
    }

    /// The datasheet's 4-bit CRC over one channel's exponent E[3:0], mantissa R[19:0] and
    /// counter C[3:0] (Table "Result register CRC"; Adafruit's driver computes the same):
    ///   bit 0: every bit of E, R and C
    ///   bit 1: C1 C3, R1 R3 .. R19, E1 E3
    ///   bit 2: C3, R3 R7 R11 R15 R19, E3
    ///   bit 3: R3 R11 R19
    [[nodiscard]] constexpr std::uint8_t crc(std::uint8_t  exponent,
                                             std::uint32_t mantissa,
                                             std::uint8_t  counter) {
        auto const parity
          = [](std::uint32_t v) { return static_cast<unsigned>(std::popcount(v) & 1); };
        auto const x0 = parity(exponent) ^ parity(mantissa) ^ parity(counter);
        auto const x1
          = parity(counter & 0x0AU) ^ parity(mantissa & 0xAAAAAU) ^ parity(exponent & 0x0AU);
        auto const x2
          = parity(counter & 0x08U) ^ parity(mantissa & 0x88888U) ^ parity(exponent & 0x08U);
        auto const x3 = parity(mantissa & 0x80808U);
        return static_cast<std::uint8_t>((x3 << 3) | (x2 << 2) | (x1 << 1) | x0);
    }
}   // namespace Opt4048Detail

/// Texas Instruments OPT4048 tri-stimulus XYZ colour sensor (SBOSA84). One-byte pointer,
/// 16-bit big-endian registers. Each of the four channels -- CH0 is X, CH1 Y, CH2 Z and
/// CH3 the wide channel -- occupies a register pair: the even one holds EXPONENT_CHx in
/// bits 15:12 and RESULT_MSB_CHx in 11:0, the odd one RESULT_LSB_CHx in 15:8, COUNTER_CHx
/// in 7:4 and CRC_CHx in 3:0. Then THRESHOLD_L 0x08, THRESHOLD_H 0x09, configuration 0x0A,
/// interrupt configuration 0x0B (I2C_BURST bit 0, set out of reset), flags 0x0C and device
/// id 0x11: DIDL in 13:12 and DIDH in 11:0. The register figure gives DIDL 0h and DIDH 821h,
/// so the word reads 0x0821 (the section heading's "reset = 820h" disagrees with its own
/// fields); bring-up compares DIDH only, so a DIDL revision does not lock the part out.
/// 0x2084 -- what SparkFun's library compares against -- is 0x0821 shifted left by two bits,
/// and is not the id; Adafruit's compares 0x0821.
///
/// A channel's reading is semi-logarithmic and is linearised as
///
///     MANTISSA = (RESULT_MSB << 8) | RESULT_LSB     (20 bits)
///     ADC_CODE = MANTISSA << EXPONENT               (28 bits)
///
/// and lux is `ADC_CODE_CH1 x 2.15e-3` -- the one conversion the datasheet gives in text.
/// The 3x4 matrix for CIE X, Y and u'v' coordinates is a *figure* in section 9.2.4 and TI
/// says it should be re-fitted per application (cover glass, illuminant), so it is
/// deliberately not baked in here: the linearised per-channel codes are what a caller needs
/// to apply their own matrix, and `lux()` covers the common case.
///
/// With I2C_BURST set the pointer auto-increments, so the eight result registers come out of
/// one sixteen-byte read. It is set out of reset, but a part whose earlier firmware cleared it
/// would answer the burst with channel 0 four times over, which passes every CRC, so bring-up
/// writes register 0Bh with its reset value 8011h (8.4.1.12: bits 15:7 must be 128, INT_CFG
/// and INT_DIR at their defaults, I2C_BURST 1). Each channel's CRC is checked and a frame
/// with a bad one is rejected, and so is one with an exponent above 8, the largest the part
/// reports (8.3.4.5). A frame whose four counters *and* readings equal the previous
/// sample's is the same conversion again and is Outcome::unchanged(). The counters alone do not
/// say so: they are four bits and wrap (8.3.3.1), and at a short conversion time one poll period
/// spans many conversions, so a new frame lands on the same counters one time in sixteen.
///
/// Configuration 0x0A: QWAKE 15, RANGE 13:10 (12 selects auto-range), CONVERSION_TIME 9:6,
/// OPERATING_MODE 5:4 (3 is continuous), LATCH 3, INT_POL 2, FAULT_COUNT 1:0. The template
/// values are the Initial of the Config write group; the Colour period follows the
/// conversion time written. LATCH is written 0 (its reset value is 1): it only decides how
/// the INT pin and the threshold flags behave, which this description does not use.
/// 0x44 by default, or 0x45..0x47 by the ADDR pin.
template<Opt4048Detail::ConversionTime ConversionTime = Opt4048Detail::ConversionTime::ms100,
         Opt4048Detail::Range          Range          = Opt4048Detail::Range::auto_>
struct Opt4048 {
    static constexpr std::string_view Name = "OPT4048";
    /// TI OPT4048. OPT4048.md:963 and :1452..1453, register 11h: DIDH 821h in 11:0 (DIDL in 13:12).
    static constexpr std::array Identity{
      RegisterCheck{"device-id", 0x11, 2, true, 0x0FFF, 0x0821},
    };
    static constexpr Address7    Address       = 0x44;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 4> Addresses{0x44, 0x45, 0x46, 0x47};

    static constexpr std::size_t Channels = 4;

    static constexpr std::uint8_t ConversionCode = static_cast<std::uint8_t>(ConversionTime);
    static constexpr std::uint8_t RangeCode      = static_cast<std::uint8_t>(Range);

    /// The range, the chosen conversion time, continuous mode.
    static constexpr std::uint16_t Configuration
      = static_cast<std::uint16_t>((RangeCode << 10) | (ConversionCode << 6) | (3U << 4));

    static constexpr std::array<std::chrono::milliseconds, 12> ConversionTimes
      = Opt4048Detail::ConversionTimes;

    /// Four channels at the conversion time, plus headroom.
    [[nodiscard]] static constexpr std::chrono::milliseconds
    readPeriod(std::uint8_t conversionCode) {
        return 4 * Opt4048Detail::conversionTime(conversionCode) + std::chrono::milliseconds{50};
    }

    static constexpr std::uint16_t DeviceId = 0x0821;

    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    static constexpr std::array Init{
      Step::write({.reg = 0x0B, .payload = {0x80, 0x11}}
      ), // I2C_BURST, the rest at reset
      Step::write({.reg     = 0x0A,
                   .payload = {static_cast<std::uint8_t>(Configuration >> 8),
                               static_cast<std::uint8_t>(Configuration & 0xFF)},
                   .delay   = ConversionTimes[ConversionCode] * 4}
      )
    };

    struct State {
        std::uint16_t deviceId{};
        std::uint8_t  conversionTime{ConversionCode};   ///< CONVERSION_TIME as written now
        std::uint8_t  range{RangeCode};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId       = static_cast<std::uint16_t>(ids[0]);
        state.conversionTime = ConversionCode;
        state.range          = RangeCode;
    }

    struct Colour {
        static constexpr auto Period = readPeriod(ConversionCode);

        /// The conversion time the configuration holds now, which a Config write changes.
        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            return readPeriod(state.conversionTime);
        }

        /// The eight result registers in one burst (I2C_BURST is set out of reset).
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 16, .offset = 0})};

        struct Sample {
            /// Linearised 28-bit codes: X, Y, Z and the wide channel.
            std::array<std::uint32_t, Channels> code{};
            std::array<std::uint8_t, Channels>  exponent{};
            std::array<std::uint8_t, Channels>  counter{};

            /// lux = ADC_CODE_CH1 x 2.15e-3.
            [[nodiscard]] constexpr MilliLux lux() const {
                return Units::milliLux(static_cast<std::uint64_t>(code[1]) * 215ULL / 100ULL);
            }

            [[nodiscard]] constexpr std::uint32_t x() const { return code[0]; }

            [[nodiscard]] constexpr std::uint32_t y() const { return code[1]; }

            [[nodiscard]] constexpr std::uint32_t z() const { return code[2]; }

            [[nodiscard]] constexpr std::uint32_t wide() const { return code[3]; }
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes         data,
                                                              Sample const& previous) {
            Sample sample{};
            bool   same = true;
            for(std::size_t c = 0; c < Channels; ++c) {
                auto const hi      = data.be16(4 * c);
                auto const lo      = data.be16(4 * c + 2);
                auto const exp     = static_cast<std::uint8_t>(hi >> 12);
                auto const man     = (static_cast<std::uint32_t>(hi & 0x0FFFU) << 8)
                                   | static_cast<std::uint32_t>(lo >> 8);
                auto const counter = static_cast<std::uint8_t>((lo >> 4) & 0x0FU);
                if(exp > 8 || Opt4048Detail::crc(exp, man, counter) != (lo & 0x0FU)) {
                    return Outcome<Sample>::reject();
                }
                sample.exponent[c] = exp;
                sample.counter[c]  = counter;
                sample.code[c]     = man << exp;
                same = same && counter == previous.counter[c] && exp == previous.exponent[c]
                    && sample.code[c] == previous.code[c];
            }
            if(same) { return Outcome<Sample>::unchanged(); }
            return Outcome<Sample>::ok(sample);
        }
    };

    /// The configuration register, put back after every bring-up; a new conversion time is
    /// the period Colour is read at from the write's completion on.
    struct Config {
        using Value                          = std::uint16_t;
        static constexpr std::size_t Bytes   = 2;
        static constexpr Value       Initial = Configuration;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            putBe16(buffer, 0, value);
            return Step::writeBuffer({.reg = 0x0A, .offset = 0, .count = 2});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.conversionTime = static_cast<std::uint8_t>((value >> 6U) & 0x0FU);
            state.range          = static_cast<std::uint8_t>((value >> 10U) & 0x0FU);
        }
    };

    using Reads  = List<Colour>;
    using Writes = List<Config>;
};

static_assert(Opt4048Detail::crc(0,
                                 0,
                                 0)
                == 0,
              "an empty result has an empty CRC");
static_assert(Opt4048Detail::crc(0,
                                 1,
                                 0)
                == 1,
              "R0 alone: only the all-bits parity");
static_assert(Opt4048Detail::crc(0,
                                 0x8,
                                 0)
                == 0x0F,
              "R3 is in every one of the four parities");

}   // namespace Kvasir::I2C::Chips
