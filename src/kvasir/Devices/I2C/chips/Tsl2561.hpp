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

namespace Tsl2561Detail {
    /// TIMING INTEG (1:0): 13.7, 101 or 402 ms. 3 is manual and not used here.
    enum class Integration : std::uint8_t { ms13_7 = 0, ms101 = 1, ms402 = 2 };

    /// TIMING GAIN (bit 4): 1x or 16x.
    enum class Gain : std::uint8_t { x1 = 0, x16 = 1 };

    inline constexpr std::array<std::chrono::milliseconds, 3> IntegrationTimes{
      std::chrono::milliseconds{14},
      std::chrono::milliseconds{101},
      std::chrono::milliseconds{402}};

    /// The datasheet's CH_SCALE per integration time (x 2^10): 322/11, 322/81 and 1.
    inline constexpr std::array<std::uint32_t, 3> ChScale{0x7517, 0x0FE7, 1U << 10};

    /// The full-scale count per integration time (Operating Characteristics, note 6: one count
    /// per two oscillator periods less a 2-count offset): 5047 at 13.7 ms, 37177 at 101 ms,
    /// and the 16-bit register's 65535 at 402 ms.
    inline constexpr std::array<std::uint16_t, 3> Saturation{5047, 37177, 65535};

    /// {K, B, M} per ratio band, in the datasheet's fixed point (RATIO_SCALE 9, LUX_SCALE 14).
    using Bands = std::array<std::array<std::uint32_t, 3>, 8>;

    /// The T, FN and CL packages.
    inline constexpr Bands BandsT{
      {
       {0x0040, 0x01F2, 0x01BE},
       {0x0080, 0x0214, 0x02D1},
       {0x00C0, 0x023F, 0x037B},
       {0x0100, 0x0270, 0x03FE},
       {0x0138, 0x016F, 0x01FC},
       {0x019A, 0x00D2, 0x00FB},
       {0x029A, 0x0018, 0x0012},
       {0xFFFF, 0x0000, 0x0000},
       }
    };

    /// The CS package: the datasheet calculation's second coefficient set. Its header comment
    /// gives B6C as 0.00157 and its #define as 0.0157 (0x0101); 0x0101 is the one the code
    /// uses, and Linux tsl2563.c has the same.
    inline constexpr Bands BandsCs{
      {
       {0x0043, 0x0204, 0x01AD},
       {0x0085, 0x0228, 0x02C1},
       {0x00C8, 0x0253, 0x0363},
       {0x010A, 0x0282, 0x03DF},
       {0x014D, 0x0177, 0x01DD},
       {0x019A, 0x0101, 0x0127},
       {0x029A, 0x0037, 0x002B},
       {0xFFFF, 0x0000, 0x0000},
       }
    };
}   // namespace Tsl2561Detail

/// ams / TAOS TSL2561 (TAOS059N). Command register: bit 7 set, WORD bit 5 for a two-byte
/// read, the register in bits 3:0. Bring-up: CONTROL 0x00 = 0x03 (power up), ID 0x0A part
/// number in bits 7:4 (0x0 TSL2560CS, 0x1 TSL2561CS, 0x4 TSL2560T/FN/CL, 0x5 TSL2561T/FN/CL),
/// TIMING 0x01 = gain and integration (0x02: 402 ms, gain 1x by default). Data: channel 0
/// (visible + IR) and channel 1 (IR) as words at 0x0C and 0x0E, little-endian. `lux()` is
/// the datasheet's integer calculation ("Simplified Lux Calculation"), which scales the
/// counts to the 402 ms / 16x gain its coefficients assume and picks the coefficient set by
/// the package the ID read found -- the CS package has its own. A channel at its integration
/// time's clipping level is `saturated()`, and lux() is 0 there.
///
/// Gain and integration time are the Initial of the Timing write group, so they can be
/// changed at run time; each Sample carries the pair it was taken under.
/// 0x39 (ADDR floating), 0x29 (GND), 0x49 (VDD).
template<Tsl2561Detail::Gain        Gain            = Tsl2561Detail::Gain::x1,
         Tsl2561Detail::Integration IntegrationTime = Tsl2561Detail::Integration::ms402>
struct Tsl2561X {
    static constexpr std::string_view        Name    = "TSL2561";
    static constexpr Address7                Address = 0x39;
    static constexpr std::array<Address7, 3> Addresses{0x29, 0x39, 0x49};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::uint8_t Cmd  = 0x80;
    static constexpr std::uint8_t Word = 0xA0;

    static constexpr std::uint8_t GainCode        = static_cast<std::uint8_t>(Gain);
    static constexpr std::uint8_t IntegrationCode = static_cast<std::uint8_t>(IntegrationTime);
    static constexpr std::uint8_t Timing
      = static_cast<std::uint8_t>((GainCode << 4) | IntegrationCode);
    static constexpr std::chrono::milliseconds Integrating
      = Tsl2561Detail::IntegrationTimes[IntegrationCode];

    static constexpr std::array Init{
      Step::write({.reg = Cmd | 0x00, .payload = {0x03}}),
      Step::read({.reg = Cmd | 0x0A, .count = 1, .offset = 0}),
      Step::write({.reg     = Cmd | 0x01,
                   .payload = {Timing},
                   .delay   = Integrating + std::chrono::milliseconds{10}}),
    };

    struct State {
        std::uint8_t deviceId{};
        bool         csPackage{};   ///< ID part number 0x0 or 0x1: the CS coefficients apply
        std::uint8_t gain{GainCode};
        std::uint8_t integration{IntegrationCode};
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.deviceId    = data.u8(0);
        state.gain        = GainCode;
        state.integration = IntegrationCode;
        auto const part   = static_cast<std::uint8_t>(state.deviceId >> 4);
        state.csPackage   = part == 0x0 || part == 0x1;
        return part == 0x0 || part == 0x1 || part == 0x4 || part == 0x5;
    }

    struct Light {
        static constexpr auto       Period = std::chrono::milliseconds{500};
        static constexpr std::array Steps{
          Step::read({.reg = Word | 0x0C, .count = 2, .offset = 0}),
          Step::read({.reg = Word | 0x0E, .count = 2, .offset = 2})};

        struct Sample {
            std::uint16_t full{};          ///< channel 0: visible + infrared
            std::uint16_t infrared{};      ///< channel 1
            std::uint8_t  gain{};          ///< TIMING GAIN the frame was taken at
            std::uint8_t  integration{};   ///< TIMING INTEG
            bool          csPackage{};     ///< which coefficient set lux() uses

            /// A channel at the clipping level of its integration time.
            [[nodiscard]] constexpr bool saturated() const {
                auto const limit = Tsl2561Detail::Saturation[integration % 3];
                return full >= limit || infrared >= limit;
            }

            /// The datasheet's fixed point: LUX_SCALE 14, RATIO_SCALE 9, CH_SCALE 10, the
            /// integration time's channel scale and x16 for 1x gain.
            [[nodiscard]] constexpr MilliLux lux() const {
                if(saturated()) { return Units::milliLux(0); }
                std::uint32_t chScale = Tsl2561Detail::ChScale[integration % 3];
                if(gain == 0) { chScale <<= 4; }
                auto const ch0 = (std::uint64_t{full} * chScale) >> 10;
                auto const ch1 = (std::uint64_t{infrared} * chScale) >> 10;
                if(ch0 == 0) { return Units::milliLux(0); }
                auto const  ratio = (((ch1 << 10) / ch0) + 1) >> 1;
                auto const& bands = csPackage ? Tsl2561Detail::BandsCs : Tsl2561Detail::BandsT;
                std::size_t band  = 0;
                while(band + 1 < bands.size() && ratio > bands[band][0]) { ++band; }
                auto const plus  = ch0 * bands[band][1];
                auto const minus = ch1 * bands[band][2];
                auto const temp  = plus > minus ? plus - minus : 0;
                return Units::milliLux((temp * 1000U + (1U << 13)) >> 14);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes        data,
                                                     State const& state) {
            return {data.le16(0), data.le16(2), state.gain, state.integration, state.csPackage};
        }
    };

    /// TIMING: GAIN in bit 4 and INTEG in 1:0, changeable at run time.
    struct TimingSetting {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Timing;

        /// The write is followed by two integration cycles at the new setting, the part's other
        /// groups off the wire meanwhile (Linux tsl2563.c waits the same): the ADC results
        /// only move at the end of a cycle, so a frame read sooner holds counts integrated
        /// under the old gain and time, which applied() has already replaced.
        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer(
              {.reg    = Cmd | 0x01,
               .offset = 0,
               .count  = 1,
               .delay  = 2 * Tsl2561Detail::IntegrationTimes[(value & 0x03U) % 3]
                       + std::chrono::milliseconds{10}});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gain        = static_cast<std::uint8_t>((value >> 4U) & 0x01U);
            state.integration = static_cast<std::uint8_t>(value & 0x03U);
        }
    };

    using Reads  = List<Light>;
    using Writes = List<TimingSetting>;
};

/// The part at 402 ms and gain 1x.
using Tsl2561 = Tsl2561X<>;

}   // namespace Kvasir::I2C::Chips
