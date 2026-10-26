#pragma once

#include "../../Engine/Description.hpp"
#include "../Address.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Opt3001Detail {
    /// CT, configuration bit 11: how long one conversion integrates. 800 ms is the part's
    /// low-noise setting and what a room-light reading wants; 100 ms is for following a
    /// light that moves.
    enum class ConversionTime : std::uint8_t { ms100 = 0, ms800 = 1 };

    [[nodiscard]] constexpr std::chrono::milliseconds conversionTime(ConversionTime ct) {
        return ct == ConversionTime::ms800 ? std::chrono::milliseconds{800}
                                           : std::chrono::milliseconds{100};
    }

    /// One conversion plus a margin: the result register is read without consulting CRF
    /// (configuration bit 7), so the read has to sit far enough behind the conversion that
    /// it never laps it. Re-reading the same conversion would cost a sample, not correctness
    /// -- the register holds the last completed result either way.
    [[nodiscard]] constexpr std::chrono::milliseconds readPeriod(ConversionTime ct) {
        return conversionTime(ct) + std::chrono::milliseconds{50};
    }
}   // namespace Opt3001Detail

/// TI OPT3001 ambient light sensor (datasheet SBOS681C, i2c/OPT3001.md). One-byte register
/// pointer, 16-bit big-endian registers: 00h Result, 01h Configuration, 02h/03h the limit
/// registers, 7Eh Manufacturer ID (5449h, "TI"), 7Fh Device ID (3001h) (7.6.1.1, Table 14 and
/// 15, md l.1174-1222). Bring-up reads both IDs and writes Configuration = automatic
/// full-scale (RN 1100b), the conversion time this template was given (CT), continuous
/// conversion (M 11b), every other writable bit at its reset value (Table 10, md l.993-1042).
/// The register resets to C810h (7.6.1.1.2, md l.975): RN, L, POL, ME and FC keep their reset
/// values - L (bit 4, md l.1042) stays 1, latched window-style comparison; it only decides how
/// the INT pin and the FH/FL flags report, and neither is used here.
///
/// The result is not a count but a float in a word: E 15:12 an exponent, R 11:0 a mantissa,
/// and lux = 0.01 x 2^E x R (Equation 3, md l.938). That makes 10 mlx the LSB at E 0, so the
/// whole range is exact in MilliLux -- full scale, E 1011b with R 0FFFh, is 83 865 600 mlx
/// (83 865.6 lx, Table 9, md l.967) and still inside the uint32 MilliLux carries.
///
/// The part has no conversion-in-progress reading to reject: every Result word is a
/// completed conversion, the one before this if the read laps the conversion. `saturated()`
/// marks the top of the range, which in automatic full-scale means the light is off the
/// part's scale rather than merely off the current range's.
/// ADDR to GND is 0x44, to VDD 0x45, to SDA 0x46, to SCL 0x47 (Table 1, md l.560-567).
template<Opt3001Detail::ConversionTime CT = Opt3001Detail::ConversionTime::ms800>
struct Opt3001 {
    using ConversionTime = Opt3001Detail::ConversionTime;

    static constexpr std::string_view Name = "OPT3001";
    /// SBOS681C Table 14 "The manufacturer ID reads 5449h", Table 15 "The device ID reads 3001h"
    /// (7.6.1.1.5/6, md l.1196, l.1222). The engine reads both before anything is written: a part
    /// with another identity at the address gets nothing.
    static constexpr std::array Identity{
      Engine::RegisterCheck{"manufacturer-id", 0x7E, 2, true, 0xFFFF, 0x5449},
      Engine::RegisterCheck{      "device-id", 0x7F, 2, true, 0xFFFF, 0x3001},
    };
    static constexpr Address7    Address       = 0x44;
    static constexpr auto        Addresses     = addressRun<0x44, 4>();
    static constexpr std::size_t RegisterBytes = 1;

    /// Power-on to the first transaction. The part is ready well inside this; the figure is
    /// what a power-gated sensor -- the enable pin as the Device's Reset line -- needs for
    /// its rail to come up and settle.
    static constexpr auto StartupDelay = std::chrono::milliseconds{20};

    /// Held low long enough for the rail to actually fall, when the "reset" is a power gate.
    static constexpr auto ResetLow    = std::chrono::milliseconds{10};
    static constexpr auto ResetSettle = std::chrono::milliseconds{10};

    /// RN 1100b automatic full-scale, CT as given, M 11b continuous (Table 10, md l.993-1042).
    /// Below bit 9 the reset state (C810h): L (bit 4) 1, POL, ME and FC 0; bits 8:5 are
    /// read-only flags.
    static constexpr std::uint16_t LatchedWindow = 0x0010U;
    static constexpr std::uint16_t Conf          = static_cast<std::uint16_t>(
      0xC000U | (static_cast<std::uint16_t>(CT) << 11U) | (0x3U << 9U) | LatchedWindow);

    // clang-format off
    static constexpr std::array Init{
      Engine::Step::write(
        {.reg     = 0x01,
         .payload = {static_cast<std::uint8_t>(Conf >> 8), static_cast<std::uint8_t>(Conf & 0xFF)},
         .delay   = Opt3001Detail::conversionTime(CT)}),
    };
    // clang-format on

    struct State {
        std::uint16_t manufacturerId{};
        std::uint16_t deviceId{};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.manufacturerId = static_cast<std::uint16_t>(ids[0]);
        state.deviceId       = static_cast<std::uint16_t>(ids[1]);
    }

    struct Light {
        static constexpr auto       Period = Opt3001Detail::readPeriod(CT);
        static constexpr std::array Steps{
          Engine::Step::read({.reg = 0x00, .count = 2, .offset = 0})};

        struct Sample {
            std::uint16_t raw{};   ///< the Result register, exponent and mantissa together

            [[nodiscard]] constexpr std::uint8_t exponent() const {
                return static_cast<std::uint8_t>(raw >> 12U);
            }

            [[nodiscard]] constexpr std::uint16_t mantissa() const {
                return static_cast<std::uint16_t>(raw & 0x0FFFU);
            }

            /// 0.01 x 2^E x R lux, which is 10 x 2^E x R millilux exactly.
            [[nodiscard]] constexpr MilliLux lux() const {
                return Units::milliLux((static_cast<std::uint32_t>(mantissa()) << exponent())
                                       * 10U);
            }

            /// The top of the part's automatic full-scale range: the light is at least this.
            [[nodiscard]] constexpr bool saturated() const { return raw == 0xBFFF; }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.be16(0)}; }
    };

    using Reads = Engine::List<Light>;
};

}   // namespace Kvasir::I2C::Chips
