#pragma once

#include "../Device.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Holtek HT16K33 LED matrix / 16-segment driver (HT16K33 rev 1.10; the Adafruit LED backpacks). No
/// registers: a command is one byte, the display RAM is written as address 0x00 followed by up to
/// 16 bytes (8 little-endian words, one per COM0..COM7, whose 16 bits drive ROW0..ROW15). Bring-up:
/// system setup 0x21 (oscillator on), ROW/INT 0xA0, display setup 0x81 (on, no blink), dimming 0xEF
/// (16/16). Write-only: `set<Display>(rows)`, `set<Brightness>(0..15)`, `set<Blink>(rate)`.
/// 0x70..0x77 by A0..A2.
struct Ht16k33 {
    static constexpr std::string_view Name          = "HT16K33";
    static constexpr Address7         Address       = 0x70;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array<Address7, 8>
      Addresses{0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77};

    static constexpr std::array Init{
      Step::command({.payload = {0x21}, .delay = std::chrono::milliseconds{1}}),
      Step::command({.payload = {0xA0}}),
      Step::command({.payload = {0x81}}),
      Step::command({.payload = {0xEF}}),
    };

    static constexpr std::size_t RowCount = 8;

    /// The eight 16-bit words of the display RAM: one per COM0..COM7, bit n driving ROWn.
    using Rows = std::array<std::uint16_t, RowCount>;

    struct Display {
        using Value = Rows;
        /// The RAM address byte, then two bytes a row.
        static constexpr std::size_t Bytes = 1 + 2 * RowCount;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{0x00};
            for(std::size_t i = 0; i < RowCount; ++i) {
                buffer[1 + 2 * i] = static_cast<std::byte>(value[i] & 0xFF);
                buffer[2 + 2 * i] = static_cast<std::byte>(value[i] >> 8);
            }
            return Step::commandBuffer({.offset = 0, .count = Bytes});
        }
    };

    struct Brightness {
        using Value                          = std::uint8_t;   ///< 0..15
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = 15;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(0xE0U | (static_cast<unsigned>(value) & 0x0FU));
            return Step::commandBuffer({.offset = 0, .count = 1});
        }
    };

    enum class BlinkRate : std::uint8_t { off = 0, hz2 = 1, hz1 = 2, hz0_5 = 3 };
    static constexpr std::size_t BlinkRateCount = 4;
    static_assert(static_cast<std::size_t>(BlinkRate::hz0_5) + 1 == BlinkRateCount,
                  "hz0_5 is the last rate");

    struct Blink {
        using Value                          = BlinkRate;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = BlinkRate::off;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(0x81U | (static_cast<unsigned>(value) << 1));
            return Step::commandBuffer({.offset = 0, .count = 1});
        }
    };

    using Writes = List<Display, Brightness, Blink>;

    /// Seven-segment glyphs (a = bit 0 .. g = bit 6, dp = bit 7) for 0..9, A..F, the
    /// Adafruit 4-digit backpack's layout: digits in rows 0, 1, 3, 4; row 2 bit 1 is the
    /// colon.
    static constexpr std::array<std::uint8_t, 16> SevenSegment{0x3F,
                                                               0x06,
                                                               0x5B,
                                                               0x4F,
                                                               0x66,
                                                               0x6D,
                                                               0x7D,
                                                               0x07,
                                                               0x7F,
                                                               0x6F,
                                                               0x77,
                                                               0x7C,
                                                               0x39,
                                                               0x5E,
                                                               0x79,
                                                               0x71};

    static constexpr std::array<std::size_t, 4> FourDigitRows{0, 1, 3, 4};
    static constexpr std::size_t                ColonRow = 2;
    static constexpr std::uint16_t              ColonBit = 0x02;

    /// Four hex digits (and the colon) on a 4-digit 7-segment backpack; a digit of 0xFF is blank.
    [[nodiscard]] static constexpr Rows fourDigits(std::array<std::uint8_t,
                                                              4> digits,
                                                   bool          colon = false,
                                                   std::uint8_t  dots  = 0) {
        Rows r{};
        for(std::size_t i = 0; i < 4; ++i) {
            std::uint16_t g = digits[i] < 16 ? SevenSegment[digits[i]] : 0;
            if((dots >> i) & 1U) { g |= 0x80U; }
            r[FourDigitRows[i]] = g;
        }
        if(colon) { r[ColonRow] = ColonBit; }
        return r;
    }
};

}   // namespace Kvasir::I2C::Chips
