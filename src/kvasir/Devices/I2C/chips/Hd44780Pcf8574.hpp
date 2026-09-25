#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Hd44780Detail {
    /// Port pin P3: the backlight transistor.
    enum class Backlight : std::uint8_t { off, on };
}   // namespace Hd44780Detail

/// A Hitachi HD44780 character LCD (16 x 2, 20 x 4) behind the usual PCF8574 backpack
/// (P0 RS, P1 R/W, P2 E, P3 backlight, P4..P7 D4..D7). No registers, and the LCD never
/// answers: every byte on the bus is one state of the eight port pins, and every LCD byte
/// is four of them (high nibble with E up, E down, low nibble with E up, E down; HD44780
/// data sheet, 4-bit interface, figure 24). Bring-up is the data sheet's initialisation by
/// instruction (three 0x3, then 0x2 for 4-bit mode), function set 0x28 (two lines,
/// 5 x 8), display off, clear (3 ms: Table 6 gives Clear display no time, only Return home's
/// 1.52 ms at the nominal 270 kHz oscillator -- about 2.2 ms at its 190 kHz low end -- which
/// clear includes; Linux hd44780_common.c waits 2 ms), entry mode 0x06, display on 0x0C.
///
/// `set<Line>(row, text)` writes one row (the DDRAM address, then Columns characters);
/// `set<Control>(v)` a display-control byte (Control::on / off / cursor / blink).
///
/// The backlight is port pin P3, so *every* byte on the wire says whether it is on: a
/// description cannot write a row without also writing the backlight, and `encode()` sees
/// only the value it is given. So the backlight flag is part of both values -- `Text`
/// (Line) and `Control::Value` -- and defaults to on in each. An application that dims the
/// backlight passes `Backlight::off` to `Text::of(s, Backlight::off)` and
/// `Control::Value{Control::on.mode, Backlight::off}` alike, and keeps doing so: a row written with the default afterwards turns the
/// light back on. To switch the light without redrawing, write Control with the flag
/// changed and the same mode. There is no separate backlight group: the next Line write
/// would silently undo it.
///
/// 0x27 (0x20..0x27 by A0..A2; the PCF8574A part 0x38..0x3F).
template<std::size_t Columns = 16, std::size_t Lines = 2>
struct Hd44780Pcf8574 {
    using Backlight = Hd44780Detail::Backlight;
    static_assert(Columns == 16 || Columns == 20,
                  "16 x 2 or 20 x 4");
    static_assert(Lines == 2 || Lines == 4,
                  "16 x 2 or 20 x 4");

    static constexpr std::size_t      ColumnCount   = Columns;
    static constexpr std::size_t      LineCount     = Lines;
    static constexpr std::string_view Name          = Columns == 16 ? "LCD1602" : "LCD2004";
    static constexpr Address7         Address       = 0x27;
    static constexpr std::size_t      RegisterBytes = 0;
    static constexpr auto             StartupDelay  = std::chrono::milliseconds{50};

    static constexpr std::array<Address7, 16> Addresses{0x20,
                                                        0x21,
                                                        0x22,
                                                        0x23,
                                                        0x24,
                                                        0x25,
                                                        0x26,
                                                        0x27,
                                                        0x38,
                                                        0x39,
                                                        0x3A,
                                                        0x3B,
                                                        0x3C,
                                                        0x3D,
                                                        0x3E,
                                                        0x3F};

    static constexpr std::uint8_t Rs           = 0x01;
    static constexpr std::uint8_t Enable       = 0x04;
    static constexpr std::uint8_t BacklightBit = 0x08;

    /// The port byte's backlight bit for a flag.
    [[nodiscard]] static constexpr std::uint8_t light(Backlight backlight) {
        return backlight == Backlight::on ? BacklightBit : std::uint8_t{0};
    }

    /// The four port states that send one byte (mode: 0 command, Rs data).
    [[nodiscard]] static constexpr std::array<std::uint8_t,
                                              4>
    nibbles(std::uint8_t v,
            std::uint8_t mode,
            std::uint8_t lit = BacklightBit) {
        auto const hi = static_cast<std::uint8_t>((v & 0xF0U) | mode | lit);
        auto const lo
          = static_cast<std::uint8_t>(((static_cast<unsigned>(v) << 4) & 0xF0U) | mode | lit);
        return {static_cast<std::uint8_t>(hi | Enable),
                hi,
                static_cast<std::uint8_t>(lo | Enable),
                lo};
    }

    /// One command byte as a Step, backlight on (the bring-up's). No default for the delay:
    /// a default argument may not be used from inside the class that declares it.
    [[nodiscard]] static constexpr Step command(std::uint8_t              v,
                                                std::chrono::milliseconds delay) {
        auto const n = nibbles(v, 0);
        return Step::command({
          .payload = {n[0], n[1], n[2], n[3]},
          .delay   = delay
        });
    }

    static constexpr std::array Init{
      // A single high nibble, three times, then the switch to 4-bit: E pulsed once each.
      Step::command({.payload = {static_cast<std::uint8_t>(0x30 | BacklightBit | Enable),
                                 static_cast<std::uint8_t>(0x30 | BacklightBit)},
                     .delay   = std::chrono::milliseconds{5}}
      ),
      Step::command({.payload = {static_cast<std::uint8_t>(0x30 | BacklightBit | Enable),
                                 static_cast<std::uint8_t>(0x30 | BacklightBit)},
                     .delay   = std::chrono::milliseconds{1}}
      ),
      Step::command({.payload = {static_cast<std::uint8_t>(0x30 | BacklightBit | Enable),
                                 static_cast<std::uint8_t>(0x30 | BacklightBit)},
                     .delay   = std::chrono::milliseconds{1}}
      ),
      Step::command({.payload = {static_cast<std::uint8_t>(0x20 | BacklightBit | Enable),
                                 static_cast<std::uint8_t>(0x20 | BacklightBit)},
                     .delay   = std::chrono::milliseconds{1}}
      ),
      command(0x28, std::chrono::milliseconds{0}
      ), // function set: 4-bit, two lines, 5 x 8
      command(0x08, std::chrono::milliseconds{0}
      ), // display off
      command(0x01, std::chrono::milliseconds{3}
      ), // clear
      command(0x06, std::chrono::milliseconds{0}
      ), // entry mode: increment, no shift
      command(0x0C, std::chrono::milliseconds{0}
      ), // display on, cursor off
    };

    /// One row of text, and the backlight state its bytes carry. Shorter strings are padded
    /// with spaces.
    struct Text {
        std::array<char, Columns> chars{};
        Backlight                 backlight{Backlight::on};

        [[nodiscard]] static constexpr Text of(std::string_view s,
                                               Backlight        backlight = Backlight::on) {
            Text t{};
            for(std::size_t i = 0; i < Columns; ++i) { t.chars[i] = i < s.size() ? s[i] : ' '; }
            t.backlight = backlight;
            return t;
        }

        constexpr bool operator==(Text const&) const = default;
    };

    static constexpr std::array<std::uint8_t, 4> RowAddress{
      0x00,
      0x40,
      static_cast<std::uint8_t>(Columns),
      static_cast<std::uint8_t>(0x40 + Columns)};

    struct Line {
        using Value                        = Text;
        static constexpr std::size_t Items = Lines;
        static constexpr std::size_t Bytes = 4 + 4 * Columns;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            auto const lit  = light(value.backlight);
            auto const addr = nibbles(static_cast<std::uint8_t>(0x80U | RowAddress[item]), 0, lit);
            for(std::size_t i = 0; i < 4; ++i) { buffer[i] = std::byte{addr[i]}; }
            for(std::size_t c = 0; c < Columns; ++c) {
                auto const n = nibbles(static_cast<std::uint8_t>(value.chars[c]), Rs, lit);
                for(std::size_t i = 0; i < 4; ++i) { buffer[4 + 4 * c + i] = std::byte{n[i]}; }
            }
            return Step::commandBuffer({.offset = 0, .count = static_cast<std::uint8_t>(Bytes)});
        }
    };

    /// The display-control byte (0x08 | D C B) and the backlight state its bytes carry.
    struct Display {
        std::uint8_t mode{0x0C};
        Backlight    backlight{Backlight::on};

        constexpr bool operator==(Display const&) const = default;
    };

    struct Control {
        using Value                        = Display;
        static constexpr std::size_t Bytes = 4;

        static constexpr Value on{0x0C, Backlight::on};       ///< display on, cursor off
        static constexpr Value off{0x08, Backlight::on};      ///< display off (RAM kept)
        static constexpr Value cursor{0x0E, Backlight::on};   ///< display on, underline cursor
        static constexpr Value blink{0x0F, Backlight::on};    ///< display on, blinking cursor
        static constexpr Value Initial = on;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            auto const n = nibbles(value.mode, 0, light(value.backlight));
            for(std::size_t i = 0; i < 4; ++i) { buffer[i] = std::byte{n[i]}; }
            return Step::commandBuffer({.offset = 0, .count = 4});
        }
    };

    using Writes = List<Line, Control>;
};

using Lcd1602 = Hd44780Pcf8574<16, 2>;
using Lcd2004 = Hd44780Pcf8574<20, 4>;

}   // namespace Kvasir::I2C::Chips
