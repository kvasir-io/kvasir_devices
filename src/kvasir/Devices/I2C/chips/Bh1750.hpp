#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// ROHM BH1750FVI ambient light sensor (bh1750fvi-e datasheet, "Instruction Set
/// Architecture"). No registers: one-byte opcodes, a two-byte big-endian result read
/// bare. Power on (01h), reset (07h), MTreg set to its default 69 (42h, 65h: "Change
/// measurement time", high bits then low bits), continuous H-resolution (10h: 1 lx, 120 ms typ
/// / 180 ms max per measurement), then the result every 200 ms; lux = raw / 1.2. The reset
/// clears only the data register ("Reset" in the instruction set), so a part whose MTreg
/// earlier firmware changed would keep it, and raw / 1.2 holds for MTreg 69 alone; Linux
/// bh1750.c writes it at probe too.
///
/// The part has no status or identification: every two bytes it returns are a count, and
/// 0xFFFF -- 54612 lx -- is its own full scale rather than a frame that can be told from a
/// bus that answered nothing, so no frame is rejected; `saturated()` marks it.
/// 0x23 with ADDR low, 0x5C with ADDR high.
struct Bh1750 {
    static constexpr std::string_view        Name    = "BH1750";
    static constexpr Address7                Address = 0x23;
    static constexpr std::array<Address7, 2> Addresses{0x23, 0x5C};
    static constexpr std::size_t             RegisterBytes = 0;

    static constexpr std::array Init{
      Step::command({.payload = {0x01}}),
      Step::command({.payload = {0x07}}),
      Step::command({.payload = {0x42}}),   // MTreg bits 7:5 = 010
      Step::command({.payload = {0x65}}),   // MTreg bits 4:0 = 00101: 69
      Step::command({.payload = {0x10}, .delay = std::chrono::milliseconds{180}}),
    };

    struct Light {
        static constexpr auto       Period = std::chrono::milliseconds{200};
        static constexpr std::array Steps{Step::receive({.count = 2})};

        struct Sample {
            std::uint16_t raw{};

            /// lux = raw / 1.2.
            [[nodiscard]] constexpr MilliLux lux() const {
                return Units::milliLux(static_cast<std::uint32_t>(raw) * 10000U / 12U);
            }

            /// The count at the top of the part's range: the light is at least this.
            [[nodiscard]] constexpr bool saturated() const { return raw == 0xFFFF; }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.be16(0)}; }
    };

    using Reads = List<Light>;
};

}   // namespace Kvasir::I2C::Chips
