#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Microchip MCP47A1 single-channel 6-bit DAC (DS25154A), as carried on the Soldered easyC
/// breakout (333052). A write is the command code 0x00 followed by one data byte (Figure 5-11),
/// so the part behaves like a one-register chip. The data byte's lower 7 bits (the serial shift
/// register) decode to a wiper of 65 taps: code N gives VOUT = N / 64 x VREF for N = 0..64
/// (Equation 7-1), so 0x40 is full scale and 0x41..0x7F decode to full scale too (Table 6-1).
/// Linux mcp47a1.c accepts 0..63 and so never reaches full scale.
/// Power-on is mid-scale, 0x20 (Table 6-2).
///
/// The address is an ordering option, not a pin (Table 5-2): 0x2E for the A0 option (8-bit
/// 0x5C), 0x3E for A1 (0x7C).
///
/// `Supply` only scales `codeFor()`; it does not reach the chip. It stands for VREF, which is
/// its own pin: the scale is right when the board ties VREF to the supply.
template<MilliVolt Supply = Units::milliVolt(3300)>
struct Mcp47a1 {
    static constexpr std::string_view Name          = "MCP47A1";
    static constexpr Address7         Address       = 0x2E;
    static constexpr std::size_t      RegisterBytes = 1;

    /// By ordering option; there is no address pin.
    static constexpr std::array<Address7, 2> Addresses{0x2E, 0x3E};

    /// 65 positions: codes 0..64, 64 being VREF.
    static constexpr std::uint8_t Taps      = 65;
    static constexpr std::uint8_t FullScale = Taps - 1;
    /// `Taps` under the name generic potentiometer views read. A read group's `Steps` is its
    /// script, so a chip-level number of that name misleads; prefer `Taps`.
    static constexpr std::uint8_t Steps = Taps;

    /// The code at or below a wanted output, for callers that think in volts: `Supply` is 64.
    [[nodiscard]] static constexpr std::uint8_t codeFor(MilliVolt output) {
        auto const mV = Units::value(output);
        auto const c  = mV <= 0 ? 0 : mV * FullScale / Units::value(Supply);
        return static_cast<std::uint8_t>(c >= FullScale ? FullScale : c);
    }

    /// The output level, 0..64; more is full scale. No Initial: the part powers up at mid-scale and forcing it
    /// somewhere at every bring-up would be a surprise; once set, the engine re-sends it after a
    /// reset.
    struct Level {
        using Value                        = std::uint8_t;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value >= FullScale ? FullScale : value);
            return Step::writeBuffer({.reg = 0x00, .offset = 0, .count = 1});
        }
    };

    using Writes = List<Level>;
};

}   // namespace Kvasir::I2C::Chips
