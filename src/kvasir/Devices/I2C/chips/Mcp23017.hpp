#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Microchip MCP23017 16-bit port expander (DS20001952C). One-byte register address,
/// auto-increment; with IOCON.BANK = 0 the A and B registers pair up (Table 3-5): IODIR
/// 0x00/0x01, GPPU 0x0C/0x0D, GPIO 0x12/0x13, OLAT 0x14/0x15, so each pair is one 16-bit
/// write, A in the low byte.
///
/// Bring-up puts the part in BANK = 0 whichever bank it is in: a part left in BANK = 1 by a
/// warm reset has IOCON at 0x05, where a BANK = 0 part has GPINTENB, so 0x05 = 0x00 is written
/// first (IOCON cleared in BANK = 1, a harmless "no interrupt-on-change on port B" in BANK = 0) and
/// then IOCON at its BANK = 0 address 0x0A = 0x00 (sequential, no mirroring).
///
/// The pins are read every 50 ms; Output, Direction (1 = input, the reset state) and
/// PullUp are write groups, restored in that order after every bring-up so that a replay
/// after a reset sets the output latches before any pin is turned into an output.
/// 0x20..0x27 by A2..A0.
struct Mcp23017 {
    static constexpr std::string_view Name = "MCP23017";
    /// Microchip MCP23017. IOCON (0x0A with BANK 0): BANK, MIRROR and SEQOP clear -- the register
    /// pairs the description writes as 16-bit words are only adjacent with BANK 0, and only written as
    /// a pair with sequential operation on (MCP23017.md:480..541). The part has no identity register.
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"iocon", 0x0A, 1, true, 0xE0, 0x00},
    };
    static constexpr Address7    Address       = 0x20;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 8>
      Addresses{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};

    static constexpr std::array Init{
      Step::write(
        {.reg     = 0x05,
         .payload = {0x00}}),   // IOCON if BANK = 1 (else GPINTENB): BANK = 0 from here on
      Step::write(
        {.reg = 0x0A, .payload = {0x00}}),   // IOCON in BANK = 0: sequential, no mirroring
    };

    struct Pins {
        static constexpr auto       Period = std::chrono::milliseconds{50};
        static constexpr std::array Steps{Step::read({.reg = 0x12, .count = 2})};

        struct Sample {
            std::uint16_t port{};   ///< GPIOA | GPIOB << 8

            /// Pin i: 0..7 is GPA0..GPA7, 8..15 is GPB0..GPB7.
            [[nodiscard]] constexpr bool pin(std::size_t i) const {
                return (port & (1U << i)) != 0;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.le16(0)}; }
    };

    /// One 16-bit pair, A in the low byte, with the reset value the chip powers up in:
    /// written again after every bring-up.
    template<std::uint16_t Reg, std::uint16_t Init>
    struct Pair {
        using Value                          = std::uint16_t;
        static constexpr std::size_t Bytes   = 2;
        static constexpr Value       Initial = Init;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            putLe16(buffer, 0, value);
            return Step::writeBuffer({.reg = Reg, .offset = 0, .count = 2});
        }
    };

    using Direction = Pair<0x00, 0xFFFF>;   ///< IODIR: 1 = input
    using PullUp    = Pair<0x0C, 0x0000>;   ///< GPPU: 1 = 100 k pull-up on an input
    using Output    = Pair<0x14, 0x0000>;   ///< OLAT

    using Reads = List<Pins>;
    /// Output before Direction: the latches hold their value before a pin becomes an output.
    using Writes = List<Output, Direction, PullUp>;
};

}   // namespace Kvasir::I2C::Chips
