#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Texas Instruments TCA9555 16-bit I/O expander. One-byte pointer and paired 8-bit registers; the
/// pointer does not advance but alternates within the pair (9.5.2.1.2), so each pair is read and
/// written as one little-endian word (port 0 low, port 1 high): input 0x00/0x01 (read only), output
/// 0x02/0x03 (reset 0xFFFF), polarity inversion 0x04/0x05 (reset 0x0000), configuration 0x06/0x07
/// (reset 0xFFFF, every pin an input). The write groups are restored after every bring-up in the
/// order Output, Direction, Polarity, so a replay after a reset never drives a pin with a stale
/// latch. 0x20..0x27 by A2..A0.
struct Tca9555 {
    static constexpr std::string_view Name          = "TCA9555";
    static constexpr Address7         Address       = 0x20;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array<Address7, 8>
      Addresses{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};

    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

    static constexpr std::array Init{Step::read({.reg = 0x00, .count = 2, .offset = 0})};

    struct Pins {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2, .offset = 0})};

        struct Sample {
            std::uint16_t port{};   ///< port 0 | port 1 << 8

            /// Pin i: 0..7 is P00..P07, 8..15 is P10..P17.
            [[nodiscard]] constexpr bool pin(std::size_t i) const {
                return (port & (1U << i)) != 0;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.le16(0)}; }
    };

    /// One 16-bit pair, with the reset value the chip powers up in: written again after
    /// every bring-up. Same shape as Mcp23017::Pair.
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

    using Direction = Pair<0x06, 0xFFFF>;   ///< 1 = input
    using Output    = Pair<0x02, 0xFFFF>;
    using Polarity  = Pair<0x04, 0x0000>;

    using Reads = List<Pins>;
    /// Output before Direction: the latches hold their value before a pin becomes an output.
    using Writes = List<Output, Direction, Polarity>;
};

}   // namespace Kvasir::I2C::Chips
