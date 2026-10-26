#pragma once

#include "../../Engine/Description.hpp"
#include "../../Engine/Groups.hpp"
#include "../Address.hpp"

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

    static constexpr auto Addresses = addressRun<0x20, 8>();

    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

    static constexpr std::array Init{Engine::Step::read({.reg = 0x00, .count = 2, .offset = 0})};

    struct Pins {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{
          Engine::Step::read({.reg = 0x00, .count = 2, .offset = 0})};

        struct Sample {
            std::uint16_t port{};   ///< port 0 | port 1 << 8

            /// Pin i: 0..7 is P00..P07, 8..15 is P10..P17.
            [[nodiscard]] constexpr bool pin(std::size_t i) const {
                return (port & (1U << i)) != 0;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.le16(0)}; }
    };

    /// One 16-bit pair each, port 0 in the low byte, with the reset value the chip powers up in:
    /// written again after every bring-up.
    using Direction = Engine::Groups::WordLe<0x06, 0xFFFF>;   ///< 1 = input
    using Output    = Engine::Groups::WordLe<0x02, 0xFFFF>;
    using Polarity  = Engine::Groups::WordLe<0x04, 0x0000>;

    using Reads = Engine::List<Pins>;
    /// Output before Direction: the latches hold their value before a pin becomes an output.
    using Writes = Engine::List<Output, Direction, Polarity>;
};

}   // namespace Kvasir::I2C::Chips
