#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// NXP / TI PCA9557 8-bit I/O expander (TI SCPS133K, 7.5). One-byte pointer, four 8-bit
/// registers: input port 0x00 (read only), output port 0x01 (reset 0x00, Table 7-5),
/// polarity inversion 0x02 (reset 0xF0, Table 7-6: the upper four inputs come up inverted,
/// which is this part's oddity) and configuration 0x03 (reset 0xFF, Table 7-7: every pin an
/// input). A bit set in the configuration is an input. P0 is open drain (the pin table): as an
/// output it only pulls low, and needs a pull-up to read high.
///
/// The part has no id register, and 0x18..0x1F are shared with the MCP9808 and the MMA8451,
/// so a part that answers there is taken for this one.
///
/// The write groups carry those reset values as their Initial, so a bring-up puts the part
/// back where it powers up -- including the 0xF0 polarity; an application that wants its
/// inputs read straight sets `Polarity` to 0x00 once and the engine keeps it there. They are
/// restored in the order Output, Direction, Polarity, so a replay after a reset never
/// drives a pin from a stale latch. 0x18..0x1F by A2..A0.
struct Pca9557 {
    static constexpr std::string_view Name          = "PCA9557";
    static constexpr Address7         Address       = 0x18;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array<Address7, 8>
      Addresses{0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};

    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

    /// The input port read is the presence probe.
    static constexpr std::array Init{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

    struct Pins {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        struct Sample {
            std::uint8_t port{};

            [[nodiscard]] constexpr bool pin(std::size_t i) const {
                return (port & (1U << i)) != 0;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.u8(0)}; }
    };

    /// One 8-bit register, with the reset value the chip powers up in: written again after
    /// every bring-up, so a device that was reconfigured by something else comes back to a
    /// known state.
    template<std::uint16_t Reg, std::uint8_t Init>
    using Byte = Groups::InitialByte<Reg, Init>;

    using Direction = Byte<0x03, 0xFF>;   ///< configuration: 1 = input (reset 0xFF)
    using Output    = Byte<0x01, 0x00>;   ///< output port (reset 0x00)
    using Polarity  = Byte<0x02, 0xF0>;   ///< polarity inversion (reset 0xF0)

    using Reads = List<Pins>;
    /// Output before Direction: the latch holds its value before a pin becomes an output.
    using Writes = List<Output, Direction, Polarity>;
};

}   // namespace Kvasir::I2C::Chips
