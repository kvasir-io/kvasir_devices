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

/// Microchip MCP4017/4018/4019 single 7-bit digital potentiometer. There is no register
/// pointer and no command byte: a write is the one wiper byte and a read returns it, so
/// this is a register-less chip whose whole interface is 128 taps (0x00 zero scale to 0x7F
/// full scale, 127 resistors). The address is fixed at '0101111' = 0x2F for all three
/// parts; the difference between them is the terminal wiring, not the protocol.
///
/// `EndToEnd` is the RAB of the part fitted (5 k on the -502 the easyC board carries),
/// used only to turn a tap into an approximate resistance -- wiper resistance is not
/// modelled, and the datasheet's own figures show it varies with voltage and temperature.
template<Ohm EndToEnd = Units::ohm(5000)>
struct Mcp4018 {
    static constexpr std::string_view Name          = "MCP4018";
    static constexpr Address7         Address       = 0x2F;
    static constexpr std::size_t      RegisterBytes = 0;

    /// Fixed; there is no address pin.
    static constexpr std::array<Address7, 1> Addresses{0x2F};

    static constexpr std::uint8_t MaxTap = 0x7F;

    struct Wiper {
        static constexpr auto       Period = std::chrono::milliseconds{500};
        static constexpr std::array Steps{Step::receive({.count = 1, .offset = 0})};

        struct Sample {
            std::uint8_t tap{};

            /// The W-to-B resistance the tap approximates, RWB = RAB * N / 127 (Equation 6-2),
            /// ignoring wiper resistance. B is VSS on the MCP4018.
            [[nodiscard]] constexpr Ohm resistance() const {
                return Units::ohm(static_cast<std::uint32_t>(tap) * Units::value(EndToEnd)
                                  / MaxTap);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {static_cast<std::uint8_t>(data.u8(0) & MaxTap)};
        }
    };

    /// The one byte the part takes. No Initial: the wiper powers up at mid-scale (0x3F, Table 6-3)
    /// and forcing it somewhere else at every bring-up would be a surprise, but once the
    /// application has set a tap it is written again after a reset.
    struct Tap {
        using Value                        = std::uint8_t;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value & MaxTap);
            return Step::commandBuffer({.offset = 0, .count = 1});
        }
    };

    using Reads  = List<Wiper>;
    using Writes = List<Tap>;
};

}   // namespace Kvasir::I2C::Chips
