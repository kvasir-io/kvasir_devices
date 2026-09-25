#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// PCF8574 / PCF8574A 8-bit quasi-bidirectional port (SCPS068K, 7.3). No registers: a
/// one-byte write drives the port (1 = weak high, i.e. input; 0 = driven low), a one-byte
/// read returns the pins. Pins are read every 50 ms; `set<Port>(v)` writes when changed;
/// all pins start released. 0x20..0x27, the A part 0x38..0x3F.
struct Pcf8574 {
    static constexpr std::string_view Name          = "PCF8574";
    static constexpr Address7         Address       = 0x20;
    static constexpr std::size_t      RegisterBytes = 0;

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

    struct Pins {
        static constexpr auto       Period = std::chrono::milliseconds{50};
        static constexpr std::array Steps{Step::receive({.count = 1})};

        struct Sample {
            std::uint8_t port{};

            [[nodiscard]] constexpr bool pin(std::size_t i) const {
                return (port & (1U << i)) != 0;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.u8(0)}; }
    };

    struct Port {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = 0xFF;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{value};
            return Step::commandBuffer({.offset = 0, .count = 1});
        }
    };

    using Reads  = List<Pins>;
    using Writes = List<Port>;
};

/// PCF8575 16-bit quasi-bidirectional port (SCPS121I, 8.3.2): the PCF8574's protocol
/// with two bytes, P07..P00 first, then P17..P10. 0x20..0x27.
struct Pcf8575 {
    static constexpr std::string_view Name          = "PCF8575";
    static constexpr Address7         Address       = 0x20;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array<Address7, 8>
      Addresses{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};

    struct Pins {
        static constexpr auto       Period = std::chrono::milliseconds{50};
        static constexpr std::array Steps{Step::receive({.count = 2})};

        struct Sample {
            std::uint16_t port{};   ///< P0x | P1x << 8

            /// Pin i: 0..7 is P00..P07, 8..15 is P10..P17.
            [[nodiscard]] constexpr bool pin(std::size_t i) const {
                return (port & (1U << i)) != 0;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.le16(0)}; }
    };

    struct Port {
        using Value                          = std::uint16_t;
        static constexpr std::size_t Bytes   = 2;
        static constexpr Value       Initial = 0xFFFF;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            putLe16(buffer, 0, value);
            return Step::commandBuffer({.offset = 0, .count = 2});
        }
    };

    using Reads  = List<Pins>;
    using Writes = List<Port>;
};

}   // namespace Kvasir::I2C::Chips
