#pragma once

#include <cstdint>
#include <string_view>

namespace Kvasir::I2C {

/// One register the data sheet lets a description be held against: an identity, or a
/// configuration field a bring-up must leave in a known state.
///
/// A description declares them about itself, transcribed from the data sheet with the line they
/// stand on -- never derived from the rest of the description, which is what they judge:
///
///     /// INA226.md:874..883: Manufacturer ID (FEh) 5449h, Die ID (FFh) 2260h or 2261h.
///     static constexpr std::array Identity{
///       RegisterCheck{"manufacturer-id", 0xFE, 2, true, 0xFFFF, 0x5449},
///       RegisterCheck{"die-id", 0xFF, 2, true, 0xFFFE, 0x2260},
///     };
///
/// `Identity` is what tells the part from any other: the engine reads and compares it first in
/// every bring-up, and the device is not identified, not answering and not written to until it
/// matches (Device.hpp). `AfterBringUp`, a second array of the same kind, is what a finished
/// bring-up leaves in the part; the engine does not read it, the hardware test of i2c_testing
/// does -- off the real part, past the driver.
struct RegisterCheck {
    static constexpr std::uint32_t None = 0xFFFF'FFFFU;

    std::string_view name;          ///< the data sheet's name for it
    std::uint16_t    reg;           ///< the register's address
    std::uint8_t     width;         ///< bytes read, 1..4
    bool             bigEndian;     ///< how a multi-byte value comes off the wire
    std::uint32_t    mask;          ///< the bits that are predictable
    std::uint32_t    expect;        ///< what they must be
    std::uint32_t    also{None};    ///< a second value that is as good: a family's other part
    std::uint8_t     regBytes{1};   ///< bytes of register address (2: VL53L1X)

    [[nodiscard]] constexpr bool matches(std::uint32_t value) const {
        return (value & mask) == expect || (also != None && (value & mask) == also);
    }

    /// `width` bytes as they came off the wire, as one value.
    template<typename Byte>
    [[nodiscard]] constexpr std::uint32_t value(Byte const* bytes) const {
        std::uint32_t v = 0;
        for(std::size_t b = 0; b < width; ++b) {
            auto const at = bigEndian ? b : static_cast<std::size_t>(width) - 1U - b;
            v             = (v << 8U) | static_cast<std::uint8_t>(bytes[at]);
        }
        return v;
    }
};

}   // namespace Kvasir::I2C
