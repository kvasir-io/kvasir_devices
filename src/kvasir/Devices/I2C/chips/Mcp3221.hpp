#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Microchip MCP3221 12-bit successive-approximation ADC (DS20001732E). No registers and no
/// configuration: every read is a conversion, sampled while the address byte is clocked in, and
/// answered as two bytes -- four 0 bits, then the upper four data bits, then the lower eight: the
/// 12-bit code MSB first (section 5.3.2, figure 5-5). The reference is VDD, so a code only becomes
/// a voltage with the supply the board runs the part on: `Supply` (3.3 V by default, as the
/// Mcp47a1's) scales `voltage()`, and `voltage(vdd)` takes a measured one instead. Neither reaches
/// the chip. 0x48..0x4F is the part's address code 1001 A2 A1 A0, fixed at the factory per part
/// number; MCP3221A5 (0x4D) is the common one, and what MikroE fits to Vibra Sense 2 Click.
template<MilliVolt Supply = Units::milliVolt(3300)>
struct Mcp3221 {
    static constexpr std::string_view Name          = "MCP3221";
    static constexpr Address7         Address       = 0x4D;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array<Address7, 8>
      Addresses{0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F};

    static_assert(Supply > Units::milliVolt(0),
                  "the supply is the reference every code is scaled by; it may not be zero");

    static constexpr std::uint16_t FullScale = 4096;

    struct Conversion {
        static constexpr auto       Period = std::chrono::milliseconds{20};
        static constexpr std::array Steps{Step::receive({.count = 2})};

        struct Sample {
            std::uint16_t code{};   ///< 0..4095 of VDD

            /// The input, against the supply the part runs on.
            [[nodiscard]] constexpr MilliVolt voltage(MilliVolt vdd) const {
                return Units::milliVolt(static_cast<std::int32_t>(code) * Units::value(vdd)
                                        / FullScale);
            }

            /// Against the `Supply` parameter.
            [[nodiscard]] constexpr MilliVolt voltage() const { return voltage(Supply); }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {static_cast<std::uint16_t>(data.be16(0) & 0x0FFFU)};
        }
    };

    using Reads = List<Conversion>;
};

}   // namespace Kvasir::I2C::Chips
