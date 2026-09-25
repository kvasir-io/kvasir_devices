#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Microchip MCP3421 18-bit delta-sigma ADC (DS22003E). No registers: a one-byte
/// configuration write, a read of the data followed by the configuration byte. Config
/// 0x9C: continuous conversion, 18 bits (3.75 SPS), gain 1 (Register 5-1). At 18 bits the
/// result is three bytes, two's complement, 15.625 uV/LSB (Table 4-1); RDY (bit 7 of the
/// trailing configuration byte) set means the register has not been updated since the
/// last read, so the read is repeated. 0x68 by default (0x68..0x6F are factory options).
struct Mcp3421 {
    static constexpr std::string_view Name          = "MCP3421";
    static constexpr Address7         Address       = 0x68;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array<Address7, 8>
      Addresses{0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F};

    static constexpr std::array Init{
      Step::command({.payload = {0x9C}, .delay = std::chrono::milliseconds{270}})};

    struct Conversion {
        static constexpr auto       Period = std::chrono::milliseconds{300};
        static constexpr std::array Steps{Step::receive({.count = 4, .offset = 0})};

        struct Sample {
            std::int32_t code{};   ///< 18-bit two's complement

            /// 15.625 uV a count.
            [[nodiscard]] constexpr MicroVolt voltage() const {
                return Units::microVolt(code * 125 / 8);
            }
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if((data.u8(3) & 0x80U) != 0) {
                return Outcome<Sample>::retry(std::chrono::milliseconds{50});
            }
            // 18 bits: the top six of the first byte repeat the sign and are dropped
            return Outcome<Sample>::ok({Bytes::signExtend(data.be24(0) & 0x3FFFFU, 18)});
        }
    };

    using Reads = List<Conversion>;
};

}   // namespace Kvasir::I2C::Chips
