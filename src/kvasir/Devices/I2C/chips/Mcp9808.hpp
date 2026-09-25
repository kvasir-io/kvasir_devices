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

/// Microchip MCP9808 (DS25095A). One-byte pointer, 16-bit big-endian registers except the
/// 8-bit resolution register. Bring-up: manufacturer 0x06 = 0x0054, device 0x07 upper
/// byte 0x04 (5.1.4, 5.1.5), resolution 0x08 = 3 (0.0625 degC, 250 ms), configuration
/// 0x01 = 0 (continuous, alerts off). Temperature 0x05: bits 15..13 are alert flags, bit
/// 12 the sign, 12 fraction/integer bits at 1/16 degC (5.1.3). 0x18..0x1F by A2..A0.
struct Mcp9808 {
    static constexpr std::string_view Name = "MCP9808";
    /// Microchip MCP9808. MCP9808.md:802 (manufacturer ID 0x0054, register 6) and :836 (device ID 0x04
    /// in the upper byte of register 7, the revision below it).
    /// Configuration: CONFIG (0x01) SHDN, bit 8, clear -- converting (MCP9808.md:550); resolution
    /// register 0x08 = 11b, 0.0625 degC, which is what the description's decode assumes.
    static constexpr std::array Identity{
      RegisterCheck{"manufacturer-id", 0x06, 2, true, 0xFFFF, 0x0054},
      RegisterCheck{      "device-id", 0x07, 2, true, 0xFF00, 0x0400},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"not-shut-down", 0x01, 2, true, 0x0100, 0x0000},
      RegisterCheck{   "resolution", 0x08, 1, true,   0x03,   0x03},
    };
    static constexpr Address7    Address       = 0x18;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 8>
      Addresses{0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};

    static constexpr std::array Init{
      Step::write({.reg = 0x08,       .payload = {0x03}}
      ),
      Step::write({.reg = 0x01, .payload = {0x00, 0x00}}
      ),
    };

    struct State {
        std::uint16_t manufacturer{};
        std::uint16_t deviceId{};   ///< Device ID (0x07)
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.manufacturer = static_cast<std::uint16_t>(ids[0]);
        state.deviceId     = static_cast<std::uint16_t>(ids[1]);
    }

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{250};
        static constexpr std::array Steps{Step::read({.reg = 0x05, .count = 2})};

        struct Sample {
            CentiDegC temperature{};
            bool      aboveCritical{};
            bool      aboveUpper{};
            bool      belowLower{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            auto const raw = data.be16(0);
            Sample     sample{};
            sample.aboveCritical = (raw & 0x8000U) != 0;
            sample.aboveUpper    = (raw & 0x4000U) != 0;
            sample.belowLower    = (raw & 0x2000U) != 0;
            sample.temperature   = Units::centiDegC(Bytes::signExtend(raw & 0x1FFFU, 13) * 25 / 4);
            return sample;
        }
    };

    /// The three limit registers (5.1.2): T_UPPER 0x02, T_LOWER 0x03, T_CRIT 0x04, each a
    /// 13-bit two's complement temperature in 0.25 degC at bits 12..2. Items 0, 1, 2. The
    /// flags in Temperature::Sample compare against them whatever the alert output does.
    struct Limits {
        using Value                        = CentiDegC;
        static constexpr std::size_t Items = 3;
        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr Step encode(Value const&         limit,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            // Clamped to the part's -40 .. +125 degC (Linux jc42.c clamps to the same): the
            // register holds -256 .. +255.75 degC, and past that the sign bit would wrap.
            auto const asked
              = limit.numerical_value_in(Units::si::centi<Units::si::degree_Celsius>);
            auto const centi    = asked < -4000 ? -4000 : asked > 12500 ? 12500 : asked;
            auto const quarters = static_cast<std::int16_t>(centi / 25);
            auto const raw
              = static_cast<std::uint16_t>(static_cast<std::uint16_t>(quarters << 2) & 0x1FFCU);
            putBe16(buffer, 0, raw);
            return Step::writeBuffer(
              {.reg = static_cast<std::uint16_t>(0x02 + item), .offset = 0, .count = 2});
        }
    };

    /// Limits items, in register order: 0x02 upper, 0x03 lower, 0x04 critical.
    static constexpr std::size_t High     = 0;
    static constexpr std::size_t Low      = 1;
    static constexpr std::size_t Critical = 2;

    using Reads  = List<Temperature>;
    using Writes = List<Limits>;
};

}   // namespace Kvasir::I2C::Chips
