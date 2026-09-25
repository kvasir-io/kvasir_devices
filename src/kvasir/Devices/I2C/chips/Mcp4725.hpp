#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Mcp4725Detail {
    /// PD1 PD0: normal, or the output pulled down through 1 k, 100 k or 500 k (6.1.1).
    enum class PowerDown : std::uint8_t { normal = 0, res1k = 1, res100k = 2, res500k = 3 };

    /// A code and the power-down mode that goes out with it. Converts implicitly from a plain
    /// code, so set<Level>(2048) sets mid-scale in normal mode. Out here rather than nested in
    /// the chip because a constructor may not be called from inside the class that encloses
    /// it.
    struct Output {
        std::uint16_t code{};
        PowerDown     powerDown{PowerDown::normal};

        constexpr Output() = default;

        constexpr Output(std::uint16_t c,
                         PowerDown     pd = PowerDown::normal)
          : code{static_cast<std::uint16_t>(c & 0x0FFFU)}
          , powerDown{pd} {}

        constexpr bool operator==(Output const&) const = default;
    };
}   // namespace Mcp4725Detail

/// Microchip MCP4725 12-bit DAC (DS22039D). No register pointer: the fast-mode write is
/// two bytes, 0 0 PD1 PD0 D11..D8 then D7..D0 (6.1.1); the "write DAC and EEPROM" command is
/// 0x60 | PD << 1, D11..D4, D3..D0 << 4 (6.1.3), the EEPROM taking up to 50 ms. A read
/// returns five bytes (6.2): the status byte (RDY/BSY bit 7, POR bit 6, PD1 PD0 in bits
/// 2:1), the DAC register (D11..D4, D3..D0 << 4) and the EEPROM contents (PD1 PD0 in bits
/// 6:5 and D11..D8 in the low nibble, then D7..D0).
///
/// `Level` has no Initial: the part loads its output from the EEPROM at power-up, which is
/// what `Persist` is for, and a bring-up that forced 0 V would override the very value the
/// board stored to come up with. Once the application has set a level it is written again
/// after a reset. `Supply` only scales `codeFor()` and `Status::Sample::voltage()`; it does
/// not reach the chip. 0x60..0x67: A0 is a pin, A2 A1 are factory options.
template<MilliVolt Supply = Units::milliVolt(3300)>
struct Mcp4725 {
    static constexpr std::string_view Name          = "MCP4725";
    static constexpr Address7         Address       = 0x60;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array<Address7, 8>
      Addresses{0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67};

    static_assert(Supply > Units::milliVolt(0),
                  "the supply is what a code is a fraction of");

    using PowerDown = Mcp4725Detail::PowerDown;
    using Output    = Mcp4725Detail::Output;

    /// 4096 codes across VDD.
    static constexpr std::uint16_t FullScale = 4096;

    /// The code at or below a wanted output, for callers that think in volts.
    [[nodiscard]] static constexpr std::uint16_t codeFor(MilliVolt output) {
        auto const mV = Units::value(output);
        if(mV <= 0) { return 0; }
        auto const c = static_cast<std::uint32_t>(mV) * FullScale
                     / static_cast<std::uint32_t>(Units::value(Supply));
        return static_cast<std::uint16_t>(c >= FullScale ? FullScale - 1 : c);
    }

    /// What a code amounts to against a supply.
    [[nodiscard]] static constexpr MilliVolt voltageOf(std::uint16_t code,
                                                       MilliVolt     supply) {
        return Units::milliVolt(static_cast<std::int32_t>(
          static_cast<std::uint32_t>(code) * static_cast<std::uint32_t>(Units::value(supply))
          / FullScale));
    }

    /// The output, 0..4095 of VDD.
    struct Level {
        using Value                        = Output;
        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>((static_cast<unsigned>(value.powerDown) << 4)
                                               | ((value.code >> 8) & 0x0FU));
            buffer[1] = static_cast<std::byte>(value.code & 0xFF);
            return Step::commandBuffer({.offset = 0, .count = 2});
        }
    };

    /// The output and the power-on default together (an EEPROM write: 50 ms).
    struct Persist {
        using Value                        = Output;
        static constexpr std::size_t Bytes = 3;

        /// Writing the EEPROM is a one-shot command: replaying it after a reset would burn
        /// another of the part's finite write cycles. Level (volatile) is replayed instead.
        static constexpr bool Transient = true;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0]
              = static_cast<std::byte>(0x60U | (static_cast<unsigned>(value.powerDown) << 1));
            buffer[1] = static_cast<std::byte>((value.code >> 4) & 0xFF);
            buffer[2] = static_cast<std::byte>((value.code & 0x0F) << 4);
            return Step::commandBuffer(
              {.offset = 0, .count = 3, .delay = std::chrono::milliseconds{50}});
        }
    };

    /// On demand (request<Status>()): what the part holds.
    struct Status {
        static constexpr std::array Steps{Step::receive({.count = 5, .offset = 0})};

        struct Sample {
            bool          ready{};       ///< RDY/BSY: no EEPROM write in progress
            bool          poweredOn{};   ///< POR: the supply is above the reset threshold
            std::uint16_t dac{};         ///< the DAC register, 0..4095
            std::uint16_t eeprom{};      ///< the EEPROM's code, 0..4095
            PowerDown     dacPowerDown{PowerDown::normal};      ///< the mode the output is in
            PowerDown     eepromPowerDown{PowerDown::normal};   ///< the mode it powers up in

            /// The output the DAC register amounts to against `Supply`.
            [[nodiscard]] constexpr MilliVolt voltage() const { return voltageOf(dac, Supply); }

            /// The same against a measured supply.
            [[nodiscard]] constexpr MilliVolt voltage(MilliVolt supply) const {
                return voltageOf(dac, supply);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.ready        = (data.u8(0) & 0x80U) != 0;
            sample.poweredOn    = (data.u8(0) & 0x40U) != 0;
            sample.dacPowerDown = static_cast<PowerDown>((data.u8(0) >> 1) & 0x03U);
            sample.dac          = static_cast<std::uint16_t>((data.u8(1) << 4) | (data.u8(2) >> 4));
            sample.eepromPowerDown = static_cast<PowerDown>((data.u8(3) >> 5) & 0x03U);
            sample.eeprom = static_cast<std::uint16_t>(((data.u8(3) & 0x0FU) << 8) | data.u8(4));
            return sample;
        }
    };

    using Reads  = List<Status>;
    using Writes = List<Level, Persist>;
};

}   // namespace Kvasir::I2C::Chips
