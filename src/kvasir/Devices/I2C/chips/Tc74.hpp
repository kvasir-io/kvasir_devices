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

namespace Tc74Detail {
    /// CONFIG (01h) bit 7, SHDN: the A/D converter halted and TEMP frozen; the I2C port still
    /// answers (TC74.md:225, Table 4-2 :317, Table 3-1 :229-232).
    enum class Mode : std::uint8_t { normal, standby };

    inline constexpr std::uint8_t ConfigStandby  = 0x80;   ///< D7, read/write
    inline constexpr std::uint8_t ConfigReady    = 0x40;   ///< D6 DATA_RDY, read only
    inline constexpr std::uint8_t ConfigReserved = 0x3F;   ///< D5..D0 "always returns zero"

    /// TEMP is an 8-bit two's complement temperature, one degree C per count (4.1, TC74.md:334-343,
    /// Table 4-4 :345-364: 0x7F +127, 0xE7 -25, 0xC9 -55).
    [[nodiscard]] constexpr CentiDegC temperatureOf(std::uint8_t raw) {
        return Units::centiDegC(static_cast<std::int32_t>(static_cast<std::int8_t>(raw)) * 100);
    }
}   // namespace Tc74Detail

/// Microchip TC74 (DS21462D, TC74.md). Two 8-bit registers behind a one-byte command (Table 4-1,
/// :304-309): RTR 00h reads TEMP, RWCR 01h reads or writes CONFIG. The address is fixed by the
/// ordered part, not by pins: TC74Ax answers at 1001 xxx = 0x48 + x, A5 (0x4D) the default
/// (marking table :402-415, product identification :504-507) - so the variant is the template
/// parameter, `Tc74<1>` for a TC74A1 at 0x49.
///
/// No id register. What the part guarantees is CONFIG D5..D0 reading zero (Table 4-2, :319),
/// checked before anything is written (Step::identify); a part that reads all zeros passes it,
/// as it would for the RV-8803. Bring-up then leaves SHDN clear through the Power group's
/// Initial (normal mode; CONFIG resets to 00h, Table 4-5 :377).
///
/// A reading is CONFIG and TEMP: TEMP holds 00h until the first conversion after power-up or
/// standby, which DATA_RDY (D6) marks (Table 4-2 note :323, Table 4-5 note :381), at most 250 ms
/// after POR (Note 2, :112); without DATA_RDY the frame is Outcome::unchanged(). Nominal 8
/// conversions a second, 4 at least (CR, :94), so a 250 ms period sees a new one each time.
/// SMBus/I2C clock 10..100 kHz (fSMB, :102): a faster bus fails the build unless it runs each
/// device at its own clock (I2CConfig::perDeviceClock).
template<std::uint8_t Variant = 5>
struct Tc74 {
    static_assert(Variant <= 7,
                  "the TC74 is ordered as A0..A7 (TC74.md:402-411)");

    static constexpr std::string_view Name    = "TC74";
    static constexpr Address7         Address = static_cast<Address7>(0x48 + Variant);
    static constexpr std::array<Address7, 8>
                                  Addresses{0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F};
    static constexpr std::size_t  RegisterBytes = 1;
    static constexpr Units::Hertz I2cMaxClock   = Units::hertz(100'000);

    static constexpr std::uint8_t Rtr  = 0x00;   ///< read TEMP
    static constexpr std::uint8_t Rwcr = 0x01;   ///< read/write CONFIG

    using Mode = Tc74Detail::Mode;

    static constexpr std::array Init{
      Step::read({.reg = Rwcr, .count = 1, .offset = 0}),
      Step::identify(),
    };

    struct State {
        std::uint8_t configuration{};   ///< CONFIG as the part held it at bring-up
    };

    /// Checked before anything is written (Step::identify): D5..D0 read zero.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.configuration = data.u8(0);
        return (state.configuration & Tc74Detail::ConfigReserved) == 0;
    }

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{250};
        static constexpr std::array Steps{
          Step::read({.reg = Rwcr, .count = 1, .offset = 0}),   // CONFIG: DATA_RDY, SHDN
          Step::read({.reg = Rtr, .count = 1, .offset = 1}),    // TEMP
        };

        struct Sample {
            CentiDegC temperature{};   ///< whole degrees, 0.01 degC units
        };

        /// Reserved bits set: not a TC74 answering (a floating bus reads FFh) - rejected. No
        /// DATA_RDY, or in standby (TEMP frozen, 3.1.1): nothing new.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            auto const config = data.u8(0);
            if((config & Tc74Detail::ConfigReserved) != 0) { return Outcome<Sample>::reject(); }
            if((config & Tc74Detail::ConfigStandby) != 0 || (config & Tc74Detail::ConfigReady) == 0)
            {
                return Outcome<Sample>::unchanged();
            }
            return Outcome<Sample>::ok({Tc74Detail::temperatureOf(data.u8(1))});
        }
    };

    /// CONFIG's SHDN: dev.set<Power>(Tc74<>::Mode::standby). Standby resets DATA_RDY (Table 4-2
    /// note), so readings stop until normal mode is back and a conversion is done.
    struct Power {
        using Value                        = Mode;
        static constexpr std::size_t Bytes = 1;
        static constexpr Value       Initial{Mode::normal};

        [[nodiscard]] static constexpr Step encode(Value const&         mode,
                                                   std::span<std::byte> buffer) {
            buffer[0]
              = std::byte{mode == Mode::standby ? Tc74Detail::ConfigStandby : std::uint8_t{0}};
            return Step::writeBuffer({.reg = Rwcr, .offset = 0, .count = 1});
        }
    };

    using Reads  = List<Temperature>;
    using Writes = List<Power>;
};

}   // namespace Kvasir::I2C::Chips
