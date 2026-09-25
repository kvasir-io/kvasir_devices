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

/// Maxim / Analog Devices DS3502 high-voltage 7-bit digital potentiometer, 0x28..0x2B by
/// A1 A0. Two registers behind a one-byte pointer: WIPER 0x00 (WR/IVR, 0..127) and MODE 0x02
/// (CR).
///
/// MODE decides where a wiper write lands, and it is the whole reason this part needs care:
///
///  * `MODE = 0x80` -- a write to WIPER changes the wiper only, volatile;
///  * `MODE = 0x00` -- a write to WIPER *also programs the non-volatile IVR*, which sets the
///    power-up position. That is an EEPROM write cycle (tW, typ 10 ms and max 20 ms,
///    beginning after the STOP, note 12; the 100 ms this description waits is margin, not
///    the datasheet figure) and spends one of a finite number of write cycles.
///
/// So bring-up writes 0x80, ordinary `set<Wiper>` stays volatile, and the non-volatile
/// default is a separate `Persist` group marked `Transient` -- it is a one-shot command, and
/// replaying it after a device reset would burn another cycle for nothing. `Mcp4725::Persist`
/// and `At24cxx::WritePage` get exactly the same treatment.
template<Ohm EndToEnd = Units::ohm(10000)>
struct Ds3502 {
    static constexpr std::string_view Name          = "DS3502";
    static constexpr Address7         Address       = 0x28;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array<Address7, 4> Addresses{0x28, 0x29, 0x2A, 0x2B};

    static constexpr std::uint8_t MaxTap = 127;

    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    /// MODE = 0x80: wiper writes are volatile from here on.
    static constexpr std::array Init{
      Step::write({.reg = 0x02, .payload = {0x80}, .delay = std::chrono::milliseconds{1}}),
      Step::read({.reg = 0x00, .count = 1, .offset = 0})};

    struct State {
        std::uint8_t powerUpTap{};   ///< what the IVR had it at
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.powerUpTap = static_cast<std::uint8_t>(data.u8(0) & MaxTap);
        return true;   // there is no id register to check
    }

    struct Position {
        static constexpr auto       Period = std::chrono::milliseconds{500};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

        struct Sample {
            std::uint8_t tap{};

            /// The wiper-to-low-terminal resistance the tap approximates; wiper resistance
            /// is not modelled.
            [[nodiscard]] constexpr Ohm resistance() const {
                return Units::ohm(static_cast<std::uint32_t>(tap) * Units::value(EndToEnd)
                                  / MaxTap);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {static_cast<std::uint8_t>(data.u8(0) & MaxTap)};
        }
    };

    /// The volatile wiper. No Initial: the part comes up at whatever its IVR holds, and
    /// overriding that at every bring-up would be a surprise. Once the application has set a
    /// tap it is written again after a reset.
    struct Wiper {
        using Value                        = std::uint8_t;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value > MaxTap ? MaxTap : value);
            return Step::writeBuffer({.reg = 0x00, .offset = 0, .count = 1});
        }
    };

    /// The power-up default, which is an EEPROM write: MODE to 0x00 so the wiper write programs
    /// the IVR, the wiper and a 100 ms wait (a margin over tW, typ 10 ms and max 20 ms
    /// after the STOP), then MODE back to 0x80 so later wiper writes are volatile again. One
    /// item, three transactions, so the part is never left in non-volatile mode between two
    /// sets.
    struct Persist {
        using Value                        = std::uint8_t;
        static constexpr std::size_t Bytes = 1;
        /// A one-shot command: replaying it after a reset would spend another write cycle.
        static constexpr bool Transient = true;

        [[nodiscard]] static constexpr std::array<Step,
                                                  3>
        encode(Value const&         value,
               std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value > MaxTap ? MaxTap : value);
            return {
              Step::write({.reg = 0x02, .payload = {0x00}}),
              Step::writeBuffer(
                {.reg    = 0x00,
                 .offset = 0,
                 .count  = 1,
                 .delay  = std::chrono::milliseconds{100}}),   // the IVR write cycle, with margin
              Step::write({.reg = 0x02, .payload = {0x80}})};
        }
    };

    // MODE is not a write group: with it at 0x00 every Wiper write would also program the IVR
    // and spend an EEPROM cycle. Init sets 0x80, and only Persist leaves it, for one write.

    using Reads  = List<Position>;
    using Writes = List<Wiper, Persist>;
};

}   // namespace Kvasir::I2C::Chips
