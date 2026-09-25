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

/// NXP MPL3115A2 barometer / altimeter (MPL3115A2 data sheet rev 3.0). One-byte
/// registers, auto-increment. Bring-up: WHO_AM_I (0x0C) = 0xC4; the software reset,
/// CTRL_REG1 (0x26) RST -- every register back to its default, so a warm part left with
/// the FIFO on (which moves STATUS and zeroes the data registers, 7.8) or a slow ST step in
/// CTRL_REG2 starts clean; the reset also resets the I2C interface, so that write is not
/// acknowledged (Linux mpl3115.c: "I2C transfer is aborted (fails)", then 50 ms); then
/// PT_DATA_CFG (0x13) = 0x07 (data-ready event flags on); CTRL_REG1 = 0x00 (standby: the OS
/// bits can only be written while SBYB is clear), 0x38 (barometer, OS = 128: 512 ms per
/// measurement), then 0x39 (active). A read waits for STATUS (0x00) PTDR (bit 3) -- polled
/// every 100 ms, so the engine's retries span more than the first 512 ms conversion -- then
/// takes OUT_P_MSB..OUT_T_LSB (0x01..0x05): pressure a 20-bit Q18.2 in Pa (7.1.3),
/// temperature a 12-bit Q8.4 in degC. Fixed address 0x60.
struct Mpl3115a2 {
    static constexpr std::string_view Name = "MPL3115A2";
    /// NXP MPL3115A2. MPL3115A2.md:670, :969: WHO_AM_I (0x0C) is 0xC4.
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0C, 1, true, 0xFF, 0xC4},
    };
    static constexpr Address7                Address = 0x60;
    static constexpr std::array<Address7, 1> Addresses{0x60};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::array Init{
      Step::write({.reg     = 0x26,
                   .payload = {0x04},
                   .delay   = std::chrono::milliseconds{50},
                   .mayNak  = true}),   // RST
      Step::write({.reg = 0x13, .payload = {0x07}}),
      Step::write(
        {.reg = 0x26, .payload = {0x00}}),   // standby first: OS is writable only with SBYB clear
      Step::write({.reg = 0x26, .payload = {0x38}}),
      Step::write({.reg = 0x26, .payload = {0x39}}),
    };

    struct State {
        std::uint8_t deviceId{};   ///< WHO_AM_I (0x0C)
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint8_t>(ids[0]);
    }

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0}),
                                          Step::check(std::chrono::milliseconds{100}),
                                          Step::read({.reg = 0x01, .count = 5, .offset = 1})};

        [[nodiscard]] static constexpr bool ready(Bytes data) { return (data.u8(0) & 0x08U) != 0; }

        struct Sample {
            Pascal    pressure{};
            CentiDegC temperature{};
        };

        /// Five 0xFF bytes would be 262143.75 Pa at -0.0625 degC: a floating bus, not air.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if(data.be24(1) == 0xFFFFFFU && data.be16(4) == 0xFFFF) {
                return Outcome<Sample>::reject();
            }
            Sample sample{};
            sample.pressure = Units::pascal((data.be24(1) >> 4) / 4);   // Q18.2
            sample.temperature
              = Units::centiDegC(static_cast<std::int32_t>(data.s16be(4) >> 4) * 100 / 16);
            return Outcome<Sample>::ok(sample);
        }
    };

    using Reads = List<Measurement>;
};

}   // namespace Kvasir::I2C::Chips
