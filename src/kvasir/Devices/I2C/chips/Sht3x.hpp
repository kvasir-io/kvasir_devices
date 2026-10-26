#pragma once

#include "../../Engine/Description.hpp"
#include "../Address.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Sht3xDetail {
    /// The internal heater (4.10).
    enum class Heater : std::uint8_t { off, on };
}   // namespace Sht3xDetail

/// Sensirion SHT30/31/35 (SHT3x-DIS datasheet v7). No registers: 16-bit commands, a
/// bare read of the result, a CRC after every 16-bit word (ch. 4). Bring-up: soft reset
/// 0x30A2 (1.5 ms), status 0xF32D read as 3 bytes. Measurement: 0x2400 (single shot, high
/// repeatability, no clock stretching), 15.5 ms, 6 bytes: T, CRC, RH, CRC (4.4).
/// T = -45 + 175 * S / 65535 degC, RH = 100 * S / 65535 % (4.13; Linux sht3x.c divides by
/// 65536). 0x44 / 0x45 (ADDR).
///
/// The soft reset is only specified from idle (4.9), and a part left in periodic mode by earlier
/// firmware is not idle, so the bring-up opens with Break 0x3093 (4.8), which stops periodic
/// data acquisition and is harmless on an idle part. Power-up takes up to 1.5 ms (tPU, 4.1).
struct Sht3x {
    static constexpr std::string_view        Name    = "SHT3x";
    static constexpr Address7                Address = 0x44;
    static constexpr std::array<Address7, 2> Addresses{0x44, 0x45};
    static constexpr std::size_t             RegisterBytes = 0;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{2};

    // clang-format off
    static constexpr std::array Init{
      Engine::Step::command(
        {.payload = {0x30, 0x93}, .delay = std::chrono::milliseconds{2}}),   // Break
      Engine::Step::command({.payload = {0x30, 0xA2}, .delay = std::chrono::milliseconds{2}}),
      Engine::Step::command({.payload = {0xF3, 0x2D}}),
      Engine::Step::receive({.count = 3, .offset = 0}),
    };
    // clang-format on

    struct State {
        std::uint16_t status{};
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.status = data.be16(0);
        return Sensirion::wordOk(data, 0);
    }

    struct Measurement {
        static constexpr auto Period = std::chrono::milliseconds{1000};
        // clang-format off
        static constexpr std::array Steps{
          Engine::Step::command({.payload = {0x24, 0x00}, .delay = std::chrono::milliseconds{16}}),
          Engine::Step::receive({.count = 6, .offset = 0})};
        // clang-format on

        struct Sample {
            CentiDegC    temperature{};   ///< 0.01 degC
            CentiPercent humidity{};      ///< 0.01 %RH
        };

        [[nodiscard]] static constexpr Engine::Outcome<Sample> decode(Bytes data) {
            if(!Sensirion::wordOk(data, 0) || !Sensirion::wordOk(data, 3)) {
                return Engine::Outcome<Sample>::reject();
            }
            Sample sample{};
            sample.temperature = Units::centiDegC(Sensirion::centiDegC(data.be16(0)));
            sample.humidity    = Units::centiPercent(Sensirion::centiPercentRh(data.be16(3)));
            return Engine::Outcome<Sample>::ok(sample);
        }
    };

    /// The internal heater, "meant for plausibility checking only" (4.10): 0x306D on, 0x3066 off,
    /// a few degrees of rise depending on the supply (3.6 .. 33 mW, Table 2) and the humidity
    /// reading falling with it. No Initial: the soft reset of the bring-up leaves it off, and a
    /// part that was reset under a heater the application had set gets it set again, like any
    /// write group. An application that turns it on turns it off: nothing here times it.
    struct Heater {
        using Value                        = Sht3xDetail::Heater;
        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr Engine::Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
            buffer[0] = std::byte{0x30};
            buffer[1] = value == Value::on ? std::byte{0x6D} : std::byte{0x66};
            return Engine::Step::commandBuffer({.offset = 0, .count = 2});
        }
    };

    using Reads  = Engine::List<Measurement>;
    using Writes = Engine::List<Heater>;
};

}   // namespace Kvasir::I2C::Chips
