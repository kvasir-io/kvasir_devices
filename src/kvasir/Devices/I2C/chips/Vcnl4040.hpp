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

/// Vishay VCNL4040 proximity and ambient light sensor (datasheet 84274). Fixed address
/// 0x60, one-byte command code, 16-bit little-endian registers. Bring-up: ID 0x0C =
/// 0x0186 (low byte 0x86, high byte 0x01); ALS_CONF 0x00 = 0 (80 ms integration: 0.1 lx/step, power on); PS_CONF1/2
/// 0x03 = 0 (proximity on, 1T, 12 bit); PS_CONF3/MS 0x04 = 0. Data: PS 0x08, ALS 0x09,
/// WHITE 0x0A (Table 13).
struct Vcnl4040 {
    static constexpr std::string_view Name = "VCNL4040";
    /// Vishay VCNL4040. VCNL4040.md:442..443, :620..621: command code 0x0C reads ID_L 0x86 then ID_M
    /// 0x01, low byte first.
    static constexpr std::array Identity{
      RegisterCheck{"id", 0x0C, 2, false, 0xFFFF, 0x0186},
    };
    static constexpr Address7                Address = 0x60;
    static constexpr std::array<Address7, 1> Addresses{0x60};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::uint16_t DeviceId = 0x0186;

    static constexpr std::array Init{
      Step::write({.reg = 0x00, .payload = {0x00, 0x00}}
      ),
      Step::write({.reg = 0x03, .payload = {0x00, 0x00}}
      ),
      Step::write({.reg = 0x04, .payload = {0x00, 0x00}, .delay = std::chrono::milliseconds{100}}
      ),
    };

    struct State {
        std::uint16_t deviceId{};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
        // ID_L 0x86 is the part; ID_M is 00 and the address option 00 (0x60) in bits 7:4, and
        // a version code in bits 3:0 (Table "ID"), so a later silicon version is not turned
        // down -- Linux vcnl4000.c holds to ID_L alone.
    }

    struct Light {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{Step::read({.reg = 0x08, .count = 2, .offset = 0}),
                                          Step::read({.reg = 0x09, .count = 2, .offset = 2}),
                                          Step::read({.reg = 0x0A, .count = 2, .offset = 4})};

        struct Sample {
            std::uint16_t proximity{};
            std::uint16_t als{};   ///< 0.1 lx per count at 80 ms
            std::uint16_t white{};

            [[nodiscard]] constexpr MilliLux lux() const {
                return Units::milliLux(static_cast<std::uint32_t>(als) * 100U);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {data.le16(0), data.le16(2), data.le16(4)};
        }
    };

    using Reads = List<Light>;
};

}   // namespace Kvasir::I2C::Chips
