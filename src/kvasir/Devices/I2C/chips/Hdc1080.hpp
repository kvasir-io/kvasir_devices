#pragma once

#include "../../Engine/Description.hpp"
#include "../Address.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Texas Instruments HDC1080 (SNAS672A). Fixed address 0x40, one-byte pointer, 16-bit
/// big-endian registers. Bring-up (15 ms after power): manufacturer 0xFE = 0x5449, device
/// 0xFF = 0x1050, configuration 0x02 = 0x1000 (temperature then humidity, both 14 bit,
/// 8.5.1.3). A measurement is triggered by writing the pointer 0x00 alone, takes 6.35 +
/// 6.5 ms, and is read as four bytes with no pointer: T msb lsb, RH msb lsb (8.5.1.3).
/// T = raw / 2^16 * 165 - 40, RH = raw / 2^16 * 100 (8.6.1, 8.6.2).
struct Hdc1080 {
    static constexpr std::string_view Name = "HDC1080";
    /// "The manufacturer ID reads 0x5449" (8.6.5, Table 8), "A reading of the Device ID register
    /// returns 0x1050" (8.6.6, Table 9); i2c/HDC1080.md l.628-629, l.728, l.752. The engine reads
    /// both before anything is written: a part with another identity at 0x40 gets nothing.
    static constexpr std::array Identity{
      Engine::RegisterCheck{"manufacturer-id", 0xFE, 2, true, 0xFFFF, 0x5449},
      Engine::RegisterCheck{      "device-id", 0xFF, 2, true, 0xFFFF, 0x1050},
    };
    static constexpr Address7                Address = 0x40;
    static constexpr std::array<Address7, 1> Addresses{0x40};
    static constexpr std::size_t             RegisterBytes = 1;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{15};

    // clang-format off
    static constexpr std::array Init{
      Engine::Step::write({.reg = 0x02, .payload = {0x10, 0x00}}),
    };
    // clang-format on

    struct State {
        std::uint16_t manufacturer{};
        std::uint16_t deviceId{};   ///< Device ID (0xFF)
    };

    /// What the engine read of the Identity above, once it matched and before Init runs.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.manufacturer = static_cast<std::uint16_t>(ids[0]);
        state.deviceId     = static_cast<std::uint16_t>(ids[1]);
    }

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Engine::Step::write({.reg = 0x00, .payload = {}, .delay = std::chrono::milliseconds{20}}),
          Engine::Step::receive({.count = 4, .offset = 0})};

        struct Sample {
            CentiDegC    temperature{};
            CentiPercent humidity{};
        };

        /// Both words 0xFFFF is 125 degC at 100 %RH, outside what the part measures: a read
        /// that came back before the conversion finished (the part NAKs then, but a bus that
        /// floats high reads as ones) is rejected rather than reported.
        [[nodiscard]] static constexpr Engine::Outcome<Sample> decode(Bytes data) {
            if(data.be16(0) == 0xFFFF && data.be16(2) == 0xFFFF) {
                return Engine::Outcome<Sample>::reject();
            }
            Sample sample{};
            sample.temperature
              = Units::centiDegC(static_cast<std::int32_t>((16500LL * data.be16(0)) >> 16) - 4000);
            sample.humidity
              = Units::centiPercent(static_cast<std::uint32_t>((10000ULL * data.be16(2)) >> 16));
            return Engine::Outcome<Sample>::ok(sample);
        }
    };

    using Reads = Engine::List<Measurement>;
};

}   // namespace Kvasir::I2C::Chips
