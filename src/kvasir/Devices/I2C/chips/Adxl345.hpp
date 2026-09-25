#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Analog Devices ADXL345 (Rev. 0). One-byte registers, multi-byte reads auto-increment.
/// Bring-up: DEVID 0x00 = 0xE5; DATA_FORMAT 0x31 = 0x08 (full resolution, 2 g: 256 LSB/g,
/// about 4 mg/LSB, at every range); BW_RATE 0x2C = 0x0A (100 Hz); POWER_CTL 0x2D = 0x08
/// (measure). Data: INT_SOURCE 0x30 (DATA_READY bit 7), then six bytes from DATAX0 0x32,
/// int16 little-endian, in one burst. A frame without DATA_READY is Outcome::unchanged();
/// six bytes of 0xFF -- minus one count, 4 mg, on every axis at once, which is not a
/// reading a part in gravity gives -- is a bus that answered nothing and is rejected.
/// 0x53 (SDO low) or 0x1D.
struct Adxl345 {
    static constexpr std::string_view Name = "ADXL345";
    /// Analog Devices ADXL345. ADXL345.md:614, :661: DEVID (0x00) is 0xE5.
    static constexpr std::array Identity{
      RegisterCheck{"devid", 0x00, 1, true, 0xFF, 0xE5},
    };
    static constexpr Address7                Address = 0x53;
    static constexpr std::array<Address7, 2> Addresses{0x53, 0x1D};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::array Init{
      Step::write({.reg = 0x31, .payload = {0x08}}),
      Step::write({.reg = 0x2C, .payload = {0x0A}}),
      Step::write({.reg = 0x2D, .payload = {0x08}}),
    };

    using State = Groups::DeviceId;

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    struct Motion {
        static constexpr auto       Period = std::chrono::milliseconds{20};
        static constexpr std::array Steps{
          Step::read({.reg = 0x30, .count = 8})};   // INT_SOURCE, 0x31, DATAX0..DATAZ1

        struct Sample {
            MicroG x{}, y{}, z{};
        };

        /// 256 counts per g is 1'000'000 / 256 = 15625 / 4 ug a count, exact.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data,
                                                              Sample const&) {
            if((data.u8(0) & 0x80U) == 0) { return Outcome<Sample>::unchanged(); }   // DATA_READY
            bool allOnes = true;
            for(std::size_t i = 2; i < 8; ++i) { allOnes = allOnes && data.u8(i) == 0xFF; }
            if(allOnes) { return Outcome<Sample>::reject(); }
            auto const axis = [&](std::size_t i) {
                return Units::microG(static_cast<std::int32_t>(data.s16le(2 + 2 * i)) * 15625 / 4);
            };
            return Outcome<Sample>::ok({axis(0), axis(1), axis(2)});
        }
    };

    using Reads = List<Motion>;
};

}   // namespace Kvasir::I2C::Chips
