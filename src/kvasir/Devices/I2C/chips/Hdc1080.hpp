#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Texas Instruments HDC1080 (SNAS672A). Fixed address 0x40, one-byte pointer, 16-bit
/// big-endian registers. Bring-up (15 ms after power): manufacturer 0xFE = 0x5449, device
/// 0xFF = 0x1050, configuration 0x02 = 0x1000 (temperature then humidity, both 14 bit,
/// 8.5.1.3). A measurement is triggered by writing the pointer 0x00 alone, takes 6.35 +
/// 6.5 ms, and is read as four bytes with no pointer: T msb lsb, RH msb lsb (8.5.1.3).
/// T = raw / 2^16 * 165 - 40, RH = raw / 2^16 * 100 (8.6.1, 8.6.2).
struct Hdc1080 {
    static constexpr std::string_view        Name    = "HDC1080";
    static constexpr Address7                Address = 0x40;
    static constexpr std::array<Address7, 1> Addresses{0x40};
    static constexpr std::size_t             RegisterBytes = 1;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{15};

    static constexpr std::array Init{
      Step::read({.reg = 0xFE, .count = 2, .offset = 0}
      ),
      Step::read({.reg = 0xFF, .count = 2, .offset = 2}
      ),
      Step::identify(),
      Step::write({.reg = 0x02, .payload = {0x10, 0x00}}
      ),
    };

    struct State {
        std::uint16_t manufacturer{};
        std::uint16_t deviceId{};   ///< Device ID (0xFF)
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.manufacturer = data.be16(0);
        state.deviceId     = data.be16(2);
        return state.manufacturer == 0x5449 && state.deviceId == 0x1050;
    }

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Step::write({.reg = 0x00, .payload = {}, .delay = std::chrono::milliseconds{20}}),
          Step::receive({.count = 4, .offset = 0})};

        struct Sample {
            CentiDegC    temperature{};
            CentiPercent humidity{};
        };

        /// Both words 0xFFFF is 125 degC at 100 %RH, outside what the part measures: a read
        /// that came back before the conversion finished (the part NAKs then, but a bus that
        /// floats high reads as ones) is rejected rather than reported.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if(data.be16(0) == 0xFFFF && data.be16(2) == 0xFFFF) {
                return Outcome<Sample>::reject();
            }
            Sample sample{};
            sample.temperature
              = Units::centiDegC(static_cast<std::int32_t>((16500LL * data.be16(0)) >> 16) - 4000);
            sample.humidity
              = Units::centiPercent(static_cast<std::uint32_t>((10000ULL * data.be16(2)) >> 16));
            return Outcome<Sample>::ok(sample);
        }
    };

    using Reads = List<Measurement>;
};

}   // namespace Kvasir::I2C::Chips
