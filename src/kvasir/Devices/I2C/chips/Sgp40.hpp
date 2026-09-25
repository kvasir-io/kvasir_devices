#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Sensirion SGP40 VOC sensor (datasheet v1.2). Fixed address 0x59, 16-bit commands.
/// Bring-up: serial number 0x3682 (three words). Every second: measure_raw_signal 0x260F
/// with the default humidity and temperature compensation words (50 %RH is 0x8000, 25 degC
/// 0x6666, each framed with its CRC), 30 ms max, one word: the raw SRAW_VOC ticks.
/// Sensirion's VOC index algorithm is not part of this.
struct Sgp40 {
    static constexpr std::string_view        Name    = "SGP40";
    static constexpr Address7                Address = 0x59;
    static constexpr std::array<Address7, 1> Addresses{0x59};
    static constexpr std::size_t             RegisterBytes = 0;

    static constexpr std::array Init{
      Step::command({.payload = {0x36, 0x82}, .delay = std::chrono::milliseconds{1}}
      ),
      Step::receive({             .count = 9,                           .offset = 0}
      ),
    };

    struct State {
        std::uint64_t serial{};
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.serial = (static_cast<std::uint64_t>(data.be16(0)) << 32)
                     | (static_cast<std::uint64_t>(data.be16(3)) << 16) | data.be16(6);
        return Sensirion::wordOk(data, 0) && Sensirion::wordOk(data, 3)
            && Sensirion::wordOk(data, 6);
    }

    /// The default compensation: 50 %RH and 25 degC as the datasheet's ticks, with the CRC
    /// after each word (Sensirion::framed).
    static constexpr std::array<std::uint8_t, 3> DefaultHumidity    = Sensirion::framed(0x8000);
    static constexpr std::array<std::uint8_t, 3> DefaultTemperature = Sensirion::framed(0x6666);

    struct Raw {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Step::command({.payload = {0x26,
0x0F,
DefaultHumidity[0],
DefaultHumidity[1],
DefaultHumidity[2],
DefaultTemperature[0],
DefaultTemperature[1],
DefaultTemperature[2]},
                         .delay   = std::chrono::milliseconds{30}                                     }
          ),
          Step::receive({                                                 .count = 3,      .offset = 0}
          ),
        };

        struct Sample {
            std::uint16_t ticks{};
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if(!Sensirion::wordOk(data, 0)) { return Outcome<Sample>::reject(); }
            return Outcome<Sample>::ok({data.be16(0)});
        }
    };

    using Reads = List<Raw>;
};

}   // namespace Kvasir::I2C::Chips
