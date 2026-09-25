#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Sensirion SHTC3 (datasheet v4). Fixed address 0x70, 16-bit commands, CRC per word.
/// It sleeps between measurements: every cycle is wake-up 0x3517 (240 us), the
/// measurement command, the read, sleep 0xB098 (5.4). Bring-up: wake, soft reset 0x805D
/// (240 us), wake, ID 0xEFC8 (bits 11 and 5:0 are 0x0807, 5.9), sleep. Measurement 0x7866:
/// normal mode, temperature first, no clock stretching; 12.1 ms max (Table 5). Same
/// conversion as the SHT3x (5.11).
struct Shtc3 {
    static constexpr std::string_view        Name    = "SHTC3";
    static constexpr Address7                Address = 0x70;
    static constexpr std::array<Address7, 1> Addresses{0x70};
    static constexpr std::size_t             RegisterBytes = 0;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{1};

    static constexpr std::array Init{
      Step::command({.payload = {0x35, 0x17}, .delay = std::chrono::milliseconds{1}}
      ),
      Step::command({.payload = {0x80, 0x5D}, .delay = std::chrono::milliseconds{1}}
      ),
      Step::command({.payload = {0x35, 0x17}, .delay = std::chrono::milliseconds{1}}
      ),
      Step::command({.payload = {0xEF, 0xC8}}
      ),
      Step::receive({.count = 3, .offset = 0}
      ),
      Step::command({.payload = {0xB0, 0x98}}
      ),
    };

    struct State {
        std::uint16_t deviceId{};
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.deviceId = data.be16(0);
        return Sensirion::wordOk(data, 0) && (state.deviceId & 0x083F) == 0x0807;
    }

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Step::command({.payload = {0x35, 0x17}, .delay = std::chrono::milliseconds{1}}
          ),
          Step::command({.payload = {0x78, 0x66}, .delay = std::chrono::milliseconds{13}}
          ),
          Step::receive({.count = 6, .offset = 0}
          ),
          Step::command({.payload = {0xB0, 0x98}}
          ),
        };

        struct Sample {
            CentiDegC    temperature{};
            CentiPercent humidity{};
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if(!Sensirion::wordOk(data, 0) || !Sensirion::wordOk(data, 3)) {
                return Outcome<Sample>::reject();
            }
            Sample sample{};
            sample.temperature = Units::centiDegC(
              -4500 + static_cast<std::int32_t>((17500LL * data.be16(0)) / 65535));
            sample.humidity
              = Units::centiPercent(static_cast<std::uint32_t>((10000ULL * data.be16(3)) / 65535));
            return Outcome<Sample>::ok(sample);
        }
    };

    using Reads = List<Measurement>;
};

}   // namespace Kvasir::I2C::Chips
