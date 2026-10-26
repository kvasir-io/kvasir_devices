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

    // clang-format off
    static constexpr std::array Init{
      Engine::Step::command({.payload = {0x35, 0x17}, .delay = std::chrono::milliseconds{1}}),
      Engine::Step::command({.payload = {0x80, 0x5D}, .delay = std::chrono::milliseconds{1}}),
      Engine::Step::command({.payload = {0x35, 0x17}, .delay = std::chrono::milliseconds{1}}),
      Engine::Step::command({.payload = {0xEF, 0xC8}}),
      Engine::Step::receive({.count = 3, .offset = 0}),
      Engine::Step::command({.payload = {0xB0, 0x98}}),
    };
    // clang-format on

    struct State {
        std::uint16_t deviceId{};
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.deviceId = data.be16(0);
        return Sensirion::wordOk(data, 0) && (state.deviceId & 0x083F) == 0x0807;
    }

    struct Measurement {
        static constexpr auto Period = std::chrono::milliseconds{1000};
        // clang-format off
        static constexpr std::array Steps{
          Engine::Step::command({.payload = {0x35, 0x17}, .delay = std::chrono::milliseconds{1}}),
          Engine::Step::command({.payload = {0x78, 0x66}, .delay = std::chrono::milliseconds{13}}),
          Engine::Step::receive({.count = 6, .offset = 0}),
          Engine::Step::command({.payload = {0xB0, 0x98}}),
        };
        // clang-format on

        struct Sample {
            CentiDegC    temperature{};
            CentiPercent humidity{};
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

    using Reads = Engine::List<Measurement>;
};

}   // namespace Kvasir::I2C::Chips
