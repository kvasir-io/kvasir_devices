#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Sensirion SCD40 / SCD41 CO2 sensor (SCD4x datasheet v1.5). Fixed address 0x62, 16-bit
/// commands, words with CRC (3.2). Bring-up: stop_periodic_measurement 0x3F86 (500 ms; a
/// warm part may be running), get_serial_number 0x3682 (three words),
/// start_periodic_measurement 0x21B1: a new value every 5 s. Reading: get_data_ready_status
/// 0xE4B8 (a word whose low 11 bits are non-zero when data is there; polled again every
/// second, so the engine's eight retries outlast the 5 s the first measurement takes), then
/// read_measurement 0xEC05: CO2 ppm, T = -45 + 175 word / 65535, RH = 100 word / 65535 (Linux
/// scd4x.c divides by 65536)
/// (3.5.2; the datasheet's example 0x01F4 0x6667 0x5EB9 is 500 ppm, 25 degC, 37 % -- its
/// printed CRC for the first word, 0x7B, is a typo: the polynomial gives 0x33).
struct Scd4x {
    static constexpr std::string_view        Name    = "SCD4x";
    static constexpr Address7                Address = 0x62;
    static constexpr std::array<Address7, 1> Addresses{0x62};
    static constexpr std::size_t             RegisterBytes = 0;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{1000};

    static constexpr std::array Init{
      Step::command({.payload = {0x3F, 0x86}, .delay = std::chrono::milliseconds{500}}
      ),
      Step::command({.payload = {0x36, 0x82},   .delay = std::chrono::milliseconds{1}}
      ),
      Step::receive({             .count = 9,                             .offset = 0}
      ),
      Step::command({.payload = {0x21, 0xB1},  .delay = std::chrono::milliseconds{10}}
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

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{5000};
        static constexpr std::array Steps{
          Step::command({.payload = {0xE4, 0xB8}, .delay = std::chrono::milliseconds{1}}
          ),
          Step::receive({.count = 3, .offset = 0}
          ),
          Step::check(std::chrono::milliseconds{1000}
          ),
          Step::command({.payload = {0xEC, 0x05}, .delay = std::chrono::milliseconds{1}}
          ),
          Step::receive({.count = 9, .offset = 3}
          ),
        };

        [[nodiscard]] static constexpr bool ready(Bytes data) {
            return Sensirion::wordOk(data, 0) && (data.be16(0) & 0x07FFU) != 0;
        }

        struct Sample {
            Ppm          co2{};
            CentiDegC    temperature{};
            CentiPercent humidity{};
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if(!Sensirion::wordOk(data, 3) || !Sensirion::wordOk(data, 6)
               || !Sensirion::wordOk(data, 9))
            {
                return Outcome<Sample>::reject();
            }
            Sample sample{};
            sample.co2         = Units::ppm(data.be16(3));
            sample.temperature = Units::centiDegC(
              -4500 + static_cast<std::int32_t>((17500LL * data.be16(6)) / 65535));
            sample.humidity
              = Units::centiPercent(static_cast<std::uint32_t>((10000ULL * data.be16(9)) / 65535));
            return Outcome<Sample>::ok(sample);
        }
    };

    using Reads = List<Measurement>;
};

}   // namespace Kvasir::I2C::Chips
