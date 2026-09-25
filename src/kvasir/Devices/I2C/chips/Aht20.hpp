#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Aosong AHT20 (data sheet V1.0, 2021). Fixed address 0x38, no registers (7.3, 7.4).
/// 100 ms after power: soft reset 0xBA (20 ms), initialisation 0xBE 0x08 0x00 (10 ms).
/// Neither command is in data sheet V1.0, whose 7.4 checks status & 0x18 and otherwise
/// initialises registers 0x1B, 0x1C and 0x1E per the vendor routine; 0xBA and 0xBE come
/// from Aosong's reference routine and earlier documentation, and sending them always is
/// what the reference drivers do. Then the status byte is read and checked for CAL (bit 3).
/// Measurement: 0xAC 0x33 0x00, 80 ms, 7 bytes: status, RH[19:12], RH[11:4], RH[3:0] T[19:16],
/// T[15:8], T[7:0], CRC-8 (poly 0x31, init 0xFF) over the six. Busy (status bit 7) means read
/// again later. RH = S / 2^20 * 100 %, T = S / 2^20 * 200 - 50 degC (8.1, 8.2).
struct Aht20 {
    static constexpr std::string_view        Name    = "AHT20";
    static constexpr Address7                Address = 0x38;
    static constexpr std::array<Address7, 1> Addresses{0x38};
    static constexpr std::size_t             RegisterBytes = 0;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{100};

    static constexpr std::array Init{
      Step::command({            .payload = {0xBA}, .delay = std::chrono::milliseconds{20}}
      ),
      Step::command({.payload = {0xBE, 0x08, 0x00}, .delay = std::chrono::milliseconds{10}}
      ),
      Step::receive({                   .count = 1,                            .offset = 0}
      ),
    };

    struct State {
        std::uint8_t status{};
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.status = data.u8(0);
        // 7.4: bits 4 and 3 both set; otherwise registers 1Bh, 1Ch and 1Eh need initialising,
        // which the DHT20's Restore does
        return (state.status & 0x18U) == 0x18U;
    }

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Step::command({.payload = {0xAC, 0x33, 0x00}, .delay = std::chrono::milliseconds{80}}
          ),
          Step::receive({                   .count = 7,                            .offset = 0}
          )
        };

        struct Sample {
            CentiDegC    temperature{};
            CentiPercent humidity{};
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if((data.u8(0) & 0x80U) != 0) {
                return Outcome<Sample>::retry(std::chrono::milliseconds{10});
            }
            if(Sensirion::crc8(data.sub(0, 6)) != data.u8(6)) { return Outcome<Sample>::reject(); }
            std::uint32_t const rh = (static_cast<std::uint32_t>(data.u8(1)) << 12)
                                   | (static_cast<std::uint32_t>(data.u8(2)) << 4)
                                   | (data.u8(3) >> 4);
            std::uint32_t const t  = (static_cast<std::uint32_t>(data.u8(3) & 0x0FU) << 16)
                                   | (static_cast<std::uint32_t>(data.u8(4)) << 8) | data.u8(5);
            Sample              sample{};
            sample.humidity = Units::centiPercent(
              static_cast<std::uint32_t>((static_cast<std::uint64_t>(rh) * 10000U) >> 20));
            sample.temperature = Units::centiDegC(
              static_cast<std::int32_t>((static_cast<std::uint64_t>(t) * 20000U) >> 20) - 5000);
            return Outcome<Sample>::ok(sample);
        }
    };

    using Reads = List<Measurement>;
};

}   // namespace Kvasir::I2C::Chips
