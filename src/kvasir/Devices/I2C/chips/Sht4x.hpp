#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Sensirion SHT40/41/45 (SHT4x datasheet v6.4). No registers: one-byte commands, six
/// byte results with a CRC after each word (4.3, 4.4). Bring-up: soft reset 0x94 (1 ms),
/// serial number 0x89 (two words). Measurement: 0xFD high precision, 8.3 ms max (Table 5),
/// T msb lsb crc, RH msb lsb crc. T = -45 + 175 S / 65535, RH = -6 + 125 S / 65535 (4.6;
/// clamped to 0..100, the datasheet's N.B.). 0x44 (SHT40-AD1B, SHT41, SHT45), 0x45
/// (-BD1B), 0x46 (-CD1B).
struct Sht4x {
    static constexpr std::string_view        Name    = "SHT4x";
    static constexpr Address7                Address = 0x44;
    static constexpr std::array<Address7, 3> Addresses{0x44, 0x45, 0x46};
    static constexpr std::size_t             RegisterBytes = 0;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{1};

    static constexpr std::array Init{
      Step::command({.payload = {0x94}, .delay = std::chrono::milliseconds{1}}),
      Step::command({.payload = {0x89}, .delay = std::chrono::milliseconds{1}}),
      Step::receive({.count = 6, .offset = 0}),
    };

    struct State {
        std::uint32_t serial{};
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.serial = (static_cast<std::uint32_t>(data.be16(0)) << 16) | data.be16(3);
        return Sensirion::wordOk(data, 0) && Sensirion::wordOk(data, 3);
    }

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Step::command({.payload = {0xFD}, .delay = std::chrono::milliseconds{9}}),
          Step::receive({.count = 6, .offset = 0})};

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
            auto const rh   = -600 + static_cast<std::int32_t>((12500LL * data.be16(3)) / 65535);
            sample.humidity = Units::centiPercent(
              static_cast<std::uint32_t>(rh < 0 ? 0 : (rh > 10000 ? 10000 : rh)));
            return Outcome<Sample>::ok(sample);
        }
    };

    /// One measurement at the end of a heater pulse (4.9, Table 8): command 0x24, 110 mW for
    /// 0.1 s (0.11 s at most, Table 5) with a high-precision measurement just before the heater
    /// goes off, so the reading is the heated one. On demand only -- `request<Heated>()` -- and
    /// never cyclic: the data sheet limits the heater's duty cycle to 10 %, and the part needs a
    /// while to be back at ambient. For a plausibility check of the sensor (the temperature
    /// rises, the humidity falls) and for driving off condensation.
    struct Heated {
        static constexpr std::array Steps{
          Step::command({.payload = {0x24}, .delay = std::chrono::milliseconds{120}}),
          Step::receive({.count = 6, .offset = 0})};

        using Sample = Measurement::Sample;

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            return Measurement::decode(data);
        }
    };

    /// What latest() and fresh() mean without a group name.
    using Primary = Measurement;

    using Reads = List<Measurement, Heated>;
};

}   // namespace Kvasir::I2C::Chips
