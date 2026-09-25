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

/// Texas Instruments INA3221 triple-channel, 0..26 V bus and shunt monitor. One-byte
/// pointer, 16-bit big-endian registers: configuration 0x00 (reset 0x7127), then shunt and
/// bus voltage interleaved per channel at 0x01..0x06, the alert limits at 0x07..0x0C, the
/// shunt sum at 0x0D, mask/enable 0x0F, manufacturer id 0xFE (0x5449) and die id 0xFF
/// (0x3220).
///
/// Both measurements put their value in bits 15:3 (full scale reads 0x7FF8), so the code is
/// the register shifted right by three: 40 uV per shunt count and 8 mV per bus count.
/// Unlike the INA226 family there is no calibration register and no current register -- the
/// part reports shunt voltage and the current is worked out here, which is why the shunt
/// value is a template parameter. `Ina3221<>` is the 50 mOhm the +-3.2 A breakout uses,
/// read every `Timing::Period` (200 ms; at the reset configuration a full round of the three
/// channels takes 6.6 ms). 0x40..0x43 by the A0 pin.
template<MicroOhm Shunt = Units::microOhm(50000), typename Timing = DefaultTiming>
struct Ina3221 {
    static constexpr std::string_view Name = "INA3221";
    /// TI INA3221. INA3221.md:914..915, Manufacturer ID FEh = 5449h and Die ID FFh = 3220h.
    /// Configuration (00h, INA3221.md:939..998): RST clear, CH1en..CH3en set, MODE3..1 = 111b, shunt
    /// and bus continuous.
    static constexpr std::array Identity{
      RegisterCheck{"manufacturer-id", 0xFE, 2, true, 0xFFFF, 0x5449},
      RegisterCheck{         "die-id", 0xFF, 2, true, 0xFFFF, 0x3220},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"channels-continuous", 0x00, 2, true, 0xF007, 0x7007},
    };
    static constexpr Address7    Address       = 0x40;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 4> Addresses{0x40, 0x41, 0x42, 0x43};

    static_assert(Shunt >= Units::microOhm(77),
                  "the shunt value is what the current is worked out from: below 77 uOhm the "
                  "full-scale current is past the 32 bits a MicroAmp holds");
    static constexpr std::chrono::milliseconds ReadPeriod = [] {
        if constexpr(requires { Timing::Period; }) {
            return Kvasir::asDuration(Timing::Period);
        } else {
            return std::chrono::milliseconds{200};
        }
    }();
    static_assert(ReadPeriod > std::chrono::milliseconds::zero(),
                  "Timing::Period is how often Power is read; 0 never reads it");

    static constexpr std::size_t Channels = 3;

    struct State {
        std::uint16_t manufacturer{};
        std::uint16_t deviceId{};   ///< Die ID (0xFF)
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.manufacturer = static_cast<std::uint16_t>(ids[0]);
        state.deviceId     = static_cast<std::uint16_t>(ids[1]);
    }

    struct Power {
        static constexpr auto Period = ReadPeriod;

        /// Shunt and bus interleaved, channel by channel; the pointer does not
        /// auto-increment, so each register is its own read.
        static constexpr std::array Steps{
          Step::read({.reg = 0x01, .count = 2, .offset = 0}),
          Step::read({.reg = 0x02, .count = 2, .offset = 2}),
          Step::read({.reg = 0x03, .count = 2, .offset = 4}),
          Step::read({.reg = 0x04, .count = 2, .offset = 6}),
          Step::read({.reg = 0x05, .count = 2, .offset = 8}),
          Step::read({.reg = 0x06, .count = 2, .offset = 10}),
        };

        struct Sample {
            std::array<NanoVolt, Channels>  shuntVoltage{};
            std::array<MilliVolt, Channels> busVoltage{};
            std::array<MicroAmp, Channels>  current{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            for(std::size_t i = 0; i < Channels; ++i) {
                auto const shunt       = static_cast<std::int32_t>(data.s16be(4 * i) >> 3);
                auto const bus         = static_cast<std::int32_t>(data.s16be(4 * i + 2) >> 3);
                sample.shuntVoltage[i] = Units::nanoVolt(shunt * 40000);   // 40 uV per count
                sample.busVoltage[i]   = Units::milliVolt(bus * 8);        // 8 mV per count
                // I[uA] = V[nV] * 1000 / R[uOhm]
                sample.current[i] = Units::microAmp(
                  static_cast<std::int32_t>(static_cast<std::int64_t>(shunt) * 40'000'000LL
                                            / static_cast<std::int64_t>(Units::value(Shunt))));
            }
            return sample;
        }
    };

    struct Config {
        using Value                        = std::uint16_t;
        static constexpr std::size_t Bytes = 2;
        /// The reset configuration: all three channels on, 1.1 ms conversions, 1 average.
        /// Written after every bring-up like any Initial: a board that wires the TC pin should
        /// know that any configuration write before the power-up timing-control sequence has
        /// run its course disables that alert until a power cycle or a reset (7.3.2.4).
        static constexpr Value Initial = 0x7127;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            putBe16(buffer, 0, value);
            return Step::writeBuffer({.reg = 0x00, .offset = 0, .count = 2});
        }
    };

    /// Critical then warning limit, per channel: 0x07..0x0C, in shunt counts of 40 uV.
    struct Limits {
        using Value                        = NanoVolt;   ///< across the shunt
        static constexpr std::size_t Items = 6;
        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr Step encode(Value const&         limit,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            // 13 bits signed: -4096..4095 counts, so a limit past +-163.8 mV clamps rather than
            // wrapping into one of the other sign
            auto const raw = Units::value(limit) / 40000;
            auto const code
              = static_cast<std::int16_t>(raw > 4095 ? 4095 : (raw < -4096 ? -4096 : raw));
            putBe16(buffer, 0, static_cast<std::uint16_t>(static_cast<std::uint16_t>(code) << 3));
            return Step::writeBuffer(
              {.reg = static_cast<std::uint16_t>(0x07 + item), .offset = 0, .count = 2});
        }
    };

    /// Limits item indices: channel 1 critical is 0, channel 1 warning 1, and so on.
    [[nodiscard]] static constexpr std::size_t critical(std::size_t channel) { return 2 * channel; }

    [[nodiscard]] static constexpr std::size_t warning(std::size_t channel) {
        return 2 * channel + 1;
    }

    using Reads  = List<Power>;
    using Writes = List<Config, Limits>;
};

}   // namespace Kvasir::I2C::Chips
