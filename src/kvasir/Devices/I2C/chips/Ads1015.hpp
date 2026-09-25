#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"
#include "Ads1115.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Ads1015Detail {
    /// One single-shot conversion of AINn: the config write, then 2 ms to be safely past
    /// the 0.625 ms conversion at DR 100b (1600 SPS on the ADS101x).
    [[nodiscard]] constexpr Step start(unsigned            channel,
                                       Ads1x15Detail::Gain gain) {
        auto const cfg = Ads1x15Detail::config(channel, gain);
        return Step::write({
          .reg     = 0x01,
          .payload = {static_cast<std::uint8_t>(cfg >> 8), static_cast<std::uint8_t>(cfg & 0xFF)},
          .delay   = std::chrono::milliseconds{2}
        });
    }
}   // namespace Ads1015Detail

/// Texas Instruments ADS1015 (SBAS473), the 12-bit sibling of the ADS1115: the same two
/// registers behind a one-byte pointer (00b Conversion, 01b Config) and the same Config
/// layout, but the result is 12 bits two's complement *left-justified* in the 16-bit
/// Conversion register (bits 15:4, +FS = 7FFh) and DR 100b is 1600 SPS rather than 128, so
/// a conversion is 0.625 ms instead of 7.8 ms. All four channels in one sweep every 100 ms.
/// The PGA range is the `Gain` parameter (Ads1x15Detail::Gain, +-2.048 V by default).
/// 0x48..0x4B by the ADDR pin.
template<Ads1x15Detail::Gain Gain = Ads1x15Detail::Gain::fsr2V048>
struct Ads1015 {
    static constexpr std::string_view Name = "ADS1015";
    /// TI ADS1115 and ADS1015. The config register (01h) while the description sweeps the four
    /// inputs with single-shot conversions (ADS1115.md:1031..1058): MUX bit 14 set -- an input against
    /// GND, whichever of the four the sweep is at --, PGA 11:9 = 010b, +-2.048 V, which is what the
    /// bench's gain and the description's volts per count assume (:669..673), MODE, bit 8, single-shot,
    /// and COMP_QUE 1:0 = 11b, the comparator off. The parts have no identity register. The ADS1219 is
    /// not read this way: its RDATA command and the read after it are two transactions, and a register
    /// read between them would be taken for the conversion result.
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"single-ended-2v048-single-shot", 0x01, 2, true, 0x4F03, 0x4503},
    };
    static constexpr Address7                Address = 0x48;
    static constexpr std::array<Address7, 4> Addresses{0x48, 0x49, 0x4A, 0x4B};
    static constexpr std::size_t             RegisterBytes = 1;

    /// +FSR, what a code of 0x7FF + 1 would be.
    static constexpr MicroVolt FullScale
      = Units::microVolt(Ads1x15Detail::fullScaleMicroVolt(Gain));

    struct Sweep {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{
          Ads1015Detail::start(0, Gain),
          Step::read({.reg = 0x00, .count = 2, .offset = 0}),
          Ads1015Detail::start(1, Gain),
          Step::read({.reg = 0x00, .count = 2, .offset = 2}),
          Ads1015Detail::start(2, Gain),
          Step::read({.reg = 0x00, .count = 2, .offset = 4}),
          Ads1015Detail::start(3, Gain),
          Step::read({.reg = 0x00, .count = 2, .offset = 6}),
        };

        struct Sample {
            std::array<std::int16_t, 4> code{};   ///< the 12-bit code, sign extended

            /// A code at the configured full-scale range: 2048 counts is +FSR.
            [[nodiscard]] static constexpr MicroVolt toVoltage(std::int16_t c) {
                return Units::microVolt(static_cast<std::int32_t>(
                  static_cast<std::int64_t>(c) * Units::value(FullScale) / 2048));
            }

            /// Channel `ch`.
            [[nodiscard]] constexpr MicroVolt voltage(std::size_t ch) const {
                return toVoltage(code[ch]);
            }
        };

        /// Left-justified: the 12 bits sit in 15:4, so an arithmetic shift recovers them
        /// with their sign.
        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            for(std::size_t i = 0; i < 4; ++i) {
                sample.code[i] = static_cast<std::int16_t>(data.s16be(2 * i) >> 4);
            }
            return sample;
        }
    };

    using Reads = List<Sweep>;
};

}   // namespace Kvasir::I2C::Chips
