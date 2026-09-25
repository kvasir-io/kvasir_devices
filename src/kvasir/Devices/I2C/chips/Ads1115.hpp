#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Shared by the ADS1115 and the ADS1015: the same Config register layout.
namespace Ads1x15Detail {
    /// The PGA field (Config bits 11:9) as the full-scale range it selects. The input may
    /// never exceed VDD + 0.3 V whatever the range says.
    enum class Gain : std::uint8_t {
        fsr6V144 = 0,   ///< +-6.144 V
        fsr4V096 = 1,   ///< +-4.096 V
        fsr2V048 = 2,   ///< +-2.048 V, the reset default
        fsr1V024 = 3,   ///< +-1.024 V
        fsr0V512 = 4,   ///< +-0.512 V
        fsr0V256 = 5,   ///< +-0.256 V
    };

    /// The positive full scale of each Gain, in microvolts.
    [[nodiscard]] constexpr std::int32_t fullScaleMicroVolt(Gain gain) {
        constexpr std::array<std::int32_t, 6> Fsr{6'144'000,
                                                  4'096'000,
                                                  2'048'000,
                                                  1'024'000,
                                                  512'000,
                                                  256'000};
        return Fsr[static_cast<std::size_t>(gain)];
    }

    /// Config for one single-shot conversion of AINn against GND: OS | MUX 100b + n | the
    /// PGA field | MODE single | DR 100b | COMP_QUE 11 = C183h + n << 12 + gain << 9.
    [[nodiscard]] constexpr std::uint16_t config(unsigned channel,
                                                 Gain     gain) {
        return static_cast<std::uint16_t>(0xC183U | (channel << 12)
                                          | (static_cast<unsigned>(gain) << 9));
    }
}   // namespace Ads1x15Detail

namespace Ads1115Detail {
    /// One single-shot conversion of AINn: the config write, then the 7.8 ms conversion at
    /// DR 100b (128 SPS).
    [[nodiscard]] constexpr Step start(unsigned            channel,
                                       Ads1x15Detail::Gain gain) {
        auto const cfg = Ads1x15Detail::config(channel, gain);
        return Step::write({
          .reg     = 0x01,
          .payload = {static_cast<std::uint8_t>(cfg >> 8), static_cast<std::uint8_t>(cfg & 0xFF)},
          // 128 SPS is 7.8 ms, and the data rate may be 10 % slow (Electrical Characteristics),
          // plus the 25 us wake-up: 8.7 ms at worst, with room for a coarse clock
          .delay = std::chrono::milliseconds{12}
        });
    }
}   // namespace Ads1115Detail

/// Texas Instruments ADS1115 (SBAS444E). Four 16-bit big-endian registers behind a
/// one-byte pointer: 00b Conversion, 01b Config. Each channel single-shot: Config = OS |
/// MUX 100b + n (AINn to GND) | PGA (the `Gain` parameter; 010b, 2.048 V, by default) |
/// MODE single | DR 100b (128 SPS, 7.8 ms) | COMP_QUE 11 -- C583h for AIN0 at the default
/// gain, +1000h per channel -- wait 10 ms, read the Conversion register (two's complement,
/// 7FFFh = +FSR). All four in one sweep every 100 ms. The gain is a template parameter
/// because it is part of every conversion's config word in the Steps script. 0x48..0x4B
/// by the ADDR pin.
template<Ads1x15Detail::Gain Gain = Ads1x15Detail::Gain::fsr2V048>
struct Ads1115 {
    static constexpr std::string_view Name = "ADS1115";
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

    /// +FSR, what a code of 0x7FFF + 1 would be.
    static constexpr MicroVolt FullScale
      = Units::microVolt(Ads1x15Detail::fullScaleMicroVolt(Gain));

    struct Sweep {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{
          Ads1115Detail::start(0, Gain),
          Step::read({.reg = 0x00, .count = 2, .offset = 0}),
          Ads1115Detail::start(1, Gain),
          Step::read({.reg = 0x00, .count = 2, .offset = 2}),
          Ads1115Detail::start(2, Gain),
          Step::read({.reg = 0x00, .count = 2, .offset = 4}),
          Ads1115Detail::start(3, Gain),
          Step::read({.reg = 0x00, .count = 2, .offset = 6}),
        };

        struct Sample {
            std::array<std::int16_t, 4> code{};

            /// A code at the configured full-scale range: 32768 counts is +FSR.
            [[nodiscard]] static constexpr MicroVolt toVoltage(std::int16_t c) {
                return Units::microVolt(static_cast<std::int32_t>(
                  static_cast<std::int64_t>(c) * Units::value(FullScale) / 32768));
            }

            /// Channel `ch`.
            [[nodiscard]] constexpr MicroVolt voltage(std::size_t ch) const {
                return toVoltage(code[ch]);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            for(std::size_t i = 0; i < 4; ++i) { sample.code[i] = data.s16be(2 * i); }
            return sample;
        }
    };

    using Reads = List<Sweep>;
};

}   // namespace Kvasir::I2C::Chips
