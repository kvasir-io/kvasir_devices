#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Ad7291Detail {
    /// Command D7 TSENSE: whether the part converts its temperature sensor.
    enum class Tsense : std::uint8_t { off, on };
}   // namespace Ad7291Detail

/// Analog Devices AD7291 8-channel 12-bit ADC with a die temperature sensor. One-byte
/// pointer: command register 0x00 (two bytes, write only), voltage result 0x01, TSENSE
/// 0x02.
///
/// The part is run in command mode, not autocycle. The voltage result register holds "the
/// most recent conversion result", so in autocycle it is whatever channel happened to
/// finish last; in command mode the sequence is driven by the reads themselves -- one
/// channel per word, restarting at the lowest enabled channel after every STOP -- so one
/// burst of eight words yields all eight channels in a known order. Each word carries its
/// channel in D15..D12, which is what lets a desynchronised burst read as "no reading"
/// rather than as a neighbouring channel's voltage.
///
/// The command register cannot be read back, so it is rewritten every five seconds by a
/// write group with a Period. Command low byte: D7 TSENSE, D5 noise-delayed bit trial
/// (the datasheet recommends it unconditionally), D4 EXT_REF = 0 (internal 2.5 V),
/// D0 autocycle = 0.
///
/// The address is strapped by the three-state pins AS1 and AS0 (H, NC, L each; Table 31), so
/// it is a template parameter with no default: `Ad7291<0x20>`. The nine strappings give
/// 0x20, 0x22, 0x23, 0x28, 0x2A, 0x2B, 0x2C, 0x2E and 0x2F; any other address does not compile.
template<Address7             Addr,
         std::uint8_t         ChannelMask = 0xFF,
         Ad7291Detail::Tsense TempSense   = Ad7291Detail::Tsense::on>
struct Ad7291 {
    static constexpr std::string_view Name          = "AD7291";
    static constexpr Address7         Address       = Addr;
    static constexpr std::size_t      RegisterBytes = 1;

    /// Table 31, AS1/AS0: HH, HNC, HL, NCH, NCNC, NCL, LH, LNC, LL.
    static constexpr std::array<Address7, 9>
      Addresses{0x20, 0x22, 0x23, 0x28, 0x2A, 0x2B, 0x2C, 0x2E, 0x2F};

    static_assert(std::ranges::find(Addresses,
                                    Addr)
                    != Addresses.end(),
                  "an AD7291 answers only at the nine AS1/AS0 strappings of Table 31");

    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

    static constexpr std::size_t Channels = 8;

    /// 1 LSB is VREF/4096 with the internal 2.5 V reference.
    static constexpr MicroVolt Vref = Units::microVolt(2'500'000);

    /// The command register wants the channel mask MIRRORED: D15 enables channel 0 and D8
    /// enables channel 7 (Rev. C Table 12), the opposite of the natural bit-N-is-channel-N
    /// order. The two agree only for symmetric masks such as the default 0xFF.
    [[nodiscard]] static constexpr std::uint8_t mirrored(std::uint8_t mask) {
        std::uint8_t out = 0;
        for(std::uint8_t i = 0; i < Channels; ++i) {
            if((mask & static_cast<std::uint8_t>(1U << i)) != 0) {
                out = static_cast<std::uint8_t>(out | (1U << (Channels - 1U - i)));
            }
        }
        return out;
    }

    /// Write-only, so there is nothing to read back and nothing to verify. Rewritten every
    /// five seconds instead, which is what Period on a write group does.
    struct Command {
        using Value                         = std::uint16_t;
        static constexpr std::size_t Bytes  = 2;
        static constexpr auto        Period = std::chrono::seconds{5};

        static constexpr Value Initial = static_cast<std::uint16_t>(
          (static_cast<unsigned>(mirrored(ChannelMask)) << 8)
          | (TempSense == Ad7291Detail::Tsense::on ? 0x80U : 0x00U) | 0x20U);

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            putBe16(buffer, 0, value);
            return Step::writeBuffer({.reg = 0x00, .offset = 0, .count = 2});
        }
    };

    struct Voltages {
        static constexpr auto       Period = std::chrono::milliseconds{20};
        static constexpr std::array Steps{
          Step::read({.reg = 0x01, .count = Channels * 2, .offset = 0})};

        struct Sample {
            /// Per channel: the 12-bit code, and whether this burst carried it.
            std::array<std::uint16_t, Channels> code{};
            std::uint8_t                        seen{};   ///< bit N: channel N was in the burst

            [[nodiscard]] constexpr bool valid(std::size_t channel) const {
                return (seen & (1U << channel)) != 0;
            }

            [[nodiscard]] constexpr MicroVolt voltage(std::size_t channel) const {
                return Units::microVolt(static_cast<std::int32_t>(
                  static_cast<std::int64_t>(code[channel]) * Units::value(Vref) / 4096));
            }
        };

        /// Matched on the channel-address bits rather than on position, so a burst that
        /// slipped reports nothing for a channel instead of its neighbour's voltage.
        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            for(std::size_t i = 0; i < Channels; ++i) {
                auto const w       = data.be16(2 * i);
                auto const channel = static_cast<std::size_t>((w >> 12) & 0x0FU);
                if(channel < Channels) {
                    sample.code[channel] = static_cast<std::uint16_t>(w & 0x0FFFU);
                    sample.seen          = static_cast<std::uint8_t>(sample.seen | (1U << channel));
                }
            }
            return sample;
        }
    };

    /// Hardware-averaged and slow moving, so it is not worth the bus time the voltages get.
    /// Only meaningful with `TempSense` on; without it the part never converts the channel.
    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{500};
        static constexpr std::array Steps{Step::read({.reg = 0x02, .count = 2, .offset = 0})};

        struct Sample {
            CentiDegC temperature{};   ///< 0.01 degC
        };

        /// D15..D12 are the channel address (1000 for TSENSE), D11..D0 the reading, two's
        /// complement with bit 11 as the sign, so the 12-bit field is sign-extended from bit
        /// 11. 1 LSB is 0.25 degC. A word without the TSENSE address is not a temperature.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            auto const w = data.be16(0);
            if((w >> 12) != 0x8U) { return Outcome<Sample>::reject(); }
            auto const value = Bytes::signExtend(w & 0x0FFFU, 12);
            return Outcome<Sample>::ok({Units::centiDegC(value * 25)});
        }
    };

    using Reads  = List<Voltages, Temperature>;
    using Writes = List<Command>;
};

}   // namespace Kvasir::I2C::Chips
