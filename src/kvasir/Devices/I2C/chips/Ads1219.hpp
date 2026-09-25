#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Ads1219Detail {
    /// The GAIN bit of the configuration register.
    enum class Gain : std::uint8_t { x1 = 0, x4 = 1 };

    /// DR, the data rate field: conversions per second.
    enum class DataRate : std::uint8_t { sps20 = 0, sps90 = 1, sps330 = 2, sps1000 = 3 };

    /// Config for one single-shot conversion of AINn against AGND: MUX 011b + n, the gain
    /// bit, the data rate in 3:2, single-shot, internal 2.048 V reference.
    [[nodiscard]] constexpr std::uint8_t config(unsigned channel,
                                                Gain     gain,
                                                DataRate dataRate) {
        return static_cast<std::uint8_t>(((3U + channel) << 5) | (static_cast<unsigned>(gain) << 4)
                                         | (static_cast<unsigned>(dataRate) << 2));
    }
}   // namespace Ads1219Detail

/// Texas Instruments ADS1219 24-bit, four-channel delta-sigma ADC. There is no register
/// pointer: the interface is a command byte, optionally followed by data. RESET 0x06,
/// START/SYNC 0x08, POWERDOWN 0x02, RDATA 0x10, RREG 0x20 | r << 2, WREG 0x40 followed by
/// the configuration byte. The configuration is MUX in 7:5, GAIN in 4 (1 or 4), DR in 3:2
/// (20, 90, 330, 1000 SPS), CM in 1 (single-shot or continuous) and VREF in 0 (internal
/// 2.048 V or external).
///
/// Each channel is converted on its own: WREG selects it, START/SYNC begins the conversion, and
/// after the conversion time RDATA plus a three-byte read collects the result. RDATA is its own
/// write transaction, so there is a STOP between the command and the read rather than the
/// datasheet's repeated START; the part keeps the result until the next conversion, so a plain read
/// after the STOP returns it (the two-frame read of 8.5.3.5, Figure 36). All four single-ended
/// channels make one sweep. The result is 24-bit two's complement, full scale +-VREF/gain, so one
/// count is 244.14 nV at gain 1 against the internal reference.
///
/// The gain and rate are template parameters because they are part of every conversion's
/// WREG byte in the Steps script. A0 and A1 each tie to DGND, DVDD, SDA or SCL, giving
/// sixteen addresses from 0x40.
template<Ads1219Detail::Gain     Gain     = Ads1219Detail::Gain::x1,
         Ads1219Detail::DataRate DataRate = Ads1219Detail::DataRate::sps90>
struct Ads1219 {
    static constexpr std::string_view Name    = "ADS1219";
    static constexpr Address7         Address = 0x40;
    /// A0 and A1 each tie to DGND, DVDD, SDA or SCL: sixteen addresses from 0x40.
    static constexpr std::array<Address7, 16> Addresses{0x40,
                                                        0x41,
                                                        0x42,
                                                        0x43,
                                                        0x44,
                                                        0x45,
                                                        0x46,
                                                        0x47,
                                                        0x48,
                                                        0x49,
                                                        0x4A,
                                                        0x4B,
                                                        0x4C,
                                                        0x4D,
                                                        0x4E,
                                                        0x4F};
    static constexpr std::size_t              RegisterBytes = 0;

    static constexpr std::size_t Channels = 4;

    /// One conversion, rounded up generously: 50, 11.1, 3.03 and 1 ms (Table 4's single-shot
    /// times; one Arduino library returns 1 ms for 90 SPS, which is not the table's).
    static constexpr std::chrono::milliseconds Conversion
      = std::array<std::chrono::milliseconds, 4>{
        std::chrono::milliseconds{60},
        std::chrono::milliseconds{15},
        std::chrono::milliseconds{5},
        std::chrono::milliseconds{3}}[static_cast<std::size_t>(DataRate)];

    /// Full scale is VREF / gain over 2^23 counts; the internal reference is 2.048 V.
    static constexpr MicroVolt FullScale
      = Units::microVolt(Gain == Ads1219Detail::Gain::x1 ? 2'048'000 : 512'000);

    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    /// RESET, then the configuration register read back with RREG: the reset leaves it 00h
    /// (Table 9), which is the check that the part is an ADS1219 (SparkFun's begin() makes the
    /// same one) before a sweep writes to it.
    static constexpr std::array Init{
      Step::command({.payload = {0x06}, .delay = std::chrono::milliseconds{1}}),   // RESET
      Step::command({.payload = {0x20}}),                                          // RREG config
      Step::receive({.count = 1, .offset = 0}),
    };

    struct State {
        std::uint8_t config{};   ///< the configuration register after the reset
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.config = data.u8(0);
        return state.config == 0x00;
    }

    struct Sweep {
        /// A sweep is four conversions: every 200 ms, or as fast as four of them allow.
        static constexpr auto Period
          = 4 * Conversion + std::chrono::milliseconds{40} > std::chrono::milliseconds{200}
            ? 4 * Conversion + std::chrono::milliseconds{40}
            : std::chrono::milliseconds{200};

        /// Per channel: select it, start it, wait it out, then ask for the result.
        static constexpr auto Steps = [] {
            std::array<Step, 4 * Channels> s{};
            std::size_t                    i = 0;
            for(unsigned c = 0; c < Channels; ++c) {
                s[i++] = Step::command({
                  .payload = {0x40, Ads1219Detail::config(c, Gain, DataRate)}
                });
                s[i++] = Step::command({.payload = {0x08}, .delay = Conversion});   // START/SYNC
                s[i++] = Step::command({.payload = {0x10}});   // RDATA, then a STOP
                s[i++] = Step::receive({.count = 3, .offset = static_cast<std::uint8_t>(3 * c)});
            }
            return s;
        }();

        struct Sample {
            std::array<std::int32_t, Channels> code{};   ///< 24-bit two's complement

            /// A raw code; 2^23 counts span the full-scale range.
            [[nodiscard]] static constexpr MicroVolt toVoltage(std::int32_t c) {
                return Units::microVolt(static_cast<std::int64_t>(c) * Units::value(FullScale)
                                        / 8388608LL);
            }

            /// Channel `ch`.
            [[nodiscard]] constexpr MicroVolt voltage(std::size_t ch) const {
                return toVoltage(code[ch]);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            for(std::size_t c = 0; c < Channels; ++c) {
                sample.code[c] = Bytes::signExtend(data.be24(3 * c), 24);
            }
            return sample;
        }
    };

    using Reads = List<Sweep>;
};

}   // namespace Kvasir::I2C::Chips
