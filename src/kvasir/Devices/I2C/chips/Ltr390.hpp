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

namespace Ltr390Detail {
    /// GAIN (2:0): 1x, 3x, 6x, 9x, 18x. 5..7 are reserved.
    enum class Gain : std::uint8_t { x1 = 0, x3 = 1, x6 = 2, x9 = 3, x18 = 4 };

    /// MEAS_RATE resolution (6:4): 20 down to 13 bits; the conversion takes 400 ms down to
    /// 12.5 ms.
    enum class Resolution : std::uint8_t {
        bit20 = 0,
        bit19 = 1,
        bit18 = 2,
        bit17 = 3,
        bit16 = 4,
        bit13 = 5
    };

    inline constexpr std::array<std::uint16_t, 5> GainValue{1, 3, 6, 9, 18};
    inline constexpr std::array<std::uint8_t, 6>  ResolutionBits{20, 19, 18, 17, 16, 13};
    /// The conversion takes about this long at each resolution, rounded up.
    inline constexpr std::array<std::chrono::milliseconds, 6> ConversionTimes{
      std::chrono::milliseconds{400},
      std::chrono::milliseconds{200},
      std::chrono::milliseconds{100},
      std::chrono::milliseconds{50},
      std::chrono::milliseconds{25},
      std::chrono::milliseconds{13}};

    [[nodiscard]] constexpr std::uint16_t gainValue(std::uint8_t gain) {
        return gain < GainValue.size() ? GainValue[gain] : GainValue[0];
    }

    [[nodiscard]] constexpr std::uint8_t resolutionBits(std::uint8_t resolution) {
        return resolution < ResolutionBits.size() ? ResolutionBits[resolution] : ResolutionBits[0];
    }

    [[nodiscard]] constexpr std::chrono::milliseconds conversionTime(std::uint8_t resolution) {
        return resolution < ConversionTimes.size() ? ConversionTimes[resolution]
                                                   : ConversionTimes[0];
    }
}   // namespace Ltr390Detail

/// Lite-On LTR390 UV and ambient light sensor, address 0x53. One-byte pointer: MAIN_CTRL
/// 0x00 (enable bit 1, ALS/UVS mode bit 3, software reset bit 4), MEAS_RATE 0x04
/// (resolution 6:4, measurement rate 2:0), GAIN 0x05, PART_ID 0x06 (the upper nibble reads
/// 0xB), MAIN_STATUS 0x07 (data ready bit 3), ALSDATA 0x0D and UVSDATA 0x10 -- each a
/// 20-bit value over three little-endian bytes.
///
/// The part measures ambient light *or* UV, whichever the mode bit selects; there is only
/// one ADC. A full reading is therefore two passes with a mode switch between them, and one
/// read group runs both so a Sample carries a matched pair -- the same reason the AS7341
/// runs both of its SMUX passes in one group. MAIN_STATUS is read with each result, and a
/// run in which either pass had no new data is Outcome::unchanged().
///
/// The conversions, with the gain and resolution the frame was taken at (each Sample carries
/// them, so a run-time change through the write groups is followed):
///
///     lux = ALS * 0.6 / (gain * integrationFactor)
///     UVI = UVS / ((gain / 18) * (2^resolutionBits / 2^20) * 2300)
///
/// where the integration factor is 4, 2, 1, 0.5, 0.25 and 0.03125 for 20 down to 13 bits:
/// exactly 2^(bits - 18), which is how it is computed. 2300 counts per UV index at 18x gain
/// and 20-bit resolution is the part's rated sensitivity. `Window` is the transmission of any
/// glass in front of the sensor, 100 for none; both readings divide by it.
///
/// Both constants come from the datasheet: 0.6 is the lux formula's (7.1, with the window
/// factor WFAC), and 2300 counts per UVI is the UV sensitivity at 18x gain and 20-bit
/// resolution (4.5). Scaling 2300 to other gains and resolutions is an extrapolation.
///
/// The step delays are the template resolution's conversion time. A run-time write of a
/// slower resolution is not waited for -- the status bit then says there is nothing new and
/// the run is unchanged -- so change the resolution downwards only, or rebuild.
template<Ltr390Detail::Gain       Gain       = Ltr390Detail::Gain::x3,
         Ltr390Detail::Resolution Resolution = Ltr390Detail::Resolution::bit18,
         Percent                  Window     = Units::percent(100)>
struct Ltr390 {
    static constexpr std::string_view Name = "LTR390";
    /// Lite-On LTR-390UV. LTR390.md:448..461, PART_ID 0x06: part number 1011b in 7:4 (default 0xB2).
    /// Configuration: MAIN_CTRL (0x00) ALS_UVS_EN, bit 1, set and the software reset, bit 4, over
    /// (LTR390.md:359..373).
    static constexpr std::array Identity{
      RegisterCheck{"part-id", 0x06, 1, true, 0xF0, 0xB0},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"enabled", 0x00, 1, true, 0x12, 0x02},
    };
    static constexpr Address7                Address = 0x53;
    static constexpr std::array<Address7, 1> Addresses{0x53};
    static constexpr std::size_t             RegisterBytes = 1;

    static_assert(Window >= Units::percent(1),
                  "a window transmission of zero would divide by zero");

    static constexpr std::uint8_t  GainCode       = static_cast<std::uint8_t>(Gain);
    static constexpr std::uint8_t  ResolutionCode = static_cast<std::uint8_t>(Resolution);
    static constexpr std::uint64_t WindowValue    = Units::value(Window);
    static constexpr std::chrono::milliseconds ConversionTime
      = Ltr390Detail::conversionTime(ResolutionCode);

    /// Resolution, and the measurement rate at 100 ms (2).
    static constexpr std::uint8_t MeasRate
      = static_cast<std::uint8_t>((ResolutionCode << 4) | 0x02U);
    static constexpr std::uint8_t Enabled = 0x02;   // bit 1
    static constexpr std::uint8_t AlsMode = 0x02;   // enable, mode 0
    static constexpr std::uint8_t UvsMode = 0x0A;   // enable, mode 1

    static constexpr auto StartupDelay = std::chrono::milliseconds{20};

    static constexpr std::array Init{
      // Software reset: the part resets before it acknowledges the byte, so the write NAKs
      // (Linux ltr390.c: "chip fails to respond to this, so ignore any errors"; Adafruit's
      // reset() says the same).
      Step::write(
        {.reg = 0x00, .payload = {0x10}, .delay = std::chrono::milliseconds{20}, .mayNak = true}),
      Step::write({.reg = 0x04, .payload = {MeasRate}}),
      Step::write({.reg = 0x05, .payload = {GainCode}}),
      Step::write({.reg     = 0x00,
                   .payload = {Enabled},
                   .delay   = ConversionTime + std::chrono::milliseconds{10}}),
    };

    struct State {
        std::uint8_t partId{};
        std::uint8_t gain{GainCode};
        std::uint8_t resolution{ResolutionCode};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.partId     = static_cast<std::uint8_t>(ids[0]);
        state.gain       = GainCode;
        state.resolution = ResolutionCode;
    }

    struct Light {
        static constexpr auto Period = 2 * ConversionTime + std::chrono::milliseconds{100};
        /// So the runs that found nothing new are counted (unchanged<Light>()).
        static constexpr bool Timestamped = true;

        /// Ambient light, then the same ADC switched to the UV photodiode; MAIN_STATUS before
        /// each result.
        static constexpr std::array Steps{
          Step::write({.reg     = 0x00,
                       .payload = {AlsMode},
                       .delay   = ConversionTime + std::chrono::milliseconds{10}}),
          Step::read({.reg = 0x07, .count = 1, .offset = 0}),   // MAIN_STATUS
          Step::read({.reg = 0x0D, .count = 3, .offset = 1}),   // ALSDATA
          Step::write({.reg     = 0x00,
                       .payload = {UvsMode},
                       .delay   = ConversionTime + std::chrono::milliseconds{10}}),
          Step::read({.reg = 0x07, .count = 1, .offset = 4}),   // MAIN_STATUS
          Step::read({.reg = 0x10, .count = 3, .offset = 5}),   // UVSDATA
        };

        struct Sample {
            std::uint32_t als{};          ///< raw 20-bit ambient light count
            std::uint32_t uvs{};          ///< raw 20-bit UV count
            std::uint8_t  gain{};         ///< GAIN the frame was taken at
            std::uint8_t  resolution{};   ///< MEAS_RATE resolution

            /// lux = ALS x 0.6 / (gain x 2^(bits - 18)), scaled by the window transmission.
            [[nodiscard]] constexpr MilliLux lux() const {
                auto const bits = Ltr390Detail::resolutionBits(resolution);
                // 0.6 / 2^(bits - 18) = 600 x 2^(20 - bits) / (4 x 1000)
                auto const num
                  = static_cast<std::uint64_t>(als) * 600ULL * (1ULL << (20U - bits)) * 100ULL;
                auto const den
                  = static_cast<std::uint64_t>(Ltr390Detail::gainValue(gain)) * 4ULL * WindowValue;
                return Units::milliLux(num / den);
            }

            /// The rated sensitivity is 2300 counts per UVI at 18x gain and 20-bit resolution;
            /// both scale the divisor from there, and the window transmission too.
            [[nodiscard]] constexpr MilliUvi uvi() const {
                // counts per UVI = 2300 * (gain/18) * 2^bits / 2^20 * window
                auto const shift = 20U - Ltr390Detail::resolutionBits(resolution);
                auto const den   = 2300ULL * Ltr390Detail::gainValue(gain) * WindowValue;
                auto const num
                  = static_cast<std::uint64_t>(uvs) * 1000ULL * 18ULL * (1ULL << shift) * 100ULL;
                return Units::milliUvi(num / den);
            }
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if((data.u8(0) & 0x08U) == 0 || (data.u8(4) & 0x08U) == 0) {
                return Outcome<Sample>::unchanged();
            }
            return Outcome<Sample>::ok(
              {data.le24(1) & 0x0FFFFFU, data.le24(5) & 0x0FFFFFU, state.gain, state.resolution});
        }
    };

    /// GAIN, so the range can be changed at run time; a Sample decoded after the write
    /// carries the new gain.
    struct GainSetting {
        using Value                          = Ltr390Detail::Gain;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Gain;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x05, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gain = static_cast<std::uint8_t>(value);
        }
    };

    /// MEAS_RATE's resolution (6:4); the measurement rate stays at 100 ms.
    struct ResolutionSetting {
        using Value                          = Ltr390Detail::Resolution;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Resolution;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>((static_cast<unsigned>(value) << 4U) | 0x02U);
            return Step::writeBuffer({.reg = 0x04, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.resolution = static_cast<std::uint8_t>(value);
        }
    };

    using Reads  = List<Light>;
    using Writes = List<GainSetting, ResolutionSetting>;
};

}   // namespace Kvasir::I2C::Chips
