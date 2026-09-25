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

namespace Ltr507Detail {
    /// ALS_CONTR gain (4:3): the four dynamic ranges.
    enum class AlsGain : std::uint8_t { range1 = 0, range2 = 1, range3 = 2, range4 = 3 };

    /// ALS_MEAS_RATE repeat rate (2:0).
    enum class AlsRate : std::uint8_t { ms100 = 0, ms200 = 1, ms500 = 2, ms1000 = 3, ms2000 = 4 };

    inline constexpr std::array<std::chrono::milliseconds, 5> AlsRates{
      std::chrono::milliseconds{100},
      std::chrono::milliseconds{200},
      std::chrono::milliseconds{500},
      std::chrono::milliseconds{1000},
      std::chrono::milliseconds{2000}};

    /// Millilux per count at 16-bit resolution for the four ranges (datasheet 6.1, ALS_CONTR
    /// gain): 1, 0.5, 0.01 and 0.005 lux/count, full scale 64k, 32k, 640 and 320 lux.
    inline constexpr std::array<std::uint16_t, 4> MilliLuxPerCount{1000, 500, 10, 5};
}   // namespace Ltr507Detail

/// Lite-On LTR-507ALS ambient light and proximity sensor with a built-in emitter. Address by
/// the SEL pin (5.2): GND 0x3A, VDD 0x3B, floating 0x23.
/// Its registers start at 0x80, not at zero: ALS_CONTR 0x80 (ALS mode bit 1, SW reset bit 2,
/// gain 4:3), PS_CONTR 0x81 (PS mode bit 1, gain 3:2), PS_LED 0x82, PS_N_PULSES 0x83, PS_MEAS_RATE 0x84,
/// ALS_MEAS_RATE 0x85, PART_ID 0x86 (0x91), MANUFAC_ID 0x87 (0x05), ALS_DATA 0x88/0x89 (16 bit
/// little endian), ALS_PS_STATUS 0x8A (bit 2 new ALS data, bit 0 new PS data), PS_DATA
/// 0x8B/0x8C (11 bit, and 0x8C bit 4 set when the PS reading overflowed),
/// the wider per-channel ALS results at 0x8D..0x92, and the interrupt and threshold registers
/// from 0x98 (datasheet section 6).
///
/// The status bit says whether the ambient reading is fresh; a run that finds it clear has
/// nothing new to report, which is what `Outcome::unchanged()` is for -- the previous sample
/// stays.
///
/// The sample carries the raw count and the gain range it was taken at; `lux()` scales it by
/// Ltr507Detail::MilliLuxPerCount, the datasheet's per-range figure at the 16-bit resolution
/// the bring-up sets. The gain is the Initial of the AlsControl write
/// group and can be changed at run time.
///
/// The ALS repeats at `Rate` (100 ms by default) and the poll period is 10 ms longer, so a poll
/// does not land on a result it has already read. PS measures every 100 ms too: with a
/// shorter PS rate (70 ms) the ALS result only updates every second PS period, about 140 ms,
/// regardless of the poll period.
template<Ltr507Detail::AlsGain AlsGain   = Ltr507Detail::AlsGain::range1,
         unsigned              LedPulses = 8,   ///< PS_N_PULSES
         Ltr507Detail::AlsRate Rate      = Ltr507Detail::AlsRate::ms100>
struct Ltr507 {
    static constexpr std::string_view Name = "LTR-507ALS";
    /// Lite-On LTR-507ALS. LTR507.md:430..431, PART_ID 0x86 = 0x91 (part number in 7:4) and
    /// MANUFAC_ID 0x87 = 0x05.
    static constexpr std::array Identity{
      RegisterCheck{        "part-id", 0x86, 1, true, 0xF0, 0x90},
      RegisterCheck{"manufacturer-id", 0x87, 1, true, 0xFF, 0x05},
    };
    static constexpr Address7                Address = 0x3A;
    static constexpr std::array<Address7, 3> Addresses{0x3A, 0x3B, 0x23};
    static constexpr std::size_t             RegisterBytes = 1;

    static_assert(LedPulses >= 1 && LedPulses <= 255,
                  "PS_N_PULSES is 1..255");

    static constexpr std::uint8_t              GainCode = static_cast<std::uint8_t>(AlsGain);
    static constexpr std::chrono::milliseconds MeasurementPeriod
      = Ltr507Detail::AlsRates[static_cast<std::size_t>(Rate)];

    /// ALS_CONTR: gain in 4:3, active mode in bit 1 -- the same bit as PS_CONTR's. Bit 0 is
    /// reserved; setting it instead of bit 1 leaves the ALS in standby, so ALS_PS_STATUS
    /// never reports new ambient data.
    static constexpr std::uint8_t AlsContr = static_cast<std::uint8_t>((GainCode << 3) | 0x02U);
    /// PS_CONTR: active mode in bit 1, and the PS gain field 3:2, which "must write as 11"
    /// (6.2; the datasheet's own enable sequence writes 0x0E).
    static constexpr std::uint8_t PsContr = 0x0E;

    static constexpr auto StartupDelay = std::chrono::milliseconds{100};

    /// PS_LED 0x82: pulse frequency 7:5, duty cycle 4:3, peak current 2:0. The duty field
    /// must always read 01 (50 %); 60 kHz and 100 mA, the highest current the part has.
    static constexpr std::uint8_t PsLed = 0x6C;
    /// ALS_MEAS_RATE 0x85: resolution and integration 7:5, repeat rate 2:0. 16 bit and 75 ms,
    /// repeated at `Rate` -- a fresh result for every poll. The reset value 0x82 is the same
    /// resolution at a 500 ms rate, too slow for a 100 ms poll.
    static constexpr std::uint8_t AlsMeasRate
      = static_cast<std::uint8_t>(0x80U | static_cast<unsigned>(Rate));

    static constexpr std::array Init{
      Step::write({.reg = 0x80, .payload = {AlsContr}}),
      Step::write({.reg = 0x81, .payload = {PsContr}}),
      Step::write({.reg = 0x82, .payload = {PsLed}}),
      Step::write({.reg = 0x83, .payload = {static_cast<std::uint8_t>(LedPulses)}}),
      Step::write(
        {.reg     = 0x84,
         .payload = {0x03}}),   // PS measurement rate: 100 ms, the ALS's (0x00 would stop the ALS)
      Step::write({.reg     = 0x85,
                   .payload = {AlsMeasRate},
                   .delay   = MeasurementPeriod}),   // then one repeat's worth of wait
    };

    struct State {
        std::uint8_t partId{};
        std::uint8_t manufacturerId{};
        std::uint8_t gain{GainCode};   ///< ALS_CONTR's gain field now
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.partId         = static_cast<std::uint8_t>(ids[0]);
        state.manufacturerId = static_cast<std::uint8_t>(ids[1]);
        state.gain           = GainCode;
    }

    struct Light {
        static constexpr auto Period = MeasurementPeriod + std::chrono::milliseconds{10};

        static constexpr std::array Steps{
          Step::read({.reg = 0x8A, .count = 1, .offset = 0}),   // ALS_PS_STATUS
          Step::read({.reg = 0x88, .count = 2, .offset = 1}),   // ALS_DATA
          Step::read({.reg = 0x8B, .count = 2, .offset = 3}),   // PS_DATA
        };

        struct Sample {
            std::uint16_t light{};               ///< raw ALS count at the gain below
            std::uint16_t proximity{};           ///< raw 11-bit proximity count
            bool          proximityOverflow{};   ///< PS_DATA_1 bit 4 (6.11): the count overflowed
            std::uint8_t  status{};
            std::uint8_t  gain{};   ///< ALS_CONTR gain field the count was taken at, 0..3

            /// Which of the four gain ranges the light count was taken at.
            [[nodiscard]] constexpr std::uint8_t gainRange() const {
                return static_cast<std::uint8_t>((gain & 0x03U) + 1U);
            }

            /// The datasheet's lux per count for the range, at 16-bit resolution.
            [[nodiscard]] constexpr MilliLux lux() const {
                return Units::milliLux(static_cast<std::uint32_t>(light)
                                       * Ltr507Detail::MilliLuxPerCount[gain & 0x03U]);
            }

            /// PS_DATA_1's Valid bit (6.11): clear is a valid reading, set an overflow.
            [[nodiscard]] constexpr bool proximityValid() const { return !proximityOverflow; }

            /// ALS_PS_STATUS bit 0 (6.10): the PS reading had not been read before this frame.
            [[nodiscard]] constexpr bool proximityNew() const { return (status & 0x01U) != 0; }
        };

        /// ALS_PS_STATUS bit 2 clear is "old data (data has been read)" (6.10): no conversion has
        /// finished since the last readout. Nothing is wrong with the part or the frame, so it is
        /// `unchanged`, not a rejection -- a read that lands just before a conversion ends does
        /// that now and then (i2c_testing hardware test, 2026-09-18: one in a minute).
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            auto const status = data.u8(0);
            if((status & 0x04U) == 0) { return Outcome<Sample>::unchanged(); }
            Sample sample{};
            sample.status            = status;
            sample.light             = data.le16(1);
            sample.proximity         = static_cast<std::uint16_t>(data.le16(3) & 0x07FFU);
            sample.proximityOverflow = (data.u8(4) & 0x10U) != 0;
            sample.gain              = state.gain;
            return Outcome<Sample>::ok(sample);
        }
    };

    /// ALS_CONTR, so the gain can be changed at run time; a Sample decoded after the write
    /// carries the new range.
    struct AlsControl {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = AlsContr;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x80, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gain = static_cast<std::uint8_t>((value >> 3U) & 0x03U);
        }
    };

    using Reads  = List<Light>;
    using Writes = List<AlsControl>;
};

}   // namespace Kvasir::I2C::Chips
