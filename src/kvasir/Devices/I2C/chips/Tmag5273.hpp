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

namespace Tmag5273Detail {
    /// CONV_AVG, DEVICE_CONFIG_1 bits 4:2 (Table 8-3): how many conversions of each channel one
    /// result averages.
    enum class Averaging : std::uint8_t { x1 = 0, x2 = 1, x4 = 2, x8 = 3, x16 = 4, x32 = 5 };

    /// MAG_CH_EN, SENSOR_CONFIG_1 bits 7:4 (Table 8-5). The last four are the pseudo-simultaneous
    /// sequences of 7.1.3.3 (XYX: X, Y, then X again).
    enum class Channels : std::uint8_t {
        off = 0x0,
        x   = 0x1,
        y   = 0x2,
        xy  = 0x3,
        z   = 0x4,
        zx  = 0x5,
        yz  = 0x6,
        xyz = 0x7,
        xyx = 0x8,
        yxy = 0x9,
        yzy = 0xA,
        xzx = 0xB,
    };

    /// X_Y_RANGE and Z_RANGE, SENSOR_CONFIG_2 bits 1 and 0 (Table 6-3, 8-6): +-40 / +-80 mT on
    /// the x1 versions, +-133 / +-266 mT on the x2 versions.
    enum class Range : std::uint8_t { low = 0, high = 1 };

    /// ANGLE_EN, SENSOR_CONFIG_2 bits 3:2 (Table 8-6): the pair the angle and magnitude are
    /// computed from.
    enum class AnglePair : std::uint8_t { off = 0, xy = 1, yz = 2, xz = 3 };

    /// OPERATING_MODE, DEVICE_CONFIG_2 bits 1:0 (Table 8-4, 6.4). Only continuous delivers
    /// samples to the Field group: standby converts only on a trigger, and in sleep and
    /// wake-up-and-sleep the part does not acknowledge its address (sleep: 5.11 note 1;
    /// wake-up-and-sleep: 6.4.3, Table 6-5).
    enum class Mode : std::uint8_t {
        standby        = 0,
        sleep          = 1,
        continuous     = 2,
        wakeUpAndSleep = 3,
    };

    /// The Ranges group's value. Out here because a default member initializer may not be
    /// used from inside the class that encloses it.
    struct Ranges {
        Range     xy{Range::low};
        Range     z{Range::low};
        AnglePair angle{AnglePair::xy};

        friend constexpr bool operator==(Ranges const&,
                                         Ranges const&) = default;
    };

    /// SENSOR_CONFIG_2 for `r`: THRX_COUNT, MAG_THR_DIR and MAG_GAIN_CH at 0.
    [[nodiscard]] constexpr std::uint8_t sensorConfig2(Ranges const& r) {
        return static_cast<std::uint8_t>((static_cast<unsigned>(r.angle) << 2U)
                                         | (static_cast<unsigned>(r.xy) << 1U)
                                         | static_cast<unsigned>(r.z));
    }

    /// How many magnetic conversions one set of `c` is (XYX is three).
    [[nodiscard]] constexpr unsigned conversions(Channels c) {
        static constexpr std::array<std::uint8_t, 12> N{0, 1, 1, 2, 1, 2, 2, 3, 3, 3, 3, 3};
        auto const                                    i = static_cast<std::size_t>(c);
        return i < N.size() ? N[i] : 0;
    }

    [[nodiscard]] constexpr bool hasX(Channels c) {
        auto const v = static_cast<unsigned>(c);
        return v == 0x1 || v == 0x3 || v == 0x5 || v == 0x7 || v == 0x8 || v == 0x9 || v == 0xB;
    }

    [[nodiscard]] constexpr bool hasY(Channels c) {
        auto const v = static_cast<unsigned>(c);
        return v == 0x2 || v == 0x3 || v == 0x6 || v == 0x7 || v == 0x8 || v == 0x9 || v == 0xA;
    }

    [[nodiscard]] constexpr bool hasZ(Channels c) {
        auto const v = static_cast<unsigned>(c);
        return v == 0x4 || v == 0x5 || v == 0x6 || v == 0x7 || v == 0xA || v == 0xB;
    }

    /// The time one set of conversions takes, in microseconds: 25 us x (averages x channels + 1).
    /// Over the magnetic channels alone that reproduces every entry of Table 6-4 (1x, one axis:
    /// 50 us = 20 kSPS; 32x, three axes: 2425 us = 0.4 kSPS) and both rows of 5.11 (50 us +
    /// 25 us per further channel at 1x; 825 us + 800 us per further channel at 32x). Here the
    /// temperature channel is counted as one more (1x, one axis: 75 us): 5.11 note 2 says it adds nothing at 1x, and says nothing for the other
    /// averages -- verify; counting it only makes the period longer.
    [[nodiscard]] constexpr std::chrono::microseconds conversionTime(Averaging a,
                                                                     Channels  c) {
        auto const averages = 1U << static_cast<unsigned>(a);
        return std::chrono::microseconds{25U * (averages * (conversions(c) + 1U) + 1U)};
    }

    /// That time rounded up to whole milliseconds, and at least one: the Field group's period.
    [[nodiscard]] constexpr std::chrono::milliseconds readPeriod(Averaging a,
                                                                 Channels  c) {
        auto const p = std::chrono::ceil<std::chrono::milliseconds>(conversionTime(a, c));
        return p == std::chrono::milliseconds::zero() ? std::chrono::milliseconds{1} : p;
    }

    /// The range BR in nanotesla: VER 1 is the +-40 / +-80 mT part, VER 2 the +-133 / +-266 mT
    /// one (Table 8-16, 5.7, 5.8).
    [[nodiscard]] constexpr std::int64_t rangeNt(std::uint8_t version,
                                                 Range        r) {
        auto const base = version == 2 ? 133'000'000LL : 40'000'000LL;
        return r == Range::high ? 2 * base : base;
    }

    /// A 16-bit two's complement result at range BR: B = code x BR / 2^15, the result spanning
    /// -BR .. +BR (6.5.2.1, Equation 10). The converted data sheet lost the equation's image,
    /// but the sensitivity tables of 5.7 and 5.8 confirm the scale (820 LSB/mT at +-40 mT,
    /// 250 LSB/mT at +-133 mT).
    [[nodiscard]] constexpr std::int32_t fieldNt(std::int16_t code,
                                                 std::int64_t brNt) {
        return static_cast<std::int32_t>(static_cast<std::int64_t>(code) * brNt / 32768);
    }
}   // namespace Tmag5273Detail

/// Texas Instruments TMAG5273 3D linear Hall-effect sensor (SLYS045C, April 2026). One-byte
/// register pointer, auto-incrementing (6.5.1.3); the pointer byte's MSB is the conversion
/// trigger (6.5.1.3.1), which every register used here leaves 0.
///
/// Register map (Table 8-1): DEVICE_CONFIG_1 0x00 (CRC_EN 7, MAG_TEMPCO 6:5, CONV_AVG 4:2,
/// I2C_RD 1:0), DEVICE_CONFIG_2 0x01 (THR_HYST 7:5, LP_LN 4, I2C_GLITCH_FILTER 3, TRIGGER_MODE
/// 2, OPERATING_MODE 1:0), SENSOR_CONFIG_1 0x02 (MAG_CH_EN 7:4, SLEEPTIME 3:0), SENSOR_CONFIG_2
/// 0x03 (THRX_COUNT 6, MAG_THR_DIR 5, MAG_GAIN_CH 4, ANGLE_EN 3:2, X_Y_RANGE 1, Z_RANGE 0),
/// thresholds 0x04..0x06, T_CONFIG 0x07 (T_THR_CONFIG 7:1, T_CH_EN 0), INT_CONFIG_1 0x08, gain
/// and offset correction 0x09..0x0B, I2C_ADDRESS 0x0C, DEVICE_ID 0x0D (VER 1:0),
/// MANUFACTURER_ID 0x0E (LSB, 0x49) and 0x0F (MSB, 0x54) = 0x5449, then the results: T 0x10,
/// X 0x12, Y 0x14, Z 0x16 (16-bit two's complement, MSB first), CONV_STATUS 0x18 (SET_COUNT
/// 7:5, POR 4, DIAG_STATUS 1, RESULT_STATUS 0), ANGLE 0x19..0x1A, MAGNITUDE 0x1B, DEVICE_STATUS
/// 0x1C.
///
/// Addresses (Table 6-2): the version letter is the factory address -- A 0x35, B 0x22, C 0x78,
/// D 0x44 -- for both sensitivities (A1/A2 and so on). The default here is 0x22, the B1 that
/// SparkFun's library addresses; SparkFun's hookup guide says 0x35, so which part a SparkFun
/// board carries is unclear. Select the part's with `At<0x35>` (or the others).
///
/// I2C_ADDRESS (8.1.13) is not used. The engine's address is a compile-time constant that the
/// bus checks for collisions and the scan hints are built from, and the register is volatile:
/// "at each power cycle these bits must be written again to avoid going back to default
/// factory address". A part moved by a write would answer somewhere the Device does not look
/// after the write, and back at the factory address after every brown-out.
///
/// CRC (6.5.1.3.6) is left off (CRC_EN 0): the data sheet warns that "the first CRC can be
/// incorrect after switching communication between devices" on a shared bus, which is every
/// bus this engine runs; I2C_RD stays at the standard 3-byte read.
///
/// Bring-up reads DEVICE_ID and MANUFACTURER_ID; setup() refuses a manufacturer other than 0x5449
/// or a VER that is reserved (0 or 3), and keeps VER, which says which ranges BR the part has. Then
/// it writes the configuration from the template parameters, enables the temperature channel,
/// clears POR (write 1 to clear; the access code also says "requires privileged access", and the
/// data sheet is silent on whether a plain write clears it) and last selects the operating mode, so
/// the part starts converting with the rest already set. MAG_TEMPCO stays 0 (no magnet
/// compensation), LP_LN 0 (low active current), the glitch filter on, interrupts off.
///
/// Every knob is also a write group with applied(), so decode() scales by the range the part
/// holds now and the Field period follows the averaging and the channels (`conversionTime`).
/// None has an Initial: the Init script writes the template's values, and a value the
/// application set is put back after a later bring-up.
///
/// Temperature (5.6, 6.5.2.2): T = 25 degC + (code - 17508) / 58 LSB/degC, with TADC_T0 17508
/// and TADC_RES 58 LSB/degC from revision C's 5.6; the older revisions, and SparkFun's
/// library, use 60.1.
///
/// Angle (6.5.2.3, Table 8-28): 13 bits, 9 integer and 4 fraction bits of a degree; reported
/// as CentiDegree rounded to the nearest, with the 1/16-degree code beside it. Magnitude
/// (Equation 15, Table 8-30): an 8-bit result of the two angle channels, reported here at
/// BR / 128 per count, the range of the pair's first channel (X or Y: X_Y_RANGE). The data
/// sheet is silent on the unit: the equation is an image the conversion lost and the table
/// gives none; 8 bits at the 8-bit result scale (6.5.2.1, Equation 11) is the reading that fits.
template<Tmag5273Detail::Averaging Avg     = Tmag5273Detail::Averaging::x32,
         Tmag5273Detail::Channels  Ch      = Tmag5273Detail::Channels::xyz,
         Tmag5273Detail::Range     XyRange = Tmag5273Detail::Range::low,
         Tmag5273Detail::Range     ZRange  = Tmag5273Detail::Range::low,
         Tmag5273Detail::AnglePair Angle   = Tmag5273Detail::AnglePair::xy,
         Tmag5273Detail::Mode      OpMode  = Tmag5273Detail::Mode::continuous>
struct Tmag5273 {
    static constexpr std::string_view Name = "TMAG5273";
    /// TI TMAG5273. TMAG5273.md:2177, :2203: MANUFACTURER_ID_LSB (Eh) 49h and _MSB (Fh) 54h.
    /// DEVICE_ID (Dh) VER, bits 1:0, is 1h (the 40 / 80 mT part) or 2h (133 / 266 mT); 0h and 3h are
    /// reserved (TMAG5273.md:2162..2173).
    static constexpr std::array Identity{
      RegisterCheck{"device-id", 0x0D, 1, true, 0x03, 0x01, 0x02},
      RegisterCheck{"manufacturer-id-lsb", 0x0E, 1, true, 0xFF, 0x49},
      RegisterCheck{"manufacturer-id-msb", 0x0F, 1, true, 0xFF, 0x54},
    };
    static constexpr Address7                Address = 0x22;
    static constexpr std::array<Address7, 4> Addresses{0x22, 0x35, Address7::reserved(0x78), 0x44};
    static constexpr std::size_t             RegisterBytes = 1;

    /// tstart_power_up, 270 us (5.11), rounded up.
    static constexpr auto StartupDelay = std::chrono::milliseconds{1};

    /// A part left in sleep mode by earlier firmware does not acknowledge the first address
    /// it is woken by (5.11 note 1; Linux tmag5273.c makes a throw-away read and waits before
    /// probing), so a NAK is put back on the wire rather than counted.
    static constexpr std::uint8_t WakeRetries = 2;

    using AveragingCount = Tmag5273Detail::Averaging;
    using ChannelSet     = Tmag5273Detail::Channels;
    using RangeSetting   = Tmag5273Detail::Range;
    using AngleChannels  = Tmag5273Detail::AnglePair;
    using OperatingMode  = Tmag5273Detail::Mode;
    using RangeConfig    = Tmag5273Detail::Ranges;

    static constexpr std::uint16_t ManufacturerId = 0x5449;

    static constexpr std::uint8_t Config1
      = static_cast<std::uint8_t>(static_cast<unsigned>(Avg) << 2U);
    static constexpr std::uint8_t SensorConfig1
      = static_cast<std::uint8_t>(static_cast<unsigned>(Ch) << 4U);
    static constexpr std::uint8_t SensorConfig2
      = Tmag5273Detail::sensorConfig2(RangeConfig{XyRange, ZRange, Angle});
    static constexpr std::uint8_t Config2 = static_cast<std::uint8_t>(OpMode);

    static constexpr std::chrono::milliseconds ReadPeriod = Tmag5273Detail::readPeriod(Avg, Ch);

    static constexpr std::array Init{
      Step::write({.reg = 0x00,                      .payload = {Config1}}
      ), // CONV_AVG
      Step::write(
        {.reg = 0x02, .payload = {SensorConfig1, SensorConfig2}}
      ), // channels; angle and ranges
      Step::write({.reg = 0x07,                         .payload = {0x01}}
      ), // T_CH_EN
      Step::write({.reg = 0x18,                         .payload = {0x10}}
      ), // CONV_STATUS: clear POR
      Step::write({.reg = 0x01,                      .payload = {Config2}}
      ), // the operating mode, last
    };

    struct State {
        std::uint8_t   deviceId{};       ///< DEVICE_ID
        std::uint16_t  manufacturer{};   ///< MANUFACTURER_ID, 0x5449
        std::uint8_t   version{};        ///< VER: 1 the +-40/80 mT part, 2 the +-133/266 mT part
        AveragingCount averaging{Avg};
        ChannelSet     channels{Ch};
        RangeConfig    ranges{XyRange, ZRange, Angle};
        OperatingMode  mode{OpMode};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId     = static_cast<std::uint8_t>(ids[0]);
        state.manufacturer = static_cast<std::uint16_t>((ids[2] << 8U) | ids[1]);
        state.version      = static_cast<std::uint8_t>(state.deviceId & 0x03U);
        state.averaging    = Avg;   // what the Init script wrote
        state.channels     = Ch;
        state.ranges       = RangeConfig{XyRange, ZRange, Angle};
        state.mode         = OpMode;
    }

    /// T, X, Y, Z, CONV_STATUS, ANGLE and MAGNITUDE: 0x10..0x1B in one burst.
    struct Field {
        static constexpr auto Period = ReadPeriod;
        /// Keeps stamp<Field>(), takeGaps<Field>() and unchanged<Field>(): a frame polled
        /// before RESULT_STATUS says the set is complete is counted, not dropped silently.
        static constexpr bool Timestamped = true;

        /// The conversion time of the averaging and channels the part holds now.
        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            return Tmag5273Detail::readPeriod(state.averaging, state.channels);
        }

        static constexpr std::array Steps{Step::read({.reg = 0x10, .count = 12, .offset = 0})};

        struct Sample {
            NanoTesla     x{};   ///< 0 when the channel is not enabled
            NanoTesla     y{};
            NanoTesla     z{};
            MilliDegC     temperature{};
            CentiDegree   angle{};          ///< 0 .. 360 degrees, when ANGLE_EN selects a pair
            std::uint16_t angleCode{};      ///< the same in 1/16 degree
            NanoTesla     magnitude{};      ///< of the angle pair (scale inferred, see above)
            bool          diagnostic{};     ///< DIAG_STATUS: an internal diagnostic failed
            bool          powerOnReset{};   ///< POR: the part reset since the bring-up cleared it
        };

        /// Unchanged while RESULT_STATUS says no conversion is complete (Table 8-27). The data
        /// sheet is silent on whether a read clears it; if not, a read inside one conversion period
        /// repeats the last set as a new sample.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            auto const status = data.u8(8);
            if((status & 0x01U) == 0) { return Outcome<Sample>::unchanged(); }
            auto const xyBr = Tmag5273Detail::rangeNt(state.version, state.ranges.xy);
            auto const zBr  = Tmag5273Detail::rangeNt(state.version, state.ranges.z);
            Sample     s{};
            if(Tmag5273Detail::hasX(state.channels)) {
                s.x = Units::nanoTesla(Tmag5273Detail::fieldNt(data.s16be(2), xyBr));
            }
            if(Tmag5273Detail::hasY(state.channels)) {
                s.y = Units::nanoTesla(Tmag5273Detail::fieldNt(data.s16be(4), xyBr));
            }
            if(Tmag5273Detail::hasZ(state.channels)) {
                s.z = Units::nanoTesla(Tmag5273Detail::fieldNt(data.s16be(6), zBr));
            }
            // 25 degC + (code - 17508) / 58, in millidegrees
            s.temperature = Units::milliDegC(
              25000 + (static_cast<std::int32_t>(data.s16be(0)) - 17508) * 1000 / 58);
            s.angleCode = static_cast<std::uint16_t>(data.be16(9) & 0x1FFFU);
            // 1/16 degree is 6.25 centidegrees: x 25 / 4, rounded
            s.angle = Units::centiDegree((static_cast<std::int32_t>(s.angleCode) * 25 + 2) / 4);
            // every pair starts with X or Y, so X_Y_RANGE
            s.magnitude = Units::nanoTesla(
              static_cast<std::int32_t>(static_cast<std::int64_t>(data.u8(11)) * xyBr / 128));
            s.diagnostic   = (status & 0x02U) != 0;
            s.powerOnReset = (status & 0x10U) != 0;
            return Outcome<Sample>::ok(s);
        }
    };

    /// CONV_AVG (DEVICE_CONFIG_1). CRC, magnet temperature compensation and the read mode are
    /// written 0 with it.
    struct Averaging {
        using Value                        = AveragingCount;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(static_cast<unsigned>(value) << 2U);
            return Step::writeBuffer({.reg = 0x00, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.averaging = value;
        }
    };

    /// MAG_CH_EN (SENSOR_CONFIG_1); SLEEPTIME is written 0 (1 ms, used only in
    /// wake-up-and-sleep).
    struct MagneticChannels {
        using Value                        = ChannelSet;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(static_cast<unsigned>(value) << 4U);
            return Step::writeBuffer({.reg = 0x02, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.channels = value;
        }
    };

    /// X_Y_RANGE, Z_RANGE and ANGLE_EN (SENSOR_CONFIG_2).
    struct Ranges {
        using Value                        = RangeConfig;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(Tmag5273Detail::sensorConfig2(value));
            return Step::writeBuffer({.reg = 0x03, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.ranges = value;
        }
    };

    /// OPERATING_MODE (DEVICE_CONFIG_2); the rest of the register written 0. In sleep and
    /// wake-up-and-sleep the part is woken by any address on the bus (6.4.2, 6.4.3), so the
    /// Field poll would take it out of the mode it was put in: park the read first,
    /// `period<Field>(0ms)`, and give it its period back with continuous or standby.
    struct Operating {
        using Value                        = OperatingMode;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.mode = value;
        }
    };

    using Reads  = List<Field>;
    using Writes = List<Averaging, MagneticChannels, Ranges, Operating>;
};

}   // namespace Kvasir::I2C::Chips
