#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Lsm9ds1Detail {
    /// CTRL_REG6_XL FS_XL (4:3), in the datasheet's own order: 00 +-2 g, 01 +-16 g, 10 +-4 g,
    /// 11 +-8 g.
    enum class AccelRange : std::uint8_t { g2 = 0, g16 = 1, g4 = 2, g8 = 3 };
    /// CTRL_REG1_G FS_G (4:3): 00 245 dps, 01 500 dps, 11 2000 dps; 10 is not available.
    enum class GyroRange : std::uint8_t { dps245 = 0, dps500 = 1, dps2000 = 3 };
    /// ODR_G / ODR_XL (7:5): 14.9, 59.5, 119, 238, 476 and 952 Hz while the gyroscope runs
    /// (Table 46). 0 is power-down. In accelerometer-only mode ODR_XL codes 1 and 2 are 10 and
    /// 50 Hz instead (Table 68).
    enum class Odr : std::uint8_t {
        hz14_9 = 1,
        hz59_5 = 2,
        hz119  = 3,
        hz238  = 4,
        hz476  = 5,
        hz952  = 6
    };
    /// CTRL_REG2_M FS (6:5): +-4, +-8, +-12, +-16 gauss.
    enum class MagRange : std::uint8_t { gauss4 = 0, gauss8 = 1, gauss12 = 2, gauss16 = 3 };

    /// Per LSB, in the datasheet's own order for FS_XL: 0.061, 0.732, 0.122, 0.244 mg.
    inline constexpr std::array<MicroG, 4> AccelPerCount{Units::microG(61),
                                                         Units::microG(732),
                                                         Units::microG(122),
                                                         Units::microG(244)};
    /// Per LSB for FS_G: 8.75, 17.5 and 70 mdps (2 is not available).
    inline constexpr std::array<MicroDegPerSec, 4> RatePerCount{Units::microDegPerSec(8750),
                                                                Units::microDegPerSec(17500),
                                                                Units::microDegPerSec(0),
                                                                Units::microDegPerSec(70000)};
    /// Per LSB for the magnetometer's FS, in tenths of a nanotesla: 146, 292, 438 and 584
    /// ugauss (1 gauss is 100000 nT). The datasheet's table rounds these to 0.14, 0.29, 0.43
    /// and 0.58 mgauss -- 4 % low at +-4 gauss; the magnetometer die's sensitivities are 6842,
    /// 3421, 2281 and 1711 LSB/gauss, which is what Linux st_magn_core.c uses.
    inline constexpr std::array<std::uint16_t, 4> FieldPerCountDeciNt{146, 292, 438, 584};

    /// FS_G 10 is not available; a CTRL_REG1_G value carrying it is written with 11 (2000 dps)
    /// instead, so the range decode() scales by is always one the part has.
    [[nodiscard]] constexpr std::uint8_t validGyroConfig(std::uint8_t ctrl1g) {
        return ((ctrl1g >> 3U) & 0x03U) == 2U ? static_cast<std::uint8_t>(ctrl1g | 0x18U) : ctrl1g;
    }

    /// The sampling period each ODR asks for, rounded up to whole milliseconds so the part is
    /// never read faster than it converts: power-down, then 14.9, 59.5, 119, 238, 476, 952 Hz.
    inline constexpr std::array<std::chrono::milliseconds, 8> OdrPeriod{
      std::chrono::milliseconds{0},
      std::chrono::milliseconds{68},
      std::chrono::milliseconds{17},
      std::chrono::milliseconds{9},
      std::chrono::milliseconds{5},
      std::chrono::milliseconds{3},
      std::chrono::milliseconds{2},
      std::chrono::milliseconds{0}};

    /// The accelerometer/gyroscope half's State: WHO_AM_I, and the full scales and rate the
    /// CTRL registers hold now, which decode() and period() follow.
    struct AgState : Groups::DeviceId {
        std::uint8_t accelFs{};   ///< CTRL_REG6_XL FS_XL
        std::uint8_t gyroFs{};    ///< CTRL_REG1_G FS_G
        std::uint8_t odr{};       ///< CTRL_REG1_G ODR_G, which also paces the accelerometer
    };

    struct MagState : Groups::DeviceId {
        std::uint8_t fs{};   ///< CTRL_REG2_M FS
    };
}   // namespace Lsm9ds1Detail

/// STMicroelectronics LSM9DS1 9-DoF module. It is two I2C devices in one package and is
/// modelled as two descriptions: the accelerometer and gyroscope answer at 0x6A/0x6B and
/// the magnetometer at 0x1C/0x1E, each with its own WHO_AM_I and its own register file.
/// Instantiate whichever halves the board exposes.
///
/// Accelerometer and gyroscope (`Lsm9ds1Ag`): WHO_AM_I 0x0F (0x68), CTRL_REG1_G 0x10
/// (ODR_G 7:5, FS_G 4:3, BW_G 1:0), CTRL_REG6_XL 0x20 (ODR_XL 7:5, FS_XL 4:3), CTRL_REG8
/// 0x22 whose IF_ADD_INC bit is set out of reset so a burst read auto-increments,
/// OUT_TEMP_L 0x15 (12-bit, 16 LSB/degC with 0 at 25 degC), STATUS_REG 0x17 (XLDA 0, GDA 1,
/// TDA 2), OUT_X_L_G 0x18 and OUT_X_L_XL 0x28, both six bytes little endian.
///
/// Note the accelerometer's full-scale encoding is not in order: 00 is +-2 g, 01 is +-16 g,
/// 10 is +-4 g and 11 is +-8 g. The gyroscope's is 00 = 245 dps, 01 = 500, 11 = 2000, with
/// 10 unused. The template ranges are the Initial of the two write groups, so a full scale can
/// be changed at run time and decode() follows; the Motion period follows the ODR.
///
/// A frame whose STATUS_REG says neither the gyroscope nor the accelerometer has new data is
/// reported as Outcome::unchanged().
///
/// Bring-up resets the half first -- CTRL_REG8 SW_RESET, then BOOT, 50 ms each, as Linux
/// st_lsm6dsx does for this part -- so a warm part left with a FIFO, interrupts or other
/// ranges by earlier firmware starts from its defaults; then CTRL_REG8 = 0x44: BDU, so the
/// low and high byte of an output are from the same sample, and IF_ADD_INC as out of reset.
/// The gyroscope wants its first samples discarded (Table 12: 3 at 119 Hz), which the 30 ms
/// after the configuration covers.
template<Lsm9ds1Detail::AccelRange AccelFs = Lsm9ds1Detail::AccelRange::g2,
         Lsm9ds1Detail::GyroRange  GyroFs  = Lsm9ds1Detail::GyroRange::dps245,
         Lsm9ds1Detail::Odr        Odr     = Lsm9ds1Detail::Odr::hz119>
struct Lsm9ds1Ag {
    static constexpr std::string_view Name = "LSM9DS1-AG";
    /// ST LSM9DS1. LSM9DS1.md:1534 (WHO_AM_I 0Fh of the accelerometer and gyroscope: 68h) and :1236
    /// (WHO_AM_I_M 0Fh of the magnetometer: 3Dh).
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0F, 1, true, 0xFF, 0x68},
    };
    static constexpr Address7    Address       = 0x6B;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 2> Addresses{0x6A, 0x6B};

    static constexpr std::array<MicroG, 4>         AccelPerCount = Lsm9ds1Detail::AccelPerCount;
    static constexpr std::array<MicroDegPerSec, 4> RatePerCount  = Lsm9ds1Detail::RatePerCount;

    static constexpr std::uint8_t Ctrl1G = static_cast<std::uint8_t>(
      (static_cast<unsigned>(Odr) << 5) | (static_cast<unsigned>(GyroFs) << 3));
    static constexpr std::uint8_t Ctrl6Xl = static_cast<std::uint8_t>(
      (static_cast<unsigned>(Odr) << 5) | (static_cast<unsigned>(AccelFs) << 3));

    static constexpr std::chrono::milliseconds OutputPeriod
      = Lsm9ds1Detail::OdrPeriod[static_cast<std::size_t>(Odr)];

    static constexpr auto StartupDelay = std::chrono::milliseconds{20};

    static constexpr std::array Init{
      Step::write({.reg     = 0x22,
                   .payload = {0x05},
                   .delay   = std::chrono::milliseconds{50}}),   // CTRL_REG8: SW_RESET
      Step::write(
        {.reg = 0x22, .payload = {0x84}, .delay = std::chrono::milliseconds{50}}),   // BOOT
      Step::write({.reg = 0x22, .payload = {0x44}}),   // BDU, IF_ADD_INC
      Step::write(
        {.reg = 0x10, .payload = {Ctrl1G}}),   // gyroscope on, which also runs the accelerometer
      Step::write({.reg     = 0x20,
                   .payload = {Ctrl6Xl},
                   .delay   = std::chrono::milliseconds{30}}),   // accelerometer ODR and full scale
    };

    using State = Lsm9ds1Detail::AgState;

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.accelFs  = static_cast<std::uint8_t>(AccelFs);
        state.gyroFs   = static_cast<std::uint8_t>(GyroFs);
        state.odr      = static_cast<std::uint8_t>(Odr);
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    struct Motion {
        static constexpr auto Period = OutputPeriod;
        /// So the frames that said nothing new are counted (unchanged<Motion>()).
        static constexpr bool Timestamped = true;

        /// The rate CTRL_REG1_G holds now, which a GyroConfig write changes.
        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            auto const p = Lsm9ds1Detail::OdrPeriod[state.odr & 0x07U];
            return p != std::chrono::milliseconds::zero() ? p : OutputPeriod;
        }

        /// Temperature, status, then the gyroscope and accelerometer bursts; IF_ADD_INC is set
        /// out of reset, so each burst auto-increments. STATUS_REG is its own read: the
        /// datasheet's multiple-read sequence (3.3, Figure 7) goes from OUT_TEMP straight to
        /// OUT_X_G and does not say a burst passes through 0x17.
        static constexpr std::array Steps{
          Step::read({.reg = 0x15, .count = 2, .offset = 0}),   // OUT_TEMP_L/H
          Step::read({.reg = 0x17, .count = 1, .offset = 2}),   // STATUS_REG
          Step::read({.reg = 0x18, .count = 6, .offset = 3}),
          Step::read({.reg = 0x28, .count = 6, .offset = 9}),
        };

        struct Sample {
            std::array<MicroG, 3>         accel{};
            std::array<MilliDegPerSec, 3> gyro{};
            MilliDegC                     temperature{};   ///< 0.001 degC
        };

        [[nodiscard]] static constexpr MicroG toAccel(std::int16_t raw,
                                                      unsigned     fs) {
            return Units::microG(static_cast<std::int64_t>(raw)
                                 * Units::value(AccelPerCount[fs & 0x03U]));
        }

        [[nodiscard]] static constexpr MilliDegPerSec toRate(std::int16_t raw,
                                                             unsigned     fs) {
            return Units::milliDegPerSec(static_cast<std::int64_t>(raw)
                                         * Units::value(RatePerCount[fs & 0x03U]) / 1000);
        }

        /// 12-bit, 16 LSB per degree, zero at 25 degC; at the full scales the part holds now.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            auto const status = data.u8(2);
            if((status & 0x03U) == 0) { return Outcome<Sample>::unchanged(); }   // no XLDA, no GDA
            Sample     sample{};
            auto const t       = Bytes::signExtend(data.le16(0) & 0x0FFFU, 12);
            sample.temperature = Units::milliDegC(25000 + t * 1000 / 16);
            for(std::size_t i = 0; i < 3; ++i) {
                sample.gyro[i]  = toRate(data.s16le(3 + 2 * i), state.gyroFs);
                sample.accel[i] = toAccel(data.s16le(9 + 2 * i), state.accelFs);
            }
            return Outcome<Sample>::ok(sample);
        }
    };

    /// CTRL_REG1_G: the gyroscope's ODR (7:5, which paces the accelerometer too) and full
    /// scale (4:3), changeable at run time.
    struct GyroConfig {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Ctrl1G;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(Lsm9ds1Detail::validGyroConfig(value));
            return Step::writeBuffer({.reg = 0x10, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            auto const written = Lsm9ds1Detail::validGyroConfig(value);
            state.gyroFs       = static_cast<std::uint8_t>((written >> 3U) & 0x03U);
            state.odr          = static_cast<std::uint8_t>(written >> 5U);
        }
    };

    /// CTRL_REG6_XL: the accelerometer's ODR (7:5) and full scale (4:3).
    struct AccelConfig {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Ctrl6Xl;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x20, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.accelFs = static_cast<std::uint8_t>((value >> 3U) & 0x03U);
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<GyroConfig, AccelConfig>;
};

/// The magnetometer half. WHO_AM_I_M 0x0F (0x3D), CTRL_REG1_M 0x20 (TEMP_COMP 7, OM 6:5,
/// DO 4:2), CTRL_REG2_M 0x21 (FS 6:5), CTRL_REG3_M 0x22 (MD 1:0; the reset value 0x03 is
/// power-down, so it must be written to run), CTRL_REG4_M 0x23 (OMZ 3:2), STATUS_REG_M 0x27
/// (ZYXDA bit 3), OUT_X_L_M 0x28.
///
/// Unlike the accelerometer half, auto-increment here is the MSB of the sub-address, so a
/// burst read addresses 0x80 | reg. The template range is the Initial of the Range write
/// group, so it can be changed at run time and decode() follows.
template<Lsm9ds1Detail::MagRange MagFs = Lsm9ds1Detail::MagRange::gauss4>
struct Lsm9ds1Mag {
    static constexpr std::string_view Name = "LSM9DS1-M";
    /// ST LSM9DS1. LSM9DS1.md:1534 (WHO_AM_I 0Fh of the accelerometer and gyroscope: 68h) and :1236
    /// (WHO_AM_I_M 0Fh of the magnetometer: 3Dh).
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0F, 1, true, 0xFF, 0x3D},
    };
    static constexpr Address7    Address       = 0x1E;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 2> Addresses{0x1C, 0x1E};

    static constexpr std::array<std::uint16_t, 4> FieldPerCountDeciNt
      = Lsm9ds1Detail::FieldPerCountDeciNt;

    static constexpr std::uint8_t Ctrl2M
      = static_cast<std::uint8_t>(static_cast<unsigned>(MagFs) << 5);

    static constexpr auto StartupDelay = std::chrono::milliseconds{20};

    static constexpr std::array Init{
      Step::write({.reg = 0x20, .payload = {0xFC}}),     // temp comp, ultra-high perf, 80 Hz
      Step::write({.reg = 0x21, .payload = {Ctrl2M}}),   // full scale
      Step::write({.reg = 0x23, .payload = {0x0C}}),     // Z axis ultra-high performance
      Step::write({.reg = 0x24, .payload = {0x40}}),     // CTRL_REG5_M: BDU
      Step::write({.reg     = 0x22,
                   .payload = {0x00},
                   .delay   = std::chrono::milliseconds{20}}),   // continuous conversion
    };

    using State = Lsm9ds1Detail::MagState;

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.fs       = static_cast<std::uint8_t>(MagFs);
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    struct Field {
        static constexpr auto Period = std::chrono::milliseconds{50};
        /// So the frames that said nothing new are counted (unchanged<Field>()).
        static constexpr bool Timestamped = true;

        /// 0x80 sets the sub-address auto-increment for this half of the part: STATUS_REG_M,
        /// then the six output bytes.
        static constexpr std::array Steps{Step::read({.reg = 0xA7, .count = 7, .offset = 0})};

        struct Sample {
            NanoTesla x{};
            NanoTesla y{};
            NanoTesla z{};
        };

        [[nodiscard]] static constexpr NanoTesla field(std::int16_t raw,
                                                       unsigned     fs) {
            return Units::nanoTesla(static_cast<std::int64_t>(raw) * FieldPerCountDeciNt[fs & 0x03U]
                                    / 10);
        }

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if((data.u8(0) & 0x08U) == 0) { return Outcome<Sample>::unchanged(); }   // ZYXDA
            return Outcome<Sample>::ok({field(data.s16le(1), state.fs),
                                        field(data.s16le(3), state.fs),
                                        field(data.s16le(5), state.fs)});
        }
    };

    /// CTRL_REG2_M: FS in bits 6:5, changeable at run time.
    struct Range {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Ctrl2M;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x21, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.fs = static_cast<std::uint8_t>((value >> 5U) & 0x03U);
        }
    };

    using Reads  = List<Field>;
    using Writes = List<Range>;
};

}   // namespace Kvasir::I2C::Chips
