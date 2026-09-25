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

namespace Lsm303agrDetail {
    /// CTRL_REG4_A FS (5:4): +-2, +-4, +-8, +-16 g.
    enum class AccelRange : std::uint8_t { g2 = 0, g4 = 1, g8 = 2, g16 = 3 };

    /// CTRL_REG1_A ODR (7:4). 0 is power-down. Code 8, 1620 Hz, exists in low-power mode only
    /// (Table 35), and this description runs high resolution (LPen 0, HR 1; Table 14), so it is
    /// not offered.
    enum class AccelOdr : std::uint8_t {
        hz1    = 1,
        hz10   = 2,
        hz25   = 3,
        hz50   = 4,
        hz100  = 5,
        hz200  = 6,
        hz400  = 7,
        hz1344 = 9,
    };

    /// CFG_REG_A_M ODR (3:2): 10, 20, 50, 100 Hz.
    enum class MagOdr : std::uint8_t { hz10 = 0, hz20 = 1, hz50 = 2, hz100 = 3 };

    /// High-resolution mode: 12 significant bits, so the raw word is shifted by four and
    /// then scaled. ST's factors at +-2/4/8/16 g, in micro-g per count of the *shifted*
    /// value: 0.98, 1.95, 3.9 and 11.72 mg.
    inline constexpr std::array<MicroG, 4> AccelPerCount{Units::microG(980),
                                                         Units::microG(1950),
                                                         Units::microG(3900),
                                                         Units::microG(11720)};

    /// The period of each ODR code, rounded up to whole milliseconds; 0 for a code with none.
    [[nodiscard]] constexpr std::chrono::milliseconds odrPeriod(AccelOdr odr) {
        switch(odr) {
        case AccelOdr::hz1:    return std::chrono::milliseconds{1000};
        case AccelOdr::hz10:   return std::chrono::milliseconds{100};
        case AccelOdr::hz25:   return std::chrono::milliseconds{40};
        case AccelOdr::hz50:   return std::chrono::milliseconds{20};
        case AccelOdr::hz100:  return std::chrono::milliseconds{10};
        case AccelOdr::hz200:  return std::chrono::milliseconds{5};
        case AccelOdr::hz400:  return std::chrono::milliseconds{3};
        case AccelOdr::hz1344: return std::chrono::milliseconds{1};
        }
        return std::chrono::milliseconds{0};
    }

    struct AccelState : Groups::DeviceId {
        std::uint8_t fs{};   ///< CTRL_REG4_A FS as written now
    };
}   // namespace Lsm303agrDetail

/// STMicroelectronics LSM303AGR: a 3-axis accelerometer and a 3-axis magnetometer that
/// answer at two different I2C addresses, so this is two descriptions -- the same shape the
/// LSM9DS1 has. Instantiate whichever halves the board exposes.
///
/// Accelerometer (`Lsm303agrAccel`) at 0x19: WHO_AM_I_A 0x0F reads 0x33, OUT_TEMP_L_A 0x0C,
/// TEMP_CFG_REG_A 0x1F (TEMP_EN in 7:6), CTRL_REG1_A 0x20 (X/Y/Z enable 2:0, LPen 3, ODR
/// 7:4), CTRL_REG4_A 0x23 (SPI 0, ST 2:1, HR 3, FS 5:4, BLE 6, BDU 7), STATUS_REG_A 0x27
/// (ZYXDA bit 3), OUT_X_L_A 0x28.
///
/// Two things about this part are easy to get wrong, so they are spelt out here:
///
///  * Auto-increment is the MSB of the sub-address, as on the LSM9DS1's magnetometer
///    half -- a seven-byte burst reads from 0xA7 (STATUS_REG_A, then the axes), not 0x27.
///  * The sensitivity is not the LSM9DS1's. The reading is left-aligned in 16 bits with
///    only the top bits significant (12 in high resolution, 10 normal, 8 low power), and
///    ST's own conversion is `(lsb / 16) * 0.98` mg for high resolution at +-2 g -- the /16
///    is the left-alignment and the 0.98 a calibration factor. Temperature is
///    `(lsb / 64) / 4 + 25` degC.
///
/// A frame whose STATUS_REG_A has no ZYXDA is Outcome::unchanged(). The full scale is the
/// Initial of the Range write group and can be changed at run time; decode() follows.
template<Lsm303agrDetail::AccelRange Fs  = Lsm303agrDetail::AccelRange::g2,
         Lsm303agrDetail::AccelOdr   Odr = Lsm303agrDetail::AccelOdr::hz50>
struct Lsm303agrAccel {
    static constexpr std::string_view Name = "LSM303AGR-A";
    /// ST LSM303AGR. LSM303AGR.md:1654 (WHO_AM_I_A 0Fh = 33h) and :1713 (WHO_AM_I_M 4Fh = 40h).
    /// Configuration: TEMP_CFG_REG_A (1Fh) TEMP_EN[1:0] = 11b, the temperature sensor on
    /// (LSM303AGR.md:1298).
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0F, 1, true, 0xFF, 0x33},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"temp-en", 0x1F, 1, true, 0xC0, 0xC0},
    };
    static constexpr Address7                Address = 0x19;
    static constexpr std::array<Address7, 1> Addresses{0x19};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::array<MicroG, 4> AccelPerCount = Lsm303agrDetail::AccelPerCount;

    static constexpr std::uint8_t FsCode = static_cast<std::uint8_t>(Fs);

    static constexpr std::uint8_t Ctrl1
      = static_cast<std::uint8_t>((static_cast<unsigned>(Odr) << 4) | 0x07U);   // XYZ enabled
    static constexpr std::uint8_t Ctrl4
      = static_cast<std::uint8_t>(0x80U | (FsCode << 4) | 0x08U);   // BDU, FS, HR

    static constexpr auto StartupDelay = std::chrono::milliseconds{20};

    static constexpr std::array Init{
      Step::write({.reg = 0x20, .payload = {Ctrl1}}),
      Step::write({.reg = 0x23, .payload = {Ctrl4}}),
      // TEMP_EN; then the high-resolution turn-on time, 7/ODR (Table 14): the samples before
      // it are not settled.
      Step::write({.reg = 0x1F, .payload = {0xC0}, .delay = 7 * Lsm303agrDetail::odrPeriod(Odr)}),
    };

    using State = Lsm303agrDetail::AccelState;

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.fs       = FsCode;
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    struct Motion {
        static constexpr auto Period = std::chrono::milliseconds{50};
        /// So the frames that said nothing new are counted (unchanged<Motion>()).
        static constexpr bool Timestamped = true;

        /// 0xA7 and 0x8C: the MSB is what turns a burst into an auto-incrementing read.
        static constexpr std::array Steps{
          Step::read({.reg = 0xA7, .count = 7, .offset = 0}),    // STATUS_REG_A, OUT_X_L_A ..
          Step::read({.reg = 0x8C, .count = 2, .offset = 7})};   // OUT_TEMP_L_A

        struct Sample {
            MicroG    x{};
            MicroG    y{};
            MicroG    z{};
            MilliDegC temperature{};   ///< 0.001 degC
        };

        /// Left-aligned: the twelve significant bits sit in 15:4.
        [[nodiscard]] static constexpr std::int32_t accel(std::int16_t raw,
                                                          std::uint8_t fs) {
            return static_cast<std::int32_t>(static_cast<std::int64_t>(raw >> 4)
                                             * Units::value(AccelPerCount[fs & 0x03U]));
        }

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if((data.u8(0) & 0x08U) == 0) { return Outcome<Sample>::unchanged(); }   // ZYXDA
            Sample sample{};
            sample.x = Units::microG(accel(data.s16le(1), state.fs));
            sample.y = Units::microG(accel(data.s16le(3), state.fs));
            sample.z = Units::microG(accel(data.s16le(5), state.fs));
            // (lsb / 64) / 4 + 25 degC, in millidegrees
            sample.temperature
              = Units::milliDegC(static_cast<std::int32_t>(data.s16le(7)) * 1000 / 256 + 25000);
            return Outcome<Sample>::ok(sample);
        }
    };

    /// CTRL_REG4_A's FS (5:4), with BDU and HR kept; changeable at run time.
    struct Range {
        using Value                          = Lsm303agrDetail::AccelRange;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Fs;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(0x88U | (static_cast<unsigned>(value) << 4U));
            return Step::writeBuffer({.reg = 0x23, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.fs = static_cast<std::uint8_t>(value);
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<Range>;
};

/// The magnetometer half, at 0x1E. WHO_AM_I_M 0x4F reads 0x40, CFG_REG_A_M 0x60 (MD 1:0,
/// ODR 3:2, LP 4, SOFT_RST 5, REBOOT 6, COMP_TEMP_EN 7), CFG_REG_B_M 0x61, CFG_REG_C_M 0x62
/// (INT_MAG 0, BLE 3, BDU 4, I2C_DIS 5), STATUS_REG_M 0x67 (Zyxda bit 3), OUTX_L_REG_M 0x68.
///
/// MD resets to idle (3), so the part measures nothing until CFG_REG_A_M is written --
/// 0 selects continuous. The full scale is fixed at +-50 gauss and one count is 1.5 mgauss,
/// which is why there is no full-scale parameter here. A frame without Zyxda is
/// Outcome::unchanged().
template<Lsm303agrDetail::MagOdr Odr = Lsm303agrDetail::MagOdr::hz50>
struct Lsm303agrMag {
    static constexpr std::string_view Name = "LSM303AGR-M";
    /// Configuration: CFG_REG_A_M (60h) MD[1:0] = 00b continuous; CFG_REG_B_M (61h) OFF_CANC, which
    /// the offset specification assumes (LSM303AGR.md:852..887); CFG_REG_C_M (62h) BDU.
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x4F, 1, true, 0xFF, 0x40},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{         "continuous", 0x60, 1, true, 0x03, 0x00},
      RegisterCheck{"offset-cancellation", 0x61, 1, true, 0x02, 0x02},
      RegisterCheck{                "bdu", 0x62, 1, true, 0x10, 0x10},
    };
    static constexpr Address7                Address = 0x1E;
    static constexpr std::array<Address7, 1> Addresses{0x1E};
    static constexpr std::size_t             RegisterBytes = 1;

    /// 1.5 mgauss per count, and 1 gauss is 100000 nT, so 150 nT per count.
    static constexpr NanoTesla FieldPerCount = Units::nanoTesla(150);

    /// Continuous mode with temperature compensation on.
    static constexpr std::uint8_t CfgA
      = static_cast<std::uint8_t>(0x80U | (static_cast<unsigned>(Odr) << 2));

    static constexpr auto StartupDelay = std::chrono::milliseconds{20};

    static constexpr std::array Init{
      Step::write({.reg = 0x62, .payload = {0x10}}),   // CFG_REG_C_M: BDU
      // CFG_REG_B_M: OFF_CANC -- the magnetic offset of +-60 mgauss (M_TyOff) is specified
      // with offset cancellation on (4.1.2)
      Step::write({.reg = 0x61, .payload = {0x02}}),
      Step::write(
        {.reg     = 0x60,
         .payload = {CfgA},
         .delay
         = std::chrono::milliseconds{20}}),   // CFG_REG_A_M: out of power-down into continuous
    };

    using State = Groups::DeviceId;

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    struct Field {
        static constexpr auto Period = std::chrono::milliseconds{50};
        /// STATUS_REG_M, then the six output bytes. Addressed at 0x67 without the MSB: the
        /// datasheet's I2C section gives every register the MSB as the increment bit, but ST's
        /// own lsm303agr_reg.c and Linux (st_magn, multi_read_bit false for this part) both
        /// read the magnetometer's bursts without it, and 0xE7 is not a register of this half.
        static constexpr std::array Steps{Step::read({.reg = 0x67, .count = 7, .offset = 0})};

        struct Sample {
            NanoTesla x{};
            NanoTesla y{};
            NanoTesla z{};
        };

        [[nodiscard]] static constexpr NanoTesla field(std::int16_t raw) {
            return Units::nanoTesla(static_cast<std::int64_t>(raw) * Units::value(FieldPerCount));
        }

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data,
                                                              Sample const&) {
            if((data.u8(0) & 0x08U) == 0) { return Outcome<Sample>::unchanged(); }   // Zyxda
            return Outcome<Sample>::ok(
              {field(data.s16le(1)), field(data.s16le(3)), field(data.s16le(5))});
        }
    };

    using Reads = List<Field>;
};

}   // namespace Kvasir::I2C::Chips
