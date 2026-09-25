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

namespace Lsm6ds3Detail {
    /// CTRL1_XL FS_XL (3:2), in ST's order: 00 +-2 g, 01 +-16 g, 10 +-4 g, 11 +-8 g.
    enum class AccelRange : std::uint8_t { g2 = 0, g16 = 1, g4 = 2, g8 = 3 };
    /// CTRL2_G FS_G (3:2): 245/250, 500, 1000, 2000 dps.
    enum class GyroRange : std::uint8_t { dps250 = 0, dps500 = 1, dps1000 = 2, dps2000 = 3 };

    /// Per LSB in FS_XL's order (0.061, 0.488, 0.122, 0.244 mg), and in FS_G's (8.75, 17.5,
    /// 35 and 70 mdps).
    inline constexpr std::array<MicroG, 4>         AccelPerCount{Units::microG(61),
                                                                 Units::microG(488),
                                                                 Units::microG(122),
                                                                 Units::microG(244)};
    inline constexpr std::array<MicroDegPerSec, 4> RatePerCount{Units::microDegPerSec(8750),
                                                                Units::microDegPerSec(17500),
                                                                Units::microDegPerSec(35000),
                                                                Units::microDegPerSec(70000)};

    /// The chip State: what the bring-up read, and the full scales the two CTRL registers
    /// hold now, which decode() scales by and a Config write changes (applied()).
    struct State : Groups::DeviceId {
        std::uint8_t accelFs{};   ///< CTRL1_XL FS_XL, 0..3 in ST's order
        std::uint8_t gyroFs{};    ///< CTRL2_G FS_G
    };
}   // namespace Lsm6ds3Detail

/// ST LSM6DS3 / LSM6DS3TR-C (DocID026899 / DocID029330). One-byte sub-address, auto-increment
/// on by default (CTRL3_C.IF_INC). Bring-up: WHO_AM_I 0x0F = 0x69 (0x6A on the TR-C); CTRL3_C
/// 0x12 = 0x44 (BDU so a burst never straddles an update, IF_INC kept); CTRL1_XL 0x10 = 0x40
/// (104 Hz, 2 g: 0.061 mg/LSB); CTRL2_G 0x11 = 0x40 (104 Hz, 245 dps: 8.75 mdps/LSB), then
/// the gyro's turn-on time (80 ms). Data: 14 bytes from OUT_TEMP_L 0x20: temperature, gyro
/// xyz, accel xyz, all int16 little-endian, reported as quantities at those scales.
///
/// The temperature scale differs between the two parts and is picked by WHO_AM_I: 16 LSB/degC
/// on the LSM6DS3 (0x69), 256 LSB/degC on the LSM6DS3TR-C (0x6A) and the LSM6DSO (0x6C), 0 at
/// 25 degC on all of them.
///
/// The template ranges are the Initial of the AccelConfig / GyroConfig write groups, so the
/// application can change a full scale at run time and decode() follows from the write's
/// completion on. 0x6A (SA0 low) or 0x6B.
template<Lsm6ds3Detail::AccelRange Accel = Lsm6ds3Detail::AccelRange::g2,
         Lsm6ds3Detail::GyroRange  Gyro  = Lsm6ds3Detail::GyroRange::dps250>
struct Lsm6ds3x {
    static constexpr std::string_view Name = "LSM6DS3";
    /// ST LSM6DS3. LSM6DS3.md:2157: WHO_AM_I (0Fh) "fixed at 69h"; 6Ah is the LSM6DS3TR-C, whose data
    /// sheet is not in the folder (Linux st_lsm6dsx_core.c: `.wai = 0x6a`).
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0F, 1, true, 0xFF, 0x69, 0x6A},
    };
    static constexpr Address7                Address = 0x6A;
    static constexpr std::array<Address7, 2> Addresses{0x6A, 0x6B};
    static constexpr std::size_t             RegisterBytes = 1;

    /// Wait after power-up before the first access. The 15 ms is a margin, not a figure from
    /// this datasheet.
    static constexpr auto StartupDelay = std::chrono::milliseconds{15};

    /// Wait for the gyroscope to turn on from power-down. The 80 ms is a margin, not a figure
    /// from this datasheet.
    static constexpr std::chrono::milliseconds GyroSettle{80};

    static constexpr std::uint8_t Ctrl1Xl
      = static_cast<std::uint8_t>(0x40U | (static_cast<unsigned>(Accel) << 2U));
    static constexpr std::uint8_t Ctrl2G
      = static_cast<std::uint8_t>(0x40U | (static_cast<unsigned>(Gyro) << 2U));
    /// CTRL3_C: BDU (bit 6) and IF_INC (bit 2, the reset value).
    static constexpr std::uint8_t Ctrl3C = 0x44;

    /// After WHO_AM_I, CTRL3_C SW_RESET and then BOOT, 50 ms each (Linux st_lsm6dsx does the
    /// same): a warm part keeps what earlier firmware set -- the FIFO, interrupts, filters --
    /// through a bring-up otherwise.
    static constexpr std::array Init{
      Step::write({.reg = 0x12, .payload = {0x01}, .delay = std::chrono::milliseconds{50}}),
      Step::write({.reg = 0x12, .payload = {0x80}, .delay = std::chrono::milliseconds{50}}),
      Step::write({.reg = 0x12, .payload = {Ctrl3C}}),    // BDU, IF_INC
      Step::write({.reg = 0x10, .payload = {Ctrl1Xl}}),   // 104 Hz and the accelerometer's range
      Step::write({.reg     = 0x11,
                   .payload = {Ctrl2G},
                   .delay   = GyroSettle}),   // 104 Hz and the gyro's, then its turn-on
    };

    using State = Lsm6ds3Detail::State;

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.accelFs  = static_cast<std::uint8_t>(Accel);   // what the Init script wrote
        state.gyroFs   = static_cast<std::uint8_t>(Gyro);
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    /// The scales at the ranges the bring-up sets (0.061 mg and 8.75 mdps an LSB at the defaults).
    static constexpr MicroG AccelPerCount
      = Lsm6ds3Detail::AccelPerCount[static_cast<std::size_t>(Accel)];
    static constexpr MicroDegPerSec RatePerCount
      = Lsm6ds3Detail::RatePerCount[static_cast<std::size_t>(Gyro)];

    /// Temperature LSB per degC: 16 on the LSM6DS3, 256 on the TR-C and the LSM6DSO.
    [[nodiscard]] static constexpr std::int32_t temperatureLsbPerDegC(std::uint16_t deviceId) {
        return deviceId == 0x69 ? 16 : 256;
    }

    struct Motion {
        static constexpr auto       Period = std::chrono::milliseconds{20};
        static constexpr std::array Steps{Step::read({.reg = 0x20, .count = 14})};

        struct Sample {
            std::array<MicroG, 3>         accel{};
            std::array<MilliDegPerSec, 3> gyro{};
            CentiDegC                     temperature{};   ///< 0 at 25 degC
        };

        /// At the full scales the part holds now (a Config write changes them), and the
        /// temperature scale of the part WHO_AM_I found.
        [[nodiscard]] static constexpr Sample decode(Bytes        data,
                                                     State const& state) {
            Sample     sample{};
            auto const perCount = Units::value(Lsm6ds3Detail::AccelPerCount[state.accelFs & 0x03U]);
            auto const perRate  = Units::value(Lsm6ds3Detail::RatePerCount[state.gyroFs & 0x03U]);
            sample.temperature  = Units::centiDegC(2500
                                                   + static_cast<std::int32_t>(data.s16le(0)) * 100
                                                       / temperatureLsbPerDegC(state.deviceId));
            for(std::size_t i = 0; i < 3; ++i) {
                auto const rate = static_cast<std::int64_t>(data.s16le(2 + 2 * i)) * perRate / 1000;
                sample.gyro[i]  = Units::milliDegPerSec(rate);
                sample.accel[i]
                  = Units::microG(static_cast<std::int64_t>(data.s16le(8 + 2 * i)) * perCount);
            }
            return sample;
        }
    };

    /// CTRL1_XL, so the accelerometer's full scale (and ODR) can be changed at run time. The
    /// new scale is what decode() multiplies by from the write's completion on.
    struct AccelConfig {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Ctrl1Xl;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x10, .offset = 0, .count = 1});
        }

        /// FS_XL is bits 3:2.
        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.accelFs = static_cast<std::uint8_t>((value >> 2U) & 0x03U);
        }
    };

    /// CTRL2_G, the gyroscope's: FS_G in bits 3:2.
    struct GyroConfig {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Ctrl2G;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x11, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gyroFs = static_cast<std::uint8_t>((value >> 2U) & 0x03U);
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<AccelConfig, GyroConfig>;
};

/// ST LSM6DSO (DS12140), the LSM6DS3's successor. For what this description touches it is the
/// same part: CTRL1_XL 0x10 and CTRL2_G 0x11 take the same codes (0x40: 104 Hz, 2 g at 0.061
/// mg/LSB and 250 dps at 8.75 mdps/LSB), and the same 14 bytes start at OUT_TEMP_L 0x20
/// (256 LSB/degC, 0 at 25 degC). Only WHO_AM_I differs: 0x6C. 0x6A (SA0 low) or 0x6B.
template<Lsm6ds3Detail::AccelRange Accel = Lsm6ds3Detail::AccelRange::g2,
         Lsm6ds3Detail::GyroRange  Gyro  = Lsm6ds3Detail::GyroRange::dps250>
struct Lsm6dsoX : Lsm6ds3x<Accel, Gyro> {
    static constexpr std::string_view Name = "LSM6DSO";
    /// ST LSM6DSO. lsm6dso.md:2768: WHO_AM_I (0Fh) "fixed at 6Ch".
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0F, 1, true, 0xFF, 0x6C},
    };
};

/// The two parts at the default ranges: +-2 g and 250 dps.
using Lsm6ds3 = Lsm6ds3x<>;
using Lsm6dso = Lsm6dsoX<>;

}   // namespace Kvasir::I2C::Chips
