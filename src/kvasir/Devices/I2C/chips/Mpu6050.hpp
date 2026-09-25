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

namespace Mpu6050Detail {
    /// ACCEL_CONFIG AFS_SEL (4:3): +-2, 4, 8, 16 g, at 16384 LSB/g halving with each step.
    enum class AccelRange : std::uint8_t { g2 = 0, g4 = 1, g8 = 2, g16 = 3 };
    /// GYRO_CONFIG FS_SEL (4:3): 250, 500, 1000, 2000 dps, at 131, 65.5, 32.8, 16.4 LSB/(deg/s).
    enum class GyroRange : std::uint8_t { dps250 = 0, dps500 = 1, dps1000 = 2, dps2000 = 3 };

    /// Per LSB in FS_SEL's order: 1 / 131, 1 / 65.5, 1 / 32.8 and 1 / 16.4 deg/s, to the
    /// nearest micro-degree a second.
    inline constexpr std::array<MicroDegPerSec, 4> RatePerCount{Units::microDegPerSec(7634),
                                                                Units::microDegPerSec(15267),
                                                                Units::microDegPerSec(30488),
                                                                Units::microDegPerSec(60976)};

    /// The chip State: WHO_AM_I, and the two range fields as the CONFIG registers hold them
    /// now, which decode() scales by and a write changes (applied()).
    struct State : Groups::DeviceId {
        std::uint8_t accelRange{};   ///< AFS_SEL, 0..3
        std::uint8_t gyroRange{};    ///< FS_SEL, 0..3
    };
}   // namespace Mpu6050Detail

/// InvenSense MPU-6050 (RM-MPU-6000A-00 rev 4.0). One-byte registers, burst reads
/// auto-increment. Bring-up: PWR_MGMT_1 (0x6B) DEVICE_RESET then 100 ms; PWR_MGMT_1 = 0x01
/// (awake, PLL on the X gyro); WHO_AM_I (0x75) = 0x68; SMPLRT_DIV (0x19) = 9 and CONFIG
/// (0x1A) DLPF_CFG = 3: 100 Hz samples, 44 Hz bandwidth; GYRO_CONFIG (0x1B) FS_SEL = 0
/// (250 dps, 131 LSB/dps) and ACCEL_CONFIG (0x1C) AFS_SEL = 0 (2 g, 16384 LSB/g) by
/// default, the template parameters otherwise; INT_ENABLE (0x38) DATA_RDY_EN, so that
/// INT_STATUS (0x3A) DATA_RDY_INT says when a new sample is there. Data: INT_STATUS then 14
/// bytes from 0x3B in one burst: accel xyz, temperature, gyro xyz, int16 big-endian,
/// reported as quantities at those scales. Reading INT_STATUS clears the flag, so a poll
/// that finds it clear is Outcome::unchanged(); fifteen bytes of 0xFF (minus one count on
/// every axis at once) is a bus that answered nothing and is rejected.
///
/// The template ranges are the Initial of the AccelConfig / GyroConfig write groups, so a
/// range can be changed at run time and decode() follows from the write's completion on.
/// 0x68 / 0x69 (AD0).
template<Mpu6050Detail::AccelRange Accel = Mpu6050Detail::AccelRange::g2,
         Mpu6050Detail::GyroRange  Gyro  = Mpu6050Detail::GyroRange::dps250>
struct Mpu6050x {
    static constexpr std::string_view Name = "MPU6050";
    /// InvenSense MPU-6050. MPU6050_RegMap.md:1929..1943, WHO_AM_I (register 117) defaults to 0x68,
    /// bits 6:1.
    /// Configuration (MPU6050_RegMap.md:255, :155, :419): PWR_MGMT_1 SLEEP clear with CLKSEL 1, the
    /// X gyro's PLL; with the DLPF on (DLPF_CFG 3) the gyro runs at 1 kHz, so SMPLRT_DIV 9 is the
    /// 100 Hz output rate; FS_SEL and AFS_SEL 0 are 250 dps and 2 g.
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x75, 1, true, 0x7E, 0x68},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{     "awake-pll", 0x6B, 1, true, 0x47, 0x01},
      RegisterCheck{"sample-divider", 0x19, 1, true, 0xFF, 0x09},
      RegisterCheck{          "dlpf", 0x1A, 1, true, 0x07, 0x03},
      RegisterCheck{   "gyro-250dps", 0x1B, 1, true, 0x18, 0x00},
      RegisterCheck{      "accel-2g", 0x1C, 1, true, 0x18, 0x00},
    };
    static constexpr Address7                Address = 0x68;
    static constexpr std::array<Address7, 2> Addresses{0x68, 0x69};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::uint8_t GyroConfigByte
      = static_cast<std::uint8_t>(static_cast<unsigned>(Gyro) << 3U);   // FS_SEL
    static constexpr std::uint8_t AccelConfigByte
      = static_cast<std::uint8_t>(static_cast<unsigned>(Accel) << 3U);   // AFS_SEL

    /// WHO_AM_I first, and checked before anything is written (Step::identify): 0x68 is also
    /// the address of the DS1307, DS3231 and PCF8523 clocks. Linux reads it first too.
    static constexpr std::array Init{
      Step::write({.reg = 0x6B, .payload = {0x80}, .delay = std::chrono::milliseconds{100}}),
      Step::write({.reg = 0x6B, .payload = {0x01}}),
      Step::write({.reg = 0x19, .payload = {9}}),
      Step::write({.reg = 0x1A, .payload = {3}}),
      Step::write({.reg = 0x1B, .payload = {GyroConfigByte}}),
      Step::write({.reg = 0x1C, .payload = {AccelConfigByte}}),
      Step::write({.reg = 0x38, .payload = {0x01}}),   // INT_ENABLE: DATA_RDY_EN
    };

    using State = Mpu6050Detail::State;

    /// The rate one count stands for at the range the bring-up sets.
    static constexpr MicroDegPerSec RatePerCount
      = Mpu6050Detail::RatePerCount[static_cast<std::size_t>(Gyro)];

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.accelRange = static_cast<std::uint8_t>(Accel);
        state.gyroRange  = static_cast<std::uint8_t>(Gyro);
        state.deviceId   = static_cast<std::uint16_t>(ids[0]);
    }

    struct Motion {
        static constexpr auto Period = std::chrono::milliseconds{20};
        /// So the frames that said nothing new are counted (unchanged<Motion>()).
        static constexpr bool Timestamped = true;

        static constexpr std::array Steps{
          Step::read({.reg = 0x3A, .count = 15})};   // INT_STATUS, then 0x3B..0x48

        struct Sample {
            std::array<MicroG, 3>         accel{};         ///< 16384 LSB/g at +-2 g
            std::array<MilliDegPerSec, 3> gyro{};          ///< 131 LSB/(deg/s) at 250 deg/s
            CentiDegC                     temperature{};   ///< degC = raw / 340 + 36.53
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            bool allOnes = true;
            for(std::size_t i = 0; i < 15; ++i) { allOnes = allOnes && data.u8(i) == 0xFF; }
            if(allOnes) { return Outcome<Sample>::reject(); }
            if((data.u8(0) & 0x01U) == 0) { return Outcome<Sample>::unchanged(); }   // DATA_RDY_INT
            Sample     sample{};
            auto const perRate = Units::value(Mpu6050Detail::RatePerCount[state.gyroRange & 0x03U]);
            for(std::size_t i = 0; i < 3; ++i) {
                // 1e6 / 16384 = 15625 / 256 micro-g at +-2 g, doubling with each range step.
                auto const accel = static_cast<std::int64_t>(data.s16be(1 + 2 * i)) * 15625
                                 * (std::int64_t{1} << (state.accelRange & 0x03U)) / 256;
                auto const gyro = static_cast<std::int64_t>(data.s16be(9 + 2 * i)) * perRate / 1000;
                sample.accel[i] = Units::microG(accel);
                sample.gyro[i]  = Units::milliDegPerSec(gyro);
            }
            sample.temperature
              = Units::centiDegC(static_cast<std::int32_t>(data.s16be(7)) * 100 / 340 + 3653);
            return Outcome<Sample>::ok(sample);
        }
    };

    /// ACCEL_CONFIG: AFS_SEL in 4:3 (the self-test bits above it), changeable at run time.
    struct AccelConfig {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = AccelConfigByte;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x1C, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.accelRange = static_cast<std::uint8_t>((value >> 3U) & 0x03U);
        }
    };

    /// GYRO_CONFIG: FS_SEL in 4:3.
    struct GyroConfig {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = GyroConfigByte;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x1B, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gyroRange = static_cast<std::uint8_t>((value >> 3U) & 0x03U);
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<AccelConfig, GyroConfig>;
};

/// The part at the default ranges: +-2 g and 250 dps.
using Mpu6050 = Mpu6050x<>;

}   // namespace Kvasir::I2C::Chips
