#pragma once
// InvenSense MPU-9250 on SPI, a chip description for the engine; the ranges are write groups the
// decode follows. Data sheet PS-MPU-9250A-01 (MPU9250.md), register map RM-MPU-9250A-00
// (MPU9250_RegMap.md). The magnetometer behind the auxiliary I2C master is not brought up.
#include "../../I2C/Groups.hpp"
#include "../../Quantities.hpp"
#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir { namespace SPI { namespace Chips {

    namespace Mpu9250Detail {
        enum class AccelRange : std::uint8_t { g2 = 0, g4 = 1, g8 = 2, g16 = 3 };
        enum class GyroRange : std::uint8_t { dps250 = 0, dps500 = 1, dps1000 = 2, dps2000 = 3 };

        /// 1 / 131, 1 / 65.5, 1 / 32.8, 1 / 16.4 deg/s a count, to the micro-degree a second.
        inline constexpr std::array<Units::MicroDegPerSec, 4> RatePerCount{
          Units::microDegPerSec(7634),
          Units::microDegPerSec(15267),
          Units::microDegPerSec(30488),
          Units::microDegPerSec(60976)};

        struct State : I2C::Groups::DeviceId {
            std::uint8_t accelRange{};
            std::uint8_t gyroRange{};
        };
    }   // namespace Mpu9250Detail

    /// Bring-up: H_RESET + 100 ms start-up (data sheet 3.4.2); SIGNAL_PATH_RESET + 100 ms (Linux
    /// inv_mpu_core.c: "required for spi connection"); PLL; I2C_IF_DIS (RegMap:1515) so the I2C
    /// slave cannot answer a glitch; 100 Hz with DLPF 41 Hz, and A_DLPF_CFG = 3 too - the accel
    /// filter is 460 Hz wide out of reset (RegMap 4.7 table 2) and would alias.
    template<Mpu9250Detail::AccelRange Accel = Mpu9250Detail::AccelRange::g2,
             Mpu9250Detail::GyroRange  Gyro  = Mpu9250Detail::GyroRange::dps250>
    struct Mpu9250x {
        static constexpr std::string_view Name = "MPU9250";
        /// 0x71 (RegMap:1665). 0x73 = MPU-9255, not checked against a data sheet.
        static constexpr std::array Identity{
          I2C::RegisterCheck{"who-am-i", 0x75, 1, true, 0xFF, 0x71, 0x73},
        };
        /// 1 MHz for every register (MPU9250.md:124). Modes 0 and 3 both fit (7.5,
        /// MPU9250.md:1217-1219); 0 shares a bus with mode-0 parts without a polarity change.
        static constexpr ClockMode    Mode          = ClockMode::_0;
        static constexpr Units::Hertz MaxClock      = Units::hertz(1'000'000);
        static constexpr std::size_t  RegisterBytes = 1;

        static constexpr std::uint8_t GyroConfigByte
          = static_cast<std::uint8_t>(static_cast<unsigned>(Gyro) << 3U);
        static constexpr std::uint8_t AccelConfigByte
          = static_cast<std::uint8_t>(static_cast<unsigned>(Accel) << 3U);

        static constexpr std::array Init{
          Step::write({.reg = 0x6B, .payload = {0x80}, .delay = std::chrono::milliseconds{100}}),
          Step::write({.reg = 0x68, .payload = {0x07}, .delay = std::chrono::milliseconds{100}}),
          Step::write({.reg = 0x6B, .payload = {0x01}}),
          Step::write({.reg = 0x6A, .payload = {0x10}}),
          Step::write({.reg = 0x19, .payload = {9}}),
          Step::write({.reg = 0x1A, .payload = {3}}),
          Step::write({.reg = 0x1D, .payload = {3}}),
        };

        using State = Mpu9250Detail::State;

        static constexpr void identified(std::span<std::uint32_t const> ids,
                                         State&                         state) {
            state.accelRange = static_cast<std::uint8_t>(Accel);
            state.gyroRange  = static_cast<std::uint8_t>(Gyro);
            state.deviceId   = static_cast<std::uint16_t>(ids[0]);
        }

        struct Motion {
            static constexpr auto       Period = std::chrono::milliseconds{20};
            static constexpr std::array Steps{Step::read({.reg = 0x3B, .count = 14})};

            struct Sample {
                std::array<Units::MicroG, 3>         accel{};
                Units::CentiDegC                     temperature{};
                std::array<Units::MilliDegPerSec, 3> gyro{};
            };

            [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                                  State const& state) {
                bool allOnes = true;
                for(std::size_t i = 0; i < 14; ++i) { allOnes = allOnes && data.u8(i) == 0xFF; }
                if(allOnes) { return Outcome<Sample>::reject(); }
                Sample     sample{};
                auto const perRate
                  = Units::value(Mpu9250Detail::RatePerCount[state.gyroRange & 0x03U]);
                for(std::size_t i = 0; i < 3; ++i) {
                    auto const accel = static_cast<std::int64_t>(data.s16be(2 * i)) * 15625
                                     * (std::int64_t{1} << (state.accelRange & 0x03U)) / 256;
                    auto const gyro
                      = static_cast<std::int64_t>(data.s16be(8 + 2 * i)) * perRate / 1000;
                    sample.accel[i] = Units::microG(accel);
                    sample.gyro[i]  = Units::milliDegPerSec(gyro);
                }
                // TEMP_degC = TEMP_OUT / 333.87 + 21 (RegMap:1210, MPU9250.md:421; offset 0 LSB at 21 degC)
                sample.temperature = Units::centiDegC(static_cast<std::int32_t>(
                  static_cast<std::int64_t>(data.s16be(6)) * 10000 / 33387 + 2100));
                return Outcome<Sample>::ok(sample);
            }
        };

        /// ACCEL_CONFIG (1Ch)
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

        /// GYRO_CONFIG (1Bh)
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

    using Mpu9250 = Mpu9250x<>;

}}}   // namespace Kvasir::SPI::Chips
