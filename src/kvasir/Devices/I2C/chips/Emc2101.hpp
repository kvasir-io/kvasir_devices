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

/// Microchip EMC2101 fan controller with an internal and an external (diode) temperature
/// channel. One-byte pointer, 8-bit registers: internal temperature 0x00, external high
/// byte 0x01 and its fractional low byte 0x10 (bits 7:5, 0.125 degC each), status 0x02,
/// configuration 0x03 (DIS_TO and TACH set: 0x0C), conversion rate 0x04, tachometer 0x46
/// low and 0x47 high, fan configuration 0x4A, fan setting 0x4C (6 bits), PWM frequency
/// 0x4D and its divider 0x4E, averaging filter 0xBF, product id 0xFD (0x16 EMC2101, 0x28
/// EMC2101-R) and manufacturer id 0xFE (0x5D). Bring-up reads both ids and rejects
/// anything else.
///
/// Reading 0x46 latches 0x47, so the tachometer low byte must be read first; as a script
/// that is simply the step order. Every configuration register is read back 100 ms after it
/// is written and again every 10 seconds: a fan controller that has quietly lost its fan
/// setting is a thermal problem.
struct Emc2101 {
    static constexpr std::string_view Name = "EMC2101";
    /// SMSC EMC2101. EMC2101.md:2179..2180: the product ID (FDh) is 16h, or 28h for the EMC2101-R;
    /// :2192: the SMSC ID (FEh) is 5Dh.
    static constexpr std::array Identity{
      RegisterCheck{"product-id", 0xFD, 1, true, 0xFF, 0x16, 0x28},
      RegisterCheck{"smsc-id", 0xFE, 1, true, 0xFF, 0x5D},
    };
    static constexpr Address7                Address = 0x4C;
    static constexpr std::array<Address7, 1> Addresses{0x4C};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

    struct State {
        std::uint8_t deviceId{};       ///< Product ID (0xFD): 0x16, or 0x28 on the -R
        std::uint8_t manufacturer{};   ///< Manufacturer ID (0xFE): 0x5D
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId     = static_cast<std::uint8_t>(ids[0]);
        state.manufacturer = static_cast<std::uint8_t>(ids[1]);
    }

    struct Sensors {
        static constexpr auto Period = std::chrono::milliseconds{500};

        /// 0x46 before 0x47: reading the low byte latches the high one.
        static constexpr std::array Steps{
          Step::read({.reg = 0x00, .count = 1, .offset = 0}),   // internal temperature
          Step::read({.reg = 0x01, .count = 1, .offset = 1}),   // external, whole degrees
          Step::read({.reg = 0x10, .count = 1, .offset = 2}),   // external, fraction in bits 7:5
          Step::read({.reg = 0x46, .count = 1, .offset = 3}),   // tachometer low
          Step::read({.reg = 0x47, .count = 1, .offset = 4}),   // tachometer high
          Step::read({.reg = 0x02, .count = 1, .offset = 5}),   // status
        };

        /// Table 5.2: an open DP-DN, or DP shorted to VDD, reads exactly +127.000 degC; a
        /// DP-DN short, or DN to ground, exactly +127.875 degC.
        static constexpr MilliDegC DiodeOpen  = Units::milliDegC(127000);
        static constexpr MilliDegC DiodeShort = Units::milliDegC(127875);

        struct Sample {
            MilliDegC     temperature{};           ///< the internal (die) channel, 0.001 degC
            MilliDegC     externalTemperature{};   ///< the diode channel, in eighths
            std::uint16_t tach{};
            std::uint8_t  status{};

            [[nodiscard]] constexpr bool busy() const { return (status & 0x80U) != 0; }

            [[nodiscard]] constexpr bool internalTempHigh() const { return (status & 0x40U) != 0; }

            [[nodiscard]] constexpr bool externalTempHigh() const { return (status & 0x10U) != 0; }

            [[nodiscard]] constexpr bool externalTempLow() const { return (status & 0x08U) != 0; }

            [[nodiscard]] constexpr bool externalDiodeFault() const {
                return (status & 0x04U) != 0;
            }

            [[nodiscard]] constexpr bool externalDiodeOpen() const {
                return externalTemperature == DiodeOpen;
            }

            [[nodiscard]] constexpr bool externalDiodeShort() const {
                return externalTemperature == DiodeShort;
            }

            [[nodiscard]] constexpr bool externalValid() const {
                return !externalDiodeOpen() && !externalDiodeShort();
            }

            /// RPM = 5'400'000 / TACH (datasheet 6.14); 0 when the tachometer reads nothing
            /// (stopped, or no fan).
            [[nodiscard]] constexpr Rpm fanSpeed() const {
                if(tach == 0 || tach == 0xFFFF) { return Units::rpm(0); }
                return Units::rpm(5'400'000U / tach);
            }
        };

        /// The external channel is one 11-bit two's complement number in eighths of a
        /// degree: the signed high byte times eight plus the three fraction bits, for both
        /// signs (-1.875 degC is 0xFE, 0x20: -2 * 8 + 1 = -15 eighths).
        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.temperature = Units::milliDegC(static_cast<std::int32_t>(data.s8(0)) * 1000);
            auto const eighths = static_cast<std::int32_t>(data.s8(1)) * 8
                               + static_cast<std::int32_t>(data.u8(2) >> 5);
            sample.externalTemperature = Units::milliDegC(eighths * 125);
            sample.tach                = data.le16(3);
            sample.status              = data.u8(5);
            return sample;
        }
    };

    /// Read back 100 ms after a write, then every ten seconds.
    struct ByteTiming {
        static constexpr std::chrono::milliseconds VerifyDelay{100};
        static constexpr std::chrono::milliseconds VerifyInterval{10000};
    };

    /// One configuration register, put back after every bring-up and checked afterwards.
    template<std::uint16_t Reg, std::uint8_t Init>
    using Byte = Groups::VerifiedByte<Reg, Init, ByteTiming>;

    /// MASK, DIS_TO and TACH set. MASK is set because reading the status register sets it
    /// whenever an alarm bit is up (6.4), so a Config read back without it would mismatch and
    /// be rewritten every ten seconds for as long as the alarm lasts; with pin 6 as TACH there
    /// is no ALERT output for it to mask anyway.
    using Config         = Byte<0x03, 0x8C>;
    using ConversionRate = Byte<0x04, 0x08>;
    using FanConfig      = Byte<0x4A, 0x20>;
    using FanSetting     = Byte<0x4C, 0x00>;   ///< 6 bits, 0..63
    using PwmFrequency   = Byte<0x4D, 0x17>;
    using PwmDivide      = Byte<0x4E, 0x01>;
    using Filter         = Byte<0xBF, 0x06>;   ///< level 2 averaging

    static constexpr std::uint8_t FanSettingMax = 63;

    /// set<FanSetting>(Emc2101::fanSetting(Units::percent(40))).
    [[nodiscard]] static constexpr std::uint8_t fanSetting(Percent duty) {
        auto const p = Units::value(duty) > 100U ? 100U : Units::value(duty);
        return static_cast<std::uint8_t>(p * FanSettingMax / 100U);
    }

    using Reads = List<Sensors>;
    using Writes
      = List<Config, ConversionRate, FanConfig, FanSetting, PwmFrequency, PwmDivide, Filter>;
};

}   // namespace Kvasir::I2C::Chips
