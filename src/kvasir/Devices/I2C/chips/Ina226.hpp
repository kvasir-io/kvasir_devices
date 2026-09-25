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

/// Texas Instruments INA226 (SBOS547C). One-byte pointer, 16-bit big-endian registers.
/// Bring-up: manufacturer 0xFE = 0x5449, die 0xFF = 0x226x; configuration 0x00 = 0x4127
/// (the reset default: 1 average, 1.1 ms conversions, shunt and bus continuous);
/// calibration 0x05 = 0.00512 / (Current_LSB * R_shunt) (6.5.1: 2560 for 2 mOhm and 1
/// mA). Shunt 0x01 at 2.5 uV/LSB, bus 0x02 at 1.25 mV/LSB, current 0x04 in Current_LSB,
/// power 0x03 in 25 x Current_LSB (Table 6-1), every `Timing::Period` (100 ms). The shunt is in
/// microohms as on the rest of the family; a `MilliOhm` argument converts. 0x40..0x4F by
/// A1/A0.
template<MicroOhm Shunt      = Units::microOhm(2000),
         MicroAmp CurrentLsb = Units::microAmp(1000),
         typename Timing     = DefaultTiming>
struct Ina226 {
    static constexpr std::string_view Name = "INA226";
    /// TI INA226. INA226.md:874..883: Manufacturer ID (FEh) 5449h, Die ID (FFh) 2260h or 2261h.
    static constexpr std::array Identity{
      RegisterCheck{"manufacturer-id", 0xFE, 2, true, 0xFFFF, 0x5449},
      RegisterCheck{         "die-id", 0xFF, 2, true, 0xFFFE, 0x2260},
    };
    static constexpr Address7    Address       = 0x40;
    static constexpr std::size_t RegisterBytes = 1;

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

    static_assert(Shunt > Units::microOhm(0) && CurrentLsb > Units::microAmp(0),
                  "the shunt value and the current LSB are what the calibration is computed "
                  "from; neither may be zero");
    static constexpr std::chrono::milliseconds ReadPeriod = [] {
        if constexpr(requires { Timing::Period; }) {
            return Kvasir::asDuration(Timing::Period);
        } else {
            return std::chrono::milliseconds{100};
        }
    }();
    static_assert(ReadPeriod > std::chrono::milliseconds::zero(),
                  "Timing::Period is how often Power is read; 0 never reads it");

    /// 0.00512 / (Current_LSB[A] * R[Ohm]) = 5.12e9 / (LSB[uA] * R[uOhm]).
    static constexpr std::uint64_t CalibrationValue
      = 5'120'000'000ULL
      / (static_cast<std::uint64_t>(Units::value(CurrentLsb)) * Units::value(Shunt));

    static_assert(CalibrationValue >= 1 && CalibrationValue <= 0x7FFF,
                  "the INA226 calibration register is 15 bits: 0.00512 / (CurrentLsb * Shunt) "
                  "must fit -- raise CurrentLsb (or the shunt) until it does");

    static constexpr std::uint16_t Calibration = static_cast<std::uint16_t>(CalibrationValue);

    static_assert(CurrentLsb <= Units::microAmp(65535),
                  "a full-scale current count times CurrentLsb is carried in 32 bits");

    static constexpr std::array Init{
      Step::write({.reg = 0x00, .payload = {0x41, 0x27}}
      ),
    };

    /// Calibration (05h). It resets to 0 -- current and power then read 0 -- when the part
    /// browns out or sees a general-call reset while the MCU keeps running, so it is read back
    /// every second and written again when it has been lost, as Linux ina2xx.c restores it.
    struct ShuntCal : Groups::Word<0x05, Calibration> {
        static constexpr std::chrono::milliseconds VerifyDelay{3};
        static constexpr std::chrono::milliseconds VerifyInterval{1000};
    };

    struct State {
        std::uint16_t manufacturer{};
        std::uint16_t deviceId{};   ///< Die ID (0xFF)
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.manufacturer = static_cast<std::uint16_t>(ids[0]);
        state.deviceId     = static_cast<std::uint16_t>(ids[1]);
    }

    struct Power {
        static constexpr auto       Period = ReadPeriod;
        static constexpr std::array Steps{Step::read({.reg = 0x01, .count = 2, .offset = 0}),
                                          Step::read({.reg = 0x02, .count = 2, .offset = 2}),
                                          Step::read({.reg = 0x03, .count = 2, .offset = 4}),
                                          Step::read({.reg = 0x04, .count = 2, .offset = 6})};

        struct Sample {
            MicroVolt shuntVoltage{};
            MilliVolt busVoltage{};
            MicroAmp  current{};
            MilliWatt power{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.shuntVoltage
              = Units::microVolt(static_cast<std::int32_t>(data.s16be(0)) * 5 / 2);
            sample.busVoltage
              = Units::milliVolt(static_cast<std::uint32_t>(data.be16(2)) * 5U / 4U);
            sample.power   = Units::milliWatt(static_cast<std::int64_t>(data.be16(4)) * 25
                                              * Units::value(CurrentLsb) / 1000);
            sample.current = Units::microAmp(static_cast<std::int64_t>(data.s16be(6))
                                             * Units::value(CurrentLsb));
            return sample;
        }
    };

    using Reads  = List<Power>;
    using Writes = List<ShuntCal>;
};

}   // namespace Kvasir::I2C::Chips
