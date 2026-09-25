#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Texas Instruments INA219 (SBOS448G). Six 16-bit big-endian registers behind a
/// one-byte pointer. Configuration (00h) = 399Fh: 32 V, PGA /8 (320 mV), 12 bit, shunt and
/// bus continuous. Calibration (05h) = trunc(0.04096 / (Current_LSB * R_shunt)) (8.5.1
/// eq. 1): 4096 for 100 mOhm and 100 uA/LSB. Then Shunt (01h, 10 uV/LSB signed), Bus (02h,
/// bits 15..3 at 4 mV, OVF bit 0), Power (03h, 20 x Current_LSB), Current (04h, signed)
/// every `Timing::Period` (100 ms; the 12-bit conversions take 532 us each). The shunt is in
/// microohms as on the rest of the family; a `MilliOhm` argument converts. 0x40..0x4F by
/// A1/A0.
///
/// Bit 0 of the calibration register (FS0) is void and reads 0 (Figure 27), so the value
/// written is the equation's rounded down to even. The calibration register resets to 0 --
/// current and power then read 0 -- when the part browns out or sees a general-call reset
/// while the MCU keeps running, so it is a verified write group (`ShuntCal`), read back once a
/// second and written again when it has been lost, as Linux ina2xx.c restores it.
template<MicroOhm Shunt      = Units::microOhm(100000),
         MicroAmp CurrentLsb = Units::microAmp(100),
         typename Timing     = DefaultTiming>
struct Ina219 {
    static constexpr std::string_view Name = "INA219";
    /// TI INA219. Configuration (00h): RST clear and MODE 2:0 = 111b, "shunt and bus, continuous"
    /// (INA219.md:474, :925). The part has no identity register.
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"continuous", 0x00, 2, true, 0x8007, 0x0007},
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
    static_assert(CurrentLsb <= Units::microAmp(65535),
                  "a full-scale current count times CurrentLsb is carried in 32 bits");
    static constexpr std::chrono::milliseconds ReadPeriod = [] {
        if constexpr(requires { Timing::Period; }) {
            return Kvasir::asDuration(Timing::Period);
        } else {
            return std::chrono::milliseconds{100};
        }
    }();
    static_assert(ReadPeriod > std::chrono::milliseconds::zero(),
                  "Timing::Period is how often Power is read; 0 never reads it");

    /// 0.04096 / (Current_LSB[A] * R[Ohm]) = 4.096e10 / (LSB[uA] * R[uOhm]).
    static constexpr std::uint64_t CalibrationValue
      = 40'960'000'000ULL
      / (static_cast<std::uint64_t>(Units::value(CurrentLsb)) * Units::value(Shunt));

    static_assert(CalibrationValue >= 1 && CalibrationValue <= 0xFFFF,
                  "the INA219 calibration register is 16 bits: 0.04096 / (CurrentLsb * Shunt) "
                  "must fit -- raise CurrentLsb (or the shunt) until it does");

    static constexpr std::uint16_t Calibration
      = static_cast<std::uint16_t>(CalibrationValue & 0xFFFEU);   // FS0 is void

    static constexpr std::array Init{
      Step::write({.reg = 0x00, .payload = {0x39, 0x9F}}
      ),
    };

    /// Calibration (05h), read back every second and written again when it has been lost.
    struct ShuntCal : Groups::Word<0x05, Calibration> {
        static constexpr std::chrono::milliseconds VerifyDelay{2};
        static constexpr std::chrono::milliseconds VerifyInterval{1000};
    };

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
            bool      overflow{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample     sample{};
            auto const bus      = data.be16(2);
            sample.shuntVoltage = Units::microVolt(static_cast<std::int32_t>(data.s16be(0)) * 10);
            sample.busVoltage   = Units::milliVolt(static_cast<std::uint32_t>(bus >> 3) * 4U);
            sample.overflow     = (bus & 0x01U) != 0;
            sample.power        = Units::milliWatt(static_cast<std::int64_t>(data.be16(4)) * 20
                                                   * Units::value(CurrentLsb) / 1000);
            sample.current      = Units::microAmp(static_cast<std::int64_t>(data.s16be(6))
                                                  * Units::value(CurrentLsb));
            return sample;
        }
    };

    using Reads  = List<Power>;
    using Writes = List<ShuntCal>;
};

}   // namespace Kvasir::I2C::Chips
