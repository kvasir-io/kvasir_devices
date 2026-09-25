#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// X-Powers AXP2101 power management unit, from the AXP2101 datasheet V1.0. Two registers
/// where the vendor's Arduino driver disagrees with the datasheet are noted below.
///
/// Fixed address 0x34, one-byte pointer. IC type 0x03 reads 0x4A (the vendor driver's
/// register and value: the datasheet's register list skips 0x02..0x03). This description covers the
/// *monitoring* half only -- what a battery display needs -- and deliberately not the rails:
/// writing a DCDC or LDO voltage on a board whose regulators are already configured for its
/// panel and PSRAM is a good way to brown something out, and nothing here needs to.
struct Axp2101 {
    static constexpr std::string_view        Name    = "AXP2101";
    static constexpr Address7                Address = 0x34;
    static constexpr std::array<Address7, 1> Addresses{0x34};
    static constexpr std::size_t             RegisterBytes = 1;

    /// Read the IC type, then turn on the ADC channels the readings below come from:
    /// ADC_CHANNEL_CTRL 0x30 bit 0 battery voltage, bit 2 VBUS, bit 3 system, bit 4 die
    /// temperature; BAT_DET_CTRL 0x68 bit 0 is the battery-present detector. The vendor sets
    /// each of these bits read-modify-write; the whole byte is written here. Bit 1, the TS pin
    /// channel, is kept at its reset value 1 (6.13.2.29): the charger's thermistor protection
    /// measures through it. Bit 5, the general-purpose ADC, stays at its reset value 0.
    static constexpr std::array Init{
      Step::read({.reg = 0x03, .count = 1, .offset = 0}),
      Step::identify(),
      Step::write({.reg = 0x30, .payload = {0x1F}}),
      Step::write({.reg = 0x68, .payload = {0x01}, .delay = std::chrono::milliseconds{10}}),
    };

    struct State {
        std::uint8_t deviceId{};   ///< IC type (0x03)
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.deviceId = data.u8(0);
        return state.deviceId == 0x4A;
    }

    /// What the charge state machine is doing (STATUS2 bits 2..0).
    enum class Charge : std::uint8_t {
        trickle     = 0,
        preCharge   = 1,
        constantI   = 2,
        constantV   = 3,
        done        = 4,
        notCharging = 5
    };

    struct Power {
        static constexpr auto Period = std::chrono::milliseconds{500};

        /// STATUS1 and STATUS2 at 0x00, then the ADC results at 0x34. Two reads rather than
        /// one because 0x02..0x33 in between is the interrupt and rail configuration, which
        /// a status poll has no reason to touch.
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2, .offset = 0}),
                                          Step::read({.reg = 0x34, .count = 10, .offset = 2})};

        struct Sample {
            MilliVolt    battery{};
            MilliVolt    vbus{};
            MilliVolt    system{};
            CentiDegC    dieTemperature{};
            std::uint8_t status1{};
            std::uint8_t status2{};

            /// STATUS1 bit 3: a battery is actually connected.
            [[nodiscard]] constexpr bool batteryPresent() const { return (status1 & 0x08U) != 0; }

            /// STATUS1 bit 5: VBUS good. The vendor driver also requires STATUS2 bit 3 to be
            /// clear, but the datasheet calls that bit VINDPM status -- the input is being
            /// current-limited, which is a thing that happens *while* VBUS is present -- so
            /// it is reported separately rather than folded in here.
            [[nodiscard]] constexpr bool vbusPresent() const { return (status1 & 0x20U) != 0; }

            /// STATUS2 bit 3: the input is in dynamic power management, i.e. being limited.
            [[nodiscard]] constexpr bool inVindpm() const { return (status2 & 0x08U) != 0; }

            /// STATUS2 bits 6..5, the battery current direction: 00 standby, 01 charge,
            /// 10 discharge.
            [[nodiscard]] constexpr bool charging() const { return ((status2 >> 5U) & 0x03U) == 1; }

            [[nodiscard]] constexpr bool discharging() const {
                return ((status2 >> 5U) & 0x03U) == 2;
            }

            [[nodiscard]] constexpr Charge charge() const {
                return static_cast<Charge>(status2 & 0x07U);
            }
        };

        /// A 14-bit result across two registers, high byte first. Every channel's high
        /// register keeps the value in bits 5..0 (REG34 vbat[13:8], REG38 vbus[13:8], REG3A
        /// vsys[13:8], REG3C tdie[13:8]), so the mask is 0x3F throughout -- the vendor's
        /// ReadRegisterH5L8 masks the battery to five bits and drops vbat[13]. 1 mV per count.
        [[nodiscard]] static constexpr std::int32_t adc(Bytes       data,
                                                        std::size_t i) {
            return static_cast<std::int32_t>(data.be16(i) & 0x3FFFU);
        }

        /// The ten ADC bytes land at buffer offset 2, so 0x34 is index 2: battery 0x34/0x35,
        /// 0x36/0x37 the TS pin (not read), VBUS 0x38/0x39, system 0x3A/0x3B, die 0x3C/0x3D.
        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.status1 = data.u8(0);
            sample.status2 = data.u8(1);
            sample.battery = Units::milliVolt(adc(data, 2));
            sample.vbus    = Units::milliVolt(adc(data, 6));
            sample.system  = Units::milliVolt(adc(data, 8));
            // The datasheet gives the die channel's registers but not its transfer function.
            // This is the vendor library's, 22 + (7274 - raw) / 20 degC, at hundredths so the
            // number keeps the resolution the channel has. It is the one figure here that is
            // not from the datasheet.
            sample.dieTemperature = Units::centiDegC(2200 + (7274 - adc(data, 10)) * 100 / 20);
            return sample;
        }
    };

    /// The fuel gauge's percentage, on its own because it is a single byte far from the ADC
    /// block and changes slowly.
    struct Gauge {
        static constexpr auto       Period = std::chrono::milliseconds{2000};
        static constexpr std::array Steps{Step::read({.reg = 0xA4, .count = 1, .offset = 0})};

        using Sample = Percent;

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            auto const p = data.u8(0);
            // A percentage above 100 is not one (the register resets to 00h; the vendor's
            // driver clamps nothing), so it is not reported.
            if(p > 100) { return Outcome<Sample>::reject(); }
            return Outcome<Sample>::ok(Units::percent(p));
        }
    };

    using Reads = List<Power, Gauge>;
};

}   // namespace Kvasir::I2C::Chips
