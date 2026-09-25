#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Aled7709Detail {
    /// The two order codes differ in their hardwired address and in what they power up as: the
    /// ALED7709A (28h) waits in standby for the I2C master, the ALED7709B (29h) starts by itself
    /// from the EN pin with its POR configuration (8.2, Tables 16 and 17).
    enum class Variant : std::uint8_t { a, b };

    /// DIMCFG GLDM: one PWM and one gain for all four channels (PWM1, GAIN1), or one of each per
    /// channel.
    enum class Dimming : std::uint8_t { global = 0, local = 1 };

    /// DIMCFG LECC: how the PWM register maps to the duty cycle.
    enum class Curve : std::uint8_t { linear = 0, exponential = 1 };

    /// DIMCFG FDIM[2:0]: the PWM dimming frequency, CLKINT / 2^(16 - n). The resolution of the
    /// duty cycle falls as the frequency rises: 16 bits at 100 Hz, 9 at 12.8 kHz.
    enum class DimFrequency : std::uint8_t {
        hz100   = 0,
        hz200   = 1,
        hz400   = 2,
        hz800   = 3,
        hz1600  = 4,
        hz3200  = 5,
        hz6400  = 6,
        hz12800 = 7
    };

    /// One channel's level: the PWM duty cycle, 0..65535 of the dimming period, and the analog
    /// gain, 0..255 of the current RISET sets (IOUT = IOUT,max * GAIN / 255, 8.3.4).
    struct Level {
        std::uint16_t duty{};
        std::uint8_t  gain{};

        friend constexpr bool operator==(Level const&,
                                         Level const&) = default;
    };
}   // namespace Aled7709Detail

/// STMicroelectronics ALED7709A / ALED7709B (DS14214 rev 5): four 200 mA constant-current sinks
/// with a boost / SEPIC controller in front of them that regulates the LED supply to the headroom
/// the strings need.
///
/// One-byte register pointer, auto-increment over sequential registers, and a write takes effect at
/// the STOP -- so the three bytes of one channel (PWMxH, PWMxL, GAINx: 02h + 3 * (x - 1)) are
/// written as one transaction and the outputs never see half a value (8.3.3).
///
///   00h DEVID      VERS_ID 7:4, REV_ID 3:0
///   01h DEVEN      CLRF (7, write only: clears the latched faults), DEN (0: operation mode)
///   02h..0Dh       PWM1H PWM1L GAIN1 ... PWM4H PWM4L GAIN4
///   11h CHCFG      CENx 3:0 (channel on), CHxCHy 6:4 (two channels tied together by hardware)
///   12h OUTCFG, 13h BOOSTCFG, 15h FMCFG, 16h FMASK: left at their POR values here
///   14h DIMCFG     REG_PWMI (7: registers, not the PWMI pin), PWMI_DRCT (6), FDIM 5:3, LECC (2),
///                  UMDM (1: mixed dimming), GLDM (0: local)
///   17h DEVSTA     IOCP IOVP CHCK OTEA LEDF OOVP THSD
///   18h CHSTA      SH4..SH1 (7:4, LED short), OP4..OP1 (3:0, open string)
///   19h INITSTA    RISET (7), CHx-CHy (6:4), CHxGND (3:0): what the check before start-up found
///
/// The bring-up configures the part in standby and sets DEN last, with CLRF in the same write, so
/// an ALED7709A comes out of standby configured and a warm part drops the faults a previous run
/// latched. The registers are volatile and are kept only while EN is high (7.6); the part does
/// not acknowledge at all with EN low or VIN under its UVLO (8.2.4), which is how it looks absent.
///
/// The output current is RISET's: ICH_SET = 1022 V / RISET, 25 mA (40.7 k) to 200 mA (5.11 k)
/// (Table 5). `Riset` is the board's resistor, and only scales `gainFor()`.
///
/// Not written from a part: no ALED7709 has been on the bench yet (2026-09-18). The i2c_testing
/// hardware test holds the description against its `Identity` and `AfterBringUp` the day one is.
template<Aled7709Detail::Variant      V           = Aled7709Detail::Variant::a,
         Ohm                          Riset       = Units::ohm(10'000),
         Aled7709Detail::Dimming      Dim         = Aled7709Detail::Dimming::local,
         Aled7709Detail::DimFrequency Frequency   = Aled7709Detail::DimFrequency::hz200,
         Aled7709Detail::Curve        Conversion  = Aled7709Detail::Curve::linear,
         std::uint8_t                 ChannelMask = 0x0F>
struct Aled7709 {
    static constexpr std::string_view Name = "ALED7709";

    /// DEVID (00h): VERS_ID, bits 7:4, is 1h, "cut 1.x (hardwired)"; REV_ID below it is 3h on the
    /// parts of DS14214 rev 5 and is left open (ALED7709.md:1745..1766).
    static constexpr std::array Identity{
      RegisterCheck{"devid", 0x00, 1, true, 0xF0, 0x10},
    };

    static constexpr std::uint8_t DimCfg = static_cast<std::uint8_t>(
      0x80U | (static_cast<unsigned>(Frequency) << 3U) | (static_cast<unsigned>(Conversion) << 2U)
      | static_cast<unsigned>(Dim));

    /// What a finished bring-up leaves in the part: operation mode, the channels of `ChannelMask`
    /// on and none tied together, the registers (not the PWMI pin) in control, user dimming at the
    /// frequency, curve and global / local mode of the template arguments.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"operating", 0x01, 1, true, 0x01,        0x01},
      RegisterCheck{ "channels", 0x11, 1, true, 0x7F, ChannelMask},
      RegisterCheck{  "dimming", 0x14, 1, true, 0xFF,      DimCfg},
    };

    static constexpr Address7 Address
      = V == Aled7709Detail::Variant::a ? Address7{0x28} : Address7{0x29};
    static constexpr std::array<Address7, 2> Addresses{0x28, 0x29};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::size_t Channels = 4;

    static_assert((ChannelMask & 0xF0U) == 0,
                  "four channels: CEN1..CEN4 are bits 0..3");

    /// The current a gain of 255 gives: ICH_SET = 1022 V / RISET (Table 5).
    static constexpr MicroAmp MaxCurrent
      = Units::microAmp(static_cast<std::int32_t>(1'022'000'000LL / Units::value(Riset)));

    static_assert(Units::value(MaxCurrent) >= 25'000 && Units::value(MaxCurrent) <= 200'000,
                  "RISET sets 25 mA (40.7 k) to 200 mA (5.11 k)");

    /// The gain at or below a wanted current: IOUT = IOUT,max * GAIN / 255 (8.3.4). The data
    /// sheet's accuracy figures hold from 25 mA up; below that the part still follows the code.
    [[nodiscard]] static constexpr std::uint8_t gainFor(MicroAmp current) {
        auto const wanted = Units::value(current);
        auto const max    = Units::value(MaxCurrent);
        if(wanted <= 0) { return 0; }
        if(wanted >= max) { return 255; }
        return static_cast<std::uint8_t>(static_cast<std::int64_t>(wanted) * 255 / max);
    }

    /// Dimming and channels while the part is in standby, then DEN with CLRF: operation mode, and
    /// whatever an earlier run latched is gone. 2 ms for the start-up the part runs from there
    /// before its status means anything is not in the data sheet as a number; the first Status
    /// read a period later is what sees it.
    static constexpr std::array Init{
      Step::write({.reg = 0x01, .payload = {0x00}}),   // standby: configuration first
      Step::write({.reg = 0x14, .payload = {DimCfg}}),
      Step::write({.reg = 0x11, .payload = {ChannelMask}}),
      Step::write({.reg = 0x01, .payload = {0x81}, .delay = std::chrono::milliseconds{2}}),
    };

    /// DEVSTA, CHSTA, INITSTA in one read.
    struct Status {
        static constexpr auto       Period = std::chrono::milliseconds{100};
        static constexpr std::array Steps{Step::read({.reg = 0x17, .count = 3, .offset = 0})};

        struct Sample {
            std::uint8_t device{};    ///< DEVSTA
            std::uint8_t channel{};   ///< CHSTA
            std::uint8_t initial{};   ///< INITSTA

            [[nodiscard]] constexpr bool inputOverCurrent() const { return (device & 0x80U) != 0; }

            [[nodiscard]] constexpr bool inputOverVoltage() const { return (device & 0x40U) != 0; }

            /// The check before start-up found something: `initial` says what.
            [[nodiscard]] constexpr bool initialCheckFailed() const {
                return (device & 0x20U) != 0;
            }

            /// The early warning, before the thermal shutdown.
            [[nodiscard]] constexpr bool overTemperatureAlert() const {
                return (device & 0x10U) != 0;
            }

            [[nodiscard]] constexpr bool ledFault() const { return (device & 0x08U) != 0; }

            [[nodiscard]] constexpr bool outputOverVoltage() const { return (device & 0x04U) != 0; }

            [[nodiscard]] constexpr bool thermalShutdown() const { return (device & 0x02U) != 0; }

            /// Channel 0..3: its string is open, or shorted.
            [[nodiscard]] constexpr bool open(std::size_t ch) const {
                return (channel & (1U << ch)) != 0;
            }

            [[nodiscard]] constexpr bool shorted(std::size_t ch) const {
                return (channel & (0x10U << ch)) != 0;
            }

            /// Channel 0..3 was found shorted to ground before start-up.
            [[nodiscard]] constexpr bool shortedToGround(std::size_t ch) const {
                return (initial & (1U << ch)) != 0;
            }

            [[nodiscard]] constexpr bool risetFault() const { return (initial & 0x80U) != 0; }

            [[nodiscard]] constexpr bool any() const {
                return device != 0 || channel != 0 || initial != 0;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {data.u8(0), data.u8(1), data.u8(2)};
        }
    };

    /// A channel's duty cycle and gain, as one transaction so that both change on the same STOP.
    /// Item 0..3 is channel 1..4; in global dimming only item 0 is what the outputs follow. No
    /// Initial: the ALED7709A powers up dark (all 00h), and what the LEDs show is the
    /// application's -- but a level it has set is put back after a bring-up, like any write group.
    struct Level {
        using Value                        = Aled7709Detail::Level;
        static constexpr std::size_t Items = Channels;
        static constexpr std::size_t Bytes = 3;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value.duty >> 8U);
            buffer[1] = static_cast<std::byte>(value.duty & 0xFFU);
            buffer[2] = static_cast<std::byte>(value.gain);
            return Step::writeBuffer(
              {.reg = static_cast<std::uint16_t>(0x02 + 3 * item), .offset = 0, .count = 3});
        }
    };

    /// CLRF with DEN: the latched faults cleared, and the part back in (or kept in) operation mode
    /// -- after a channel shorted to ground was found before start-up this is the only way on
    /// (7.4). A command, not a state.
    struct ClearFaults {
        struct Value {};

        static constexpr std::size_t Bytes     = 1;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr Step encode(Value const&,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{0x81};
            return Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 1});
        }
    };

    using Reads  = List<Status>;
    using Writes = List<Level, ClearFaults>;
};

}   // namespace Kvasir::I2C::Chips
