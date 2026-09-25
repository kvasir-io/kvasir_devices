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

/// NXP PCA9956B 24-channel constant-current LED driver (data sheet rev 1.2). A one-byte
/// pointer whose top bits are the auto-increment control, so every register address here
/// carries 0x80 (increment through all registers): MODE2 0x81, LEDOUT0..5 0x82 (six bytes,
/// two bits per channel), PWM0..23 0x8A (twenty-four bytes), IREF0..23 0xA2 (twenty-four).
/// PWMALL 3Fh and IREFALL 40h lie past the increment range at MODE1's AI1:AI0 = 00, which
/// is 00h..3Eh (Table 6), so they are addressed without the flag.
///
/// The output current per channel is set by IREF against the external resistor, section
/// 7.3.13.1 "Adjusting output current" and Fig 5:
///
///     I_O(mA) = IREF x (0.9 / 4) / Rext(kOhm)   ->   IREF = I_O(mA) x 4 x Rext(Ohm) / 900
///
/// so 255 at 1 kOhm is 57.4 mA, the part's maximum. `iref()` spells that out, rounded to the
/// nearest code, with Rext a template parameter so the arithmetic happens once:
/// `iref(milliAmp(5))` at 2.2 kOhm is 49.
///
/// MODE2 carries the ERROR flag (bit 6) and OVERTEMP (bit 7), so it is polled once a second.
/// Both are decoded against Table 9: 1 is the fault in each case. Bits 2, 1 and 0 are
/// reserved and *read only* (Table 9: 1, 0, 1 by default), so a healthy part reads 0x05 and
/// one with a latched LED error 0x45, and what a write puts in those bits does not matter --
/// `Mode2::Initial` writes 0x00.
///
/// The address is strapped over a wide range (three quinary, five-level pins giving 125 addresses,
/// Tables 4 and 5), so it is a template parameter with no default: `Pca9956b<0x3F, 1000>`.
/// `Addresses` therefore names only the one this instance is strapped to. Out of reset the part
/// also answers the LED All Call address 0x70 and sub-address 1 at 0x77 (MODE1 0x89, Table 8;
/// ALLCALLADR E0h, SUBADR1 EEh) -- the defaults of a TCA9548A and a BME280 -- so bring-up
/// writes MODE1 = 0x00, as Linux's leds-pca995x does: both off, SLEEP clear, AI1:AI0 = 00.
/// `Timing::StartupDelay` is the wait after power before the first transaction. Section 7.5 asks
/// for 2 ms after power-on reset and at most 1.5 ms after RESET is released; 500 ms is a
/// conservative default, not a requirement of the part.
template<Address7 Addr, Ohm Rext = Units::ohm(1000), typename Timing = DefaultTiming>
struct Pca9956b {
    static constexpr std::string_view Name = "PCA9956B";
    /// NXP PCA9956B. MODE1 (00h) SLEEP, bit 4, clear: normal mode, the oscillator runs
    /// (PCA9956B.md:847; it is also the power-up value, so this only says nothing put the part to
    /// sleep). The part has no identity register.
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"awake", 0x00, 1, true, 0x10, 0x00},
    };
    static constexpr Address7    Address       = Addr;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 1> Addresses{Addr};

    static constexpr std::size_t Channels = 24;

    static_assert(Rext > Units::ohm(0),
                  "Rext is what the output current is set against");

    static constexpr std::chrono::milliseconds StartupDelay = [] {
        if constexpr(requires { Timing::StartupDelay; }) {
            return Kvasir::asDuration(Timing::StartupDelay);
        } else {
            return std::chrono::milliseconds{500};
        }
    }();

    /// The largest current the part can be asked for at this Rext: code 255.
    static constexpr MilliAmp MaxCurrent
      = Units::milliAmp(static_cast<std::int32_t>(255U * 900U / (4U * Units::value(Rext))));

    /// IREF = I_O(mA) x 4 x Rext(Ohm) / 900, rounded to the nearest code and clamped to 255
    /// (section 7.3.13.1). At 1 kOhm the codes are 0.225 mA apart; at 2.2 kOhm, 0.102 mA.
    [[nodiscard]] static constexpr std::uint8_t iref(MilliAmp current) {
        auto const mA = Units::value(current);
        if(mA <= 0) { return 0; }
        auto const v = (static_cast<std::uint32_t>(mA) * 4U * Units::value(Rext) + 450U) / 900U;
        return static_cast<std::uint8_t>(v > 255 ? 255 : v);
    }

    /// MODE2 is the probe as well as the error flag; then MODE1 turns the All Call and
    /// sub-address responses off.
    static constexpr std::array Init{Step::read({.reg = 0x81, .count = 1, .offset = 0}),
                                     Step::write({.reg = 0x80, .payload = {0x00}})};

    struct Status {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{Step::read({.reg = 0x81, .count = 1, .offset = 0})};

        struct Sample {
            std::uint8_t mode2{};

            /// Set when any channel reports an open or shorted LED. It latches: the flag
            /// and the EFLAGn bits behind it stay set until CLRERR is written, so a
            /// channel that was driven before its current was set (LEDOUT's own Initial
            /// puts every channel in PWM mode, and IREF comes up at zero) leaves an error
            /// behind that says nothing about the wiring. Write `Mode2` with `ClrErr` once
            /// the outputs are actually driven; after that the flag means what it says.
            ///
            /// Table 22 and the Remark in 7.3.14: a channel whose LDRx is 00, or whose PWM is under
            /// 8, is not tested at all -- "detection not possible" -- so unused channels do not
            /// raise it either way.
            [[nodiscard]] constexpr bool error() const { return (mode2 & 0x40U) != 0; }

            /// Table 9: OVERTEMP is 1 for an over-temperature condition and 0 for O.K.
            /// The outputs are off while it is set, and it clears itself once the die is
            /// back below the threshold less its hysteresis.
            [[nodiscard]] constexpr bool overTemperature() const { return (mode2 & 0x80U) != 0; }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.u8(0)}; }
    };

    /// Twenty-four bytes written as one transaction: a change to any channel re-sends the
    /// block.
    template<std::uint16_t Reg>
    struct Block {
        using Value                        = std::array<std::uint8_t, Channels>;
        static constexpr std::size_t Bytes = Channels;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            for(std::size_t i = 0; i < Channels; ++i) { buffer[i] = std::byte{value[i]}; }
            return Step::writeBuffer({.reg = Reg, .offset = 0, .count = Channels});
        }
    };

    /// One byte.
    template<std::uint16_t Reg, std::uint8_t Init>
    using Byte = Groups::InitialByte<Reg, Init>;

    /// Two bits per channel: 00 off, 01 fully on, 10 PWM, 11 PWM with group dimming.
    struct LedOut {
        using Value                        = std::array<std::uint8_t, 6>;
        static constexpr std::size_t Bytes = 6;
        /// Every channel in PWM mode, which is what the PWM registers are for.
        static constexpr Value Initial{0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            for(std::size_t i = 0; i < 6; ++i) { buffer[i] = std::byte{value[i]}; }
            return Step::writeBuffer({.reg = 0x82, .offset = 0, .count = 6});
        }
    };

    /// MODE2 bit 4, write only and self-clearing: clears ERROR and every EFLAGn bit.
    /// `set<Mode2>(Pca9956b::ClrErr)`.
    static constexpr std::uint8_t ClrErr = 0x10;

    using Pwm     = Block<0x8A>;
    using Iref    = Block<0xA2>;
    using PwmAll  = Byte<0x3F, 0x00>;
    using IrefAll = Byte<0x40, 0x00>;

    /// MODE2. CLRERR clears itself, so writing the same value again is a new command: every
    /// set goes out. The Initial 0x00 leaves the read-only reserved bits (2:0) to the part.
    struct Mode2 : Byte<0x81, 0x00> {
        static constexpr bool AlwaysWrite = true;
    };

    /// Which outputs the ERROR flag is about. EFLAG0..5 (41h..46h), two bits per channel with
    /// LED0 in the low bits of EFLAG0: 01 is a short-circuit, 10 an open-circuit (Table 21).
    /// One register per transaction, without the Auto-Increment flag: with MODE1 at its
    /// default the pointer only increments through 00h..3Eh (Table 6), and these sit above
    /// it. The bits latch like ERROR does, until CLRERR (Status::error()), and a channel off
    /// or at a PWM under 8 is not tested at all.
    ///
    /// Not read by `Pca9956b` itself -- six transactions a second is a cost a user that only
    /// needs the ERROR flag should not pay -- but by `Pca9956bDiagnostics` below.
    struct Errors {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{Step::read({.reg = 0x41, .count = 1, .offset = 0}),
                                          Step::read({.reg = 0x42, .count = 1, .offset = 1}),
                                          Step::read({.reg = 0x43, .count = 1, .offset = 2}),
                                          Step::read({.reg = 0x44, .count = 1, .offset = 3}),
                                          Step::read({.reg = 0x45, .count = 1, .offset = 4}),
                                          Step::read({.reg = 0x46, .count = 1, .offset = 5})};

        struct Sample {
            std::array<std::uint8_t, 6> eflag{};

            /// The two error bits of one channel: 00 fine, 01 short, 10 open.
            [[nodiscard]] constexpr std::uint8_t code(std::size_t channel) const {
                return static_cast<std::uint8_t>((eflag[channel / 4] >> (2U * (channel % 4)))
                                                 & 0x03U);
            }

            /// Bit n set: LEDn reports a short-circuit.
            [[nodiscard]] constexpr std::uint32_t shorted() const { return mask_(0x01); }

            /// Bit n set: LEDn reports an open-circuit.
            [[nodiscard]] constexpr std::uint32_t open() const { return mask_(0x02); }

        private:
            [[nodiscard]] constexpr std::uint32_t mask_(std::uint8_t which) const {
                std::uint32_t m = 0;
                for(std::size_t c = 0; c < Channels; ++c) {
                    if(code(c) == which) { m |= 1UL << c; }
                }
                return m;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample s{};
            for(std::size_t i = 0; i < s.eflag.size(); ++i) { s.eflag[i] = data.u8(i); }
            return s;
        }
    };

    using Reads  = List<Status>;
    using Writes = List<Mode2, LedOut, IrefAll, PwmAll, Iref, Pwm>;
};

/// The same part with the EFLAGn registers read once a second beside MODE2, for a user that
/// wants to know *which* output the ERROR flag is about. Two read groups, so a Sample is
/// named by its group: `latest<Status>()`, `latest<Errors>()`.
template<Address7 Addr, Ohm Rext = Units::ohm(1000), typename Timing = DefaultTiming>
struct Pca9956bDiagnostics : Pca9956b<Addr, Rext, Timing> {
    using Status = typename Pca9956b<Addr, Rext, Timing>::Status;
    using Errors = typename Pca9956b<Addr, Rext, Timing>::Errors;
    using Reads  = List<Status, Errors>;
};

}   // namespace Kvasir::I2C::Chips
