#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Ktd2061Detail {
    /// Control (02h) bits 7:6.
    enum class Mode : std::uint8_t { globalOff = 0, night = 1, normal = 2, reset = 3 };

    /// Control (02h) bits 2:0, FADE_RATE: the exponential time constant of every current change.
    /// Nearly settled after three of them; to the eye a rise looks done in half of one and a fall
    /// takes four to six, so a breathing pattern wants a slow rate up and a fast one down (data
    /// sheet rev 04e, "Fade Rate"). Fading cannot be turned off: ms31 is the "instant" setting.
    enum class FadeRate : std::uint8_t {
        ms31   = 0,
        ms63   = 1,
        ms125  = 2,
        ms250  = 3,
        ms500  = 4,
        ms1000 = 5,
        ms2000 = 6,
        ms4000 = 7
    };

    /// Control (02h) bits 4:3, CE_TEMP: the die temperature at which the part starts derating.
    enum class CoolExtend : std::uint8_t { c135 = 0, c120 = 1, c105 = 2, c90 = 3 };

    /// The twelve RGB modules as the data sheet names them, by the anode pin each hangs on. In
    /// register order: ISELA12 (09h) holds A1 in bits 7:4 and A2 in bits 3:0, ISELA34 (0Ah) A3
    /// and A4, and so on to ISELC34 (0Eh).
    enum class Module : std::uint8_t { A1, A2, A3, A4, B1, B2, B3, B4, C1, C2, C3, C4 };

    /// One LED's four bits in a select register: bit 3 turns the LED on, and bits 2, 1 and 0 say
    /// which of the two colour slots its red, green and blue each take. The choice is per
    /// channel, not per LED, so two slots give eight colours: with slot 0 white and slot 1 black,
    /// `{.red = 0, .green = 1, .blue = 1}` is a red LED.
    struct Channels {
        bool on{};
        bool red{};   ///< false: slot 0, true: slot 1
        bool green{};
        bool blue{};

        [[nodiscard]] constexpr std::uint8_t bits() const {
            return static_cast<std::uint8_t>((on ? 0x8U : 0U) | (red ? 0x4U : 0U)
                                             | (green ? 0x2U : 0U) | (blue ? 0x1U : 0U));
        }
    };

    /// The LED off, whatever the slots hold.
    inline constexpr Channels Off{};
}   // namespace Ktd2061Detail

/// Kinetic KTD2061 12-channel RGB LED driver: twelve LEDs, each assigned one of two colours.
///
/// One-byte register pointer. 00h ID, 01h Monitor, 02h Control, 03h..08h the two colour slots
/// (R0 G0 B0 R1 G1 B1, one current byte each), 09h..0Eh the six select registers, four bits per
/// LED and two LEDs to a register, the first of the pair in the HIGH nibble: A1 in bits 7:4 of
/// 09h and A2 in its bits 3:0, A3 and A4 in 0Ah and so on (register map, ISELA12..ISELC34). A
/// nibble is `Channels`: enable, then one slot choice each for red, green and blue.
///
/// Twelve LEDs cost eight bytes on the wire whatever they are showing -- six colour bytes and six
/// select bytes, written as two blocks -- because the colour is per slot, not per LED. An
/// animation therefore changes at most two things per chip per step, which is what lets one
/// SERCOM carry a ring of them.
///
/// Checked against the KTD2061/58/59/60 data sheet, rev 04e (March 2022): the register map
/// table, the CONTROL, MONITOR, IRED0..IBLU1 and ISELA12 register descriptions. A current code
/// is 125 uA a step and 192 (C0h) is the 24 mA full scale; larger codes are clamped and read
/// back as C0h. In night mode (`Mode::night`) every current is divided by 16: 1.5 mA at most.
///
/// The address is strapped, so it is a template parameter; 0x68 is what the ring boards use.
template<Address7                  Addr = 0x68,
         Ktd2061Detail::Mode       M    = Ktd2061Detail::Mode::night,
         Ktd2061Detail::FadeRate   FR   = Ktd2061Detail::FadeRate::ms1000,
         Ktd2061Detail::CoolExtend CE   = Ktd2061Detail::CoolExtend::c90>
struct Ktd2061 {
    using Mode       = Ktd2061Detail::Mode;
    using FadeRate   = Ktd2061Detail::FadeRate;
    using CoolExtend = Ktd2061Detail::CoolExtend;
    using Channels   = Ktd2061Detail::Channels;
    using Module     = Ktd2061Detail::Module;

    static constexpr Channels Off = Ktd2061Detail::Off;

    static constexpr std::string_view        Name    = "KTD2061";
    static constexpr Address7                Address = Addr;
    static constexpr std::array<Address7, 1> Addresses{Addr};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::size_t Leds  = 12;
    static constexpr std::size_t Slots = 2;

    /// The largest current code, 24 mA (1.5 mA in night mode); anything above reads back as it.
    static constexpr std::uint8_t FullScale = 192;

    /// What ID (00h) reads: VENDOR 101, DIE_ID 00100.
    static constexpr std::uint8_t ChipId = 0xA4;

    /// Mode 7:6, BrightExtend 5 (off), CoolExtend 4:3, FadeRate 2:0.
    static constexpr std::uint8_t ControlValue = static_cast<std::uint8_t>(
      (static_cast<std::uint8_t>(M) << 6U) | (static_cast<std::uint8_t>(CE) << 3U)
      | static_cast<std::uint8_t>(FR));

    static constexpr auto StartupDelay = std::chrono::milliseconds{500};

    static constexpr std::array Init{
      Step::write({.reg = 0x02, .payload = {ControlValue}}),
    };

    /// ID (00h) and Monitor (01h), once a second: the part answering, and what it reports about
    /// itself. Monitor is DIE_REV 7:4 (3 is mass production), SC_STAT 3, BE_STAT 2, COOL_STAT 1,
    /// UV/OT_STAT 0.
    struct Status {
        static constexpr auto       Period = std::chrono::seconds{1};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2, .offset = 0})};

        struct Sample {
            std::uint8_t id{};
            std::uint8_t monitor{};

            [[nodiscard]] constexpr bool idMatches() const { return id == ChipId; }

            [[nodiscard]] constexpr std::uint8_t dieRevision() const { return monitor >> 4U; }

            /// At least one LED output is shorted to ground.
            [[nodiscard]] constexpr bool shortCircuit() const { return (monitor & 0x08U) != 0; }

            /// BrightExtend is scaling the currents down because a sink is in dropout.
            [[nodiscard]] constexpr bool brightExtend() const { return (monitor & 0x04U) != 0; }

            /// CoolExtend is scaling the currents down because the die is hot.
            [[nodiscard]] constexpr bool coolExtend() const { return (monitor & 0x02U) != 0; }

            /// VCC under its lockout threshold, or thermal shutdown.
            [[nodiscard]] constexpr bool underVoltageOrOverTemp() const {
                return (monitor & 0x01U) != 0;
            }

            [[nodiscard]] constexpr bool fault() const { return (monitor & 0x0FU) != 0; }

            [[nodiscard]] constexpr bool operator==(Sample const&) const = default;
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {data.u8(0), data.u8(1)};
        }
    };

    /// The two colour slots as one block at 03h: R0 G0 B0 R1 G1 B1, a current code each.
    struct Colour {
        using Value                        = std::array<std::uint8_t, 3 * Slots>;
        static constexpr std::size_t Bytes = 3 * Slots;
        static constexpr Value       Initial{};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            for(std::size_t i = 0; i < Bytes; ++i) { buffer[i] = std::byte{value[i]}; }
            return Step::writeBuffer({.reg = 0x03, .offset = 0, .count = Bytes});
        }
    };

    /// The six select registers as one block at 09h. Every LED off to begin with, so a part that
    /// comes up mid-animation shows nothing rather than the last frame.
    struct Select {
        using Value                        = std::array<std::uint8_t, Leds / 2>;
        static constexpr std::size_t Bytes = Leds / 2;
        static constexpr Value       Initial{};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            for(std::size_t i = 0; i < Bytes; ++i) { buffer[i] = std::byte{value[i]}; }
            return Step::writeBuffer({.reg = 0x09, .offset = 0, .count = Bytes});
        }
    };

    /// Control (02h), so the mode and the fade rate can be changed at run time.
    using Control = Groups::InitialByte<0x02, ControlValue>;

    /// The select block with `module` set to `channels`, for
    /// `set<Select>(Ktd2061::with(current, Module::B3, channels))`.
    [[nodiscard]] static constexpr typename Select::Value with(typename Select::Value block,
                                                               Module                 module,
                                                               Channels               channels) {
        return with(block, static_cast<std::size_t>(module) + 1, channels);
    }

    /// `led` is 1-based in register order: 1 is A1, 2 is A2, ... 12 is C4. The first module of a
    /// register's pair sits in its high nibble.
    [[nodiscard]] static constexpr typename Select::Value with(typename Select::Value block,
                                                               std::size_t            led,
                                                               Channels               channels) {
        auto const index = (led - 1) / 2;
        auto const shift = (led - 1) % 2 == 0 ? 4U : 0U;
        auto const bits  = static_cast<unsigned>(channels.bits()) << shift;
        block[index]     = static_cast<std::uint8_t>((block[index] & ~(0x0FU << shift)) | bits);
        return block;
    }

    using Reads  = List<Status>;
    using Writes = List<Control, Colour, Select>;
};

}   // namespace Kvasir::I2C::Chips
