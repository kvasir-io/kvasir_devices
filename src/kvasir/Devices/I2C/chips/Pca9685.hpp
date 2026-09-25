#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// NXP PCA9685 16-channel 12-bit PWM (data sheet rev. 4). One-byte control register as
/// pointer, auto-increment with MODE1.AI. Bring-up: MODE1 = 0x10 (SLEEP: PRE_SCALE is
/// writable only asleep, 7.3.5), PRE_SCALE = round(25 MHz / (4096 f)) - 1 (eq. 1: 121 for
/// 50 Hz), MODE2 = 0x04 (totem pole), MODE1 = 0x20 (AI, oscillator on), 500 us, MODE1 =
/// 0xA0 (RESTART | AI). A channel is four registers from 06h + 4n: ON_L, ON_H (bit 4 full
/// on), OFF_L, OFF_H (bit 4 full off), counts 0..4095 (7.3.3). `set<Channel>(n, {on,
/// off})` writes that channel's four registers in one transaction; all sixteen start off.
///
/// The part has no id register and its addresses overlap the INA219, HTU21D and many more, so
/// the bring-up reads PRE_SCALE and MODE2 back at the end: a part that does not hold the
/// prescale and the 0x04 just written is not this one, and nothing more goes to it.
template<Hertz Frequency = Units::hertz(50)>
struct Pca9685 {
    static constexpr std::string_view Name    = "PCA9685";
    static constexpr Address7         Address = 0x40;
    /// 1 A5 A4 A3 A2 A1 A0 (7.1), less 0x70 -- the LED All Call address every part answers
    /// at by default (ALLCALLADR, 7.1.2), so a part strapped there cannot be told apart --
    /// and 0x78..0x7F, which the I2C specification reserves. 0x71..0x77 are usable: the Sub Call
    /// addresses that overlap them are disabled at power-up (7.1.3) and only answer once a SUBx
    /// bit is set.
    static constexpr std::array<Address7, 55> Addresses{
      0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D,
      0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B,
      0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
      0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77};
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::uint8_t Prescale = [] {
        constexpr auto f = Units::value(Frequency);
        constexpr auto p = (25'000'000U + 2048U * f) / (4096U * f) - 1U;
        static_assert(p >= 3 && p <= 255, "the PWM frequency is 24..1526 Hz");
        return static_cast<std::uint8_t>(p);
    }();

    static constexpr std::array Init{
      Step::write({.reg = 0x00, .payload = {0x10}}),
      Step::write({.reg = 0xFE, .payload = {Prescale}}),
      Step::write({.reg = 0x01, .payload = {0x04}}),
      Step::write({.reg = 0x00, .payload = {0x20}, .delay = std::chrono::milliseconds{1}}),
      Step::write({.reg = 0x00, .payload = {0xA0}}),
      Step::read({.reg = 0xFE, .count = 1, .offset = 0}),   // PRE_SCALE back
      Step::read({.reg = 0x01, .count = 1, .offset = 1}),   // MODE2 back
    };

    struct State {
        std::uint8_t prescale{};   ///< PRE_SCALE as read back
        std::uint8_t mode2{};      ///< MODE2 as read back
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.prescale = data.u8(0);
        state.mode2    = data.u8(1);
        return state.prescale == Prescale && state.mode2 == 0x04;
    }

    struct Pwm {
        std::uint16_t on{};    ///< 0..4095, 4096: always on
        std::uint16_t off{};   ///< 0..4095, 4096: always off

        /// A duty of 0..4095 with no phase delay.
        [[nodiscard]] static constexpr Pwm duty(std::uint16_t d) {
            if(d == 0) { return {0, 4096}; }
            if(d >= 4095) { return {4096, 0}; }
            return {0, d};
        }
    };

    struct Channel {
        using Value                        = Pwm;
        static constexpr std::size_t Items = 16;
        static constexpr std::size_t Bytes = 4;
        static constexpr Value       Initial{0, 4096};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value.on & 0xFF);
            buffer[1] = static_cast<std::byte>(value.on >> 8);
            buffer[2] = static_cast<std::byte>(value.off & 0xFF);
            buffer[3] = static_cast<std::byte>(value.off >> 8);
            return Step::writeBuffer(
              {.reg = static_cast<std::uint16_t>(0x06 + 4 * item), .offset = 0, .count = 4});
        }
    };

    using Writes = List<Channel>;
};

}   // namespace Kvasir::I2C::Chips
