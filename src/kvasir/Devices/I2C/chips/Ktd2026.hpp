#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Ktd2026Detail {
    /// Which of the part's channels each colour is wired to: 0 is D1 (current register
    /// 0x06, channel-control bits 1:0), 1 is D2 (0x07, bits 3:2), 2 is D3 (0x08, bits 5:4).
    /// The default is red on D1, blue on D2, green on D3.
    struct Wiring {
        std::uint8_t red{};
        std::uint8_t green{};
        std::uint8_t blue{};
    };

    inline constexpr Wiring DefaultWiring{0, 2, 1};

    /// Red, green and blue, 0 meaning off, then 1..192 for the part's 0.125..24 mA.
    struct Rgb {
        std::uint8_t r{};
        std::uint8_t g{};
        std::uint8_t b{};

        constexpr bool operator==(Rgb const&) const = default;
    };
}   // namespace Ktd2026Detail

/// Kinetic KTD2026 three-channel RGB LED driver (KTD2026-7 datasheet). One-byte pointer:
/// EN_RST 0x00 (enable control in bits 4:3, reset and timer-slot commands in bits 2:0), the
/// flash period and on-time registers 0x01..0x03, channel control 0x04 (two bits per
/// channel, D1 in bits 1:0: 00 off, 01 always on, 10 and 11 the two flash timers), the ramp
/// register 0x05, then the current registers 0x06 D1, 0x07 D2, 0x08 D3. The datasheet only
/// shows single-register writes and says nothing of the pointer incrementing, and Linux's
/// leds-ktd202x writes one register at a time too, so every register here is its own
/// transaction.
///
/// Bring-up: after power-up the part is to be reset with Reg0[2:0] = 111 and left 200 us;
/// the last byte of that command is not acknowledged, which is to be ignored ("Reg0 [2:0]"
/// and the note under "Write"). The probe before it is a write of 0x00 to EN_RST, which is
/// what the reset puts there anyway. Then EN_RST = 0x1C: enable control 11, "always on" --
/// the part never enters shutdown, where with 00 it would shut down whenever SCL or SDA went
/// low, which on a shared bus is every transfer -- and 100 in bits 2:0, "do nothing".
///
/// A current register counts from zero: 0x00 is 0.125 mA, 0xBF (191) is 24 mA, and any
/// higher code is 24 mA too ("Reg6, Reg7, Reg8, Reg9 LED Current Setting"). The application
/// value is offset by one against that: 0 means "channel off", 1..192 programs value - 1,
/// and more is clamped to 191.
///
/// Which colour sits on which channel is board wiring, so it is the `Wiring` template
/// parameter. `Timing::StartupDelay` is the wait after power before the first transaction: the
/// datasheet gives no figure and 500 ms is a conservative default. Address 0x30; the -B and -C variants are 0x31 and 0x32.
template<Ktd2026Detail::Wiring Wire = Ktd2026Detail::DefaultWiring, typename Timing = DefaultTiming>
struct Ktd2026 {
    static constexpr std::string_view Name          = "KTD2026";
    static constexpr Address7         Address       = 0x30;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array<Address7, 3> Addresses{0x30, 0x31, 0x32};

    static_assert(Wire.red < 3 && Wire.green < 3 && Wire.blue < 3,
                  "a colour is wired to D1, D2 or D3 (0, 1, 2)");
    static_assert(Wire.red != Wire.green && Wire.green != Wire.blue && Wire.red != Wire.blue,
                  "one colour per channel");

    static constexpr std::chrono::milliseconds StartupDelay = [] {
        if constexpr(requires { Timing::StartupDelay; }) {
            return Kvasir::asDuration(Timing::StartupDelay);
        } else {
            return std::chrono::milliseconds{500};
        }
    }();

    /// The highest current code: 24 mA. Higher codes are 24 mA too.
    static constexpr std::uint8_t MaxLevel = 0xBF;

    using Rgb = Ktd2026Detail::Rgb;

    static constexpr std::array Init{
      Step::write({.reg = 0x00, .payload = {0x00}}),
      Step::write({.reg     = 0x00,
                   .payload = {0x07},
                   .delay   = std::chrono::milliseconds{1},
                   .mayNak  = true}),   // reset complete chip, 200 us
      Step::write({.reg = 0x00, .payload = {0x1C}}),
    };

    /// The current register code for an application level: 0 stays off (the channel is
    /// switched off in channel control instead), else level - 1 clamped to MaxLevel.
    [[nodiscard]] static constexpr std::uint8_t code(std::uint8_t level) {
        if(level == 0) { return 0; }
        auto const c = static_cast<std::uint8_t>(level - 1);
        return c > MaxLevel ? MaxLevel : c;
    }

    /// Red, green and blue, 0 meaning off.
    struct Colour {
        using Value = Rgb;

        static constexpr std::size_t Bytes = 5;
        static constexpr Value       Initial{0, 0, 0};

        /// The ramp byte and the three currents, in the chip's D1 D2 D3 order, each from the
        /// colour `Wiring` puts there, then channel control last, so a channel is switched on
        /// at its new current.
        [[nodiscard]] static constexpr std::array<Step,
                                                  5>
        encode(Value const&         value,
               std::span<std::byte> buffer) {
            std::array<std::uint8_t, 3> level{};
            level[Wire.red]   = value.r;
            level[Wire.green] = value.g;
            level[Wire.blue]  = value.b;
            std::uint8_t on   = 0;
            for(std::size_t i = 0; i < 3; ++i) {
                if(level[i] != 0) { on = static_cast<std::uint8_t>(on | (0x01U << (2 * i))); }
                buffer[2 + i] = std::byte{code(level[i])};
            }
            buffer[0] = std::byte{on};
            buffer[1] = std::byte{0x00};   // no ramp
            return {Step::writeBuffer({.reg = 0x05, .offset = 1, .count = 1}),
                    Step::writeBuffer({.reg = 0x06, .offset = 2, .count = 1}),
                    Step::writeBuffer({.reg = 0x07, .offset = 3, .count = 1}),
                    Step::writeBuffer({.reg = 0x08, .offset = 4, .count = 1}),
                    Step::writeBuffer({.reg = 0x04, .offset = 0, .count = 1})};
        }
    };

    using Writes = List<Colour>;
};

}   // namespace Kvasir::I2C::Chips
