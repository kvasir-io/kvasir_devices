#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// NXP PCF85063A real-time clock (datasheet 4.1, 2015). Fixed address 0x51, one-byte pointer
/// that auto-increments: Control_1 0x00, Control_2 0x01, Offset 0x02, RAM_byte 0x03, then the
/// seven time registers 0x04..0x0A -- seconds (bit 7 is the oscillator-stop flag), minutes,
/// hours, days, weekdays, months, years -- all BCD.
///
/// The shape is `Ds1307`'s: one `Time` read group and one `SetTime` write group, and the same
/// warning applies -- a `year` is the two BCD digits the part holds, so it is 0..99 against
/// whatever century the application means. `Epoch` is that century's first year, 2000 by
/// default; a SetTime year before it is written as 0 and one past Epoch + 99 as 99. The part
/// adds 29 February whenever the year register is divisible by 4, 00 included (Table 23 note
/// 1), so leap years come out right across the whole range only when Epoch is a multiple of
/// 4 and the hundred years it spans contain neither 1900 nor 2100, as with the default 2000.
///
/// `LoadCapacitance` is Control_1 bit 0 (CAP_SEL, Table 6). It is a property of the crystal
/// fitted, not a preference: 7 pF against a 12.5 pF crystal pulls the oscillator and the
/// clock drifts. It defaults to 12.5 pF, the commoner part; the reset value is 7 pF, so a
/// design with a 7 pF crystal must say so.
namespace Pcf85063aDetail {
    enum class LoadCapacitance : std::uint8_t {
        pf7    = 0x00,
        pf12_5 = 0x01
    }; }   // namespace Pcf85063aDetail

template<unsigned                         Epoch = 2000,
         Pcf85063aDetail::LoadCapacitance Load  = Pcf85063aDetail::LoadCapacitance::pf12_5>
struct Pcf85063a {
    static constexpr std::string_view        Name    = "PCF85063A";
    static constexpr Address7                Address = 0x51;
    static constexpr std::array<Address7, 1> Addresses{0x51};
    static constexpr std::size_t             RegisterBytes = 1;

    static_assert(Epoch >= 1900 && Epoch <= 2100,
                  "Epoch is the first year of the century the two-digit year counts from");

    /// Control_1: normal mode, the clock running, no software reset, no correction interrupt,
    /// 24-hour mode, and the crystal's load capacitance. Written at every bring-up, so a part
    /// that came up in 12-hour mode or at the wrong load after losing power is put back.
    static constexpr std::array Init{
      Step::write({.reg = 0x00, .payload = {static_cast<std::uint8_t>(Load)}})};

    struct Time {
        static constexpr auto Period = std::chrono::milliseconds{1000};

        static constexpr std::array Steps{Step::read({.reg = 0x04, .count = 7, .offset = 0})};

        struct Sample {
            std::uint8_t  second{};
            std::uint8_t  minute{};
            std::uint8_t  hour{};
            std::uint8_t  day{};       ///< 1..31
            std::uint8_t  weekday{};   ///< 0 = Sunday
            std::uint8_t  month{};     ///< 1..12
            std::uint16_t year{};      ///< Epoch + the part's two digits
            /// Seconds bit 7: the oscillator stopped, so the time is not to be trusted.
            bool integrity{};
        };

        /// Every field's flag bits are masked before the BCD conversion, which is what
        /// `Bytes::bcd(i, mask)` is for.
        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {data.bcd(0, 0x7F),
                    data.bcd(1, 0x7F),
                    data.bcd(2, 0x3F),
                    data.bcd(3, 0x3F),
                    data.bcd(4, 0x07),
                    data.bcd(5, 0x1F),
                    static_cast<std::uint16_t>(Epoch + data.bcd(6)),
                    (data.u8(0) & 0x80U) == 0};
        }
    };

    /// Setting the clock. Transient: a reset must not replay it, or the part would be put
    /// back to whatever o'clock the application last happened to write. The time goes in with
    /// the STOP bit set around it (8.2.1.2): STOP holds the prescaler in reset, so the first
    /// second after the set is a whole second from releasing it, instead of anywhere between
    /// 0 and 1 s, as Linux rtc-pcf85063.c sets it.
    struct SetTime {
        using Value = typename Time::Sample;

        static constexpr std::size_t Bytes     = 7;
        static constexpr bool        Transient = true;

        /// The two digits a year is written as: 0 for anything before Epoch, 99 for anything
        /// past Epoch + 99.
        [[nodiscard]] static constexpr std::uint8_t yearDigits(std::uint16_t year) {
            if(year < Epoch) { return 0; }
            auto const y = year - Epoch;
            return static_cast<std::uint8_t>(y > 99 ? 99 : y);
        }

        [[nodiscard]] static constexpr std::array<Step,
                                                  3>
        encode(Value const&         value,
               std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(toBcd(value.second));   // clears the stop flag
            buffer[1] = static_cast<std::byte>(toBcd(value.minute));
            buffer[2] = static_cast<std::byte>(toBcd(value.hour));
            buffer[3] = static_cast<std::byte>(toBcd(value.day));
            buffer[4] = static_cast<std::byte>(toBcd(value.weekday));
            buffer[5] = static_cast<std::byte>(toBcd(value.month));
            buffer[6] = static_cast<std::byte>(toBcd(yearDigits(value.year)));
            return {Step::write({.reg     = 0x00,
                                 .payload = {static_cast<std::uint8_t>(
                                   0x20U | static_cast<unsigned>(Load))}}),
                    Step::writeBuffer({.reg = 0x04, .offset = 0, .count = 7}),
                    Step::write({.reg = 0x00, .payload = {static_cast<std::uint8_t>(Load)}})};
        }
    };

    using Reads  = List<Time>;
    using Writes = List<SetTime>;
};

}   // namespace Kvasir::I2C::Chips
