#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Ds1307Detail {
    struct Time {
        std::uint8_t second{}, minute{}, hour{};   ///< 24-hour
        std::uint8_t weekday{1}, day{1}, month{1};
        std::uint8_t year{};     ///< 0..99
        bool         halted{};   ///< the oscillator is stopped (read only)
    };

    /// Two BCD digits are each 0..9: 0xFF from a part that is not there, or a register that
    /// was never written, is not a time.
    [[nodiscard]] constexpr bool bcdOk(std::uint8_t byte,
                                       std::uint8_t mask) {
        auto const v = static_cast<std::uint8_t>(byte & mask);
        return (v >> 4) <= 9 && (v & 0x0FU) <= 9;
    }

    /// The seven BCD registers both Maxim clocks share (DS1307 Table 2, DS3231 Figure 1):
    /// seconds (bit 7 CH on the DS1307), minutes, hours, day of week, date, month (bit 7
    /// century on the DS3231), year. Hours: bit 6 set is 12-hour mode, and then bit 5 is
    /// PM and bits 4:0 the hour 1..12, which is turned into 0..23 here; SetTime always
    /// writes 24-hour mode. False when a digit is not a digit or a field is out of range:
    /// the frame is rejected.
    [[nodiscard]] constexpr bool decodeTime(Bytes data,
                                            Time& t) {
        auto const hours   = data.u8(2);
        bool const mode12  = (hours & 0x40U) != 0;
        auto const hourMsk = static_cast<std::uint8_t>(mode12 ? 0x1F : 0x3F);
        if(!bcdOk(data.u8(0), 0x7F) || !bcdOk(data.u8(1), 0x7F) || !bcdOk(hours, hourMsk)
           || !bcdOk(data.u8(3), 0x07) || !bcdOk(data.u8(4), 0x3F) || !bcdOk(data.u8(5), 0x1F)
           || !bcdOk(data.u8(6), 0xFF))
        {
            return false;
        }
        t.halted  = (data.u8(0) & 0x80U) != 0;
        t.second  = data.bcd(0, 0x7F);
        t.minute  = data.bcd(1, 0x7F);
        t.hour    = data.bcd(2, hourMsk);
        t.weekday = data.bcd(3, 0x07);
        t.day     = data.bcd(4, 0x3F);
        t.month   = data.bcd(5, 0x1F);
        t.year    = data.bcd(6);
        if(mode12) {
            // 12 AM is 0, 1..11 AM stay, 12 PM is 12, 1..11 PM are 13..23
            bool const pm = (hours & 0x20U) != 0;
            if(t.hour == 12) { t.hour = 0; }
            if(pm) { t.hour = static_cast<std::uint8_t>(t.hour + 12); }
        }
        return t.second < 60 && t.minute < 60 && t.hour < 24;
    }

    constexpr void encodeTime(Time const&          t,
                              std::span<std::byte> out) {
        out[0] = std::byte{toBcd(t.second)};   // CH = 0: the oscillator runs
        out[1] = std::byte{toBcd(t.minute)};
        out[2] = std::byte{toBcd(t.hour)};   // bit 6 clear: 24-hour mode
        out[3] = std::byte{toBcd(t.weekday)};
        out[4] = std::byte{toBcd(t.day)};
        out[5] = std::byte{toBcd(t.month)};
        out[6] = std::byte{toBcd(t.year)};
    }

    struct Clock {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 7})};
        using Sample = Time;

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            Sample t{};
            if(!decodeTime(data, t)) { return Outcome<Sample>::reject(); }
            return Outcome<Sample>::ok(t);
        }
    };

    struct SetTime {
        using Value                        = Time;
        static constexpr std::size_t Bytes = 7;

        /// Setting the clock is a one-shot command: replaying it after a reset would put
        /// the stale time back on a chip that has been keeping its own since.
        static constexpr bool Transient = true;

        [[nodiscard]] static constexpr Step encode(Value const&         time,
                                                   std::span<std::byte> buffer) {
            encodeTime(time, buffer);
            return Step::writeBuffer({.reg = 0x00, .offset = 0, .count = 7});
        }
    };
}   // namespace Ds1307Detail

/// Maxim DS1307 real-time clock (DS1307 datasheet, Table 2). Fixed address 0x68, one-byte
/// pointer, auto-increment. The time is read every second, in 24-hour form whichever mode
/// the part is in, and a frame with a non-BCD digit is rejected; `set<SetTime>(t)` writes
/// all seven registers with CH cleared and 24-hour mode. A frame with CH set is rejected too:
/// the oscillator is stopped -- a new part comes that way, reading 01/01/00 -- so the
/// registers hold no time (Linux rtc-ds1307.c refuses it the same way); rejected<Clock>()
/// counts those reads until a time is set. The DS3231 has no CH bit and shares the rest.
/// Standard mode only: 100 kHz at most on the bus it sits on.
struct Ds1307 {
    static constexpr std::string_view        Name    = "DS1307";
    static constexpr Address7                Address = 0x68;
    static constexpr std::array<Address7, 1> Addresses{0x68};
    static constexpr std::size_t             RegisterBytes = 1;

    using Time    = Ds1307Detail::Time;
    using SetTime = Ds1307Detail::SetTime;

    struct Clock : Ds1307Detail::Clock {
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            auto const got = Ds1307Detail::Clock::decode(data);
            if(got.kind == Outcome<Sample>::Kind::ok && got.value.halted) {
                return Outcome<Sample>::reject();
            }
            return got;
        }
    };

    using Reads  = List<Clock>;
    using Writes = List<SetTime>;
};

/// Maxim DS3231: the same seven time registers at 0x00, so the layout is the DS1307's plus
/// control 0x0E, status 0x0F with OSF bit 7, aging 0x10, and temperature 0x11 (int8 degC) /
/// 0x12 (bits 7..6: quarters). Temperature is read every 10 s along with the status byte.
/// Fixed address 0x68.
struct Ds3231 {
    static constexpr std::string_view        Name    = "DS3231";
    static constexpr Address7                Address = 0x68;
    static constexpr std::array<Address7, 1> Addresses{0x68};
    static constexpr std::size_t             RegisterBytes = 1;

    using Time    = Ds1307Detail::Time;
    using Clock   = Ds1307Detail::Clock;
    using SetTime = Ds1307Detail::SetTime;

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{10000};
        static constexpr std::array Steps{Step::read({.reg = 0x0F, .count = 1, .offset = 0}),
                                          Step::read({.reg = 0x11, .count = 2, .offset = 1})};

        struct Sample {
            CentiDegC temperature{};         ///< the die, in quarter degrees (0.25 degC steps)
            bool      oscillatorStopped{};   ///< OSF: the time is not to be trusted
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.oscillatorStopped = (data.u8(0) & 0x80U) != 0;
            auto const quarters      = static_cast<std::int32_t>(data.s8(1)) * 4
                                     + static_cast<std::int32_t>(data.u8(2) >> 6);
            sample.temperature       = Units::centiDegC(quarters * 25);
            return sample;
        }
    };

    using Reads  = List<Clock, Temperature>;
    using Writes = List<SetTime>;
};

}   // namespace Kvasir::I2C::Chips
