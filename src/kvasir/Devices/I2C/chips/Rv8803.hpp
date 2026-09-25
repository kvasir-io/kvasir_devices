#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ratio>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Rv8803Detail {
    /// Hundredths of a second: the finest step the part counts in (3.2, register 10h).
    using Centiseconds = std::chrono::duration<std::int32_t, std::centi>;

    /// A wall-clock instant as the part keeps it. There is no time zone and no leap second in
    /// the part, so the natural chrono spelling is a `local_time` (local()).
    ///
    /// `date` is 2000-01-01 .. 2099-12-31: the part holds two BCD year digits and handles leap
    /// years "correctly from 2000 to 2099" (3.3, register 17h), so the century is fixed.
    /// `weekday` is kept by the part, not derived by it (3.3: "each bit represents one weekday
    /// that is assigned by the user"); this description assigns bit 0 to Sunday, bit 6 to
    /// Saturday, which is `std::chrono::weekday::c_encoding()` and SparkFun's convention.
    /// `time` is the time of day, hours down to hundredths.
    struct Time {
        std::chrono::year_month_day date{std::chrono::year{2000},
                                         std::chrono::January,
                                         std::chrono::day{1}};
        std::chrono::weekday        weekday{std::chrono::Saturday};
        Centiseconds                time{};
        /// V2F clear (3.7): no supply drop below VLOW2 since the time was last set, so the
        /// time and date are the ones that were set. False after a power-on reset.
        bool valid{};
        /// V1F clear (3.7): the temperature compensation has run without a supply drop below
        /// VLOW1, so the accuracy specification holds. Read only; SetTime ignores both flags.
        bool compensated{};

        /// A Time at `day`, `sinceMidnight` in: the weekday derived from the date. What an
        /// application hands SetTime.
        [[nodiscard]] static constexpr Time at(std::chrono::local_days day,
                                               Centiseconds            sinceMidnight) {
            Time t{};
            t.date    = std::chrono::year_month_day{day};
            t.weekday = std::chrono::weekday{day};
            t.time    = sinceMidnight;
            return t;
        }

        /// Hours, minutes, seconds and hundredths.
        [[nodiscard]] constexpr std::chrono::hh_mm_ss<Centiseconds> hms() const {
            return std::chrono::hh_mm_ss<Centiseconds>{time};
        }

        /// The instant as a chrono time point, for arithmetic and formatting.
        [[nodiscard]] constexpr std::chrono::local_time<std::chrono::duration<std::int64_t,
                                                                              std::centi>>
        local() const {
            using Precise = std::chrono::duration<std::int64_t, std::centi>;
            return std::chrono::local_time<Precise>{
              std::chrono::duration_cast<Precise>(std::chrono::local_days{date}.time_since_epoch())
              + Precise{time}};
        }
    };

    /// FD, extension register 1Dh bits 3:2 (3.6, 4.9): the square wave on CLKOUT while CLKOE is
    /// high. FD = 11 is 32.768 kHz as well and is not listed.
    enum class ClockOut : std::uint8_t {
        khz32_768 = 0,   ///< the reset value
        hz1024    = 1,   ///< carries the compensation pulses (4.9 note 1)
        hz1       = 2,   ///< what 5.4's offset calibration measures
    };

    /// EHL, event control 2Fh bit 6 (3.10).
    enum class EventLevel : std::uint8_t {
        low  = 0,   ///< a low level / falling edge on EVI is the event (reset value)
        high = 1,
    };

    /// ET, event control 2Fh bits 5:4 (3.10, 4.8.2): EVI's digital debounce.
    enum class EventFilter : std::uint8_t {
        none   = 0,   ///< edge detection, pulses of 30.5 us and longer
        ms3_9  = 1,   ///< sampled at 256 Hz
        ms15_6 = 2,   ///< 64 Hz
        ms125  = 3,   ///< 8 Hz
    };

    /// The EventControl group's value. Out here because a default member initializer may not
    /// be used from inside the class that encloses it.
    /// Event control ECP: copy seconds and hundredths into 20h/21h on an event.
    enum class Capture : std::uint8_t { off, on };

    struct EventCapture {
        Capture     capture{Capture::off};   ///< ECP
        EventLevel  level{EventLevel::low};
        EventFilter filter{EventFilter::none};

        friend constexpr bool operator==(EventCapture const&,
                                         EventCapture const&) = default;
    };

    /// The Event group's sample: when within its minute the last event on EVI happened.
    struct EventStamp {
        /// Seconds CP and hundredths CP (21h, 20h) as one duration, 0 .. 59.99 s. Negative
        /// until the first event has been seen: no stamp yet.
        Centiseconds stamp{-1};
    };

    /// Two BCD digits, each 0..9, after `mask`: 0xFF from a part that is not there, or a
    /// register left undefined by a power-on reset (3.11: "X means undefined"), is not a time.
    [[nodiscard]] constexpr bool bcdOk(std::uint8_t byte,
                                       std::uint8_t mask) {
        auto const v = static_cast<std::uint8_t>(byte & mask);
        return (v >> 4U) <= 9 && (v & 0x0FU) <= 9;
    }

    /// Weekday register (3.3): exactly one of bits 0..6, bit 7 read as 0. The bit index is the
    /// weekday's c_encoding(), or 7 when the byte is not one-hot.
    [[nodiscard]] constexpr unsigned weekdayBit(std::uint8_t byte) {
        for(unsigned bit = 0; bit < 7; ++bit) {
            if(byte == (1U << bit)) { return bit; }
        }
        return 7;
    }
    /// Control EIE: an event on EVI pulls INT low.
    enum class EventInterrupt : std::uint8_t { off, on };
}   // namespace Rv8803Detail

/// Micro Crystal RV-8803-C7 temperature-compensated real-time clock (Application Manual rev
/// 1.7, September 2026). Fixed address 0x32 (6.6: 0110010b), one-byte pointer that auto-increments
/// (3, 6.7).
///
/// Register map used here (3.1): the extension range 10h..1Fh, which mirrors 00h..0Fh and
/// adds hundredths -- 10h hundredths (read only), 11h seconds, 12h minutes, 13h hours (24 h
/// only), 14h weekday (one-hot), 15h date, 16h month, 17h year, all BCD; 1Dh extension (TEST 7,
/// WADA 6, USEL 5, TE 4, FD 3:2, TD 1:0), 1Eh flags (UF 5, TF 4, AF 3, EVF 2, V2F 1, V1F 0),
/// 1Fh control (UIE 5, TIE 4, AIE 3, EIE 2, RESET 0); then 20h hundredths CP and 21h seconds CP
/// (read only), 2Ch offset (6-bit two's complement) and 2Fh event control (ECP 7, EHL 6,
/// ET 5:4, ERST 0).
///
/// There is no identification register. setup() instead checks the bits the manual says
/// always read 0 -- flags 7:6, control 1 and offset 7:6 (3.1, 3.9) -- so a part answering at
/// 0x32 that is not an RV-8803 is at least likely to be refused; it is not proof.
///
/// Bring-up. A power-on reset leaves the time undefined and sets V1F and V2F (3.7, 3.11).
/// The Init script reads the flag, control and offset registers first, so State records what
/// the part held -- `timeLost` is V2F -- and then clears UF, TF, AF and EVF (writing 0) while
/// writing 1 to V1F and V2F, which leaves them unchanged (3.7, "Write: The V2F bit remains
/// unchanged"). The voltage flags are deliberately not cleared at bring-up: writing 0 to
/// either clears both (3.7), and a V2F cleared by a bring-up would make an undefined time
/// look valid to every later reader, including a bring-up after a mere bus fault. V2F stays
/// set, and every Clock sample says `valid = false`, until SetTime writes a time. Control
/// is then written with RESET clear, so a clock left stopped by an interrupted SetTime runs
/// again; `State::wasStopped` says it had been.
///
/// Reading (4.12). The registers cannot be frozen during a read, so a read that straddles a
/// 1 Hz tick can return a mix (4.12.1's 01:59 read as 02:59). The manual re-reads the seconds
/// when they were 59 (4.12.2); this description re-reads them after every burst and runs the
/// read again when they changed, because the hundredths read first are just as torn by a
/// tick at any other second (x.99 then the next second is a whole second late).
///
/// Setting the time follows 4.13: RESET set (the divider chain below 1 Hz and the hundredths
/// are held at 0), the seven registers written, the voltage flags cleared, RESET released --
/// so the second starts counting at the last transaction, and the time is marked valid before
/// it does. The four transactions are one SetTime item (a failure anywhere repeats them all).
///
/// Interrupts. `Events` (EventInterrupt) is the only one this description can drive: EIE, so an event
/// on EVI pulls INT low until the Event group clears EVF. The alarm, countdown timer and
/// periodic update functions are not driven; their enables stay 0, and the extension
/// register's fields for them (WADA, USEL, TE, TD) are written as their reset values 0
/// whenever ClockOut is written. ERST (4.14, the event-synchronised time set) is not used.
///
/// The ~7.4 ppm OFFSET (3.9, 5.4) is a write group in raw steps of 1/(32768 * 128) =
/// 0.23842 ppm, -32 .. +31: Units has no signed sub-ppm quantity to carry it.
///
/// "For devices with production date code 853 and earlier it is recommended ... to complete
/// the I2C-bus access always with a Read Operation followed by the STOP condition" (7.6).
/// Parts that old are not accommodated: Init, SetTime and the write groups end with a write.
template<Rv8803Detail::ClockOut       Fd     = Rv8803Detail::ClockOut::khz32_768,
         Rv8803Detail::EventInterrupt Events = Rv8803Detail::EventInterrupt::off>
struct Rv8803 {
    static constexpr std::string_view        Name    = "RV-8803";
    static constexpr Address7                Address = 0x32;
    static constexpr std::array<Address7, 1> Addresses{0x32};
    static constexpr std::size_t             RegisterBytes = 1;

    /// The part does not acknowledge its address during a 61 us window while it updates its
    /// internal counters, once a second (Linux rtc-rv8803.c retries every transfer up to four
    /// times for it); a NAK there is put back on the wire rather than counted as a fault.
    static constexpr std::uint8_t WakeRetries = 3;

    /// tPOR1, the power-on delay (7.4: typ 3 ms, max 10 ms). Verify: the table describes the
    /// CLKOUT pin, and the power-on reset duration tPOR2 behind it is 80 ms typ, 500 ms max;
    /// the manual does not say when the I2C interface answers.
    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    using Time         = Rv8803Detail::Time;
    using Centiseconds = Rv8803Detail::Centiseconds;
    using ClockOutRate = Rv8803Detail::ClockOut;
    using Capture      = Rv8803Detail::Capture;
    using EventCapture = Rv8803Detail::EventCapture;
    using EventLevel   = Rv8803Detail::EventLevel;
    using EventFilter  = Rv8803Detail::EventFilter;

    // flag register 1Eh (3.7)
    static constexpr std::uint8_t V1F = 0x01;
    static constexpr std::uint8_t V2F = 0x02;
    static constexpr std::uint8_t EVF = 0x04;
    static constexpr std::uint8_t AF  = 0x08;
    static constexpr std::uint8_t TF  = 0x10;
    static constexpr std::uint8_t UF  = 0x20;

    // control register 1Fh (3.8)
    static constexpr std::uint8_t Reset = 0x01;
    static constexpr std::uint8_t Eie   = 0x04;

    /// Control as this description keeps it: every interrupt off but EIE when asked, RESET 0.
    static constexpr std::uint8_t Control = Events == Rv8803Detail::EventInterrupt::on ? Eie : 0x00;

    /// A write of 0 clears a flag; a write of 1 leaves it (3.7 says so for V1F and V2F; for UF,
    /// TF, AF and EVF it only says "cleared by writing a 0" -- verify that a 1 is ignored).
    static constexpr std::uint8_t ClearEventFlags   = static_cast<std::uint8_t>(V1F | V2F);
    static constexpr std::uint8_t ClearVoltageFlags = static_cast<std::uint8_t>(UF | TF | AF | EVF);
    static constexpr std::uint8_t ClearEvfOnly
      = static_cast<std::uint8_t>(UF | TF | AF | V2F | V1F);

    static constexpr std::array Init{
      Step::read({.reg = 0x1E, .count = 2, .offset = 0}),   // flags, control: what the part held
      Step::read({.reg = 0x2C, .count = 1, .offset = 2}),   // offset
      Step::identify(),
      Step::write(
        {.reg     = 0x1E,
         .payload = {ClearEventFlags}}),   // UF TF AF EVF cleared, V1F V2F left as they are
      Step::write(
        {.reg = 0x1F, .payload = {Control}}),   // RESET released, interrupts as configured
    };

    struct State {
        bool         timeLost{};              ///< V2F at bring-up: the time is undefined
        bool         compensationStopped{};   ///< V1F at bring-up
        bool         wasStopped{};            ///< RESET was set at bring-up: the clock had stopped
        std::int8_t  offset{};                ///< OFFSET steps, 0.23842 ppm each
        ClockOutRate clockOut{Fd};            ///< FD as last written
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        auto const flags          = data.u8(0);
        auto const control        = data.u8(1);
        auto const offset         = data.u8(2);
        state.timeLost            = (flags & V2F) != 0;
        state.compensationStopped = (flags & V1F) != 0;
        state.wasStopped          = (control & Reset) != 0;
        state.offset              = static_cast<std::int8_t>(Bytes::signExtend(offset & 0x3FU, 6));
        state.clockOut            = Fd;
        return (flags & 0xC0U) == 0 && (control & 0x02U) == 0 && (offset & 0xC0U) == 0;
    }

    /// The time, once a second: 10h..1Eh in one burst (the time, the alarm and timer
    /// registers nothing reads, the extension and the flag register), then the seconds again.
    struct Clock {
        static constexpr auto Period = std::chrono::milliseconds{1000};

        static constexpr std::array Steps{Step::read({.reg = 0x10, .count = 15, .offset = 0}),
                                          Step::read({.reg = 0x11, .count = 1, .offset = 15})};

        using Sample = Time;

        /// Rejected: a digit that is not BCD, a field out of range, a date that does not exist,
        /// a weekday that is not one-hot. Retried: the seconds changed between the burst and
        /// the re-read (4.12.2).
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            using Rv8803Detail::bcdOk;
            if(data.u8(15) != data.u8(1)) {
                return Outcome<Sample>::retry(std::chrono::milliseconds{1});
            }
            if(!bcdOk(data.u8(0), 0xFF) || !bcdOk(data.u8(1), 0x7F) || !bcdOk(data.u8(2), 0x7F)
               || !bcdOk(data.u8(3), 0x3F) || !bcdOk(data.u8(5), 0x3F) || !bcdOk(data.u8(6), 0x1F)
               || !bcdOk(data.u8(7), 0xFF))
            {
                return Outcome<Sample>::reject();
            }
            auto const                        weekday = Rv8803Detail::weekdayBit(data.u8(4));
            auto const                        cc      = data.bcd(0, 0xFF);
            auto const                        ss      = data.bcd(1, 0x7F);
            auto const                        mm      = data.bcd(2, 0x7F);
            auto const                        hh      = data.bcd(3, 0x3F);
            std::chrono::year_month_day const date{
              std::chrono::year{2000 + static_cast<int>(data.bcd(7, 0xFF))},
              std::chrono::month{data.bcd(6, 0x1F)},
              std::chrono::day{data.bcd(5, 0x3F)}};
            if(weekday > 6 || ss > 59 || mm > 59 || hh > 23 || !date.ok()) {
                return Outcome<Sample>::reject();
            }
            Sample t{};
            t.date           = date;
            t.weekday        = std::chrono::weekday{weekday};
            t.time           = Centiseconds{((hh * 60 + mm) * 60 + ss) * 100 + cc};
            auto const flags = data.u8(14);
            t.valid          = (flags & V2F) == 0;
            t.compensated    = (flags & V1F) == 0;
            return Outcome<Sample>::ok(t);
        }
    };

    /// Setting the clock (4.13). Transient: a reset must not put back whatever o'clock the
    /// application last wrote. The hundredths are not written -- setting RESET clears them
    /// (3.8) -- so `time` is taken to the whole second, and `valid` and `compensated` are
    /// ignored. A year outside 2000..2099 is written as 00 or 99.
    struct SetTime {
        using Value                            = Time;
        static constexpr std::size_t Bytes     = 7;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr std::array<Step,
                                                  4>
        encode(Value const&         t,
               std::span<std::byte> buffer) {
            auto const hms   = t.hms();
            auto const year  = static_cast<int>(t.date.year());
            auto const digit = year < 2000 ? 0 : (year > 2099 ? 99 : year - 2000);
            auto const put   = [&](std::size_t i, unsigned v) {
                buffer[i] = static_cast<std::byte>(toBcd(static_cast<std::uint8_t>(v)));
            };
            put(0, static_cast<unsigned>(hms.seconds().count()));
            put(1, static_cast<unsigned>(hms.minutes().count()));
            put(2, static_cast<unsigned>(hms.hours().count()));
            buffer[3] = static_cast<std::byte>(1U << (t.weekday.c_encoding() % 7U));
            put(4, static_cast<unsigned>(t.date.day()));
            put(5, static_cast<unsigned>(t.date.month()));
            put(6, static_cast<unsigned>(digit));
            return {
              Step::write({.reg = 0x1F, .payload = {static_cast<std::uint8_t>(Control | Reset)}}),
              Step::writeBuffer({.reg = 0x11, .offset = 0, .count = 7}),
              Step::write(
                {.reg     = 0x1E,
                 .payload = {ClearVoltageFlags}}),   // V2F and V1F: the time is valid again
              Step::write(
                {.reg = 0x1F, .payload = {Control}}),   // RESET released: the second starts now
            };
        }

        static constexpr void applied(Value const&,
                                      State& state) {
            state.timeLost            = false;
            state.compensationStopped = false;
            state.wasStopped          = false;
        }
    };

    /// OFFSET (2Ch, 3.9), the aging correction: steps of 0.23842 ppm, -32 .. +31, clamped.
    /// Positive values correct a clock that runs fast (5.4: OFFSET is the measured offset).
    /// No Initial: the part keeps the value it holds -- its registers survive on a backup
    /// supply -- until the application sets one, and a value set is put back after a later
    /// bring-up.
    struct Offset {
        using Value                        = std::int8_t;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr std::int8_t clamp(Value v) {
            return v < -32 ? std::int8_t{-32} : (v > 31 ? std::int8_t{31} : v);
        }

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(static_cast<std::uint8_t>(clamp(value)) & 0x3FU);
            return Step::writeBuffer({.reg = 0x2C, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.offset = clamp(value);
        }
    };

    /// FD (1Dh bits 3:2, 4.9): the CLKOUT frequency, `Fd` from every bring-up on. The rest of
    /// the extension register is written as its reset value 0 (TEST must be 0, 3.6).
    struct ClockOut {
        using Value                          = ClockOutRate;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Fd;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(static_cast<unsigned>(value) << 2U);
            return Step::writeBuffer({.reg = 0x1D, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.clockOut = value;
        }
    };

    /// Event control (2Fh, 3.10, 4.8.2): time stamping of an EVI event, its level and its
    /// debounce. ERST is written 0. No Initial: the reset value is capture off.
    struct EventControl {
        using Value                        = EventCapture;
        static constexpr std::size_t Bytes = 1;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0]
              = static_cast<std::byte>((value.capture == Rv8803Detail::Capture::on ? 0x80U : 0x00U)
                                       | (static_cast<unsigned>(value.level) << 6U)
                                       | (static_cast<unsigned>(value.filter) << 4U));
            return Step::writeBuffer({.reg = 0x2F, .offset = 0, .count = 1});
        }
    };

    /// The time stamp of the last EVI event (4.8), on request: `request<Event>()` from the INT
    /// edge, or on the application's own schedule. It reads EVF, clears it (writing 0 to EVF
    /// and 1 to the rest), then reads the capture registers.
    ///
    /// An event is reported when EVF was set, or when the capture registers moved since the
    /// previous stamp: an event landing between the flag read and the clear loses its flag,
    /// but its capture (taken whatever EVF is, 4.8.1 step 6) is read after the clear. Two
    /// events between two requests report the later one; two events at the same second and
    /// hundredth of different minutes, without EVF, look like one. Until the first event, a
    /// request with EVF clear is unchanged.
    struct Event {
        static constexpr std::array Steps{
          Step::read({.reg = 0x1E, .count = 1, .offset = 0}),
          Step::write({.reg = 0x1E, .payload = {ClearEvfOnly}}),
          Step::read({.reg = 0x20, .count = 2, .offset = 1}),
        };

        using Sample = Rv8803Detail::EventStamp;

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes         data,
                                                              Sample const& previous) {
            using Rv8803Detail::bcdOk;
            if(!bcdOk(data.u8(1), 0xFF) || !bcdOk(data.u8(2), 0x7F)) {
                return Outcome<Sample>::reject();
            }
            auto const cc = data.bcd(1, 0xFF);
            auto const ss = data.bcd(2, 0x7F);
            if(ss > 59) { return Outcome<Sample>::reject(); }
            Sample const now{Centiseconds{ss * 100 + cc}};
            bool const   flagged = (data.u8(0) & EVF) != 0;
            bool const   moved
              = previous.stamp >= Centiseconds::zero() && now.stamp != previous.stamp;
            if(flagged || moved) { return Outcome<Sample>::ok(now); }
            return Outcome<Sample>::unchanged();
        }
    };

    using Reads   = List<Clock, Event>;
    using Writes  = List<SetTime, Offset, ClockOut, EventControl>;
    using Primary = Clock;
};

}   // namespace Kvasir::I2C::Chips
