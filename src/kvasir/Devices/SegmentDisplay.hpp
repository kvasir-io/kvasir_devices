#pragma once

#include "Duration.hpp"

#include <array>
#include <bitset>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

/// Seven-segment digits on whatever drives them: a constant-current LED driver, an I2C port
/// expander, or the controller's own pins -- with the wiring, the number of digits and the
/// segments that are there at all fixed at compile time.
///
/// Three layers:
///
/// - **Glyphs**: a `Glyph` is the segments to light, a..g then the decimal point; `number()`,
///   `hex()` and `glyph()` make rows of them. Pure and constexpr.
/// - **Layouts**: where each segment of each digit is wired. A `StaticLayout<N>` gives every
///   (digit, segment) its own output line -- `perDigit()` builds the regular case, a board with
///   irregular wiring writes the table out -- and a `MultiplexedLayout<N>` shares eight segment
///   lines between the digits and selects one digit at a time. A segment that is not on the
///   glass (no decimal point, say) is `NotWired`. `frame()` and `scanFrame()` turn a row of
///   glyphs into the level of every output line.
/// - **Backends** put those levels on the hardware: `I2C::Pca9956bSegments`,
///   `I2C::ExpanderSegments` (I2C/SegmentBackends.hpp) and `GpioSegments`
///   (SegmentDisplay/Gpio.hpp). `Display` ties a backend and a layout together and is what the
///   application talks to:
///
///     using Leds = Kvasir::I2C::Chips::Pca9956b<0x3F, Kvasir::Units::ohm(2200)>;
///     Kvasir::I2C::Device<I2c1, Clock, Leds> leds{};
///     Kvasir::I2C::Pca9956bDisplay<decltype(leds)> digits{leds, Kvasir::Units::milliAmp(5)};
///     ...
///     digits.setNumber(234, 1);        // "23.4"
///     digits.update(Clock::now());     // once per loop turn
///     leds.handler();                  // or the Bus's handler()
///
/// Every check on a layout is at compile time: a line past the backend's outputs, one output
/// wired to two segments, or a multiplexed layout on a backend that cannot scan does not build.
namespace Kvasir::SegmentDisplay {

/// Whether the whole display blinks.
enum class Blink : std::uint8_t { off, on };

/// The level a lit segment, or a selected digit, is driven to: high for a common-cathode glass
/// on a source driver, low for common anode or an open-drain sink like the PCF8574.
enum class Active : std::uint8_t { low, high };

/// The electrical level of one output line.
enum class Level : std::uint8_t { low, high };

/// Which digit, as read, is the first slot of a `perDigit()` layout.
enum class FirstDigit : std::uint8_t {
    right,   ///< the rightmost: slot 0 is the last digit read
    left,    ///< the leftmost
};

/// Whether a backend can switch digits fast enough to scan a multiplexed glass.
enum class Scan : std::uint8_t { unsupported, supported };

/// Whether the display is lit at the moment: what a backend with its own dimming (PWM) blinks
/// with instead of a blank frame.
enum class Light : std::uint8_t { dark, lit };

/// The segments to light: bit 0 is a, then b..g, bit 7 the decimal point.
///
///        aaaa
///       f    b
///       f    b
///        gggg
///       e    c
///       e    c
///        dddd  dp
using Glyph = std::uint8_t;

/// Segments a digit has room for: a..g and the point.
inline constexpr std::size_t Segments = 8;

namespace Seg {
    inline constexpr Glyph A  = 1U << 0U;
    inline constexpr Glyph B  = 1U << 1U;
    inline constexpr Glyph C  = 1U << 2U;
    inline constexpr Glyph D  = 1U << 3U;
    inline constexpr Glyph E  = 1U << 4U;
    inline constexpr Glyph F  = 1U << 5U;
    inline constexpr Glyph G  = 1U << 6U;
    inline constexpr Glyph Dp = 1U << 7U;
}   // namespace Seg

inline constexpr Glyph Blank = 0;
inline constexpr Glyph Minus = Seg::G;

/// 0..9, then A b C d E F, so an address or a register can go on the glass as hex.
inline constexpr std::array<Glyph, 16> HexDigits{
  Seg::A | Seg::B | Seg::C | Seg::D | Seg::E | Seg::F,            // 0
  Seg::B | Seg::C,                                                // 1
  Seg::A | Seg::B | Seg::G | Seg::E | Seg::D,                     // 2
  Seg::A | Seg::B | Seg::C | Seg::D | Seg::G,                     // 3
  Seg::F | Seg::G | Seg::B | Seg::C,                              // 4
  Seg::A | Seg::F | Seg::G | Seg::C | Seg::D,                     // 5
  Seg::A | Seg::F | Seg::G | Seg::E | Seg::C | Seg::D,            // 6
  Seg::A | Seg::B | Seg::C,                                       // 7
  Seg::A | Seg::B | Seg::C | Seg::D | Seg::E | Seg::F | Seg::G,   // 8
  Seg::A | Seg::B | Seg::C | Seg::D | Seg::F | Seg::G,            // 9
  Seg::A | Seg::B | Seg::C | Seg::E | Seg::F | Seg::G,            // A
  Seg::C | Seg::D | Seg::E | Seg::F | Seg::G,                     // b
  Seg::A | Seg::D | Seg::E | Seg::F,                              // C
  Seg::B | Seg::C | Seg::D | Seg::E | Seg::G,                     // d
  Seg::A | Seg::D | Seg::E | Seg::F | Seg::G,                     // E
  Seg::A | Seg::E | Seg::F | Seg::G,                              // F
};

/// A character as a glyph: the hex digits in either case, a few letters that read
/// unambiguously on seven segments (H L n o P r U, for "Err", "OFF", "Hi"), '-', '_' and
/// ' '. Anything else is blank.
[[nodiscard]] constexpr Glyph glyph(char c) {
    if(c >= '0' && c <= '9') { return HexDigits[static_cast<std::size_t>(c - '0')]; }
    if(c >= 'a' && c <= 'f') { return HexDigits[10 + static_cast<std::size_t>(c - 'a')]; }
    if(c >= 'A' && c <= 'F') { return HexDigits[10 + static_cast<std::size_t>(c - 'A')]; }
    switch(c) {
    case '-': return Minus;
    case '_': return Seg::D;
    case 'H':
    case 'h': return Seg::B | Seg::C | Seg::E | Seg::F | Seg::G;
    case 'L':
    case 'l': return Seg::D | Seg::E | Seg::F;
    case 'n':
    case 'N': return Seg::C | Seg::E | Seg::G;
    case 'o':
    case 'O': return Seg::C | Seg::D | Seg::E | Seg::G;
    case 'P':
    case 'p': return Seg::A | Seg::B | Seg::E | Seg::F | Seg::G;
    case 'r':
    case 'R': return Seg::E | Seg::G;
    case 'U':
    case 'u': return Seg::B | Seg::C | Seg::D | Seg::E | Seg::F;
    default:  return Blank;
    }
}

template<std::size_t N>
using Digits = std::array<Glyph, N>;

/// What `number()` does with a value that does not fit.
enum class Overflow : std::uint8_t {
    dashes,   ///< every digit a dash: the number exists and does not fit
    clamp,    ///< the nearest value that does
};

template<std::size_t N>
[[nodiscard]] constexpr Digits<N> dashes() {
    Digits<N> d{};
    for(auto& g : d) { g = Minus; }
    return d;
}

/// `value` right-aligned over N digits, with `decimals` digits after the point (0: no point)
/// and the leading zeros dark -- except the ones up to the point, so 5 with one decimal is
/// "0.5". A negative value takes a minus in front of its first digit.
template<std::size_t N>
[[nodiscard]] constexpr Digits<N> number(std::int32_t value,
                                         std::size_t  decimals = 0,
                                         Overflow     overflow = Overflow::dashes) {
    std::size_t const point    = decimals < N ? decimals : 0;
    bool const        negative = value < 0;
    std::uint32_t     v        = negative ? static_cast<std::uint32_t>(-(value + 1)) + 1U
                                          : static_cast<std::uint32_t>(value);

    // Digits the value needs, and at least one more than the point.
    auto const width = [&](std::uint32_t x) {
        std::size_t n = 1;
        while(x >= 10U) {
            x /= 10U;
            ++n;
        }
        return n > point ? n : point + 1;
    };
    std::size_t const room = negative ? N - 1 : N;
    if(width(v) > room) {
        if(overflow == Overflow::dashes || room < point + 1) { return dashes<N>(); }
        v = 0;
        for(std::size_t i = 0; i < room; ++i) { v = (v * 10U) + 9U; }
    }

    Digits<N>         d{};
    std::size_t const used = width(v);
    for(std::size_t i = 0; i < used; ++i) {   // i counts from the right
        d[N - 1 - i] = HexDigits[v % 10U];
        v /= 10U;
    }
    if(negative) { d[N - 1 - used] = Minus; }
    if(point != 0) { d[N - 1 - point] = static_cast<Glyph>(d[N - 1 - point] | Seg::Dp); }
    return d;
}

/// The low `width` hex digits of `value`, right-aligned, the digits left of them dark.
template<std::size_t N>
[[nodiscard]] constexpr Digits<N> hex(std::uint32_t value,
                                      std::size_t   width = N) {
    Digits<N> d{};
    for(std::size_t i = 0; i < width && i < N; ++i) {
        d[N - 1 - i] = HexDigits[(value >> (4U * i)) & 0xFU];
    }
    return d;
}

static_assert(number<3>(1000) == dashes<3>(),
              "past three digits is three dashes");
static_assert(number<3>(5,
                        1)
                == Digits<3>{Blank,
                             HexDigits[0] | Seg::Dp,
                             HexDigits[5]},
              "\"0.5\": the zero before the point is drawn");
static_assert(number<3>(-12)
                == Digits<3>{Minus,
                             HexDigits[1],
                             HexDigits[2]},
              "a minus in front");
static_assert(number<3>(-7)
                == Digits<3>{Blank,
                             Minus,
                             HexDigits[7]},
              "right against its digit");
static_assert(number<3>(-100) == dashes<3>(),
              "no room for the minus");
static_assert(number<3>(-100,
                        0,
                        Overflow::clamp)
                == Digits<3>{Minus,
                             HexDigits[9],
                             HexDigits[9]},
              "clamped, -99");
static_assert(hex<3>(0x3F,
                     2)
                == Digits<3>{Blank,
                             HexDigits[3],
                             HexDigits[15]},
              "two hex digits, right-aligned");

// -- layouts --------------------------------------------------------------------------------

namespace LayoutDetail {
    // Declared and never defined: reaching one while building or checking a layout is not a
    // constant expression, so the layout fails to compile and the diagnostic names the problem.
    void segment_display_line_index_is_0_to_255();
}   // namespace LayoutDetail

/// One output line of a backend: a PCA9956B channel, an expander pin, an entry of a pin list.
/// Written as its number, `{2, 3, 5, ...}`; a Line left out of a list is `NotWired`, so the
/// pattern of a glass without decimal points can stop after g.
struct Line {
    std::uint8_t index{0xFF};

    constexpr Line() = default;

    template<std::integral T>
    constexpr Line(T line)   // NOLINT(google-explicit-constructor): `.pattern = {2, 3, 5}`
      : index{static_cast<std::uint8_t>(line)} {
        if(std::cmp_less(line, 0) || std::cmp_greater(line, 0xFF)) {
            LayoutDetail::segment_display_line_index_is_0_to_255();
        }
    }

    friend constexpr bool operator==(Line,
                                     Line) = default;
};

/// A segment, or a whole line, the glass does not have.
inline constexpr Line NotWired{0xFF};

/// Every digit on lines of its own, driven all the time: `segment[position][s]` is the line
/// for segment s (a..g, dp -- the Glyph bit order) of the digit at `position`, 0 the leftmost
/// as read.
template<std::size_t N>
struct StaticLayout {
    std::array<std::array<Line, Segments>, N> segment;
    Active                                    active;
};

/// Eight segment lines shared by the digits, one select line a digit, scanned one digit at a
/// time: `select[position]` switches in the digit at `position`, 0 the leftmost as read.
template<std::size_t N>
struct MultiplexedLayout {
    std::array<Line, Segments> segment;
    std::array<Line, N>        select;
    Active                     segmentActive;
    Active                     selectActive;
};

/// A regular board: every digit wired alike, `stride` lines a digit.
struct PerDigit {
    std::array<Line, Segments> pattern;   ///< the line of segment s within one digit's lines
    std::uint8_t               stride = Segments;
    FirstDigit                 first;
    Active                     active;
};

namespace LayoutDetail {
    void segment_display_line_is_past_the_last_output_of_the_backend();
    void segment_display_one_output_line_is_wired_twice();
    void segment_display_every_digit_needs_a_select_line();
    void segment_display_perDigit_line_does_not_fit_in_a_byte();

    template<typename L>
    struct Traits {
        static constexpr bool Layout = false;
    };

    template<std::size_t N>
    struct Traits<StaticLayout<N>> {
        static constexpr bool        Layout      = true;
        static constexpr bool        Multiplexed = false;
        static constexpr std::size_t Digits      = N;
    };

    template<std::size_t N>
    struct Traits<MultiplexedLayout<N>> {
        static constexpr bool        Layout      = true;
        static constexpr bool        Multiplexed = true;
        static constexpr std::size_t Digits      = N;
    };

    /// Marks `line` used, stopping the build if it is past `Outputs` or already taken.
    template<std::size_t Outputs>
    constexpr void claim(std::bitset<Outputs>& used,
                         Line                  line) {
        if(line == NotWired) { return; }
        if(line.index >= Outputs) { segment_display_line_is_past_the_last_output_of_the_backend(); }
        if(used.test(line.index)) { segment_display_one_output_line_is_wired_twice(); }
        used.set(line.index);
    }

    [[nodiscard]] constexpr Level level(bool   on,
                                        Active active) {
        return on == (active == Active::high) ? Level::high : Level::low;
    }
}   // namespace LayoutDetail

template<typename L>
concept Layout = LayoutDetail::Traits<std::remove_cvref_t<L>>::Layout;

template<std::size_t N>
[[nodiscard]] constexpr StaticLayout<N> perDigit(PerDigit const& p) {
    StaticLayout<N> l{.segment = {}, .active = p.active};
    for(std::size_t slot = 0; slot < N; ++slot) {
        std::size_t const position = p.first == FirstDigit::right ? N - 1 - slot : slot;
        for(std::size_t s = 0; s < Segments; ++s) {
            Line const  in    = p.pattern[s];
            std::size_t index = (slot * p.stride) + in.index;
            if(in == NotWired) {
                l.segment[position][s] = NotWired;
                continue;
            }
            if(index >= NotWired.index) {
                LayoutDetail::segment_display_perDigit_line_does_not_fit_in_a_byte();
            }
            l.segment[position][s] = Line{index};
        }
    }
    return l;
}

/// The lines a layout drives. Checks it on the way: every line below `Outputs`, none twice,
/// every digit of a multiplexed layout with its select line.
template<std::size_t Outputs,
         std::size_t N>
[[nodiscard]] constexpr std::bitset<Outputs> usedLines(StaticLayout<N> const& l) {
    std::bitset<Outputs> used{};
    for(auto const& digit : l.segment) {
        for(auto const line : digit) { LayoutDetail::claim(used, line); }
    }
    return used;
}

template<std::size_t Outputs,
         std::size_t N>
[[nodiscard]] constexpr std::bitset<Outputs> usedLines(MultiplexedLayout<N> const& l) {
    std::bitset<Outputs> used{};
    for(auto const line : l.segment) { LayoutDetail::claim(used, line); }
    for(auto const line : l.select) {
        if(line == NotWired) { LayoutDetail::segment_display_every_digit_needs_a_select_line(); }
        LayoutDetail::claim(used, line);
    }
    return used;
}

/// The level of every output line for `digits` on a static layout: a set bit is a high line,
/// so a dark segment on an active-low line is a 1. Lines the layout does not use are 0.
template<std::size_t Outputs,
         std::size_t N>
[[nodiscard]] constexpr std::bitset<Outputs> frame(StaticLayout<N> const& l,
                                                   Digits<N> const&       digits) {
    std::bitset<Outputs> out{};
    for(std::size_t position = 0; position < N; ++position) {
        for(std::size_t s = 0; s < Segments; ++s) {
            Line const line = l.segment[position][s];
            if(line == NotWired) { continue; }
            bool const lit = ((digits[position] >> s) & 1U) != 0;
            out.set(line.index, LayoutDetail::level(lit, l.active) == Level::high);
        }
    }
    return out;
}

/// The level of every output line with the digit at `position` switched in and showing its
/// glyph; a `position` of N or more selects no digit and leaves every segment dark.
template<std::size_t Outputs,
         std::size_t N>
[[nodiscard]] constexpr std::bitset<Outputs> scanFrame(MultiplexedLayout<N> const& l,
                                                       Digits<N> const&            digits,
                                                       std::size_t                 position) {
    std::bitset<Outputs> out{};
    Glyph const          g = position < N ? digits[position] : Blank;
    for(std::size_t s = 0; s < Segments; ++s) {
        Line const line = l.segment[s];
        if(line == NotWired) { continue; }
        bool const lit = ((g >> s) & 1U) != 0;
        out.set(line.index, LayoutDetail::level(lit, l.segmentActive) == Level::high);
    }
    for(std::size_t p = 0; p < N; ++p) {
        out.set(l.select[p].index,
                LayoutDetail::level(p == position, l.selectActive) == Level::high);
    }
    return out;
}

// -- backends and the display ---------------------------------------------------------------

/// What `Display` needs of a backend: how many output lines it has, whether it can scan, and a
/// `show()` that puts a frame on them and says whether anything new went out. Optional, found
/// by `requires`: `lines(used)` (told once which lines the glass uses), `update(now)` (its own
/// housekeeping each loop turn), `light(Light)` (blink by dimming rather than a blank frame),
/// `handler()`, and whatever else the Display forwards (`brightness`, `current`).
template<typename B>
concept Backend = requires(B& b, std::bitset<B::Outputs> const& bits) {
    typename B::TimePoint;
    { B::Outputs } -> std::convertible_to<std::size_t>;
    { B::Scanning } -> std::convertible_to<Scan>;
    { b.show(bits) } -> std::same_as<bool>;
};

/// The scan rate of a multiplexed display. Another rate is a type with its own
/// `static constexpr std::chrono::microseconds DigitPeriod`.
struct DefaultDisplayTiming {
    /// How long each digit is switched in: 2 ms, so four digits refresh at 125 Hz.
    static constexpr std::chrono::microseconds DigitPeriod{2000};
};

/// N digits of a `LayoutV` on a `B`. The backend is built in place from the constructor's
/// arguments, so a display on a PCA9956B is `Display<...>{device, milliAmp(5)}`.
///
/// Everything is handed over in `update(now)`, once per loop turn: a static layout sends the
/// frame (and a backend skips what it already holds), a multiplexed layout switches to the next
/// digit every `Timing::DigitPeriod`, first deselecting the last one so nothing ghosts.
template<Backend B, Layout auto LayoutV, typename Timing = DefaultDisplayTiming>
class Display {
    using Traits = LayoutDetail::Traits<std::remove_cvref_t<decltype(LayoutV)>>;

public:
    using BackendType = B;
    using TimePoint   = typename B::TimePoint;
    using Duration    = typename TimePoint::duration;
    using Glyph       = SegmentDisplay::Glyph;
    using Overflow    = SegmentDisplay::Overflow;

    static constexpr std::size_t Count       = Traits::Digits;
    static constexpr bool        Multiplexed = Traits::Multiplexed;
    static constexpr std::size_t Outputs     = B::Outputs;
    using Digits                             = SegmentDisplay::Digits<Count>;
    using Bits                               = std::bitset<Outputs>;

    static_assert(Count >= 1,
                  "a display has at least one digit");
    static_assert(!Multiplexed || B::Scanning == Scan::supported,
                  "scan a multiplexed display from GPIO; over I2C use static wiring");

    /// The lines the glass uses; checking the layout against the backend on the way.
    static constexpr Bits Used = usedLines<Outputs>(LayoutV);
    static_assert(Used.count() <= Outputs,
                  "a display checks its layout whether or not its backend asks for the lines");

    static constexpr std::chrono::microseconds DigitPeriod = [] {
        if constexpr(requires { Timing::DigitPeriod; }) {
            return Kvasir::asDuration<std::chrono::microseconds>(Timing::DigitPeriod);
        } else {
            return DefaultDisplayTiming::DigitPeriod;
        }
    }();

    /// How long a blinking display is lit, and dark, unless blink() is told otherwise.
    static constexpr auto BlinkOn  = std::chrono::milliseconds{400};
    static constexpr auto BlinkOff = std::chrono::milliseconds{200};

    template<typename... Args>
        requires std::constructible_from<B,
                                         Args&&...>
    explicit Display(Args&&... args) : backend_(std::forward<Args>(args)...) {
        if constexpr(requires { backend_.lines(Used); }) { backend_.lines(Used); }
        (void)backend_.show(dark_());
    }

    // -- what the glass shows ------------------------------------------------------------------

    void setDigits(Digits const& digits) { digits_ = digits; }

    /// Position 0 is the leftmost digit as read.
    void setDigit(std::size_t position,
                  Glyph       glyph) {
        if(position < Count) { digits_[position] = glyph; }
    }

    /// `value` with `decimals` digits after the point (SegmentDisplay::number).
    void setNumber(std::int32_t value,
                   std::size_t  decimals = 0,
                   Overflow     overflow = Overflow::dashes) {
        digits_ = SegmentDisplay::number<Count>(value, decimals, overflow);
    }

    /// The low `width` hex digits of `value`, right-aligned.
    void setHex(std::uint32_t value,
                std::size_t   width = Count) {
        digits_ = SegmentDisplay::hex<Count>(value, width);
    }

    void blank() { digits_ = Digits{}; }

    /// Every digit a dash: a reading that is not there.
    void dashes() { digits_ = SegmentDisplay::dashes<Count>(); }

    [[nodiscard]] Digits const& digits() const { return digits_; }

    // -- how it is lit ---------------------------------------------------------------------------

    /// Blink the whole display, `on` lit and `off` dark. Turning it on starts with the dark
    /// half; turning it off lights the display at the next update().
    void blink(Blink                     mode,
               std::chrono::milliseconds on  = BlinkOn,
               std::chrono::milliseconds off = BlinkOff) {
        auto const enable = mode == Blink::on;
        if(enable && !blinking_) { toggleAt_ = TimePoint{}; }
        blinking_ = enable;
        on_       = std::chrono::duration_cast<Duration>(on);
        off_      = std::chrono::duration_cast<Duration>(off);
    }

    [[nodiscard]] bool blinking() const { return blinking_; }

    /// The backend's brightness, where it has one (the PCA9956B's PWMALL duty while lit).
    template<typename T>
        requires requires(B& b,
                          T  v) { b.brightness(v); }
    void brightness(T value) {
        backend_.brightness(value);
    }

    [[nodiscard]] auto brightness() const
        requires requires(B const& b) { b.brightness(); }
    {
        return backend_.brightness();
    }

    /// The backend's segment current, where it has one (the PCA9956B's IREFALL).
    template<typename T>
        requires requires(B& b,
                          T  v) { b.current(v); }
    void current(T value) {
        backend_.current(value);
    }

    [[nodiscard]] auto current() const
        requires requires(B const& b) { b.current(); }
    {
        return backend_.current();
    }

    // -- the loop ---------------------------------------------------------------------------------

    /// Hand the state over; true when a new frame went to the backend.
    bool update(TimePoint now) {
        if constexpr(requires { backend_.update(now); }) { backend_.update(now); }

        if(!blinking_) {
            lit_ = true;
        } else if(now >= toggleAt_) {
            lit_      = !lit_;
            toggleAt_ = now + (lit_ ? on_ : off_);
        }

        Digits const* shown = &digits_;
        if constexpr(requires { backend_.light(Light::lit); }) {
            backend_.light(lit_ ? Light::lit : Light::dark);
        } else {
            static constexpr Digits None{};
            if(!lit_) { shown = &None; }
        }

        bool changed = false;
        if constexpr(Multiplexed) {
            if(now < scanAt_) { return false; }
            scanAt_   = now + std::chrono::duration_cast<Duration>(DigitPeriod);
            position_ = (position_ + 1) % Count;
            (void)backend_.show(scanFrame<Outputs>(LayoutV, *shown, Count));
            changed = backend_.show(scanFrame<Outputs>(LayoutV, *shown, position_));
        } else {
            changed = backend_.show(frame<Outputs>(LayoutV, *shown));
        }
        if(changed) { ++updates_; }
        return changed;
    }

    /// The backend's own handler (a device outside a Bus), where it has one.
    void handler()
        requires requires(B& b) { b.handler(); }
    {
        backend_.handler();
    }

    /// Frames that put something new on the lines since construction.
    [[nodiscard]] std::uint32_t updates() const { return updates_; }

    [[nodiscard]] B& backend() { return backend_; }

    [[nodiscard]] B const& backend() const { return backend_; }

private:
    [[nodiscard]] static constexpr Bits dark_() {
        if constexpr(Multiplexed) {
            return scanFrame<Outputs>(LayoutV, Digits{}, Count);
        } else {
            return frame<Outputs>(LayoutV, Digits{});
        }
    }

    B             backend_;
    Digits        digits_{};
    Duration      on_{std::chrono::duration_cast<Duration>(BlinkOn)};
    Duration      off_{std::chrono::duration_cast<Duration>(BlinkOff)};
    TimePoint     toggleAt_{};
    TimePoint     scanAt_{};
    std::size_t   position_{Count - 1};
    std::uint32_t updates_{};
    bool          blinking_{};
    bool          lit_{true};
};

}   // namespace Kvasir::SegmentDisplay
