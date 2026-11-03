#pragma once

#include "Duration.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Periodic.hpp"

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
///   `I2C::ExpanderSegments` (I2C/Adapters/SegmentBackends.hpp) and `GpioSegments`
///   (SegmentDisplay/Gpio.hpp). `Display` ties a backend and a layout together and is what the
///   application talks to:
///
///     using Leds = Kvasir::I2C::Chips::Pca9956b<0x3F, Kvasir::Units::ohm(2200)>;
///     Kvasir::I2C::Device<I2c1, Clock, Leds> leds{};
///     Kvasir::I2C::Pca9956bDisplay<decltype(leds)> digits{leds, Kvasir::Units::milliAmp(5)};
///     ...
///     digits.setNumber(234, 1);        // "23.4"
///     digits.update(Clock::now());     // once per loop turn
///     leds.turn();                     // or the Bus's turn()
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
inline constexpr std::size_t segments = 8;

namespace Seg {
    inline constexpr Glyph a  = 1U << 0U;
    inline constexpr Glyph b  = 1U << 1U;
    inline constexpr Glyph c  = 1U << 2U;
    inline constexpr Glyph d  = 1U << 3U;
    inline constexpr Glyph e  = 1U << 4U;
    inline constexpr Glyph f  = 1U << 5U;
    inline constexpr Glyph g  = 1U << 6U;
    inline constexpr Glyph dp = 1U << 7U;
}   // namespace Seg

inline constexpr Glyph blank = 0;
inline constexpr Glyph minus = Seg::g;

/// 0..9, then A b C d E F, so an address or a register can go on the glass as hex.
inline constexpr std::array<Glyph, 16> hexDigits{
  Seg::a | Seg::b | Seg::c | Seg::d | Seg::e | Seg::f,            // 0
  Seg::b | Seg::c,                                                // 1
  Seg::a | Seg::b | Seg::g | Seg::e | Seg::d,                     // 2
  Seg::a | Seg::b | Seg::c | Seg::d | Seg::g,                     // 3
  Seg::f | Seg::g | Seg::b | Seg::c,                              // 4
  Seg::a | Seg::f | Seg::g | Seg::c | Seg::d,                     // 5
  Seg::a | Seg::f | Seg::g | Seg::e | Seg::c | Seg::d,            // 6
  Seg::a | Seg::b | Seg::c,                                       // 7
  Seg::a | Seg::b | Seg::c | Seg::d | Seg::e | Seg::f | Seg::g,   // 8
  Seg::a | Seg::b | Seg::c | Seg::d | Seg::f | Seg::g,            // 9
  Seg::a | Seg::b | Seg::c | Seg::e | Seg::f | Seg::g,            // A
  Seg::c | Seg::d | Seg::e | Seg::f | Seg::g,                     // b
  Seg::a | Seg::d | Seg::e | Seg::f,                              // C
  Seg::b | Seg::c | Seg::d | Seg::e | Seg::g,                     // d
  Seg::a | Seg::d | Seg::e | Seg::f | Seg::g,                     // E
  Seg::a | Seg::e | Seg::f | Seg::g,                              // F
};

/// A character as a glyph: the hex digits in either case, a few letters that read
/// unambiguously on seven segments (H L n o P r U, for "Err", "OFF", "Hi"), '-', '_' and
/// ' '. Anything else is blank.
[[nodiscard]] constexpr Glyph glyph(char c) {
    if(c >= '0' && c <= '9') { return hexDigits[static_cast<std::size_t>(c - '0')]; }
    if(c >= 'a' && c <= 'f') { return hexDigits[10 + static_cast<std::size_t>(c - 'a')]; }
    if(c >= 'A' && c <= 'F') { return hexDigits[10 + static_cast<std::size_t>(c - 'A')]; }
    switch(c) {
    case '-': return minus;
    case '_': return Seg::d;
    case 'H':
    case 'h': return Seg::b | Seg::c | Seg::e | Seg::f | Seg::g;
    case 'L':
    case 'l': return Seg::d | Seg::e | Seg::f;
    case 'n':
    case 'N': return Seg::c | Seg::e | Seg::g;
    case 'o':
    case 'O': return Seg::c | Seg::d | Seg::e | Seg::g;
    case 'P':
    case 'p': return Seg::a | Seg::b | Seg::e | Seg::f | Seg::g;
    case 'r':
    case 'R': return Seg::e | Seg::g;
    case 'U':
    case 'u': return Seg::b | Seg::c | Seg::d | Seg::e | Seg::f;
    default:  return blank;
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
    for(auto& g : d) { g = minus; }
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
        d[N - 1 - i] = hexDigits[v % 10U];
        v /= 10U;
    }
    if(negative) { d[N - 1 - used] = minus; }
    if(point != 0) { d[N - 1 - point] = static_cast<Glyph>(d[N - 1 - point] | Seg::dp); }
    return d;
}

/// The low `width` hex digits of `value`, right-aligned, the digits left of them dark.
template<std::size_t N>
[[nodiscard]] constexpr Digits<N> hex(std::uint32_t value,
                                      std::size_t   width = N) {
    Digits<N> d{};
    for(std::size_t i = 0; i < width && i < N; ++i) {
        d[N - 1 - i] = hexDigits[(value >> (4U * i)) & 0xFU];
    }
    return d;
}

static_assert(number<3>(1000) == dashes<3>(),
              "past three digits is three dashes");
static_assert(number<3>(5,
                        1)
                == Digits<3>{blank,
                             hexDigits[0] | Seg::dp,
                             hexDigits[5]},
              "\"0.5\": the zero before the point is drawn");
static_assert(number<3>(-12)
                == Digits<3>{minus,
                             hexDigits[1],
                             hexDigits[2]},
              "a minus in front");
static_assert(number<3>(-7)
                == Digits<3>{blank,
                             minus,
                             hexDigits[7]},
              "right against its digit");
static_assert(number<3>(-100) == dashes<3>(),
              "no room for the minus");
static_assert(number<3>(-100,
                        0,
                        Overflow::clamp)
                == Digits<3>{minus,
                             hexDigits[9],
                             hexDigits[9]},
              "clamped, -99");
static_assert(hex<3>(0x3F,
                     2)
                == Digits<3>{blank,
                             hexDigits[3],
                             hexDigits[15]},
              "two hex digits, right-aligned");

// -- layouts --------------------------------------------------------------------------------

namespace LayoutDetail {
    // Declared and never defined: reaching one while building or checking a layout is not a
    // constant expression, so the layout fails to compile and the diagnostic names the problem.
    void segmentDisplayLineIndexIs0To255();
}   // namespace LayoutDetail

/// One output line of a backend: a PCA9956B channel, an expander pin, an entry of a pin list.
/// Written as its number, `{2, 3, 5, ...}`; a Line left out of a list is `NotWired`, so the
/// pattern of a glass without decimal points can stop after g.
struct Line {
    std::uint8_t index{0xFF};

    constexpr Line() = default;

    template<std::integral T>
    constexpr Line(T line)   // implicit: `.pattern = {2, 3, 5}`
      : index{static_cast<std::uint8_t>(line)} {
        if(std::cmp_less(line, 0) || std::cmp_greater(line, 0xFF)) {
            LayoutDetail::segmentDisplayLineIndexIs0To255();
        }
    }

    friend constexpr bool operator==(Line,
                                     Line) = default;
};

/// A segment, or a whole line, the glass does not have.
inline constexpr Line notWired{0xFF};

/// Every digit on lines of its own, driven all the time: `segment[position][s]` is the line
/// for segment s (a..g, dp -- the Glyph bit order) of the digit at `position`, 0 the leftmost
/// as read.
template<std::size_t N>
struct StaticLayout {
    std::array<std::array<Line, segments>, N> segment;
    Active                                    active;
};

/// Eight segment lines shared by the digits, one select line a digit, scanned one digit at a
/// time: `select[position]` switches in the digit at `position`, 0 the leftmost as read.
template<std::size_t N>
struct MultiplexedLayout {
    std::array<Line, segments> segment;
    std::array<Line, N>        select;
    Active                     segmentActive;
    Active                     selectActive;
};

/// A regular board: every digit wired alike, `stride` lines a digit.
struct PerDigit {
    std::array<Line, segments> pattern;   ///< the line of segment s within one digit's lines
    std::uint8_t               stride = segments;
    FirstDigit                 first;
    Active                     active;
};

namespace LayoutDetail {
    void segmentDisplayLineIsPastTheLastOutputOfTheBackend();
    void segmentDisplayOneOutputLineIsWiredTwice();
    void segmentDisplayEveryDigitNeedsASelectLine();
    void segmentDisplayPerDigitLineDoesNotFitInAByte();

    template<typename L>
    struct Traits {
        static constexpr bool layout = false;
    };

    template<std::size_t N>
    struct Traits<StaticLayout<N>> {
        static constexpr bool        layout      = true;
        static constexpr bool        multiplexed = false;
        static constexpr std::size_t digits      = N;
    };

    template<std::size_t N>
    struct Traits<MultiplexedLayout<N>> {
        static constexpr bool        layout      = true;
        static constexpr bool        multiplexed = true;
        static constexpr std::size_t digits      = N;
    };

    /// Marks `line` used, stopping the build if it is past `Outputs` or already taken.
    template<std::size_t Outputs>
    constexpr void claim(std::bitset<Outputs>& used,
                         Line                  line) {
        if(line == notWired) { return; }
        if(line.index >= Outputs) { segmentDisplayLineIsPastTheLastOutputOfTheBackend(); }
        if(used.test(line.index)) { segmentDisplayOneOutputLineIsWiredTwice(); }
        used.set(line.index);
    }

    [[nodiscard]] constexpr Level level(bool   on,
                                        Active active) {
        return on == (active == Active::high) ? Level::high : Level::low;
    }
}   // namespace LayoutDetail

template<typename L>
concept Layout = LayoutDetail::Traits<std::remove_cvref_t<L>>::layout;

template<std::size_t N>
[[nodiscard]] constexpr StaticLayout<N> perDigit(PerDigit const& p) {
    StaticLayout<N> l{.segment = {}, .active = p.active};
    for(std::size_t slot = 0; slot < N; ++slot) {
        std::size_t const position = p.first == FirstDigit::right ? N - 1 - slot : slot;
        for(std::size_t s = 0; s < segments; ++s) {
            Line const        in    = p.pattern[s];
            std::size_t const index = (slot * p.stride) + in.index;
            if(in == notWired) {
                l.segment[position][s] = notWired;
                continue;
            }
            if(index >= notWired.index) {
                LayoutDetail::segmentDisplayPerDigitLineDoesNotFitInAByte();
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
        if(line == notWired) { LayoutDetail::segmentDisplayEveryDigitNeedsASelectLine(); }
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
        for(std::size_t s = 0; s < segments; ++s) {
            Line const line = l.segment[position][s];
            if(line == notWired) { continue; }
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
    Glyph const          g = position < N ? digits[position] : blank;
    for(std::size_t s = 0; s < segments; ++s) {
        Line const line = l.segment[s];
        if(line == notWired) { continue; }
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
/// `turn()`, and whatever else the Display forwards (`brightness`, `current`).
template<typename B>
concept Backend = requires(B& b, std::bitset<B::outputs> const& bits) {
    typename B::TimePoint;
    { B::outputs } -> std::convertible_to<std::size_t>;
    { B::scanning } -> std::convertible_to<Scan>;
    { b.show(bits) } -> std::same_as<bool>;
};

/// The scan rate of a multiplexed display. Another rate is a type with its own
/// `static constexpr std::chrono::microseconds digitPeriod`.
struct DefaultDisplayTiming {
    /// How long each digit is switched in: 2 ms, so four digits refresh at 125 Hz.
    static constexpr std::chrono::microseconds digitPeriod{2000};
};

/// N digits of a `LayoutV` on a `B`. The backend is built in place from the constructor's
/// arguments, so a display on a PCA9956B is `Display<...>{device, milliAmp(5)}`.
///
/// Everything is handed over in `update(now)`, once per loop turn: a static layout sends the
/// frame (and a backend skips what it already holds), a multiplexed layout switches to the next
/// digit every `Timing::digitPeriod`, first deselecting the last one so nothing ghosts.
template<Backend B, Layout auto LayoutV, typename Timing = DefaultDisplayTiming>
class Display {
    using Traits = LayoutDetail::Traits<std::remove_cvref_t<decltype(LayoutV)>>;

public:
    using BackendType = B;
    using TimePoint   = typename B::TimePoint;
    using Duration    = typename TimePoint::duration;
    using Glyph       = SegmentDisplay::Glyph;
    using Overflow    = SegmentDisplay::Overflow;

    static constexpr std::size_t count       = Traits::digits;
    static constexpr bool        multiplexed = Traits::multiplexed;
    static constexpr std::size_t outputs     = B::outputs;
    using Digits                             = SegmentDisplay::Digits<count>;
    using Bits                               = std::bitset<outputs>;

    static_assert(count >= 1,
                  "a display has at least one digit");
    static_assert(!multiplexed || B::scanning == Scan::supported,
                  "scan a multiplexed display from GPIO; over I2C use static wiring");

    /// The lines the glass uses; checking the layout against the backend on the way.
    static constexpr Bits used = usedLines<outputs>(LayoutV);
    static_assert(used.count() <= outputs,
                  "a display checks its layout whether or not its backend asks for the lines");

    static constexpr std::chrono::microseconds digitPeriod
      = Kvasir::asDuration<std::chrono::microseconds>(
        KVASIR_CONFIG_OR(Timing, digitPeriod, DefaultDisplayTiming::digitPeriod));

    /// How long a blinking display is lit, and dark, unless blink() is told otherwise.
    static constexpr auto blinkOn  = std::chrono::milliseconds{400};
    static constexpr auto blinkOff = std::chrono::milliseconds{200};

    template<typename... Args>
        requires std::constructible_from<B,
                                         Args&&...>
    explicit Display(Args&&... args) : backend_(std::forward<Args>(args)...) {
        if constexpr(requires { backend_.lines(used); }) { backend_.lines(used); }
        (void)backend_.show(dark_());
    }

    // -- what the glass shows ------------------------------------------------------------------

    void setDigits(Digits const& digits) { digits_ = digits; }

    /// Position 0 is the leftmost digit as read.
    void setDigit(std::size_t position,
                  Glyph       glyph) {
        if(position < count) { digits_[position] = glyph; }
    }

    /// `value` with `decimals` digits after the point (SegmentDisplay::number).
    void setNumber(std::int32_t value,
                   std::size_t  decimals = 0,
                   Overflow     overflow = Overflow::dashes) {
        digits_ = SegmentDisplay::number<count>(value, decimals, overflow);
    }

    /// The low `width` hex digits of `value`, right-aligned.
    void setHex(std::uint32_t value,
                std::size_t   width = count) {
        digits_ = SegmentDisplay::hex<count>(value, width);
    }

    void blank() { digits_ = Digits{}; }

    /// Every digit a dash: a reading that is not there.
    void dashes() { digits_ = SegmentDisplay::dashes<count>(); }

    [[nodiscard]] Digits const& digits() const { return digits_; }

    // -- how it is lit ---------------------------------------------------------------------------

    /// Blink the whole display, `on` lit and `off` dark. Turning it on starts with the dark
    /// half; turning it off lights the display at the next update().
    void blink(Blink                     mode,
               std::chrono::milliseconds on  = blinkOn,
               std::chrono::milliseconds off = blinkOff) {
        auto const enable = mode == Blink::on;
        if(enable && !blinking_) { toggle_.stop(); }   // toggles at the next update()
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
        } else if(!toggle_.armed(now)) {
            lit_ = !lit_;
            toggle_.restart(lit_ ? on_ : off_, now);
        }

        Digits const* shown = &digits_;
        if constexpr(requires { backend_.light(Light::lit); }) {
            backend_.light(lit_ ? Light::lit : Light::dark);
        } else {
            static constexpr Digits none{};
            if(!lit_) { shown = &none; }
        }

        bool changed = false;
        if constexpr(multiplexed) {
            if(scan_.armed(now)) { return false; }
            scan_.restart(std::chrono::duration_cast<Duration>(digitPeriod), now);
            position_ = (position_ + 1) % count;
            (void)backend_.show(scanFrame<outputs>(LayoutV, *shown, count));
            changed = backend_.show(scanFrame<outputs>(LayoutV, *shown, position_));
        } else {
            changed = backend_.show(frame<outputs>(LayoutV, *shown));
        }
        if(changed) { ++updates_; }
        return changed;
    }

    /// The backend's own turn (a device outside a Bus), where it has one.
    void turn()
        requires requires(B& b) { b.turn(); }
    {
        backend_.turn();
    }

    /// Frames that put something new on the lines since construction.
    [[nodiscard]] std::uint32_t updates() const { return updates_; }

    [[nodiscard]] B& backend() { return backend_; }

    [[nodiscard]] B const& backend() const { return backend_; }

private:
    [[nodiscard]] static constexpr Bits dark_() {
        if constexpr(multiplexed) {
            return scanFrame<outputs>(LayoutV, Digits{}, count);
        } else {
            return frame<outputs>(LayoutV, Digits{});
        }
    }

    B        backend_;
    Digits   digits_{};
    Duration on_{std::chrono::duration_cast<Duration>(blinkOn)};
    Duration off_{std::chrono::duration_cast<Duration>(blinkOff)};
    // Stopped = due at once: the next update() toggles / switches to the next digit.
    Kvasir::Deadline<typename TimePoint::clock> toggle_{};
    Kvasir::Deadline<typename TimePoint::clock> scan_{};
    std::size_t                                 position_{count - 1};
    std::uint32_t                               updates_{};
    bool                                        blinking_{};
    bool                                        lit_{true};
};

}   // namespace Kvasir::SegmentDisplay
