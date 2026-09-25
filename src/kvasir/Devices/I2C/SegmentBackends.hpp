#pragma once

#include "../SegmentDisplay.hpp"
#include "Quantities.hpp"

#include <array>
#include <bitset>
#include <chrono>
#include <cstddef>
#include <cstdint>

/// SegmentDisplay backends on an I2C part (SegmentDisplay.hpp has the layers): a PCA9956B
/// constant-current driver, or any of the port expanders -- PCF8574/PCF8575, MCP23017,
/// TCA9555, PCA9557. Both hold the device by reference, as MonoPanelRef does, so the device may
/// be a local, a member, or inside a Bus, and both drive statically: an I2C transaction a digit
/// is far too slow to scan a multiplexed glass without flicker.
namespace Kvasir::I2C {

/// Output lines on a PCA9956B's channels: LEDOUT for the frame (0b11, PWM with group dimming,
/// for a lit line, 0b00 for a dark one), PWMALL for the brightness and the blink, IREFALL for
/// the current. `set()` skips a value the part already holds, so an unchanged display costs
/// nothing on the bus, and the engine replays all three after a reset of the part by itself.
///
/// The current and brightness are set from the constructor, so the first bring-up writes them in
/// place of the description's Initial zeros instead of after them. What the engine does not
/// replay is CLRERR: the part comes up with every channel in PWM mode and IREF at zero -- an
/// output that is on and passes no current, an open circuit to the detector -- so MODE2's ERROR
/// is latched before the application has said anything. With `clearErrorsAfter` non-zero, one
/// CLRERR goes out that long after each bring-up (`broughtUp(seen)`), once the outputs are real,
/// and from then on the flag means an actual open or shorted segment.
template<typename DeviceT>
class Pca9956bSegments {
public:
    using Device    = DeviceT;
    using Chip      = typename Device::Chip;
    using TimePoint = typename Device::TimePoint;
    using Duration  = typename Device::Duration;
    using LedOut    = typename Chip::LedOut::Value;

    static constexpr std::size_t          Outputs  = Chip::Channels;
    static constexpr SegmentDisplay::Scan Scanning = SegmentDisplay::Scan::unsupported;

    using Bits = std::bitset<Outputs>;

    /// `current` per segment (IREFALL), `brightness` the PWMALL duty while lit, and the CLRERR
    /// after each bring-up (zero: never).
    Pca9956bSegments(Device&                   device,
                     MilliAmp                  current,
                     std::uint8_t              brightness       = 255,
                     std::chrono::milliseconds clearErrorsAfter = std::chrono::milliseconds::zero())
      : device_{&device}
      , current_{current}
      , brightness_{brightness}
      , clearErrorsAfter_{std::chrono::duration_cast<Duration>(clearErrorsAfter)} {
        device_->template set<typename Chip::IrefAll>(Chip::iref(current_));
        device_->template set<typename Chip::PwmAll>(brightness_);
    }

    /// A frame as the six LEDOUT bytes: two bits a channel, LED0 in the low bits of LEDOUT0.
    [[nodiscard]] static constexpr LedOut ledOut(Bits const& levels) {
        LedOut out{};
        for(std::size_t ch = 0; ch < Outputs; ++ch) {
            if(!levels.test(ch)) { continue; }
            auto& byte = out[ch / 4];
            byte       = static_cast<std::uint8_t>(byte | (0b11U << (2U * (ch % 4U))));
        }
        return out;
    }

    bool show(Bits const& levels) {
        return device_->template set<typename Chip::LedOut>(ledOut(levels));
    }

    /// PWMALL: the brightness while lit, zero while dark.
    void light(SegmentDisplay::Light l) {
        device_->template set<typename Chip::PwmAll>(
          l == SegmentDisplay::Light::lit ? brightness_ : std::uint8_t{0});
    }

    /// The CLRERR after a bring-up.
    void update(TimePoint now) {
        if(device_->broughtUp(bringUp_)) {
            clearPending_ = clearErrorsAfter_ != Duration::zero();
            clearAt_      = now + clearErrorsAfter_;
        }
        if(clearPending_ && device_->answering() && now >= clearAt_) {
            device_->template set<typename Chip::Mode2>(Chip::ClrErr);   // Mode2 always writes
            clearPending_ = false;
        }
    }

    /// The segment current, IREFALL: sent once, and only when it changed.
    void current(MilliAmp value) {
        current_ = value;
        device_->template set<typename Chip::IrefAll>(Chip::iref(current_));
    }

    [[nodiscard]] MilliAmp current() const { return current_; }

    /// The PWMALL duty while lit, from the next update() on.
    void brightness(std::uint8_t duty) { brightness_ = duty; }

    [[nodiscard]] std::uint8_t brightness() const { return brightness_; }

    /// The device's own handler, for a device outside a Bus.
    void handler() { device_->handler(); }

    [[nodiscard]] Device& device() const { return *device_; }

private:
    Device*       device_;
    MilliAmp      current_;
    std::uint8_t  brightness_;
    Duration      clearErrorsAfter_;
    TimePoint     clearAt_{};
    std::uint16_t bringUp_{};   ///< the bring-up the CLRERR was armed for
    bool          clearPending_{};
};

namespace SegmentBackendsDetail {
    /// An expander whose port is written as it is: PCF8574/PCF8575, open-drain with weak
    /// pull-ups, no direction register.
    template<typename Chip>
    concept PortExpander = requires { typename Chip::Port::Value; };

    /// An expander with an output latch and a direction register (1 = input): MCP23017,
    /// TCA9555, PCA9557.
    template<typename Chip>
    concept LatchExpander = requires {
        typename Chip::Output::Value;
        typename Chip::Direction::Value;
    };

    template<typename Chip>
    struct OutputOf {
        using type = typename Chip::Output;
    };

    template<PortExpander Chip>
    struct OutputOf<Chip> {
        using type = typename Chip::Port;
    };
}   // namespace SegmentBackendsDetail

/// Output lines on an I2C port expander's pins, line n the expander's pin n. The display owns
/// the output register (and, on a part that has one, the direction register): the lines the
/// glass uses become outputs, every other pin keeps the register's Initial value -- high on a
/// PCF8574, which is its input state; an input on the others.
template<typename DeviceT>
    requires SegmentBackendsDetail::PortExpander<typename DeviceT::Chip>
          || SegmentBackendsDetail::LatchExpander<typename DeviceT::Chip>
class ExpanderSegments {
public:
    using Device    = DeviceT;
    using Chip      = typename Device::Chip;
    using TimePoint = typename Device::TimePoint;
    using Output    = typename SegmentBackendsDetail::OutputOf<Chip>::type;
    using Value     = typename Output::Value;

    static constexpr std::size_t          Outputs  = 8 * sizeof(Value);
    static constexpr SegmentDisplay::Scan Scanning = SegmentDisplay::Scan::unsupported;

    using Bits = std::bitset<Outputs>;

    explicit ExpanderSegments(Device& device) : device_{&device} {}

    /// The lines the glass uses: outputs from here on.
    void lines(Bits const& used) {
        used_ = valueOf(used);
        if constexpr(SegmentBackendsDetail::LatchExpander<Chip>) {
            using Direction = typename Chip::Direction;
            device_->template set<Direction>(
              static_cast<typename Direction::Value>(Direction::Initial & ~used_));
        }
    }

    bool show(Bits const& levels) {
        auto const idle = static_cast<Value>(Output::Initial & static_cast<Value>(~used_));
        return device_->template set<Output>(static_cast<Value>((valueOf(levels) & used_) | idle));
    }

    /// The device's own handler, for a device outside a Bus.
    void handler() { device_->handler(); }

    [[nodiscard]] Device& device() const { return *device_; }

private:
    [[nodiscard]] static constexpr Value valueOf(Bits const& bits) {
        return static_cast<Value>(bits.to_ulong());
    }

    Device* device_;
    Value   used_{};
};

/// A common PCA9956B seven-segment board wiring:
/// three digits of eight channels, channel 0 of a digit the middle bar, then round the outside,
/// the decimal point on channel 4; the rightmost digit on channels 0..7.
///
///     channel 0  g      channel 4  dp
///             1  f              5  c
///             2  a              6  d
///             3  b              7  e
inline constexpr auto Pca9956bBoard = SegmentDisplay::perDigit<3>({
  .pattern = {2, 3, 5, 6, 7, 1, 0, 4},
  .stride  = 8,
  .first   = SegmentDisplay::FirstDigit::right,
  .active  = SegmentDisplay::Active::high,
});

/// Digits on a PCA9956B, the board wiring above unless told otherwise:
/// `Pca9956bDisplay<decltype(leds)> digits{leds, Units::milliAmp(5)}`.
template<typename DeviceT,
         SegmentDisplay::Layout auto Layout = Pca9956bBoard,
         typename Timing                    = SegmentDisplay::DefaultDisplayTiming>
using Pca9956bDisplay = SegmentDisplay::Display<Pca9956bSegments<DeviceT>, Layout, Timing>;

/// Digits on a port expander: `ExpanderDisplay<decltype(port), Layout> digits{port}`.
template<typename DeviceT,
         SegmentDisplay::Layout auto Layout,
         typename Timing = SegmentDisplay::DefaultDisplayTiming>
using ExpanderDisplay = SegmentDisplay::Display<ExpanderSegments<DeviceT>, Layout, Timing>;

}   // namespace Kvasir::I2C
