#pragma once

#include "../Log.hpp"
#include "Device.hpp"
#include "chips/Touch.hpp"

#include <algorithm>
#include <chrono>
#include <concepts>
#include <cstdint>
#include <utility>

/// A capacitive touch panel: a controller description on the I2C engine (chips/Touch.hpp)
/// plus everything that is about the *panel* rather than the chip -- the mount transform,
/// the report a page reads, a sequence number, a timestamp, and the counters a log line
/// wants.
///
///     using TouchReset = MyResetLine;   // any type with hold()/release() on the reset pin
///     using Touch = Kvasir::I2C::Touch::Controller<I2c1, Clock,
///                                                  Kvasir::I2C::Chips::TouchDetail::Cst9217,
///                                                  HW::TouchConfig, TouchReset>;
///     constinit Touch touch{};   // static here: the ISR below has to reach it by name
///     ...
///     void onEdge(std::uint32_t) { touch.interrupt(); }   // a flag; the read is in the loop
///     touch.handler();                                    // once per loop turn
///     if(auto const& t = touch.latest(); t.seq != seen) { seen = t.seq; use(t); }
///
/// `UC_LOG_I` / `UC_LOG_W` come from ../Log.hpp: uc_log on the target, the definitions a host
/// test provides.
///
/// In Kvasir::I2C since 2026-09-21, so its log lines are under the "i2c" module
/// ("i2c.touch.controller", derived from the scope).
namespace Kvasir::I2C::Touch {

/// One report, controller-independent: what the application publishes and feeds to an event
/// layer. `points` is the finger count (0: none); `x`/`y` are finger 0 in panel pixels after
/// the config's transform, and a release keeps the last point. `seq` steps for every report
/// that says something new -- a finger, a move, the first "no finger" after a finger -- and
/// not for the idle polls that repeat "no finger", so a reader that watches `seq` is not
/// woken ten times a second by an untouched panel. `at` is the I2C completion, on the clock.
template<typename TimePoint>
struct Report {
    std::uint8_t  points{};
    std::uint16_t x{};
    std::uint16_t y{};
    std::uint32_t seq{};
    TimePoint     at{};
};

using Kvasir::I2C::Chips::TouchDetail::ChipInfo;
using Kvasir::I2C::Chips::TouchDetail::IntEdge;

/// One axis as the panel shows it: as the controller reports it, or mirrored.
enum class Mirror : std::uint8_t { none, mirrored };

/// The two axes: as the controller reports them, or swapped (a panel mounted rotated by 90
/// degrees).
enum class Axes : std::uint8_t { asReported, swapped };

/// The knobs a config may leave out, with what they are then. A config derives from this and
/// redeclares what it sets:
///     struct TouchConfig : Kvasir::I2C::Touch::Defaults {
///         static constexpr std::uint16_t Width = 466, Height = 466;
///         static constexpr auto FlipX = Kvasir::I2C::Touch::Mirror::mirrored;
///         static constexpr auto FlipY = Kvasir::I2C::Touch::Mirror::mirrored;
///     };
struct Defaults : Kvasir::I2C::Chips::TouchDetail::PollDefaults {
    static constexpr Mirror FlipX  = Mirror::none;   ///< mirror after the swap
    static constexpr Mirror FlipY  = Mirror::none;
    static constexpr Axes   SwapXY = Axes::asReported;
};

template<typename I2c,
         typename Clock,
         Kvasir::I2C::Chips::TouchDetail::Controller Ctrl,
         typename Config,
         typename Reset = Kvasir::I2C::NoReset>
struct Controller : Kvasir::I2C::detail::ResetClaims<Reset> {
    using Chip      = Kvasir::I2C::Chips::TouchPanel<Ctrl, Config>;
    using Device    = Kvasir::I2C::Device<I2c, Clock, Chip, Config, Reset>;
    using Data      = typename Chip::Data;
    using Sample    = typename Chip::Sample;
    using TimePoint = typename Clock::time_point;
    using Report    = Kvasir::I2C::Touch::Report<TimePoint>;

    static_assert(std::derived_from<Config,
                                    Defaults>,
                  "derive the touch config from Kvasir::I2C::Touch::Defaults");
    static_assert(Config::Width > 0 && Config::Height > 0,
                  "Config needs Width and Height");
    static_assert(Config::Width <= 4096 && Config::Height <= 4096,
                  "a 12-bit controller coordinate");

    /// The INT edge the controller produces, for the application's GPIO interrupt.
    static constexpr IntEdge Int = Chip::Int;

    /// From the INT line's interrupt: asks for one data read on the next free turn. The
    /// engine's request<> is an atomic store, and a requested group is picked ahead of every
    /// periodic one, so this is the whole of the INT path.
    void interrupt() { device_.template request<Data>(); }

    [[nodiscard]] Report const& latest() const { return latest_; }

    /// Where the controller is (Device::link()): answering once it is brought up and reading
    /// reports, absent while it is parked and probed.
    [[nodiscard]] Kvasir::I2C::Link link() const { return device_.link(); }

    [[nodiscard]] bool answering() const { return device_.answering(); }

    /// What the init script's identification bytes said.
    [[nodiscard]] ChipInfo const& chipInfo() const { return device_.state(); }

    /// The policy's verdict: this is the chip it is for.
    [[nodiscard]] bool identified() const { return device_.identified(); }

    /// Frames that said something: a finger, a move, or the release after one.
    [[nodiscard]] std::uint32_t reports() const { return device_.template samples<Data>(); }

    /// Of those, the ones carrying a finger.
    [[nodiscard]] std::uint32_t touches() const { return touches_; }

    /// Frames that were well formed and said nothing new: the glass still untouched, or a
    /// controller's "no new data" bit (only the GT911 has one, and the two are one count).
    [[nodiscard]] std::uint32_t quiet() const { return device_.template unchanged<Data>(); }

    /// Frames the controller's decoder could not believe.
    [[nodiscard]] std::uint32_t malformed() const { return device_.template rejected<Data>(); }

    /// Transactions that failed (NAK, bus fault, timeout).
    [[nodiscard]] std::uint32_t errors() const { return device_.errors(); }

    [[nodiscard]] Device& device() { return device_; }

    [[nodiscard]] Device const& device() const { return device_; }

    /// Controller coordinates to panel pixels: swap, clamp, mirror, per Config.
    [[nodiscard]] static constexpr std::pair<std::uint16_t,
                                             std::uint16_t>
    transform(std::uint16_t rx,
              std::uint16_t ry) {
        if constexpr(Config::SwapXY == Axes::swapped) { std::swap(rx, ry); }
        return {mirror_(rx, Config::Width, Config::FlipX),
                mirror_(ry, Config::Height, Config::FlipY)};
    }

    /// Once per loop turn: the device's own turn, then anything it has to say.
    void handler() {
        device_.handler();

        if(!device_.answering()) {
            // The controller went away, or is being brought up again. A finger it was
            // reporting is released here, as a report, so a reader does not keep it down
            // while the panel is gone.
            if(latest_.points != 0) {
                latest_.points = 0;
                ++latest_.seq;
                latest_.at = Clock::now();
            }
            wasUp_ = false;
            return;
        }

        if(!wasUp_) {
            wasUp_ = true;
            checkResolution_();
        }

        if(device_.template fresh<Data>()) {
            auto const& sample = device_.template latest<Data>();
            auto const [x, y]  = transform(sample.x, sample.y);
            latest_.points     = sample.points;
            latest_.x          = x;
            latest_.y          = y;
            latest_.at         = device_.template stamp<Data>();
            ++latest_.seq;
            if(sample.points != 0) { ++touches_; }
        }
    }

private:
    static constexpr std::uint16_t mirror_(std::uint16_t v,
                                           std::uint16_t extent,
                                           Mirror        flip) {
        auto const clamped = std::min<std::uint16_t>(v, static_cast<std::uint16_t>(extent - 1));
        return flip == Mirror::mirrored ? static_cast<std::uint16_t>(extent - 1 - clamped)
                                        : clamped;
    }

    /// The controller's own resolution against the config's, once per bring-up. The config
    /// is in panel coordinates, after the swap, so under SwapXY its width and height are
    /// swapped back before the comparison.
    void checkResolution_() {
        auto const& id = device_.state();
        if(id.width == 0) { return; }
        auto const [cw, ch]
          = Config::SwapXY == Axes::swapped
            ? std::pair<std::uint16_t, std::uint16_t>{Config::Height, Config::Width}
            : std::pair<std::uint16_t, std::uint16_t>{Config::Width, Config::Height};
        if(id.width != cw || id.height != ch) {
            UC_LOG_W("{}: the controller reports {}x{}, the config says {}x{}",
                     Chip::Name,
                     id.width,
                     id.height,
                     cw,
                     ch);
        }
    }

    Device        device_{};
    Report        latest_{};
    std::uint32_t touches_{};
    bool          wasUp_{false};
};

}   // namespace Kvasir::I2C::Touch
