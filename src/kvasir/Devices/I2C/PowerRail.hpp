#pragma once

#include "../Duration.hpp"
#include "../Log.hpp"

#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>

/// A switched supply that the parts of one Bus share, cycled when a part stops answering.
///
/// A part that hangs -- SDA held low after a glitch, a sensor that latched up in the wet --
/// is not brought back by talking to it; taking its supply away does. A Device's own `Reset`
/// (Device.hpp) is a line of one part. A rail is not: switching it takes every part on it
/// down, the ones that were fine included, and they come back at their power-on defaults
/// without their devices having seen anything. So the rail sits round the Bus rather than
/// inside one Device:
///
///     struct SensorSupply {
///         using Claims = Kvasir::Io::PinClaims<HW::Pin::sensorVcc>;   // configured elsewhere
///         static void off() { apply(clear(HW::Pin::sensorVcc{})); }
///         static void on() { apply(set(HW::Pin::sensorVcc{})); }
///     };
///
///     Sensors                                          bus{};
///     Kvasir::I2C::PowerRail<Clock, SensorSupply>      rail{};
///
///     I2c::handler();      // the bus behavior's own handler: every turn, as ever
///     rail.handler(bus);   // once per loop turn, INSTEAD of bus.handler()
///
/// It starts with the rail off for `OffTime` (so a warm reset of the controller is a cold
/// start of the parts), switches it on, waits `SettleTime`, and from then on runs
/// `bus.handler()`. When a part has been absent (Device::absent(): parked after NAKs in a
/// row, Presence.hpp) for `AbsentBeforeCycle`, the rail is cycled and `bus.restart()` starts
/// every device over, since every part lost what it was configured to.
///
/// A part that is not fitted, or is dead, stays absent through the cycle. It must not take
/// the others down every few seconds for ever, so the wait before the next cycle doubles
/// with each one that did not bring everything back, from `AbsentBeforeCycle` up to
/// `CycleIntervalMax`; it is `AbsentBeforeCycle` again once every part answers.
///
/// While the rail is off or settling nothing is put on the bus: bus.handler() is not run.
/// The bus behavior's handler() has to keep running all the same -- it owns the timeout of
/// whatever was in flight when the rail went.
///
/// What was in flight: restart() makes the devices forget it (a late callback is ignored,
/// Pending.hpp), but the request is still in the behavior's queue and the peripheral goes on
/// clocking it into parts that are losing their supply; one that dies in the middle of a byte
/// can leave the peripheral thinking the bus is busy. So a Switch may declare
///
///         static void abandonBus() { I2c::reset(); }   // drop the queue, reset the peripheral
///
/// which is called after restart() and before off(). Without it the cycle relies on the
/// behavior's own timeout being over before the rail is back -- keep OffTime above it (the
/// SAM's Sercom_I2CQueued gives a transfer 100 ms).
///
/// Two things the rail takes for granted about the board and the Bus:
///   - The bus pull-ups hang on the switched rail. On the permanent supply they would feed
///     the parts through SDA/SCL and their protection diodes while the rail is "off", and
///     the cycle would reset nothing.
///   - A switch (Mux.hpp) on the rail is a member of the Bus. It comes back with every
///     channel closed; restart() makes its device write the control byte again before a
///     gated part talks. A switch kept outside the Bus is not restarted.
///
/// Everything here runs in the loop; plain data, no atomics.
namespace Kvasir::I2C {

/// The defaults. A PowerRail's Config may redeclare any of them.
struct PowerRailDefaults {
    /// How long the rail stays off: long enough for the parts' decoupling to discharge
    /// through whatever load there is, which no data sheet of a part can say.
    static constexpr auto OffTime = std::chrono::milliseconds{500};
    /// From switching the rail on to the first transaction. The parts' own StartupDelay
    /// comes on top, per device.
    static constexpr auto SettleTime = std::chrono::milliseconds{100};
    /// A part has to be absent for this long before the rail is cycled for it. Longer than
    /// Presence's first probe interval, so a part that only missed a few transactions gets
    /// its probe first.
    static constexpr auto AbsentBeforeCycle = std::chrono::seconds{5};
    /// What the wait grows to while a part stays absent through every cycle.
    static constexpr auto CycleIntervalMax = std::chrono::minutes{5};
};

namespace detail {
    /// The switch's pin claim, if it has one, becomes the rail's (as a Device takes its reset
    /// line's), so a Startup that is handed the rail sees the pin as taken.
    template<typename S>
    struct RailClaims {};

    template<typename S>
        requires requires { typename S::Claims; }
    struct RailClaims<S> {
        using Claims = typename S::Claims;
    };
}   // namespace detail

template<typename Clock, typename Switch, typename Config = PowerRailDefaults>
struct PowerRail : detail::RailClaims<Switch> {
    using TimePoint = typename Clock::time_point;

    static constexpr std::chrono::milliseconds OffTime = [] {
        if constexpr(requires { Config::OffTime; }) {
            return Kvasir::asDuration(Config::OffTime);
        } else {
            return Kvasir::asDuration(PowerRailDefaults::OffTime);
        }
    }();

    static constexpr std::chrono::milliseconds SettleTime = [] {
        if constexpr(requires { Config::SettleTime; }) {
            return Kvasir::asDuration(Config::SettleTime);
        } else {
            return Kvasir::asDuration(PowerRailDefaults::SettleTime);
        }
    }();

    static constexpr std::chrono::milliseconds AbsentBeforeCycle = [] {
        if constexpr(requires { Config::AbsentBeforeCycle; }) {
            return Kvasir::asDuration(Config::AbsentBeforeCycle);
        } else {
            return Kvasir::asDuration(PowerRailDefaults::AbsentBeforeCycle);
        }
    }();

    static constexpr std::chrono::milliseconds CycleIntervalMax = [] {
        if constexpr(requires { Config::CycleIntervalMax; }) {
            return Kvasir::asDuration(Config::CycleIntervalMax);
        } else {
            return Kvasir::asDuration(PowerRailDefaults::CycleIntervalMax);
        }
    }();

    static_assert(OffTime > std::chrono::milliseconds::zero()
                    && SettleTime >= std::chrono::milliseconds::zero()
                    && AbsentBeforeCycle > std::chrono::milliseconds::zero()
                    && CycleIntervalMax >= AbsentBeforeCycle,
                  "an off time, a wait before a cycle, and a maximum that wait grows towards");

    enum class State : std::uint8_t { off, settling, on };

    [[nodiscard]] State state() const { return state_; }

    /// The rail is on and settled: the bus is being run.
    [[nodiscard]] bool powered() const { return state_ == State::on; }

    /// Cycles since start, the first power-on not counted.
    [[nodiscard]] std::uint32_t cycles() const { return cycles_; }

    /// The wait before the next cycle for a part that stays absent.
    [[nodiscard]] std::chrono::milliseconds cycleInterval() const { return interval_; }

    /// Cycle the rail now, whatever the parts say: a test mode, a command. While the rail is
    /// already off or settling it is being cycled, and the request is nothing more.
    void cycle() {
        if(state_ == State::on) { cycleRequested_ = true; }
    }

    /// Once per loop turn, in place of bus.handler().
    template<typename B>
        requires requires(B& b) {
            b.handler();
            b.restart();
            { b.absentCount() } -> std::convertible_to<std::size_t>;
            { b.answeringCount() } -> std::convertible_to<std::size_t>;
            { B::Parts } -> std::convertible_to<std::size_t>;
        }
    void handler(B& bus) {
        auto const now = Clock::now();
        if(!started_) {
            started_ = true;
            switchOff_(now);
            return;
        }
        switch(state_) {
        case State::off:
            if(now < waitUntil_) { break; }
            Switch::on();
            waitUntil_ = now + SettleTime;
            state_     = State::settling;
            break;
        case State::settling:
            if(now < waitUntil_) { break; }
            absentSince_.reset();
            presentSince_.reset();
            state_ = State::on;
            break;
        case State::on:
            bus.handler();
            switch(due_(bus.absentCount(), bus.answeringCount() + offline_(bus) == B::Parts, now)) {
            case Due::no: return;
            case Due::requested:
                UC_LOG_I("i2c power rail: cycle {} on request", cycles_ + 1);
                break;
            case Due::absent:
                // Not rate limited as Presence's lines are: the interval it doubles is the limit.
                UC_LOG_W("i2c power rail: {} part(s) absent -- cycle {}, the next not before {}",
                         bus.absentCount(),
                         cycles_ + 1,
                         interval_);
                break;
            }
            ++cycles_;
            bus.restart();
            if constexpr(requires { Switch::abandonBus(); }) { Switch::abandonBus(); }
            switchOff_(now);
            break;
        }
    }

private:
    enum class Due : std::uint8_t { no, requested, absent };

    void switchOff_(TimePoint now) {
        Switch::off();
        cycleRequested_ = false;
        waitUntil_      = now + OffTime;
        state_          = State::off;
    }

    /// Parts behind a bridge that is not active (Bridge.hpp) are not meant to answer: "every
    /// part answers" is said of the others.
    template<typename B>
    [[nodiscard]] static std::size_t offline_(B const& bus) {
        if constexpr(requires { bus.offlineCount(); }) {
            return bus.offlineCount();
        } else {
            return 0;
        }
    }

    [[nodiscard]] Due due_(std::size_t absent,
                           bool        allAnswering,
                           TimePoint   now) {
        if(cycleRequested_) { return Due::requested; }
        // The wait is the first one again only once every part answers: right after a cycle
        // none is absent yet -- the one that will not come back has not been NAKed often
        // enough to be parked -- and that says nothing about it.
        if(allAnswering) { interval_ = AbsentBeforeCycle; }
        if(absent == 0) {
            // A part that is parked, answers one probe and is parked again is what the rail
            // is for as much as one that stays away: the absence is only over once no part
            // has been absent for as long as it takes to be cycled for.
            if(absentSince_) {
                if(!presentSince_) { presentSince_ = now; }
                if(now - *presentSince_ >= AbsentBeforeCycle) { absentSince_.reset(); }
            }
            return Due::no;
        }
        presentSince_.reset();
        if(!absentSince_) {
            absentSince_ = now;
            return Due::no;
        }
        if(now - *absentSince_ < interval_) { return Due::no; }
        // Doubled without overflowing, whatever CycleIntervalMax a Config declares.
        interval_ = interval_ >= CycleIntervalMax / 2 ? CycleIntervalMax : interval_ * 2;
        return Due::absent;
    }

    State                     state_{State::off};
    bool                      started_{false};
    bool                      cycleRequested_{false};
    std::uint32_t             cycles_{};
    std::chrono::milliseconds interval_{AbsentBeforeCycle};
    TimePoint                 waitUntil_{};
    std::optional<TimePoint>  absentSince_{};    ///< since when a part has been absent
    std::optional<TimePoint>  presentSince_{};   ///< since when none has been, after that
};

}   // namespace Kvasir::I2C
