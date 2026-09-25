#pragma once

#include "../Duration.hpp"
#include "../Log.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>

/// Whether a device is there: the absent-device policy of the I2C engine (Device.hpp).
///
/// A NAK is the device not answering; a bus fault says nothing about it. After
/// `AbsentAfterNaks` NAKs in a row the device is parked -- the engine runs no turn for it
/// and puts nothing on the bus -- and probed with one bring-up per interval, 1 s, 2 s, 4 s
/// ... `ProbeIntervalMax`. The probe is the driver's own first transaction -- its first Init
/// step, or for a chip with no Init whatever it sends first, and only its ACK makes such a chip
/// answering again (Device::link()); an ACK
/// unparks the device at once, a NAK leaves it parked until the next interval.
///
/// Everything here runs in the loop, from the outcome the engine takes each turn, so it is
/// plain data: no atomics, no interrupt ever touches it. `UC_LOG_W` / `UC_LOG_I` come from
/// ../Log.hpp: uc_log on the target, the definitions a host test provides.
namespace Kvasir::I2C {

/// The defaults. A Device's Config may redeclare any of them:
///     struct Config { static constexpr std::uint8_t AbsentAfterNaks = 0; };   // never park
struct PresenceDefaults {
    /// Consecutive NAKs after which the device is parked; 0 disables parking.
    static constexpr std::uint8_t AbsentAfterNaks = 3;
    /// A parked device is probed once per interval, which doubles after each failed
    /// probe up to ProbeIntervalMax.
    static constexpr auto ProbeInterval    = std::chrono::seconds{1};
    static constexpr auto ProbeIntervalMax = std::chrono::seconds{30};
};

/// What the engine may do this turn. Outside `Presence` so that it is one type whatever
/// knobs a device's Config sets: the engine (Engine.hpp) takes it from every device.
enum class PresenceTurn : std::uint8_t {
    talk,    ///< present, or a probe is under way: run the turn
    wait,    ///< parked and no probe due: run nothing, submit nothing
    probe,   ///< parked and a probe is due: start the bring-up again, it is the probe
    park,    ///< the streak just reached the threshold: start over, then wait
};

template<typename Clock, typename Cfg = PresenceDefaults>
struct Presence {
    using TimePoint = typename Clock::time_point;

    static constexpr std::uint8_t AbsentAfterNaks = [] {
        if constexpr(requires { Cfg::AbsentAfterNaks; }) {
            return static_cast<std::uint8_t>(Cfg::AbsentAfterNaks);
        } else {
            return PresenceDefaults::AbsentAfterNaks;
        }
    }();

    static constexpr std::chrono::milliseconds ProbeInterval = [] {
        if constexpr(requires { Cfg::ProbeInterval; }) {
            return Kvasir::asDuration(Cfg::ProbeInterval);
        } else {
            return Kvasir::asDuration(PresenceDefaults::ProbeInterval);
        }
    }();

    static constexpr std::chrono::milliseconds ProbeIntervalMax = [] {
        if constexpr(requires { Cfg::ProbeIntervalMax; }) {
            return Kvasir::asDuration(Cfg::ProbeIntervalMax);
        } else {
            return Kvasir::asDuration(PresenceDefaults::ProbeIntervalMax);
        }
    }();

    static_assert(ProbeInterval > std::chrono::milliseconds::zero()
                    && ProbeIntervalMax >= ProbeInterval,
                  "a probe interval, and a maximum it grows towards");

    using Turn = PresenceTurn;

    [[nodiscard]] bool present() const { return !absent_; }

    [[nodiscard]] bool absent() const { return absent_; }

    [[nodiscard]] std::uint8_t consecutiveNaks() const { return naks_; }

    /// Probes since the device was parked (0 while present).
    [[nodiscard]] std::uint16_t probes() const { return probes_; }

    /// The device answered.
    /// (`address` and `now` only reach the log line; a host build stubs it out.)
    void ack([[maybe_unused]] TimePoint    now,
             [[maybe_unused]] std::uint8_t address) {
        naks_  = 0;
        probe_ = Probe::none;
        if(absent_) {
            absent_ = false;
            KVASIR_LOG_LIMITED(log_.allow(PresentAgain, now),
                               UC_LOG_I,
                               "i2c device {:#04x} present again after {} probe(s)",
                               address,
                               probes_);
        }
    }

    /// The device did not answer.
    void nak() {
        if(naks_ != std::numeric_limits<std::uint8_t>::max()) { ++naks_; }
        probe_ = Probe::none;
    }

    /// A bus fault: the streak is left alone, but a probe it happened to is over.
    void fault() { probe_ = Probe::none; }

    /// The part was started over from outside -- its supply was cycled (PowerRail.hpp): what
    /// it did before says nothing about it now, so it is present again and the streak starts
    /// at nothing. The probe count stays what it was until the next time it is parked.
    void restart() {
        naks_   = 0;
        absent_ = false;
        probe_  = Probe::none;
    }

    /// Once per turn, before anything else.
    Turn turn(TimePoint                     now,
              [[maybe_unused]] std::uint8_t address) {
        if(!absent_) {
            if(AbsentAfterNaks == 0 || naks_ < AbsentAfterNaks) { return Turn::talk; }
            absent_    = true;
            probe_     = Probe::none;
            probes_    = 0;
            interval_  = ProbeInterval;
            nextProbe_ = now + interval_;
            KVASIR_LOG_LIMITED(log_.allow(NotResponding, now),
                               UC_LOG_W,
                               "i2c device {:#04x} not responding ({} NAKs in a row) -- "
                               "probing every {} .. {}",
                               address,
                               naks_,
                               ProbeInterval,
                               ProbeIntervalMax);
            return Turn::park;
        }
        // Parked. A probe that is armed or on the wire keeps the engine running so that
        // it can be submitted and its outcome taken; between probes nothing runs.
        if(probe_ != Probe::none) { return Turn::talk; }
        if(now < nextProbe_) { return Turn::wait; }
        probe_     = Probe::armed;
        nextProbe_ = now + interval_;
        interval_  = std::min(interval_ * 2, ProbeIntervalMax);
        ++probes_;
        return Turn::probe;
    }

    /// Asked before every submit: a present device may always talk, a parked one only
    /// for the probe that turn() armed -- and for the rest of that bring-up while the
    /// probe is on the wire, since the outcome ends it either way.
    [[nodiscard]] bool mayTalk() {
        if(!absent_) { return true; }
        if(probe_ == Probe::none) { return false; }
        probe_ = Probe::inFlight;
        return true;
    }

private:
    enum class Probe : std::uint8_t { none, armed, inFlight };

    static constexpr std::uint32_t NotResponding = rateLimitKey(1);
    static constexpr std::uint32_t PresentAgain  = rateLimitKey(2);

    std::uint8_t              naks_{};
    bool                      absent_{false};
    Probe                     probe_{Probe::none};
    std::uint16_t             probes_{};
    std::chrono::milliseconds interval_{ProbeInterval};
    TimePoint                 nextProbe_{};
    RateLimiter<Clock>        log_{};   ///< a flapping device must not flood the log
};

}   // namespace Kvasir::I2C
