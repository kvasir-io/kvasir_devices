#pragma once

#include "../Duration.hpp"
#include "../Log.hpp"
#include "kvasir/Log/LogLimited.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Periodic.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>

/// Whether a device is there: the absent-device policy of the device engine (Device.hpp).
///
/// A NAK is the device not answering; a bus fault says nothing about it. After
/// `absentAfterNaks` NAKs in a row the device is parked -- the engine runs no turn for it
/// and puts nothing on the bus -- and probed with one bring-up per interval, 1 s, 2 s, 4 s
/// ... `probeIntervalMax`. The probe is the driver's own first transaction -- its first Init
/// step, or for a chip with no Init whatever it sends first, and only its ACK makes such a chip
/// answering again (Device::link()); an ACK
/// unparks the device at once, a NAK leaves it parked until the next interval.
///
/// Everything here runs in the loop, from the outcome the engine takes each turn, so it is
/// plain data: no atomics, no interrupt ever touches it. `UC_LOG_W` / `UC_LOG_I` come from
/// ../Log.hpp: uc_log on the target, the definitions a host test provides.
namespace Kvasir::Engine {

/// The defaults. A Device's Config may redeclare any of them:
///     struct Config { static constexpr std::uint8_t absentAfterNaks = 0; };   // never park
struct PresenceDefaults {
    /// Consecutive NAKs after which the device is parked; 0 disables parking.
    static constexpr std::uint8_t absentAfterNaks = 3;
    /// A parked device is probed once per interval, which doubles after each failed
    /// probe up to probeIntervalMax.
    static constexpr auto probeInterval    = std::chrono::seconds{1};
    static constexpr auto probeIntervalMax = std::chrono::seconds{30};
};

/// What the engine may do this turn. Outside `Presence` so that it is one type whatever
/// knobs a device's Config sets: the engine (Engine.hpp) takes it from every device.
enum class PresenceTurn : std::uint8_t {
    talk,    ///< present, or a probe is under way: run the turn
    wait,    ///< parked and no probe due: run nothing, submit nothing
    probe,   ///< parked and a probe is due: start the bring-up again, it is the probe
    park,    ///< the streak just reached the threshold: start over, then wait
};

/// Whether a turn may rest (Engine::rest). Outside `Presence` for the same reason, and so that
/// NoPresence answers with the same type.
enum class PresenceRest : std::uint8_t {
    busy,     ///< a probe is armed or on the wire, or the streak is at the threshold: no rest
    talk,     ///< present: the turn decides
    parked,   ///< parked: rest until the next probe
};

/// The knobs as values, so `Presence` is one type whatever a Config sets (DeviceKnobs::presence).
struct PresenceKnobs {
    std::uint32_t probeIntervalMs{};
    std::uint32_t probeIntervalMaxMs{};
    std::uint8_t  absentAfterNaks{};
};

/// A Config's knobs, each falling back to PresenceDefaults.
template<typename Cfg>
constexpr PresenceKnobs presenceKnobs() {
    auto const absent = static_cast<std::uint8_t>(
      KVASIR_CONFIG_OR(Cfg, absentAfterNaks, PresenceDefaults::absentAfterNaks));
    auto const interval
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Cfg, probeInterval, PresenceDefaults::probeInterval));
    auto const intervalMax = Kvasir::asDuration(
      KVASIR_CONFIG_OR(Cfg, probeIntervalMax, PresenceDefaults::probeIntervalMax));
    return {.probeIntervalMs    = static_cast<std::uint32_t>(interval.count()),
            .probeIntervalMaxMs = static_cast<std::uint32_t>(intervalMax.count()),
            .absentAfterNaks    = absent};
}

template<typename Cfg>
inline constexpr bool presenceKnobsOk
  = presenceKnobs<Cfg>().probeIntervalMs > 0
 && presenceKnobs<Cfg>().probeIntervalMaxMs >= presenceKnobs<Cfg>().probeIntervalMs;

template<typename Clock>
struct Presence {
    using TimePoint = typename Clock::time_point;

    using Step = PresenceTurn;

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
    Step step(TimePoint                     now,
              [[maybe_unused]] std::uint8_t address,
              PresenceKnobs const&          k) {
        if(!absent_) {
            if(k.absentAfterNaks == 0 || naks_ < k.absentAfterNaks) { return Step::talk; }
            absent_   = true;
            probe_    = Probe::none;
            probes_   = 0;
            interval_ = std::chrono::milliseconds{k.probeIntervalMs};
            nextProbe_.restart(interval_, now);
            KVASIR_LOG_LIMITED(log_.allow(NotResponding, now),
                               UC_LOG_W,
                               "i2c device {:#04x} not responding ({} NAKs in a row) -- "
                               "probing every {} .. {}",
                               address,
                               naks_,
                               std::chrono::milliseconds{k.probeIntervalMs},
                               std::chrono::milliseconds{k.probeIntervalMaxMs});
            return Step::park;
        }
        // Parked. A probe that is armed or on the wire keeps the engine running so that
        // it can be submitted and its outcome taken; between probes nothing runs.
        if(probe_ != Probe::none) { return Step::talk; }
        if(nextProbe_.armed(now)) { return Step::wait; }
        probe_ = Probe::armed;
        nextProbe_.restart(interval_, now);
        interval_
          = std::min<std::chrono::milliseconds>(interval_ * 2,
                                                std::chrono::milliseconds{k.probeIntervalMaxMs});
        ++probes_;
        return Step::probe;
    }

    /// For Engine::rest: `talk` while present, `parked` until the next probe (in `until`),
    /// `busy` when the next turn acts (a probe armed or on the wire, a streak about to park).
    using Rest = PresenceRest;

    [[nodiscard]] Rest rest(PresenceKnobs const& k,
                            TimePoint&           until) const {
        if(!absent_) {
            return k.absentAfterNaks != 0 && naks_ >= k.absentAfterNaks ? Rest::busy : Rest::talk;
        }
        if(probe_ != Probe::none) { return Rest::busy; }
        until = nextProbe_.end();
        return Rest::parked;
    }

    /// Asked before every submit: a present device may always talk, a parked one only
    /// for the probe that step() armed -- and for the rest of that bring-up while the
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

    // In order of alignment: the small fields share the tail.
    Kvasir::Deadline<Clock> nextProbe_{};   ///< armed while parked
    Millis32                interval_{};    ///< set when the device is parked
    std::uint16_t           probes_{};
    std::uint8_t            naks_{};
    bool                    absent_{false};
    Probe                   probe_{Probe::none};
    /// A flapping device must not flood the log; empty without logging (LogRateLimiter).
    [[no_unique_address]] LogRateLimiter<Clock> log_{};
};

/// Presence on a bus whose engine features leave it out (EngineFeatures::presence): no state,
/// and every answer is "present, talk" - the part is never parked, a NAK is a failure like any
/// other. The same calls as Presence, so the engine's code is the same for both and folds away.
template<typename Clock>
struct NoPresence {
    using TimePoint = typename Clock::time_point;
    using Step      = PresenceTurn;
    using Rest      = PresenceRest;

    [[nodiscard]] static constexpr bool present() { return true; }

    [[nodiscard]] static constexpr bool absent() { return false; }

    [[nodiscard]] static constexpr std::uint8_t consecutiveNaks() { return 0; }

    [[nodiscard]] static constexpr std::uint16_t probes() { return 0; }

    static constexpr void ack(TimePoint,
                              std::uint8_t) {}

    static constexpr void nak() {}

    static constexpr void fault() {}

    static constexpr void restart() {}

    // The knobs are any type: on such a port DeviceKnobs has none (detail::Absent).
    template<typename Knobs>
    static constexpr Step step(TimePoint,
                               std::uint8_t,
                               Knobs const&) {
        return Step::talk;
    }

    template<typename Knobs>
    [[nodiscard]] static constexpr Rest rest(Knobs const&,
                                             TimePoint&) {
        return Rest::talk;
    }

    [[nodiscard]] static constexpr bool mayTalk() { return true; }
};

}   // namespace Kvasir::Engine
