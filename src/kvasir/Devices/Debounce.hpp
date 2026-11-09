#pragma once

#include <chrono>

namespace Kvasir {

/// One boolean input debounced against a clock: value() takes a new level only after the
/// input has read that level, without a break, for the settle time. Every sample of the old
/// level in between restarts the wait, so a bounce, a spike or a drop-out shorter than the
/// settle time is never seen, and a level that holds is seen `settle` after its last bounce
/// (on the first sample at or after that). update() says when value() changed.
///
///     Kvasir::Debounce<Clock> lid{20ms, readLid()};   // 20 ms, starting at the level read now
///     while(true) {
///         if(lid.update(readLid())) { UC_LOG_I("lid {}", lid.value() ? "closed" : "open"); }
///     }
///
/// Two settle times, `Debounce{rise, fall}`, where the two directions guard against different
/// things: a push button's press against a spike on the wire (a few ms, the press is late by as
/// much), its release against the contact's drop-outs in the middle of a press (longer than
/// the longest one) - PushButton's debouncePress / debounceRelease, the same rule.
///
/// Time is the clock's own duration, integer ticks on every Kvasir clock: no floating point
/// (a duration<double> is a soft-float library call on a core without an FPU). A settle time is
/// converted to it implicitly only where that is exact. The wait is measured as `now - start`,
/// never as a deadline `start + settle` compared with now, so a clock with an unsigned rep may
/// wrap while a change is pending (the difference is taken modulo the rep) and a signed one may
/// run up to the top of its range. On a wrapping clock call update() at least once per wrap
/// period minus the settle time while a change is pending, or the elapsed time aliases.
///
/// Samples, not edges: what the input does between two samples is not seen, so sample it
/// several times per settle time. One object belongs to one context: an interrupt and the main
/// loop must not share one.
template<typename Clock>
class Debounce {
public:
    using duration   = typename Clock::duration;
    using time_point = typename Clock::time_point;

    /// The same settle time both ways; value() starts at `initial` (the level read at start-up,
    /// so the first sample is no change).
    constexpr explicit Debounce(duration settle,
                                bool     initial = false)
      : Debounce{settle,
                 settle,
                 initial} {}

    /// `rise`: how long the input must read true before value() turns true; `fall`: the same
    /// for false. Zero (or less): that change passes at once.
    constexpr Debounce(duration rise,
                       duration fall,
                       bool     initial = false)
      : rise_{rise}
      , fall_{fall}
      , level_{initial} {}

    /// One sample of the input, read at `now`. True when value() changed with it.
    constexpr bool update(bool       raw,
                          time_point now) {
        if(raw == level_) {
            pending_ = false;   // back (a bounce): the next change waits from its own start
            return false;
        }
        if(!pending_) {
            pending_ = true;
            since_   = now;
        }
        if(now - since_ < (raw ? rise_ : fall_)) { return false; }
        level_   = raw;
        pending_ = false;
        return true;
    }

    /// One sample of the input, read now.
    bool update(bool raw) { return update(raw, Clock::now()); }

    /// The debounced level.
    [[nodiscard]] constexpr bool value() const { return level_; }

private:
    duration   rise_;
    duration   fall_;
    time_point since_{};   ///< the input has read !level_ since then, while pending_
    bool       level_;
    bool       pending_{false};
};

}   // namespace Kvasir
