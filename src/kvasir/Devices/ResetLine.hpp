#pragma once

#include <chrono>
#include <cstdint>

/// The reset line policy of every driver with a RESET pin, and its pulse. SDK-free.
namespace Kvasir {

/// `hold()` puts the device in reset, `release()` lets it run. NoReset: no line, or one
/// another device's driver pulses.
template<typename R>
concept ResetLine = requires {
    R::hold();
    R::release();
};

struct NoReset {
    /// A driver tests this marker, not the type.
    static constexpr bool NoLine = true;

    static void hold() {}

    static void release() {}
};

namespace detail {
    /// The reset line's pin claim, if any, becomes the driver's.
    template<typename R>
    struct ResetClaims {};

    template<typename R>
        requires requires { typename R::Claims; }
    struct ResetClaims<R> {
        using Claims = typename R::Claims;
    };
}   // namespace detail

/// Non-blocking: `begin(now)` holds the line; `done(now)` releases it after `low` and
/// answers true once `settle` more has passed.
template<ResetLine Reset, typename Clock>
struct ResetPulse {
    using tp = typename Clock::time_point;

    void begin(tp                        now,
               std::chrono::milliseconds low,
               std::chrono::milliseconds settle) {
        Reset::hold();
        settle_ = settle;
        until_  = now + low;
        phase_  = Phase::low;
    }

    [[nodiscard]] bool done(tp now) {
        switch(phase_) {
        case Phase::low:
            if(now > until_) {
                Reset::release();
                until_ = now + settle_;
                phase_ = Phase::settling;
            }
            return false;
        case Phase::settling:
            if(now > until_) { phase_ = Phase::done; }
            return phase_ == Phase::done;
        case Phase::idle:
        case Phase::done: return true;
        }
        return true;
    }

private:
    enum class Phase : std::uint8_t { idle, low, settling, done };

    Phase                     phase_{Phase::idle};
    tp                        until_{};
    std::chrono::milliseconds settle_{};
};

}   // namespace Kvasir
