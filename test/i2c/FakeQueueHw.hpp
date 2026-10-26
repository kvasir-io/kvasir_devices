#pragma once
/// The queued I2C core (I2C/Controller/QueueCore.hpp) over a fake hardware policy, for the host tests that drive it
/// (i2c/queue_core_test.cpp, bus/request_queue_contract_test.cpp): a scripted LineRecovery and a block whose
/// interrupt the test raises with what the wire did. `Tag` makes another bus of the same shape - its own policy, its
/// own recovery, so its own core.
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/I2C/Controller/QueueCore.hpp>
#include <kvasir/Devices/I2C/Controller/Request.hpp>
#include <support/FakeClock.hpp>
#include <utility>

namespace Kvasir::Test::I2cQueue {
using namespace std::chrono_literals;
using Kvasir::I2C::QueueState;

/// LineRecovery's static API, scripted.
template<typename Tag>
struct FakeRecoveryOf {
    enum class TickResult { busy, idle, needsReinit };

    static inline TickResult            nextTick{TickResult::idle};
    static inline bool                  sdaHigh{true};
    static inline bool                  stuck{};
    static inline bool                  active{};
    static inline int                   begins{};
    static inline int                   defers{};
    static inline FakeClock::time_point sickUntil{};

    static TickResult tick(FakeClock::time_point) {
        auto const r = nextTick;
        if(r == TickResult::needsReinit) { active = false; }
        nextTick = TickResult::idle;
        return r;
    }

    static void resetState() {}

    static void begin() {
        ++begins;
        active = true;
    }

    static bool isActive() { return active; }

    static bool checkBusStuck(FakeClock::time_point) { return stuck; }

    static bool isPastSettle(FakeClock::time_point now) { return now >= sickUntil; }

    static void deferSettle(FakeClock::time_point until) {
        ++defers;
        sickUntil = until;
    }

    static void noteBusFree() {}

    static bool sdaIsHigh() { return sdaHigh; }

    static bool sclIsHigh() { return true; }

    static std::uint32_t recoveries() { return 0; }

    static std::uint32_t forcedStops() { return 0; }

    static int clocksLeft() { return 0; }

    static void reset() {
        nextTick  = TickResult::idle;
        sdaHigh   = true;
        stuck     = false;
        active    = false;
        begins    = 0;
        defers    = 0;
        sickUntil = {};
    }
};

using FakeRecovery = FakeRecoveryOf<void>;

/// What the interrupt finds when the test raises it.
enum class Wire { nothing, ok, nack, fault, stopped };

struct NoTiming {};

/// The core over a policy, named where the policy is complete (a member alias would check the concept on an
/// incomplete class).
template<typename H>
using CoreOf = Kvasir::I2C::QueueCore<H, FakeClock, 4, 16>;

/// `f` with tickets and deadlines as `track` says: on a chip bus Cancellable/Deadlines ARE queueFeatures.cancel/.deadlines
/// (QueuedShape), and the shared front (Bus/QueuedFront.hpp) gates cancel() on Features.cancel.
consteval Kvasir::Bus::QueueFeatures tracking(Kvasir::Bus::QueueFeatures f,
                                              bool                       track) {
    f.cancel    = track;
    f.deadlines = track;
    return f;
}

/// `F`: the bus's optional parts (queueFeatures); all on by default (the counters too, unlike a chip bus's default);
/// .cancel/.deadlines follow `Track`.
template<bool Track,
         typename Tag                 = void,
         Kvasir::Bus::QueueFeatures F = Kvasir::Bus::QueueFeatures{.counters = true}>
struct FakeHw {
    static constexpr unsigned                   instance       = 0;
    static constexpr bool                       cancellable    = Track;
    static constexpr bool                       deadlines      = Track;
    static constexpr bool                       perDeviceClock = false;
    static constexpr Kvasir::Bus::QueueFeatures busFeatures    = tracking(F, Track);
    using Recovery                                             = FakeRecoveryOf<Tag>;
    using Request                                              = Kvasir::I2C::Queue::
      QueuedRequestFor<16, FakeClock::time_point, Track, Track, false, NoTiming, NoTiming{}>;

    /// The block's registers, as far as a fake has any: a marker the test checks arrives as `hw`.
    struct Snapshot {
        std::uint32_t marker{};
    };

    static inline bool lineMasked{};
    /// Hw::unmask() reached the line with a masked section still open.
    static inline bool unmaskedNested{};
    /// Hw::mask() calls since clear().
    static inline int maskCalls{};

    /// Read through a function: clang counts a read of a class template's static from outside as no use.
    static bool nested() { return unmaskedNested; }

    static inline int starts{};

    static int startCount() { return starts; }

    static inline bool startedUnmasked{};
    static inline bool inIsr{};
    static inline bool startedInCallback{};
    static inline bool inCallback{};
    /// A start while a request was still on the wire (state_ not idle).
    static inline bool startedOverActive{};
    static inline int  finishes{};
    static inline int  stops{};
    static inline int  timeoutAborts{};
    static inline int  reinits{};
    static inline bool busy{};
    static inline Wire next{Wire::nothing};

    static void mask() {
        ++maskCalls;
        lineMasked = true;
    }

    static void unmask() {
        if(CoreOf<FakeHw>::mask_.depth() != 0) { unmaskedNested = true; }
        lineMasked = false;
    }

    static void reinit() { ++reinits; }

    static auto transferTimeout(std::size_t bytes,
                                Request const&) {
        return std::chrono::microseconds{100 * (bytes + 1)} + 10ms;
    }

    static void start(Request const& r) {
        ++starts;
        if(!lineMasked && !inIsr) { startedUnmasked = true; }
        if(inCallback) { startedInCallback = true; }
        if(CoreOf<FakeHw>::state_ != QueueState::idle) { startedOverActive = true; }
        CoreOf<FakeHw>::state_ = r.sendBytes() != 0 ? QueueState::sending : QueueState::receiving;
    }

    static void finish() { ++finishes; }

    static void requestStop() { ++stops; }

    static void timeoutAbort() { ++timeoutAborts; }

    static bool busActive() { return busy; }

    static void logTimeout(FakeClock::time_point) {}

    static Snapshot snapshot() { return {42}; }

    static inline int lastTimeoutLogs{};

    static void logLastTimeout(Kvasir::I2C::TimeoutSnapshot<Snapshot> const&) { ++lastTimeoutLogs; }

    static inline std::uint32_t exhausted{};

    static std::uint32_t waitsExhausted() { return exhausted; }

    static std::uint32_t clockSwitches() { return 0; }

    static void onIsr() {
        if(CoreOf<FakeHw>::state_ == QueueState::idle) {
            ++CoreOf<FakeHw>::spuriousIsr_;
            return;
        }
        auto const w = std::exchange(next, Wire::nothing);
        switch(w) {
        case Wire::nothing: ++CoreOf<FakeHw>::spuriousIsr_; return;
        case Wire::ok:      CoreOf<FakeHw>::complete(CoreOf<FakeHw>::Result::ok); return;
        case Wire::nack:    CoreOf<FakeHw>::complete(CoreOf<FakeHw>::Result::nack); return;
        case Wire::fault:   CoreOf<FakeHw>::complete(CoreOf<FakeHw>::Result::failed); return;
        case Wire::stopped:
            if constexpr(Track) {
                if(CoreOf<FakeHw>::cancelPending_) {
                    CoreOf<FakeHw>::complete(CoreOf<FakeHw>::stopAs_);
                }
            }
            return;
        }
    }

    /// The interrupt with `w` on the wire, as the NVIC would take it (only while unmasked).
    static void raise(Wire w) {
        check(!lineMasked, "the interrupt is raised only while its line is unmasked");
        next  = w;
        inIsr = true;
        CoreOf<FakeHw>::onIsr();
        inIsr = false;
    }

    static void clear() {
        lineMasked = false;
        starts = finishes = stops = timeoutAborts = reinits = lastTimeoutLogs = maskCalls = 0;
        exhausted                                                                         = 0;
        unmaskedNested = startedUnmasked = startedInCallback = inCallback = busy = startedOverActive
          = false;
        next = Wire::nothing;
    }
};

}   // namespace Kvasir::Test::I2cQueue
