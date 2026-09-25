#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

namespace Kvasir::I2C {

/// One request in flight on a queued bus, and how its result reaches the loop: the bus's
/// callback runs in the interrupt and only stamps the time and stores the result; the
/// engine's next turn takes it. Nothing else crosses from the interrupt to the loop, which
/// is what lets a Device keep every counter it has as plain data.
///
/// A callback carries the generation it was issued in: a request that was given up on
/// (clear()) may still complete later, and its answer must not be taken for the next
/// request's. The same guard a scan's ProbeSlot has.
///
///     if(I2c::submit(request /* with pending_.callback() */)) { inFlight_ = true; }
///     ...
///     switch(pending_.take()) {
///     case Outcome::running:         break;
///     case Outcome::ok:              ...
///     case Outcome::notAcknowledged: ...   // the device did not answer
///     case Outcome::failed:          ...   // a bus fault: says nothing about the device
///     }
template<typename I2c, typename Clock>
struct Pending {
    using TimePoint = typename Clock::time_point;

    enum class Outcome : std::uint8_t { running, ok, notAcknowledged, failed };

    /// The callback to hand the bus: interrupt context, stamps and stores. Its size is
    /// what the bus's CallbackSize has to hold (Device::CallbackBytes).
    auto callback() {
        auto const gen = generation_.load(std::memory_order_relaxed);
        return [this, gen](typename I2c::Result r) { complete(gen, r); };
    }

    void complete(std::uint32_t        gen,
                  typename I2c::Result r) {
        if(gen != generation_.load(std::memory_order_relaxed)) { return; }   // given up on
        stamp_             = Clock::now();
        auto const outcome = r == I2c::Result::succeeded       ? Outcome::ok
                           : r == I2c::Result::notAcknowledged ? Outcome::notAcknowledged
                                                               : Outcome::failed;
        result_.store(outcome, std::memory_order_relaxed);
        done_.store(true, std::memory_order_release);
    }

    /// Loop context: the outcome once, `running` until the callback has run.
    Outcome take() {
        if(!done_.load(std::memory_order_acquire)) { return Outcome::running; }
        done_.store(false, std::memory_order_relaxed);
        return result_.load(std::memory_order_relaxed);
    }

    /// Forget a request that is being abandoned (a reset while one is in flight, the
    /// in-flight net): its callback, should it still come, is ignored.
    void clear() {
        generation_.fetch_add(1, std::memory_order_relaxed);
        done_.store(false, std::memory_order_relaxed);
    }

    /// When the last completion arrived, on the clock; written before the release-store,
    /// read after the acquire-load.
    [[nodiscard]] TimePoint stamp() const { return stamp_; }

private:
    std::atomic<bool>          done_{false};
    std::atomic<Outcome>       result_{Outcome::failed};
    std::atomic<std::uint32_t> generation_{0};
    TimePoint                  stamp_{};
};

}   // namespace Kvasir::I2C
