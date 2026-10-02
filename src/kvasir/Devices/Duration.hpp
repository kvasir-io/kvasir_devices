#pragma once

#include <chrono>
#include <cstdint>

namespace Kvasir {

/// A driver's or its config's time as milliseconds (or as `To`). Only a std::chrono duration
/// is taken -- a bare integer does not say what it counts -- and only one that converts without
/// losing precision, so `static constexpr auto RetryDelay = 5;` in a Config is a compile error
/// rather than five of whatever unit the driver happens to use.
template<typename To = std::chrono::milliseconds,
         typename Rep,
         typename Period>
constexpr To asDuration(std::chrono::duration<Rep,
                                              Period> d) {
    return d;
}

/// Milliseconds in 32 bits (up to 49.7 days), for a time the engine keeps many copies of - a
/// script step's delay, a read group's period: half the bytes of std::chrono::milliseconds and
/// only 4-byte aligned, so it packs with the small fields around it. Built from any
/// std::chrono::milliseconds implicitly, and compares and adds with it as it is.
using Millis32 = std::chrono::duration<std::uint32_t, std::milli>;

}   // namespace Kvasir
