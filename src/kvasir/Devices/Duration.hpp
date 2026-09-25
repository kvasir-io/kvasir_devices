#pragma once

#include <chrono>

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

}   // namespace Kvasir
