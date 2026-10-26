#pragma once
// Latency bookkeeping of the queued buses (I2C/Controller/QueueCore.hpp, SPI/QueueCore.hpp): a wait is kept in the
// clock's own ticks, 32 bits, saturated (2^32 ticks are 28 s or more: "very long" for a latency), and turned into
// microseconds only where it is read - converting where it is measured would be a 64-bit division per transfer or
// interrupt.
#include <algorithm>
#include <chrono>
#include <cstdint>

namespace Kvasir::Bus {
/// A duration in its own ticks, saturated at 32 bits (a negative one saturates too).
template<typename Rep,
         typename Period>
[[nodiscard]] constexpr std::uint32_t ticks32(std::chrono::duration<Rep,
                                                                    Period> d) {
    auto const n = static_cast<std::uint64_t>(d.count());
    return n > 0xFFFF'FFFFU ? 0xFFFF'FFFFU : static_cast<std::uint32_t>(n);
}

/// ticks32()'s ticks of Clock in microseconds.
template<typename Clock>
[[nodiscard]] constexpr std::uint32_t usOfTicks(std::uint32_t ticks) {
    return static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(typename Clock::duration{ticks})
        .count());
}

/// The longest so far: `slot` raised to `ticks` if that is longer.
constexpr void raiseMax(std::uint32_t& slot,
                        std::uint32_t  ticks) {
    slot = std::max(slot, ticks);
}
}   // namespace Kvasir::Bus
