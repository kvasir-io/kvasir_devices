#pragma once

#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Config.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <type_traits>

namespace Kvasir {

/// One step of a rotary encoder's acceleration curve: an edge that follows the previous one
/// within `within` counts `steps` detents.
struct RotaryAcceleration {
    std::chrono::microseconds within{};
    std::size_t               steps{};
};

/// A quadrature rotary encoder counted from an edge interrupt on PinA. UserConfig may
/// have `useAcceleration` (false) and, with it, `acceleration`: the curve as an array of
/// RotaryAcceleration, fastest first; the default is 20 detents under 2.5 ms, 10 under
/// 5 ms, 5 under 10 ms, 2 under 20 ms.
template<typename Clock, typename PinA, typename PinB, typename ValueType, typename UserConfig>
struct RotaryEncoder {
    using type      = ValueType;
    using TimePoint = typename Clock::time_point;
    static inline std::atomic<ValueType> cnt{};
    static inline TimePoint              lastTime{};

    static constexpr std::array<RotaryAcceleration, 4> DefaultAcceleration{
      {{std::chrono::microseconds{2500}, 20},
       {std::chrono::microseconds{5000}, 10},
       {std::chrono::microseconds{10000}, 5},
       {std::chrono::microseconds{20000}, 2}}
    };

    struct Config : UserConfig {
        static constexpr auto useAcceleration
          = KVASIR_CONFIG_OR(UserConfig, useAcceleration, false);

        static constexpr auto acceleration
          = KVASIR_CONFIG_OR(UserConfig, acceleration, DefaultAcceleration);
    };

    static void edgeCallback() {
        auto const pins = apply(read(PinA{}, PinB{}));
        auto const now  = Clock::now();
        auto       cnt2 = cnt.load(std::memory_order_relaxed);
        auto const diff = now - lastTime;
        lastTime        = now;

        std::size_t addValue{1};
        if constexpr(Config::useAcceleration) {
            for(auto const& step : Config::acceleration) {
                if(step.within > diff) {
                    addValue = step.steps;
                    break;
                }
            }
        }

        bool const down = Kvasir::Register::get<0>(pins) == Kvasir::Register::get<1>(pins);
        if constexpr(std::is_arithmetic_v<ValueType>) {
            // what `cnt2 -= addValue` does, spelled out: the arithmetic in the common type of the
            // two (std::size_t for a 32-bit counter: wraps, no signed overflow), then back to ValueType
            using Wide = decltype(cnt2 + addValue);
            cnt2       = down ? static_cast<ValueType>(static_cast<Wide>(cnt2) - addValue)
                              : static_cast<ValueType>(static_cast<Wide>(cnt2) + addValue);
        } else if(down) {
            // a class type (SaturatingValue) has its own operators: they saturate at its bounds,
            // which the spelled-out form would skip (5 - 20 would wrap and clamp to the maximum)
            cnt2 -= addValue;
        } else {
            cnt2 += addValue;
        }
        cnt.store(cnt2, std::memory_order_relaxed);
    }
};
}   // namespace Kvasir
