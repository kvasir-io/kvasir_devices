#pragma once

#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/using_literals.hpp"

#include <atomic>

namespace Kvasir {
template<typename Clock, typename pinA, typename pinB, typename ValueType, typename Config_>
struct RotaryEncoder {
    using type = ValueType;
    using tp   = typename Clock::time_point;
    static inline std::atomic<ValueType> cnt{};
    static inline tp                     lastTime{};

    struct Config : Config_ {
        static constexpr auto useAcceleration = [] {
            if constexpr(requires { Config_::useAcceleration; }) {
                return Config_::useAcceleration;
            } else {
                return false;
            }
        }();
    };

    static void edgeCallback() {
        auto const pins = apply(read(pinA{}, pinB{}));
        auto const now  = Clock::now();
        auto       cnt2 = cnt.load(std::memory_order_relaxed);
        auto const diff = now - lastTime;
        lastTime        = now;

        std::size_t addValue{1};
        if constexpr(Config::useAcceleration) {
            //TODO make configurable
            if(2500us > diff) {
                addValue = 20;
            } else if(5ms > diff) {
                addValue = 10;
            } else if(10ms > diff) {
                addValue = 5;
            } else if(20ms > diff) {
                addValue = 2;
            }
        }

        if(Kvasir::Register::get<0>(pins) == Kvasir::Register::get<1>(pins)) {
            cnt2 -= addValue;
        } else {
            cnt2 += addValue;
        }
        cnt.store(cnt2, std::memory_order_relaxed);
    }
};
}   // namespace Kvasir
