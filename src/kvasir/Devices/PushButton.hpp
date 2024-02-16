#pragma once

#include "kvasir/Atomic/Queue.hpp"

#include <cstdint>
#include <type_traits>

namespace Kvasir {
template<typename Clock, typename Pin, std::size_t EventQSize, typename Config_>
struct PushButton {
    struct Config : Config_ {
        static constexpr auto hasLongPressTime = [] {
            if constexpr(requires { Config_::longPressTime; }) {
                return true;
            } else {
                return false;
            }
        }();

        static constexpr auto useLong = [] {
            if constexpr(requires { Config_::useLong; }) {
                return Config_::useLong;
            } else {
                return false;
            }
        }();

        static constexpr auto useLongRelease = [] {
            if constexpr(requires { Config_::useLongRelease; }) {
                return Config_::useLongRelease;
            } else {
                return false;
            }
        }();

        static constexpr auto useHit = [] {
            if constexpr(requires { Config_::useHit; }) {
                return Config_::useHit;
            } else {
                return false;
            }
        }();

        static constexpr auto useShortRelease = [] {
            if constexpr(requires { Config_::useShortRelease; }) {
                return Config_::useShortRelease;
            } else {
                return false;
            }
        }();

        static constexpr auto useTime = [] {
            if constexpr(requires { Config_::useTime; }) {
                return Config_::useTime;
            } else {
                return false;
            }
        }();

        static constexpr auto invert = [] {
            if constexpr(requires { Config_::invert; }) {
                return Config_::invert;
            } else {
                return false;
            }
        }();
    };

    using tp = typename Clock::time_point;
    using dt = typename Clock::duration;

    static constexpr bool isConfigValid() {
        if(Config::useLong || Config::useLongRelease) {
            if(!Config::hasLongPressTime) {
                return false;
            }
        }
        if(Config::hasLongPressTime) {
            if(!Config::useLong && !Config::useLongRelease) {
                return false;
            }
        }
        if(
          !Config::useHit && !Config::useLong && !Config::useShortRelease
          && !Config::useLongRelease) {
            return false;
        }
        return true;
    }

    static_assert(isConfigValid(), "Config Invalid");

    struct EventBaseTime {
        tp time{};
    };
    struct EventBaseNoTime {};

    using EventBase = std::conditional_t<Config::useTime, EventBaseTime, EventBaseNoTime>;

    enum class Type_HSLR : std::uint8_t { Hit, Release_Short, Long, Release_Long };

    enum class Type_SLR : std::uint8_t { Release_Short, Long, Release_Long };
    enum class Type_HLR : std::uint8_t { Hit, Long, Release_Long };
    enum class Type_HSR : std::uint8_t { Hit, Release_Short, Release_Long };
    enum class Type_HSL : std::uint8_t { Hit, Release_Short, Long };

    enum class Type_LR : std::uint8_t { Long, Release_Long };
    enum class Type_SR : std::uint8_t { Release_Short, Release_Long };
    enum class Type_SL : std::uint8_t { Release_Short, Long };
    enum class Type_HR : std::uint8_t { Hit, Release_Long };
    enum class Type_HL : std::uint8_t { Hit, Long };
    enum class Type_HS : std::uint8_t { Hit, Release_Short };

    enum class Type_H : std::uint8_t { Hit };
    enum class Type_S : std::uint8_t { Release_Short };
    enum class Type_L : std::uint8_t { Long };
    enum class Type_R : std::uint8_t { Release_Long };

    static constexpr auto Type_ = []() {
        if constexpr(
          Config::useHit && Config::useShortRelease && Config::useLong && Config::useLongRelease) {
            return Type_HSLR{};
        } else if constexpr(Config::useShortRelease && Config::useLong && Config::useLongRelease) {
            return Type_SLR{};
        } else if constexpr(Config::useHit && Config::useLong && Config::useLongRelease) {
            return Type_HLR{};
        } else if constexpr(Config::useHit && Config::useShortRelease && Config::useLongRelease) {
            return Type_HSR{};
        } else if constexpr(Config::useHit && Config::useShortRelease && Config::useLong) {
            return Type_HSL{};
        } else if constexpr(Config::useLong && Config::useLongRelease) {
            return Type_LR{};
        } else if constexpr(Config::useShortRelease && Config::useLongRelease) {
            return Type_SR{};
        } else if constexpr(Config::useHit && Config::useLongRelease) {
            return Type_HR{};
        } else if constexpr(Config::useHit && Config::useLong) {
            return Type_HL{};
        } else if constexpr(Config::useHit && Config::useShortRelease) {
            return Type_HS{};
        } else if constexpr(Config::useHit) {
            return Type_H{};
        } else if constexpr(Config::useShortRelease) {
            return Type_S{};
        } else if constexpr(Config::useLong) {
            return Type_L{};
        } else if constexpr(Config::useLongRelease) {
            return Type_R{};
        }
    }();

    struct Event : EventBase {
        using Type = std::remove_cvref_t<decltype(Type_)>;
        Type type{};
    };

    static inline Kvasir::Atomic::Queue<Event, EventQSize> queue{};

    using lastTime_t = std::conditional_t<
      Config::useLong,
      std::atomic<tp>,
      std::conditional_t<Config::useTime || Config::useLongRelease, tp, bool>>;

    static inline lastTime_t lastTime{};

    using hit_t = std::conditional_t<Config::useLong, std::atomic<bool>, bool>;

    static inline std::atomic<bool> isHit{};

    static void edgeCallback() {
        bool const pin = []() {
            if constexpr(Config::invert) {
                return !apply(read(Pin{}));
            } else {
                return apply(read(Pin{}));
            }
        }();
        auto const now = []() {
            if constexpr(Config::useTime || Config::useLong || Config::useLongRelease) {
                return Clock::now();
            } else {
                return false;
            }
        }();
        auto const diff = [&]() {
            if constexpr(Config::useTime || Config::useLong || Config::useLongRelease) {
                if constexpr(Config::useLong) {
                    auto const diff_ = now - lastTime.load(std::memory_order_relaxed);
                    lastTime.store(now, std::memory_order_relaxed);
                    return diff_;
                } else {
                    auto const diff_ = now - lastTime;
                    lastTime         = now;
                    return diff_;
                }

            } else {
                return false;
            }
        }();

        auto pushTimed = [&](auto ee) {
            if constexpr(Config::useTime) {
                Event e{{now}, ee};
                queue.push(e);
            } else {
                Event e{.type = ee};
                queue.push(e);
            }
        };

        if(pin) {
            if constexpr(Config::useHit) {
                pushTimed(Event::Type::Hit);
            }
            if constexpr(Config::useLong) {
                isHit.store(true, std::memory_order_relaxed);
            }
        } else {
            if constexpr(Config::useLong) {
                isHit.store(false, std::memory_order_relaxed);
            }

            if constexpr(Config::useLongRelease) {
                if(diff > Config::longPressTime) {
                    pushTimed(Event::Type::Release_Long);
                } else {
                    if constexpr(Config::useShortRelease) {
                        pushTimed(Event::Type::Release_Short);
                    }
                }
            } else {
                if constexpr(Config::useShortRelease) {
                    pushTimed(Event::Type::Release_Short);
                }
            }
        }
    }
    template<typename Callback>
    static void handler(Callback cb) {
        if(Event e; queue.pop_into(e)) {
            if constexpr(Config::useTime) {
                cb(e.type, e.time);
            } else {
                cb(e.type);
            }
        } else {
            if constexpr(Config::useLong) {
                bool const hit = isHit.load(std::memory_order_relaxed);
                if(hit) {
                    auto const now  = Clock::now();
                    auto const last = lastTime.load(std::memory_order_relaxed);
                    if(now > last + Config::longPressTime) {
                        isHit.store(false, std::memory_order_relaxed);
                        if constexpr(Config::useTime) {
                            cb(Event::Type::Long, now);
                        } else {
                            cb(Event::Type::Long);
                        }
                    }
                }
            }
        }
    }
};
}   // namespace Kvasir
