#pragma once

#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Util/Config.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace Kvasir {
/// Events from a push button. Without `debouncePress` / `debounceRelease` in the config every
/// edge the interrupt sees is an event at once - right for a contact that is debounced in
/// hardware, wrong for a bare one: a tactile switch measured on a board
/// lost contact for 0.1 to 8 ms in the middle of a press, several times per press,
/// and each time was a release and a new hit.
///
/// With both in the config, handler() polls the pin instead (call it every turn of the main
/// loop; the edge interrupt is not used): a level counts once it has been read without a
/// break for that long, and the event carries the time the level was first read. The two are
/// separate because they guard against different things - debouncePress against a spike on
/// the wire (a few ms are plenty, and the hit is late by as much), debounceRelease against
/// the contact's drop-outs (longer than the longest one; a release is late by as much).
///
/// The events are one `Event::Type` (`hit`, `releaseShort`, `longPress`, `releaseLong`) whatever
/// the config enables; a disabled one is never delivered.
template<typename Clock, typename Pin, std::size_t EventQSize, typename UserConfig>
struct PushButton {
    struct Config : UserConfig {
        /// Both debounce times given: handler() polls the pin, the edge interrupt is not used.
        static constexpr bool debounce = requires {
            UserConfig::debouncePress;
            UserConfig::debounceRelease;
        };

        static constexpr bool hasLongPressTime = requires { UserConfig::longPressTime; };

        static constexpr auto useLong = KVASIR_CONFIG_OR(UserConfig, useLong, false);

        static constexpr auto useLongRelease = KVASIR_CONFIG_OR(UserConfig, useLongRelease, false);

        static constexpr auto useHit = KVASIR_CONFIG_OR(UserConfig, useHit, false);

        static constexpr auto useShortRelease
          = KVASIR_CONFIG_OR(UserConfig, useShortRelease, false);

        static constexpr auto useTime = KVASIR_CONFIG_OR(UserConfig, useTime, false);

        static constexpr auto invert = KVASIR_CONFIG_OR(UserConfig, invert, false);
    };

    using TimePoint = typename Clock::time_point;
    using dt        = typename Clock::duration;

    static constexpr bool isConfigValid() {
        if(Config::useLong || Config::useLongRelease) {
            if(!Config::hasLongPressTime) { return false; }
        }
        if(Config::hasLongPressTime) {
            if(!Config::useLong && !Config::useLongRelease) { return false; }
        }
        if(!Config::useHit && !Config::useLong && !Config::useShortRelease
           && !Config::useLongRelease)
        {
            return false;
        }
        return true;
    }

    static_assert(
      isConfigValid(),
      "a PushButton config enables at least one event (useHit, useShortRelease, useLong, "
      "useLongRelease), and longPressTime exactly when it enables useLong or useLongRelease");

    struct EventBaseTime {
        TimePoint time{};
    };

    struct EventBaseNoTime {};

    using EventBase = std::conditional_t<Config::useTime, EventBaseTime, EventBaseNoTime>;

    /// Every event a button has; a config's disabled ones are never delivered.
    enum class Type : std::uint8_t { hit, releaseShort, longPress, releaseLong };

    struct Event : EventBase {
        using Type = PushButton::Type;
        Type type{};
    };

    static inline Kvasir::Atomic::Queue<Event, EventQSize> queue{};

    using lastTime_t = std::conditional_t<
      Config::useLong,
      std::atomic<TimePoint>,
      std::conditional_t<Config::useTime || Config::useLongRelease, TimePoint, bool>>;

    static inline lastTime_t lastTime{};

    static inline std::atomic<bool> isHit{};

    static bool readPin() {
        if constexpr(Config::invert) {
            return !apply(read(Pin{}));
        } else {
            return apply(read(Pin{}));
        }
    }

    // the debounced button, main() only
    static inline bool      stable{};     ///< the level that counts
    static inline bool      changing{};   ///< the pin reads the other one ...
    static inline TimePoint changeAt{};   ///< ... since then
    static inline TimePoint pressedAt{};
    static inline bool      longReported{};

    template<typename Callback>
    static void call(Callback&            cb,
                     typename Event::Type type,
                     TimePoint            time) {
        if constexpr(Config::useTime) {
            cb(type, time);
        } else {
            cb(type);
        }
    }

    template<typename Callback>
    static void debouncedHandler(Callback& cb) {
        auto const now = Clock::now();
        bool const raw = readPin();
        if(raw == stable) {
            changing = false;
        } else if(!changing) {
            changing = true;
            changeAt = now;
        } else if(now - changeAt
                  >= (raw ? dt{UserConfig::debouncePress} : dt{UserConfig::debounceRelease}))
        {
            stable   = raw;
            changing = false;
            if(raw) {
                pressedAt    = changeAt;
                longReported = false;
                if constexpr(Config::useHit) { call(cb, Event::Type::hit, changeAt); }
            } else {
                if constexpr(Config::useLongRelease) {
                    if(changeAt - pressedAt > Config::longPressTime) {
                        call(cb, Event::Type::releaseLong, changeAt);
                        return;
                    }
                }
                if constexpr(Config::useShortRelease) {
                    call(cb, Event::Type::releaseShort, changeAt);
                }
            }
            return;
        }
        if constexpr(Config::useLong) {
            if(stable && !longReported && now > pressedAt + Config::longPressTime) {
                longReported = true;
                call(cb, Event::Type::longPress, now);
            }
        }
    }

    static void edgeCallback() {
        if constexpr(Config::debounce) { return; }
        bool const pin = readPin();
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
                    auto const elapsed = now - lastTime.load(std::memory_order_relaxed);
                    lastTime.store(now, std::memory_order_relaxed);
                    return elapsed;
                } else {
                    auto const elapsed = now - lastTime;
                    lastTime           = now;
                    return elapsed;
                }

            } else {
                return false;
            }
        }();

        auto const pushTimed = [&](auto ee) {
            if constexpr(Config::useTime) {
                Event const e{{now}, ee};
                queue.push(e);
            } else {
                Event const e{.type = ee};
                queue.push(e);
            }
        };

        if(pin) {
            if constexpr(Config::useHit) { pushTimed(Event::Type::hit); }
            if constexpr(Config::useLong) { isHit.store(true, std::memory_order_relaxed); }
        } else {
            if constexpr(Config::useLong) { isHit.store(false, std::memory_order_relaxed); }

            if constexpr(Config::useLongRelease) {
                if(diff > Config::longPressTime) {
                    pushTimed(Event::Type::releaseLong);
                } else {
                    if constexpr(Config::useShortRelease) { pushTimed(Event::Type::releaseShort); }
                }
            } else {
                if constexpr(Config::useShortRelease) { pushTimed(Event::Type::releaseShort); }
            }
        }
    }

    template<typename Callback>
    static void handler(Callback cb) {
        if constexpr(Config::debounce) {
            debouncedHandler(cb);
            return;
        }
        if(Event e; queue.popInto(e)) {
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
                            cb(Event::Type::longPress, now);
                        } else {
                            cb(Event::Type::longPress);
                        }
                    }
                }
            }
        }
    }
};
}   // namespace Kvasir
