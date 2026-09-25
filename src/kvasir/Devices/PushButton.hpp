#pragma once

#include "kvasir/Atomic/Queue.hpp"

#include <cstdint>
#include <type_traits>

namespace Kvasir {
/// Events from a push button. Without `debouncePress` / `debounceRelease` in the config every
/// edge the interrupt sees is an event at once - right for a contact that is debounced in
/// hardware, wrong for a bare one: a tactile switch measured on the water_mix board
/// (2026-09-19) lost contact for 0.1 to 8 ms in the middle of a press, several times per press,
/// and each time was a release and a new hit.
///
/// With both in the config, handler() polls the pin instead (call it every turn of the main
/// loop; the edge interrupt is not used): a level counts once it has been read without a
/// break for that long, and the event carries the time the level was first read. The two are
/// separate because they guard against different things - debouncePress against a spike on
/// the wire (a few ms are plenty, and the hit is late by as much), debounceRelease against
/// the contact's drop-outs (longer than the longest one; a release is late by as much).
template<typename Clock, typename Pin, std::size_t EventQSize, typename UserConfig>
struct PushButton {
    struct Config : UserConfig {
        static constexpr auto debounce = [] {
            if constexpr(requires {
                             UserConfig::debouncePress;
                             UserConfig::debounceRelease;
                         })
            {
                return true;
            } else {
                return false;
            }
        }();

        static constexpr auto hasLongPressTime = [] {
            if constexpr(requires { UserConfig::longPressTime; }) {
                return true;
            } else {
                return false;
            }
        }();

        static constexpr auto useLong = [] {
            if constexpr(requires { UserConfig::useLong; }) {
                return UserConfig::useLong;
            } else {
                return false;
            }
        }();

        static constexpr auto useLongRelease = [] {
            if constexpr(requires { UserConfig::useLongRelease; }) {
                return UserConfig::useLongRelease;
            } else {
                return false;
            }
        }();

        static constexpr auto useHit = [] {
            if constexpr(requires { UserConfig::useHit; }) {
                return UserConfig::useHit;
            } else {
                return false;
            }
        }();

        static constexpr auto useShortRelease = [] {
            if constexpr(requires { UserConfig::useShortRelease; }) {
                return UserConfig::useShortRelease;
            } else {
                return false;
            }
        }();

        static constexpr auto useTime = [] {
            if constexpr(requires { UserConfig::useTime; }) {
                return UserConfig::useTime;
            } else {
                return false;
            }
        }();

        static constexpr auto invert = [] {
            if constexpr(requires { UserConfig::invert; }) {
                return UserConfig::invert;
            } else {
                return false;
            }
        }();
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

    static_assert(isConfigValid(),
                  "Config Invalid");

    struct EventBaseTime {
        TimePoint time{};
    };

    struct EventBaseNoTime {};

    using EventBase = std::conditional_t<Config::useTime, EventBaseTime, EventBaseNoTime>;

    enum class EventTypeHslr : std::uint8_t { hit, releaseShort, longPress, releaseLong };

    enum class EventTypeSlr : std::uint8_t { releaseShort, longPress, releaseLong };
    enum class EventTypeHlr : std::uint8_t { hit, longPress, releaseLong };
    enum class EventTypeHsr : std::uint8_t { hit, releaseShort, releaseLong };
    enum class EventTypeHsl : std::uint8_t { hit, releaseShort, longPress };

    enum class EventTypeLr : std::uint8_t { longPress, releaseLong };
    enum class EventTypeSr : std::uint8_t { releaseShort, releaseLong };
    enum class EventTypeSl : std::uint8_t { releaseShort, longPress };
    enum class EventTypeHr : std::uint8_t { hit, releaseLong };
    enum class EventTypeHl : std::uint8_t { hit, longPress };
    enum class EventTypeHs : std::uint8_t { hit, releaseShort };

    enum class EventTypeH : std::uint8_t { hit };
    enum class EventTypeS : std::uint8_t { releaseShort };
    enum class EventTypeL : std::uint8_t { longPress };
    enum class EventTypeR : std::uint8_t { releaseLong };

    static constexpr auto SelectedEventType = []() {
        if constexpr(Config::useHit && Config::useShortRelease && Config::useLong
                     && Config::useLongRelease)
        {
            return EventTypeHslr{};
        } else if constexpr(Config::useShortRelease && Config::useLong && Config::useLongRelease) {
            return EventTypeSlr{};
        } else if constexpr(Config::useHit && Config::useLong && Config::useLongRelease) {
            return EventTypeHlr{};
        } else if constexpr(Config::useHit && Config::useShortRelease && Config::useLongRelease) {
            return EventTypeHsr{};
        } else if constexpr(Config::useHit && Config::useShortRelease && Config::useLong) {
            return EventTypeHsl{};
        } else if constexpr(Config::useLong && Config::useLongRelease) {
            return EventTypeLr{};
        } else if constexpr(Config::useShortRelease && Config::useLongRelease) {
            return EventTypeSr{};
        } else if constexpr(Config::useHit && Config::useLongRelease) {
            return EventTypeHr{};
        } else if constexpr(Config::useHit && Config::useLong) {
            return EventTypeHl{};
        } else if constexpr(Config::useHit && Config::useShortRelease) {
            return EventTypeHs{};
        } else if constexpr(Config::useShortRelease && Config::useLong) {
            return EventTypeSl{};
        } else if constexpr(Config::useHit) {
            return EventTypeH{};
        } else if constexpr(Config::useShortRelease) {
            return EventTypeS{};
        } else if constexpr(Config::useLong) {
            return EventTypeL{};
        } else if constexpr(Config::useLongRelease) {
            return EventTypeR{};
        }
    }();

    struct Event : EventBase {
        using Type = std::remove_cvref_t<decltype(SelectedEventType)>;
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
