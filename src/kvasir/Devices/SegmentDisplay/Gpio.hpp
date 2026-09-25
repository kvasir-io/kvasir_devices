#pragma once

#include "../SegmentDisplay.hpp"

#include <bitset>
#include <cstddef>

/// A SegmentDisplay backend on the controller's own pins (SegmentDisplay.hpp has the layers).
/// Pins are fast enough to scan, so this is the backend a `MultiplexedLayout` runs on.
namespace Kvasir::SegmentDisplay {

/// Output lines on pins, through a policy: `Pins::Count` lines, `Pins::write(line, Level)`, and
/// an optional `Pins::init()` run once from the constructor. `KvasirPins` is the policy for
/// Kvasir pin types; a test hands in one that records. Only the lines that changed are
/// written, so an update that changes nothing touches no pin.
template<typename Pins, typename Clock>
class GpioSegments {
public:
    using TimePoint = typename Clock::time_point;

    static constexpr std::size_t Outputs  = Pins::Count;
    static constexpr Scan        Scanning = Scan::supported;

    using Bits = std::bitset<Outputs>;

    GpioSegments() {
        if constexpr(requires { Pins::init(); }) { Pins::init(); }
    }

    bool show(Bits const& levels) {
        bool changed = false;
        for(std::size_t line = 0; line < Outputs; ++line) {
            if(known_ && levels.test(line) == shadow_.test(line)) { continue; }
            Pins::write(line, levels.test(line) ? Level::high : Level::low);
            changed = true;
        }
        shadow_ = levels;
        known_  = true;
        return changed;
    }

private:
    Bits shadow_{};
    bool known_{};
};

/// The pin policy for Kvasir pin types, line n the nth pin: every pin an output from the
/// constructor on. Include the part's Io header before this one.
template<typename... P>
struct KvasirPins {
    static constexpr std::size_t Count = sizeof...(P);

    static void init() { apply(makeOutput(P{})...); }

    static void write(std::size_t line,
                      Level       level) {
        std::size_t i = 0;
        (
          [&] {
              if(i++ != line) { return; }
              if(level == Level::high) {
                  apply(set(P{}));
              } else {
                  apply(clear(P{}));
              }
          }(),
          ...);
    }
};

/// Digits on pins: `GpioDisplay<KvasirPins<...>, Clock, Layout> digits{}`.
template<typename Pins, typename Clock, Layout auto LayoutV, typename Timing = DefaultDisplayTiming>
using GpioDisplay = Display<GpioSegments<Pins, Clock>, LayoutV, Timing>;

}   // namespace Kvasir::SegmentDisplay
