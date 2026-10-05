#pragma once

#include <array>
#include <cstdint>

namespace Kvasir {

/// A quadrature (Gray code) decoder for a mechanical rotary encoder: fed the levels of A and B
/// after every edge of either, it follows the four states and counts a detent only when the
/// knob arrives in a rest state. Plain code, no hardware: the edge interrupt reads the pins and
/// hands them to update().
///
/// Why both lines and a state machine instead of one edge of A: a contact that bounces moves
/// the state back and forth between two neighbours, and each step back takes back the step
/// before it - so bounce, a knob turned half way and let go, or one rocked on its detent counts
/// nothing. Counting one edge of A counts every bounce of A that happens to see B on the
/// other side.
///
/// StepsPerDetent: the quarter steps (state changes) from one detent to the next - 4 for an
/// encoder that does a whole quadrature cycle per click (its A and B rest at RestState in every
/// detent), 2 for half a cycle (they rest at RestState and its complement), 1 for one state
/// change per click. RestState is (A << 1) | B in a detent.
///
/// The count goes up for the sequence 00 -> 10 -> 11 -> 01 -> 00 (A leads B), the direction
/// RotaryEncoder's edge counting gives the same knob.
template<unsigned StepsPerDetent = 4, unsigned RestState = 0>
class QuadratureDecoder {
    static_assert(StepsPerDetent == 1 || StepsPerDetent == 2 || StepsPerDetent == 4,
                  "a detent is 1, 2 or 4 state changes");
    static_assert(RestState < 4,
                  "RestState is (A << 1) | B");

    /// The step of one change from state `from` to `to`, indexed (from << 2) | to: +1 forward,
    /// -1 back, 0 for no change and for a change of both lines at once (an edge was missed: the
    /// direction is unknown).
    static constexpr std::array<std::int8_t, 16> Steps{
      // to:  00  01  10  11
      0,
      -1,
      +1,
      0,   // from 00
      +1,
      0,
      0,
      -1,   // from 01
      -1,
      0,
      0,
      +1,   // from 10
      0,
      +1,
      -1,
      0,   // from 11
    };

    static constexpr bool isRest(unsigned state) {
        if constexpr(StepsPerDetent == 4) {
            return state == RestState;
        } else if constexpr(StepsPerDetent == 2) {
            return state == RestState || state == (RestState ^ 3U);
        } else {
            return true;
        }
    }

public:
    /// Starts from the levels the lines have now.
    constexpr explicit QuadratureDecoder(bool a,
                                         bool b)
      : state_{stateOf(a,
                       b)} {}

    constexpr QuadratureDecoder()
      : QuadratureDecoder{(RestState & 2U) != 0,
                          (RestState & 1U) != 0} {}

    /// The new levels of A and B; returns the detents this completes (usually 0 or +-1).
    /// A rest state reached with at least half a detent of steps behind it counts as one: one
    /// lost edge (a change of both lines, Steps 0) on the way still gives the click.
    constexpr int update(bool a,
                         bool b) {
        auto const next = stateOf(a, b);
        if(next == state_) { return 0; }
        auto const step = Steps[(state_ << 2U) | next];
        if(step == 0) { ++skipped_; }
        state_ = next;
        steps_ += step;
        if(!isRest(next)) { return 0; }
        constexpr int half    = static_cast<int>(StepsPerDetent) / 2;
        int const     rounded = steps_ >= 0 ? steps_ + half : steps_ - half;
        int const     detents
          = StepsPerDetent == 1 ? steps_ : rounded / static_cast<int>(StepsPerDetent);
        steps_ = 0;
        return detents;
    }

    /// Changes of both lines at once seen so far: each is an edge the interrupt came too late
    /// for. Not zero on a slowly turned knob = the decoder is not fast enough for this encoder.
    [[nodiscard]] constexpr std::uint32_t skipped() const { return skipped_; }

    /// (A << 1) | B as last seen.
    [[nodiscard]] constexpr unsigned state() const { return state_; }

private:
    static constexpr unsigned stateOf(bool a,
                                      bool b) {
        return (a ? 2U : 0U) | (b ? 1U : 0U);
    }

    unsigned      state_{};
    int           steps_{};
    std::uint32_t skipped_{};
};

}   // namespace Kvasir
