/// RotaryEncoder: one edge per detent, its direction from the two pins, the acceleration curve - over a plain
/// counter and over a SaturatingValue, whose own operators must decide what a step past a bound does (a fast turn
/// down past 0 stays at 0, it does not wrap to the maximum).
#include <chrono>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/RotaryEncoder.hpp>
#include <kvasir/Util/SaturatingValue.hpp>
#include <support/FakeClock.hpp>

using namespace Kvasir::Test;

namespace {
using Clock = FakeClockT<std::chrono::microseconds>;

/// The two encoder lines as the edge callback reads them: equal = a step down.
namespace Lines {
    inline bool a = false;
    inline bool b = false;

    struct PinA {};

    struct PinB {};

    struct ReadBoth {};

    struct Levels {
        bool a;
        bool b;

        template<std::size_t I>
        [[nodiscard]] bool get() const {
            if constexpr(I == 0) {
                return a;
            } else {
                return b;
            }
        }
    };

    constexpr ReadBoth read(PinA,
                            PinB) {
        return {};
    }

    inline Levels apply(ReadBoth) { return {a, b}; }
}   // namespace Lines

/// One edge `ago` after the last one, turning `down` or up.
template<typename E>
void edge(bool                      down,
          std::chrono::microseconds ago) {
    Lines::a = true;
    Lines::b = down;
    Clock::advance(ago);
    E::edgeCallback();
}

struct Plain {};

struct Accelerated {
    static constexpr bool useAcceleration = true;
};

using Saturating = Kvasir::SaturatingValue<std::uint16_t, 0, 512>;

constexpr std::chrono::microseconds slow{100'000};   // past every step of the curve: one detent
constexpr std::chrono::microseconds fast{1'000};     // the curve's fastest step: 20 detents

void plainCounter() {
    testCase("int counter: one detent per edge, both ways");
    using E = Kvasir::RotaryEncoder<Clock, Lines::PinA, Lines::PinB, int, Plain>;
    E::cnt  = 0;
    edge<E>(false, slow);
    edge<E>(false, fast);   // no acceleration configured: still one
    checkEq(E::cnt.load(), 2, "two up");
    edge<E>(true, slow);
    edge<E>(true, slow);
    edge<E>(true, slow);
    checkEq(E::cnt.load(), -1, "three down, below 0");
}

void accelerated() {
    testCase("int counter: the default acceleration curve");
    using E = Kvasir::RotaryEncoder<Clock, Lines::PinA, Lines::PinB, int, Accelerated>;
    E::cnt  = 0;
    edge<E>(false, slow);
    checkEq(E::cnt.load(), 1, "slow: one");
    edge<E>(false, fast);
    checkEq(E::cnt.load(), 21, "under 2.5 ms: 20");
    edge<E>(true, std::chrono::microseconds{7'000});
    checkEq(E::cnt.load(), 16, "under 10 ms: 5");
    edge<E>(true, fast);
    checkEq(E::cnt.load(), -4, "down past 0 by 20");
}

void unsignedCounter() {
    testCase("uint16_t counter: wraps like the compound assignment");
    using E = Kvasir::RotaryEncoder<Clock, Lines::PinA, Lines::PinB, std::uint16_t, Accelerated>;
    E::cnt  = 5;
    edge<E>(true, slow);
    edge<E>(true, fast);
    checkEq(E::cnt.load(), std::uint16_t{65520}, "5 - 1 - 20 wraps");
}

void saturating() {
    testCase("SaturatingValue: a fast turn past a bound stops at it");
    using E = Kvasir::RotaryEncoder<Clock, Lines::PinA, Lines::PinB, Saturating, Accelerated>;
    E::cnt  = Saturating{5};
    edge<E>(true, slow);
    checkEq(E::cnt.load().value(), 4, "one down");
    edge<E>(true, fast);
    checkEq(E::cnt.load().value(), 0, "4 - 20 saturates at 0, not at the maximum");
    edge<E>(true, fast);
    checkEq(E::cnt.load().value(), 0, "stays at 0");
    edge<E>(false, slow);
    checkEq(E::cnt.load().value(), 1, "and comes back up");

    E::cnt = Saturating{500};
    edge<E>(false, slow);
    edge<E>(false, fast);
    checkEq(E::cnt.load().value(), 512, "501 + 20 saturates at 512");
    edge<E>(true, slow);
    checkEq(E::cnt.load().value(), 511, "one down from the top");
}

}   // namespace

int main() {
    plainCounter();
    accelerated();
    unsignedCounter();
    saturating();
    return finish();
}
