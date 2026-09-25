// Every description that can tell its part from another before it writes anything does so: a part
// that only shares the address is turned down, and nothing is written to it.
//
// Two ways a description does that. One is its `Identity` (RegisterCheck.hpp), which the engine
// reads and compares first in every bring-up. The other, for a part with no identity register, is
// `Step::identify()` behind the reads its setup() decides on. Walked over chips/Every.hpp, so a
// new description is held to it without being listed here. Left out by construction: a
// description that has to write before it can identify (the MPR121 resets the part to tell it by
// its reset values; the Sensirion parts are asked for their serial number by command).
#include "Harness.hpp"

#include <chrono>
#include <cstddef>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Every.hpp>
#include <span>
#include <string_view>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

/// The description decides before its first write: it has an Identity, or an identify step with
/// no write in front of it.
template<typename Chip>
constexpr bool decidesBeforeWriting() {
    if constexpr(Device<FakeBus, FakeClock, Chip>::HasIdentity) {
        return true;
    } else if constexpr(requires { std::span<Step const>{Chip::Init}; }) {
        for(auto const& step : Chip::Init) {
            if(step.kind == Step::Kind::identify) { return true; }
            if(step.kind == Step::Kind::write) { return false; }
        }
        return false;
    } else {
        return false;
    }
}

/// A description whose identify step cannot tell a part that reads as zeros from its own: the
/// RV-8803 has no identity register, and what its setup() looks at -- flag and control bits that
/// must be clear -- is clear in zeros too.
template<typename Chip>
constexpr bool ZerosLookLikeIt = false;
template<>
constexpr bool ZerosLookLikeIt<Chips::Rv8803<>> = true;

std::size_t held = 0;

/// A part that acknowledges everything and reads as zeros is nobody's identity.
template<typename Chip>
void foreignPartIsNotWrittenTo() {
    if constexpr(decidesBeforeWriting<Chip>() && !ZerosLookLikeIt<Chip>) {
        ++held;
        fresh();
        FakeBus::respond = zeros;
        Device<FakeBus, FakeClock, Chip> d{};
        runFor(d, 3s);
        std::size_t writes = 0;
        for(auto const& t : FakeBus::log) { writes += t.isWrite() ? 1U : 0U; }
        check(!d.identified() && !d.answering() && d.unidentified() >= 1, Chip::Name.data());
        checkEq(writes, std::size_t{0}, Chip::Name.data());
    }
}

template<typename L>
struct ForEach;

template<typename... Cs>
struct ForEach<List<Cs...>> {
    static void run() { (foreignPartIsNotWrittenTo<Cs>(), ...); }
};

void all() {
    testCase("identify: a foreign part at the address is turned down before anything is written");
    ForEach<Chips::Every>::run();
    check(held >= 45, "the descriptions that decide before writing: at least the 45 of 2026-09-18");
}

}   // namespace

int main() {
    all();
    return finish();
}
