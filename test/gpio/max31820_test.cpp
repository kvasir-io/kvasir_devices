/// MAX31820 / DS18B20 driver over a scripted 1-Wire bus: link() is starting before the first reading, answering with
/// one, still starting after one or two failed transactions, absent from the third (absentAfter) and answering again
/// with the next good reading - the retry runs by itself.
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/Max31820.hpp>
#include <span>
#include <support/FakeClock.hpp>

using namespace Kvasir::Test;
using Kvasir::Link;

namespace {
using Clock = Kvasir::Test::FakeClockT<std::chrono::milliseconds>;

/// Every transaction completes at once, failed while `fail` is set; a read returns `scratchpad`.
struct FakeBus {
    static constexpr std::size_t BufferSize = 9;
    enum class OperationState : std::uint8_t { succeeded, failed, ongoing };

    static inline bool                     fail = false;
    static inline std::array<std::byte, 9> scratchpad{};
    static inline bool                     held = false;

    static bool acquire() {
        if(held) { return false; }
        held = true;
        return true;
    }

    static void release() { held = false; }

    template<std::size_t N>
    static void send(Clock::time_point,
                     std::array<std::byte,
                                N> const&) {}

    template<std::size_t N>
    static void sendReceive(Clock::time_point,
                            std::array<std::byte,
                                       N> const&,
                            std::size_t) {}

    static OperationState operationState(Clock::time_point) {
        return fail ? OperationState::failed : OperationState::succeeded;
    }

    template<std::size_t N>
    static void getReceivedBytes(std::array<std::byte,
                                            N>& out) {
        out = scratchpad;
    }
};

/// 25.0625 degC (0x0191), 12 bits, with its CRC.
void goodScratchpad() {
    std::array<std::byte, 9> s{std::byte{0x91},
                               std::byte{0x01},
                               std::byte{0x4B},
                               std::byte{0x46},
                               std::byte{0x7F},
                               std::byte{0xFF},
                               std::byte{0x0F},
                               std::byte{0x10},
                               std::byte{}};
    s[8] = std::byte{
      Kvasir::Dallas::crc8(Kvasir::Bytes{std::span<std::byte const>{s.data(), 8}}
      )
    };
    FakeBus::scratchpad = s;
}

using Sensor = Kvasir::Max31820<FakeBus, Clock>;

void turns(Sensor& s,
           int     ms) {
    for(int i = 0; i < ms; ++i) {
        Clock::advance(std::chrono::milliseconds{1});
        s.turn();
    }
}
}   // namespace

int main() {
    testCase(
      "Max31820 link(): starting, answering, absent after absentAfter failures, answering again");
    goodScratchpad();
    Sensor s{};
    check(s.link() == Link::starting, "starting before the first reading");
    turns(s, 900);   // Convert T, 800 ms, the read
    check(s.valid() && s.link() == Link::answering, "answering with a CRC-checked reading");
    FakeBus::fail     = true;
    auto const errors = s.errors();
    while(s.errors() < errors + 2) { turns(s, 1); }
    check(s.link() == Link::starting, "two failed transactions: starting, not absent yet");
    while(s.errors() < errors + Sensor::AbsentAfter) { turns(s, 1); }
    check(s.link() == Link::absent, "absent from the third failure in a row");
    FakeBus::fail = false;
    turns(s, 1200);   // the retry after retryDelay, Convert T, the read
    check(s.link() == Link::answering, "answering again, by itself");
    return finish();
}
