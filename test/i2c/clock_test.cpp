/// Each device at its own clock on one bus (a bus with I2CConfig::perDeviceClock): the clock a
/// device is run at comes from Config::BusClock, else Chip::I2cMaxClock, else the bus's own
/// rate; every request of that device carries the bus's timing for it; a mux's switch writes
/// carry the mux's. And a bus without it pays nothing: no member in its request, and no
/// difference to what a device submits.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Mux.hpp>
#include <kvasir/Devices/I2C/chips/All.hpp>
#include <map>
#include <set>
#include <string_view>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;
namespace Units = Kvasir::Units;

namespace {

struct ClockTag {};

/// A request with the timing member a perDeviceClock bus has. The fake's "timing" is just the
/// rate: what matters here is which device's value arrives with which request.
struct TimedRequest : FakeBusRequest {
    std::uint32_t timing{};
};

/// The fake bus with a real driver's interface for the feature: `timing(hz)` at compile time,
/// capped at the bus's rate, and a request that carries it.
struct TimedBus {
    using Base    = FakeBusFor<ClockTag>;
    using Result  = FakeBusResult;
    using Request = TimedRequest;

    static constexpr std::uint32_t BaudRate   = 400'000;
    static constexpr std::size_t   QueueDepth = 8;

    static consteval std::uint32_t timing(std::uint32_t hz) { return std::min(hz, BaudRate); }

    /// Each submitted request's address and timing, in order.
    static inline std::vector<std::pair<std::uint8_t, std::uint32_t>> seen{};

    static bool submit(Request const& r) {
        bool const ok = Base::submit(r);
        if(ok) { seen.emplace_back(r.address, r.timing); }
        return ok;
    }

    static void reset() {
        Base::reset();
        seen.clear();
    }
};

/// A part that must not be clocked above 100 kHz, as a description says it.
struct SlowChip {
    static constexpr std::string_view Name          = "SLOW";
    static constexpr Address7         Address       = 0x21;
    static constexpr std::size_t      RegisterBytes = 1;
    static constexpr Units::Hertz     I2cMaxClock   = Units::hertz(100'000);
    static constexpr std::array       Init{Step::read({.reg = 0x00, .count = 1, .offset = 0})};

    struct Value {
        static constexpr auto       Period = std::chrono::milliseconds{10};
        static constexpr std::array Steps{Step::read({.reg = 0x01, .count = 1, .offset = 0})};
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
    };

    using Reads = List<Value>;
};

/// The same part with no limit of its own, at another address.
struct FreeChip {
    static constexpr std::string_view Name          = "FREE";
    static constexpr Address7         Address       = 0x22;
    static constexpr std::size_t      RegisterBytes = 1;
    static constexpr auto             Init          = SlowChip::Init;
    using Value                                     = SlowChip::Value;
    using Reads                                     = List<Value>;
};

struct At250k {
    static constexpr std::uint8_t Address  = 0x23;
    static constexpr Units::Hertz BusClock = Units::hertz(250'000);
};

/// Asks for more than the part may take: the part's own limit is the cap (checked below).
struct At50kOfSlow {
    static constexpr std::uint8_t Address  = 0x24;
    static constexpr Units::Hertz BusClock = Units::hertz(50'000);
};

using Slow      = Device<TimedBus, FakeClock, SlowChip>;
using Free      = Device<TimedBus, FakeClock, FreeChip>;
using Mid       = Device<TimedBus, FakeClock, FreeChip, At250k>;
using Slower    = Device<TimedBus, FakeClock, SlowChip, At50kOfSlow>;
using Clocked   = Kvasir::I2C::Bus<TimedBus, FakeClock, Slow, Free, Mid, Slower>;
using Unclocked = Device<FakeBus, FakeClock, FreeChip>;

// What each device is run at: its Config's clock, else its chip's limit, else the bus's rate.
static_assert(Slow::BusHz == 100'000);
static_assert(Free::BusHz == 400'000);
static_assert(Mid::BusHz == 250'000);
static_assert(Slower::BusHz == 50'000);
static_assert(Unclocked::BusHz == 0,
              "a bus that says no rate has none to give");

// Nothing on a bus without the feature: the request has no member it could be carried in.
template<typename R>
concept HasTiming = requires(R r) { r.timing; };
static_assert(!HasTiming<FakeBus::Request>);
static_assert(HasTiming<TimedBus::Request>);

/// The four devices on the one bus, answering every read with a zero.
FakeBusResult zeros(std::uint8_t,
                    std::span<std::byte const>,
                    std::span<std::byte> recv) {
    std::ranges::fill(recv, std::byte{});
    return FakeBusResult::succeeded;
}

void eachDeviceItsOwnClock() {
    testCase("each device's requests carry the timing for its own clock");
    TimedBus::reset();
    TimedBus::Base::respond = zeros;
    Clocked bus{};
    for(int i = 0; i < 200; ++i) { turn<ClockTag>(bus); }

    std::map<std::uint8_t, std::set<std::uint32_t>> byAddress{};
    for(auto const& [a, t] : TimedBus::seen) { byAddress[a].insert(t); }
    check(byAddress.size() == 4, "all four devices went on the wire");
    checkEq(byAddress[Slow::Address].size(), 1U, "one timing per device");
    checkEq(*byAddress[Slow::Address].begin(), 100'000U, "the chip's limit");
    checkEq(*byAddress[Free::Address].begin(), 400'000U, "no limit: the bus's rate");
    checkEq(*byAddress[Mid::Address].begin(), 250'000U, "the Config's clock");
    checkEq(*byAddress[Slower::Address].begin(), 50'000U, "the Config's, below the chip's");
    check(bus.template get<Slow>().link() == Link::answering, "a clocked device comes up");
}

void loadCountsEachDeviceAtItsClock() {
    testCase("the bus load counts each device's bits at its own clock");
    // Four identical parts: the load is each one's bits over its own rate, so the one at
    // 50 kHz weighs eight times the one at 400 kHz.
    constexpr double bits = detail::cyclicBitsPerSecond<Free>();
    constexpr double want
      = bits / 100'000.0 + bits / 400'000.0 + bits / 250'000.0 + bits / 50'000.0;
    constexpr double got = Clocked::busLoad();
    static_assert(got > want * 0.999 && got < want * 1.001);
    check(true, "static_assert above");
}

/// Two parts behind a switch at different clocks: each transaction carries its device's
/// timing, the switch's writes the switch's own.
namespace Behind {
    using Mux     = Device<TimedBus, FakeClock, Chips::Tca9548a>;
    using SlowCh0 = Device<TimedBus, FakeClock, SlowChip, DefaultConfig, NoReset, MuxGate<Mux, 0>>;
    using MidCh1  = Device<TimedBus, FakeClock, FreeChip, At250k, NoReset, MuxGate<Mux, 1>>;
    using Wired   = Kvasir::I2C::Bus<TimedBus, FakeClock, Mux, SlowCh0, MidCh1>;
}   // namespace Behind

void muxWritesCarryTheMuxClock() {
    testCase("behind a switch: the switch's writes at its clock, each part at its own");
    TimedBus::reset();
    std::uint8_t     control = 0;
    ScopedHook const answering{
      TimedBus::Base::respond,
      [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
          if(a == Behind::Mux::Address) {
              if(!sent.empty()) { control = std::to_integer<std::uint8_t>(sent[0]); }
              if(!recv.empty()) { recv[0] = std::byte{control}; }
              return FakeBusResult::succeeded;
          }
          return zeros(a, sent, recv);
      }};
    Behind::Wired bus{};
    for(int i = 0; i < 400; ++i) { turn<ClockTag>(bus); }

    std::map<std::uint8_t, std::set<std::uint32_t>> byAddress{};
    for(auto const& [a, t] : TimedBus::seen) { byAddress[a].insert(t); }
    check(!byAddress[Behind::Mux::Address].empty(), "the switch was written");
    check(byAddress[Behind::Mux::Address] == std::set<std::uint32_t>{400'000U},
          "every switch write at the switch's clock");
    check(byAddress[Behind::SlowCh0::Address] == std::set<std::uint32_t>{100'000U},
          "channel 0's part at 100 kHz");
    check(byAddress[Behind::MidCh1::Address] == std::set<std::uint32_t>{250'000U},
          "channel 1's part at 250 kHz");
}

}   // namespace

int main() {
    eachDeviceItsOwnClock();
    loadCountsEachDeviceAtItsClock();
    muxWritesCarryTheMuxClock();
    return finish();
}
