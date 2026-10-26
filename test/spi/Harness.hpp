#pragma once

/// What every SPI test shares, the twin of i2c/Harness.hpp: the queued master's fake (FakeQueuedSpi.hpp) and the pins
/// (FakeSpi.hpp), a clean start, one loop turn, a run until a predicate holds or a span is up, the frame transcript
/// and the guard that empties the bus while a case's device still lives.
///
/// The tests do not all turn the loop the same way, and each keeps the way it had: `Loop` says how long a turn is,
/// whether the clock moves before the bus and the device run or after them, and whether the device's turn runs
/// before the master's. The callables after the Loop run every turn right after an advance-first clock step (or
/// first thing in an advance-last turn): a part model's tick(), a pin it drives.
///
/// The functions are in `Spi::Harness` and take the master as an explicit template argument: in `Spi` itself,
/// argument-dependent lookup would find them from every driver instantiated on `Spi::Cs` and make a test's own
/// `runUntil` ambiguous, and a defaulted master would let a call without `<Bus>` run on the wrong one.
#include "FakeQueuedSpi.hpp"
#include "FakeSpi.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kd/Check.hpp>
#include <optional>
#include <support/FakeClock.hpp>

namespace Kvasir::Test::Spi {

using namespace std::chrono_literals;

struct DefaultTag {};

using DefaultBus = QueuedSpi::Bus<DefaultTag>;

/// Where in a turn the clock moves: `first` (then the part, the master, the device, the transfer completes) or
/// `last` (the master, the device, the transfer completes, then the clock).
enum class ClockOrder : std::uint8_t { first, last };

struct Loop {
    std::chrono::microseconds step{100};
    ClockOrder                order{ClockOrder::first};
    /// The device's turn before the master's, as a driver that polls its own transfers (SdCard) is run.
    bool deviceFirst{false};
    /// The device runs the master's turn itself (the display streams): the turn does not.
    bool deviceRunsMaster{false};
};

/// The part is selected while its chip select is low.
inline bool csLow() { return !Pins::level[Cs::id]; }

namespace Harness {

    /// A clean master and pins, the log counters at zero, the clock at `start` (left as it is with std::nullopt), the
    /// part on `exchange` and selected while CS is low (`selected` for a part that may be absent). `Knobs` is the fake
    /// that holds them when `Bus` is a wrapper around it (a narrower master).
    template<typename Bus,
             typename Knobs = Bus>
    void fresh(std::function<std::uint8_t(std::uint8_t)> exchange = {},
               std::optional<std::chrono::milliseconds>  start    = 1s,
               std::function<bool()>                     selected = csLow) {
        Bus::reset();
        Pins::reset();
        if(start) { FakeClock::set(*start); }
        Log::reset();
        Knobs::partSelected = std::move(selected);
        Knobs::exchange     = std::move(exchange);
    }

    namespace HarnessImpl {
        template<typename D>
        void handle(D& d) {
            if constexpr(requires { d.turn(); }) {
                d.turn();
            } else {
                d();
            }
        }
    }   // namespace HarnessImpl

    /// One loop turn of the device (anything with turn(), or a callable that turns several) on the master `Bus`.
    template<typename Bus,
             typename D,
             typename... Each>
    void turn(D&          d,
              Loop const& loop,
              Each&&... each) {
        if(loop.order == ClockOrder::first) { FakeClock::advance(loop.step); }
        (each(), ...);
        if(loop.deviceRunsMaster) {
            HarnessImpl::handle(d);
        } else if(loop.deviceFirst) {
            HarnessImpl::handle(d);
            Bus::turn();
        } else {
            Bus::turn();
            HarnessImpl::handle(d);
        }
        Bus::tick();
        if(loop.order == ClockOrder::last) { FakeClock::advance(loop.step); }
    }

    /// Turns until `pred()` holds after a turn, or the clock has moved `within`; whether it held.
    template<typename Bus,
             typename D,
             typename Pred,
             typename... Each>
    bool runUntil(D&                        d,
                  Pred                      pred,
                  std::chrono::microseconds within,
                  Loop const&               loop = {},
                  Each&&... each) {
        auto const end = FakeClock::now() + within;
        while(FakeClock::now() < end) {
            turn<Bus>(d, loop, each...);
            if(pred()) { return true; }
        }
        return false;
    }

    /// Turns until the clock has moved `span`.
    template<typename Bus,
             typename D,
             typename... Each>
    void runFor(D&                        d,
                std::chrono::microseconds span,
                Loop const&               loop = {},
                Each&&... each) {
        auto const until = FakeClock::now() + span;
        while(FakeClock::now() < until) { turn<Bus>(d, loop, each...); }
    }

    /// The frames on the wire, for a failing case.
    template<typename Bus>
    void dump() {
        for(auto const& f : Bus::frames) {
            std::printf("  %s mode %u %u Hz mosi",
                        f.selected ? "CS " : "   ",
                        f.setup.mode,
                        f.setup.hz);
            for(auto const b : f.mosi) { std::printf(" %02x", b); }
            std::printf(" | miso");
            for(auto const b : f.miso) { std::printf(" %02x", b); }
            std::printf("\n");
        }
    }

}   // namespace Harness

/// Declared right AFTER a case's device: empties the master while the device still lives. Bus::reset() resets the
/// queue core, which completes what is still queued into its device; after the device is gone that would be a call
/// into a dead frame in the next case. (The I2C fake calls nothing back on reset; this one does.) It also empties
/// `Bus::frames`: a case that prints them on failure does so before its scope ends,
/// `if(testcase.failed()) { Spi::Harness::dump<Bus>(); }` with a `Case` (kd/Case.hpp).
template<typename Bus>
struct BusScope {
    BusScope() = default;

    ~BusScope() { Bus::reset(); }

    BusScope(BusScope const&)            = delete;
    BusScope& operator=(BusScope const&) = delete;
};

}   // namespace Kvasir::Test::Spi
