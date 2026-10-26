#pragma once
/// The two queued-bus cores behind one test vocabulary, so a scenario is written once and run on both: the I2C core
/// over i2c/FakeQueueHw.hpp, the SPI core over spi/FakeQueuedSpi.hpp, each with cancel and deadlines on. A bus is a
/// type per `Tag`: two tags are two buses of the same shape in one binary.
///
/// What differs between the protocols is said here and nowhere else: a cancel on the wire (I2C stops and completes
/// from the interrupt, SPI aborts at once), a deadline cut on the wire (the same), and what a failed transfer is on
/// the fake wire (I2C a bus fault, SPI a receive overrun).
#include "i2c/FakeQueueHw.hpp"
#include "spi/FakeQueuedSpi.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <kvasir/Devices/BusTypes.hpp>
#include <span>
#include <support/FakeClock.hpp>
#include <utility>
#include <vector>

namespace Kvasir::Test::QueueBuses {
using namespace std::chrono_literals;
using tp = FakeClock::time_point;

/// A request's end, in one vocabulary for both buses (Bus::TrackedOutcome's order, both buses' result since LG3 P4a).
enum class R : std::uint8_t { failed, nack, ok, cancelled, timedOut };

/// How a bus ends the request on the wire, for the dead-bus streak cases.
enum class End : std::uint8_t {
    ok,
    nack,          ///< I2C only: an address nobody answered
    fault,         ///< the wire failed (I2C bus fault, SPI receive overrun)
    lost,          ///< no completion: turn()'s transfer timeout
    cancelled,     ///< cancel() of the request on the wire
    deadlineCut,   ///< its deadline passed on the wire
    ownReset,      ///< reset() with it on the wire
};

/// What every bus adapter shares: the callbacks' record and the hook a scenario runs from inside a callback.
template<typename Self>
struct Record {
    static inline std::vector<std::pair<int, R>> seen{};
    /// Runs after the record, inside the callback: a scenario's submit/cancel/reset from a callback.
    static inline std::function<void(int, R)> after{};

    static void note(int tag,
                     R   r) {
        seen.emplace_back(tag, r);
        if(after) { after(tag, r); }
    }

    /// Callbacks of `tag` so far.
    static int callbacksOf(int tag) {
        int n = 0;
        for(auto const& e : seen) { n += e.first == tag ? 1 : 0; }
        return n;
    }

    static bool saw(int tag,
                    R   r) {
        for(auto const& e : seen) {
            if(e.first == tag && e.second == r) { return true; }
        }
        return false;
    }
};

inline std::array<std::byte, 4> txBuf{};
inline std::array<std::byte, 4> rxBuf{};

template<typename Tag>
struct I2cBus : Record<I2cBus<Tag>> {
    using Base     = Record<I2cBus<Tag>>;
    using Hw       = I2cQueue::FakeHw<true, Tag>;
    using Core     = I2cQueue::CoreOf<Hw>;
    using Recovery = typename Hw::Recovery;
    using Request  = typename Core::Request;
    using Result   = typename Core::Result;

    static constexpr char const* name  = "i2c";
    static constexpr std::size_t Depth = Core::QueueDepth;

    static R map(Result r) { return static_cast<R>(r); }

    static Request make(int tag,
                        tp  deadline = tp::max()) {
        Request r{};
        r.address     = static_cast<std::uint8_t>(0x40 + (tag & 0x3F));
        r.receiveData = std::span{rxBuf}.first(2);
        r.deadline    = deadline;
        r.callback    = [tag](Result x) { Base::note(tag, map(x)); };
        return r;
    }

    /// make(), but its callback runs `after` first and records its tag last: a callback whose own storage is
    /// overwritten while it runs records another tag.
    static Request makeRecordingLast(int tag) {
        Request r  = make(tag);
        r.callback = [tag](Result x) {
            if(Base::after) { Base::after(tag, map(x)); }
            Base::seen.emplace_back(tag, map(x));
        };
        return r;
    }

    /// Requests submitted from now on wait in the queue with nothing on the wire (SDA reads low), until
    /// releaseIdle().
    static void holdIdle() { Recovery::sdaHigh = false; }

    static void releaseIdle() { Recovery::sdaHigh = true; }

    static bool submit(int tag,
                       tp  deadline = tp::max()) {
        return Core::submit(make(tag, deadline));
    }

    static Kvasir::Bus::Ticket submitTracked(int tag,
                                             tp  deadline = tp::max()) {
        return Core::submitTracked(make(tag, deadline));
    }

    static Kvasir::Bus::Cancel cancel(Kvasir::Bus::Ticket t) { return Core::cancel(t); }

    static void end(End e) {
        switch(e) {
        case End::ok:    Hw::raise(I2cQueue::Wire::ok); return;
        case End::nack:  Hw::raise(I2cQueue::Wire::nack); return;
        case End::fault: Hw::raise(I2cQueue::Wire::fault); return;
        case End::lost:
            FakeClock::advance(15ms);
            Core::turn();
            return;
        case End::cancelled: return;   // cancelOnWire()
        case End::deadlineCut:
            FakeClock::advance(1500us);
            Core::turn();
            Hw::raise(I2cQueue::Wire::stopped);
            return;
        case End::ownReset: Core::reset(); return;
        }
    }

    /// cancel() of the request on the wire, to its callback: I2C asks for a STOP, the interrupt completes it.
    static void cancelOnWire(Kvasir::Bus::Ticket t) {
        check(Core::cancel(t) == Kvasir::Bus::Cancel::stopping, "i2c: on the wire, stopping");
        Hw::raise(I2cQueue::Wire::stopped);
    }

    static void ok() { end(End::ok); }

    static void fail() { end(End::fault); }

    static void turn() { Core::turn(); }

    static void reset() { Core::reset(); }

    static int starts() { return Hw::starts; }

    static int maskCalls() { return Hw::maskCalls; }

    static bool masked() { return Hw::lineMasked; }

    static std::uint32_t streak() { return Core::consecutiveFailures(); }

    static std::uint32_t drained() { return Core::drainedRequests(); }

    static void fresh() {
        Base::after = {};
        Core::reset();
        Recovery::reset();
        // the dead-bus streak: one success clears it
        if(Core::consecutiveFailures() != 0) {
            Core::submit(make(99));
            Hw::raise(I2cQueue::Wire::ok);
        }
        Hw::clear();
        Base::seen.clear();
    }
};

/// SPI's features for the scenarios: tickets, deadlines and the counters (latency and snapshot as by default).
struct TrackedSpiTiming : Kvasir::SPI::QueueCoreDefaults {
    static constexpr Kvasir::Bus::QueueFeatures queueFeatures{.counters  = true,
                                                              .cancel    = true,
                                                              .deadlines = true};
};

template<typename Tag>
struct SpiBus : Record<SpiBus<Tag>> {
    using Base    = Record<SpiBus<Tag>>;
    using Wire    = Kvasir::Test::QueuedSpi::Bus<Tag, TrackedSpiTiming>;
    using Core    = typename Wire::Core;
    using Request = typename Core::RequestT;
    using Result  = typename Core::Result;

    static constexpr char const* name  = "spi";
    static constexpr std::size_t Depth = Core::QueueDepth;

    static R map(Result r) {
        switch(r) {
        case Result::failed:    return R::failed;
        case Result::nack:      return R::nack;
        case Result::ok:        return R::ok;
        case Result::cancelled: return R::cancelled;
        case Result::timedOut:  return R::timedOut;
        }
        return R::failed;
    }

    static Request make(int tag,
                        tp  deadline = tp::max()) {
        Request r{};
        r.tx       = std::span<std::byte const>{txBuf};
        r.deadline = deadline;
        r.callback = [tag](Result x) { Base::note(tag, map(x)); };
        return r;
    }

    static Request makeRecordingLast(int tag) {
        Request r  = make(tag);
        r.callback = [tag](Result x) {
            if(Base::after) { Base::after(tag, map(x)); }
            Base::seen.emplace_back(tag, map(x));
        };
        return r;
    }

    /// An idle SPI bus starts what is submitted at once: nothing to hold.
    static void holdIdle() {}

    static void releaseIdle() {}

    static bool submit(int tag,
                       tp  deadline = tp::max()) {
        return Core::submit(make(tag, deadline));
    }

    static Kvasir::Bus::Ticket submitTracked(int tag,
                                             tp  deadline = tp::max()) {
        return Core::submitTracked(make(tag, deadline));
    }

    static Kvasir::Bus::Cancel cancel(Kvasir::Bus::Ticket t) { return Core::cancel(t); }

    static void end(End e) {
        switch(e) {
        case End::ok:   Wire::tick(); return;
        case End::nack: check(false, "spi: no NAK on SPI"); return;
        case End::fault:
            Wire::overrunNext = true;
            Wire::tick();
            return;
        case End::lost:
            FakeClock::advance(2200us);   // past 2 x the wire time + the 2 ms margin
            Core::turn();
            return;
        case End::cancelled: return;   // cancelOnWire()
        case End::deadlineCut:
            FakeClock::advance(1500us);
            Core::turn();
            return;
        case End::ownReset: Core::reset(); return;
        }
    }

    /// cancel() of the request on the wire, to its callback: SPI aborts at once.
    static void cancelOnWire(Kvasir::Bus::Ticket t) {
        check(Core::cancel(t) == Kvasir::Bus::Cancel::removed, "spi: on the wire, removed at once");
    }

    static void ok() { end(End::ok); }

    static void fail() { end(End::fault); }

    static void turn() { Core::turn(); }

    static void reset() { Core::reset(); }

    static int starts() { return static_cast<int>(Wire::frames.size()); }

    static int maskCalls() { return Wire::maskCalls; }

    static bool masked() { return Wire::masked; }

    static std::uint32_t streak() { return Core::consecutiveFailures(); }

    static std::uint32_t drained() { return Core::drainedRequests(); }

    static void fresh() {
        Base::after = {};
        Wire::reset();
        if(Core::consecutiveFailures() != 0) {
            Core::submit(make(99));
            Wire::tick();
        }
        Wire::reset();
        Base::seen.clear();
    }
};

struct TagA {};

struct TagB {};

}   // namespace Kvasir::Test::QueueBuses
