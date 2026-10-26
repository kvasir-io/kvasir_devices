#pragma once
// The members both queued masters offer, once (I2C/Controller/QueuedFront.hpp and SPI/QueuedFront.hpp derive from it
// and add only their own): submit and its tracked form, cancel, turn (and the MainLoop hook), reset, the counters, the
// timeout snapshot and the latency. One line each into the protocol core `Core` (I2C/Controller/QueueCore.hpp,
// SPI/QueueCore.hpp); no state. A part the bus switched off (Bus::QueueFeatures, BusTypes.hpp) has no accessor here:
// calling it does not compile, on both buses.
//
// `Request` is the protocol's to name: on I2C it is the shape's (a base of the chip's half, which needs it before the
// core exists), so a second `Request` here would make the master's name ambiguous. The protocol front passes it in.
// The same goes for `Extends`: it is declared here only, and each protocol front checks that it reaches this one - an
// ambiguous `Extends` reads as "absent" to the Startup hook probe, and the bus's turn() would silently stop running.
#include "../BusTypes.hpp"
#include "kvasir/Port/Extends.hpp"

#include <cstddef>
#include <cstdint>

namespace Kvasir::Bus {
template<typename Core, typename Request>
struct QueuedFront {
    static constexpr std::size_t QueueDepth   = Core::QueueDepth;
    static constexpr std::size_t CallbackSize = Core::CallbackSize;

    using Result  = typename Core::Result;
    using Latency = typename Core::Latency;

    /// False when the queue is full or the request is refused; the caller tries again next turn.
    static bool submit(Request const& r) { return Core::submit(r); }

    /// submit(), with a ticket for cancel(); an invalid ticket (refused, no callback) when submit() would be false.
    static Ticket submitTracked(Request const& r)
        requires(Core::Tracked)
    {
        return Core::submitTracked(r);
    }

    /// removed: queued, never started - its callback (cancelled) ran before this returns. stopping (I2C): on the
    /// wire - the chip stops it at the next chance and its interrupt completes it (cancelled); its buffers belong to
    /// the bus until then. tooLate: completed already, or an unknown ticket.
    static Cancel cancel(Ticket t)
        requires(Core::Features.cancel)
    {
        return Core::cancel(t);
    }

    /// Once per main-loop turn per bus: timeouts, deadlines, the dead-bus watch.
    static void turn() { Core::turn(); }

    // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() calls it (Port/Extends.hpp);
    // a firmware that runs the hook must not also call turn() by hand
    using Extends = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &turn>;

    /// Every accepted request gets exactly one callback: the one on the wire and what was queued when reset() was
    /// called are failed, then the block is set up again and what was submitted since starts.
    static void reset() { Core::reset(); }

    /// Faults with no working transfer between them (Bus/DeadBus.hpp's rule).
    static std::uint32_t consecutiveFailures() { return Core::consecutiveFailures(); }

    /// Times the dead-bus watch stepped in.
    static std::uint32_t resuscitations()
        requires(Core::Features.counters)
    {
        return Core::resuscitations();
    }

    /// Transfers turn() gave up on after their timeout.
    static std::uint32_t timeouts()
        requires(Core::Features.counters)
    {
        return Core::timeouts();
    }

    /// Requests accepted for the wire since boot.
    static std::uint32_t transfers()
        requires(Core::Features.counters)
    {
        return Core::transfers();
    }

    /// Queued requests failed without going on the wire (a reset or recovery, an expired deadline).
    static std::uint32_t drainedRequests()
        requires(Core::Features.counters)
    {
        return Core::drainedRequests();
    }

    /// The registers and times of the last timeout.
    static typename Core::TimeoutSnapshot const& lastTimeout()
        requires(Core::Features.timeoutSnapshot)
    {
        return Core::lastTimeout();
    }

    /// lastTimeout() as a log line.
    static void logLastTimeout()
        requires(Core::Features.timeoutSnapshot)
    {
        Core::logLastTimeout();
    }

    /// The protocol's waits (I2C: first interrupt and interrupt gap; SPI: queue wait and lateness) since the last
    /// call. Taken and cleared.
    static Latency takeLatency()
        requires(Core::Features.latency)
    {
        return Core::takeLatency();
    }
};
}   // namespace Kvasir::Bus
