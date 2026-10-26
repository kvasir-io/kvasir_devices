#pragma once
// The vocabulary both queued buses share (I2C/Controller/QueueCore.hpp, SPI/QueueCore.hpp): results, tickets, cancel
// outcomes and the optional parts (QueueFeatures) - `cancel` gives a bus tickets and cancel(), `deadlines` bounds a
// request end to end. Off, a bus keeps its request and result types byte for byte.
//
//     auto const t = Bus::submitTracked(req);          // a Kvasir::Bus::Ticket; invalid: refused, no callback
//     switch(Bus::cancel(t)) {
//     case Kvasir::Bus::Cancel::removed:  ...          // never went out, or (SPI) stopped at once: the callback
//                                                      // (cancelled) has run before cancel() returned
//     case Kvasir::Bus::Cancel::stopping: ...          // on the wire (I2C): the callback comes when it has stopped
//     case Kvasir::Bus::Cancel::tooLate:  ...          // completed already, an unknown ticket, or a transfer on
//     }                                                // the wire that must not be cut (abortable = false)
//
// Every accepted request still gets exactly one callback, and the buffers it points at belong to the bus until then.
#include "Features.hpp"

#include <cstdint>

namespace Kvasir::Bus {
enum class Cancel : std::uint8_t { tooLate, removed, stopping };

/// What a finished transfer did.
enum class Outcome : std::uint8_t {
    failed,
    nack,
    ok,
};

/// The result on a bus that tracks requests (QueueFeatures::cancel / ::deadlines): Outcome's three, then the two ends
/// the bus gives a request itself.
enum class TrackedOutcome : std::uint8_t {
    failed,
    nack,
    ok,
    cancelled,
    timedOut,
};

/// 0 is "no request"; a bus counts from 1 and skips 0 on the wrap. 16 bits: a ticket is long stale before 65 535
/// newer requests have passed through a queue of 4-16.
struct Ticket {
    std::uint16_t id{};

    [[nodiscard]] constexpr bool valid() const { return id != 0; }

    constexpr bool operator==(Ticket const&) const = default;
};

/// What a queued bus keeps beside moving requests, set by the bus config's `queueFeatures` (I2C: the I2CConfig, SPI:
/// the master's Timing), else latency and the timeout snapshot only. Off, a part costs nothing, and its accessors do not
/// compile.
struct QueueFeatures {
    /// takeLatency(). SPI: the queue wait and the lateness of every transfer - a time stamp in every queue slot and a
    /// clock read per submit(). I2C: the wait for the first interrupt and between two.
    bool latency = true;
    /// timeouts(), drainedRequests(), resuscitations(), transfers() and the bus's own counters (SPI: overruns(),
    /// holdTimeouts(), staleCompletions(), refused(); I2C: sdaLowAfterAbort()). Off by default: a firmware that reads
    /// a counter turns them on.
    bool counters = false;
    /// lastTimeout() / logLastTimeout(): the hardware's state at the last timeout.
    bool timeoutSnapshot = true;
    /// submitTracked() -> Ticket, cancel(ticket). Off by default: the request and result types without them.
    bool cancel = false;
    /// Request::deadline: done by then, queue wait included, else `timedOut` (checked by the bus's turn()).
    bool deadlines = false;
};

/// The next ticket of a counter, never 0.
constexpr std::uint16_t nextTicket(std::uint16_t& counter) {
    counter = static_cast<std::uint16_t>(counter + 1U);
    if(counter == 0) { counter = 1; }
    return counter;
}
}   // namespace Kvasir::Bus
