#pragma once
// Tickets, cancel outcomes and the extra request fields of a queued bus that can cancel (`cancellable` /
// QueueCoreFeatures::cancel) or bound a request end to end (`requestDeadlines` / QueueCoreFeatures::deadlines).
// Off, a bus keeps its request and result types byte for byte.
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
#include "I2C/EngineFeatures.hpp"

#include <cstdint>

namespace Kvasir::Bus {
enum class Cancel : std::uint8_t { tooLate, removed, stopping };

/// 0 is "no request"; a bus counts from 1 and skips 0 on the wrap. 16 bits: a ticket is long stale before 65 535
/// newer requests have passed through a queue of 4-16.
struct Ticket {
    std::uint16_t id{};

    [[nodiscard]] constexpr bool valid() const { return id != 0; }

    constexpr bool operator==(Ticket const&) const = default;
};

/// The extra fields of a request on a port that tracks requests, around the port's own request type.
template<typename Base, typename TimePoint, bool Deadlines>
struct Tracked : Base {
    std::uint16_t ticket{};   // set by submitTracked(); 0 from a plain submit()
    bool
      cancelled{};   // a tombstone in the queue: dropped when the bus reaches it, its callback has run
    bool abortable{
      true};   // false: a transfer already on the wire runs to its end, cancel() says tooLate
               // (an SD block write, a flash page program)
    [[no_unique_address]] I2C::detail::IfFeature<Deadlines, TimePoint, 30> deadline{
      TimePoint::max()};
};

/// The next ticket of a counter, never 0.
constexpr std::uint16_t nextTicket(std::uint16_t& counter) {
    counter = static_cast<std::uint16_t>(counter + 1U);
    if(counter == 0) { counter = 1; }
    return counter;
}
}   // namespace Kvasir::Bus
