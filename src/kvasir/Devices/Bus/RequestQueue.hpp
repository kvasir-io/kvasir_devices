#pragma once
// The request queue of a queued bus (I2C/Controller/QueueCore.hpp, SPI/QueueCore.hpp): what waits behind the request
// on the wire, and what both cores do with it - the queued cancel and the deadline sweep (a tombstone: the slot
// stays, its callback has run, the head drops it), the pop of the next live request (tombstones dropped, a request out
// of time before it started failed timedOut), the drain of a reset. The request on the wire is not in it.
//
// Clock is the bus's; Spec, the bus's view of an entry:
//   using Result;                                    the bus's callback result
//   static constexpr bool Tracked, Deadlines;        tickets/tombstones, Request::deadline
//   static auto& request(Entry&);                    the request in an entry (I2C: the entry itself; SPI: .request)
//
// Every call runs with the bus interrupt masked (the caller's Bus::MaskDepth). The callbacks go through the caller's
// Bus::Invoker: what they submit only queues. A walk copies the callback before calling it (a callback that resets
// the bus and submits again can push into the slot being walked); a popped entry is out of the queue and is called
// in place.
// The calls hand back how many requests they failed without going on the wire; the caller adds that to its counter
// (a StatCount that may count nothing). The counter never goes into a call by reference: the core's statistics stay
// the core's, and one that is off is not touched at all.
// A walk reads head and tail once (Atomic::Queue::forEachQueued). A callback that resets the bus inside it drains -
// pops - every request still queued, and their slots keep their bytes: the walk would reach them and call them a
// second time. So a drain that pops counts itself (`drains_`, tracked buses only: only they walk), and a walk stops
// at the first slot after one; what the callbacks queued meanwhile is checked by the next sweep or when it starts.
// An object, owned by its bus core (an `inline static` member): two buses of one shape are two queues.
#include "Callbacks.hpp"
#include "kvasir/Atomic/Queue.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace Kvasir::Bus {
template<typename Entry, std::size_t Depth, typename Clock, typename Spec>
class RequestQueue {
    using Result = typename Spec::Result;
    using tp     = typename Clock::time_point;

public:
    static constexpr std::size_t capacity() { return Depth; }

    [[nodiscard]] std::size_t size() const { return q_.size(); }

    [[nodiscard]] bool empty() const { return q_.empty(); }

    /// Depth entries waiting; a tombstone still takes its slot.
    [[nodiscard]] bool full() const { return q_.size() >= q_.max_size(); }

    /// Only when not full().
    void push(Entry const& e) { q_.push(e); }

    /// The queued request with ticket `id` becomes a tombstone, its callback (cancelled) runs: true. False when no
    /// live queued request has it.
    bool cancelQueued(std::uint16_t id,
                      Invoker&      inv)
        requires(Spec::Tracked)
    {
        bool       removed = false;
        auto const drains  = drains_;
        q_.forEachQueued([&](Entry& e) {
            auto& r = Spec::request(e);
            // a callback drained the queue: the rest of the walk is stale
            if(drains != drains_) { return; }
            if(r.ticket == id && !r.cancelled) {
                r.cancelled   = true;
                auto const cb = r.callback;
                inv.invoke(cb, Result::cancelled);
                removed = true;
            }
        });
        return removed;
    }

    /// Every live queued request past its deadline becomes a tombstone, its callback timedOut. Returns how many.
    std::uint32_t expireQueued(tp       now,
                               Invoker& inv)
        requires(Spec::Deadlines)
    {
        std::uint32_t drained = 0;
        auto const    drains  = drains_;
        q_.forEachQueued([&](Entry& e) {
            auto& r = Spec::request(e);
            // a callback drained the queue: the rest of the walk is stale
            if(drains != drains_) { return; }
            if(!r.cancelled && now > r.deadline) {
                r.cancelled = true;
                ++drained;
                auto const cb = r.callback;
                inv.invoke(cb, Result::timedOut);
            }
        });
        return drained;
    }

    /// The next live request into `out`: tombstones dropped, one out of time before it started failed timedOut and
    /// counted in `expired` (the caller's local; Clock::now() per entry). False when nothing live is left.
    bool popLive(Entry&         out,
                 Invoker&       inv,
                 std::uint32_t& expired) {
        while(true) {
            if(!q_.popInto(out)) { return false; }
            if constexpr(Spec::Tracked) {
                if(Spec::request(out).cancelled) { continue; }   // a tombstone: dropped
            }
            if constexpr(Spec::Deadlines) {
                if(Clock::now() > Spec::request(out).deadline) {   // out of time before it started
                    ++expired;
                    inv.invoke(Spec::request(out).callback,
                               Result::timedOut);   // its submits only queue
                    continue;
                }
            }
            return true;
        }
    }

    /// The first `n` entries out, each live one failed; tombstones dropped (their callback has run). Returns how many
    /// it failed. What a callback pushes meanwhile stays when `n` was the size before: it waits for the bus.
    std::uint32_t drain(std::size_t n,
                        Invoker&    inv) {
        std::uint32_t drained = 0;
        Entry         e{};
        if constexpr(Spec::Tracked) {
            // before the callbacks: a walk around them stops
            if(n != 0 && !q_.empty()) { ++drains_; }
        }
        while(n-- != 0 && q_.popInto(e)) {
            if constexpr(Spec::Tracked) {
                if(Spec::request(e).cancelled) { continue; }
            }
            ++drained;
            inv.invoke(Spec::request(e).callback, Result::failed);
        }
        return drained;
    }

private:
    // Depth requests wait at once (a Fifo holds its Capacity).
    Kvasir::Atomic::Fifo<Entry, Depth> q_{};

    struct NoCount {};

    // drains that popped, wrapping (tracked buses: their walks compare it)
    [[no_unique_address]] std::conditional_t<Spec::Tracked, std::uint8_t, NoCount> drains_{};
};
}   // namespace Kvasir::Bus
