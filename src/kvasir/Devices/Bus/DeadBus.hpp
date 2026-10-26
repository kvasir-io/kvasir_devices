#pragma once
// The dead-bus watch of a queued bus (I2C/Controller/QueueCore.hpp, SPI/QueueCore.hpp): line-state checks miss
// failures on a healthy-looking wire (an unconfigured block, a dropped address write), so the bus asks instead whether
// anything works at all. One rule for both buses, for every end of the request on the wire:
//
// - a fault counts (saturating at the u32 maximum): the block or the wire failed the transfer;
// - a transfer that worked resets it: `ok`, and on I2C a NAK - the address went out and nobody took it, a working
//   wire with nothing at that address, so an empty header never looks like a dead bus;
// - everything else leaves it: a cancel and a deadline cut are the caller's choice, and a request failed by the bus's
//   own reset() or recovery (`ownReset`) says nothing about the wire.
//
// takeIfDead() runs masked in the core's turn(): once the streak reached Threshold it clears the streak, counts a
// resuscitation and hands the reaction (the core's: I2C alternates a re-init and a full recovery, SPI re-inits) to the
// caller. `Resuscitations` is the counter's type: a plain u32 where the reaction reads it (I2C), a statistics counter
// that may be switched off where it is only read out (SPI, Kvasir::detail::StatCount). An object, owned by its bus
// core: two buses of one shape are two cores, so two streaks.
#include <cstdint>
#include <limits>

namespace Kvasir::Bus {
template<std::uint32_t Threshold, typename Resuscitations = std::uint32_t>
class DeadBusWatch {
public:
    static constexpr std::uint32_t threshold = Threshold;

    /// The request on the wire ended as `result` (Bus::Outcome or Bus::TrackedOutcome); `ownReset`: failed by our
    /// own reset.
    template<typename Result>
    void note(Result result,
              bool   ownReset) {
        if(result == Result::failed) {
            if(!ownReset && streak_ != std::numeric_limits<std::uint32_t>::max()) { ++streak_; }
        } else if(worked_(result)) {
            streak_ = 0;
        }
    }

    /// Caller masked. True once the streak reached Threshold: the streak starts over, one resuscitation more.
    [[nodiscard]] bool takeIfDead() {
        if(streak_ < Threshold) { return false; }
        streak_ = 0;
        ++resuscitations_;
        return true;
    }

    /// Faults with no working transfer between them.
    [[nodiscard]] std::uint32_t consecutiveFailures() const { return streak_; }

    /// How often takeIfDead() was true.
    [[nodiscard]] Resuscitations const& resuscitations() const { return resuscitations_; }

private:
    template<typename Result>
    static constexpr bool worked_(Result r) {
        return r == Result::ok || r == Result::nack;
    }

    std::uint32_t                        streak_{};
    [[no_unique_address]] Resuscitations resuscitations_{};
};
}   // namespace Kvasir::Bus
