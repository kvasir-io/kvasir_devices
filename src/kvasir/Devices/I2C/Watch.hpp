#pragma once

#include <cstdint>

/// A part that goes on answering and stops delivering: acknowledged transactions, no new
/// sample. The engine cannot call that a fault -- every transaction succeeded -- and what to do
/// about it is the part's business (a TLV493D that stopped converting hands back its last frame
/// for ever, and a general call gets it going again). This says when.
namespace Kvasir::I2C {

/// Watches read group G of a device of type D.
///
///     StallWatch<Tlv, Chips::Tlv493d::Field> stall{3s, 10s};
///     ...
///     if(stall.handler(tlv, Clock::now())) { resetTheTlv(); }   // once per loop turn
template<typename D, typename G>
class StallWatch {
public:
    using TimePoint = typename D::ClockT::time_point;
    using Duration  = typename D::ClockT::duration;

    constexpr StallWatch(Duration stalled,
                         Duration between)
      : stalled_{stalled}
      , between_{between} {}

    /// Once per loop turn. True, once, when `d` has been answering for `stalled` without a new
    /// sample of G -- and not again within `between`, so a part that will not come back is not
    /// acted on in a loop. A new sample, or the part not answering, starts the count over.
    bool handler(D const&  d,
                 TimePoint now) {
        auto const seq = d.template seq<G>();
        if(!primed_ || seq != lastSeq_ || !d.answering()) {
            primed_       = true;
            lastSeq_      = seq;
            lastProgress_ = now;
            return false;
        }
        if(now - lastProgress_ < stalled_ || now < notBefore_) { return false; }
        ++stalls_;
        notBefore_    = now + between_;
        lastProgress_ = now;
        return true;
    }

    /// How often handler() said so.
    [[nodiscard]] std::uint32_t stalls() const { return stalls_; }

    [[nodiscard]] Duration stalled() const { return stalled_; }

private:
    Duration      stalled_;
    Duration      between_;
    TimePoint     lastProgress_{};
    TimePoint     notBefore_{};
    std::uint32_t lastSeq_{};
    std::uint32_t stalls_{};
    bool          primed_{};
};

/// The other way a part can be lost without being gone: it is there, and not at its address.
/// A TLV493D-A1B6 reads its address pin again at every general call -- and nine clocks with SDA
/// low, which is what a bus recovery puts on the wire, are a general call to it -- so it comes
/// back at 0x1F, where its Device never looks. What brings it back is the part's business too (a
/// general call with 0xFF); this says when: `d` has not been answering for `absent`, and not
/// again within `between`.
///
///     AbsentWatch<Tlv> lost{2s, 10s};
///     if(lost.handler(tlv, Clock::now())) { resetTheTlv(); }   // once per loop turn
template<typename D>
class AbsentWatch {
public:
    using TimePoint = typename D::ClockT::time_point;
    using Duration  = typename D::ClockT::duration;

    constexpr AbsentWatch(Duration absent,
                          Duration between)
      : absent_{absent}
      , between_{between} {}

    bool handler(D const&  d,
                 TimePoint now) {
        if(!primed_ || d.answering()) {
            primed_     = true;
            lastAnswer_ = now;
            return false;
        }
        if(now - lastAnswer_ < absent_ || now < notBefore_) { return false; }
        ++count_;
        notBefore_ = now + between_;
        return true;
    }

    /// How often handler() said so.
    [[nodiscard]] std::uint32_t count() const { return count_; }

private:
    Duration      absent_;
    Duration      between_;
    TimePoint     lastAnswer_{};
    TimePoint     notBefore_{};
    std::uint32_t count_{};
    bool          primed_{};
};

}   // namespace Kvasir::I2C
