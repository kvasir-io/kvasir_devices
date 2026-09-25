#pragma once

#include "Bus.hpp"
#include "Mux.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <type_traits>

/// Rates, errors and a switch's share of the wire, as deltas between two reports.
///
/// A Bus and an arbiter keep running totals, which cannot say whether a part is getting the
/// reads its description asks for *now*, or which channel this half-minute's errors came from.
/// These take the differences; what the stats line says is the firmware's.
namespace Kvasir::I2C {

/// Per device and per segment of a Bus, over the interval between two update() calls.
///
/// A part's rate is samples a second when it has a cyclic read group, and writes a second when
/// it has none (a display, a DAC, a memory): what "doing its job" means for either. Its nominal
/// rate is what its description's Periods ask for, so 100 % is a ceiling. A segment adds up the
/// parts that were answering at the update, and of those only the reading parts' rates: a
/// display's page writes have no nominal to be held against and would only inflate the figure.
/// A part that is not fitted NAKs its probes, which is the engine working and not the wire
/// failing, so it counts towards neither; nor does a Broadcast, which is not a part.
template<typename BusT>
class BusRates {
public:
    struct Part {
        double        rate{};
        double        nominal{};
        std::uint32_t errors{};
        bool          answering{};
        bool          writing{};   ///< no cyclic read group: the rate counts writes
    };

    struct Segment {
        double        rate{};
        double        nominal{};
        std::uint32_t errors{};
        std::uint8_t  parts{};   ///< answering parts
    };

    /// The deltas since the last call, over `interval`. The first call only takes the totals
    /// and returns false; every later one returns true.
    bool update(BusT const&                   bus,
                std::chrono::duration<double> interval) {
        std::array<typename BusT::Counters, BusT::Count> now{};
        bus.snapshot(now);
        segments_         = {};
        std::size_t index = 0;
        bus.forEach([&](auto const& d) {
            using D                = std::remove_cvref_t<decltype(d)>;
            auto const       i     = index++;
            auto const&      was   = last_[i];
            auto const&      is    = now[i];
            auto&            p     = parts_[i];
            constexpr double reads = BusT::template nominalSamplesPerSecond<D>();
            p.writing              = reads == 0.0;
            p.nominal              = p.writing ? BusT::template nominalWritesPerSecond<D>() : reads;
            auto const n           = p.writing ? is.writes - was.writes : is.samples - was.samples;
            p.rate   = interval > interval.zero() ? static_cast<double>(n) / interval.count() : 0.0;
            p.errors = is.errors - was.errors;
            p.answering = d.answering();
            if(!primed_ || !p.answering || !BusT::template isPart<D>()) { return; }
            auto& s = segments_[BusT::segmentOf(i)];
            ++s.parts;
            s.errors += p.errors;
            if(!p.writing) {
                s.rate += p.rate;
                s.nominal += p.nominal;
            }
        });
        last_            = now;
        bool const ready = primed_;
        primed_          = true;
        return ready;
    }

    [[nodiscard]] Part const& part(std::size_t index) const { return parts_[index]; }

    /// The same by device type, the way a firmware names them: `rates.part<Bh1750>()`.
    template<typename D>
    [[nodiscard]] Part const& part() const {
        return parts_[BusT::template indexOf<D>()];
    }

    [[nodiscard]] Segment const& segment(std::size_t s) const { return segments_[s]; }

private:
    std::array<typename BusT::Counters, BusT::Count> last_{};
    std::array<Part, BusT::Count>                    parts_{};
    std::array<Segment, BusT::Segments>              segments_{};
    bool                                             primed_{};
};

/// One switch's share of the wire per channel, over the interval between two update() calls:
/// how often a channel took the switch, how long it held it (as its policy counts holding,
/// Mux.hpp), and how often its parts had to wait for another channel -- by that channel, so a
/// part short of its rate can be traced to the channel that kept it off the wire.
template<typename Arbiter>
class SwitchStats {
public:
    using Duration = typename Arbiter::Duration;

    struct Channel {
        std::uint32_t                grants{};
        Duration                     held{};
        std::array<std::uint32_t, 8> waitedBehind{};
        std::uint32_t                waits{};   ///< all of waitedBehind
    };

    /// The deltas since the last call. The first call only takes the totals and returns false.
    bool update(Arbiter const& arbiter) {
        for(std::size_t ch = 0; ch < 8; ++ch) {
            auto& c  = channels_[ch];
            c.grants = arbiter.grants[ch] - grants_[ch];
            c.held   = arbiter.held[ch] - held_[ch];
            c.waits  = 0;
            for(std::size_t holder = 0; holder < 8; ++holder) {
                c.waitedBehind[holder] = arbiter.waited[ch][holder] - waited_[ch][holder];
                c.waits += c.waitedBehind[holder];
            }
        }
        grants_          = arbiter.grants;
        held_            = arbiter.held;
        waited_          = arbiter.waited;
        bool const ready = primed_;
        primed_          = true;
        return ready;
    }

    [[nodiscard]] Channel const& channel(std::size_t ch) const { return channels_[ch]; }

private:
    std::array<Channel, 8>                      channels_{};
    std::array<std::uint32_t, 8>                grants_{};
    std::array<Duration, 8>                     held_{};
    std::array<std::array<std::uint32_t, 8>, 8> waited_{};
    bool                                        primed_{};
};

}   // namespace Kvasir::I2C
