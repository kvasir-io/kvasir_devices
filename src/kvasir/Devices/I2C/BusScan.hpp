#pragma once

#include "../Log.hpp"
#include "Bus.hpp"
#include "Mux.hpp"
#include "Scanner.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

/// A scan of every stretch of wire a Bus has -- in front of its switches, then behind each
/// channel of each -- that takes part in the Bus instead of going round it.
///
/// A plain Scanner submits its probes past the devices: run while they talk, it finds whatever
/// channel a part has open at that moment, and it probes the parts' own addresses, which can
/// land in the middle of a part's script (an SHT4x between its measurement command and the read
/// of the result). BusScan claims the switches for the segment it is probing, through the
/// arbiters the gates use and one probe at a time, so the parts go on between two probes: the
/// segment's switch on its channel, every other switch shut. By default it leaves every address
/// a part of the Bus owns alone and asks the part instead.
///
///     using Scan = Kvasir::I2C::BusScan<Sensors, Catalogue>;
///     Scan scan{bus};                      // the Bus's own arbiters (`Scan scan{bus, arbiter}`
///                                          // for gates the application bound itself)
///     scan.start(Scan::Owned::probe);      // at boot, before the parts start: every address
///     ...
///     scan.handler();                      // once per loop turn
///     scan.start();                        // later, while they run: their addresses alone
///
/// Switches in front of every part (none behind another switch's channel), and a bus queue one
/// request deeper than the Bus needs.
namespace Kvasir::I2C {

namespace detail {
    /// A MuxPort for every switch of a Bus, in the order Bus::Segments numbers them.
    template<typename BusT, typename Seq>
    struct ScanPorts;

    template<typename BusT, std::size_t... Ks>
    struct ScanPorts<BusT, std::index_sequence<Ks...>> {
        using type = std::tuple<MuxPort<typename BusT::template SwitchDeviceAt<Ks>,
                                        typename BusT::template SwitchArbiterAt<Ks>::PolicyT>...>;
    };
}   // namespace detail

template<typename BusT,
         typename Hints  = NoHints,
         typename Probe  = AddressProbe,
         typename Config = ScanDefaults>
class BusScan {
public:
    using I2c   = typename BusT::I2cBus;
    using Clock = typename BusT::ClockT;
    using Slot  = ProbeSlot<I2c, Clock, Probe, Config>;

    static constexpr std::size_t Switches  = BusT::Switches;
    static constexpr bool        HasSwitch = Switches != 0;

    static_assert(Switches <= 32,
                  "more switches than a scan keeps track of");

    static_assert(
      []<std::size_t... Ks>(std::index_sequence<Ks...>) {
          return ((BusT::template segmentOf<typename BusT::template SwitchDeviceAt<Ks>>() == 0)
                  && ... && true);
      }(std::make_index_sequence<Switches>{}),
      "BusScan sets switches that are in front of every part: a switch behind "
      "another switch's channel is not supported");

    static_assert(detail::queueFits<I2c>(BusT::QueueDepth + 1),
                  "a bus scan's probe needs a queue slot of its own beside the Bus's devices: "
                  "raise the bus behavior's QueueDepth");

    /// 0x00..0x07 and 0x78..0x7F are reserved by the I2C specification and never probed.
    static constexpr std::uint8_t First = 0x08;
    static constexpr std::uint8_t Last  = 0x77;

    /// How long the switches may take to be claimed and set for a segment before the scan
    /// decides one that is not set is not there, and leaves its channels out
    /// (ScanDefaults::SelectTimeout), until selectTimeout(ms) says otherwise.
    static constexpr auto SelectTimeout = detail::selectTimeoutOf<Config>();

    void selectTimeout(std::chrono::milliseconds t) { selectTimeout_ = t; }

    /// Give up on a probe after `t` from now on.
    void probeTimeout(std::chrono::milliseconds t) { slot_.timeout(t); }

    /// Whether the addresses the Bus's parts own are probed.
    enum class Owned : std::uint8_t {
        skip,    ///< left alone, and the parts asked instead: what a scan among running parts does
        probe,   ///< probed like any other: before the parts start, what answers where
    };

    /// The switches are the Bus's own and so are their arbiters (Bus::arbiter<K>()), which is
    /// what the gates are bound to unless the application bound them elsewhere.
    explicit BusScan(BusT& bus) : bus_{bus} {
        if constexpr(HasSwitch) { bindOwn_(std::make_index_sequence<Switches>{}); }
    }

    /// The switches are the Bus's own, and `arbiters` the ones the application bound their
    /// gates to itself: one for each, in the order Bus::Segments numbers the switches.
    template<typename... Arbiters>
    BusScan(BusT& bus,
            Arbiters&... arbiters)
        requires(HasSwitch && sizeof...(Arbiters) == Switches)
      : bus_{bus} {
        bindPorts_(std::make_index_sequence<Switches>{}, arbiters...);
    }

    BusScan(BusScan const&)            = delete;
    BusScan& operator=(BusScan const&) = delete;

    /// Scan every segment. Ignored while a scan is under way.
    void start(Owned owned = Owned::skip) {
        if(state_ != State::idle) { return; }
        owned_         = owned;
        found_         = {};
        others_        = 0;
        faults_        = 0;
        partsAnswered_ = 0;
        missing_       = 0;
        segment_       = 0;
        asking_        = false;
        beginSegment_();
    }

    [[nodiscard]] bool done() const { return state_ == State::idle; }

    /// At least one scan has finished since boot.
    [[nodiscard]] bool scanned() const { return scans_ != 0; }

    [[nodiscard]] std::uint32_t scans() const { return scans_; }

    /// Something answered a probe of `address` on `segment` in the last scan. An address that
    /// answers in front of the switches is not probed again behind a channel, where it would
    /// answer too.
    [[nodiscard]] bool found(std::size_t  segment,
                             std::uint8_t address) const {
        return segment < BusT::Segments && address < 0x80
            && (found_[segment][address / 64] & (std::uint64_t{1} << (address % 64))) != 0;
    }

    /// Devices that answered and that no part of the Bus owns, in the last scan.
    [[nodiscard]] std::size_t others() const { return others_; }

    /// Bus faults (not NAKs) and lost probes in the last scan.
    [[nodiscard]] std::size_t faults() const { return faults_; }

    /// True when any switch was missing in the last scan (its channels left out), or when the
    /// Bus has no switch at all and the front is all there is.
    [[nodiscard]] bool switchless() const { return !HasSwitch || missing_ != 0; }

    /// The switches that were not there in the last scan, bit K for switch K.
    [[nodiscard]] std::uint32_t missingSwitches() const { return missing_; }

    /// Once per loop turn.
    void handler() {
        switch(state_) {
        case State::idle: return;
        case State::probe:
            if(!ready_() || state_ != State::probe) { return; }
            // A full queue is not an error: the same address next turn.
            if(slot_.submit(addr_)) { state_ = State::wait; }
            return;
        case State::wait:
            {
                auto const outcome = slot_.poll();
                if(outcome == Slot::Outcome::waiting) { return; }
                record_(outcome);
                releasePorts_();
                if(seek_(static_cast<unsigned>(addr_) + 1U)) {
                    state_ = State::probe;
                } else {
                    endSegment_();
                }
            }
            return;
        }
    }

private:
    enum class State : std::uint8_t { idle, probe, wait };

    using Ports = typename detail::ScanPorts<BusT, std::make_index_sequence<Switches>>::type;

    /// For each segment and address, 1 + the index of the part of the Bus that owns it there,
    /// or 0.
    static constexpr auto Owners = [] {
        std::array<std::array<std::uint8_t, 0x80>, BusT::Segments> owners{};
        detail::forEach<typename BusT::Devices>([&](auto i) {
            using D =
              typename detail::Nth<typename BusT::Devices>::template type<decltype(i)::value>;
            owners[BusT::segmentOf(decltype(i)::value)][D::Address & 0x7FU]
              = static_cast<std::uint8_t>(decltype(i)::value + 1U);
        });
        return owners;
    }();

    template<std::size_t... Ks>
    void bindOwn_(std::index_sequence<Ks...>) {
        ((std::get<Ks>(ports_).bind(bus_.template get<typename BusT::template SwitchDeviceAt<Ks>>(),
                                    bus_.template arbiter<Ks>())),
         ...);
    }

    template<std::size_t... Ks,
             typename... Arbiters>
    void bindPorts_(std::index_sequence<Ks...>,
                    Arbiters&... arbiters) {
        ((std::get<Ks>(ports_).bind(bus_.template get<typename BusT::template SwitchDeviceAt<Ks>>(),
                                    arbiters)),
         ...);
    }

    /// f(K, port, switch) for every switch.
    template<typename F>
    void forPorts_(F&& f) {
        forPorts_(f, std::make_index_sequence<Switches>{});
    }

    template<typename F,
             std::size_t... Ks>
    void forPorts_([[maybe_unused]] F& f,
                   std::index_sequence<Ks...>) {
        ((f(Ks,
            std::get<Ks>(ports_),
            bus_.template get<typename BusT::template SwitchDeviceAt<Ks>>())),
         ...);
    }

    [[nodiscard]] bool missingAt_(std::size_t k) const { return ((missing_ >> k) & 1U) != 0; }

    /// Behind a channel of a switch that was not there.
    [[nodiscard]] bool skipped_(std::size_t segment) const {
        return segment != 0 && missingAt_(BusT::switchOfSegment(segment));
    }

    void beginSegment_() {
        if(seek_(First)) {
            state_ = State::probe;
        } else {
            endSegment_();
        }
    }

    /// Whether the scan probes `address` on the current segment.
    [[nodiscard]] bool wanted_(std::uint8_t address) const {
        if(segment_ != 0) {
            // What answers in front of the switches answers on every channel too, and a part in
            // front of them is looked at there.
            if(found(0, address) || Owners[0][address] != 0) { return false; }
        }
        return owned_ == Owned::probe || Owners[segment_][address] == 0;
    }

    /// The next address from `from` on that the segment wants; false when there is none.
    bool seek_(unsigned from) {
        for(unsigned a = from; a <= Last; ++a) {
            if(wanted_(static_cast<std::uint8_t>(a))) {
                addr_ = static_cast<std::uint8_t>(a);
                return true;
            }
        }
        return false;
    }

    /// This turn may probe: every switch that is there is this scan's and set for the segment
    /// -- the segment's own on its channel, the others shut. One that is absent, or not set
    /// within SelectTimeout, is taken to be not there: the scan goes on without it and leaves
    /// its channels out.
    bool ready_() {
        if constexpr(HasSwitch) {
            std::uint32_t unready = 0;
            forPorts_([&](std::size_t k, auto& port, auto const&) {
                if(missingAt_(k)) { return; }
                using P             = std::remove_cvref_t<decltype(port)>;
                auto const position = segment_ != 0 && BusT::switchOfSegment(segment_) == k
                                      ? BusT::channelOfSegment(segment_)
                                      : P::Closed;
                if(!port.claim(position)) { unready |= std::uint32_t{1} << k; }
            });
            if(unready == 0) {
                asking_ = false;
                return true;
            }
            auto const now = Clock::now();
            if(!asking_) {
                asking_  = true;
                askedAt_ = now;
            }
            std::uint32_t absent = 0;
            forPorts_([&](std::size_t k, auto&, auto const& sw) {
                if(((unready >> k) & 1U) != 0 && sw.absent()) { absent |= std::uint32_t{1} << k; }
            });
            if(absent == 0 && now - askedAt_ <= selectTimeout_) { return false; }
            auto const gone = absent != 0 ? absent : unready;
            asking_         = false;
            forPorts_([&](std::size_t k, auto& port, [[maybe_unused]] auto const& sw) {
                if(((gone >> k) & 1U) == 0) { return; }
                port.release();
                missing_ |= std::uint32_t{1} << k;
                UC_LOG_W(
                  "bus scan {}: the switch at {:#04x} was not set ({}), so its channels are "
                  "not scanned",
                  BusT::segmentName(segment_),
                  std::remove_cvref_t<decltype(sw)>::Address,
                  sw.link());
            });
            if(skipped_(segment_)) { nextSegment_(); }
            return false;
        } else {
            return true;
        }
    }

    void record_(typename Slot::Outcome outcome) {
        auto const owner = Owners[segment_][addr_];
        switch(outcome) {
        case Slot::Outcome::waiting: return;
        case Slot::Outcome::ack:
            found_[segment_][addr_ / 64] |= std::uint64_t{1} << (addr_ % 64);
            if(owner == 0) {
                ++others_;
                UC_LOG_I("bus scan {}: {:#04x} answers, and is not a part of the Bus: {}",
                         BusT::segmentName(segment_),
                         addr_,
                         Hints::hint(addr_));
            } else {
                ++partsAnswered_;
                logPart_(owner - 1U, "answers");
            }
            return;
        case Slot::Outcome::nak:
            if(owner != 0) {
                // Behind a bridge that is not active (Bridge.hpp) it cannot answer, and that
                // is no finding.
                bool offline = false;
                bus_.visit(owner - 1U, [&](auto const& d) { offline = d.offline(); });
                logPart_(owner - 1U,
                         offline ? "is behind a bridge that is not active" : "does not answer");
            }
            return;
        case Slot::Outcome::fault:
            ++faults_;
            UC_LOG_W("bus scan {}: {:#04x} bus fault (not a NAK)",
                     BusT::segmentName(segment_),
                     addr_);
            return;
        case Slot::Outcome::lost:
            ++faults_;
            UC_LOG_W("bus scan {}: {:#04x} no answer from the bus -- counted as a fault",
                     BusT::segmentName(segment_),
                     addr_);
            return;
        }
    }

    void logPart_([[maybe_unused]] std::size_t      index,
                  [[maybe_unused]] std::string_view what) {
        bus_.visit(index, [&]([[maybe_unused]] auto const& d) {
            UC_LOG_I("bus scan {}: {:#04x} {} {}",
                     BusT::segmentName(segment_),
                     std::remove_cvref_t<decltype(d)>::Address,
                     std::remove_cvref_t<decltype(d)>::Chip::Name,
                     what);
        });
    }

    /// The segment is done. Its parts were not probed when they were skipped, so they say for
    /// themselves: a part that is not answering is a line (a Broadcast is not a part).
    void endSegment_() {
        if(owned_ == Owned::skip) {
            for(std::size_t index = 0; index < BusT::Count; ++index) {
                if(BusT::segmentOf(index) != segment_) { continue; }
                bus_.visit(index, [&](auto const& d) {
                    using D = std::remove_cvref_t<decltype(d)>;
                    if(d.answering() || d.offline() || !BusT::template isPart<D>()) { return; }
                    UC_LOG_W("bus scan {}: {:#04x} {} {}",
                             BusT::segmentName(segment_),
                             std::remove_cvref_t<decltype(d)>::Address,
                             std::remove_cvref_t<decltype(d)>::Chip::Name,
                             d.link());
                });
            }
        }
        nextSegment_();
    }

    /// On to the next segment whose switch is there; the scan ends after the last.
    void nextSegment_() {
        releasePorts_();
        do { ++segment_; } while(segment_ < BusT::Segments && skipped_(segment_));
        if(segment_ >= BusT::Segments) {
            finish_();
            return;
        }
        beginSegment_();
    }

    void finish_() {
        releasePorts_();
        ++scans_;
        state_ = State::idle;
        UC_LOG_I("bus scan #{}: {} part(s) answering, {} other device(s), {} bus fault(s){}",
                 scans_,
                 owned_ == Owned::probe ? partsAnswered_ : bus_.answeringCount(),
                 others_,
                 faults_,
                 std::string_view{missing_ != 0
                                    ? ", the channels of a switch that was not there not scanned"
                                    : ""});
    }

    void releasePorts_() {
        forPorts_([](std::size_t, auto& port, auto const&) { port.release(); });
    }

    BusT&                                                    bus_;
    [[no_unique_address]] Ports                              ports_{};
    Slot                                                     slot_{};
    std::array<std::array<std::uint64_t, 2>, BusT::Segments> found_{};
    typename Clock::time_point                               askedAt_{};
    std::chrono::milliseconds                                selectTimeout_{SelectTimeout};
    std::size_t                                              segment_{};
    std::size_t                                              others_{};
    std::size_t                                              faults_{};
    std::size_t                                              partsAnswered_{};
    std::uint32_t                                            scans_{};
    std::uint32_t                                            missing_{};
    State                                                    state_{State::idle};
    Owned                                                    owned_{Owned::skip};
    std::uint8_t                                             addr_{First};
    bool                                                     asking_{};
};

}   // namespace Kvasir::I2C
