#pragma once

#include "../Log.hpp"
#include "Device.hpp"

#include <array>
#include <chrono>
#include <concepts>
#include <cstdint>

/// A device behind an I2C switch.
///
/// Selecting the channel from the application is not enough: the engine schedules
/// autonomously, so a periodic read fires when its deadline comes and not when the
/// application says, and two devices on different channels would interleave and each read
/// whatever the switch happened to be pointing at. The selection has to be part of the
/// schedule, which is what a gate is.
///
/// A `Gate` is the sixth `Device` template parameter, beside `Reset`. The engine asks it
/// before submitting any transaction, and releases it whenever the run waits -- a step's
/// delay, a wait step, a check or decode retry -- and when the run ends: finished, failed,
/// rejected after its retries, or abandoned by a reset. What a switch needs is not to move
/// inside a transaction (a register read is a write and a read with a repeated start between
/// them), and a transaction is one request on the bus queue: the switch write another channel
/// asks for after this one lets go is queued behind it. Holding the switch through a wait
/// would only starve the other channels -- a conversion does not need the wire.
///
/// The gate is an object the Device holds, so it keeps a pointer to the switch rather than
/// naming it: the switch, the arbiter and the devices behind them are all ordinary objects
/// the application owns, and none of them has to be a global.
///
///     using MuxDev = Device<I2c1, Clock, Chips::Tca9548a>;
///     using Left   = Device<I2c1, Clock, Chips::Sht3x, DefaultConfig, NoReset,
///                           MuxGate<MuxDev, 0>>;
///     using Right  = Device<I2c1, Clock, Chips::Sht3x, DefaultConfig, NoReset,
///                           MuxGate<MuxDev, 1>>;
///
///     struct App {
///         MuxArbiter arbiter{};
///         MuxDev     mux{};
///         Left       left{};
///         Right      right{};
///
///         App() {
///             left.gate().bind(mux, arbiter);
///             right.gate().bind(mux, arbiter);
///         }
///     };
///
/// Both SHT3x are at 0x44, which is legal because they are never on the wire at the same
/// time. The mux itself is an ordinary ungated Device and must have its `handler()` called
/// like any other -- put it in the same `Bus`.
namespace Kvasir::I2C {

// -- how the parts behind one switch share it ------------------------------------------------

/// A mux policy decides the two things about sharing a switch that are a matter of taste rather
/// than of correctness. The rest holds under every policy: a free switch goes to whoever claims
/// it first, a part of another channel than the holder's waits, and nothing moves the switch
/// inside a transaction.
///
/// - `mayJoin(arbiter, channel)`: a part of the channel that holds the switch wants it too.
///   May it go at once, alongside the parts of that channel already mid-script?
/// - `freesOnRelease(arbiter, channel)`: a part lets go while others of its channel still hold
///   the switch. Is it free for the next claim anyway?
///
/// Both see the arbiter: who holds the switch (`holder`, `holders`) and which channels were
/// refused it since they last had it (`waiting`, a bit per channel). A policy is any type with
/// those two static functions; the policy is a template parameter of the gates and of their
/// arbiter, so the parts of one switch cannot disagree on it. The three below are compared
/// in test/i2c/mux_test.cpp.
template<typename P, typename Arbiter>
concept MuxPolicy = requires(Arbiter const& arbiter, std::uint8_t channel) {
    { P::mayJoin(arbiter, channel) } -> std::convertible_to<bool>;
    { P::freesOnRelease(arbiter, channel) } -> std::convertible_to<bool>;
};

/// The default. The parts of one channel share the switch without counting each other: any of
/// them goes while its channel holds it, and the first to let go frees it -- a part still
/// mid-script claims it again for its next transaction, and may find another channel has it.
/// The switch changes hands most often this way, so a fast part on another channel waits least
/// for it. What it gives up is a held time that means "until the channel was done": the
/// arbiter's held time runs from a grant to the first release.
struct ShareChannel {
    [[nodiscard]] static constexpr bool mayJoin(auto const&,
                                                std::uint8_t) {
        return true;
    }

    [[nodiscard]] static constexpr bool freesOnRelease(auto const&,
                                                       std::uint8_t) {
        return true;
    }
};

/// The switch stays with a channel until the last of its parts mid-script lets go, and a part
/// of that channel may join them at any time. No other channel gets the switch between two
/// steps of a script on this one, and the held time is the channel's real time with the switch -- but
/// a busy channel whose parts overlap keeps the switch for as long as they do.
struct HoldChannel {
    [[nodiscard]] static constexpr bool mayJoin(auto const&,
                                                std::uint8_t) {
        return true;
    }

    [[nodiscard]] static constexpr bool freesOnRelease(auto const&,
                                                       std::uint8_t) {
        return false;
    }
};

/// HoldChannel, except that a part of the holding channel does not join while another channel
/// waits: the parts mid-script finish and the switch moves on, which bounds how long one
/// channel can keep it. The price is that the parts of a channel go one after another whenever
/// anyone else waits -- on a busy bus, nearly always.
struct DrainChannel {
    [[nodiscard]] static constexpr bool mayJoin(auto const&  arbiter,
                                                std::uint8_t channel) {
        return (arbiter.waiting & static_cast<std::uint8_t>(~(1U << channel))) == 0;
    }

    [[nodiscard]] static constexpr bool freesOnRelease(auto const&,
                                                       std::uint8_t) {
        return false;
    }
};

/// Who currently owns one switch, under `Policy`.
///
/// An object the application owns rather than a static, and keyed on the *switch* rather than
/// the channel: the channels of one switch are handed the same arbiter, which is what makes
/// them exclude each other. Two switches get two arbiters and never interact.
///
/// Plain, not atomic: every caller is the main loop -- `handler()` and the `progress_` under
/// it. `request<G>()` is the only thing an interrupt calls, and it submits nothing.
template<typename Policy = ShareChannel>
struct BasicMuxArbiter {
    using PolicyT = Policy;

    static constexpr std::uint8_t NoHolder = 0xFF;

    std::uint8_t  holder{NoHolder};   ///< the channel the switch belongs to
    std::uint8_t  holders{};          ///< parts of that channel holding it now
    std::uint8_t  waiting{};          ///< a bit per channel refused since it last got it
    std::uint32_t stint{};            ///< steps at every grant: a hold from before it is over

    /// How long a channel held the switch, and when the current holder got it: typed
    /// durations, in whatever the clock counts, so a stats line converts once.
    using Duration = std::chrono::microseconds;

    // Where the switch's time goes, per channel since power-up: how often a channel took it,
    // how long it held it, and how many times a part of it had to wait for another channel --
    // `waited[channel][holder]`, one per wait however many turns it lasted, so a part short of
    // its rate can be traced to the channel that kept it off the wire. A poller takes deltas.
    std::array<std::uint32_t, 8>                grants{};
    std::array<Duration, 8>                     held{};
    std::array<std::array<std::uint32_t, 8>, 8> waited{};
    Duration                                    grantedAt{};   ///< on the clock's epoch
};

/// The arbiter under the default policy, which is what a firmware that does not choose names.
using MuxArbiter = BasicMuxArbiter<>;

/// One place on a switch a client claims: a channel, 0..7, or `Closed` -- every channel shut,
/// only the wire in front of the switch. A MuxGate is one of these with its channel fixed in
/// its type; a scan of the whole bus moves one over every position (BusScan.hpp).
///
/// `claim(position)` is called every turn the client wants to submit and answers "not yet"
/// until the client holds the switch *and* the switch has been written for that position -- the
/// write is an ordinary write group on the mux device, so it goes out through the same queue and
/// the same retry and absent handling as everything else. Only one position holds the switch at
/// a time; how the clients of one position share it is the `Policy`'s.
template<typename MuxDevice, typename Policy = ShareChannel>
class MuxPort {
public:
    using Mux      = MuxDevice;
    using Channels = typename Mux::Chip::Channels;
    using Arbiter  = BasicMuxArbiter<Policy>;

    /// Every channel shut. It has no row in the arbiter's per-channel statistics.
    static constexpr std::uint8_t Closed = 8;

    static_assert(MuxPolicy<Policy,
                            Arbiter>,
                  "a mux policy is a type with static mayJoin(arbiter, channel) and "
                  "freesOnRelease(arbiter, channel), each returning bool (Mux.hpp)");

    /// Say which switch this is on, and which arbiter it shares with the other clients of that
    /// switch. Called once, before the client runs.
    void bind(Mux&     mux,
              Arbiter& arbiter) {
        mux_     = &mux;
        arbiter_ = &arbiter;
    }

    [[nodiscard]] bool bound() const { return mux_ != nullptr && arbiter_ != nullptr; }

    /// True once this client holds the switch at `position` *and* the switch has been told.
    /// Anything else is "come back next turn". Claiming another position than the one held lets
    /// that one go first.
    bool claim(std::uint8_t position) {
        if(!bound()) {
            // Nothing was ever bound, so this client can never talk. Silence would look like
            // an absent part; say it once instead.
            if(!warned_) {
                warned_ = true;
                UC_LOG_W(
                  "a device on mux channel {} was never bound to a switch: bind() it "
                  "from the object that owns the mux",
                  position);
            }
            return false;
        }
        if(holding_ && position != position_) { release(); }
        auto const mask = maskOf_(position);
        if(!holds_()) {
            if(arbiter_->holder == Arbiter::NoHolder) {
                refused_            = false;
                arbiter_->holder    = position;
                arbiter_->holders   = 1;
                arbiter_->waiting   = static_cast<std::uint8_t>(arbiter_->waiting & ~mask);
                arbiter_->grantedAt = sinceEpoch_();
                if(position < Closed) { ++arbiter_->grants[position]; }
                ++arbiter_->stint;
                hold_(position);
                // A switch already at this position -- the write done and not owed again after
                // a reset -- needs no write, and the check below lets the transaction go this
                // turn. Otherwise the write goes on the bus now rather than on the switch's own
                // turn: waiting for it costs a whole loop turn per channel change, and in a busy
                // loop a turn can be milliseconds, enough for a part on a 10 ms period behind
                // the switch to lose samples.
                if(mux_->template value<Channels>() != mask) {
                    mux_->template set<Channels>(mask);
                    mux_->handler();
                    return false;   // the write has to complete first
                }
            } else if(arbiter_->holder == position && Policy::mayJoin(*arbiter_, position)) {
                refused_ = false;
                ++arbiter_->holders;
                hold_(position);
            } else {
                arbiter_->waiting = static_cast<std::uint8_t>(arbiter_->waiting | mask);
                // One wait, however many turns it is asked again before it ends.
                if(!refused_ && position < Closed && arbiter_->holder < Closed) {
                    ++arbiter_->waited[position][arbiter_->holder];
                }
                refused_   = true;
                waitingAt_ = position;
                return false;
            }
        }
        return mux_->answering() && !mux_->pending() && mux_->template value<Channels>() == mask;
    }

    /// Done with the switch for now: the run waits or ends. Only a hold this client still has
    /// is given back -- a second release, a release by a client that never got the switch, or
    /// one whose hold ended when the policy freed the switch under it, frees nobody else's.
    void release() {
        if(refused_ && bound()) {
            // A run that ends while waiting ends the wait too, and takes its bit out of
            // `waiting`: left there, it would keep DrainChannel refusing joins to the holder
            // for good. Another client still waiting on that position sets it again on its
            // next claim.
            arbiter_->waiting = static_cast<std::uint8_t>(arbiter_->waiting & ~maskOf_(waitingAt_));
        }
        refused_ = false;
        if(!bound() || !holds_()) {
            holding_ = false;
            return;
        }
        holding_ = false;
        --arbiter_->holders;
        if(arbiter_->holders == 0 || Policy::freesOnRelease(*arbiter_, position_)) {
            if(position_ < Closed) {
                arbiter_->held[position_] += sinceEpoch_() - arbiter_->grantedAt;
            }
            arbiter_->holder  = Arbiter::NoHolder;
            arbiter_->holders = 0;
        }
    }

private:
    [[nodiscard]] static constexpr std::uint8_t maskOf_(std::uint8_t position) {
        return position < Closed ? static_cast<std::uint8_t>(1U << position) : std::uint8_t{0};
    }

    /// This client got the switch in the arbiter's current stint and has not let go. A policy
    /// that frees the switch while others of the position still hold it ends their holds too:
    /// the next grant is a new stint, and a hold from an older one is over.
    [[nodiscard]] bool holds_() const {
        return holding_ && stint_ == arbiter_->stint && arbiter_->holder == position_;
    }

    void hold_(std::uint8_t position) {
        holding_  = true;
        stint_    = arbiter_->stint;
        position_ = position;
    }

    [[nodiscard]] static typename Arbiter::Duration sinceEpoch_() {
        using Clock = typename Mux::ClockT;
        return std::chrono::duration_cast<typename Arbiter::Duration>(
          Clock::now().time_since_epoch());
    }

    Mux*          mux_{nullptr};
    Arbiter*      arbiter_{nullptr};
    std::uint32_t stint_{};
    std::uint8_t  position_{};
    bool          holding_{};     ///< this client got the switch and has not let it go
    bool          refused_{};     ///< refused since it last got the switch: a wait is under way
    std::uint8_t  waitingAt_{};   ///< the position that wait is for
    bool          warned_{};
};

/// One channel of an I2C switch, as a gate: a MuxPort that always claims the same channel.
///
/// `Channel` is a template parameter and not a member because `Bus` decides whether two
/// devices collide by comparing their gate *types* (Bus.hpp): two channels of one switch have
/// to be different types for "the same part on two channels" to stay legal, and for two
/// devices on the *same* channel to still be caught.
template<typename MuxDevice, std::uint8_t Channel, typename Policy = ShareChannel>
struct MuxGate {
    static constexpr bool Gated = true;
    static constexpr bool Front = false;

    using Mux      = MuxDevice;
    using Channels = typename Mux::Chip::Channels;
    using Arbiter  = BasicMuxArbiter<Policy>;

    static constexpr std::uint8_t Mask = static_cast<std::uint8_t>(1U << Channel);

    static_assert(Channel < 8,
                  "a TCA9548A has eight channels");

    /// Say which switch this channel is on, and which arbiter it shares with the other
    /// channels of that switch. Called once, before the device is handled.
    void bind(Mux&     mux,
              Arbiter& arbiter) {
        port_.bind(mux, arbiter);
    }

    [[nodiscard]] bool bound() const { return port_.bound(); }

    /// True once this part holds the switch *and* the switch has been told (MuxPort::claim).
    bool claim() { return port_.claim(channel_); }

    void release() { port_.release(); }

private:
    MuxPort<MuxDevice, Policy> port_{};
    /// The channel as a byte rather than the template parameter, though the parameter is where
    /// it comes from. `Channel` stays in the *type*, which is what `Bus` compares to decide
    /// whether two parts at one address can be on the wire together -- but nothing the
    /// compiler generates depends on it any more, so the driver of a part on channel 2 and the
    /// driver of the same part on channel 3 are identical code and the linker folds them into
    /// one (`--icf=all` is in the SDK's link flags). Two SHT4x, two LCDs and four OLED panels
    /// cost 4 566 bytes of second copies on the i2c_testing bench before this (2026-09-21).
    std::uint8_t channel_{Channel};
};

/// A part in front of the switch that must have the wire to itself: a gate that claims the
/// switch *closed* (MuxPort::Closed), so no channel is open while the part talks, and no part
/// behind the switch sees its traffic.
///
/// A part in front of a switch is otherwise on the wire of whatever channel is open, which
/// every well-behaved part tolerates -- it is not addressed, it does not answer. This gate is for
/// the pair that does not get along: found with a u-blox SAM-M8Q in front and a TLV493D-A1B6
/// behind channel 6 (i2c_testing, 2026-09-19, seen with the RP2350's second core as a logic
/// analyser). The receiver stretches the clock after its address, and when it lets go it puts
/// its first data bit on SDA in the same 40 ns as it releases SCL (0.7 us before it on a quiet
/// bus): with a 0 bit, SDA falls as SCL rises, which is a START to a listener whose SCL input
/// sees the edge first. The count of waiting bytes is 0x0000 on nearly every poll, so what
/// follows that START is the general call address -- and the TLV493D resets on a general call
/// and reads its address pin again (user manual 5.7.1), SDA low: it comes back at 0x1F, and
/// acknowledges into the receiver's transfer on the way. The master loses arbitration, the
/// stream loses the bytes of the aborted read, and the TLV493D is gone.
///
/// For the Bus the part stays what it is, a part of segment 0 -- its address is on every
/// channel's wire whenever somebody else talks, so it must still be unique -- and only its
/// transactions wait for the switch. It shares the arbiter with the channels, so it costs them
/// two switch writes per run; a part polled twenty times a second is cheap, a display is not.
///
///     using Gnss = Device<I2c1, Clock, Chips::SamM8q<>, DefaultConfig, NoReset,
///                         MuxFrontGate<MuxDev>>;
template<typename MuxDevice, typename Policy = ShareChannel>
struct MuxFrontGate {
    static constexpr bool Gated = true;
    static constexpr bool Front = true;

    using Mux      = MuxDevice;
    using Channels = typename Mux::Chip::Channels;
    using Arbiter  = BasicMuxArbiter<Policy>;

    static constexpr std::uint8_t Mask = 0;

    void bind(Mux&     mux,
              Arbiter& arbiter) {
        port_.bind(mux, arbiter);
    }

    [[nodiscard]] bool bound() const { return port_.bound(); }

    /// True once this part holds the switch *and* the switch is closed (MuxPort::claim).
    bool claim() { return port_.claim(MuxPort<MuxDevice, Policy>::Closed); }

    void release() { port_.release(); }

private:
    MuxPort<MuxDevice, Policy> port_{};
};

}   // namespace Kvasir::I2C
