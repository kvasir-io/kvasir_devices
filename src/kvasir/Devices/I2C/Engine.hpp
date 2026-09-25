#pragma once

/// The state machine `Device<>` runs, in one place that knows nothing about the chip.
///
/// `Device<I2c, Clock, Chip, Config, Reset, Gate>` is instantiated once per device, so every
/// line of the engine used to be printed into the image that often: 96 KB of `Device<>` members
/// plus 40 KB of `Bus::handler()` with each device's `handler()` inlined, in a 312 KB firmware
/// with 57 devices (i2c_testing, 2026-09-20). `--icf=all` is already in the SDK's link flags
/// and folds none of it -- the copies differ by constants and types -- so the only way down is
/// to share by design.
///
/// The state is what makes that possible: of `Device`'s members, the ones below know nothing
/// about the chip. Where the run is (`phase_`, `running_`, `group_`, `step_`, `item_`), what it
/// waits for (`waiting_`, `waitUntil_`, `holdUntil_`), what the bus owes it (`inFlight_`,
/// `submittedAt_`), and what has happened to it so far (`errors_`, `bringUps_`,
/// `consecutiveFailures_`) are the same fields for a thermometer and a display. Only the
/// buffers, the decoded samples and the chip's own functions differ, and those stay in
/// `Device`.
///
/// `docs/engine-factoring.md` has the plan this is the second stage of; `Clock` is the only
/// template parameter, so a firmware has one of these however many devices it has.
#include "EngineLog.hpp"
#include "Pending.hpp"
#include "Presence.hpp"
#include "Step.hpp"

#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

namespace Kvasir::I2C::detail {

/// What became of an on-demand read (`Device::request<G>()` and `answer<G>()`): what a run of
/// the group did for the requests it took on.
enum class Answer : std::uint8_t {
    pending,     ///< no run has ended since the request: not started yet, or on the wire
    ok,          ///< a run after the request delivered a sample
    unchanged,   ///< it ran, and the part had nothing new to say (Outcome::unchanged)
    rejected,    ///< it ran, and decode rejected what came back or its retries ran out
    failed,      ///< the part or the bus failed the run: a NAK, a bus fault, the in-flight net
};

/// Where a device is with its part, from power-on to running.
enum class Phase : std::uint8_t { reset, resetHeld, settle, run };

/// Which kind of script is on the wire, if any.
enum class Running : std::uint8_t { none, init, read, write, verify };

/// The engine's own state: every member of `Device` that does not depend on the chip. `I2c`
/// and `Clock` are one type each per firmware, so this exists once however many devices there
/// are.
template<typename I2c, typename Clock>
struct EngineState {
    using TimePoint = typename Clock::time_point;
    using PendingT  = Pending<I2c, Clock>;

    Phase   phase_{Phase::reset};
    Running running_{Running::none};

    bool up_{false};
    bool acked_{false};   ///< the part acknowledged a transaction since the last start
    bool identified_{false};
    bool waiting_{false};
    bool afterDelay_{false};
    bool inFlight_{false};
    bool initFailed_{false};   ///< the last bring-up failed: back off before the next
    bool unidentifiedLogged_{false};
    bool oracleFailed_{false};   ///< this bring-up: an Identity register did not match

    std::uint8_t group_{};
    std::uint8_t step_{};
    std::uint8_t item_{};
    std::uint8_t consecutiveFailures_{};

    /// The transaction on the wire, or the one about to go.
    Step current_{};

    TimePoint waitUntil_{};
    TimePoint holdUntil_{};
    TimePoint submittedAt_{};

    std::uint32_t errors_{};
    std::uint16_t bringUps_{};
    std::uint16_t unidentified_{};   ///< bring-ups setup() turned down

    /// The handshake with the bus: the callback handed to it, and what came back.
    PendingT pending_{};
};

/// The half of a read group's slot that does not depend on the chip: where the group is in
/// its cycle, what its last run did, and who is waiting for one. `ReadSlot<G>` adds the
/// buffer, the decoded Sample and the group's Request; the engine only ever needs these, so a
/// group can be walked without knowing its type.
template<typename Clock>
struct ReadSlotBase {
    using TimePoint = typename Clock::time_point;

    /// Bytes the read step transfers, and how much of the buffer decode then sees (the read
    /// lands at an offset). Both are the whole buffer unless prepare() sized the run; the
    /// derived slot sets them, since only it knows how big its buffer is.
    std::uint8_t len{};
    std::uint8_t validBytes{};

    std::uint32_t seq{};
    std::uint32_t seen{};
    std::uint32_t samples{};
    std::uint32_t rejected{};
    bool          current{};   ///< a sample since the last bring-up (valid())

    TimePoint due{};
    /// The period the group runs at, from period<G>(ms); the description's Period until the
    /// application sets one. 0 parks the group until request<G>() asks.
    std::chrono::milliseconds period{};
    bool                      periodSet{};

    /// Requests made, from any context; the runs take them in order. A run is owed while
    /// `asked` is ahead of `started`, and the requests a run took on are answered with its
    /// outcome when it ends. The rest is the loop's.
    std::atomic<std::uint32_t> asked{0};
    std::uint32_t              started{};
    std::uint32_t              served{};
    Answer                     last{Answer::pending};
    bool                       serving{};   ///< the run on the wire took requests on
    std::uint8_t               retries{};
};

/// The half of a write group's slot that does not depend on the chip: which items the chip
/// still owes, which it has been told, and when the group is next written as a whole.
/// `WriteSlot<G>` adds the values, the buffer and the read-back state.
template<typename Clock>
struct WriteSlotBase {
    using TimePoint = typename Clock::time_point;

    /// Every item's dirty bit, set from any context.
    std::atomic<std::uint32_t> dirty{0};
    /// The application has set this group's value, so `Initial` is no longer what the group
    /// holds -- even if nothing has reached the chip yet.
    bool owned{};
    /// Items whose value is what the chip holds or is owed, so that set() with that value
    /// again has nothing to send.
    std::uint32_t known{};
    std::uint32_t writes{};
    TimePoint     due{};
};

/// What the engine needs of one write group, beside its slot.
struct WriteGroupInfo {
    /// Every item's bit set: what a periodic write re-dirties.
    std::uint32_t allItems{};
    /// The description's Period in milliseconds; 0 when the group is not written cyclically.
    std::uint32_t periodMs{};
    /// The group asks for its registers to be read back after they are written.
    bool verifies{};
};

/// What a read group's decode() made of the bytes, as the engine needs it: the sample itself
/// is the chip's business and the trampoline has already stored it.
enum class DecodeKind : std::uint8_t {
    ok,          ///< a new sample
    unchanged,   ///< a well-formed frame that says nothing new
    rejected,    ///< a bad CRC, an impossible value
    retry,       ///< not ready: run the sequence again after `retryMs`
};

struct DecodeResult {
    DecodeKind    kind{};
    std::uint32_t retryMs{};
};

/// What the engine needs of one read group of the chip, beside its slot: everything it would
/// otherwise have had to ask the group's type for.
struct ReadGroupInfo {
    /// The description's Period in milliseconds; 0 when the group is not cyclic.
    std::uint32_t periodMs{};
    /// The group's period depends on its last sample (`period(Sample const&)`), so the next
    /// deadline is measured from the answer rather than advanced on a grid.
    bool dynamicPeriod{};
};

/// What the engine needs of the chip, as constants and function pointers: one `static
/// constexpr` table per `Device` instantiation, in flash, shared by every object of that type.
/// Each pointer's target is a one-line trampoline in `Device` that casts the state back to
/// itself -- so the switch over the chip's groups is written once per chip instead of once per
/// call site per group.
template<typename I2c, typename Clock>
struct Ops {
    using State     = EngineState<I2c, Clock>;
    using TimePoint = typename Clock::time_point;

    /// What the chip is called, for the lines the engine logs about it.

    /// The scratch a transaction is assembled in (the register bytes and the payload).
    std::span<std::byte> (*tx)(State&){};
    /// The script the current run walks, and the buffer its steps read into or write from.
    std::span<Step const> (*script)(State&){};
    std::span<std::byte> (*buffer)(State&){};

    /// Presence: false while the part is parked and no probe is due.
    bool (*mayTalk)(State&){};
    /// The gate in front of the part (a switch channel): false means "not yet, try next turn".
    bool (*claimGate)(State&){};
    void (*releaseGate)(State&){};

    /// A read group whose prepare() sized this run, and a counted read whose length an
    /// earlier step read.
    void (*sizeRead)(State&){};
    void (*countRead)(State&,
                      std::uint8_t){};
    /// The running read group's ready(): a check or stopUnless step asks it.
    bool (*ready)(State&){};

    /// The Identity registers against the data sheet, and setup() on a probe copy of the
    /// State: both say whether the script may go on.
    bool (*oracle)(State&){};
    bool (*identify)(State&){};

    /// How long a failed write, read-back or bring-up waits before it is tried again, and how
    /// many failures in a row past the bring-up configure the chip again from the start.
    std::uint32_t faultBackoffMs{};

    /// The bring-up's buffer, cleared before a run: setup() and an identify step see only
    /// this run's bytes.
    void (*clearInit)(State&){};
    void (*finishInit)(State&,
                       TimePoint){};
    /// The wake-retry count, which every new transaction starts from.
    void (*wakeReset)(State&){};
    /// The write item on the wire is still owed; the read on the wire is owed again with its
    /// requests still pending; and no sample stands for a part that is being brought up again.
    void (*redirtyItem)(State&){};
    void (*unserve)(State&){};
    void (*forgetSamples)(State&){};

    /// setup() over what the bring-up read: false is "answers, but is not this chip". How
    /// long before such a part is tried again; 0 runs the description anyway.
    bool (*setupFinal)(State&){};
    std::uint32_t unidentifiedRetryMs{};
    /// Every group's first deadline after a bring-up, and the values the chip owes again
    /// because it came back at its defaults.
    void (*startupSchedule)(State&,
                            TimePoint){};

    /// Presence (Presence.hpp): what the device may do this turn, and what a transaction's
    /// outcome says about whether the part is there.
    PresenceTurn (*presenceTurn)(State&,
                                 TimePoint){};
    void (*presenceAck)(State&,
                        TimePoint){};
    void (*presenceNak)(State&){};
    void (*presenceFault)(State&){};
    /// A request the bus never answered: the net under its "exactly one callback".
    std::uint32_t inFlightTimeoutMs{};

    /// A NAK the part gives while it wakes: true when the transaction was put on the wire
    /// again rather than counted.
    bool (*wakeRetry)(State&,
                      TimePoint){};

    /// Behind a bridge that can be switched off (Bridge.hpp): false while the engine has to
    /// keep still. Always true for a device that is not behind one.
    bool (*bridgeTurn)(State&,
                       TimePoint){};

    /// A chip with a reset line of its own, and the three waits of a power-on.
    void (*resetHold)(State&){};
    void (*resetRelease)(State&){};
    std::uint32_t startupDelayMs{};
    std::uint32_t resetLowMs{};
    std::uint32_t resetSettleMs{};

    /// The chip's cyclic read groups: one entry each, and the half of their slots the engine
    /// can touch without knowing the group's type.
    std::span<ReadGroupInfo const> reads{};
    ReadSlotBase<Clock>& (*readSlot)(State&,
                                     std::uint8_t){};
    /// The period a group runs at now (what `period<G>(ms)` set, or the chip State's), and
    /// the one a group whose period follows its readings asks for after this sample.
    std::chrono::milliseconds (*periodNow)(State&,
                                           std::uint8_t){};
    std::chrono::milliseconds (*dynamicPeriod)(State&,
                                               std::uint8_t){};
    /// The group's prepare() and the buffer lengths this run works with.
    void (*prepareRead)(State&,
                        std::uint8_t){};
    /// The group's decode() over what came back: it stores the sample (and the timestamp,
    /// where the group keeps one) and says what the engine has to do about it.
    DecodeResult (*decode)(State&,
                           std::uint8_t){};

    /// The chip's write groups, the same way.
    std::span<WriteGroupInfo const> writes{};
    WriteSlotBase<Clock>& (*writeSlot)(State&,
                                       std::uint8_t){};
    /// One item encoded into the script the engine then walks.
    void (*encodeWrite)(State&,
                        std::uint8_t,
                        std::uint8_t){};
    /// The chip now holds the value: what the description makes of that (`applied`), and the
    /// read-back it owes.
    void (*afterWrite)(State&,
                       std::uint8_t,
                       std::uint8_t,
                       TimePoint){};
    /// A read-back that is due, if any: true when one went on the wire.
    bool (*startVerifyGroup)(State&,
                             std::uint8_t,
                             TimePoint){};
    /// What came back compared with what was written.
    void (*finishVerify)(State&,
                         TimePoint){};

    // The small fields last, together. Between the pointers each cost three bytes of padding,
    // and the six durations above were 8-byte `std::chrono::milliseconds` that aligned the
    // whole table to 8: 224 bytes x 57 devices was 12.8 KB of the i2c_testing bench (2026-09-21).
    std::string_view name{};
    std::uint8_t     address{};
    /// 0, 1 or 2: how a register is addressed on this chip.
    std::uint8_t registerBytes{};
    /// True when the chip sends nothing to come up: link() then stays `starting` until its
    /// first transaction is acknowledged.
    bool         initEmpty{};
    std::uint8_t faultsBeforeReinit{};
    bool         bridged{};
    bool         hasResetLine{};
    /// How often a decode or a check step may ask for a retry before the sample is rejected.
    std::uint8_t maxRetries{};
};

/// The state machine. Every function here is one copy in the image for the whole firmware:
/// what it needs of the chip comes from `Ops`, what it needs of the run from `EngineState`.
template<typename I2c, typename Clock>
struct Engine {
    using State     = EngineState<I2c, Clock>;
    using OpsT      = Ops<I2c, Clock>;
    using TimePoint = typename Clock::time_point;
    using PendingT  = Pending<I2c, Clock>;

    /// When a wait of `delay` from `now` is over. `now` was read at some point inside a tick of
    /// the clock, so one tick more makes the wait at least `delay` long however late in its
    /// tick that was: on a millisecond clock a 1 ms delay would otherwise end on the very next
    /// tick, which can be microseconds away. A zero delay stays zero.
    /// A duration from one of the table's millisecond counts.
    [[nodiscard]] static constexpr std::chrono::milliseconds ms(std::uint32_t count) {
        return std::chrono::milliseconds{count};
    }

    [[nodiscard]] static TimePoint atLeast(TimePoint                 now,
                                           std::chrono::milliseconds delay) {
        if(delay == std::chrono::milliseconds::zero()) { return now; }
        return now + delay + typename Clock::duration{1};
    }

    /// The run is about to wait, so the gate is let go for it and claimed again before the
    /// next transaction: a part behind a switch does not hold its channel while it waits.
    static void wait(State&                    e,
                     OpsT const&               ops,
                     TimePoint                 now,
                     std::chrono::milliseconds delay,
                     bool                      thenNextStep) {
        e.waiting_    = true;
        e.afterDelay_ = thenNextStep;
        e.waitUntil_  = atLeast(now, delay);
        ops.releaseGate(e);
    }

    /// A run is over -- finished, failed, rejected, or abandoned by a reset. Every exit from
    /// a script goes through here, so a gate is never held a turn past the end of a run.
    static void endRun(State&      e,
                       OpsT const& ops) {
        ops.releaseGate(e);
        e.running_ = Running::none;
    }

    /// Start over from the reset: parked, probed, or the in-flight net. Whatever was on the
    /// wire is abandoned -- but not what it was for: a write in flight is owed again (its
    /// dirty bit went at startWrite_), or a Transient one -- an EEPROM page, a clock set --
    /// would be lost, since the bring-up does not replay it. And the gate is let go.
    static void reset(State&      e,
                      OpsT const& ops) {
        abandonRun_(e, ops);
        e.phase_      = Phase::reset;
        e.up_         = false;
        e.acked_      = false;
        e.initFailed_ = false;
        ops.forgetSamples(e);
    }

    /// The bridge went, and the parts behind it keep their registers (WhileOff::disconnected):
    /// the run on the wire is abandoned as in reset() -- a write owed again, a read's requests
    /// still pending -- but the bring-up stands. The samples do not: valid() waits for one
    /// taken after the return.
    static void pause(State&      e,
                      OpsT const& ops) {
        abandonRun_(e, ops);
        ops.forgetSamples(e);
    }

    /// The bring-up from its first step, or straight to its end where the chip sends nothing.
    static void beginInit(State&      e,
                          OpsT const& ops,
                          TimePoint   now) {
        e.running_ = Running::init;
        e.step_    = 0;
        ops.clearInit(e);
        e.oracleFailed_ = false;
        if(ops.initEmpty) {
            // Nothing to send, so nothing has answered: link() stays `starting` until the
            // chip's first transaction -- a cyclic read, or whatever the application asks --
            // is acknowledged (progress_). Declared up at once, a missing chip without Init
            // would come "up" at every probe while its first read NAKs.
            ops.finishInit(e, now);
        } else {
            startStep(e, ops, now);
        }
    }

    /// The run of a read group is over: the requests it took on have their answer.
    static void answered(ReadSlotBase<Clock>& s,
                         Answer               how) {
        if(!s.serving) { return; }
        s.serving = false;
        s.served  = s.started;
        s.last    = how;
    }

    /// The next deadline one period on, or one period from now when the last one is already
    /// past: a group that fell behind resumes at its period rather than bursting to catch up.
    static void advanceDue(TimePoint&                due,
                           TimePoint                 now,
                           std::chrono::milliseconds period) {
        if(now < due) { return; }
        due += period;
        if(due < now) { due = now + period; }
    }

    /// The next deadline of a periodic group, when this run was the periodic one; a run that
    /// was only requested leaves the period alone.
    ///
    /// A group with a period(sample) is measured from this completion instead, because what
    /// it is asking for is "so long after the last answer", and the period it asks for
    /// depends on that answer. A period of 0 parks the group until request<G>() asks for it.
    static void reschedule(State&       e,
                           OpsT const&  ops,
                           std::uint8_t g,
                           TimePoint    now) {
        auto&       s  = ops.readSlot(e, g);
        auto const& gi = ops.reads[g];
        if(gi.dynamicPeriod) {
            auto const p = s.periodSet ? s.period : ops.dynamicPeriod(e, g);
            s.due        = p > std::chrono::milliseconds::zero() ? now + p : TimePoint::max();
        } else if(gi.periodMs != 0) {
            auto const p = ops.periodNow(e, g);
            if(p > std::chrono::milliseconds::zero()) {
                advanceDue(s.due, now, p);
            } else {
                s.due = TimePoint::max();   // parked by period<G>(0)
            }
        }
    }

    /// Run the read group again from its first step after `retryAfter`, or give up on this
    /// run once the retry limit is reached: the sample is rejected.
    static void again(State&                    e,
                      OpsT const&               ops,
                      TimePoint                 now,
                      std::chrono::milliseconds retryAfter) {
        auto& s = ops.readSlot(e, e.group_);
        if(s.retries < ops.maxRetries) {
            ++s.retries;
            e.step_ = 0;
            wait(e, ops, now, retryAfter, false);
        } else {
            ++s.rejected;
            s.retries = 0;
            answered(s, Answer::rejected);
            reschedule(e, ops, e.group_, now);
            endRun(e, ops);
        }
    }

    /// The due read group with the earliest deadline, a requested one first.
    static void startRead(State&      e,
                          OpsT const& ops,
                          TimePoint   now) {
        bool         found = false;
        std::uint8_t pick  = 0;
        TimePoint    best{};
        for(std::uint8_t g = 0; g < static_cast<std::uint8_t>(ops.reads.size()); ++g) {
            auto&      s         = ops.readSlot(e, g);
            bool const requested = s.asked.load(std::memory_order_acquire) != s.started;
            bool const due       = requested || (ops.reads[g].periodMs != 0 && now >= s.due);
            if(!due) { continue; }
            TimePoint const when = requested ? TimePoint{} : s.due;
            if(!found || when < best) {
                found = true;
                best  = when;
                pick  = g;
            }
        }
        if(!found) { return; }
        ops.prepareRead(e, pick);
        auto& s   = ops.readSlot(e, pick);
        s.retries = 0;
        // Taken at the start: a request() made while this run is on the wire is answered by
        // another run.
        auto const asked = s.asked.load(std::memory_order_acquire);
        s.serving        = asked != s.started;
        s.started        = asked;
        e.running_       = Running::read;
        e.group_         = pick;
        e.step_          = 0;
        startStep(e, ops, now);
    }

    /// The end of a script: what it was for decides what happens now.
    static void finish(State&      e,
                       OpsT const& ops,
                       TimePoint   now) {
        switch(e.running_) {
        case Running::init:   finishInit(e, ops, now); break;
        case Running::read:   finishRead(e, ops, now); break;
        case Running::write:  finishWrite(e, ops, now); break;
        case Running::verify: ops.finishVerify(e, now); break;
        case Running::none:   break;
        }
    }

    /// A cyclic read is over: what its decode made of the bytes, then the group's next deadline.
    static void finishRead(State&      e,
                           OpsT const& ops,
                           TimePoint   now) {
        auto const r = ops.decode(e, e.group_);
        if(r.kind == DecodeKind::retry) {
            // the whole sequence again, or rejected
            again(e, ops, now, ms(r.retryMs));
            return;
        }
        auto& s = ops.readSlot(e, e.group_);
        switch(r.kind) {
        case DecodeKind::ok:
            ++s.seq;
            ++s.samples;
            s.current = true;
            answered(s, Answer::ok);
            break;
        case DecodeKind::unchanged: answered(s, Answer::unchanged); break;
        case DecodeKind::rejected:
            ++s.rejected;
            answered(s, Answer::rejected);
            break;
        case DecodeKind::retry: break;   // handled above
        }
        s.retries = 0;
        reschedule(e, ops, e.group_, now);
        endRun(e, ops);
    }

    /// A write is on the chip: the description hears about it, and the read-back is owed.
    static void finishWrite(State&      e,
                            OpsT const& ops,
                            TimePoint   now) {
        ++ops.writeSlot(e, e.group_).writes;
        ops.afterWrite(e, e.group_, e.item_, now);
        endRun(e, ops);
    }

    /// The first dirty item of the first dirty write group, encoded and on the wire.
    static bool startWrite(State&      e,
                           OpsT const& ops,
                           TimePoint   now) {
        for(std::uint8_t w = 0; w < static_cast<std::uint8_t>(ops.writes.size()); ++w) {
            auto&       s  = ops.writeSlot(e, w);
            auto const& gi = ops.writes[w];
            if(gi.periodMs != 0 && now >= s.due) {   // a periodic write: everything again
                advanceDue(s.due, now, ms(gi.periodMs));
                s.dirty.store(gi.allItems, std::memory_order_relaxed);
            }
            auto const d = s.dirty.load(std::memory_order_acquire);
            if(d == 0) { continue; }
            auto const item = static_cast<std::uint8_t>(std::countr_zero(d));
            // set() during the write re-dirties
            s.dirty.fetch_and(~(1U << item), std::memory_order_relaxed);
            ops.encodeWrite(e, w, item);
            e.running_ = Running::write;
            e.group_   = w;
            e.item_    = item;
            e.step_    = 0;
            // Through startStep like the read and init paths, so a script of more than one
            // transaction is walked by the same code.
            startStep(e, ops, now);
            return true;
        }
        return false;
    }

    /// Read one written register back, where a group asks for it and one is due.
    static bool startVerify(State&      e,
                            OpsT const& ops,
                            TimePoint   now) {
        for(std::uint8_t w = 0; w < static_cast<std::uint8_t>(ops.writes.size()); ++w) {
            if(!ops.writes[w].verifies) { continue; }
            if(ops.startVerifyGroup(e, w, now)) { return true; }
        }
        return false;
    }

    /// Once per loop turn: everything happens here.
    static void handler(State&      e,
                        OpsT const& ops,
                        TimePoint   now) {
        if(ops.bridged && !ops.bridgeTurn(e, now)) { return; }
        switch(ops.presenceTurn(e, now)) {
        case PresenceTurn::wait: return;   // parked, no probe due: nothing runs
        case PresenceTurn::park:           // just parked: start over, then wait
            reset(e, ops);
            return;
        case PresenceTurn::probe:   // the bring-up is the probe
            reset(e, ops);
            break;
        case PresenceTurn::talk: break;
        }
        if(e.inFlight_ && now - e.submittedAt_ > ms(ops.inFlightTimeoutMs)) {
            logNoBusAnswer(ops.name, ops.address, ms(ops.inFlightTimeoutMs));
            e.inFlight_ = false;
            e.pending_.clear();
            ++e.errors_;
            ops.presenceFault(e);
            fail(e, ops, now);
        }
        turn(e, ops, now);
    }

    /// Where the device is with its part decides what the turn does.
    static void turn(State&      e,
                     OpsT const& ops,
                     TimePoint   now) {
        switch(e.phase_) {
        case Phase::reset:
            {
                // A bring-up that failed is not tried again on the very next turn: a chip
                // with no startupDelay on a faulting bus would otherwise be asked every turn.
                auto const backoff
                  = e.initFailed_ ? ms(ops.faultBackoffMs) : std::chrono::milliseconds{0};
                e.initFailed_ = false;
                if(ops.hasResetLine) {
                    ops.resetHold(e);
                    e.waitUntil_ = now + ms(ops.resetLowMs) + backoff;
                    e.phase_     = Phase::resetHeld;
                } else {
                    e.waitUntil_ = now + ms(ops.startupDelayMs) + backoff;
                    e.phase_     = Phase::settle;
                }
            }
            break;
        case Phase::resetHeld:
            if(now > e.waitUntil_) {
                ops.resetRelease(e);
                e.waitUntil_ = now + ms(ops.resetSettleMs) + ms(ops.startupDelayMs);
                e.phase_     = Phase::settle;
            }
            break;
        case Phase::settle:
            if(now > e.waitUntil_) {
                e.phase_ = Phase::run;
                beginInit(e, ops, now);
            }
            break;
        case Phase::run:
            if(e.running_ != Running::none) {
                progress(e, ops, now);
            } else if(e.up_) {
                schedule(e, ops, now);
            } else {
                beginInit(e, ops, now);
            }
            break;
        }
    }

    /// Nothing is running: what starts next. A verify goes between the writes and the reads --
    /// it only runs when nothing is dirty, so it cannot delay a write, and at a bounded
    /// interval it does not meaningfully slow the reads.
    static void schedule(State&      e,
                         OpsT const& ops,
                         TimePoint   now) {
        if(now < e.holdUntil_) { return; }
        if(startWrite(e, ops, now)) { return; }
        if(startVerify(e, ops, now)) { return; }
        startRead(e, ops, now);
    }

    /// A run is under way: the wait it is in, the transaction it owes, or what the bus said.
    static void progress(State&      e,
                         OpsT const& ops,
                         TimePoint   now) {
        if(e.waiting_) {
            if(now < e.waitUntil_) { return; }
            e.waiting_ = false;
            if(e.afterDelay_) {
                e.afterDelay_ = false;
                nextStep(e, ops, now);   // the step's delay is over
            } else {
                startStep(e, ops, now);   // a retry: step_ was put back to the start
            }
            return;
        }
        if(!e.inFlight_) {
            e.inFlight_ = submit(e, ops, e.current_, ops.buffer(e));   // refused last turn
            return;
        }
        switch(e.pending_.take()) {
        case PendingT::Outcome::running: return;
        case PendingT::Outcome::ok:
            e.inFlight_            = false;
            e.consecutiveFailures_ = 0;
            ops.wakeReset(e);
            ops.presenceAck(e, now);
            if(!e.acked_) {
                e.acked_ = true;
                // no Init: the first ACK is the chip coming up
                if(e.up_) { logUp(ops.name, ops.address, e.identified_); }
            }
            afterTransaction(e, ops, now);
            return;
        case PendingT::Outcome::notAcknowledged:
            // The device did not answer: what parks it, in the end -- unless the step is a
            // write the part is known not to acknowledge (Step mayNak), or it is a part that
            // NAKs while it wakes and has retries left for this transaction.
            e.inFlight_ = false;
            if(e.current_.mayNak) {
                afterTransaction(e, ops, now);
                return;
            }
            if(ops.wakeRetry(e, now)) { return; }
            ++e.errors_;
            ops.presenceNak(e);
            fail(e, ops, now);
            return;
        case PendingT::Outcome::failed:
            // A bus fault: says nothing about the device. After a write the part is known not
            // to acknowledge it is no news either: a part that resets in the middle of the
            // byte lets go of the bus wherever it happens to be, and the controller sees that
            // as often as a lost arbitration as a NAK (LTR390 software reset on the
            // i2c_testing bench, 2026-09-18: thirty of thirty). The script goes on as after
            // the NAK; a bus that is really at fault fails the step after it.
            e.inFlight_ = false;
            if(e.current_.mayNak) {
                afterTransaction(e, ops, now);
                return;
            }
            ++e.errors_;
            ops.presenceFault(e);
            fail(e, ops, now);
            return;
        }
    }

    /// The bring-up is over: what the chip said about itself decides whether it is the chip
    /// the description is for, and then every group starts from now.
    static void finishInit(State&      e,
                           OpsT const& ops,
                           TimePoint   now) {
        endRun(e, ops);
        e.identified_ = ops.setupFinal(e);
        // The data sheet's identity has the last word: a part that failed it is not the chip,
        // whatever setup() makes of a buffer the script never got to fill.
        if(e.oracleFailed_) { e.identified_ = false; }
        if(!e.identified_ && ms(ops.unidentifiedRetryMs) > std::chrono::milliseconds::zero()) {
            // Not the chip this description is for: nothing else goes to it. The bring-up
            // runs again after unidentifiedRetry -- a part that was still booting, or that
            // gets its id right after a reset, comes through then. link() stays `starting`.
            if(!e.unidentifiedLogged_) {
                e.unidentifiedLogged_ = true;
                logUnidentified(ops.name, ops.address, ms(ops.unidentifiedRetryMs));
            }
            ++e.unidentified_;
            e.waiting_   = false;
            e.waitUntil_ = now + ms(ops.unidentifiedRetryMs);
            e.phase_     = Phase::settle;
            return;
        }
        e.up_ = true;
        ++e.bringUps_;
        if(e.acked_) { logUp(ops.name, ops.address, e.identified_); }
        ops.startupSchedule(e, now);
    }

    /// A transaction failed: what that means for the run it was part of, and for the device.
    static void fail(State&      e,
                     OpsT const& ops,
                     TimePoint   now) {
        ops.wakeReset(e);   // the next transaction starts with every wake retry
        if(e.consecutiveFailures_ != std::numeric_limits<std::uint8_t>::max()) {
            ++e.consecutiveFailures_;
        }
        switch(e.running_) {
        case Running::init:
            // Start over from the reset, after faultBackoff; Presence parks the device after
            // enough NAKs and the first Init step becomes its probe.
            e.phase_      = Phase::reset;
            e.initFailed_ = true;
            break;
        case Running::read:
            {
                // The run is over and says nothing about the part: its requests are answered
                // and the group waits for its next period.
                auto& s   = ops.readSlot(e, e.group_);
                s.retries = 0;
                answered(s, Answer::failed);
                reschedule(e, ops, e.group_, now);
            }
            break;
        case Running::write:
            ops.redirtyItem(e);
            e.holdUntil_ = now + ms(ops.faultBackoffMs);
            break;
        case Running::verify:
            // a bus fault says nothing about the register: still owed, looked at again
            // shortly. Only a mismatch counts against the rewrite budget.
            e.holdUntil_ = now + ms(ops.faultBackoffMs);
            break;
        case Running::none: break;
        }
        endRun(e, ops);
        // Past the bring-up and still failing: the chip is configured again from the
        // start. After the per-phase handling above, so a write is owed before the
        // bring-up that would otherwise not replay a Transient one.
        if(e.phase_ == Phase::run && e.consecutiveFailures_ >= ops.faultsBeforeReinit) {
            e.consecutiveFailures_ = 0;
            e.waiting_             = false;
            e.up_                  = false;
            e.acked_               = false;
            e.phase_               = Phase::reset;
            ops.forgetSamples(e);
        }
    }

    /// Puts one transaction on the bus. False when it could not go (the gate said not yet,
    /// the device is parked with no probe due, the bus queue is full): the step is tried
    /// again next turn.
    static bool submit(State&               e,
                       OpsT const&          ops,
                       Step const&          s,
                       std::span<std::byte> buf) {
        // Parked: nothing goes out but the one probe per interval, and that one before the
        // gate is asked, so a parked device never holds a switch.
        if(!ops.mayTalk(e)) { return false; }
        auto const  tx = ops.tx(e);
        std::size_t n  = 0;
        if(s.hasRegister) {
            if(s.regFromBuffer) {
                for(std::size_t i = 0; i < ops.registerBytes; ++i) { tx[i] = buf[s.reg + i]; }
            } else if(ops.registerBytes == 2) {
                tx[0] = static_cast<std::byte>(s.reg >> 8);
                tx[1] = static_cast<std::byte>(s.reg & 0xFF);
            } else {
                tx[0] = static_cast<std::byte>(s.reg & 0xFF);
            }
            n = ops.registerBytes;
        }
        // The gate is asked every turn until it says yes, and held from here until the run
        // waits or ends. What a switch must not do is move in the middle of a transaction --
        // a register read is a write and a read with a repeated start between them -- and that
        // is one request on the queue: a switch write another channel asks for once this
        // device lets go is queued behind it, so it cannot land inside it. "Not yet" leaves
        // the request unsubmitted and the engine retries next turn, which is the path a full
        // bus queue already takes.
        if(!ops.claimGate(e)) { return false; }
        e.pending_.clear();
        typename I2c::Request req{};
        req.address  = ops.address;
        req.callback = e.pending_.callback();
        if(s.kind == Step::Kind::write) {
            for(std::size_t i = 0; i < s.count; ++i) {
                tx[n + i] = s.fromBuffer ? buf[s.offset + i] : std::byte{s.bytes[i]};
            }
            n += s.count;
        } else {
            req.receiveData = buf.subspan(s.offset, s.count);
        }
        if(n != 0) { req.sendData = std::span<std::byte const>{tx}.first(n); }
        // Stamped before the submit: a one-byte write can complete in the interrupt before
        // submit() returns, and the net must not measure from after that.
        e.submittedAt_ = Clock::now();
        return I2c::submit(req);
    }

    /// The step `step_` of the running script: a wait, a question the buffer answers, or a
    /// transaction on the wire.
    static void startStep(State&      e,
                          OpsT const& ops,
                          TimePoint   now) {
        auto const steps = ops.script(e);
        e.current_       = steps[e.step_];
        if(e.running_ == Running::read && e.current_.kind == Step::Kind::read) { ops.sizeRead(e); }
        if(e.current_.kind == Step::Kind::read && e.current_.counted) {
            // The count an earlier step of this run read; 0 is nothing to read, so no
            // transaction and on to the next step.
            auto const n = e.current_.countIn(ops.buffer(e));
            if(e.running_ == Running::read) { ops.countRead(e, n); }
            if(n == 0) {
                nextStep(e, ops, now);
                return;
            }
            e.current_.count = n;
        }
        switch(e.current_.kind) {
        case Step::Kind::wait: wait(e, ops, now, e.current_.delay, true); return;
        case Step::Kind::oracle:
            if(ops.oracle(e)) {
                nextStep(e, ops, now);
            } else {
                finish(e, ops, now);
            }
            return;
        case Step::Kind::identify:
            // The same setup() the end of the bring-up runs, on a State of its own: false
            // ends the script, and finishInit_ reaches the same verdict.
            if(ops.identify(e)) {
                nextStep(e, ops, now);
            } else {
                finish(e, ops, now);
            }
            return;
        case Step::Kind::stopUnless:
            if(ops.ready(e)) {
                nextStep(e, ops, now);
            } else {
                finish(e, ops, now);
            }
            return;
        case Step::Kind::check:
            if(ops.ready(e)) {
                nextStep(e, ops, now);
            } else {
                again(e, ops, now, e.current_.delay);
            }
            return;
        default: break;
        }
        e.inFlight_ = submit(e, ops, e.current_, ops.buffer(e));
    }

    /// The step's delay, then the next step.
    static void afterTransaction(State&      e,
                                 OpsT const& ops,
                                 TimePoint   now) {
        if(e.current_.delay != std::chrono::milliseconds::zero()) {
            wait(e, ops, now, e.current_.delay, true);
        } else {
            nextStep(e, ops, now);
        }
    }

private:
    /// What reset() and pause() have in common: the run on the wire is let go of, and what it
    /// was for is owed again.
    static void abandonRun_(State&      e,
                            OpsT const& ops) {
        if(e.running_ == Running::write) { ops.redirtyItem(e); }
        if(e.running_ == Running::read) { ops.unserve(e); }
        endRun(e, ops);
        e.waiting_             = false;
        e.inFlight_            = false;
        e.consecutiveFailures_ = 0;
        ops.wakeReset(e);
        e.pending_.clear();
    }

public:
    static void nextStep(State&      e,
                         OpsT const& ops,
                         TimePoint   now) {
        ++e.step_;
        if(e.step_ < ops.script(e).size()) {
            startStep(e, ops, now);
        } else {
            finish(e, ops, now);
        }
    }
};

}   // namespace Kvasir::I2C::detail
