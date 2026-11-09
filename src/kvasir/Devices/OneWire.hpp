#pragma once
#include "Bytes.hpp"
#include "Duration.hpp"
#include "kvasir/Atomic/CriticalSection.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Util/Config.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>
#include <utility>

namespace Kvasir {

/// How a bit slot's timed part stays inside the 1-Wire limits (OneWireDefaults::slotTiming).
enum class OneWireSlotTiming : std::uint8_t {
    /// Interrupts masked over each timed part (criticalSection: up to the 70 us of the presence wait). A slot cannot
    /// be stretched; the firmware's interrupts wait that long.
    masked,
    /// No interrupt masked. Each timed part is measured against its limit afterwards (a clock read before its first
    /// pin change and after its last); one an interrupt stretched past it starts the transaction again from its reset
    /// pulse (timingFaults() counts them; after timingRetries in a row it fails). SlotGate keeps the slots out of the
    /// way of interrupts the firmware knows are coming.
    checked,
};

/// The default SlotGate: every slot may start at once.
struct OneWireAnyTime {
    static constexpr bool open(std::chrono::microseconds) { return true; }
};

/// The SlotTimer of a Clock without a ShortInterval: its now() and delay(). A checked part reads it at its start, at
/// each step and at its end, so a clock that is slow to read (a 64-bit tick count assembled from a counter and its
/// overruns) eats the part's margins.
template<typename Clock>
struct OneWireClockTimer {
    using Stamp = typename Clock::time_point;

    static Stamp stamp() { return Clock::now(); }

    static auto since(Stamp s) { return Clock::now() - s; }

    template<typename D>
    static void wait(D d) {
        Clock::delay(d);
    }
};

namespace Detail {
    /// The default SlotTimer: the Clock's ShortInterval (the SysTick clock's: one counter read), else the Clock.
    template<typename Clock>
    struct OneWireDefaultTimer {
        using type = OneWireClockTimer<Clock>;
    };

    template<typename Clock>
        requires requires { typename Clock::ShortInterval; }
    struct OneWireDefaultTimer<Clock> {
        using type = typename Clock::ShortInterval;
    };
}   // namespace Detail

/// The knobs of the 1-Wire bus, with their defaults; derive and redeclare what you change.
struct OneWireDefaults {
    /// Deadline for the transaction in flight, beyond the time its bits take: each bit slot
    /// is timed from turn(), so on a millisecond tick a slot can take up to two of them,
    /// and a transaction is given `slotBudget` per bit on top of this. It only bounds a bus
    /// that has stopped answering.
    static constexpr auto transactionTimeout = std::chrono::milliseconds{100};
    static constexpr auto slotBudget         = std::chrono::milliseconds{2};
    static constexpr auto slotTiming         = OneWireSlotTiming::masked;
    /// checked: a transaction a stretched slot restarted this often in a row fails.
    static constexpr std::uint8_t timingRetries = 3;
    // SlotTimer (a type, no default member here: it depends on the Clock): what a checked part's times are taken with -
    // `Stamp stamp()`, `since(Stamp)` (a std::chrono duration), `wait(d)`. When the config names none: the Clock's
    // ShortInterval if it has one (core_cortex_common SystickClockBase), else OneWireClockTimer<Clock>. Only intervals
    // of a part (up to ~120 us) are asked for.
    /// `static bool open(std::chrono::microseconds length)`: may a timed part this long start now? Asked before the
    /// presence wait, every write and every read slot; closed, the slot waits for a later turn() (between slots the
    /// line is idle, a reset pulse just stays low longer - tRSTL has no maximum, MAX31820 Table "AC Electrical
    /// Characteristics", but 960 us under parasite power, its Note 14).
    using SlotGate = OneWireAnyTime;
};

/// A 1-Wire master bit-banged on `Pin` (open drain: driven low, released to an input), as a
/// state machine over Clock: one transaction at a time, a reset pulse then the bytes out
/// then the bytes in, each bit slot timed against Clock::now() from turn() and the slot's
/// own microseconds spun in Clock::delay. Standard speed: reset 480 us, presence sample at
/// 70 us, the rest 410 us, slot 64 us are Maxim application note 126's (Table 2 H, I, J, B;
/// other/AN126_1wire_communication_through_software.md l.48-64); write-1 low 5 us, write-0
/// low 65 us and the read sample at 13 us (3 us low) are not - AN126 has 6, 60 and 6 + 9 -
/// but inside the MAX31820's limits: tLOW1 1-15 us, tLOW0 60-120 us, tRDV 15 us max, tSLOT
/// 60-120 us, tRSTL >= 480 us, presence low from 15-60 us for 60-240 us (other/MAX31820.md
/// l.132-141).
///
/// Each slot's timed part - the presence sample, a write's low pulse, a read's low pulse and sample - is kept inside
/// its limit one of two ways (Config::slotTiming):
/// - masked (the default): interrupts masked over the part (criticalSection, at most the 70 us of the presence wait).
///   An interrupt there stretched a write-1 past tLOW1, a read sample past tRDV or the presence sample past a short
///   presence pulse, and the transaction failed (a 126 us interrupt - an ADC handler with a soft-float PID on a 48 MHz
///   Cortex-M0+ - failed about one a second). The firmware's interrupts wait up to 70 us.
/// - checked: nothing masked; each part measured afterwards against its limit, a stretched one restarts the
///   transaction (timingFaults()), timingRetries in a row fail it. Config::SlotGate places the slots between
///   interrupts the firmware knows are coming (a heater's mains-synchronous ones), so few restart.
/// The waits between slots are never masked.
///
///     acquire() -> send(now, bytes) | receive(now, n) | sendReceive(now, bytes, n)
///     operationState(now) until it is not ongoing -> getReceivedBytes(...) -> release()
template<typename Clock, typename Pin, std::size_t Size, typename Config = OneWireDefaults>
struct OneWire {
    template<typename C>
    using SlotGateOf = typename C::SlotGate;
    using SlotGate   = ::Kvasir::Config::TypeOr<Config, OneWireAnyTime, SlotGateOf>;
    template<typename C>
    using SlotTimerOf = typename C::SlotTimer;
    using SlotTimer   = ::Kvasir::Config::
      TypeOr<Config, typename Detail::OneWireDefaultTimer<Clock>::type, SlotTimerOf>;

    static constexpr std::size_t bufferSize = Size;
    using TimePoint                         = typename Clock::time_point;
    enum class State : std::uint8_t { idle, blocked, recovering, resetPulse, sending, receiving };
    enum class OperationState : std::uint8_t { succeeded, failed, ongoing };

    static constexpr std::chrono::milliseconds transactionTimeout = Kvasir::asDuration(
      KVASIR_CONFIG_OR(Config,
                       transactionTimeout,
                       std::chrono::milliseconds{OneWireDefaults::transactionTimeout}));

    static constexpr std::chrono::milliseconds slotBudget = Kvasir::asDuration(
      KVASIR_CONFIG_OR(Config, slotBudget, std::chrono::milliseconds{OneWireDefaults::slotBudget}));

    static constexpr OneWireSlotTiming slotTiming
      = KVASIR_CONFIG_OR(Config, slotTiming, OneWireDefaults::slotTiming);
    static constexpr bool checked = slotTiming == OneWireSlotTiming::checked;

    static constexpr std::uint8_t timingRetries
      = KVASIR_CONFIG_OR(Config, timingRetries, OneWireDefaults::timingRetries);

    /// checked: timed parts an interrupt stretched past their limit (each restarted its transaction).
    [[nodiscard]] static std::uint32_t timingFaults() { return timingFaults_; }

    /// A slot's timed part, by kind.
    enum class Part : std::uint8_t { presence, write1, write0, read };

    /// checked: the last part that came out too long, and how long it took.
    struct TimingFault {
        Part                      part{};
        std::chrono::microseconds took{};
    };

    [[nodiscard]] static TimingFault lastTimingFault() { return lastTimingFault_; }

    template<typename C>
    static void getReceivedBytes(C& c) {
        assert(c.size() <= buffer_.size());
        buffer_.popInto(c);
    }

    template<typename OIT>
    static void getReceivedBytes(OIT first,
                                 OIT last) {
        // the caller asks for what was received: front() on an empty buffer is the queue's overflow policy
        while(first != last) {
            *first = buffer_.front();
            buffer_.pop();
            ++first;
        }
    }

    static OperationState operationState(TimePoint const& currentTime) {
        auto const op = operationState_;
        if(op == OperationState::ongoing) {
            if(currentTime > timeoutTime_) {
                state_          = State::blocked;
                operationState_ = OperationState::failed;
                return OperationState::failed;
            }
        }
        return op;
    }

    static bool acquire() {
        if(state_ == State::idle) {
            state_ = State::blocked;
            return true;
        }
        return false;
    }

    static void release() {
        assert(state_ != State::idle);
        state_ = State::idle;
    }

    /// Reset pulse, then `c` out, nothing in.
    template<typename C>
    static void send(TimePoint const& currentTime,
                     C const&         c) {
        begin_(currentTime, std::size(c));
        keep_(c);
        buffer_.push(c);
        receiveSize_ = 0;
    }

    /// Reset pulse, then `size` bytes in.
    static void receive(TimePoint const& currentTime,
                        std::uint8_t     size) {
        assert(size <= buffer_.max_size());
        begin_(currentTime, size);
        receiveSize_ = size;
        rxSize_      = size;
    }

    /// Reset pulse, `c` out, then `size` bytes in.
    template<typename C>
    static void sendReceive(TimePoint const& currentTime,
                            C const&         c,
                            std::uint8_t     size) {
        assert(size <= buffer_.max_size());
        begin_(currentTime, std::size(c) + size);
        keep_(c);
        buffer_.push(c);
        receiveSize_ = size;
        rxSize_      = size;
    }

    static void turn() {
        auto const currentTime = Clock::now();
        switch(state_) {
        case State::idle:
        case State::blocked:
            {
            }
            break;
        case State::recovering:
            {
                // a restart: the line high for a slot (tREC) before the new reset pulse
                if(currentTime > waitTime_) {
                    pinClear();
                    waitTime_ = currentTime + resetLow;
                    state_    = State::resetPulse;
                }
            }
            break;
        case State::resetPulse:
            {
                if(currentTime > waitTime_ && SlotGate::open(presenceLimit)) {
                    bool       noPresence{};
                    bool const inTime = presence(noPresence);
                    if(!inTime) {
                        restart_(
                          Clock::
                            now());   // sampled past the presence window: say nothing about the part
                    } else if(noPresence) {
                        state_          = State::blocked;
                        operationState_ = OperationState::failed;
                    } else {
                        // tRSTH (480 us min) counts from the release, which was at currentTime,
                        // not from the presence sample
                        waitTime_ = currentTime + presenceSample + presenceRest;
                        bitCount_ = 0;
                        state_    = State::sending;
                    }
                }
            }
            break;
        case State::sending:
            {
                if(currentTime > waitTime_) {
                    if(auto const* const next = buffer_.peek(); next == nullptr) {
                        bitCount_    = 0;
                        currentByte_ = std::byte{};
                        state_       = State::receiving;
                    } else {
                        auto const b   = *next;
                        auto const bit = ((b >> bitCount_) & std::byte{1}) == std::byte{1};
                        if(!SlotGate::open(bit ? write1Limit : write0Limit)) { break; }
                        if(!writeBit(bit)) {
                            restart_(Clock::now());
                            break;
                        }
                        ++bitCount_;
                        if(bitCount_ == 8) {
                            bitCount_ = 0;
                            buffer_.pop();
                        }
                        waitTime_ = currentTime + slot;
                    }
                }
            }
            break;
        case State::receiving:
            {
                if(currentTime > waitTime_) {
                    if(receiveSize_ == 0) {
                        operationState_ = OperationState::succeeded;
                        state_          = State::blocked;
                    } else {
                        if(!SlotGate::open(readLimit)) { break; }
                        bool       v{};
                        bool const inTime = readBit(v);
                        if(!inTime) {
                            restart_(Clock::now());
                            break;
                        }
                        auto const b = std::byte(v);
                        currentByte_ |= b << bitCount_;
                        ++bitCount_;
                        if(bitCount_ == 8) {
                            bitCount_ = 0;
                            buffer_.push(currentByte_);
                            currentByte_ = std::byte{};
                            --receiveSize_;
                        }
                        waitTime_ = currentTime + slot;
                    }
                }
            }
            break;
        }
    }

    static void pinSet() { apply(makeInput(Pin{})); }

    static void pinClear() { apply(makeOutput(Pin{}), clear(Pin{})); }

    static bool pinRead() { return apply(read(Pin{})); }

    // The timed parts. masked: interrupts masked, each wait a Clock::delay after the step before. checked: nothing
    // masked, every wait a deadline from the part's one start instant (the clock reads and pin writes of the steps
    // before do not add up), the part measured against its limit at its end - false when it took longer: an interrupt
    // stretched it, what it saw or sent may be wrong.

    /// The presence sample after the reset pulse's release.
    static bool presence(bool& noPresence) {
        if constexpr(checked) {
            auto const start = SlotTimer::stamp();
            pinSet();
            until(start, presenceSample);
            noPresence = pinRead();
            return inTime(Part::presence, start, presenceLimit);
        } else {
            noPresence = criticalSection([] {
                pinSet();
                Clock::delay(presenceSample);
                return pinRead();
            });
            return true;
        }
    }

    /// One write slot's low pulse.
    static bool writeBit(bool v) {
        if constexpr(checked) {
            auto const start = SlotTimer::stamp();
            pinClear();
            until(start, v ? write1Low : write0Low);
            pinSet();
            return v ? inTime(Part::write1, start, write1Limit)
                     : inTime(Part::write0, start, write0Limit);
        } else {
            criticalSection([v] {
                pinClear();
                if(v) {
                    Clock::delay(
                      write1Low);   // each a constant: nothing computed inside the 5 us pulse
                } else {
                    Clock::delay(write0Low);
                }
                pinSet();
            });
            return true;
        }
    }

    /// One read slot: the low pulse, then the sample.
    static bool readBit(bool& v) {
        if constexpr(checked) {
            auto const start = SlotTimer::stamp();
            pinClear();
            until(start, readLow);
            pinSet();
            until(start, readAt);
            v = pinRead();
            return inTime(Part::read, start, readLimit);
        } else {
            v = criticalSection([] {
                pinClear();
                Clock::delay(readLow);
                pinSet();
                Clock::delay(readSample);
                return pinRead();
            });
            return true;
        }
    }

    using Stamp = typename SlotTimer::Stamp;

    /// Waits until `at` after `start` (SlotTimer::wait of what is left).
    static void until(Stamp                     start,
                      std::chrono::microseconds at) {
        if(auto const left = at - SlotTimer::since(start); left > decltype(left){}) {
            SlotTimer::wait(left);
        }
    }

    /// checked: did the part that began at `start` end inside `limit`? Notes the one that did not.
    static bool inTime(Part                      part,
                       Stamp                     start,
                       std::chrono::microseconds limit) {
        auto const took = SlotTimer::since(start);
        if(took <= limit) { return true; }
        lastTimingFault_ = {part, std::chrono::duration_cast<std::chrono::microseconds>(took)};
        return false;
    }

    /// The Dallas/Maxim CRC-8 of `data` (Bytes.hpp): 0 over a ROM or scratchpad that
    /// carries its own CRC byte.
    [[nodiscard]] static constexpr std::uint8_t crc(std::span<std::byte const> data) {
        return Dallas::crc8(Bytes{data});
    }

    template<typename IIT>
    [[nodiscard]] static constexpr std::uint8_t crc(IIT first,
                                                    IIT last) {
        Crc::Crc8MaximDow<> c;
        while(first != last) { c.update(std::byte{*first++}); }
        return c.finish();
    }

private:
    using Us = std::chrono::microseconds;
    // Standard-speed slot timings: AN126's where they match it, the others inside the MAX31820
    // limits (the class comment above). checked samples earlier inside the same windows: the measured part has to
    // stay under its limit with the clock reads and room for an interrupt.
    static constexpr Us resetLow{480};
    static constexpr Us presenceSample{
      checked ? 64 : 70};                 // presence low guaranteed 60..75 us after the release
    static constexpr Us resetHigh{480};   // tRSTH: from the release to the first slot
    static constexpr Us presenceRest{resetHigh - presenceSample};
    static constexpr Us slot{64};
    static constexpr Us write1Low{5};
    static constexpr Us write0Low{65};
    static constexpr Us readLow{checked ? 2 : 3};
    static constexpr Us readSample{10};   // masked: after the release
    static constexpr Us readAt{8};        // checked: after the fall - 6 us for the line to rise
    // The limits a checked part is measured against (MAX31820 Table "AC Electrical Characteristics"): the presence
    // sample before tPDHIGH min + tPDLOW min = 75 us, tLOW1 15 us, tLOW0 120 us, a read's sample within tRDV 15 us
    // of its fall. Their minimums need no check: the delays are the lower bounds, and an interrupt only lengthens.
    static constexpr Us presenceLimit{75};
    static constexpr Us write1Limit{15};
    static constexpr Us write0Limit{120};
    static constexpr Us readLimit{15};
    static_assert(!checked
                    || (presenceSample < presenceLimit && write1Low < write1Limit
                        && write0Low < write0Limit && readLow < readAt && readAt < readLimit),
                  "every checked part fits its limit with room for the clock reads");

    /// What a transaction sends, for a restart.
    template<typename C>
    static void keep_(C const& c) {
        txSize_ = 0;
        for(auto const b : c) { tx_[txSize_++] = static_cast<std::byte>(b); }
        rxSize_ = 0;
    }

    /// checked: a timed part came out too long - the transaction again from a reset pulse (a reset ends whatever the
    /// part made of the slot) after a slot's recovery, up to timingRetries times in a row; then it fails.
    static void restart_(TimePoint const& currentTime) {
        ++timingFaults_;
        if(++retries_ > timingRetries) {
            state_          = State::blocked;
            operationState_ = OperationState::failed;
            pinSet();
            return;
        }
        auto const receive = rxSize_;
        buffer_.clear();
        for(std::size_t i = 0; i < txSize_; ++i) { buffer_.push(tx_[i]); }
        bitCount_    = 0;
        currentByte_ = std::byte{};
        receiveSize_ = receive;
        state_       = State::recovering;
        waitTime_    = currentTime + slot;
    }

    /// Every transaction starts from nothing: an empty buffer, no bits of a previous byte,
    /// the reset pulse. Its deadline covers `bytes` bytes of slots.
    static void begin_(TimePoint const& currentTime,
                       std::size_t      bytes) {
        buffer_.clear();
        bitCount_       = 0;
        currentByte_    = std::byte{};
        receiveSize_    = 0;
        txSize_         = 0;
        retries_        = 0;
        state_          = State::resetPulse;
        operationState_ = OperationState::ongoing;
        timeoutTime_
          = currentTime + transactionTimeout + static_cast<std::int64_t>(8 * bytes) * slotBudget;
        pinClear();
        waitTime_ = currentTime + resetLow;
    }

    inline static TimePoint      waitTime_{};
    inline static TimePoint      timeoutTime_{};
    inline static State          state_{State::idle};
    inline static OperationState operationState_{OperationState::succeeded};
    inline static Kvasir::Atomic::Queue<std::byte, bufferSize> buffer_{};
    inline static std::uint8_t                                 receiveSize_{};
    inline static std::uint8_t                                 bitCount_{};
    inline static std::byte                                    currentByte_{};
    inline static std::array<std::byte, bufferSize>            tx_{};
    inline static std::uint8_t                                 txSize_{};
    inline static std::uint8_t                                 rxSize_{};
    inline static std::uint8_t                                 retries_{};
    inline static std::uint32_t                                timingFaults_{};
    inline static TimingFault                                  lastTimingFault_{};
};
}   // namespace Kvasir
