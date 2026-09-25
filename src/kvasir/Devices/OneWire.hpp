#pragma once
#include "Bytes.hpp"
#include "Duration.hpp"
#include "kvasir/Atomic/Queue.hpp"

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

namespace Kvasir {

/// The knobs of the 1-Wire bus, with their defaults; derive and redeclare what you change.
struct OneWireDefaults {
    /// Deadline for the transaction in flight, beyond the time its bits take: each bit slot
    /// is timed from handler(), so on a millisecond tick a slot can take up to two of them,
    /// and a transaction is given `SlotBudget` per bit on top of this. It only bounds a bus
    /// that has stopped answering.
    static constexpr auto TransactionTimeout = std::chrono::milliseconds{100};
    static constexpr auto SlotBudget         = std::chrono::milliseconds{2};
};

/// A 1-Wire master bit-banged on `Pin` (open drain: driven low, released to an input), as a
/// state machine over Clock: one transaction at a time, a reset pulse then the bytes out
/// then the bytes in, each bit slot timed against Clock::now() from handler() and the slot's
/// own microseconds spun in Clock::delay. The bit timings are the standard-speed ones of
/// the Maxim application note 126 (reset 480 us, presence sample at 70 us, write-1 low
/// 5 us, write-0 low 65 us, read sample at 13 us, slot 64 us).
///
///     acquire() -> send(now, bytes) | receive(now, n) | sendReceive(now, bytes, n)
///     operationState(now) until it is not ongoing -> getReceivedBytes(...) -> release()
template<typename Clock, typename Pin, std::size_t Size, typename Config = OneWireDefaults>
struct OneWire {
    static constexpr std::size_t BufferSize = Size;
    using TimePoint                         = typename Clock::time_point;
    enum class State : std::uint8_t { idle, blocked, resetPulse, sending, receiving };
    enum class OperationState : std::uint8_t { succeeded, failed, ongoing };

    static constexpr std::chrono::milliseconds TransactionTimeout = [] {
        if constexpr(requires { Config::TransactionTimeout; }) {
            return Kvasir::asDuration(Config::TransactionTimeout);
        } else {
            return std::chrono::milliseconds{OneWireDefaults::TransactionTimeout};
        }
    }();

    static constexpr std::chrono::milliseconds SlotBudget = [] {
        if constexpr(requires { Config::SlotBudget; }) {
            return Kvasir::asDuration(Config::SlotBudget);
        } else {
            return std::chrono::milliseconds{OneWireDefaults::SlotBudget};
        }
    }();

    template<typename C>
    static void getReceivedBytes(C& c) {
        assert(c.size() <= buffer_.size());
        buffer_.pop_into(c);
    }

    template<typename OIT>
    static void getReceivedBytes(OIT first,
                                 OIT last) {
        while(first != last) {
            assert(!buffer_.empty());
            *first = buffer_.front();
            buffer_.pop();
            ++first;
        }
    }

    static OperationState operationState(TimePoint const& currentTime) {
        auto op = operationState_;
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
        buffer_.push(c);
        receiveSize_ = 0;
    }

    /// Reset pulse, then `size` bytes in.
    static void receive(TimePoint const& currentTime,
                        std::uint8_t     size) {
        assert(size <= buffer_.max_size());
        begin_(currentTime, size);
        receiveSize_ = size;
    }

    /// Reset pulse, `c` out, then `size` bytes in.
    template<typename C>
    static void sendReceive(TimePoint const& currentTime,
                            C const&         c,
                            std::uint8_t     size) {
        assert(size <= buffer_.max_size());
        begin_(currentTime, std::size(c) + size);
        buffer_.push(c);
        receiveSize_ = size;
    }

    static void handler() {
        auto const currentTime = Clock::now();
        switch(state_) {
        case State::idle:
        case State::blocked:
            {
            }
            break;
        case State::resetPulse:
            {
                if(currentTime > waitTime_) {
                    pinSet();
                    Clock::delay(PresenceSample);
                    if(pinRead()) {
                        state_          = State::blocked;
                        operationState_ = OperationState::failed;
                    } else {
                        // tRSTH (480 us min) counts from the release, which was at currentTime,
                        // not from the presence sample
                        waitTime_ = currentTime + PresenceSample + PresenceRest;
                        bitCount_ = 0;
                        state_    = State::sending;
                    }
                }
            }
            break;
        case State::sending:
            {
                if(currentTime > waitTime_) {
                    if(buffer_.empty()) {
                        bitCount_    = 0;
                        currentByte_ = std::byte{};
                        state_       = State::receiving;
                    } else {
                        auto b   = buffer_.front();
                        auto bit = ((b >> bitCount_) & std::byte{1}) == std::byte{1};
                        writeBit(bit);
                        ++bitCount_;
                        if(bitCount_ == 8) {
                            bitCount_ = 0;
                            buffer_.pop();
                        }
                        waitTime_ = currentTime + Slot;
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
                        auto b = std::byte(readBit());
                        currentByte_ |= b << bitCount_;
                        ++bitCount_;
                        if(bitCount_ == 8) {
                            bitCount_ = 0;
                            buffer_.push(currentByte_);
                            currentByte_ = std::byte{};
                            --receiveSize_;
                        }
                        waitTime_ = currentTime + Slot;
                    }
                }
            }
            break;
        }
    }

    static void pinSet() { apply(makeInput(Pin{})); }

    static void pinClear() { apply(makeOutput(Pin{}), clear(Pin{})); }

    static bool pinRead() { return apply(read(Pin{})); }

    static void writeBit(bool v) {
        if(v) {
            pinClear();
            Clock::delay(Write1Low);
            pinSet();
        } else {
            pinClear();
            Clock::delay(Write0Low);
            pinSet();
        }
    }

    static bool readBit() {
        pinClear();
        Clock::delay(ReadLow);
        pinSet();
        Clock::delay(ReadSample);
        return pinRead();
    }

    /// The Dallas/Maxim CRC-8 of `data` (Bytes.hpp): 0 over a ROM or scratchpad that
    /// carries its own CRC byte.
    [[nodiscard]] static constexpr std::uint8_t crc(std::span<std::byte const> data) {
        return Dallas::crc8(Bytes{data});
    }

    template<typename IIT>
    [[nodiscard]] static constexpr std::uint8_t crc(IIT first,
                                                    IIT last) {
        std::uint8_t c = 0;
        while(first != last) {
            std::byte const b = *first++;
            c                 = Dallas::crc8(
              Bytes{
                std::span<std::byte const>{&b, 1}
            },
              c);
        }
        return c;
    }

private:
    // Standard-speed slot timings (AN126).
    static constexpr std::chrono::microseconds ResetLow{480};
    static constexpr std::chrono::microseconds PresenceSample{70};
    static constexpr std::chrono::microseconds PresenceRest{410};
    static constexpr std::chrono::microseconds Slot{64};
    static constexpr std::chrono::microseconds Write1Low{5};
    static constexpr std::chrono::microseconds Write0Low{65};
    static constexpr std::chrono::microseconds ReadLow{3};
    static constexpr std::chrono::microseconds ReadSample{10};

    /// Every transaction starts from nothing: an empty buffer, no bits of a previous byte,
    /// the reset pulse. Its deadline covers `bytes` bytes of slots.
    static void begin_(TimePoint const& currentTime,
                       std::size_t      bytes) {
        buffer_.clear();
        bitCount_       = 0;
        currentByte_    = std::byte{};
        receiveSize_    = 0;
        state_          = State::resetPulse;
        operationState_ = OperationState::ongoing;
        timeoutTime_
          = currentTime + TransactionTimeout + static_cast<std::int64_t>(8 * bytes) * SlotBudget;
        pinClear();
        waitTime_ = currentTime + ResetLow;
    }

    inline static TimePoint      waitTime_{};
    inline static TimePoint      timeoutTime_{};
    inline static State          state_{State::idle};
    inline static OperationState operationState_{OperationState::succeeded};
    inline static Kvasir::Atomic::Queue<std::byte, BufferSize> buffer_{};
    inline static std::uint8_t                                 receiveSize_{};
    inline static std::uint8_t                                 bitCount_{};
    inline static std::byte                                    currentByte_{};
};
}   // namespace Kvasir
