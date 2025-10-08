#pragma once
#include "kvasir/Atomic/Queue.hpp"

#include <cassert>
#include <cstdint>

namespace Kvasir {
template<typename Clock, typename Pin, std::size_t BufferSize_>
struct OneWire {
    static constexpr std::size_t BufferSize = BufferSize_;
    using tp                                = typename Clock::time_point;
    enum class State { idle, blocked, resetpulse, sending, receiveing };
    enum class OperationState { succeeded, failed, ongoing };
    inline static tp             waitTime_{};
    inline static tp             timeoutTime_{};
    inline static State          state_{State::idle};
    inline static OperationState operationState_{OperationState::succeeded};
    inline static Kvasir::Atomic::Queue<std::byte, BufferSize> buffer_{};
    inline static std::uint8_t                                 receiveSize_{};
    inline static std::uint8_t                                 bitCount_{};
    inline static std::byte                                    currentByte_{};

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

    static OperationState operationState(tp const& currentTime) {
        auto op = operationState_;
        if(op == OperationState::ongoing) {
            if(currentTime > timeoutTime_) {
                state_ = State::blocked;
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
        assert(state_ != State::idle);   // TODO
        state_ = State::idle;
    }

    template<typename C>
    static void send(tp const& currentTime,
                     C const&  c) {
        //assert(state_ == Kvasir::none_of(State::sending, State::receiveing, State::resetpulse));
        buffer_.clear();
        buffer_.push(c);
        state_          = State::resetpulse;
        operationState_ = OperationState::ongoing;
        timeoutTime_    = currentTime + 100ms;   // TODO baud*size
        startResetPulse(currentTime);
    }

    static void receive(tp const&    currentTime,
                        std::uint8_t size) {
        //assert(state_ == Kvasir::none_of(State::sending, State::receiveing, State::resetpulse));
        assert(size <= buffer_.max_size());
        buffer_.clear();
        receiveSize_    = size;
        state_          = State::resetpulse;
        operationState_ = OperationState::ongoing;
        timeoutTime_    = currentTime + 100ms;   // TODO baud*size
        startResetPulse(currentTime);
    }

    template<typename C>
    static void send_receive(tp const&    currentTime,
                             C const&     c,
                             std::uint8_t size) {
        //assert(state_ == Kvasir::none_of(State::sending, State::receiveing, State::resetpulse));
        assert(size <= buffer_.max_size());
        buffer_.clear();
        buffer_.push(c);
        receiveSize_    = size;
        state_          = State::resetpulse;
        operationState_ = OperationState::ongoing;
        timeoutTime_    = currentTime + 100ms;   // TODO baud*size
        startResetPulse(currentTime);
    }

    static void handler() {
        auto const currentTime = Clock::now();
        switch(state_) {
        case State::idle:
        case State::blocked:
            {
            }
            break;
        case State::resetpulse:
            {
                if(currentTime > waitTime_) {
                    pinSet();
                    Clock::template delay<std::chrono::microseconds, 70>();
                    if(pinRead()) {
                        state_          = State::blocked;
                        operationState_ = OperationState::failed;
                    } else {
                        waitTime_ = currentTime + 410us;
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
                        state_ = State::receiveing;
                    } else {
                        auto b   = buffer_.front();
                        auto bit = ((b >> bitCount_) & 1_b) == 1_b;
                        write_bit(bit);
                        ++bitCount_;
                        if(bitCount_ == 8) {
                            bitCount_ = 0;
                            buffer_.pop();
                        }
                        waitTime_ = currentTime + 64us;
                    }
                }
            }
            break;
        case State::receiveing:
            {
                if(currentTime > waitTime_) {
                    if(receiveSize_ == 0) {
                        operationState_ = OperationState::succeeded;
                        state_          = State::blocked;
                    } else {
                        auto b = std::byte(read_bit());
                        currentByte_ |= b << bitCount_;
                        ++bitCount_;
                        if(bitCount_ == 8) {
                            bitCount_ = 0;
                            buffer_.push(currentByte_);
                            currentByte_ = 0_b;
                            --receiveSize_;
                        }
                        waitTime_ = currentTime + 64us;
                    }
                }
            }
            break;
        }
    }

    static void startResetPulse(tp const& currentTime) {
        pinClear();
        waitTime_ = currentTime + 480us;
    }

    static void pinSet() { apply(makeInput(Pin{})); }

    static void pinClear() { apply(makeOutput(Pin{}), clear(Pin{})); }

    static bool pinRead() { return apply(read(Pin{})); }

    static void write_bit(bool v) {
        if(v) {
            pinClear();
            Clock::template delay<std::chrono::microseconds, 5>();
            pinSet();
        } else {
            pinClear();
            Clock::template delay<std::chrono::microseconds, 65>();
            pinSet();
        }
    }

    static bool read_bit() {
        pinClear();
        Clock::template delay<std::chrono::microseconds, 3>();
        pinSet();
        Clock::template delay<std::chrono::microseconds, 10>();
        return pinRead();
    }

    template<typename IIT>
    static std::byte crc(IIT first,
                         IIT last) {
        static constexpr std::array crcTable{0x00_b,
                                             0x9D_b,
                                             0x23_b,
                                             0xBE_b,
                                             0x46_b,
                                             0xDB_b,
                                             0x65_b,
                                             0xF8_b,
                                             0x8C_b,
                                             0x11_b,
                                             0xAF_b,
                                             0x32_b,
                                             0xCA_b,
                                             0x57_b,
                                             0xE9_b,
                                             0x74_b};
        std::byte                   crc{};
        while(first != last) {
            std::byte tmp = *first++;
            auto      i   = std::to_integer<std::size_t>(crc & 0x0F_b);
            crc           = (crc >> 4) | (tmp << 4);
            crc ^= crcTable[i];

            i   = std::to_integer<std::size_t>(crc & 0x0F_b);
            crc = (crc >> 4) | (tmp & 0xF0_b);
            crc ^= crcTable[i];
        }
        return crc;
    }
};
}   // namespace Kvasir
