#pragma once
#include "OneWire.hpp"
#include "kvasir/Util/using_literals.hpp"

#include <optional>
namespace Kvasir {
template<typename OneWire, typename Clock>
struct Max31820 {
    using tp  = typename Clock::time_point;
    using OS  = typename OneWire::OperationState;
    using dtT = float;
    enum class State { start_conversion, wait_for_conversion_start, wait_for_conversion, wait_for_data };

    static constexpr auto        sensor_read_time{std::chrono::milliseconds(800)};
    static constexpr std::size_t readout_packet_size{9};
    static constexpr auto        fail_retry_time{std::chrono::milliseconds(200)};

    static_assert(OneWire::BufferSize >= readout_packet_size, "OneWire buffer to small");

    State              st_{};
    tp                 waitTime_{};
    std::optional<dtT> traw_{};

    std::optional<dtT> t() const { return traw_; }

    constexpr Max31820() : st_{State::start_conversion}, waitTime_{tp::min()} {}

    void handler() {
        auto const currentTime = Clock::now();
        switch(st_) {
        case State::start_conversion: {
            if(currentTime > waitTime_ && OneWire::acquire()) {
                st_ = State::wait_for_conversion_start;
                OneWire::send(currentTime, std::array{0xCC_b, 0x44_b});
            }
        } break;
        case State::wait_for_conversion_start: {
            switch(OneWire::operationState(currentTime)) {
            case OS::ongoing: {
            } break;
            case OS::succeeded: {
                st_       = State::wait_for_conversion;
                waitTime_ = currentTime + sensor_read_time;
                OneWire::release();
            } break;
            case OS::failed: {
                st_       = State::start_conversion;
                waitTime_ = currentTime + fail_retry_time;
                traw_.reset();
                OneWire::release();
            } break;
            }
        } break;

        case State::wait_for_conversion: {
            if(currentTime > waitTime_ && OneWire::acquire()) {
                OneWire::send_receive(currentTime, std::array{0xCC_b, 0xBE_b}, readout_packet_size);
                st_ = State::wait_for_data;
            }
        } break;

        case State::wait_for_data: {
            switch(OneWire::operationState(currentTime)) {
            case OS::ongoing: {
            } break;
            case OS::succeeded: {
                st_ = State::start_conversion;
                std::array<std::byte, readout_packet_size> buffer{};
                OneWire::getReceivedBytes(buffer);
                if(OneWire::crc(buffer.begin(), buffer.end()) != 0_b) {
                    waitTime_ = currentTime + fail_retry_time;
                    traw_.reset();
                } else {
                    auto const raw = static_cast<std::int16_t>(
                      static_cast<std::uint16_t>(static_cast<std::uint16_t>(buffer[1]) << 8_u16)
                      | static_cast<std::uint16_t>(buffer[0]));
                    traw_ = raw;
                }
                OneWire::release();
            } break;
            case OS::failed: {
                st_       = State::start_conversion;
                waitTime_ = currentTime + fail_retry_time;
                traw_.reset();
                OneWire::release();
            } break;
            }
        } break;
        }
    }
};
}   // namespace Kvasir
