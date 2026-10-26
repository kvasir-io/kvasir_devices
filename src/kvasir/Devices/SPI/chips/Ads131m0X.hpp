#pragma once
// TI ADS131M0x simultaneous-sampling delta-sigma ADC (ADS131M04.md) on a queued SPI master. Its
// frames are fixed-length command/data words, built here. Each frame's callback runs in the
// master's completion and only ends its OneFrame (Client.hpp), which turn() takes; the master
// calls back exactly once per frame, so no watchdog of its own is needed.
#include "../../Bytes.hpp"
#include "../../Duration.hpp"
#include "../../Log.hpp"
#include "../../Quantities.hpp"
#include "../../SampleSlot.hpp"
#include "../Client.hpp"
#include "../QueueCore.hpp"
#include "kvasir/Hal/Spi.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Link.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>

namespace Kvasir { namespace SPI {

    struct Ads131m0XDefaults {
        static constexpr auto startupDelay = std::chrono::milliseconds{1000};
        /// One reading per SamplePeriod at most, once DRDY is low.
        static constexpr auto samplePeriod = std::chrono::milliseconds{5};
        /// tc(SC) >= 64 ns (ADS131M04.md:474).
        static constexpr Units::Hertz maxClock = Units::hertz(15'000'000);
        /// Failed bring-ups in a row before link() says absent; it goes on regardless.
        static constexpr std::uint8_t absentAfterFailures = 3;
    };

    /// Read in the reset configuration: gain 1, internal 1.2 V reference, the upper 16 bits of each
    /// 24-bit result. SPI mode 1 (ADS131M04.md:1471).
    ///
    /// Bring-up: RESET, then a NULL frame whose response must be FF2nh (8.5.1.10.2). Reading: two
    /// full frames back to back, the second decoded - a read after a gap gets the older sample first
    /// (8.5.1.9.1); frames are always full length (8.5.1.11). The first two readings after a reset
    /// are dropped: fast-settling filter (8.4.2, ADS131M04.md:1331-1334).
    template<typename Master, typename Clock, typename Cs, typename Drdy, typename Config>
    class Ads131m0X {
    public:
        static constexpr std::size_t MaxChannels = 8;
        static constexpr std::size_t NumChannels = Config::channels;
        static_assert(NumChannels >= 1 && NumChannels <= MaxChannels,
                      "an ADS131M0x has 1 to 8 channels");
        using TimePoint = typename Clock::time_point;
        using Codes     = std::array<std::int16_t, NumChannels>;

        static constexpr std::uint16_t ResetAck = static_cast<std::uint16_t>(0xFF20U | NumChannels);
        static constexpr std::uint8_t  SettlingReadings = 2;

        static constexpr std::chrono::milliseconds StartupDelay = Kvasir::asDuration(
          KVASIR_CONFIG_OR(Config,
                           startupDelay,
                           std::chrono::milliseconds{Ads131m0XDefaults::startupDelay}));
        static constexpr std::chrono::milliseconds SamplePeriod = Kvasir::asDuration(
          KVASIR_CONFIG_OR(Config,
                           samplePeriod,
                           std::chrono::milliseconds{Ads131m0XDefaults::samplePeriod}));
        static constexpr std::uint8_t AbsentAfterFailures = static_cast<std::uint8_t>(
          KVASIR_CONFIG_OR(Config, absentAfterFailures, Ads131m0XDefaults::absentAfterFailures));
        static constexpr auto Setup
          = Master::setup(Hal::ClockMode::mode1,
                          KVASIR_CONFIG_OR(Config, maxClock, Ads131m0XDefaults::maxClock));

        Ads131m0X() : next_{Clock::now() + StartupDelay} { ChipSelect<Cs>::release(); }

        Ads131m0X(Ads131m0X const&)            = delete;
        Ads131m0X& operator=(Ads131m0X const&) = delete;

        [[nodiscard]] static constexpr Units::MicroVolt toVoltage(std::int16_t code) {
            return Units::microVolt(std::int64_t{code} * 1'200'000 / 32768);
        }

        [[nodiscard]] std::optional<Units::MicroVolt> voltage(std::size_t channel) const {
            if(!valid_ || channel >= NumChannels) { return std::nullopt; }
            return toVoltage(slot_.latest()[channel]);
        }

        [[nodiscard]] bool valid() const { return valid_; }

        [[nodiscard]] bool answering() const {
            return state_ == State::idle || state_ == State::first || state_ == State::second;
        }

        /// Absent after AbsentAfterFailures bring-ups failed in a row.
        [[nodiscard]] Link link() const { return bringUpFailures_.link(answering()); }

        [[nodiscard]] Codes const& latest() const { return slot_.latest(); }

        [[nodiscard]] std::uint32_t seq() const { return slot_.seq(); }

        [[nodiscard]] std::uint32_t samples() const { return slot_.samples(); }

        [[nodiscard]] std::uint32_t discarded() const { return discarded_; }

        [[nodiscard]] std::uint32_t errors() const { return errors_; }

        [[nodiscard]] bool inFlight() const { return xfer_.running(); }

        void turn() {
            auto const now  = Clock::now();
            auto const poll = xfer_.take();
            if(poll == Frame::Poll::running) { return; }
            if(poll == Frame::Poll::failed) {
                // the master gave up on the frame (timeout, overrun), or refused it
                ++errors_;
                bool const reading = state_ == State::first || state_ == State::second;
                if(!reading) { bringUpFailed_(); }
                state_ = reading ? State::idle : State::startup;
                next_  = now + (reading ? SamplePeriod : StartupDelay);
                return;
            }
            bool const done = poll == Frame::Poll::ok;
            switch(state_) {
            case State::startup:
                if(now > next_) {
                    valid_ = false;
                    start_(CmdReset);
                    state_ = State::reset;
                }
                break;
            case State::reset:
                if(done) {
                    next_  = now + std::chrono::milliseconds{1};   // tREGACQ is 5 us
                    state_ = State::resetGap;
                }
                break;
            case State::resetGap:
                if(now >= next_) {
                    start_(CmdNull);
                    state_ = State::ack;
                }
                break;
            case State::ack:
                if(done) {
                    auto const response = Bytes{frame_}.be16(0);
                    if(response == ResetAck) {
                        next_     = now;
                        settling_ = SettlingReadings;
                        bringUpFailures_.cleared();
                        state_ = State::idle;
                        UC_LOG_D("ads131m0x: up, {} channels", NumChannels);
                    } else {
                        ++errors_;
                        bringUpFailed_();
                        UC_LOG_W("ads131m0x: reset answered {:#06x}, expected {:#06x}",
                                 response,
                                 ResetAck);
                        next_  = now + StartupDelay;
                        state_ = State::startup;
                    }
                }
                break;
            case State::idle:
                if(!(now > next_) || apply(read(Drdy{}))) { return; }
                next_ = now + SamplePeriod;
                start_(CmdNull);   // the older sample in the FIFO
                state_ = State::first;
                break;
            case State::first:
                if(done) {
                    start_(CmdNull);   // straight after it: the newest
                    state_ = State::second;
                }
                break;
            case State::second:
                if(done) {
                    if(settling_ != 0) {
                        --settling_;   // the fast-settling filter's: read off the part, not kept
                        ++discarded_;
                    } else {
                        decode_();
                    }
                    state_ = State::idle;
                }
                break;
            }
        }

    private:
        static constexpr std::uint16_t CmdNull  = 0x0000;
        static constexpr std::uint16_t CmdReset = 0x0011;
        enum class State : std::uint8_t { startup, reset, resetGap, ack, idle, first, second };

        using Frame = OneFrame<Master>;

        /// Input CRC is off in the reset configuration. A refused frame counts as a failed one.
        void start_(std::uint16_t command) {
            std::fill(frame_.begin(), frame_.end(), std::byte{0});
            frame_[0] = static_cast<std::byte>(command >> 8U);
            frame_[1] = static_cast<std::byte>(command & 0xFFU);
            if(!xfer_.start(typename Master::Request{.setup = Setup,
                                                     .lines = ChipSelect<Cs>::lines,
                                                     .tx    = std::span<std::byte const>{frame_},
                                                     .rx    = std::span{frame_}}))
            {
                xfer_.refuse();
            }
        }

        void decode_() {
            Codes codes{};
            for(std::size_t channel = 0; channel < NumChannels; ++channel) {
                codes[channel] = Bytes{frame_}.s16be(3 + 3 * channel);
            }
            slot_.publish(codes);
            valid_ = true;
        }

        void bringUpFailed_() { bringUpFailures_.failed(); }

        TimePoint                                      next_{};
        std::array<std::byte, 3 + 3 * NumChannels + 3> frame_{};
        Frame                                          xfer_{};
        State                                          state_{State::startup};
        bool                                           valid_{};
        std::uint8_t                                   settling_{};
        FailureStreak<AbsentAfterFailures>             bringUpFailures_{};
        SampleSlot<Codes>                              slot_{};
        std::uint32_t                                  discarded_{};
        std::uint32_t                                  errors_{};
    };

}}   // namespace Kvasir::SPI
