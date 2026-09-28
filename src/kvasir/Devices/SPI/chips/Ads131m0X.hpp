#pragma once
// TI ADS131M0x simultaneous-sampling delta-sigma ADC (ADS131M04.md) on a queued SPI master. Its
// frames are fixed-length command/data words, built here. Each frame's callback runs in the
// master's completion and only sets the atomics handler() reads; the master calls back exactly
// once per frame, so no watchdog of its own is needed.
#include "../../Duration.hpp"
#include "../../Link.hpp"
#include "../../Log.hpp"
#include "../../Quantities.hpp"
#include "../QueueCore.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <utility>

namespace Kvasir { namespace SPI {

    struct Ads131m0XDefaults {
        static constexpr auto StartupDelay = std::chrono::milliseconds{1000};
        /// One reading per SamplePeriod at most, once DRDY is low.
        static constexpr auto SamplePeriod = std::chrono::milliseconds{5};
        /// tc(SC) >= 64 ns (ADS131M04.md:474).
        static constexpr Units::Hertz MaxClock = Units::hertz(15'000'000);
        /// Failed bring-ups in a row before link() says absent; it goes on regardless.
        static constexpr std::uint8_t AbsentAfterFailures = 3;
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
        static constexpr std::size_t NumChannels = Config::Channels;
        static_assert(NumChannels >= 1 && NumChannels <= MaxChannels,
                      "an ADS131M0x has 1 to 8 channels");
        using TimePoint = typename Clock::time_point;
        using Codes     = std::array<std::int16_t, NumChannels>;

        static constexpr std::uint16_t ResetAck = static_cast<std::uint16_t>(0xFF20U | NumChannels);
        static constexpr std::uint8_t  SettlingReadings = 2;

        static constexpr std::chrono::milliseconds StartupDelay = [] {
            if constexpr(requires { Config::StartupDelay; }) {
                return Kvasir::asDuration(Config::StartupDelay);
            } else {
                return std::chrono::milliseconds{Ads131m0XDefaults::StartupDelay};
            }
        }();
        static constexpr std::chrono::milliseconds SamplePeriod = [] {
            if constexpr(requires { Config::SamplePeriod; }) {
                return Kvasir::asDuration(Config::SamplePeriod);
            } else {
                return std::chrono::milliseconds{Ads131m0XDefaults::SamplePeriod};
            }
        }();
        static constexpr std::uint8_t AbsentAfterFailures = [] {
            if constexpr(requires { Config::AbsentAfterFailures; }) {
                return static_cast<std::uint8_t>(Config::AbsentAfterFailures);
            } else {
                return Ads131m0XDefaults::AbsentAfterFailures;
            }
        }();
        static constexpr auto Setup = Master::setup(ClockMode::_1, [] {
            if constexpr(requires { Config::MaxClock; }) {
                return Config::MaxClock;
            } else {
                return Ads131m0XDefaults::MaxClock;
            }
        }());

        Ads131m0X() : next_{Clock::now() + StartupDelay} { apply(set(Cs{})); }

        Ads131m0X(Ads131m0X const&)            = delete;
        Ads131m0X& operator=(Ads131m0X const&) = delete;

        [[nodiscard]] static constexpr Units::MicroVolt toVoltage(std::int16_t code) {
            return Units::microVolt(std::int64_t{code} * 1'200'000 / 32768);
        }

        [[nodiscard]] std::optional<Units::MicroVolt> voltage(std::size_t channel) const {
            if(!valid_ || channel >= NumChannels) { return std::nullopt; }
            return toVoltage(codes_[channel]);
        }

        [[nodiscard]] bool valid() const { return valid_; }

        [[nodiscard]] bool present() const {
            return state_ == State::idle || state_ == State::first || state_ == State::second;
        }

        /// Absent after AbsentAfterFailures bring-ups failed in a row.
        [[nodiscard]] Link link() const {
            if(present()) { return Link::answering; }
            return bringUpFailures_ >= AbsentAfterFailures ? Link::absent : Link::starting;
        }

        [[nodiscard]] Codes const& latest() const { return codes_; }

        [[nodiscard]] std::uint32_t seq() const { return samples_; }

        [[nodiscard]] std::uint32_t samples() const { return samples_; }

        [[nodiscard]] std::uint32_t discarded() const { return discarded_; }

        [[nodiscard]] std::uint32_t errors() const { return errors_; }

        [[nodiscard]] bool inFlight() const { return running_.load(std::memory_order_acquire); }

        void handler() {
            auto const now = Clock::now();
            if(running_.load(std::memory_order_acquire)) { return; }
            if(failed_.load(std::memory_order_acquire)) {
                // the master gave up on the frame (timeout, overrun), or refused it
                failed_.store(false, std::memory_order_relaxed);
                ++errors_;
                bool const reading = state_ == State::first || state_ == State::second;
                if(!reading) { bringUpFailed_(); }
                state_ = reading ? State::idle : State::startup;
                next_  = now + (reading ? SamplePeriod : StartupDelay);
                return;
            }
            bool const done = done_.exchange(false, std::memory_order_acq_rel);
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
                    auto const response
                      = static_cast<std::uint16_t>((std::to_integer<unsigned>(frame_[0]) << 8U)
                                                   | std::to_integer<unsigned>(frame_[1]));
                    if(response == ResetAck) {
                        next_            = now;
                        settling_        = SettlingReadings;
                        bringUpFailures_ = 0;
                        state_           = State::idle;
                        UC_LOG_I("ads131m0x: up, {} channels", NumChannels);
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

        static void select_() { apply(clear(Cs{})); }

        static void deselect_() { apply(set(Cs{})); }

        static constexpr Lines lines_{&select_, &deselect_, nullptr, nullptr};

        /// Input CRC is off in the reset configuration. The callback lets go of `running_` last
        /// (release), which handler() takes first (acquire).
        void start_(std::uint16_t command) {
            std::fill(frame_.begin(), frame_.end(), std::byte{0});
            frame_[0] = static_cast<std::byte>(command >> 8U);
            frame_[1] = static_cast<std::byte>(command & 0xFFU);
            running_.store(true, std::memory_order_release);
            if(!Master::submit(
                 typename Master::Request{.setup = Setup,
                                          .lines = lines_,
                                          .tx    = std::span<std::byte const>{frame_},
                                          .rx    = std::span{frame_},
                                          .callback =
                                            [this](TransferResult r) {
                                                if(r == TransferResult::succeeded) {
                                                    done_.store(true, std::memory_order_relaxed);
                                                } else {
                                                    failed_.store(true, std::memory_order_relaxed);
                                                }
                                                running_.store(false, std::memory_order_release);
                                            }}))
            {
                failed_.store(true, std::memory_order_relaxed);
                running_.store(false, std::memory_order_release);
            }
        }

        void decode_() {
            for(std::size_t channel = 0; channel < NumChannels; ++channel) {
                std::uint16_t word;
                std::memcpy(&word, frame_.data() + 3 + 3 * channel, 2);
                codes_[channel] = static_cast<std::int16_t>(std::byteswap(word));
            }
            valid_ = true;
            ++samples_;
        }

        void bringUpFailed_() {
            if(bringUpFailures_ != 0xFF) { ++bringUpFailures_; }
        }

        TimePoint                                      next_{};
        std::array<std::byte, 3 + 3 * NumChannels + 3> frame_{};
        std::atomic<bool>                              running_{false};
        std::atomic<bool>                              done_{false};
        std::atomic<bool>                              failed_{false};
        State                                          state_{State::startup};
        bool                                           valid_{};
        std::uint8_t                                   settling_{};
        std::uint8_t                                   bringUpFailures_{};
        Codes                                          codes_{};
        std::uint32_t                                  samples_{};
        std::uint32_t                                  discarded_{};
        std::uint32_t                                  errors_{};
    };

}}   // namespace Kvasir::SPI
