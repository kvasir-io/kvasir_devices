#pragma once

#include "Duration.hpp"
#include "Log.hpp"
#include "Quantities.hpp"
#include "kvasir/Util/RateLimiter.hpp"

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

namespace Kvasir {

/// The knobs of the ADS131M0x driver, beside `numChannels`; a Config may leave them out.
struct Ads131m0XDefaults {
    /// Before the first frame after power-up: the part's own start-up and the supplies.
    static constexpr auto StartupDelay = std::chrono::milliseconds{1000};

    /// One frame per SamplePeriod at most, once DRDY is low.
    static constexpr auto SamplePeriod = std::chrono::milliseconds{5};

    /// A frame whose DMA completion never comes is aborted after this long, chip select
    /// raised, and counted in errors().
    static constexpr auto InFlightTimeout = std::chrono::milliseconds{200};
};

/// TI ADS131M0x simultaneous-sampling delta-sigma ADC, read in its reset configuration: gain
/// 1 on every channel against the internal 1.2 V reference. Each channel's 24-bit result is
/// kept to its upper 16 bits, so one count is 1.2 V / 2^15.
///
/// Bring-up, after StartupDelay: the RESET command (0011h) in a frame of its own, then a NULL
/// frame whose response word must be the reset acknowledge FF2nh, n the channel count (8.5.1.10.2;
/// Linux ti-ads131m02.c checks the same): that is the presence check, and a
/// part that answers anything else -- or nothing -- is counted in errors() and tried again
/// StartupDelay later. valid() stays false until then.
///
/// Reading: whenever DRDY is low and SamplePeriod has passed, two full frames (the response
/// word, one word per channel, the CRC word) are clocked out back to back and the second is
/// decoded. The part keeps two samples per channel in a FIFO and every read at SamplePeriod
/// is a read "after a gap in data collection" (8.5.1.9.1): the first frame is the older sample, and
/// with the FIFO full DRDY would only toggle at half the data rate. A frame is always the
/// full length: a short frame is only allowed while the ADC channels are disabled (8.5.1.11).
///
/// Config supplies `numChannels`, and may override the Ads131m0XDefaults members.
template<typename Clock, typename SPI, typename Cs, typename Drdy, typename Config>
struct Ads131m0X {
    static constexpr auto MaxChannels = 8;

    static_assert(MaxChannels >= Config::numChannels && Config::numChannels != 0,
                  "wrong channels");

    using TimePoint = typename Clock::time_point;
    using Codes     = std::array<std::int16_t, Config::numChannels>;

    static constexpr std::size_t NumChannels = Config::numChannels;

    /// The response word to the NULL frame after a RESET: FF2nh.
    static constexpr std::uint16_t ResetAck = static_cast<std::uint16_t>(0xFF20U | NumChannels);

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

    static constexpr std::chrono::milliseconds InFlightTimeout = [] {
        if constexpr(requires { Config::InFlightTimeout; }) {
            return Kvasir::asDuration(Config::InFlightTimeout);
        } else {
            return std::chrono::milliseconds{Ads131m0XDefaults::InFlightTimeout};
        }
    }();

    Ads131m0X() : next_{Clock::now() + StartupDelay} {}

    [[nodiscard]] static constexpr Units::MicroVolt toVoltage(std::int16_t code) {
        return Units::microVolt(std::int64_t{code} * 1'200'000 / 32768);
    }

    /// The last reading of `channel`; empty before the first frame or for a channel the
    /// part does not have.
    [[nodiscard]] std::optional<Units::MicroVolt> voltage(std::size_t channel) const {
        if(!valid_ || channel >= NumChannels) { return std::nullopt; }
        return toVoltage(codes_[channel]);
    }

    /// A frame has been decoded.
    [[nodiscard]] bool valid() const { return valid_; }

    /// The reset acknowledge came back: the part is there.
    [[nodiscard]] bool present() const {
        return state_ == State::idle || state_ == State::first || state_ == State::second;
    }

    /// The codes of the last frame, every channel.
    [[nodiscard]] Codes const& latest() const { return codes_; }

    /// Steps with every frame.
    [[nodiscard]] std::uint32_t seq() const { return samples_; }

    [[nodiscard]] std::uint32_t samples() const { return samples_; }

    /// Frames the watchdog aborted, and bring-ups whose reset was not acknowledged, over the
    /// device's life.
    [[nodiscard]] std::uint32_t errors() const { return errors_; }

    [[nodiscard]] bool inFlight() const { return running_.load(std::memory_order_acquire); }

    void handler() {
        auto const now = Clock::now();
        if(running_.load(std::memory_order_acquire)) {
            if(now - since_ > InFlightTimeout) {
                SPI::abortTransfer();
                apply(set(Cs{}));
                // A completion that still comes belongs to the aborted frame: ignored.
                generation_.fetch_add(1, std::memory_order_release);
                running_.store(false, std::memory_order_release);
                ++errors_;
                KVASIR_LOG_LIMITED(log_.allow(TimeoutKey, now),
                                   UC_LOG_W,
                                   "ads131m0x: frame timed out after {}",
                                   InFlightTimeout);
                next_ = now + SamplePeriod;
                // Mid-reading, the next DRDY starts over; mid-bring-up, the bring-up does.
                bool const reading = state_ == State::first || state_ == State::second;
                state_             = reading ? State::idle : State::startup;
            }
            return;
        }
        // Not running: the completion, if one ran, has left its frame in frame_.
        bool const done = pending_.exchange(false, std::memory_order_acquire);
        switch(state_) {
        case State::startup:
            if(now > next_) {
                valid_ = false;
                start_(now, CmdReset);
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
                start_(now, CmdNull);
                state_ = State::ack;
            }
            break;
        case State::ack:
            if(done) {
                auto const response
                  = static_cast<std::uint16_t>((std::to_integer<unsigned>(frame_[0]) << 8U)
                                               | std::to_integer<unsigned>(frame_[1]));
                if(response == ResetAck) {
                    next_  = now;
                    state_ = State::idle;
                } else {
                    ++errors_;
                    KVASIR_LOG_LIMITED(log_.allow(AckKey, now),
                                       UC_LOG_W,
                                       "ads131m0x: reset answered {:#06x}, expected {:#06x}",
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
            start_(now, CmdNull);   // the older sample in the FIFO
            state_ = State::first;
            break;
        case State::first:
            if(done) {
                start_(now, CmdNull);   // straight after it: the newest
                state_ = State::second;
            }
            break;
        case State::second:
            if(done) {
                decode_();
                state_ = State::idle;
            }
            break;
        }
    }

private:
    static constexpr std::uint32_t TimeoutKey = rateLimitKey(1);
    static constexpr std::uint32_t AckKey     = rateLimitKey(2);

    static constexpr std::uint16_t CmdNull  = 0x0000;
    static constexpr std::uint16_t CmdReset = 0x0011;

    enum class State : std::uint8_t { startup, reset, resetGap, ack, idle, first, second };

    /// One full frame with `command` in its first word; the rest zeros (input CRC is off in the
    /// reset configuration, so its word is not checked).
    void start_(TimePoint     now,
                std::uint16_t command) {
        std::fill(frame_.begin(), frame_.end(), std::byte{0});
        frame_[0]      = static_cast<std::byte>(command >> 8U);
        frame_[1]      = static_cast<std::byte>(command & 0xFFU);
        since_         = now;
        auto const gen = generation_.load(std::memory_order_acquire);
        apply(clear(Cs{}));
        running_.store(true, std::memory_order_release);
        SPI::send_receive_nocopy(std::span{frame_}, std::span{frame_}, [this, gen]() {
            apply(set(Cs{}));
            if(gen == generation_.load(std::memory_order_acquire)) {
                pending_.store(true, std::memory_order_release);
            }
            running_.store(false, std::memory_order_release);
        });
    }

    /// The channel words' upper 16 bits, big endian, after the response word.
    void decode_() {
        for(std::size_t channel = 0; channel < NumChannels; ++channel) {
            std::uint16_t word;
            std::memcpy(&word, frame_.data() + 3 + 3 * channel, 2);
            codes_[channel] = static_cast<std::int16_t>(std::byteswap(word));
        }
        valid_ = true;
        ++samples_;
    }

    TimePoint next_{};
    TimePoint since_{};
    /// The response word, one word per channel, the CRC word: 24-bit words.
    std::array<std::byte, 3 + 3 * NumChannels + 3> frame_{};
    std::atomic<bool>                              running_{false};
    std::atomic<bool>                              pending_{false};
    std::atomic<std::uint32_t>                     generation_{0};
    State                                          state_{State::startup};
    bool                                           valid_{false};
    Codes                                          codes_{};
    std::uint32_t                                  samples_{};
    std::uint32_t                                  errors_{};
    RateLimiter<Clock>                             log_{};
};
}   // namespace Kvasir
