#pragma once
// TI ADS8675 14-bit SAR ADC (ADS8675.md) on a queued SPI master. Not an engine description: it
// samples from interrupts, and its CS is also CONVST - the sample timer raises it to start a
// conversion, the RVS edge lowers it to clock the result out.
//
// The part must be ALONE on its bus: its CS stays low between conversions, and it takes the last
// 32 bits before a CS rising edge as a command word (ADS8675.md:1494-1502).
//
// Contexts: sampleCallback() from the sample timer's interrupt, pinInterrupt() (which submits)
// from the RVS pin's, the frame's callback from the master's completion, turn() from the loop.
#include "../../Bytes.hpp"
#include "../../Duration.hpp"
#include "../../Log.hpp"
#include "../../Quantities.hpp"
#include "../Client.hpp"
#include "../QueueCore.hpp"
#include "kvasir/Hal/Spi.hpp"
#include "kvasir/Log/LogLimited.hpp"
#include "kvasir/Log/RateLimiter.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Link.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <utility>

namespace Kvasir { namespace SPI {

    struct Ads8675Defaults {
        /// tD_RST_POR 20 ms typ and tPWRUP 20 ms.
        static constexpr auto resetLow     = std::chrono::milliseconds{500};
        static constexpr auto resetRecover = std::chrono::milliseconds{100};
        /// 0x04: +-0.625 x VREF (internal 4.096 V) = +-2.56 V. toVoltage() assumes this range.
        static constexpr std::uint8_t rangeSel = 0x04;
        static constexpr Units::Hertz maxClock = Units::hertz(66'670'000);
        /// Failed bring-ups in a row before link() says absent; it goes on regardless.
        static constexpr std::uint8_t absentAfterFailures = 3;
    };

    /// Bring-up writes RANGE_SEL and reads it back (the word comes in the next frame); a part that
    /// does not hold it would scale every sample wrong, so it is brought up again. SPI-00-S (mode 0)
    /// after a reset (ADS8675.md:1641).
    ///
    ///     Kvasir::SPI::Ads8675<Spi, Clock, Pin::cs, Pin::rvs, Pin::rst> adc{[](Units::MicroVolt v) { ... }};
    ///     timer ISR:  adc.sampleCallback();      RVS edge ISR: adc.pinInterrupt();      loop: adc.turn();
    ///
    /// A sample due while the previous frame is still on the wire is skipped and counted.
    template<typename Master,
             typename Clock,
             typename Cs,
             typename Rvs,
             typename Rst,
             typename Config = Ads8675Defaults>
    class Ads8675 {
    public:
        using TimePoint = typename Clock::time_point;
        using Callback  = StaticFunction<void(Units::MicroVolt), 128>;

        static constexpr std::chrono::milliseconds ResetLow = Kvasir::asDuration(
          KVASIR_CONFIG_OR(Config, resetLow, std::chrono::milliseconds{Ads8675Defaults::resetLow}));
        static constexpr std::chrono::milliseconds ResetRecover = Kvasir::asDuration(
          KVASIR_CONFIG_OR(Config,
                           resetRecover,
                           std::chrono::milliseconds{Ads8675Defaults::resetRecover}));
        static constexpr std::uint8_t RangeSel = static_cast<std::uint8_t>(
          KVASIR_CONFIG_OR(Config, rangeSel, Ads8675Defaults::rangeSel));
        static constexpr std::uint8_t AbsentAfterFailures = static_cast<std::uint8_t>(
          KVASIR_CONFIG_OR(Config, absentAfterFailures, Ads8675Defaults::absentAfterFailures));
        static constexpr auto Setup
          = Master::setup(Hal::ClockMode::mode0,
                          KVASIR_CONFIG_OR(Config, maxClock, Ads8675Defaults::maxClock));

        [[nodiscard]] static constexpr Units::MicroVolt toVoltage(std::int16_t code) {
            return Units::microVolt(std::int32_t{code} * 625 / 2);
        }

        template<typename F>
            requires(!std::same_as<std::remove_cvref_t<F>,
                                   Ads8675>)
        explicit Ads8675(F&& f) : dataF_{std::forward<F>(f)} {}

        Ads8675(Ads8675 const&)            = delete;   // the frames' callbacks point at it
        Ads8675& operator=(Ads8675 const&) = delete;

        [[nodiscard]] bool ready() const { return ready_.load(std::memory_order_acquire); }

        [[nodiscard]] bool inFlight() const { return running_.load(std::memory_order_acquire); }

        [[nodiscard]] std::uint32_t errors() const {
            return errors_.load(std::memory_order_relaxed);
        }

        /// Samples due while the previous frame was still on the wire (also in errors()).
        [[nodiscard]] std::uint32_t skipped() const {
            return skipped_.load(std::memory_order_relaxed);
        }

        [[nodiscard]] std::uint32_t samples() const {
            return samples_.load(std::memory_order_relaxed);
        }

        /// Answering once RANGE_SEL read back, absent after AbsentAfterFailures failed bring-ups.
        [[nodiscard]] Link link() const { return bringUpFailures_.link(ready()); }

        [[nodiscard]] bool answering() const { return ready(); }

        /// CS high starts a conversion; skipped while a frame is still clocked out, which it would cut.
        void sampleCallback() {
            if(!ready()) { return; }
            if(running_.load(std::memory_order_acquire)) {
                errors_.fetch_add(1, std::memory_order_relaxed);
                skipped_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            apply(set(Cs{}));
        }

        void pinInterrupt() {
            if(!ready() || running_.load(std::memory_order_acquire)) { return; }
            data_               = std::array<std::byte, 4>{};
            bool const accepted = submit_([this](auto r) {
                if(r == decltype(r)::ok) {
                    auto const code = static_cast<std::int32_t>(Bytes{data_}.be16(0) >> 2) - 8192;
                    samples_.fetch_add(1, std::memory_order_relaxed);
                    dataF_(toVoltage(static_cast<std::int16_t>(code)));
                } else {
                    errors_.fetch_add(1, std::memory_order_relaxed);
                }
                running_.store(false, std::memory_order_release);
            });
            if(!accepted) {
                // Queue full: the sample is lost. CS back low here, or no rising edge and no RVS would
                // ever come again.
                errors_.fetch_add(1, std::memory_order_relaxed);
                apply(clear(Cs{}));
            }
        }

        void turn() {
            auto const now = Clock::now();
            logSkipped_(now);
            switch(state_) {
            case State::reset:
                ready_.store(false, std::memory_order_release);
                apply(clear(Rst{}));
                apply(clear(Cs{}));
                next_  = now + ResetLow;
                state_ = State::waitReset;
                break;
            case State::waitReset:
                if(now > next_) {
                    apply(set(Rst{}));
                    next_  = now + ResetRecover;
                    state_ = State::waitInit;
                }
                break;
            case State::waitInit:
                if(now > next_) {
                    apply(set(Cs{}));   // a conversion: its RVS edge says the part is ready
                    state_ = State::waitRdy;
                }
                break;
            case State::waitRdy:
                if(apply(read(Rvs{}))) {
                    // WRITE_HWORD (0xD0) RANGE_SEL (0x14) = RangeSel
                    command_(
                      {std::byte{0xD0}, std::byte{0x14}, std::byte{0x00}, std::byte{RangeSel}});
                    state_ = State::waitInitData;
                }
                break;
            case State::waitInitData:
                if(frameDone_(now)) { state_ = State::waitReadRdy; }
                break;
            case State::waitReadRdy:
                if(now > next_ && apply(read(Rvs{}))) {
                    // READ_HWORD (0xC8) RANGE_SEL (0x14): the word comes out in the next frame
                    command_({std::byte{0xC8}, std::byte{0x14}, std::byte{0x00}, std::byte{0x00}});
                    state_ = State::waitReadData;
                }
                break;
            case State::waitReadData:
                if(frameDone_(now)) { state_ = State::waitCheckRdy; }
                break;
            case State::waitCheckRdy:
                if(now > next_ && apply(read(Rvs{}))) {
                    command_({});   // NOP: carries RANGE_SEL[15:0] out
                    state_ = State::waitCheckData;
                }
                break;
            case State::waitCheckData:
                // CS stays low: the first sampleCallback() raises it
                if(!running_.load(std::memory_order_acquire)) {
                    if(ok_ && data_[0] == std::byte{0x00} && data_[1] == std::byte{RangeSel}) {
                        state_ = State::idle;
                        bringUpFailures_.cleared();
                        ready_.store(true, std::memory_order_release);
                    } else {
                        errors_.fetch_add(1, std::memory_order_relaxed);
                        bringUpFailed_();
                        UC_LOG_W("ads8675: RANGE_SEL reads {:#04x}{:02x}, expected {:#04x}",
                                 std::to_integer<unsigned>(data_[0]),
                                 std::to_integer<unsigned>(data_[1]),
                                 RangeSel);
                        state_ = State::reset;
                    }
                }
                break;
            case State::idle: break;
            }
        }

    private:
        enum class State : std::uint8_t {
            reset,
            waitReset,
            waitInit,
            waitRdy,
            waitInitData,
            waitReadRdy,
            waitReadData,
            waitCheckRdy,
            waitCheckData,
            idle
        };

        static constexpr std::uint32_t SkippedKey = rateLimitKey(2);

        static constexpr Lines csLines = ChipSelect<Cs, false>::lines;

        template<typename F>
        [[nodiscard]] bool submit_(F&& done) {
            running_.store(true, std::memory_order_release);
            bool const accepted
              = Master::submit(typename Master::Request{.setup = Setup,
                                                        .lines = csLines,
                                                        .tx    = std::span<std::byte const>{data_},
                                                        .rx    = std::span{data_},
                                                        .callback = std::forward<F>(done)});
            if(!accepted) { running_.store(false, std::memory_order_release); }
            return accepted;
        }

        /// A refusal is treated like a frame the bus failed.
        void command_(std::array<std::byte,
                                 4> const& frame) {
            data_ = frame;
            ok_   = false;
            static_cast<void>(submit_([this](auto r) {
                ok_ = r == decltype(r)::ok;
                running_.store(false, std::memory_order_release);
            }));
        }

        /// CS high starts the conversion whose RVS edge the next frame waits for.
        [[nodiscard]] bool frameDone_(TimePoint now) {
            if(running_.load(std::memory_order_acquire)) { return false; }
            if(!ok_) {
                errors_.fetch_add(1, std::memory_order_relaxed);
                bringUpFailed_();
                state_ = State::reset;
                return false;
            }
            apply(set(Cs{}));
            next_ = now + std::chrono::milliseconds{1};
            return true;
        }

        void bringUpFailed_() { bringUpFailures_.failed(); }

        void logSkipped_(TimePoint now) {
            auto const skipped = skipped_.load(std::memory_order_relaxed);
            if(skipped == skippedLogged_) { return; }
            auto const line = log_.allow(SkippedKey, now);
            if(!line) { return; }
            KVASIR_LOG_LIMITED(
              line,
              UC_LOG_W,
              "ads8675: {} sample(s) skipped, the previous frame still on the wire",
              skipped - skippedLogged_);
            skippedLogged_ = skipped;
        }

        Callback          dataF_;
        TimePoint         next_{};
        std::atomic<bool> running_{false};
        std::atomic<bool> ready_{false};
        bool volatile ok_{};
        State                                       state_{State::reset};
        FailureStreak<AbsentAfterFailures>          bringUpFailures_{};
        std::array<std::byte, 4>                    data_{};
        std::atomic<std::uint32_t>                  errors_;
        std::atomic<std::uint32_t>                  skipped_;
        std::uint32_t                               skippedLogged_{};
        std::atomic<std::uint32_t>                  samples_;
        [[no_unique_address]] LogRateLimiter<Clock> log_{};
    };

}}   // namespace Kvasir::SPI
