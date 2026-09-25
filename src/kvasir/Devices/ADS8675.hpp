#pragma once

#include "Duration.hpp"
#include "Log.hpp"
#include "Quantities.hpp"
#include "kvasir/Util/RateLimiter.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <utility>

namespace Kvasir {

/// The knobs of the ADS8675 driver, with their defaults; derive and redeclare what you
/// change.
struct ADS8675Defaults {
    /// RST held low, then high before the part is talked to (the data sheet gives
    /// tD_RST_POR 20 ms typ and tPWRUP 20 ms; these leave the supplies time too).
    static constexpr auto ResetLow     = std::chrono::milliseconds{500};
    static constexpr auto ResetRecover = std::chrono::milliseconds{100};

    /// A frame whose DMA completion never comes is aborted after this long, chip select
    /// raised, and counted in errors().
    static constexpr auto InFlightTimeout = std::chrono::milliseconds{200};

    /// RANGE_SEL (0x14) as written at bring-up: 0x04 is +-0.625 x VREF with the internal
    /// 4.096 V reference, +-2.56 V. toVoltage() assumes this range.
    static constexpr std::uint8_t RangeSel = 0x04;
};

/// TI ADS8675 14-bit SAR ADC. Bring-up writes RANGE_SEL (0x14) = `RangeSel` (0x04: +-0.625 x
/// VREF with the internal 4.096 V reference, +-2.56 V over 2^14 codes, so one count is
/// 312.5 uV), then reads it back with READ_HWORD, whose register word comes out in the frame
/// after it (data sheet, "Input Command Word"): a part that does not hold `RangeSel` would
/// scale every sample wrong, so it is reset and brought up again instead, counted in
/// errors(). Sampling is driven from outside: the application raises chip select from
/// its sample timer (sampleCallback()), the part converts, and the RVS edge interrupt
/// (pinInterrupt()) lowers chip select and clocks the result out; the frame's completion
/// hands every conversion to the callback as the voltage it stands for. handler() runs the
/// bring-up and the in-flight watchdog.
template<typename Clock,
         typename SPI,
         typename Cs,
         typename Rvs,
         typename Rst,
         typename Config = ADS8675Defaults>
struct ADS8675 {
    using TimePoint = typename Clock::time_point;
    using Callback  = Kvasir::StaticFunction<void(Units::MicroVolt), 128>;

    static constexpr std::chrono::milliseconds ResetLow = [] {
        if constexpr(requires { Config::ResetLow; }) {
            return Kvasir::asDuration(Config::ResetLow);
        } else {
            return std::chrono::milliseconds{ADS8675Defaults::ResetLow};
        }
    }();

    static constexpr std::chrono::milliseconds ResetRecover = [] {
        if constexpr(requires { Config::ResetRecover; }) {
            return Kvasir::asDuration(Config::ResetRecover);
        } else {
            return std::chrono::milliseconds{ADS8675Defaults::ResetRecover};
        }
    }();

    static constexpr std::chrono::milliseconds InFlightTimeout = [] {
        if constexpr(requires { Config::InFlightTimeout; }) {
            return Kvasir::asDuration(Config::InFlightTimeout);
        } else {
            return std::chrono::milliseconds{ADS8675Defaults::InFlightTimeout};
        }
    }();

    static constexpr std::uint8_t RangeSel = [] {
        if constexpr(requires { Config::RangeSel; }) {
            return static_cast<std::uint8_t>(Config::RangeSel);
        } else {
            return ADS8675Defaults::RangeSel;
        }
    }();

    /// A signed 14-bit code (offset binary with 8192 removed).
    [[nodiscard]] static constexpr Units::MicroVolt toVoltage(std::int16_t code) {
        return Units::microVolt(std::int32_t{code} * 625 / 2);
    }

    template<typename F>
    explicit ADS8675(F&& f) : dataF_{std::forward<F>(f)} {}

    /// The bring-up went through: sampling is on.
    [[nodiscard]] bool ready() const { return ready_.load(std::memory_order_acquire); }

    [[nodiscard]] bool inFlight() const { return dmaRunning_.load(std::memory_order_acquire); }

    /// Frames the watchdog aborted, and samples due while a frame was still running.
    [[nodiscard]] std::uint32_t errors() const { return errors_; }

    /// Conversions handed to the callback.
    [[nodiscard]] std::uint32_t samples() const { return samples_; }

    /// From the sample timer: chip select high starts a conversion. A sample due while the
    /// previous frame is still being clocked out is skipped: raising chip select then would
    /// cut that frame short and start a conversion in the middle of it.
    void sampleCallback() {
        if(ready()) {
            if(dmaRunning_.load(std::memory_order_acquire)) {
                ++errors_;
                KVASIR_LOG_LIMITED(log_.allow(OverrunKey),
                                   UC_LOG_W,
                                   "ads8675: sample due while a transfer is still running");
                return;
            }
            apply(set(Cs{}));
        }
    }

    /// From the RVS edge: the conversion is done, clock it out.
    void pinInterrupt() {
        if(!ready() || dmaRunning_.load(std::memory_order_acquire)) { return; }
        apply(clear(Cs{}));
        since_ = Clock::now();
        dmaRunning_.store(true, std::memory_order_release);
        data_ = std::array<std::byte, 4>{};
        SPI::send_receive_nocopy(std::span{data_}, std::span{data_}, [this]() {
            std::uint16_t v{};
            std::memcpy(&v, data_.data(), 2);
            auto const code = static_cast<std::int32_t>(std::byteswap(v) >> 2) - 8192;
            ++samples_;
            dataF_(toVoltage(static_cast<std::int16_t>(code)));
            dmaRunning_.store(false, std::memory_order_release);
        });
    }

    void handler() {
        auto const now = Clock::now();
        // The watchdog: a frame started from the interrupt whose completion never came.
        if(dmaRunning_.load(std::memory_order_acquire) && now - since_ > InFlightTimeout) {
            SPI::abortTransfer();
            apply(set(Cs{}));
            dmaRunning_.store(false, std::memory_order_release);
            ++errors_;
            KVASIR_LOG_LIMITED(log_.allow(TimeoutKey, now),
                               UC_LOG_W,
                               "ads8675: frame timed out after {}",
                               InFlightTimeout);
        }
        switch(s_) {
        case State::reset:
            {
                apply(clear(Rst{}));
                apply(clear(Cs{}));
                next_ = now + ResetLow;
                s_    = State::waitReset;
            }
            break;
        case State::waitReset:
            {
                if(now > next_) {
                    apply(set(Rst{}));
                    next_ = now + ResetRecover;
                    s_    = State::waitInit;
                }
            }
            break;
        case State::waitInit:
            {
                if(now > next_) {
                    apply(set(Cs{}));
                    s_ = State::waitRdy;
                }
            }
            break;
        case State::waitRdy:
            {
                if(apply(read(Rvs{}))) {
                    // WRITE_HWORD (0xD0) RANGE_SEL (0x14) = RangeSel.
                    commandFrame_(
                      now,
                      {std::byte{0xD0}, std::byte{0x14}, std::byte{0x00}, std::byte{RangeSel}});
                    s_ = State::waitInitData;
                }
            }
            break;
        case State::waitInitData:
            {
                if(frameDone_(now)) { s_ = State::waitReadRdy; }
            }
            break;
        case State::waitReadRdy:
            {
                if(now > next_ && apply(read(Rvs{}))) {
                    // READ_HWORD (0xC8) RANGE_SEL (0x14): the word comes out in the next frame.
                    commandFrame_(
                      now,
                      {std::byte{0xC8}, std::byte{0x14}, std::byte{0x00}, std::byte{0x00}});
                    s_ = State::waitReadData;
                }
            }
            break;
        case State::waitReadData:
            {
                if(frameDone_(now)) { s_ = State::waitCheckRdy; }
            }
            break;
        case State::waitCheckRdy:
            {
                if(now > next_ && apply(read(Rvs{}))) {
                    commandFrame_(now, {});   // NOP: carries RANGE_SEL[15:0] out
                    s_ = State::waitCheckData;
                }
            }
            break;
        case State::waitCheckData:
            {
                // Chip select stays low: the first sampleCallback() raises it.
                if(!dmaRunning_.load(std::memory_order_acquire)) {
                    if(data_[0] == std::byte{0x00} && data_[1] == std::byte{RangeSel}) {
                        s_ = State::idle;
                    } else {
                        ++errors_;
                        UC_LOG_W("ads8675: RANGE_SEL reads {:#04x}{:02x}, expected {:#04x}",
                                 std::to_integer<unsigned>(data_[0]),
                                 std::to_integer<unsigned>(data_[1]),
                                 RangeSel);
                        s_ = State::reset;
                    }
                }
            }
            break;
        case State::idle:
            {
                ready_.store(true, std::memory_order_release);
            }
            break;
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

    /// One bring-up frame: chip select low, the four bytes clocked out and replaced by what
    /// the part sent.
    void commandFrame_(TimePoint            now,
                       std::array<std::byte,
                                  4> const& frame) {
        apply(clear(Cs{}));
        since_ = now;
        dmaRunning_.store(true, std::memory_order_release);
        data_ = frame;
        SPI::send_receive_nocopy(std::span{data_}, std::span{data_}, [this]() {
            dmaRunning_.store(false, std::memory_order_release);
        });
    }

    /// The bring-up frame has completed: chip select goes high, which starts the conversion
    /// whose RVS edge the next frame waits for, a millisecond on.
    [[nodiscard]] bool frameDone_(TimePoint now) {
        if(dmaRunning_.load(std::memory_order_acquire)) { return false; }
        apply(set(Cs{}));
        next_ = now + std::chrono::milliseconds{1};
        return true;
    }

    static constexpr std::uint32_t TimeoutKey = rateLimitKey(1);
    static constexpr std::uint32_t OverrunKey = rateLimitKey(2);

    Callback                 dataF_;
    TimePoint                next_{};
    TimePoint                since_{};
    std::atomic<bool>        dmaRunning_{false};
    std::atomic<bool>        ready_{false};
    State                    s_{State::reset};
    std::array<std::byte, 4> data_{};
    std::uint32_t            errors_{};
    std::uint32_t            samples_{};
    RateLimiter<Clock>       log_{};
};
}   // namespace Kvasir
