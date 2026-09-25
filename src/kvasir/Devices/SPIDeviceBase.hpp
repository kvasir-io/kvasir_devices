#pragma once

#include "Duration.hpp"
#include "Link.hpp"
#include "Log.hpp"
#include "kvasir/Util/RateLimiter.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir {

/// Tuning for SPIDeviceBase and the drivers on it; override with a struct that has the same
/// members (or derive from this and redeclare the ones you change). A member left out reads
/// its default here.
struct SPIDeviceDefaults {
    /// A frame whose DMA completion never comes is abandoned after this long.
    static constexpr auto inFlightTimeout = std::chrono::milliseconds{200};

    /// Frames that failed (an RX overrun, a timeout) in a row before resetLogic() is run.
    static constexpr std::uint8_t errorThreshold = 3;

    /// A driver waits this long after a failed frame before it sends the next one, so a
    /// stuck bus is retried at a walking pace rather than every loop turn.
    static constexpr auto RetryDelay = std::chrono::milliseconds{5};

    /// A part that does not answer (a wrong id, no id) is probed again at this interval.
    static constexpr auto AbsentRetry = std::chrono::seconds{1};

    /// Failures in a row (reportFailure()) before the part is reported absent, like the I2C
    /// engine's AbsentAfterNaks. 0: never.
    static constexpr std::uint8_t AbsentAfterFailures = 3;
};

namespace detail {
    /// An SPI master that reports the end of a transfer through a callback (the RP2040 /
    /// RP2350 SPI, from the DMA interrupt). One without it (the SAMD21 SERCOM behaviour, which
    /// completes by polling its DMA channels) is asked transferInProgress() instead.
    template<typename S>
    concept SpiCompletionCallback
      = requires(std::span<std::byte const> tx, std::span<std::byte> rx) {
            S::send_nocopy(tx, [] {});
            S::send_receive_nocopy(tx, rx, [] {});
        };
}   // namespace detail

/// CRTP base for a device on a shared SPI bus (Kvasir::SPI::SPIBehavior) with its own chip
/// select, the SPI counterpart of what the I2C engine (I2C/Device.hpp) does for itself: the
/// driver writes resetLogic() and idleLogic(), a state machine of frames, and the base owns
///   * the frame: acquire the bus, chip select low, the DMA transfer, chip select high
///     and the bus released once it is done (in handler(), the next loop turn),
///   * the frame buffers tx_ / rx_ (`FrameBytes` each) and the one-command-byte register
///     frames every register-mapped part speaks, readRegister() and writeRegister(),
///   * the in-flight watchdog and the error threshold,
///   * the link: SPI has no acknowledge, so the driver decides (an id register that reads
///     right) with markAnswering() / markStarting() / reportFailure(); the base reports it in the
///     I2C engine's words, link() / answering() / absent() / bringUps() / errors().
///
/// A frame is full duplex: `tx` goes out, and `rx`, the same length, receives what came in
/// during it, command bytes included (the driver skips them). `rx` may be empty for a
/// write-only frame. Both buffers must stay put until take() returns something other
/// than running. The chip-select pin is configured by the application (an output, high).
///
///     struct Sensor : Kvasir::SPIDeviceBase<Spi, Clock, HW::Pin::spi_cs, Sensor> {
///         void resetLogic();
///         void idleLogic() {
///             if(state_ == read) { if(this->readRegister(0x80 | Reg, 6)) { state_ = readWait; } }
///             else if(state_ == readWait) {
///                 if(auto const o = this->take(); o != Outcome::running) { ... this->rx_[1] ... }
///             }
///         }
///     };
template<typename SPI,
         typename Clock,
         typename Cs,
         typename Derived,
         typename Config        = SPIDeviceDefaults,
         std::size_t FrameBytes = 32>
struct SPIDeviceBase {
    using TimePoint = typename Clock::time_point;

    enum class Outcome : std::uint8_t { running, ok, failed };

    static constexpr std::chrono::milliseconds InFlightTimeout = [] {
        if constexpr(requires { Config::inFlightTimeout; }) {
            return Kvasir::asDuration(Config::inFlightTimeout);
        } else {
            return std::chrono::milliseconds{SPIDeviceDefaults::inFlightTimeout};
        }
    }();

    static constexpr std::uint8_t ErrorThreshold = [] {
        if constexpr(requires { Config::errorThreshold; }) {
            return static_cast<std::uint8_t>(Config::errorThreshold);
        } else {
            return SPIDeviceDefaults::errorThreshold;
        }
    }();

    static constexpr std::chrono::milliseconds RetryDelay = [] {
        if constexpr(requires { Config::RetryDelay; }) {
            return Kvasir::asDuration(Config::RetryDelay);
        } else {
            return std::chrono::milliseconds{SPIDeviceDefaults::RetryDelay};
        }
    }();

    static constexpr std::chrono::milliseconds AbsentRetry = [] {
        if constexpr(requires { Config::AbsentRetry; }) {
            return Kvasir::asDuration(Config::AbsentRetry);
        } else {
            return std::chrono::milliseconds{SPIDeviceDefaults::AbsentRetry};
        }
    }();

    static constexpr std::uint8_t AbsentAfterFailures = [] {
        if constexpr(requires { Config::AbsentAfterFailures; }) {
            return static_cast<std::uint8_t>(Config::AbsentAfterFailures);
        } else {
            return SPIDeviceDefaults::AbsentAfterFailures;
        }
    }();

    constexpr SPIDeviceBase() = default;

    /// Start one frame. False when the bus is booked by another device or a frame of this
    /// one is still in flight: try again next turn.
    [[nodiscard]] bool submit(std::span<std::byte const> tx,
                              std::span<std::byte>       rx) {
        if(inFlight_) { return false; }
        if(!SPI::acquire()) { return false; }
        done_.store(false, std::memory_order_relaxed);
        finished_ = false;
        inFlight_ = true;
        since_    = Clock::now();
        apply(clear(Cs{}));
        std::atomic_signal_fence(std::memory_order_release);
        if constexpr(detail::SpiCompletionCallback<SPI>) {
            if(rx.empty()) {
                SPI::send_nocopy(tx, [this] { done_.store(true, std::memory_order_release); });
            } else {
                SPI::send_receive_nocopy(tx, rx, [this] {
                    done_.store(true, std::memory_order_release);
                });
            }
        } else {
            if(rx.empty()) {
                SPI::send_nocopy(tx);
            } else {
                SPI::send_receive_nocopy(tx, rx);
            }
        }
        return true;
    }

    /// The outcome of the last frame, once; running until handler() has closed it.
    [[nodiscard]] Outcome take() {
        if(inFlight_ || !finished_) { return Outcome::running; }
        finished_ = false;
        return lastOk_ ? Outcome::ok : Outcome::failed;
    }

    [[nodiscard]] bool inFlight() const { return inFlight_; }

    // -- the link, in the I2C engine's words (I2C/Device.hpp) ----------------------------

    /// Where the driver is with its part: `answering` once the driver has seen it (an id that
    /// reads right, a bring-up that went through), `absent` after AbsentAfterFailures
    /// failures in a row, `starting` otherwise (power-up, a reset, a bring-up under way).
    [[nodiscard]] Link link() const { return link_; }

    [[nodiscard]] bool answering() const { return link_ == Link::answering; }

    [[nodiscard]] bool absent() const { return link_ == Link::absent; }

    /// answering(), for callers that ask a yes/no question.
    [[nodiscard]] bool present() const { return answering(); }

    /// Bring-ups finished since power-up: steps on every markAnswering() from another state.
    [[nodiscard]] std::uint16_t bringUps() const { return bringUps_; }

    /// Frames that failed (an RX overrun, the in-flight watchdog) over the device's life.
    [[nodiscard]] std::uint32_t errors() const { return errors_; }

    /// Failures in a row towards being reported absent.
    [[nodiscard]] std::uint8_t consecutiveFailures() const { return consecutiveFailures_; }

    /// The part is there and configured: the driver's bring-up went through.
    void markAnswering() {
        if(link_ != Link::answering) { ++bringUps_; }
        link_                = Link::answering;
        consecutiveFailures_ = 0;
    }

    /// The driver starts over (a reset, a lost part): not absent yet, not answering either.
    void markStarting() {
        if(link_ == Link::answering) { link_ = Link::starting; }
    }

    void markAbsent() { link_ = Link::absent; }

    /// The driver's bring-up or its running read failed (a frame, or an answer that is not
    /// the part's) and it starts over; after AbsentAfterFailures in a row the part is
    /// reported absent. Returns absent().
    bool reportFailure() {
        if(consecutiveFailures_ != 0xFF) { ++consecutiveFailures_; }
        if(link_ == Link::answering) { link_ = Link::starting; }
        if(AbsentAfterFailures != 0 && consecutiveFailures_ >= AbsentAfterFailures) {
            link_ = Link::absent;
        }
        return absent();
    }

    /// How long to wait before the next attempt after a failure: AbsentRetry once the part
    /// is reported absent, RetryDelay before.
    [[nodiscard]] std::chrono::milliseconds retryDelay() const {
        return absent() ? AbsentRetry : RetryDelay;
    }

    void handler() {
        auto& self = static_cast<Derived&>(*this);
        if(inFlight_) {
            if(completed_() && !SPI::transferInProgress()) {
                closeFrame_(SPI::operationState() == SPI::OperationState::succeeded);
            } else if(Clock::now() - since_ > InFlightTimeout) {
                SPI::abortTransfer();
                closeFrame_(false);
                KVASIR_LOG_LIMITED(timeoutLog_.allow(TimeoutKey, Clock::now()),
                                   UC_LOG_W,
                                   "spi device: frame timed out after {}",
                                   InFlightTimeout);
            } else {
                return;
            }
        }
        if(consecutiveErrors_ >= ErrorThreshold) {
            consecutiveErrors_ = 0;
            self.resetLogic();
        }
        self.idleLogic();
    }

protected:
    /// The frame buffers: what goes out, and what came in during it (the first byte of rx_
    /// arrived while the command byte went out and means nothing).
    std::array<std::byte, FrameBytes> tx_{};
    std::array<std::byte, FrameBytes> rx_{};

    /// A register read: the command byte, then `n` bytes clocked in behind it. The answer
    /// is rx_[1 .. n] once take() says ok. `cmd` carries the part's read bit already.
    [[nodiscard]] bool readRegister(std::uint8_t cmd,
                                    std::size_t  n) {
        if(n + 1 > FrameBytes) { return false; }
        tx_[0] = std::byte{cmd};
        for(std::size_t i = 1; i <= n; ++i) { tx_[i] = std::byte{0}; }
        return submit(std::span<std::byte const>{tx_}.first(n + 1),
                      std::span<std::byte>{rx_}.first(n + 1));
    }

    /// A register write: the command byte and one data byte, write-only.
    [[nodiscard]] bool writeRegister(std::uint8_t cmd,
                                     std::uint8_t value) {
        tx_[0] = std::byte{cmd};
        tx_[1] = std::byte{value};
        return submit(std::span<std::byte const>{tx_}.first(2), {});
    }

    /// A register write of `value.size()` data bytes after the command byte, write-only.
    [[nodiscard]] bool writeRegister(std::uint8_t               cmd,
                                     std::span<std::byte const> value) {
        if(value.size() + 1 > FrameBytes) { return false; }
        tx_[0] = std::byte{cmd};
        for(std::size_t i = 0; i < value.size(); ++i) { tx_[1 + i] = value[i]; }
        return submit(std::span<std::byte const>{tx_}.first(value.size() + 1), {});
    }

    /// The `n` answer bytes of the last readRegister().
    [[nodiscard]] std::span<std::byte const> received(std::size_t n) const {
        return std::span<std::byte const>{rx_}.subspan(1, n);
    }

private:
    static constexpr std::uint32_t TimeoutKey = rateLimitKey(1);

    /// The completion callback has run -- or, on a master that has none, trivially true and
    /// transferInProgress() is the whole answer.
    [[nodiscard]] bool completed_() const {
        if constexpr(detail::SpiCompletionCallback<SPI>) {
            return done_.load(std::memory_order_acquire);
        } else {
            return true;
        }
    }

    void closeFrame_(bool ok) {
        apply(set(Cs{}));
        SPI::release();
        inFlight_ = false;
        finished_ = true;
        lastOk_   = ok;
        if(ok) {
            consecutiveErrors_ = 0;
        } else {
            ++errors_;
            ++consecutiveErrors_;
            SPI::clearError();
        }
    }

    std::atomic<bool>  done_{false};
    bool               inFlight_{false};
    bool               finished_{false};
    bool               lastOk_{false};
    Link               link_{Link::starting};
    TimePoint          since_{};
    std::uint32_t      errors_{};
    std::uint16_t      bringUps_{};
    std::uint8_t       consecutiveErrors_{};
    std::uint8_t       consecutiveFailures_{};
    RateLimiter<Clock> timeoutLog_{};
};

}   // namespace Kvasir
