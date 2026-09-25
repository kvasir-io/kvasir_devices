#pragma once

#include "../SPIDeviceBase.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::SPI {

/// The knobs of the MAX7219 driver, with their defaults; derive and redeclare what you
/// change. The SPIDeviceDefaults members (RetryDelay after a failed frame, the in-flight
/// timeout) apply too.
struct Max7219Defaults : SPIDeviceDefaults {
    /// The intensity register at bring-up (0..15); setIntensity() changes it at run time.
    static constexpr std::uint8_t Intensity = 8;

    /// The scan-limit register at bring-up: digits 0..ScanLimit are driven (Table 8).
    static constexpr std::uint8_t ScanLimit = 7;

    /// How often the control registers, the intensity and every digit are written again.
    static constexpr auto RefreshPeriod = std::chrono::milliseconds{1000};
};

/// Maxim MAX7219 / MAX7221 LED driver (MAX7219 data sheet, Tables 1..10): eight digits of
/// eight segments, or an 8 x 8 matrix, per chip; chips are cascaded and every frame is one
/// 16-bit (address, data) word per chip, the last chip's first. Write-only. Bring-up:
/// shutdown mode, display test off, scan limit, decode mode (`Decode`: a bit per digit, 0xFF
/// for BCD digits on a 7-segment display, 0 for a matrix), the intensity, every digit, and
/// only then normal operation, so what the digit registers held from before is never shown
/// (LedControl clears them while shut down the same way). The driver keeps a shadow of the
/// eight digit registers per module and writes those that changed, one register across all
/// modules per frame.
///
/// After a supply brown-out the part is back in shutdown with its control registers reset
/// ("Initial Power-Up"), and a write-only part reports nothing of it, so every `RefreshPeriod`
/// the control registers -- display test, scan limit, decode mode, normal operation -- the
/// intensity and every digit are written again.
///
/// Write-only, so the link is what the bus reports: answering() once the bring-up frames
/// went out; a failed frame is retried after RetryDelay, and AbsentAfterFailures of them in
/// a row report the chain absent (probed every AbsentRetry).
///
///   using Display = Kvasir::SPI::Max7219<Spi, Clock, HW::Pin::spi_cs, 1, 0xFF>;
///   display.setDigit(0, 3, 7);        // module 0, digit 3 shows "7" (BCD decode)
///   display.setRow(0, 2, 0b10101010); // module 0, row 2 of a matrix (no decode)
///   display.setIntensity(8);          // 0..15
template<typename Spi,
         typename Clock,
         typename Cs,
         std::size_t  Modules = 1,
         std::uint8_t Decode  = 0xFF,
         typename Config      = Max7219Defaults>
struct Max7219
  : SPIDeviceBase<Spi,
                  Clock,
                  Cs,
                  Max7219<Spi, Clock, Cs, Modules, Decode, Config>,
                  Config,
                  2 * Modules> {
    static_assert(Modules >= 1 && Modules <= 8,
                  "1..8 cascaded chips");

    using Base    = SPIDeviceBase<Spi, Clock, Cs, Max7219, Config, 2 * Modules>;
    using Outcome = typename Base::Outcome;

    static constexpr std::string_view Name = "MAX7219";

    /// The digit registers RegDigit0..RegDigit0 + DigitCount - 1: eight digits, or the eight
    /// rows of a matrix, per module.
    static constexpr std::size_t DigitCount = 8;

    static constexpr std::uint8_t RegDigit0     = 0x01;
    static constexpr std::uint8_t RegDecodeMode = 0x09;
    static constexpr std::uint8_t RegIntensity  = 0x0A;
    static constexpr std::uint8_t RegScanLimit  = 0x0B;
    static constexpr std::uint8_t RegShutdown   = 0x0C;
    static constexpr std::uint8_t RegTest       = 0x0F;

    /// BCD decode: 0..9, then '-', 'E', 'H', 'L', 'P', blank (Table 5); 0x80 adds the point.
    static constexpr std::uint8_t Dash  = 0x0A;
    static constexpr std::uint8_t Blank = 0x0F;
    static constexpr std::uint8_t Point = 0x80;

    /// The intensity register's range: four bits, 0 (dimmest) to 15.
    static constexpr std::uint8_t MaxIntensity = 15;

    static constexpr std::uint8_t IntensityDefault = [] {
        if constexpr(requires { Config::Intensity; }) {
            return static_cast<std::uint8_t>(Config::Intensity & MaxIntensity);
        } else {
            return Max7219Defaults::Intensity;
        }
    }();

    static constexpr std::uint8_t ScanLimit = [] {
        if constexpr(requires { Config::ScanLimit; }) {
            return static_cast<std::uint8_t>(Config::ScanLimit & 0x07);
        } else {
            return Max7219Defaults::ScanLimit;
        }
    }();

    static constexpr std::chrono::milliseconds RefreshPeriod = [] {
        if constexpr(requires { Config::RefreshPeriod; }) {
            return Kvasir::asDuration(Config::RefreshPeriod);
        } else {
            return std::chrono::milliseconds{Max7219Defaults::RefreshPeriod};
        }
    }();

    /// Every digit blank until the application sets one.
    Max7219() { clear(); }

    /// Every register after a bring-up: the digits, and the intensity the application set.
    void resetLogic() {
        state_          = State::init;
        step_           = 0;
        dirty_          = 0xFF;
        intensityDirty_ = true;
        awake_          = false;
        this->markStarting();
    }

    void idleLogic() {
        auto const now = Clock::now();
        switch(state_) {
        case State::init:
            awake_ = false;
            if(all_(InitWrites[step_][0], InitWrites[step_][1])) { state_ = State::initWait; }
            break;
        case State::refresh:
            if(all_(RefreshWrites[step_][0], RefreshWrites[step_][1])) {
                state_ = State::refreshWait;
            }
            break;
        case State::refreshWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    dirty_          = 0xFF;
                    intensityDirty_ = true;
                    fail_(now);
                    break;
                }
                state_ = ++step_ < RefreshWrites.size() ? State::refresh : State::idle;
            }
            break;
        case State::initWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                if(++step_ < InitWrites.size()) {
                    state_ = State::init;
                } else {
                    this->markAnswering();   // write-only: answering means the bus took it
                    state_ = State::idle;
                }
            }
            break;
        case State::retry:
            if(now >= wakeAt_) {
                step_  = 0;
                state_ = State::init;
            }
            break;
        case State::idle:
            if(awake_ && !intensityDirty_ && dirty_ == 0 && now >= refreshAt_) {
                // A brown-out resets the part silently: everything again.
                refreshAt_      = now + RefreshPeriod;
                intensityDirty_ = true;
                dirty_          = 0xFF;
                step_           = 0;
                state_          = State::refresh;
            } else if(intensityDirty_) {
                if(all_(RegIntensity, intensity_)) {
                    intensityDirty_ = false;
                    state_          = State::writeWait;
                }
            } else if(dirty_ != 0) {
                std::uint8_t digit = 0;
                while(((dirty_ >> digit) & 1U) == 0) { ++digit; }
                for(std::size_t m = 0; m < Modules; ++m) {
                    // The last module in the chain gets the first word.
                    this->tx_[2 * m]     = std::byte{static_cast<std::uint8_t>(RegDigit0 + digit)};
                    this->tx_[2 * m + 1] = std::byte{shadow_[Modules - 1 - m][digit]};
                }
                if(this->submit(std::span<std::byte const>{this->tx_}, {})) {
                    dirty_ &= static_cast<std::uint8_t>(~(1U << digit));
                    state_ = State::writeWait;
                }
            } else if(!awake_) {
                // The digits and the intensity are in: now show them.
                if(all_(RegShutdown, 1)) {
                    awake_     = true;
                    refreshAt_ = now + RefreshPeriod;
                    state_     = State::writeWait;
                }
            }
            break;
        case State::writeWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    // What the chain holds is unknown: bring it up again, every register.
                    dirty_          = 0xFF;
                    intensityDirty_ = true;
                    fail_(now);
                    break;
                }
                state_ = State::idle;
            }
            break;
        }
    }

    /// Digit `digit` (0..DigitCount - 1) of module `module`: a BCD code with Decode,
    /// segments without.
    void setDigit(std::size_t  module,
                  std::size_t  digit,
                  std::uint8_t value) {
        if(module >= Modules || digit >= DigitCount) { return; }
        if(shadow_[module][digit] == value) { return; }
        shadow_[module][digit] = value;
        dirty_ |= static_cast<std::uint8_t>(1U << digit);
    }

    /// Row `row` of a matrix module (the same register as a digit).
    void setRow(std::size_t  module,
                std::size_t  row,
                std::uint8_t bits) {
        setDigit(module, row, bits);
    }

    /// A decimal number on module `module`'s digits, right aligned, blank leading digits.
    void setNumber(std::size_t   module,
                   std::uint32_t value,
                   std::uint8_t  digits = DigitCount) {
        for(std::uint8_t d = 0; d < digits && d < DigitCount; ++d) {
            setDigit(module,
                     d,
                     (value == 0 && d != 0) ? Blank : static_cast<std::uint8_t>(value % 10));
            value /= 10;
        }
    }

    void setIntensity(std::uint8_t level) {
        intensity_      = static_cast<std::uint8_t>(level & MaxIntensity);
        intensityDirty_ = true;
    }

    [[nodiscard]] std::uint8_t intensity() const { return intensity_; }

    /// Every digit off: the blank code where the digit is BCD-decoded, no segments where
    /// it is not (Decode is a bit per digit).
    void clear() {
        for(auto& m : shadow_) {
            for(std::size_t d = 0; d < DigitCount; ++d) { m[d] = blankFor(d); }
        }
        dirty_ = 0xFF;
    }

    /// The value that shows nothing on digit `digit`.
    [[nodiscard]] static constexpr std::uint8_t blankFor(std::size_t digit) {
        return ((Decode >> digit) & 1U) != 0 ? Blank : std::uint8_t{0};
    }

    [[nodiscard]] bool pending() const {
        return dirty_ != 0 || intensityDirty_ || this->inFlight();
    }

private:
    enum class State : std::uint8_t {
        init,
        initWait,
        retry,
        idle,
        writeWait,
        refresh,
        refreshWait
    };

    /// Shut down while the configuration goes in; normal operation follows the digits.
    static constexpr std::array<std::array<std::uint8_t, 2>, 4> InitWrites{
      {
       {RegShutdown, 0},
       {RegTest, 0},
       {RegScanLimit, ScanLimit},
       {RegDecodeMode, Decode},
       }
    };

    /// The periodic rewrite, which stays in normal operation throughout.
    static constexpr std::array<std::array<std::uint8_t, 2>, 4> RefreshWrites{
      {
       {RegTest, 0},
       {RegScanLimit, ScanLimit},
       {RegDecodeMode, Decode},
       {RegShutdown, 1},
       }
    };

    /// The same (register, value) to every module in the chain.
    bool all_(std::uint8_t reg,
              std::uint8_t value) {
        for(std::size_t m = 0; m < Modules; ++m) {
            this->tx_[2 * m]     = std::byte{reg};
            this->tx_[2 * m + 1] = std::byte{value};
        }
        return this->submit(std::span<std::byte const>{this->tx_}, {});
    }

    /// A failed frame: bring the chain up again after the base's retry delay.
    void fail_(typename Clock::time_point now) {
        this->reportFailure();
        wakeAt_ = now + this->retryDelay();
        state_  = State::retry;
    }

    State                                                     state_{State::init};
    std::size_t                                               step_{};
    typename Clock::time_point                                wakeAt_{};
    std::array<std::array<std::uint8_t, DigitCount>, Modules> shadow_{};
    std::uint8_t                                              dirty_{0xFF};
    std::uint8_t                                              intensity_{IntensityDefault};
    bool                                                      intensityDirty_{true};
    bool                       awake_{false};   ///< shutdown 1 written
    typename Clock::time_point refreshAt_{};
};

}   // namespace Kvasir::SPI
