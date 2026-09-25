#pragma once

#include "Bytes.hpp"
#include "Duration.hpp"
#include "Log.hpp"
#include "Quantities.hpp"
#include "SPIDeviceBase.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace Kvasir {

namespace Max31865Detail {
    /// floor(sqrt(v)), bit by bit.
    [[nodiscard]] constexpr std::uint64_t isqrt(std::uint64_t v) {
        std::uint64_t root = 0;
        std::uint64_t bit  = std::uint64_t{1} << 62;
        while(bit > v) { bit >>= 2; }
        while(bit != 0) {
            if(v >= root + bit) {
                v -= root + bit;
                root = (root >> 1) + bit;
            } else {
                root >>= 1;
            }
            bit >>= 2;
        }
        return root;
    }

    // The configuration register (0x00; written at 0x80): bit 7 VBIAS, bit 6 conversion
    // mode (1 auto), bit 5 one-shot, bit 4 3-wire, bits 3:2 fault-detection cycle, bit 1
    // fault status clear (self-clearing), bit 0 filter (1: 50 Hz).
    inline constexpr std::uint8_t FaultClear     = 0x02;
    inline constexpr std::uint8_t AutoConversion = 0x40;
    inline constexpr std::uint8_t Filter50Hz     = 0x01;

    // The fault status register (0x07): the RTD high / low threshold, REFIN- > 0.85 VBIAS,
    // REFIN- < 0.85 VBIAS (FORCE- open), RTDIN- < 0.85 VBIAS (FORCE- open), over/under
    // voltage.
    inline constexpr std::uint8_t FaultRtdHigh   = 0x80;
    inline constexpr std::uint8_t FaultRtdLow    = 0x40;
    inline constexpr std::uint8_t FaultRefInHigh = 0x20;
    inline constexpr std::uint8_t FaultRefInLow  = 0x10;
    inline constexpr std::uint8_t FaultRtdInLow  = 0x08;
    inline constexpr std::uint8_t FaultVoltage   = 0x04;
}   // namespace Max31865Detail

/// The knobs of the MAX31865 driver, with their defaults; derive and redeclare what you
/// change. The SPIDeviceDefaults members (RetryDelay, AbsentRetry, AbsentAfterFailures, the
/// in-flight timeout) apply too.
struct Max31865Defaults : SPIDeviceDefaults {
    /// Before the first configuration write after power-up.
    static constexpr auto StartupDelay = std::chrono::milliseconds{500};

    /// No conversion (DRDY) within this after the last one: the part is started over. In auto
    /// mode a conversion takes 16.7 ms at 60 Hz and 20 ms at 50 Hz, the first one after
    /// entering it 52 ms / 62.5 ms (electrical characteristics).
    static constexpr auto ConversionTimeout = std::chrono::milliseconds{500};

    /// The configuration register as written at bring-up: VBIAS on, auto conversion, 2- or
    /// 4-wire, no fault detection cycle, 50 Hz filter. setConfiguration() changes it at run
    /// time.
    static constexpr std::uint8_t Configuration = 0xC1;
};

/// Maxim MAX31865 RTD-to-digital converter over SPI (mode 1 or 3; register address, then
/// data; bit 7 of the address set for a write) on SPIDeviceBase: every frame books the
/// bus and has a watchdog. Bring-up: `Configuration` written and read back (the part is
/// answering() when it reads back right). Then, whenever DRDY goes low, the 15-bit RTD code
/// at 0x01..0x02 is read: the RTD's share of the reference resistor, R = code x Reference
/// / 2^15; bit 0 of the LSB is the fault flag, on which the fault status register 0x07 is
/// read and kept (fault()), the sample is rejected, and the flag cleared (configuration
/// bit 1) so the next conversion reports afresh.
///
/// The temperature is the Callendar-Van Dusen equation for T >= 0 degC,
/// R = R0 (1 + A T + B T^2) with A = 3.9083e-3 and B = -5.775e-7 (IEC 60751), solved for T
/// and used over the whole range; below 0 degC the C term it leaves out is worth about
/// 0.2 degC at -100 degC. It runs in 64-bit integers with A and B scaled by 1e10. (Linux
/// max31865.c reports a fixed 0.03125 degC per code instead, a linear fit that is off by
/// degrees away from 0 degC.)
///
/// `Nominal` is R0 (100 for a PT100, 1000 for a PT1000), `Reference` the board's reference
/// resistor. The defaults, a PT500 against 1 kOhm, are the board this was written for; the
/// datasheet's optimum is a reference of four times R0 ("Application Circuits": 400 Ohm for a
/// PT100, 4 kOhm for a PT1000), and a 2 x reference tops out near 260 degC.
///
/// No fault-detection cycle is run (configuration D3:D2 stay 00), so the REFIN-/RTDIN-
/// faults it finds -- an open FORCE- lead, a cable fault -- are not; the RTD thresholds keep
/// their reset values (0x0000 low, 0xFFFF high), and the fault flag reports over- and
/// under-voltage and the thresholds only. The link follows the I2C engine's contract: valid() is answering() and a
/// good conversion since the last bring-up; a failed frame, a configuration that does not
/// read back, or no conversion within ConversionTimeout starts over after RetryDelay, and
/// AbsentAfterFailures of them in a row report the part absent.
template<typename Clock,
         typename SPI,
         typename CsPin,
         typename DrdyPin,
         Units::Ohm Nominal   = Units::ohm(500),
         Units::Ohm Reference = Units::ohm(1000),
         typename Config      = Max31865Defaults>
struct Max31865
  : SPIDeviceBase<SPI,
                  Clock,
                  CsPin,
                  Max31865<Clock, SPI, CsPin, DrdyPin, Nominal, Reference, Config>,
                  Config,
                  4> {
    using Base      = SPIDeviceBase<SPI, Clock, CsPin, Max31865, Config, 4>;
    using Outcome   = typename Base::Outcome;
    using TimePoint = typename Clock::time_point;

    static constexpr std::string_view Name = "MAX31865";

    static constexpr std::chrono::milliseconds StartupDelay = [] {
        if constexpr(requires { Config::StartupDelay; }) {
            return Kvasir::asDuration(Config::StartupDelay);
        } else {
            return std::chrono::milliseconds{Max31865Defaults::StartupDelay};
        }
    }();

    static constexpr std::chrono::milliseconds ConversionTimeout = [] {
        if constexpr(requires { Config::ConversionTimeout; }) {
            return Kvasir::asDuration(Config::ConversionTimeout);
        } else {
            return std::chrono::milliseconds{Max31865Defaults::ConversionTimeout};
        }
    }();

    static constexpr std::uint8_t ConfigurationDefault = [] {
        if constexpr(requires { Config::Configuration; }) {
            return static_cast<std::uint8_t>(Config::Configuration);
        } else {
            return Max31865Defaults::Configuration;
        }
    }();

    /// The last good conversion.
    struct Sample {
        Units::MilliDegC temperature{};
        Units::MilliOhm  resistance{};
        std::uint16_t    code{};   ///< the 15-bit RTD code
    };

    Max31865() { apply(makeInput(DrdyPin{}), makeOutput(CsPin{}), set(CsPin{})); }

    /// Answering, and a conversion without the fault flag since the last bring-up.
    [[nodiscard]] bool valid() const { return this->answering() && valid_; }

    [[nodiscard]] Sample const& latest() const { return sample_; }

    /// Steps with every good conversion.
    [[nodiscard]] std::uint32_t seq() const { return samples_; }

    [[nodiscard]] std::uint32_t samples() const { return samples_; }

    /// True once per new sample for the reader that keeps `seen`.
    [[nodiscard]] bool fresh(std::uint32_t& seen) const {
        if(seen == samples_) { return false; }
        seen = samples_;
        return true;
    }

    /// The last RTD code (15 bits); empty while not valid().
    [[nodiscard]] std::optional<std::uint16_t> code() const {
        if(!valid()) { return std::nullopt; }
        return sample_.code;
    }

    [[nodiscard]] std::optional<Units::MilliOhm> resistance() const {
        if(!valid()) { return std::nullopt; }
        return sample_.resistance;
    }

    /// The temperature of the last good conversion; empty while not valid() (no conversion
    /// yet, the part away, or the fault flag set: an open or shorted RTD is a rejection, not
    /// a reading).
    [[nodiscard]] std::optional<Units::MilliDegC> temperature() const {
        if(!valid()) { return std::nullopt; }
        return sample_.temperature;
    }

    /// The fault status register (0x07, Max31865Detail::Fault*) as read the last time the
    /// fault flag was set; 0 when the part has reported none since the last bring-up.
    [[nodiscard]] std::uint8_t fault() const { return fault_; }

    /// Conversions rejected for the fault flag, over the device's life.
    [[nodiscard]] std::uint32_t faults() const { return faults_; }

    /// Clear the part's fault status (configuration bit 1) on the next turn. The driver does
    /// this itself after every faulted conversion; here for an application that wants the
    /// register's bits (fault()) back to zero.
    void clearFault() { clearFault_ = true; }

    /// The configuration register as the application wants it now (VBIAS, 3-wire, filter);
    /// written on the next turn and at every bring-up from then on. Bits 5, 3:2 and 1 are
    /// the one-shot, fault-detection cycle and fault clear commands and are masked off. The
    /// notch may not change during auto conversion (datasheet, Filter Select D0), so a new
    /// filter bit is preceded by a write that stops auto conversion.
    void setConfiguration(std::uint8_t configuration) {
        configuration_      = static_cast<std::uint8_t>(configuration & 0xD1);
        configurationDirty_ = true;
    }

    [[nodiscard]] std::uint8_t configuration() const { return configuration_; }

    [[nodiscard]] static constexpr Units::MilliOhm resistanceFor(std::uint16_t code) {
        return Units::milliOhm(std::uint64_t{code} * Units::value(Reference) * 1000U / 32768U);
    }

    /// The code is at or below R(850 degC) = R0 (1 + 850 A + 850^2 B), the top of the
    /// Callendar-Van Dusen range (3.9048 R0): a resistance above it is not a platinum RTD's.
    /// The discriminant going negative would only mark R/R0 past 7.6, which no reference of
    /// four R0 or less can read, so it is not what bounds the range.
    [[nodiscard]] static constexpr bool codeInRange(std::uint16_t code) {
        auto const topK = K + Ak * 850 + Bk * 850 * 850;   // R(850 degC) / R0, scaled by K
        return static_cast<std::int64_t>(code) * static_cast<std::int64_t>(Units::value(Reference))
               * K
            <= topK * static_cast<std::int64_t>(Units::value(Nominal)) * 32768;
    }

    /// The temperature a code stands for. A code past codeInRange() is not reported by the
    /// driver (temperature() is empty instead); where the equation has no solution it is
    /// clamped to 850 degC.
    [[nodiscard]] static constexpr Units::MilliDegC temperatureFor(std::uint16_t code) {
        auto const disc = discriminant_(code);
        if(disc < 0) { return Units::milliDegC(850'000); }
        auto const root
          = static_cast<std::int64_t>(Max31865Detail::isqrt(static_cast<std::uint64_t>(disc)));
        // T = (-A + sqrt(A^2 + 4 B (R / R0 - 1))) / (2 B), in millidegrees
        return Units::milliDegC((-Ak + root) * 1000 / (2 * Bk));
    }

    void resetLogic() {
        writtenKnown_ = false;
        state_        = State::configure;
        valid_        = false;
        this->markStarting();
    }

    void idleLogic() {
        auto const now = Clock::now();
        switch(state_) {
        case State::startup:
            wakeAt_ = now + StartupDelay;
            state_  = State::startupWait;
            break;
        case State::startupWait:
            if(now >= wakeAt_) { state_ = State::configure; }
            break;
        case State::configure:
            // Unknown after a (re)start: the part may still be converting from before.
            if(!writtenKnown_
               || ((written_ & Max31865Detail::AutoConversion) != 0
                   && ((written_ ^ configuration_) & Max31865Detail::Filter50Hz) != 0))
            {
                auto const stopped = static_cast<std::uint8_t>(
                  (writtenKnown_ ? written_ : configuration_) & ~Max31865Detail::AutoConversion);
                if(this->writeRegister(RegConfiguration | WriteBit, stopped)) {
                    written_      = stopped;
                    writtenKnown_ = true;
                    state_        = State::stopAutoWait;
                }
            } else if(this->writeRegister(RegConfiguration | WriteBit, configuration_)) {
                state_ = State::configureWait;
            }
            break;
        case State::stopAutoWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                state_ = State::configure;
            }
            break;
        case State::configureWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                configurationDirty_ = false;
                written_            = configuration_;
                state_              = State::verify;
            }
            break;
        case State::verify:
            if(this->readRegister(RegConfiguration, 1)) { state_ = State::verifyWait; }
            break;
        case State::verifyWait:
            if(auto const o = this->take(); o != Outcome::running) {
                // The command bits (one-shot, fault clear) read back as zero.
                if(o == Outcome::ok
                   && (std::to_integer<std::uint8_t>(this->rx_[1]) & 0xD1) == configuration_)
                {
                    this->markAnswering();
                    fault_  = 0;
                    wakeAt_ = now + ConversionTimeout;
                    state_  = State::idle;
                } else {
                    KVASIR_LOG_LIMITED(log_.allow(AbsentKey, now),
                                       UC_LOG_W,
                                       "max31865: configuration does not read back ({})",
                                       std::to_integer<std::uint8_t>(this->rx_[1]));
                    fail_(now);
                }
            }
            break;
        case State::retry:
            if(now >= wakeAt_) { state_ = State::configure; }
            break;
        case State::idle:
            if(configurationDirty_) {
                state_ = State::configure;
            } else if(clearFault_) {
                state_ = State::clearFault;
            } else if(!apply(read(DrdyPin{}))) {
                if(this->readRegister(RegRtd, 2)) { state_ = State::readWait; }
            } else if(now >= wakeAt_) {
                KVASIR_LOG_LIMITED(log_.allow(TimeoutKey, now),
                                   UC_LOG_W,
                                   "max31865: no conversion within {}",
                                   ConversionTimeout);
                fail_(now);
            }
            break;
        case State::readWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                wakeAt_         = now + ConversionTimeout;
                auto const word = Bytes{this->received(2)}.be16(0);
                if((word & 0x0001U) != 0) {
                    ++faults_;
                    valid_ = false;
                    state_ = State::faultStatus;
                    break;
                }
                auto const code = static_cast<std::uint16_t>(word >> 1U);
                if(!codeInRange(code)) {
                    ++faults_;
                    valid_ = false;
                    state_ = State::idle;
                    break;
                }
                sample_ = {temperatureFor(code), resistanceFor(code), code};
                valid_  = true;
                ++samples_;
                state_ = State::idle;
            }
            break;
        case State::faultStatus:
            if(this->readRegister(RegFaultStatus, 1)) { state_ = State::faultStatusWait; }
            break;
        case State::faultStatusWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                fault_ = std::to_integer<std::uint8_t>(this->rx_[1]);
                KVASIR_LOG_LIMITED(log_.allow(FaultKey, now),
                                   UC_LOG_W,
                                   "max31865: RTD fault, status {}",
                                   fault_);
                state_ = State::clearFault;
            }
            break;
        case State::clearFault:
            if(this->writeRegister(
                 RegConfiguration | WriteBit,
                 static_cast<std::uint8_t>(configuration_ | Max31865Detail::FaultClear)))
            {
                state_ = State::clearFaultWait;
            }
            break;
        case State::clearFaultWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                clearFault_ = false;
                state_      = State::idle;
            }
            break;
        }
    }

private:
    enum class State : std::uint8_t {
        startup,
        startupWait,
        configure,
        stopAutoWait,
        configureWait,
        verify,
        verifyWait,
        retry,
        idle,
        readWait,
        faultStatus,
        faultStatusWait,
        clearFault,
        clearFaultWait,
    };

    static constexpr std::uint8_t RegConfiguration = 0x00;
    static constexpr std::uint8_t RegRtd           = 0x01;   // MSB, then LSB at 0x02
    static constexpr std::uint8_t RegFaultStatus   = 0x07;
    static constexpr std::uint8_t WriteBit         = 0x80;

    static constexpr std::uint32_t AbsentKey  = rateLimitKey(2);
    static constexpr std::uint32_t TimeoutKey = rateLimitKey(3);
    static constexpr std::uint32_t FaultKey   = rateLimitKey(4);

    static constexpr std::int64_t K  = 10'000'000'000;   // A and B are scaled by this
    static constexpr std::int64_t Ak = 39'083'000;       // 3.9083e-3
    static constexpr std::int64_t Bk = -5'775;           // -5.775e-7

    /// A^2 + 4 B (R / R0 - 1), scaled by K^2, with R = code x Reference / 2^15.
    [[nodiscard]] static constexpr std::int64_t discriminant_(std::uint16_t code) {
        std::int64_t const r0  = Units::value(Nominal);
        std::int64_t const rr  = Units::value(Reference);
        auto const         rel = K * (std::int64_t{code} * rr - 32768 * r0) / (32768 * r0);
        return Ak * Ak + 4 * Bk * rel;
    }

    /// A failed frame, a configuration that does not read back, or a missing conversion:
    /// start over after the base's retry delay.
    void fail_(TimePoint now) {
        valid_        = false;
        writtenKnown_ = false;
        this->reportFailure();
        wakeAt_ = now + this->retryDelay();
        state_  = State::retry;
    }

    State              state_{State::startup};
    TimePoint          wakeAt_{};
    Sample             sample_{};
    bool               valid_{false};
    bool               clearFault_{false};
    bool               configurationDirty_{false};
    std::uint8_t       configuration_{ConfigurationDefault};
    std::uint8_t       written_{};   ///< the configuration the part holds, when writtenKnown_
    bool               writtenKnown_{false};
    std::uint8_t       fault_{};
    std::uint32_t      faults_{};
    std::uint32_t      samples_{};
    RateLimiter<Clock> log_{};
};

}   // namespace Kvasir
