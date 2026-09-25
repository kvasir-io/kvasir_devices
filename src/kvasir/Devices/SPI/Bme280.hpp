#pragma once

#include "../Bme280Compensation.hpp"
#include "../Bytes.hpp"
#include "../Duration.hpp"
#include "../Quantities.hpp"
#include "../SPIDeviceBase.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::SPI {

/// The knobs of the SPI BME280 driver, with their defaults; derive and redeclare what you
/// change. The SPIDeviceDefaults members (RetryDelay, AbsentRetry, AbsentAfterFailures, the
/// in-flight timeout) apply too.
struct Bme280Defaults : SPIDeviceDefaults {
    /// One burst read of the data registers per Period.
    static constexpr auto Period = std::chrono::milliseconds{1000};

    /// The chip's registers as written at bring-up (BST-BME280-DS001 5.4): ctrl_hum 0xF2
    /// (osrs_h x1), ctrl_meas 0xF4 (osrs_t x1, osrs_p x1, normal mode), config 0xF5
    /// (t_sb 1000 ms, filter off). setControl() changes them at run time.
    static constexpr std::uint8_t CtrlHum   = 0x01;
    static constexpr std::uint8_t CtrlMeas  = 0x27;
    static constexpr std::uint8_t ConfigReg = 0xA0;

    /// After the soft reset (5.4.2: 2 ms start-up), and before the first read after the
    /// bring-up (the first conversion in normal mode).
    static constexpr auto ResetSettle = std::chrono::milliseconds{3};
    static constexpr auto FirstRead   = std::chrono::milliseconds{10};
};

/// Bosch BME280 / BMP280 over SPI (BST-BME280-DS001, 6.3: mode 0 or 3, register address
/// with bit 7 clear for a write and set for a read, auto-increment). The same bring-up and
/// the same compensation (Bme280Compensation.hpp) as the I2C description, as a state machine
/// on SPIDeviceBase: reset, id, trimming, ctrl_hum, ctrl_meas with the mode bits cleared
/// (sleep), config, ctrl_meas as configured, then a burst read of the eight data registers
/// every `Period`. config is only written in sleep mode because in normal mode a write to it
/// may be ignored (5.4.6); setControl() goes through the same sequence. A frame whose
/// temperature is the skipped value 0x80000 is not a sample -- nothing can be compensated
/// without it; a skipped pressure or humidity (0x80000, 0x8000) is reported as 0.
///
/// The link follows the I2C engine's contract: valid() is answering() and a sample since the
/// last bring-up; a failed frame or a wrong id starts the bring-up over after RetryDelay,
/// AbsentAfterFailures of them in a row report the part absent and it is probed every
/// AbsentRetry.
template<typename Spi,
         typename Clock,
         typename Cs,
         Bme280Compensation::Model Model = Bme280Compensation::Model::bme280,
         typename Config                 = Bme280Defaults>
struct Bme280 : SPIDeviceBase<Spi, Clock, Cs, Bme280<Spi, Clock, Cs, Model, Config>, Config, 27> {
    static constexpr bool Humidity = Model == Bme280Compensation::Model::bme280;

    using Base    = SPIDeviceBase<Spi, Clock, Cs, Bme280, Config, 27>;
    using Outcome = typename Base::Outcome;
    using Trim    = Bme280Compensation::Trim;

    static constexpr std::string_view Name       = Humidity ? "BME280" : "BMP280";
    static constexpr std::uint8_t     ExpectedId = Humidity ? 0x60 : 0x58;

    static constexpr std::chrono::milliseconds Period = [] {
        if constexpr(requires { Config::Period; }) {
            return Kvasir::asDuration(Config::Period);
        } else {
            return std::chrono::milliseconds{Bme280Defaults::Period};
        }
    }();

    struct Sample {
        Units::CentiDegC    temperature{};
        Units::Pascal       pressure{};
        Units::MilliPercent humidity{};
    };

    void resetLogic() {
        state_ = State::reset;
        valid_ = false;
        this->markStarting();
    }

    void idleLogic() {
        auto const now = Clock::now();
        switch(state_) {
        case State::reset:
            // Soft reset, then ResetSettle for the chip to come back.
            if(this->writeRegister(RegReset & 0x7F, 0xB6)) { state_ = State::resetWait; }
            break;
        case State::resetWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                wakeAt_ = now + ResetSettle;
                state_  = State::settle;
            }
            break;
        case State::settle:
            if(now >= wakeAt_) { state_ = State::id; }
            break;
        case State::id:
            if(this->readRegister(RegId | 0x80, 1)) { state_ = State::idWait; }
            break;
        case State::idWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o == Outcome::ok && this->rx_[1] == std::byte{ExpectedId}) {
                    trim_.deviceId = ExpectedId;
                    state_         = State::trim1;
                } else {
                    fail_(now);
                }
            }
            break;
        case State::retry:
            if(now >= wakeAt_) { state_ = State::reset; }
            break;
        case State::trim1:
            if(this->readRegister(RegTrim1 | 0x80, 26)) { state_ = State::trim1Wait; }
            break;
        case State::trim1Wait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                trim_.loadTemperaturePressure(Bytes{this->received(26)}, 0);
                state_ = Humidity ? State::trim2 : State::sleep;
            }
            break;
        case State::trim2:
            if(this->readRegister(RegTrim2 | 0x80, 7)) { state_ = State::trim2Wait; }
            break;
        case State::trim2Wait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                trim_.loadHumidity(Bytes{this->received(7)}, 0);
                state_ = State::ctrlHum;
            }
            break;
        case State::ctrlHum:
            if(this->writeRegister(RegCtrlHum & 0x7F, ctrlHum_)) { state_ = State::ctrlHumWait; }
            break;
        case State::ctrlHumWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                state_ = State::sleep;
            }
            break;
        case State::sleep:
            if(this->writeRegister(RegCtrlMeas & 0x7F,
                                   static_cast<std::uint8_t>(ctrlMeas_ & 0xFCU)))
            {
                state_ = State::sleepWait;
            }
            break;
        case State::sleepWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                state_ = State::config;
            }
            break;
        case State::config:
            if(this->writeRegister(RegConfig & 0x7F, config_)) { state_ = State::configWait; }
            break;
        case State::configWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                state_ = State::ctrlMeas;
            }
            break;
        case State::ctrlMeas:
            if(this->writeRegister(RegCtrlMeas & 0x7F, ctrlMeas_)) { state_ = State::ctrlMeasWait; }
            break;
        case State::ctrlMeasWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                controlDirty_ = false;
                this->markAnswering();
                wakeAt_ = now + FirstRead;
                state_  = State::wait;
            }
            break;
        case State::wait:
            if(controlDirty_) {
                state_ = Humidity ? State::ctrlHum : State::sleep;
            } else if(now >= wakeAt_) {
                state_ = State::read;
            }
            break;
        case State::read:
            if(this->readRegister(RegData | 0x80, Humidity ? 8 : 6)) { state_ = State::readWait; }
            break;
        case State::readWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                decode_();
                wakeAt_ = now + Period;
                state_  = State::wait;
            }
            break;
        }
    }

    /// Answering, and a sample decoded since the last bring-up.
    [[nodiscard]] bool valid() const { return this->answering() && valid_; }

    [[nodiscard]] Sample const& latest() const { return sample_; }

    /// Steps with every new sample.
    [[nodiscard]] std::uint32_t seq() const { return samples_; }

    /// True once per new sample for the reader that keeps `seen`.
    [[nodiscard]] bool fresh(std::uint32_t& seen) const {
        if(seen == samples_) { return false; }
        seen = samples_;
        return true;
    }

    [[nodiscard]] std::uint32_t samples() const { return samples_; }

    [[nodiscard]] Trim const& trim() const { return trim_; }

    /// The control registers as the application wants them now (ctrl_hum is ignored on a
    /// BMP280); written again on the next turn, and at every bring-up from then on.
    void setControl(std::uint8_t ctrlHum,
                    std::uint8_t ctrlMeas,
                    std::uint8_t config) {
        ctrlHum_      = ctrlHum;
        ctrlMeas_     = ctrlMeas;
        config_       = config;
        controlDirty_ = true;
    }

private:
    enum class State : std::uint8_t {
        reset,
        resetWait,
        settle,
        id,
        idWait,
        retry,
        trim1,
        trim1Wait,
        trim2,
        trim2Wait,
        ctrlHum,
        ctrlHumWait,
        sleep,
        sleepWait,
        config,
        configWait,
        ctrlMeas,
        ctrlMeasWait,
        wait,
        read,
        readWait,
    };

    static constexpr std::uint8_t RegTrim1    = 0x88;
    static constexpr std::uint8_t RegId       = 0xD0;
    static constexpr std::uint8_t RegReset    = 0xE0;
    static constexpr std::uint8_t RegTrim2    = 0xE1;
    static constexpr std::uint8_t RegCtrlHum  = 0xF2;
    static constexpr std::uint8_t RegCtrlMeas = 0xF4;
    static constexpr std::uint8_t RegConfig   = 0xF5;
    static constexpr std::uint8_t RegData     = 0xF7;

    static constexpr std::chrono::milliseconds ResetSettle = [] {
        if constexpr(requires { Config::ResetSettle; }) {
            return Kvasir::asDuration(Config::ResetSettle);
        } else {
            return std::chrono::milliseconds{Bme280Defaults::ResetSettle};
        }
    }();

    static constexpr std::chrono::milliseconds FirstRead = [] {
        if constexpr(requires { Config::FirstRead; }) {
            return Kvasir::asDuration(Config::FirstRead);
        } else {
            return std::chrono::milliseconds{Bme280Defaults::FirstRead};
        }
    }();

    static constexpr std::uint8_t CtrlHumDefault = [] {
        if constexpr(requires { Config::CtrlHum; }) {
            return static_cast<std::uint8_t>(Config::CtrlHum);
        } else {
            return Bme280Defaults::CtrlHum;
        }
    }();

    static constexpr std::uint8_t CtrlMeasDefault = [] {
        if constexpr(requires { Config::CtrlMeas; }) {
            return static_cast<std::uint8_t>(Config::CtrlMeas);
        } else {
            return Bme280Defaults::CtrlMeas;
        }
    }();

    static constexpr std::uint8_t ConfigRegDefault = [] {
        if constexpr(requires { Config::ConfigReg; }) {
            return static_cast<std::uint8_t>(Config::ConfigReg);
        } else {
            return Bme280Defaults::ConfigReg;
        }
    }();

    /// A failed frame or a wrong id: start over after the base's retry delay.
    void fail_(typename Clock::time_point now) {
        valid_ = false;
        this->reportFailure();
        wakeAt_ = now + this->retryDelay();
        state_  = State::retry;
    }

    void decode_() {
        Bytes const b{this->received(Humidity ? 8 : 6)};
        auto const  adcP = static_cast<std::int32_t>(b.be24(0) >> 4);
        auto const  adcT = static_cast<std::int32_t>(b.be24(3) >> 4);
        if(adcT == Bme280Compensation::Skipped20) { return; }
        auto const tFine    = trim_.tFine(adcT);
        sample_.temperature = Units::centiDegC(Trim::centi(tFine));
        sample_.pressure    = adcP == Bme280Compensation::Skipped20
                              ? Units::pascal(0)
                              : Units::pascal(trim_.pressurePa(adcP, tFine));
        if constexpr(Humidity) {
            auto const adcH = b.be16(6);
            sample_.humidity
              = adcH == Bme280Compensation::Skipped16
                ? Units::milliPercent(0)
                : Units::milliPercent(trim_.humidityMilli(static_cast<std::int32_t>(adcH), tFine));
        }
        valid_ = true;
        ++samples_;
    }

    State                      state_{State::reset};
    typename Clock::time_point wakeAt_{};
    Trim                       trim_{};
    Sample                     sample_{};
    bool                       valid_{false};
    bool                       controlDirty_{false};
    std::uint8_t               ctrlHum_{CtrlHumDefault};
    std::uint8_t               ctrlMeas_{CtrlMeasDefault};
    std::uint8_t               config_{ConfigRegDefault};
    std::uint32_t              samples_{};
};

template<typename Spi, typename Clock, typename Cs, typename Config = Bme280Defaults>
using Bmp280 = Bme280<Spi, Clock, Cs, Bme280Compensation::Model::bmp280, Config>;

}   // namespace Kvasir::SPI
