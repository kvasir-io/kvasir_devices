#pragma once

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

/// The knobs of the MPU-9250 driver, with their defaults; derive and redeclare what you
/// change. The SPIDeviceDefaults members (RetryDelay, AbsentRetry, AbsentAfterFailures, the
/// in-flight timeout) apply too.
struct Mpu9250Defaults : SPIDeviceDefaults {
    /// One read of the 14 data registers per Period.
    static constexpr auto Period = std::chrono::milliseconds{20};

    /// After PWR_MGMT_1.H_RESET, before the part is talked to again (datasheet 3.4.2:
    /// "Start-up time for register read/write", 100 ms max).
    static constexpr auto ResetSettle = std::chrono::milliseconds{100};

    /// The registers as written at bring-up, in the order of ConfigWrites. setRanges()
    /// changes the two range registers at run time.
    static constexpr std::uint8_t SampleRateDiv = 9;   ///< SMPLRT_DIV: 1 kHz / (1 + 9) = 100 Hz
    static constexpr std::uint8_t Dlpf          = 3;   ///< CONFIG.DLPF_CFG: 41 Hz
    static constexpr std::uint8_t GyroRange     = 0;   ///< GYRO_FS_SEL: 0..3 for 250 .. 2000 dps
    static constexpr std::uint8_t AccelRange    = 0;   ///< ACCEL_FS_SEL: 0..3 for 2 .. 16 g
};

/// InvenSense MPU-9250 over SPI (PS-MPU-9250A-01, 1.3 / 7.5: up to 1 MHz for all
/// registers, 20 MHz for the sensor and interrupt registers; register address with bit 7
/// set for a read). Bring-up: WHO_AM_I (0x75) = 0x71, or 0x73 for the register-compatible
/// MPU-9255; PWR_MGMT_1 (0x6B) = 0x80 reset, ResetSettle; SIGNAL_PATH_RESET (0x68) = 0x07,
/// the gyro, accel and temperature paths, ResetSettle again (Linux inv_mpu_core.c does this
/// for the MPU-9250: "required for spi connection"); then PWR_MGMT_1 = 0x01 (PLL);
/// USER_CTRL (0x6A) I2C_IF_DIS so the part's I2C slave is off and cannot answer a line glitch
/// in SPI mode (register map 4.33, USER_CTRL bit 4); CONFIG (0x1A) DLPF; GYRO_CONFIG (0x1B)
/// and ACCEL_CONFIG (0x1C) ranges; ACCEL_CONFIG 2 (0x1D) the same DLPF code for the
/// accelerometer, whose filter is separate and out of reset is 460 Hz wide (4.7, table 2) --
/// sampled at 100 Hz that would alias. Data: 14 bytes from 0x3B every
/// `Period`: accel xyz, temperature, gyro xyz, int16 big-endian (16384 >> AccelRange LSB/g,
/// 131 / 2^GyroRange LSB/dps; degC = t / 334 + 21, the datasheet's 333.87 rounded), reported
/// as quantities at those scales. The magnetometer behind the auxiliary I2C is not brought up
/// here.
///
/// The link follows the I2C engine's contract: valid() is answering() and a sample since the
/// last bring-up; a failed frame or a wrong id starts the bring-up over after RetryDelay,
/// AbsentAfterFailures of them in a row report the part absent and it is probed every
/// AbsentRetry.
template<typename Spi, typename Clock, typename Cs, typename Config = Mpu9250Defaults>
struct Mpu9250 : SPIDeviceBase<Spi, Clock, Cs, Mpu9250<Spi, Clock, Cs, Config>, Config, 15> {
    using Base    = SPIDeviceBase<Spi, Clock, Cs, Mpu9250, Config, 15>;
    using Outcome = typename Base::Outcome;

    static constexpr std::string_view Name       = "MPU9250";
    static constexpr std::uint8_t     ExpectedId = 0x71;
    static constexpr std::uint8_t     Mpu9255Id  = 0x73;

    static constexpr std::chrono::milliseconds Period = [] {
        if constexpr(requires { Config::Period; }) {
            return Kvasir::asDuration(Config::Period);
        } else {
            return std::chrono::milliseconds{Mpu9250Defaults::Period};
        }
    }();

    struct Sample {
        std::array<Units::MicroG, 3>         accel{};         ///< 16384 >> AccelRange LSB/g
        Units::CentiDegC                     temperature{};   ///< degC = raw / 334 + 21
        std::array<Units::MilliDegPerSec, 3> gyro{};          ///< 131 / 2^GyroRange LSB/(deg/s)
    };

    void resetLogic() {
        state_ = State::id;
        valid_ = false;
        this->markStarting();
    }

    void idleLogic() {
        auto const now = Clock::now();
        switch(state_) {
        case State::id:
            if(this->readRegister(RegWhoAmI | 0x80, 1)) { state_ = State::idWait; }
            break;
        case State::idWait:
            if(auto const o = this->take(); o != Outcome::running) {
                whoAmI_ = static_cast<std::uint8_t>(this->rx_[1]);
                if(o == Outcome::ok && (whoAmI_ == ExpectedId || whoAmI_ == Mpu9255Id)) {
                    state_ = State::reset;
                } else {
                    fail_(now);
                }
            }
            break;
        case State::retry:
            if(now >= wakeAt_) { state_ = State::id; }
            break;
        case State::reset:
            if(this->writeRegister(RegPwrMgmt1, 0x80)) { state_ = State::resetWait; }
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
            if(now >= wakeAt_) { state_ = State::signalPath; }
            break;
        case State::signalPath:
            if(this->writeRegister(RegSignalPathReset, 0x07)) { state_ = State::signalPathWait; }
            break;
        case State::signalPathWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                wakeAt_ = now + ResetSettle;
                state_  = State::signalPathSettle;
            }
            break;
        case State::signalPathSettle:
            if(now >= wakeAt_) {
                // Every write goes out from here, the ranges as they are now among them: a
                // setRanges() from here on marks them for another pass.
                rangesDirty_ = false;
                step_        = 0;
                state_       = State::configure;
            }
            break;
        case State::configure:
            if(this->writeRegister(configWrites_[step_][0], configWrites_[step_][1])) {
                written_ = configWrites_[step_][1];
                state_   = State::configureWait;
            }
            break;
        case State::configureWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                // The scales decode uses follow the register as written, not as asked for.
                if(step_ == RangeStep) {
                    gyroRange_ = static_cast<std::uint8_t>(written_ >> 3U);
                } else if(step_ == RangeStep + 1) {
                    accelRange_ = static_cast<std::uint8_t>(written_ >> 3U);
                }
                if(++step_ < configWrites_.size()) {
                    state_ = State::configure;
                } else {
                    this->markAnswering();
                    wakeAt_ = now;
                    state_  = State::wait;
                }
            }
            break;
        case State::wait:
            if(rangesDirty_) {
                rangesDirty_ = false;
                step_        = RangeStep;
                state_       = State::configure;
            } else if(now >= wakeAt_) {
                state_ = State::read;
            }
            break;
        case State::read:
            if(this->readRegister(RegData | 0x80, 14)) { state_ = State::readWait; }
            break;
        case State::readWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                decode_();
                wakeAt_ += Period;
                if(wakeAt_ < now) { wakeAt_ = now + Period; }
                state_ = State::wait;
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

    [[nodiscard]] std::uint8_t whoAmI() const { return whoAmI_; }

    /// The full-scale ranges (GYRO_FS_SEL, ACCEL_FS_SEL: 0..3) as the application wants them
    /// now; written on the next turn and at every bring-up from then on. The scales of
    /// latest() follow once each register has been written -- a frame read before that is
    /// decoded at the range it was taken at.
    void setRanges(std::uint8_t gyroRange,
                   std::uint8_t accelRange) {
        configWrites_[RangeStep][1]     = static_cast<std::uint8_t>((gyroRange & 0x03U) << 3U);
        configWrites_[RangeStep + 1][1] = static_cast<std::uint8_t>((accelRange & 0x03U) << 3U);
        rangesDirty_                    = true;
    }

    /// The ranges the part holds, which latest() is scaled by.
    [[nodiscard]] std::uint8_t gyroRange() const { return gyroRange_; }

    [[nodiscard]] std::uint8_t accelRange() const { return accelRange_; }

private:
    enum class State : std::uint8_t {
        id,
        idWait,
        retry,
        reset,
        resetWait,
        settle,
        signalPath,
        signalPathWait,
        signalPathSettle,
        configure,
        configureWait,
        wait,
        read,
        readWait,
    };

    static constexpr std::uint8_t RegSmplrtDiv       = 0x19;
    static constexpr std::uint8_t RegConfig          = 0x1A;
    static constexpr std::uint8_t RegGyroConfig      = 0x1B;
    static constexpr std::uint8_t RegAccelConfig     = 0x1C;
    static constexpr std::uint8_t RegAccelConfig2    = 0x1D;
    static constexpr std::uint8_t RegSignalPathReset = 0x68;
    static constexpr std::uint8_t RegData            = 0x3B;
    static constexpr std::uint8_t RegUserCtrl        = 0x6A;
    static constexpr std::uint8_t RegPwrMgmt1        = 0x6B;
    static constexpr std::uint8_t RegWhoAmI          = 0x75;

    /// USER_CTRL.I2C_IF_DIS (bit 4): the I2C slave interface off, SPI only.
    static constexpr std::uint8_t I2cIfDis = 0x10;

    static constexpr std::chrono::milliseconds ResetSettle = [] {
        if constexpr(requires { Config::ResetSettle; }) {
            return Kvasir::asDuration(Config::ResetSettle);
        } else {
            return std::chrono::milliseconds{Mpu9250Defaults::ResetSettle};
        }
    }();

    static constexpr std::uint8_t SampleRateDiv = [] {
        if constexpr(requires { Config::SampleRateDiv; }) {
            return static_cast<std::uint8_t>(Config::SampleRateDiv);
        } else {
            return Mpu9250Defaults::SampleRateDiv;
        }
    }();

    static constexpr std::uint8_t Dlpf = [] {
        if constexpr(requires { Config::Dlpf; }) {
            return static_cast<std::uint8_t>(Config::Dlpf);
        } else {
            return Mpu9250Defaults::Dlpf;
        }
    }();

    static constexpr std::uint8_t GyroRangeDefault = [] {
        if constexpr(requires { Config::GyroRange; }) {
            return static_cast<std::uint8_t>(Config::GyroRange);
        } else {
            return Mpu9250Defaults::GyroRange;
        }
    }();

    static constexpr std::uint8_t AccelRangeDefault = [] {
        if constexpr(requires { Config::AccelRange; }) {
            return static_cast<std::uint8_t>(Config::AccelRange);
        } else {
            return Mpu9250Defaults::AccelRange;
        }
    }();

    static_assert(GyroRangeDefault <= 3 && AccelRangeDefault <= 3,
                  "GYRO_FS_SEL and ACCEL_FS_SEL are two bits");
    static_assert(Dlpf <= 7,
                  "DLPF_CFG is three bits");

    /// The index of GYRO_CONFIG in configWrites_; ACCEL_CONFIG follows. A range change
    /// re-runs the writes from here (ACCEL_CONFIG 2 after them goes out again too).
    static constexpr std::size_t RangeStep = 4;

    using Write = std::array<std::uint8_t, 2>;

    static constexpr std::array<Write, 7> InitialWrites{
      {
       {RegPwrMgmt1, 0x01},       // PWR_MGMT_1: auto-select clock (PLL when ready)
        {RegUserCtrl, I2cIfDis},   // USER_CTRL: I2C slave off, SPI only
        {RegSmplrtDiv, SampleRateDiv},
       {RegConfig, Dlpf},
       {RegGyroConfig, static_cast<std::uint8_t>(GyroRangeDefault << 3)},
       {RegAccelConfig, static_cast<std::uint8_t>(AccelRangeDefault << 3)},
       {RegAccelConfig2, Dlpf},   // ACCEL_FCHOICE_B 0: the A_DLPF_CFG filter is in
      }
    };

    /// A failed frame or a wrong id: start over after the base's retry delay.
    void fail_(typename Clock::time_point now) {
        valid_ = false;
        this->reportFailure();
        wakeAt_ = now + this->retryDelay();
        state_  = State::retry;
    }

    void decode_() {
        Bytes const b{this->received(14)};
        // 1e6 / 16384 = 15625 / 256 ug a count at 2 g, twice that per range step;
        // 1e3 / 131 mdps a count at 250 dps, likewise. The products need 64 bits: a full-scale
        // count at 16 g times 125000 is past INT32_MAX.
        auto const accelStep = std::int64_t{15625} << accelRange_;
        auto const gyroStep  = std::int64_t{1000} << gyroRange_;
        for(std::size_t i = 0; i < 3; ++i) {
            sample_.accel[i] = Units::microG(b.s16be(2 * i) * accelStep / 256);
            sample_.gyro[i]  = Units::milliDegPerSec(b.s16be(8 + 2 * i) * gyroStep / 131);
        }
        sample_.temperature
          = Units::centiDegC(static_cast<std::int32_t>(b.s16be(6)) * 100 / 334 + 2100);
        valid_ = true;
        ++samples_;
    }

    State                      state_{State::id};
    typename Clock::time_point wakeAt_{};
    std::size_t                step_{};
    std::array<Write, 7>       configWrites_{InitialWrites};
    std::uint8_t               written_{};   ///< the value of the write in flight
    Sample                     sample_{};
    bool                       valid_{false};
    bool                       rangesDirty_{false};
    std::uint8_t               gyroRange_{GyroRangeDefault};
    std::uint8_t               accelRange_{AccelRangeDefault};
    std::uint8_t               whoAmI_{};
    std::uint32_t              samples_{};
};

}   // namespace Kvasir::SPI
