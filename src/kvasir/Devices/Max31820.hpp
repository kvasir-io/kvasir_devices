#pragma once
#include "Bytes.hpp"
#include "Duration.hpp"
#include "OneWire.hpp"
#include "Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace Kvasir {

/// The knobs of the MAX31820 / DS18B20 driver, with their defaults; derive and redeclare
/// what you change.
struct Max31820Defaults {
    /// A 12-bit conversion takes 750 ms at most (DS18B20 Table 2); read the scratchpad
    /// this long after Convert T.
    static constexpr auto ConversionTime = std::chrono::milliseconds{800};

    /// After a failed transaction (no presence pulse, a timeout, a CRC that does not check)
    /// before the next Convert T.
    static constexpr auto RetryDelay = std::chrono::milliseconds{200};
};

/// A Maxim MAX31820 / DS18B20 as the one part on a 1-Wire bus (Skip ROM): Convert T
/// (0x44), ConversionTime, Read Scratchpad (0xBE) with its CRC checked, over and over.
///
/// Two scratchpads pass the CRC and are still not readings: all nine bytes 0 -- a bus held
/// low after the presence pulse reads that way, and CRC-8 of zeros is zero -- and the power-on
/// value 0550h (+85 degC, "Temperature Register", note) with reserved byte 6 at its 0x0C
/// reset value, which is a part that browned out and has not converted since (Linux
/// w1_therm.c rejects the same). Both are failures. The configuration byte (byte 4) sets the
/// resolution, which is kept in EEPROM; below 12 bits the low bits of the count are
/// undefined (Configuration Register), so they are masked off by it.
///
/// Parasite power is not supported: it needs a strong pull-up within 10 us of Convert T for
/// the whole conversion ("Powering the MAX31820"), which this driver does not drive.
///
/// valid() and latest() are the I2C engine's words: valid() once a CRC-checked reading is
/// in since the last failure; the optional forms raw() and temperature() stay as a
/// convenience.
template<typename OneWire, typename Clock, typename Config = Max31820Defaults>
struct Max31820 {
    using TimePoint = typename Clock::time_point;
    using OS        = typename OneWire::OperationState;

    enum class State : std::uint8_t {
        startConversion,
        waitForConversionStart,
        waitForConversion,
        waitForData
    };

    static constexpr std::size_t readoutPacketSize{9};

    static constexpr std::chrono::milliseconds ConversionTime = [] {
        if constexpr(requires { Config::ConversionTime; }) {
            return Kvasir::asDuration(Config::ConversionTime);
        } else {
            return std::chrono::milliseconds{Max31820Defaults::ConversionTime};
        }
    }();

    static constexpr std::chrono::milliseconds RetryDelay = [] {
        if constexpr(requires { Config::RetryDelay; }) {
            return Kvasir::asDuration(Config::RetryDelay);
        } else {
            return std::chrono::milliseconds{Max31820Defaults::RetryDelay};
        }
    }();

    static_assert(OneWire::BufferSize >= readoutPacketSize,
                  "OneWire buffer to small");

    struct Sample {
        Units::MilliDegC temperature{};
        std::int16_t     raw{};   ///< the 12-bit two's-complement count, 1/16 degC
    };

    constexpr Max31820() = default;

    /// A CRC-checked reading is in, and nothing has failed since.
    [[nodiscard]] bool valid() const { return valid_; }

    [[nodiscard]] Sample const& latest() const { return sample_; }

    /// Steps with every CRC-checked reading.
    [[nodiscard]] std::uint32_t seq() const { return samples_; }

    [[nodiscard]] std::uint32_t samples() const { return samples_; }

    /// True once per new reading for the reader that keeps `seen`.
    [[nodiscard]] bool fresh(std::uint32_t& seen) const {
        if(seen == samples_) { return false; }
        seen = samples_;
        return true;
    }

    /// Transactions that failed (no presence pulse, a timeout, a bad CRC), over the
    /// device's life.
    [[nodiscard]] std::uint32_t errors() const { return errors_; }

    /// The last CRC-checked reading as the sensor's 12-bit two's-complement count (1/16 degC);
    /// empty after a failed transaction.
    [[nodiscard]] std::optional<std::int16_t> raw() const {
        if(!valid_) { return std::nullopt; }
        return sample_.raw;
    }

    /// The same reading as a temperature. One count is 62.5 mdegC, so this is within 0.5 mdegC.
    [[nodiscard]] std::optional<Units::MilliDegC> temperature() const {
        if(!valid_) { return std::nullopt; }
        return sample_.temperature;
    }

    [[nodiscard]] static constexpr Units::MilliDegC temperatureFor(std::int16_t raw) {
        return Units::milliDegC(raw * 125 / 2);
    }

    void handler() {
        auto const currentTime = Clock::now();
        switch(st_) {
        case State::startConversion:
            {
                if(currentTime > waitTime_ && OneWire::acquire()) {
                    st_ = State::waitForConversionStart;
                    OneWire::send(currentTime, std::array{std::byte{0xCC}, std::byte{0x44}});
                }
            }
            break;
        case State::waitForConversionStart:
            {
                switch(OneWire::operationState(currentTime)) {
                case OS::ongoing:
                    {
                    }
                    break;
                case OS::succeeded:
                    {
                        st_       = State::waitForConversion;
                        waitTime_ = currentTime + ConversionTime;
                        OneWire::release();
                    }
                    break;
                case OS::failed:
                    {
                        fail_(currentTime);
                        OneWire::release();
                    }
                    break;
                }
            }
            break;

        case State::waitForConversion:
            {
                if(currentTime > waitTime_ && OneWire::acquire()) {
                    OneWire::sendReceive(currentTime,
                                         std::array{std::byte{0xCC}, std::byte{0xBE}},
                                         readoutPacketSize);
                    st_ = State::waitForData;
                }
            }
            break;

        case State::waitForData:
            {
                switch(OneWire::operationState(currentTime)) {
                case OS::ongoing:
                    {
                    }
                    break;
                case OS::succeeded:
                    {
                        st_       = State::startConversion;
                        waitTime_ = currentTime;
                        std::array<std::byte, readoutPacketSize> buffer{};
                        OneWire::getReceivedBytes(buffer);
                        // The scratchpad's ninth byte is the CRC of the first eight.
                        auto const bytes   = Bytes{buffer};
                        bool const allZero = [&] {
                            for(auto const b : buffer) {
                                if(b != std::byte{0}) { return false; }
                            }
                            return true;
                        }();
                        bool const powerOn = bytes.le16(0) == 0x0550 && bytes.u8(6) == 0x0C;
                        if(Dallas::crc8(bytes) != 0 || allZero || powerOn) {
                            fail_(currentTime);
                        } else {
                            sample_.raw         = resolved(bytes.s16le(0), bytes.u8(4));
                            sample_.temperature = temperatureFor(sample_.raw);
                            valid_              = true;
                            ++samples_;
                        }
                        OneWire::release();
                    }
                    break;
                case OS::failed:
                    {
                        fail_(currentTime);
                        OneWire::release();
                    }
                    break;
                }
            }
            break;
        }
    }

    /// The count with the bits below the configured resolution cleared: R1:R0 in bits 6:5
    /// of the configuration byte, 9 bits (00) to 12 bits (11).
    [[nodiscard]] static constexpr std::int16_t resolved(std::int16_t raw,
                                                         std::uint8_t configuration) {
        auto const r    = static_cast<unsigned>((configuration >> 5U) & 0x03U);
        auto const mask = static_cast<std::uint16_t>(~((1U << (3U - r)) - 1U));
        return static_cast<std::int16_t>(static_cast<std::uint16_t>(raw) & mask);
    }

private:
    void fail_(TimePoint currentTime) {
        st_       = State::startConversion;
        waitTime_ = currentTime + RetryDelay;
        valid_    = false;
        ++errors_;
    }

    State         st_{State::startConversion};
    TimePoint     waitTime_{TimePoint::min()};
    Sample        sample_{};
    bool          valid_{false};
    std::uint32_t samples_{};
    std::uint32_t errors_{};
};
}   // namespace Kvasir
