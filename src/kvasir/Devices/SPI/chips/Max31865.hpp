#pragma once
// Maxim MAX31865 RTD-to-digital converter (MAX31865.md): a description for the wire and the
// register map, and the driver, which reads on DRDY, keeps the fault status and brings the part
// up again when it rejects conversion after conversion.
#include "../../Duration.hpp"
#include "../../Log.hpp"
#include "../../Quantities.hpp"
#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace SPI {

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

        // Configuration register (00h read, 80h write; table 1).
        inline constexpr std::uint8_t FaultClear     = 0x02;
        inline constexpr std::uint8_t AutoConversion = 0x40;
        inline constexpr std::uint8_t SettingBits    = 0xD1;

        // Fault status register (07h).
        inline constexpr std::uint8_t FaultRtdHigh   = 0x80;
        inline constexpr std::uint8_t FaultRtdLow    = 0x40;
        inline constexpr std::uint8_t FaultRefInHigh = 0x20;
        inline constexpr std::uint8_t FaultRefInLow  = 0x10;
        inline constexpr std::uint8_t FaultRtdInLow  = 0x08;
        inline constexpr std::uint8_t FaultVoltage   = 0x04;

        inline constexpr std::uint8_t RegConfiguration = 0x00;
        inline constexpr std::uint8_t RegRtd           = 0x01;   // MSB, then LSB at 02h
        inline constexpr std::uint8_t RegHighThreshold = 0x03;   // MSB, LSB, then low at 05h
        inline constexpr std::uint8_t RegFaultStatus   = 0x07;

        inline constexpr std::array<std::uint8_t, 4> ThresholdsPor{0xFF, 0xFF, 0x00, 0x00};
    }   // namespace Max31865Detail

    /// Derive and redeclare what you change; the engine's knobs (I2C/Device.hpp EngineDefaults) too.
    struct Max31865Defaults {
        static constexpr auto StartupDelay = std::chrono::milliseconds{500};
        /// No conversion within this: the part is started over. A conversion takes 16.7 / 20 ms, the
        /// first one 52 / 62.5 ms (MAX31865.md:148-151).
        static constexpr auto ConversionTimeout = std::chrono::milliseconds{500};
        /// VBIAS on, auto conversion, 2- or 4-wire, no fault detection cycle, 50 Hz filter.
        static constexpr std::uint8_t Configuration = 0xC1;
        /// Conversions rejected in a row before a bring-up (50 = 1 s at 50 Hz). 0: never.
        static constexpr std::uint16_t RejectedRestart   = 50;
        static constexpr auto          RegisterCheck     = std::chrono::seconds{10};
        static constexpr auto          UnidentifiedRetry = std::chrono::seconds{1};
    };

    namespace Chips {
        template<std::uint8_t Configuration, std::chrono::milliseconds::rep RegisterCheckMs>
        struct Max31865 {
            static constexpr std::string_view Name = "MAX31865";
            /// Modes 1 and 3 (MAX31865.md:616, :671); SCLK up to 5 MHz (tCLK, :184).
            static constexpr ClockMode    Mode     = ClockMode::_3;
            static constexpr Units::Hertz MaxClock = Units::hertz(5'000'000);

            /// Bit 7 set for a write (table 1, :479), the reverse of the transport's default.
            static constexpr std::uint8_t readCommand(std::uint8_t reg) {
                return static_cast<std::uint8_t>(reg & 0x7FU);
            }

            static constexpr std::uint8_t writeCommand(std::uint8_t reg) {
                return static_cast<std::uint8_t>(reg | 0x80U);
            }

            static constexpr std::size_t RegisterBytes = 1;

            static constexpr std::uint8_t Stopped
              = static_cast<std::uint8_t>(Configuration & ~Max31865Detail::AutoConversion);

            /// Auto conversion stopped first: the notch may not change while it runs (D0). Then the
            /// configuration with the fault status cleared, the POR thresholds (:659), and 00h..06h read
            /// back for setup(): no identity register, and a missing part floats SDO to all ones or zeros.
            static constexpr std::array Init{
              Step::write({.reg = Max31865Detail::RegConfiguration, .payload = {Stopped}}
              ),
              Step::write({.reg     = Max31865Detail::RegConfiguration,
                           .payload = {static_cast<std::uint8_t>(Configuration
                                                                 | Max31865Detail::FaultClear)}}
              ),
              Step::write({.reg     = Max31865Detail::RegHighThreshold,
                           .payload = {Max31865Detail::ThresholdsPor[0],
                                       Max31865Detail::ThresholdsPor[1],
                                       Max31865Detail::ThresholdsPor[2],
                                       Max31865Detail::ThresholdsPor[3]}}
              ),
              Step::read({.reg = Max31865Detail::RegConfiguration, .count = 7, .offset = 0}
              ),
            };

            struct State {};

            [[nodiscard]] static constexpr bool registersRight(Bytes data) {
                if((data.u8(0) & Max31865Detail::SettingBits)
                   != (Configuration & Max31865Detail::SettingBits))
                {
                    return false;
                }
                for(std::size_t i = 0; i < Max31865Detail::ThresholdsPor.size(); ++i) {
                    if(data.u8(Max31865Detail::RegHighThreshold + i)
                       != Max31865Detail::ThresholdsPor[i])
                    {
                        return false;
                    }
                }
                return true;
            }

            [[nodiscard]] static constexpr bool setup(Bytes data,
                                                      State&) {
                return registersRight(data);
            }

            /// The RTD word; only when its bit 0 flags a fault (MAX31865.md:565) the fault status and a
            /// status clear follow. Reading 02h raises DRDY again.
            struct Conversion {
                static constexpr std::array Steps{
                  Step::read({.reg = Max31865Detail::RegRtd, .count = 2, .offset = 0}),
                  Step::stopUnless(),
                  Step::read({.reg = Max31865Detail::RegFaultStatus, .count = 1, .offset = 2}),
                  Step::write({.reg     = Max31865Detail::RegConfiguration,
                               .payload = {static_cast<std::uint8_t>(
                                 Configuration | Max31865Detail::FaultClear)}}),
                };

                struct Sample {
                    std::uint16_t word{};    ///< 01h..02h as read, fault flag in bit 0
                    std::uint8_t status{};   ///< 07h; 0 when the flag was clear and it was not read
                };

                [[nodiscard]] static constexpr bool ready(Bytes data) {
                    return (data.u8(1) & 0x01U) != 0;
                }

                [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
                    return Outcome<Sample>::ok(Sample{
                      .word   = data.be16(0),
                      .status = static_cast<std::uint8_t>(data.size() > 2 ? data.u8(2) : 0U)});
                }
            };

            /// Read back every RegisterCheck to catch a part that lost them (brown-out). Reading 01h/02h
            /// raises DRDY (MAX31865.md:301, :712), so one conversion goes unread.
            struct Registers {
                static constexpr auto       Period = std::chrono::milliseconds{RegisterCheckMs};
                static constexpr std::array Steps{
                  Step::read({.reg = Max31865Detail::RegConfiguration, .count = 7, .offset = 0})};

                struct Sample {
                    std::uint8_t configuration{};
                };

                [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
                    if(!registersRight(data)) { return Outcome<Sample>::reject(); }
                    return Outcome<Sample>::ok(Sample{.configuration = data.u8(0)});
                }
            };

            using Primary = Conversion;
            using Reads   = std::
              conditional_t<RegisterCheckMs != 0, List<Conversion, Registers>, List<Conversion>>;
        };
    }   // namespace Chips

    /// `Nominal` is R0, `Reference` the board's reference resistor (optimum 4 x R0, "Application
    /// Circuits"); the defaults are water_mix's PT500 against 1 kOhm.
    ///
    /// Callendar-Van Dusen for T >= 0 degC (IEC 60751) solved for T and used over the whole range;
    /// the C term left out is ~0.2 degC at -100 degC. A and B scaled by 1e10.
    ///
    /// No fault-detection cycle: the flag reports over/under-voltage and the thresholds only.
    template<typename Master,
             typename Clock,
             typename Cs,
             typename Drdy,
             Units::Ohm Nominal   = Units::ohm(500),
             Units::Ohm Reference = Units::ohm(1000),
             typename Config      = Max31865Defaults>
    class Max31865 {
    public:
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
        static constexpr std::uint16_t RejectedRestart = [] {
            if constexpr(requires { Config::RejectedRestart; }) {
                return static_cast<std::uint16_t>(Config::RejectedRestart);
            } else {
                return Max31865Defaults::RejectedRestart;
            }
        }();
        static constexpr std::chrono::milliseconds RegisterCheck = [] {
            if constexpr(requires { Config::RegisterCheck; }) {
                return Kvasir::asDuration(Config::RegisterCheck);
            } else {
                return std::chrono::milliseconds{Max31865Defaults::RegisterCheck};
            }
        }();
        static constexpr std::uint8_t Configuration = [] {
            if constexpr(requires { Config::Configuration; }) {
                return static_cast<std::uint8_t>(Config::Configuration);
            } else {
                return Max31865Defaults::Configuration;
            }
        }();

        struct EngineConfig : Config {
            static constexpr auto UnidentifiedRetry = [] {
                if constexpr(requires { Config::UnidentifiedRetry; }) {
                    return Kvasir::asDuration(Config::UnidentifiedRetry);
                } else {
                    return std::chrono::milliseconds{Max31865Defaults::UnidentifiedRetry};
                }
            }();
        };

        struct Chip : Chips::Max31865<Configuration, RegisterCheck.count()> {
            static constexpr auto StartupDelay = Max31865::StartupDelay;
        };

        using DeviceT   = Device<Master, Clock, Chip, Cs, EngineConfig>;
        using TimePoint = typename Clock::time_point;

        struct Sample {
            Units::MilliDegC temperature{};
            Units::MilliOhm  resistance{};
            std::uint16_t    code{};   ///< the 15-bit RTD code
        };

        Max31865() { apply(makeInput(Drdy{}), makeOutput(Cs{}), set(Cs{})); }

        Max31865(Max31865 const&)            = delete;   // the engine's callbacks point at it
        Max31865& operator=(Max31865 const&) = delete;

        void handler() {
            auto const now = Clock::now();
            device_.handler();
            if(!device_.answering()) {
                valid_         = false;
                conversionDue_ = now + ConversionTimeout;
                wasUp_         = false;
                return;
            }
            if(!wasUp_) {   // a bring-up just ended: its fault flags are cleared
                wasUp_         = true;
                fault_         = 0;
                rejected_      = 0;
                conversionDue_ = now + ConversionTimeout;
                if constexpr(RegisterCheck.count() != 0) {
                    seenRegisters_ = device_.template rejected<typename Chip::Registers>();
                }
            }
            unidentifiedWhenUp_ = device_.unidentified();
            if(device_.template fresh<typename Chip::Conversion>(seenConversion_)) {
                conversionDue_ = now + ConversionTimeout;
                conversion_(device_.template latest<typename Chip::Conversion>(), now);
                asked_ = false;
            }
            if constexpr(RegisterCheck.count() != 0) {
                auto const bad = device_.template rejected<typename Chip::Registers>();
                if(bad != seenRegisters_) {
                    seenRegisters_ = bad;
                    UC_LOG_W("max31865: registers do not read back -- bringing the part up again");
                    restart_();
                    return;
                }
            }
            if(!asked_ && !apply(read(Drdy{}))) {
                ticket_ = device_.template request<typename Chip::Conversion>();
                asked_  = true;
            } else if(asked_
                      && device_.template answer<typename Chip::Conversion>(ticket_)
                           != I2C::Answer::pending)
            {
                asked_ = false;   // failed or rejected: DRDY decides again next turn
            }
            if(now >= conversionDue_) {
                UC_LOG_W("max31865: no conversion within {}", ConversionTimeout);
                restart_();
            }
        }

        [[nodiscard]] bool valid() const { return device_.answering() && valid_; }

        [[nodiscard]] Sample const& latest() const { return sample_; }

        [[nodiscard]] std::uint32_t seq() const { return samples_; }

        [[nodiscard]] std::uint32_t samples() const { return samples_; }

        [[nodiscard]] bool fresh(std::uint32_t& seen) const {
            if(seen == samples_) { return false; }
            seen = samples_;
            return true;
        }

        [[nodiscard]] std::optional<std::uint16_t> code() const {
            if(!valid()) { return std::nullopt; }
            return sample_.code;
        }

        [[nodiscard]] std::optional<Units::MilliOhm> resistance() const {
            if(!valid()) { return std::nullopt; }
            return sample_.resistance;
        }

        /// Empty while not valid(): an open or shorted RTD is a rejection.
        [[nodiscard]] std::optional<Units::MilliDegC> temperature() const {
            if(!valid()) { return std::nullopt; }
            return sample_.temperature;
        }

        /// As read with the last flagged conversion; 0 since the last bring-up otherwise.
        [[nodiscard]] std::uint8_t fault() const { return fault_; }

        [[nodiscard]] std::uint32_t faults() const { return faults_; }

        [[nodiscard]] std::uint16_t rejectedInRow() const { return rejected_; }

        [[nodiscard]] Link link() const { return device_.link(); }

        [[nodiscard]] bool answering() const { return device_.answering(); }

        /// A bring-up ran since it last answered and the registers did not read back (no NAK on SPI).
        [[nodiscard]] bool absent() const {
            return !device_.answering() && device_.unidentified() != unidentifiedWhenUp_;
        }

        [[nodiscard]] std::uint32_t bringUps() const { return device_.bringUps(); }

        [[nodiscard]] std::uint32_t errors() const { return device_.errors(); }

        DeviceT& device() { return device_; }

        DeviceT const& device() const { return device_; }

        [[nodiscard]] static constexpr Units::MilliOhm resistanceFor(std::uint16_t code) {
            return Units::milliOhm(std::uint64_t{code} * Units::value(Reference) * 1000U / 32768U);
        }

        /// At or below R(850 degC), the top of the Callendar-Van Dusen range.
        [[nodiscard]] static constexpr bool codeInRange(std::uint16_t code) {
            auto const topK = K + Ak * 850 + Bk * 850 * 850;
            return static_cast<std::int64_t>(code)
                   * static_cast<std::int64_t>(Units::value(Reference)) * K
                <= topK * static_cast<std::int64_t>(Units::value(Nominal)) * 32768;
        }

        /// Clamped to 850 degC where the equation has no solution.
        [[nodiscard]] static constexpr Units::MilliDegC temperatureFor(std::uint16_t code) {
            auto const disc = discriminant_(code);
            if(disc < 0) { return Units::milliDegC(850'000); }
            auto const root
              = static_cast<std::int64_t>(Max31865Detail::isqrt(static_cast<std::uint64_t>(disc)));
            return Units::milliDegC((-Ak + root) * 1000 / (2 * Bk));
        }

    private:
        static constexpr std::int64_t K  = 10'000'000'000;   // A and B are scaled by this
        static constexpr std::int64_t Ak = 39'083'000;       // 3.9083e-3
        static constexpr std::int64_t Bk = -5'775;           // -5.775e-7

        [[nodiscard]] static constexpr std::int64_t discriminant_(std::uint16_t code) {
            std::int64_t const r0  = Units::value(Nominal);
            std::int64_t const rr  = Units::value(Reference);
            auto const         rel = K * (std::int64_t{code} * rr - 32768 * r0) / (32768 * r0);
            return Ak * Ak + 4 * Bk * rel;
        }

        void conversion_(typename Chip::Conversion::Sample const& s,
                         TimePoint) {
            auto const code = static_cast<std::uint16_t>(s.word >> 1U);
            if((s.word & 0x0001U) != 0 || !codeInRange(code)) {
                ++faults_;
                valid_ = false;
                if((s.word & 0x0001U) != 0) {
                    if(fault_ != s.status) {
                        UC_LOG_W("max31865: RTD fault, status {:#04x}", s.status);
                    }
                    fault_ = s.status;
                }
                if(rejected_ != 0xFFFF) { ++rejected_; }
                if(RejectedRestart != 0 && rejected_ >= RejectedRestart) {
                    UC_LOG_W(
                      "max31865: {} conversions in a row rejected (code {}, fault status {:#04x}): "
                      "bringing the part up again",
                      rejected_,
                      code,
                      fault_);
                    restart_();
                }
                return;
            }
            sample_   = {temperatureFor(code), resistanceFor(code), code};
            valid_    = true;
            rejected_ = 0;
            ++samples_;
        }

        void restart_() {
            valid_ = false;
            asked_ = false;
            wasUp_ = false;
            device_.restart();
        }

        DeviceT       device_{};
        Sample        sample_{};
        bool          valid_{};
        bool          wasUp_{};
        bool          asked_{};
        I2C::Ticket   ticket_{};
        TimePoint     conversionDue_{};
        std::uint8_t  fault_{};
        std::uint32_t faults_{};
        std::uint32_t samples_{};
        std::uint32_t seenConversion_{};
        std::uint32_t seenRegisters_{};
        std::uint32_t unidentifiedWhenUp_{};   ///< unidentified() when the part last answered
        std::uint16_t rejected_{};
    };

}}   // namespace Kvasir::SPI
