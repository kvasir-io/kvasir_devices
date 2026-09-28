#pragma once
// Maxim MAX7219 / MAX7221 LED driver (MAX7219.md): a description with no registers to read -
// every frame is one 16-bit (address, data) word per chip of the cascade - and the driver.
#include "../../Duration.hpp"
#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir { namespace SPI {

    struct Max7219Defaults {
        static constexpr std::uint8_t Intensity = 8;
        static constexpr std::uint8_t ScanLimit = 7;
        /// A write-only part says nothing of a brown-out, which resets it ("Initial Power-Up").
        static constexpr auto RefreshPeriod = std::chrono::milliseconds{1000};
    };

    namespace Chips {
        namespace Max7219Detail {
            // Table 2, Register Address Map (MAX7219.md:252)
            inline constexpr std::uint8_t RegDigit0     = 0x01;
            inline constexpr std::uint8_t RegDecodeMode = 0x09;
            inline constexpr std::uint8_t RegIntensity  = 0x0A;
            inline constexpr std::uint8_t RegScanLimit  = 0x0B;
            inline constexpr std::uint8_t RegShutdown   = 0x0C;
            inline constexpr std::uint8_t RegTest       = 0x0F;
        }   // namespace Max7219Detail

        /// A frame is one word per chip, the last chip's first, latched on LOAD's rising edge after
        /// the 16th clock edge (MAX7219.md:236).
        template<std::size_t                    Modules,
                 std::uint8_t                   Decode,
                 std::uint8_t                   ScanLimit,
                 std::uint8_t                   Intensity,
                 std::chrono::milliseconds::rep RefreshMs>
        struct Max7219 {
            static_assert(Modules >= 1 && Modules <= 8,
                          "1..8 cascaded chips");
            static constexpr std::string_view Name = "MAX7219";
            /// Mode 0 (:189, :194), 10 MHz max (:194), tCP 100 ns (:139).
            static constexpr ClockMode    Mode          = ClockMode::_0;
            static constexpr Units::Hertz MaxClock      = Units::hertz(10'000'000);
            static constexpr std::size_t  RegisterBytes = 0;
            static constexpr std::size_t  Frame         = 2 * Modules;
            /// No DOUT: present() means the frames went out.
            static constexpr bool WriteOnly = true;

            static constexpr void word(std::span<std::byte> out,
                                       std::uint8_t         reg,
                                       std::uint8_t         value) {
                for(std::size_t m = 0; m < Modules; ++m) {
                    out[2 * m]     = std::byte{reg};
                    out[2 * m + 1] = std::byte{value};
                }
            }

            /// Shutdown while the configuration and the digits go in, so stale digits are never shown
            /// (where the frame fits an inline payload).
            static constexpr auto Init = [] {
                if constexpr(Frame <= 8) {
                    std::array<std::uint8_t, Frame> w{};
                    for(std::size_t m = 0; m < Modules; ++m) {
                        w[2 * m] = Max7219Detail::RegShutdown;
                    }
                    Step s  = Step::command({.payload = {}});
                    s.count = static_cast<std::uint8_t>(Frame);
                    for(std::size_t i = 0; i < Frame; ++i) { s.bytes[i] = w[i]; }
                    return std::array{s};
                } else {
                    return std::array<Step, 0>{};
                }
            }();

            struct State {};

            static constexpr auto Refresh = std::chrono::milliseconds{RefreshMs};

            /// The control registers a brown-out resets, rewritten every Refresh.
            struct Control {
                using Value                          = std::uint8_t;
                static constexpr std::size_t Bytes   = 3 * Frame;
                static constexpr Value       Initial = 0;
                static constexpr auto        Period  = Refresh;

                [[nodiscard]] static constexpr std::array<Step,
                                                          3>
                encode(Value const&,
                       std::span<std::byte> buffer) {
                    word(buffer.subspan(0, Frame), Max7219Detail::RegTest, 0);
                    word(buffer.subspan(Frame, Frame), Max7219Detail::RegScanLimit, ScanLimit);
                    word(buffer.subspan(2 * Frame, Frame), Max7219Detail::RegDecodeMode, Decode);
                    return {Step::commandBuffer({.offset = 0, .count = Frame}),
                            Step::commandBuffer({.offset = Frame, .count = Frame}),
                            Step::commandBuffer({.offset = 2 * Frame, .count = Frame})};
                }
            };

            struct Digits {
                using Value                          = std::array<std::uint8_t, Modules>;
                static constexpr std::size_t Items   = 8;
                static constexpr std::size_t Bytes   = Frame;
                static constexpr Value       Initial = {};
                static constexpr auto        Period  = Refresh;

                [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                           std::size_t          digit,
                                                           std::span<std::byte> buffer) {
                    for(std::size_t m = 0; m < Modules; ++m) {
                        // the last chip in the chain gets the first word
                        buffer[2 * m]
                          = std::byte{static_cast<std::uint8_t>(Max7219Detail::RegDigit0 + digit)};
                        buffer[2 * m + 1] = std::byte{value[Modules - 1 - m]};
                    }
                    return Step::commandBuffer({.offset = 0, .count = Frame});
                }
            };

            struct IntensityGroup {
                using Value                          = std::uint8_t;
                static constexpr std::size_t Bytes   = Frame;
                static constexpr Value       Initial = Intensity;
                static constexpr auto        Period  = Refresh;

                [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
                    word(buffer,
                         Max7219Detail::RegIntensity,
                         static_cast<std::uint8_t>(value & 0x0FU));
                    return Step::commandBuffer({.offset = 0, .count = Frame});
                }
            };

            struct Power {
                using Value                          = std::uint8_t;
                static constexpr std::size_t Bytes   = Frame;
                static constexpr Value       Initial = 1;
                static constexpr auto        Period  = Refresh;

                [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
                    word(buffer, Max7219Detail::RegShutdown, value);
                    return Step::commandBuffer({.offset = 0, .count = Frame});
                }
            };

            using Writes = List<Control, Digits, IntensityGroup, Power>;
        };
    }   // namespace Chips

    /// `Decode`: a bit per digit (0xFF: BCD 7-segment, 0: a matrix).
    ///
    ///   using Display = Kvasir::SPI::Max7219<Spi, Clock, HW::Pin::spi_cs, 1, 0xFF>;
    ///   display.setDigit(0, 3, 7);        // module 0, digit 3 shows "7" (BCD decode)
    ///   display.setRow(0, 2, 0b10101010); // module 0, row 2 of a matrix (no decode)
    ///   display.setIntensity(8);          // 0..15
    ///
    template<typename Master,
             typename Clock,
             typename Cs,
             std::size_t  Modules = 1,
             std::uint8_t Decode  = 0xFF,
             typename Config      = Max7219Defaults>
    class Max7219 {
    public:
        static constexpr std::string_view Name         = "MAX7219";
        static constexpr std::size_t      DigitCount   = 8;
        static constexpr std::uint8_t     Dash         = 0x0A;   ///< BCD decode (Table 5)
        static constexpr std::uint8_t     Blank        = 0x0F;
        static constexpr std::uint8_t     Point        = 0x80;
        static constexpr std::uint8_t     MaxIntensity = 15;

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

        using Chip
          = Chips::Max7219<Modules, Decode, ScanLimit, IntensityDefault, RefreshPeriod.count()>;
        using DeviceT = Device<Master, Clock, Chip, Cs, Config>;

        Max7219() { clear(); }

        Max7219(Max7219 const&)            = delete;
        Max7219& operator=(Max7219 const&) = delete;

        void handler() { device_.handler(); }

        /// Digit `digit` of module `module`: a BCD code with Decode, segments without.
        void setDigit(std::size_t  module,
                      std::size_t  digit,
                      std::uint8_t value) {
            if(module >= Modules || digit >= DigitCount) { return; }
            if(shadow_[digit][module] == value) { return; }
            shadow_[digit][module] = value;
            device_.template set<typename Chip::Digits>(digit, shadow_[digit]);
        }

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
            intensity_ = static_cast<std::uint8_t>(level & MaxIntensity);
            device_.template set<typename Chip::IntensityGroup>(intensity_);
        }

        [[nodiscard]] std::uint8_t intensity() const { return intensity_; }

        void clear() {
            for(std::size_t d = 0; d < DigitCount; ++d) {
                for(auto& v : shadow_[d]) { v = blankFor(d); }
                device_.template set<typename Chip::Digits>(d, shadow_[d]);
            }
        }

        [[nodiscard]] static constexpr std::uint8_t blankFor(std::size_t digit) {
            return ((Decode >> digit) & 1U) != 0 ? Blank : std::uint8_t{0};
        }

        [[nodiscard]] bool pending() const { return device_.pending(); }

        [[nodiscard]] bool answering() const { return device_.answering(); }

        [[nodiscard]] bool present() const { return device_.answering(); }

        [[nodiscard]] Link link() const { return device_.link(); }

        [[nodiscard]] std::uint32_t errors() const { return device_.errors(); }

        DeviceT& device() { return device_; }

        DeviceT const& device() const { return device_; }

    private:
        DeviceT                                                   device_{};
        std::array<std::array<std::uint8_t, Modules>, DigitCount> shadow_{};
        std::uint8_t                                              intensity_{IntensityDefault};
    };

}}   // namespace Kvasir::SPI
