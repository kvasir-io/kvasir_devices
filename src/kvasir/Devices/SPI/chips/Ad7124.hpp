#pragma once
// Analog Devices AD7124-4 / AD7124-8 (AD7124-4.md, AD7124-8.md): a description built from the
// configuration, and the driver with the conversion API.
#include "../../Bytes.hpp"
#include "../../Duration.hpp"
#include "../../Log.hpp"
#include "../../Quantities.hpp"
#include "../Device.hpp"

#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

namespace Kvasir { namespace SPI {

    /// Register map and field encodings (datasheet, "On-Chip Registers").
    namespace Ad7124Detail {
        /// The communications register byte is the address, with bit 6 set for a read.
        enum class Reg : std::uint8_t {
            status     = 0x00,   // 8 bit
            adcControl = 0x01,   // 16 bit
            data       = 0x02,   // 24 bit (+ status with DATA_STATUS)
            ioControl1 = 0x03,   // 24 bit
            id         = 0x05,   // 8 bit
            error      = 0x06,   // 24 bit
            errorEn    = 0x07,   // 24 bit
            channel0   = 0x09,   // 16 bit, CHANNEL_0..15
            config0    = 0x19,   // 16 bit, CONFIG_0..7
            filter0    = 0x21,   // 24 bit, FILTER_0..7
            offset0    = 0x29,   // 24 bit, OFFSET_0..7
        };

        [[nodiscard]] constexpr std::uint8_t address(Reg         reg,
                                                     std::size_t offset = 0) {
            return static_cast<std::uint8_t>(std::to_underlying(reg) + offset);
        }

        inline constexpr std::uint8_t ReadBit = 0x40;

        // STATUS register bits.
        inline constexpr std::uint8_t NotReady    = 0x80;   // RDY: 0 once a conversion is ready
        inline constexpr std::uint8_t ErrorFlag   = 0x40;
        inline constexpr std::uint8_t PorFlag     = 0x10;   // cleared by reading STATUS
        inline constexpr std::uint8_t ChannelMask = 0x0F;   // CH_ACTIVE: the channel just converted

        // ERROR_EN / ERROR register bits (the two share a layout).
        inline constexpr std::uint32_t SpiCrcErr    = 0x000004;   // SPI_CRC_ERR(_EN), bit 2
        inline constexpr std::uint32_t SpiIgnoreErr = 0x000040;   // SPI_IGNORE_ERR(_EN), bit 6
        inline constexpr std::uint32_t RefDetErr    = 0x000800;   // REF_DET_ERR(_EN), bit 11

    }   // namespace Ad7124Detail

    /// ERROR_EN.SPI_CRC_ERR_EN: whether every frame carries a CRC.
    enum class Ad7124Crc : std::uint8_t { off, on };

    /// CONFIG_x BIPOLAR: offset binary around mid-scale, or unipolar from zero.
    enum class Ad7124Polarity : std::uint8_t { unipolar, bipolar };

    /// CONFIG_x REF_BUFP/REF_BUFM and AIN_BUFP/AIN_BUFM, set as pairs.
    enum class Ad7124Buffers : std::uint8_t { off, on };

    /// FILTER_x REJ60: a first notch at 60 Hz next to the 50 Hz one.
    enum class Ad7124Reject60Hz : std::uint8_t { off, on };

    /// IO_CONTROL_1 PDSW: the low-side power switch.
    enum class Ad7124PowerSwitch : std::uint8_t { open, closed };

    namespace Ad7124Detail {
        /// The reset default (SPI_IGNORE_ERR_EN) plus the reference detect and, with `crc`, the SPI CRC.
        [[nodiscard]] constexpr std::uint32_t errorEnValue(Ad7124Crc crc) {
            return SpiIgnoreErr | RefDetErr | (crc == Ad7124Crc::on ? SpiCrcErr : 0U);
        }

        /// Datasheet "CRC Checksum": CRC-8/0x07, init 0, no reflection, over the communications byte
        /// and the data bytes.
        [[nodiscard]] constexpr std::uint8_t crc(std::span<std::byte const> frame) {
            return Kvasir::crc8<0x07U>(Bytes{frame}, 0);
        }

        // The CRC-8/0x07 check value: "123456789" -> 0xF4.
        static_assert(crc(std::array{std::byte{'1'},
                                     std::byte{'2'},
                                     std::byte{'3'},
                                     std::byte{'4'},
                                     std::byte{'5'},
                                     std::byte{'6'},
                                     std::byte{'7'},
                                     std::byte{'8'},
                                     std::byte{'9'}})
                      == 0xF4);
    }   // namespace Ad7124Detail

    /// PGA gain (CONFIG_x PGA[2:0]).
    enum class Ad7124Gain : std::uint8_t { x1, x2, x4, x8, x16, x32, x64, x128 };

    /// Reference source (CONFIG_x REF_SEL[1:0]).
    enum class Ad7124Reference : std::uint8_t { refin1 = 0, refin2 = 1, internal = 2, avdd = 3 };

    /// Digital filter (FILTER_x FILTER[2:0]).
    enum class Ad7124Filter : std::uint8_t {
        sinc4      = 0,
        sinc3      = 2,
        fastSinc4  = 4,
        fastSinc3  = 5,
        postFilter = 7,
    };

    /// The post filter when `Ad7124Filter::postFilter` is selected (FILTER_x POST_FILTER[2:0]).
    enum class Ad7124PostFilter : std::uint8_t {
        none     = 0,
        sps27    = 2,   // 27.27 SPS, 47 dB rejection of 50/60 Hz
        sps25    = 3,   // 25 SPS, 62 dB
        sps20    = 5,   // 20 SPS, 86 dB
        sps16_67 = 6,   // 16.67 SPS, 92 dB
    };

    /// ADC_CONTROL POWER_MODE[1:0].
    enum class Ad7124Power : std::uint8_t { low = 0, mid = 1, full = 2 };

    enum class Ad7124Model : std::uint8_t { any, ad7124_4, ad7124_8 };

    /// 0..15 are AIN0..AIN15 (AIN0..AIN7 on the -4); internal: temperature 16, AVSS 17, reference
    /// 18, DGND 19.
    struct Ad7124Channel {
        std::uint8_t positive{};
        std::uint8_t negative{};
    };

    namespace Ad7124Detail {
        [[nodiscard]] constexpr std::uint16_t channelValue(Ad7124Channel c) {
            // Enable, setup 0, AINP[9:5], AINM[4:0].
            return static_cast<std::uint16_t>(0x8000U | ((c.positive & 0x1FU) << 5U)
                                              | (c.negative & 0x1FU));
        }

        [[nodiscard]] constexpr std::uint16_t configValue(Ad7124Polarity  polarity,
                                                          Ad7124Buffers   referenceBuffers,
                                                          Ad7124Buffers   inputBuffers,
                                                          Ad7124Reference reference,
                                                          Ad7124Gain      gain) {
            return static_cast<std::uint16_t>(
              (polarity == Ad7124Polarity::bipolar ? 0x0800U : 0U)
              | (referenceBuffers == Ad7124Buffers::on ? 0x0180U : 0U)
              | (inputBuffers == Ad7124Buffers::on ? 0x0060U : 0U)
              | (unsigned{std::to_underlying(reference)} << 3U)
              | unsigned{std::to_underlying(gain)});
        }

        [[nodiscard]] constexpr std::uint32_t filterValue(Ad7124Filter     filter,
                                                          Ad7124Reject60Hz reject60Hz,
                                                          Ad7124PostFilter post,
                                                          std::uint16_t    fs) {
            return (std::uint32_t{std::to_underlying(filter)} << 21U)
                 | (reject60Hz == Ad7124Reject60Hz::on ? 1UL << 20U : 0UL)
                 | (std::uint32_t{std::to_underlying(post)} << 17U) | (fs & 0x7FFU);
        }

        /// Continuous conversion, internal clock, DATA_STATUS (bit 10) so a conversion carries its
        /// channel; REF_EN (bit 8) when the setup selects the internal reference.
        [[nodiscard]] constexpr std::uint16_t adcControlValue(Ad7124Power     power,
                                                              Ad7124Reference reference) {
            return static_cast<std::uint16_t>(
              0x0400U | (reference == Ad7124Reference::internal ? 0x0100U : 0U)
              | (unsigned{std::to_underlying(power)} << 6U));
        }

        [[nodiscard]] constexpr std::uint32_t ioControl1Value(Ad7124PowerSwitch powerSwitch) {
            return powerSwitch == Ad7124PowerSwitch::closed ? 0x008000UL : 0UL;
        }

        [[nodiscard]] constexpr std::int32_t toSigned(std::uint32_t  code,
                                                      Ad7124Polarity polarity) {
            return polarity == Ad7124Polarity::bipolar ? static_cast<std::int32_t>(code) - 0x800000
                                                       : static_cast<std::int32_t>(code);
        }

        /// Internal zero-scale (0101) and full-scale (0110) calibration; mid power at most for full-scale.
        enum class CalibrationMode : std::uint8_t {
            internalZeroScale = 0x5,
            internalFullScale = 0x6
        };

        [[nodiscard]] constexpr std::uint16_t adcControlCalibration(Ad7124Power     power,
                                                                    Ad7124Reference reference,
                                                                    CalibrationMode mode) {
            return static_cast<std::uint16_t>(
              adcControlValue(power == Ad7124Power::full ? Ad7124Power::mid : power, reference)
              | (unsigned{std::to_underlying(mode)} << 2U));
        }

        [[nodiscard]] constexpr bool idMatches(Ad7124Model  model,
                                               std::uint8_t id) {
            auto const device = id >> 4U;
            // A silicon revision of 0 is what a floating or shorted MISO reads as, not a part.
            if((id & 0x0FU) == 0) { return false; }
            switch(model) {
            case Ad7124Model::any:      return device <= 1;
            case Ad7124Model::ad7124_4: return device == 0;
            case Ad7124Model::ad7124_8: return device == 1;
            }
            return false;
        }

        template<std::size_t N>
        [[nodiscard]] constexpr bool inputsValid(Ad7124Model          model,
                                                 std::array<Ad7124Channel,
                                                            N> const& channels) {
            auto const valid = [model](std::uint8_t in) {
                if(in > 31) { return false; }
                return model != Ad7124Model::ad7124_4 || in <= 7 || in >= 16;
            };
            for(auto const& c : channels) {
                if(!valid(c.positive) || !valid(c.negative)) { return false; }
            }
            return true;
        }

        struct Write {
            std::uint8_t  reg{};
            std::uint8_t  size{};   // bytes
            std::uint32_t value{};
        };

        // The encodings against an example configuration written out as literals.
        static_assert(channelValue({1,
                                    0})
                        == 0x8020
                      && channelValue({3,
                                       2})
                           == 0x8062
                      && channelValue({5,
                                       4})
                           == 0x80A4
                      && channelValue({7,
                                       6})
                           == 0x80E6);
        static_assert(configValue(Ad7124Polarity::bipolar,
                                  Ad7124Buffers::on,
                                  Ad7124Buffers::on,
                                  Ad7124Reference::refin1,
                                  Ad7124Gain::x8)
                      == 0x09E3);
        static_assert(filterValue(Ad7124Filter::postFilter,
                                  Ad7124Reject60Hz::off,
                                  Ad7124PostFilter::sps16_67,
                                  1)
                      == 0xEC0001);
        static_assert(adcControlValue(Ad7124Power::full,
                                      Ad7124Reference::refin1)
                        == 0x0480
                      && adcControlValue(Ad7124Power::full,
                                         Ad7124Reference::internal)
                           == 0x0580);
        static_assert(ioControl1Value(Ad7124PowerSwitch::closed) == 0x008000);
        static_assert(errorEnValue(Ad7124Crc::on) == 0x000844
                      && errorEnValue(Ad7124Crc::off) == 0x000840);
        static_assert(toSigned(0x800000,
                               Ad7124Polarity::bipolar)
                        == 0
                      && toSigned(0xFFFFFF,
                                  Ad7124Polarity::bipolar)
                           == 0x7FFFFF
                      && toSigned(0,
                                  Ad7124Polarity::bipolar)
                           == -0x800000
                      && toSigned(0x123456,
                                  Ad7124Polarity::unipolar)
                           == 0x123456);
        static_assert(idMatches(Ad7124Model::any,
                                0x04)
                      && idMatches(Ad7124Model::any,
                                   0x16)
                      && !idMatches(Ad7124Model::ad7124_4,
                                    0x14)
                      && !idMatches(Ad7124Model::any,
                                    0x00)
                      && !idMatches(Ad7124Model::any,
                                    0xFF));
        static_assert(inputsValid(Ad7124Model::ad7124_4,
                                  std::array<Ad7124Channel,
                                             1>{{{7,
                                                  16}}})
                      && !inputsValid(Ad7124Model::ad7124_4,
                                      std::array<Ad7124Channel,
                                                 1>{{{8,
                                                      0}}}));
    }   // namespace Ad7124Detail

    /// The knobs a config may leave out (the part's reset values where it has one). Every channel
    /// uses setup 0. The engine's knobs (I2C/Device.hpp EngineDefaults) may be set here too.
    struct Ad7124Defaults {
        static constexpr Ad7124Model                  Model = Ad7124Model::any;
        static constexpr std::array<Ad7124Channel, 1> Channels{{{0, 1}}};
        static constexpr Ad7124Polarity               Polarity         = Ad7124Polarity::bipolar;
        static constexpr Ad7124Gain                   Gain             = Ad7124Gain::x1;
        static constexpr Ad7124Reference              Reference        = Ad7124Reference::refin1;
        static constexpr Ad7124Buffers                ReferenceBuffers = Ad7124Buffers::off;
        static constexpr Ad7124Buffers                InputBuffers     = Ad7124Buffers::on;
        static constexpr Ad7124Filter                 Filter           = Ad7124Filter::sinc4;
        static constexpr Ad7124PostFilter             PostFilter       = Ad7124PostFilter::none;
        static constexpr Ad7124Reject60Hz             Reject60Hz       = Ad7124Reject60Hz::off;
        static constexpr std::uint16_t                FilterSelect     = 384;   // FS[10:0]
        static constexpr Ad7124Power                  Power            = Ad7124Power::low;
        static constexpr Ad7124PowerSwitch            PowerSwitch      = Ad7124PowerSwitch::open;
        /// The part holds only the latest result: a faster channel is not seen at every conversion.
        static constexpr std::chrono::milliseconds PollPeriod{5};
        static constexpr std::chrono::milliseconds StartupDelay{0};
        /// After the serial reset: 90 MCLK cycles (AD7124-4.md:2714).
        static constexpr std::chrono::milliseconds ResetSettle{5};
        static constexpr std::chrono::milliseconds UnidentifiedRetry{1000};
    };

    namespace Ad7124Detail {
        [[nodiscard]] constexpr std::uint32_t masterClockHz(Ad7124Power p) {
            switch(p) {
            case Ad7124Power::full: return 614'400;
            case Ad7124Power::mid:  return 153'600;
            case Ad7124Power::low:  return 76'800;
            }
            return 76'800;
        }

        /// Rounded up at the internal clock's slowest (+-5 %, AD7124-4.md:2571: x 20/19): sinc4 (4 x 32 x
        /// FS + dead time) / fCLK (:2850), sinc3 3 x 32 (:3026), dead time 95 (61 at FS = 1, :2882); fast
        /// settling adds 16 (8 in low power) sinc periods (:3160ff); post filters at most 62.995 ms (Table 62).
        [[nodiscard]] constexpr std::chrono::milliseconds settling(Ad7124Filter  f,
                                                                   std::uint16_t fs,
                                                                   Ad7124Power   p) {
            std::uint64_t const dead = fs == 1 ? 61U : 95U;
            std::uint64_t const avg  = p == Ad7124Power::low ? 8U : 16U;
            std::uint64_t       periods{};
            switch(f) {
            case Ad7124Filter::sinc4:      periods = 4; break;
            case Ad7124Filter::sinc3:      periods = 3; break;
            case Ad7124Filter::fastSinc4:  periods = 4 + avg - 1; break;
            case Ad7124Filter::fastSinc3:  periods = 3 + avg - 1; break;
            case Ad7124Filter::postFilter: return std::chrono::milliseconds{67};
            }
            auto const clocks = periods * 32U * fs + dead;
            auto const slow   = masterClockHz(p) * 19U;   // 0.95 fCLK, in twentieths
            auto const us     = (clocks * 20'000'000U + slow - 1U) / slow;
            return std::chrono::milliseconds{static_cast<std::int64_t>((us + 999U) / 1000U + 1U)};
        }

        static_assert(settling(Ad7124Filter::sinc4,
                               384,
                               Ad7124Power::full)
                        == std::chrono::milliseconds{86},
                      "80.15 ms (Table: full power, FS 384) / 0.95, rounded up");
        static_assert(settling(Ad7124Filter::sinc4,
                               384,
                               Ad7124Power::mid)
                        == std::chrono::milliseconds{339},
                      "320.6 ms at 153.6 kHz / 0.95 = 337.5 ms, rounded up");

        [[nodiscard]] constexpr I2C::Step writeStep(std::uint8_t              reg,
                                                    std::uint8_t              size,
                                                    std::uint32_t             value,
                                                    std::chrono::milliseconds delay = {}) {
            I2C::Step s = I2C::Step::write({.reg = reg, .payload = {}, .delay = delay});
            s.count     = size;
            for(std::uint8_t i = 0; i < size; ++i) {
                s.bytes[i] = static_cast<std::uint8_t>(value >> (8U * (size - 1U - i)));
            }
            return s;
        }
    }   // namespace Ad7124Detail

    namespace Chips {
        template<typename Config>
        struct Ad7124 {
            using Reg = Ad7124Detail::Reg;

            static constexpr std::string_view Name     = "AD7124";
            static constexpr ClockMode        Mode     = ClockMode::_3;
            static constexpr Units::Hertz     MaxClock = Units::hertz(5'000'000);

            static constexpr std::uint8_t readCommand(std::uint8_t reg) {
                return static_cast<std::uint8_t>((reg & 0x3FU) | Ad7124Detail::ReadBit);
            }

            static constexpr std::uint8_t writeCommand(std::uint8_t reg) {
                return static_cast<std::uint8_t>(reg & 0x3FU);
            }

            static constexpr std::size_t RegisterBytes = 1;
            static constexpr std::size_t ChannelCount  = Config::Channels.size();
            static constexpr bool        Calibrates    = Config::Gain != Ad7124Gain::x1;
            static constexpr auto        StartupDelay  = Config::StartupDelay;

            /// Calibrations run in mid power at most (full-scale in full power is not supported); zero-scale
            /// takes one settling period (:4249), full-scale four (:2756). The engine runs no Step::check in
            /// Init, so each is waited out at the clock's slowest instead of polling RDY (:2754). Too short a
            /// wait makes the part ignore the next ADC_CONTROL write (SPI_IGNORE_ERR, :3662, :4440).
            static constexpr Ad7124Power CalibrationPower
              = Config::Power == Ad7124Power::full ? Ad7124Power::mid : Config::Power;
            static constexpr auto ZeroScaleWait
              = Ad7124Detail::settling(Config::Filter, Config::FilterSelect, CalibrationPower);
            static constexpr auto FullScaleWait = 4 * ZeroScaleWait;

            static constexpr std::size_t InitSteps
              = 4 + 4 + ChannelCount + (Calibrates ? 3 : 0) + 2;

            /// The serial reset first - 64 ones bring an out-of-step interface back (:2714), which answers
            /// nothing before, so the ID is read here rather than by the engine's Identity. STATUS is read to
            /// clear POR_FLAG (:4181); ERROR last, for setup() to see whether a write was ignored.
            static constexpr auto Init = [] {
                using namespace Ad7124Detail;
                std::array<I2C::Step, InitSteps> s{};
                std::size_t                      i = 0;
                s[i]       = I2C::Step::command({.payload = {}, .delay = Config::ResetSettle});
                s[i].count = 8;
                for(auto& b : s[i].bytes) { b = 0xFF; }
                ++i;
                s[i++] = I2C::Step::read({.reg = address(Reg::id), .count = 1, .offset = 0});
                s[i++] = I2C::Step::read({.reg = address(Reg::status), .count = 1, .offset = 1});
                s[i++] = I2C::Step::identify();
                s[i++] = writeStep(address(Reg::errorEn), 3, errorEnValue(Ad7124Crc::off));
                s[i++]
                  = writeStep(address(Reg::ioControl1), 3, ioControl1Value(Config::PowerSwitch));
                s[i++] = writeStep(address(Reg::config0),
                                   2,
                                   configValue(Config::Polarity,
                                               Config::ReferenceBuffers,
                                               Config::InputBuffers,
                                               Config::Reference,
                                               Config::Gain));
                s[i++] = writeStep(address(Reg::filter0),
                                   3,
                                   filterValue(Config::Filter,
                                               Config::Reject60Hz,
                                               Config::PostFilter,
                                               Config::FilterSelect));
                for(std::size_t c = 0; c < ChannelCount; ++c) {
                    s[i++]
                      = writeStep(address(Reg::channel0, c), 2, channelValue(Config::Channels[c]));
                }
                if constexpr(Calibrates) {
                    s[i++] = writeStep(address(Reg::offset0), 3, 0x800000);
                    s[i++] = writeStep(address(Reg::adcControl),
                                       2,
                                       adcControlCalibration(Config::Power,
                                                             Config::Reference,
                                                             CalibrationMode::internalFullScale),
                                       FullScaleWait);
                    s[i++] = writeStep(address(Reg::adcControl),
                                       2,
                                       adcControlCalibration(Config::Power,
                                                             Config::Reference,
                                                             CalibrationMode::internalZeroScale),
                                       ZeroScaleWait);
                }
                s[i++] = writeStep(address(Reg::adcControl),
                                   2,
                                   adcControlValue(Config::Power, Config::Reference));
                s[i++] = I2C::Step::read({.reg = address(Reg::error), .count = 3, .offset = 2});
                return s;
            }();

            struct State {
                std::uint8_t id{};
            };

            /// DEVICE_ID[7:4] names the part (AD7124-4.md:2126 Table 39; AD7124-8.md:2220); the revision has
            /// changed between data sheet revisions (AD7124-4.md:253), so it only has to be non-zero, as in
            /// Linux's ad7124.c:990-1000. And no write ignored.
            [[nodiscard]] static constexpr bool setup(Bytes  data,
                                                      State& state) {
                auto const id = data.u8(0);
                state.id      = id;
                return Ad7124Detail::idMatches(Config::Model, id)
                    && (data.be24(2) & Ad7124Detail::SpiIgnoreErr) == 0;
            }

            struct Conversion {
                static constexpr auto       Period = Config::PollPeriod;
                static constexpr std::array Steps{
                  I2C::Step::read(
                    {.reg = Ad7124Detail::address(Reg::status), .count = 1, .offset = 0}),
                  I2C::Step::stopUnless(),
                  I2C::Step::read(
                    {.reg = Ad7124Detail::address(Reg::data), .count = 4, .offset = 1}),
                };

                struct Sample {
                    std::uint8_t  status{};    ///< STATUS as polled: RDY, ERROR_FLAG, POR_FLAG
                    std::uint32_t code{};      ///< the 24-bit code
                    std::uint8_t  channel{};   ///< CH_ACTIVE of the status appended to the data
                };

                [[nodiscard]] static constexpr bool ready(Bytes data) {
                    return (data.u8(0) & Ad7124Detail::NotReady) == 0
                        && (data.u8(0) & Ad7124Detail::PorFlag) == 0;
                }

                [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
                    auto const status = data.u8(0);
                    if((status & Ad7124Detail::PorFlag)
                       != 0) {   // the driver sees it and starts over
                        return Outcome<Sample>::ok(Sample{.status = status});
                    }
                    if((status & Ad7124Detail::NotReady) != 0) {
                        return Outcome<Sample>::unchanged();
                    }
                    return Outcome<Sample>::ok(Sample{.status  = status,
                                                      .code    = data.be24(1),
                                                      .channel = static_cast<std::uint8_t>(
                                                        data.u8(4) & Ad7124Detail::ChannelMask)});
                }
            };

            struct Errors {
                static constexpr std::array Steps{I2C::Step::read(
                  {.reg = Ad7124Detail::address(Reg::error), .count = 3, .offset = 0})};

                struct Sample {
                    std::uint32_t value{};
                };

                [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
                    return Outcome<Sample>::ok(Sample{.value = data.be24(0)});
                }
            };

            using Primary = Conversion;
            using Reads   = List<Conversion, Errors>;
        };
    }   // namespace Chips

    /// The driver:
    ///     Kvasir::SPI::Ad7124<Spi, Clock, HW::Pin::adc_cs, AdcConfig> adc{};
    ///     ...
    ///     Spi::handler(); adc.handler();
    ///     if(auto const c = adc.nextConversion()) { use(c->channel, c->code); }
    template<typename Spi, typename Clock, typename Cs, typename Config = Ad7124Defaults>
    class Ad7124 {
    public:
        static_assert(std::derived_from<Config,
                                        Ad7124Defaults>,
                      "derive the ADC config from Kvasir::SPI::Ad7124Defaults");
        static constexpr std::string_view Name         = "AD7124";
        static constexpr std::size_t      ChannelCount = Config::Channels.size();
        static_assert(ChannelCount >= 1 && ChannelCount <= 16,
                      "the AD7124 sequences 1 to 16 channels");
        static_assert(Ad7124Detail::inputsValid(Config::Model,
                                                Config::Channels),
                      "a channel names an input the part does not have (AD7124-4: AIN0..AIN7, "
                      "AD7124-8: AIN0..AIN15, "
                      "internal inputs from 16 to 31)");
        static_assert(Config::FilterSelect >= 1 && Config::FilterSelect <= 2047,
                      "FS[10:0] is 1..2047");

        using Chip    = Chips::Ad7124<Config>;
        using DeviceT = Device<Spi, Clock, Chip, Cs, Config>;

        /// The channel's index in `Config::Channels`; the code made signed for a bipolar setup.
        struct Conversion {
            std::uint8_t channel{};
            std::int32_t code{};
        };

        Ad7124()                         = default;
        Ad7124(Ad7124 const&)            = delete;
        Ad7124& operator=(Ad7124 const&) = delete;

        /// Code x VREF / (2^23 x gain) bipolar, / 2^24 unipolar (VREF 2.5 V internal).
        [[nodiscard]] static constexpr Units::MicroVolt toVoltage(std::int32_t     code,
                                                                  Units::MicroVolt reference
                                                                  = Units::microVolt(2'500'000)) {
            auto const gain = std::int64_t{1} << std::to_underlying(Config::Gain);
            auto const span = Config::Polarity == Ad7124Polarity::bipolar ? std::int64_t{1} << 23
                                                                          : std::int64_t{1} << 24;
            return Units::microVolt(std::int64_t{code} * Units::value(reference) / (span * gain));
        }

        [[nodiscard]] static constexpr Units::MilliDegC toTemperature(std::int32_t code) {
            return Units::milliDegC(std::int64_t{code} * 1000 / 13584 - 272'500);
        }

        void handler() {
            handler([](auto& d) { d.handler(); });
        }

        /// With `turn(device())` in place of the device's own turn: how a DeviceSet drives it
        /// together with the other parts of its port (DeviceSet.hpp), one engine for all of them.
        template<typename Turn>
        void handler(Turn&& turn) {
            turn(device_);
            if(!device_.answering()) {
                valid_ = false;
                return;
            }
            if(device_.template fresh<typename Chip::Conversion>(seenConversion_)) {
                auto const& s = device_.template latest<typename Chip::Conversion>();
                if((s.status & Ad7124Detail::PorFlag) != 0) {
                    UC_LOG_W("ad7124: the part reset itself, configuring it again");
                    valid_ = false;
                    device_.restart();
                    return;
                }
                if((s.status & Ad7124Detail::ErrorFlag) != 0) {
                    errorTicket_ = device_.template request<typename Chip::Errors>();
                }
                if(s.channel < ChannelCount) {
                    if(pending_) { ++missed_; }
                    pending_
                      = Conversion{s.channel, Ad7124Detail::toSigned(s.code, Config::Polarity)};
                    ++conversions_;
                    valid_ = true;
                }
            }
            if(device_.template fresh<typename Chip::Errors>(seenErrors_)) {
                errorRegister_ = device_.template latest<typename Chip::Errors>().value;
                ++errorReports_;
                UC_LOG_W("ad7124: ERROR register {:#08x}", errorRegister_);
            }
        }

        [[nodiscard]] std::optional<Conversion> nextConversion() {
            auto const c = pending_;
            pending_.reset();
            return c;
        }

        [[nodiscard]] bool valid() const { return device_.answering() && valid_; }

        [[nodiscard]] std::uint32_t conversions() const { return conversions_; }

        [[nodiscard]] std::uint32_t seq() const { return conversions_; }

        [[nodiscard]] std::uint32_t missed() const { return missed_; }

        [[nodiscard]] std::uint8_t id() const { return device_.state().id; }

        [[nodiscard]] std::uint32_t errorRegister() const { return errorRegister_; }

        [[nodiscard]] std::uint32_t errorReports() const { return errorReports_; }

        [[nodiscard]] bool present() const { return device_.answering(); }

        [[nodiscard]] bool answering() const { return device_.answering(); }

        [[nodiscard]] Link link() const { return device_.link(); }

        [[nodiscard]] std::uint32_t bringUps() const { return device_.bringUps(); }

        [[nodiscard]] std::uint32_t errors() const { return device_.errors(); }

        DeviceT& device() { return device_; }

        DeviceT const& device() const { return device_; }

    private:
        DeviceT                   device_{};
        std::optional<Conversion> pending_{};
        bool                      valid_{};
        std::uint32_t             conversions_{};
        std::uint32_t             missed_{};
        std::uint32_t             errorRegister_{};
        std::uint32_t             errorReports_{};
        std::uint32_t             seenConversion_{};
        std::uint32_t             seenErrors_{};
        I2C::Ticket               errorTicket_{};
    };

}}   // namespace Kvasir::SPI
