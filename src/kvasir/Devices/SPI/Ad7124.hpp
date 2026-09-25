#pragma once

#include "../Bytes.hpp"
#include "../Log.hpp"
#include "../Quantities.hpp"
#include "../SPIDeviceBase.hpp"

#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

namespace Kvasir::SPI {

/// The register map and field encodings of the AD7124-4 / AD7124-8 (datasheet, "On-Chip
/// Registers"), as constexpr functions the driver builds its configuration writes from.
namespace Ad7124Detail {
    /// Register addresses. The communications register byte is the address, with bit 6 set
    /// for a read.
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
    /// ERROR_EN as written at bring-up: the reset default (SPI_IGNORE_ERR_EN) plus the
    /// reference detect and, with `crc` on, the SPI CRC.
    [[nodiscard]] constexpr std::uint32_t errorEnValue(Ad7124Crc crc) {
        return SpiIgnoreErr | RefDetErr | (crc == Ad7124Crc::on ? SpiCrcErr : 0U);
    }

    /// The SPI CRC (datasheet "CRC Checksum"): CRC-8, polynomial x^8 + x^2 + x + 1 (0x07),
    /// initial value 0, no reflection, over the communications byte and the data bytes as
    /// they go across the wire; the part appends it to every read and expects it after
    /// every write once ERROR_EN.SPI_CRC_ERR_EN is set.
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

/// Which part: the ID register's upper nibble says (0 for the -4, 1 for the -8).
enum class Ad7124Model : std::uint8_t { any, ad7124_4, ad7124_8 };

/// One channel of the sequence: its positive and negative input. 0..15 are AIN0..AIN15
/// (AIN0..AIN7 on the -4); from 16 the internal ones: temperature sensor 16, AVSS 17,
/// internal reference 18, DGND 19.
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
        return static_cast<std::uint16_t>((polarity == Ad7124Polarity::bipolar ? 0x0800U : 0U)
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

    /// Continuous conversion on the internal clock, the status byte appended to every data
    /// read (DATA_STATUS, bit 10) so a conversion carries its channel; REF_EN (bit 8) turns
    /// the internal reference on when the setup selects it.
    [[nodiscard]] constexpr std::uint16_t adcControlValue(Ad7124Power     power,
                                                          Ad7124Reference reference) {
        return static_cast<std::uint16_t>(0x0400U
                                          | (reference == Ad7124Reference::internal ? 0x0100U : 0U)
                                          | (unsigned{std::to_underlying(power)} << 6U));
    }

    /// IO_CONTROL_1 with PDSW, the low-side power switch a bridge's excitation returns through.
    [[nodiscard]] constexpr std::uint32_t ioControl1Value(Ad7124PowerSwitch powerSwitch) {
        return powerSwitch == Ad7124PowerSwitch::closed ? 0x008000UL : 0UL;
    }

    [[nodiscard]] constexpr std::int32_t toSigned(std::uint32_t  code,
                                                  Ad7124Polarity polarity) {
        return polarity == Ad7124Polarity::bipolar ? static_cast<std::int32_t>(code) - 0x800000
                                                   : static_cast<std::int32_t>(code);
    }

    /// ADC_CONTROL MODE (bits 5:2): internal zero-scale (0101) and full-scale (0110)
    /// calibration, in the power mode given -- mid power at most for the full-scale one.
    enum class CalibrationMode : std::uint8_t { internalZeroScale = 0x5, internalFullScale = 0x6 };

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

    // The encodings against an example configuration written out as literals: differential
    // AIN1/AIN0 .. AIN7/AIN6, bipolar, all buffers, gain 8, post filter at 16.67 SPS, full
    // power, PDSW closed.
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

/// The knobs an AD7124 config may leave out, with what they are then (the part's reset
/// values where it has one); a config derives from this and redeclares what it sets. Every
/// channel uses setup 0. The SPIDeviceDefaults members (the in-flight timeout, RetryDelay,
/// AbsentRetry, AbsentAfterFailures) apply too.
struct Ad7124Defaults : SPIDeviceDefaults {
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

    /// The SPI CRC (ERROR_EN.SPI_CRC_ERR_EN): every frame after the bring-up's first write
    /// carries a checksum, a read whose checksum is wrong is dropped and counted. Off by
    /// default until the ERROR_EN bit layout and the CRC's place in a DATA_STATUS read have
    /// been verified against the datasheet on hardware (verify); a Config sets it on.
    static constexpr Ad7124Crc SpiCrc = Ad7124Crc::off;

    /// How often STATUS is read for a finished conversion. The part converts on its own
    /// clock and holds only the latest result, so a channel that converts faster than the
    /// poll is not seen at every conversion: set it below the output data rate (or read a
    /// sequence slow enough) when every sample matters.
    static constexpr std::chrono::milliseconds PollPeriod{5};

    /// Wait this long after start-up before the first reset: supplies and a bridge's
    /// excitation settling.
    static constexpr std::chrono::milliseconds StartupDelay{0};

    /// After the serial reset before the part is talked to again.
    static constexpr std::chrono::milliseconds ResetSettle{5};

    /// How long the bring-up waits for POR_FLAG to clear after the reset (2 ms typically), and
    /// for each internal calibration (four settling periods of the filter for the full-scale
    /// one) before it gives up and starts over.
    static constexpr std::chrono::milliseconds PorClearTimeout{100};
    static constexpr std::chrono::milliseconds CalibrationTimeout{5000};

    /// After a failed frame before the driver starts over (the SPIDeviceDefaults value is
    /// 5 ms; this part is left longer).
    static constexpr std::chrono::milliseconds RetryDelay{100};
};

/// Analog Devices AD7124-4 / AD7124-8, 24-bit sigma-delta ADC, over SPI (mode 3) on
/// SPIDeviceBase: every frame books the bus, has a watchdog, and three failed frames in a
/// row reset the part.
///
/// Bring-up: 64 ones (a serial reset), ID check, STATUS read until the power-on flag has
/// cleared (the internal reset takes about 2 ms; Linux ad7124.c polls it the same way), then
/// ERROR_EN (the SPI CRC from here on), IO_CONTROL_1, CONFIG_0, FILTER_0, the channels, and
/// ADC_CONTROL last, which starts continuous conversion through the channel sequence. With a
/// gain above 1, ADC_CONTROL is preceded by the internal calibration the datasheet recommends
/// whenever the gain changes ("Calibration"): OFFSET_0 back to 0x800000, an internal
/// full-scale calibration (in mid power at most, where it is supported), then an internal
/// zero-scale one, each waited out on STATUS RDY. The part factory-calibrates gain 1 and does
/// not support an internal full-scale calibration there. All channels share setup 0, so one
/// calibration covers them. Before the part counts as answering, ERROR is read: a
/// configuration write the part ignored because it was busy sets SPI_IGNORE_ERR, and the
/// bring-up starts over.
/// From then on STATUS is polled every `PollPeriod`: RDY low reads DATA with the status byte
/// appended, whose channel field says which channel the code belongs to; ERROR_FLAG reads
/// and logs the ERROR register; the power-on flag coming back means the part reset on its
/// own and is configured again.
///
/// The link follows the I2C engine's contract: answering() once the bring-up went through,
/// valid() when a conversion has arrived since; a failed frame or a wrong id starts the
/// bring-up over after RetryDelay, AbsentAfterFailures of them in a row report the part
/// absent and it is probed every AbsentRetry.
///
///     struct AdcConfig : Kvasir::SPI::Ad7124Defaults {
///         static constexpr std::array<Kvasir::SPI::Ad7124Channel, 2> Channels{{{1, 0}, {3, 2}}};
///         static constexpr auto Gain = Kvasir::SPI::Ad7124Gain::x8;
///     };
///     Kvasir::SPI::Ad7124<Spi, Clock, HW::Pin::adc_cs, AdcConfig> adc{};
///     ...
///     adc.handler();
///     if(auto const c = adc.nextConversion()) { use(c->channel, c->code); }
template<typename Spi, typename Clock, typename Cs, typename Config = Ad7124Defaults>
struct Ad7124 : SPIDeviceBase<Spi, Clock, Cs, Ad7124<Spi, Clock, Cs, Config>, Config, 9> {
    using Base    = SPIDeviceBase<Spi, Clock, Cs, Ad7124, Config, 9>;
    using Outcome = typename Base::Outcome;
    using Reg     = Ad7124Detail::Reg;

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
                  "AD7124-8: AIN0..AIN15, internal inputs from 16 to 31)");
    static_assert(Config::FilterSelect >= 1 && Config::FilterSelect <= 2047,
                  "FS[10:0] is 1..2047");

    /// A finished conversion: the channel's index in `Config::Channels` and the code,
    /// offset binary made signed for a bipolar setup.
    struct Conversion {
        std::uint8_t channel{};
        std::int32_t code{};
    };

    /// The input voltage a code stands for, against the reference the board supplies (2.5 V
    /// for `Ad7124Reference::internal`): code x VREF / (2^23 x gain) bipolar, / 2^24
    /// unipolar. At high gain one code is a few nanovolts, finer than the result's unit; a
    /// caller that needs that resolution keeps the code.
    [[nodiscard]] static constexpr Units::MicroVolt toVoltage(std::int32_t     code,
                                                              Units::MicroVolt reference
                                                              = Units::microVolt(2'500'000)) {
        auto const gain = std::int64_t{1} << std::to_underlying(Config::Gain);
        auto const span = Config::Polarity == Ad7124Polarity::bipolar ? std::int64_t{1} << 23
                                                                      : std::int64_t{1} << 24;
        return Units::microVolt(std::int64_t{code} * Units::value(reference) / (span * gain));
    }

    /// The internal temperature sensor's code (a channel whose positive input is 16, read
    /// bipolar at gain 1): degC = code / 13584 - 272.5 with the offset binary already
    /// removed.
    [[nodiscard]] static constexpr Units::MilliDegC toTemperature(std::int32_t code) {
        return Units::milliDegC(std::int64_t{code} * 1000 / 13584 - 272'500);
    }

    void resetLogic() {
        state_ = State::reset;
        crcOn_ = false;
        valid_ = false;
        this->markStarting();
    }

    void idleLogic() {
        auto const now = Clock::now();
        switch(state_) {
        case State::startup:
            wakeAt_ = now + Config::StartupDelay;
            state_  = State::startupWait;
            break;
        case State::startupWait:
            if(now >= wakeAt_) { state_ = State::reset; }
            break;
        case State::reset:
            crcOn_ = false;   // the serial reset puts every register back, ERROR_EN too
            this->tx_.fill(std::byte{0xFF});
            if(this->submit(std::span<std::byte const>{this->tx_}.first(8), {})) {
                state_ = State::resetWait;
            }
            break;
        case State::resetWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                wakeAt_ = now + Config::ResetSettle;
                state_  = State::settle;
            }
            break;
        case State::retry:
            if(now >= wakeAt_) { state_ = State::reset; }
            break;
        case State::settle:
            if(now >= wakeAt_) { state_ = State::id; }
            break;
        case State::id:
            if(read_(Reg::id, 1)) { state_ = State::idWait; }
            break;
        case State::idWait:
            if(auto const o = this->take(); o != Outcome::running) {
                auto const id = std::to_integer<std::uint8_t>(this->rx_[1]);
                if(o == Outcome::ok && Ad7124Detail::idMatches(Config::Model, id)) {
                    id_    = id;
                    state_ = State::clearPor;
                } else {
                    KVASIR_LOG_LIMITED(log_.allow(AbsentKey, now),
                                       UC_LOG_W,
                                       "ad7124: no part answers (id {})",
                                       id);
                    fail_(now);
                }
            }
            break;
        case State::clearPor:
            if(read_(Reg::status, 1)) { state_ = State::clearPorWait; }
            break;
        case State::clearPorWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                if((std::to_integer<std::uint8_t>(this->rx_[1]) & Ad7124Detail::PorFlag) != 0) {
                    if(porPolls_ == 0) { deadline_ = now + Config::PorClearTimeout; }
                    ++porPolls_;
                    if(now >= deadline_) {
                        KVASIR_LOG_LIMITED(log_.allow(PorKey, now),
                                           UC_LOG_W,
                                           "ad7124: POR_FLAG did not clear after the reset");
                        fail_(now);
                        break;
                    }
                    wakeAt_ = now + std::chrono::milliseconds{1};
                    state_  = State::clearPorGap;
                    break;
                }
                porPolls_   = 0;
                configStep_ = 0;
                state_      = State::configure;
            }
            break;
        case State::clearPorGap:
            if(now >= wakeAt_) { state_ = State::clearPor; }
            break;
        case State::configure:
            if(write_(InitWrites[configStep_])) { state_ = State::configureWait; }
            break;
        case State::configureWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                if(InitWrites[configStep_].reg == Ad7124Detail::address(Reg::errorEn)) {
                    crcOn_ = Config::SpiCrc == Ad7124Crc::on;   // from the next frame on
                }
                if(++configStep_ < InitWrites.size()) {
                    // ADC_CONTROL is the last write; the calibration goes before it
                    bool const beforeLast = configStep_ + 1 == InitWrites.size();
                    state_ = beforeLast && Calibrates ? State::calibrate : State::configure;
                    calibrationStep_ = 0;
                    break;
                }
                state_ = State::checkError;
            }
            break;
        case State::calibrate:
            if(write_(CalibrationWrites[calibrationStep_])) { state_ = State::calibrateWait; }
            break;
        case State::calibrateWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                if(calibrationStep_ == 0) {   // OFFSET_0 at its default: no wait
                    ++calibrationStep_;
                    state_ = State::calibrate;
                    break;
                }
                deadline_ = now + Config::CalibrationTimeout;
                wakeAt_   = now + Config::PollPeriod;
                state_    = State::calibratePoll;
            }
            break;
        case State::calibratePoll:
            if(now >= wakeAt_ && read_(Reg::status, 1)) { state_ = State::calibratePollWait; }
            break;
        case State::calibratePollWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                bool const done
                  = checkCrc_(1)
                 && (std::to_integer<std::uint8_t>(this->rx_[1]) & Ad7124Detail::NotReady) == 0;
                if(done) {
                    if(++calibrationStep_ < CalibrationWrites.size()) {
                        state_ = State::calibrate;
                    } else {
                        state_ = State::configure;   // ADC_CONTROL: continuous conversion
                    }
                } else if(now >= deadline_) {
                    KVASIR_LOG_LIMITED(log_.allow(ErrorKey, now),
                                       UC_LOG_W,
                                       "ad7124: internal calibration did not finish");
                    fail_(now);
                } else {
                    wakeAt_ = now + Config::PollPeriod;
                    state_  = State::calibratePoll;
                }
            }
            break;
        case State::checkError:
            if(read_(Reg::error, 3)) { state_ = State::checkErrorWait; }
            break;
        case State::checkErrorWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok || !checkCrc_(3)) {
                    fail_(now);
                    break;
                }
                if((be24_() & Ad7124Detail::SpiIgnoreErr) != 0) {
                    errorRegister_ = be24_();
                    KVASIR_LOG_LIMITED(log_.allow(ErrorKey, now),
                                       UC_LOG_W,
                                       "ad7124: a configuration write was ignored (ERROR {})",
                                       errorRegister_);
                    fail_(now);
                    break;
                }
                this->markAnswering();
                wakeAt_ = now;
                state_  = State::wait;
            }
            break;
        case State::wait:
            if(now >= wakeAt_) { state_ = State::status; }
            break;
        case State::status:
            if(read_(Reg::status, 1)) { state_ = State::statusWait; }
            break;
        case State::statusWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                if(!checkCrc_(1)) {
                    wakeAt_ = now + Config::PollPeriod;
                    state_  = State::wait;
                    break;
                }
                status_ = std::to_integer<std::uint8_t>(this->rx_[1]);
                if((status_ & Ad7124Detail::PorFlag) != 0) {
                    KVASIR_LOG_LIMITED(log_.allow(PorKey, now),
                                       UC_LOG_W,
                                       "ad7124: the part reset itself, configuring it again");
                    resetLogic();
                } else if((status_ & Ad7124Detail::ErrorFlag) != 0) {
                    state_ = State::error;
                } else if((status_ & Ad7124Detail::NotReady) == 0) {
                    state_ = State::data;
                } else {
                    wakeAt_ = now + Config::PollPeriod;
                    state_  = State::wait;
                }
            }
            break;
        case State::data:
            if(read_(Reg::data, 4)) { state_ = State::dataWait; }
            break;
        case State::dataWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    fail_(now);
                    break;
                }
                if(checkCrc_(4)) { decode_(); }
                wakeAt_ = now + Config::PollPeriod;
                state_  = State::wait;
            }
            break;
        case State::error:
            if(read_(Reg::error, 3)) { state_ = State::errorWait; }
            break;
        case State::errorWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o == Outcome::ok && checkCrc_(3)) {
                    errorRegister_ = be24_();
                    ++errorReports_;
                    KVASIR_LOG_LIMITED(log_.allow(ErrorKey, now),
                                       UC_LOG_W,
                                       "ad7124: ERROR register {}",
                                       errorRegister_);
                }
                // A conversion that is ready is still read: an error flag (a reference
                // detect, a saturated input) is reported, not a reason to stop measuring.
                if((status_ & Ad7124Detail::NotReady) == 0) {
                    state_ = State::data;
                } else {
                    wakeAt_ = now + Config::PollPeriod;
                    state_  = State::wait;
                }
            }
            break;
        }
    }

    /// The latest conversion not yet taken, once. One that is overwritten before it is taken
    /// counts in missed().
    [[nodiscard]] std::optional<Conversion> nextConversion() {
        auto const c = pending_;
        pending_.reset();
        return c;
    }

    /// Answering, and a conversion has arrived since the last bring-up.
    [[nodiscard]] bool valid() const { return this->answering() && valid_; }

    /// Conversions over the device's life; steps with each one.
    [[nodiscard]] std::uint32_t conversions() const { return conversions_; }

    [[nodiscard]] std::uint32_t seq() const { return conversions_; }

    [[nodiscard]] std::uint32_t missed() const { return missed_; }

    /// The ID register as read at bring-up.
    [[nodiscard]] std::uint8_t id() const { return id_; }

    /// The ERROR register the last time the status byte flagged one, and how often it did.
    [[nodiscard]] std::uint32_t errorRegister() const { return errorRegister_; }

    [[nodiscard]] std::uint32_t errorReports() const { return errorReports_; }

    /// Reads whose SPI CRC did not check and were dropped.
    [[nodiscard]] std::uint32_t crcErrors() const { return crcErrors_; }

    /// The configuration writes, in order, as they go on the wire.
    static constexpr auto InitWrites = [] {
        using Ad7124Detail::address;
        std::array<Ad7124Detail::Write, ChannelCount + 5> w{};
        std::size_t                                       i = 0;
        // First: from here on every frame carries the CRC.
        w[i++] = {address(Reg::errorEn), 3, Ad7124Detail::errorEnValue(Config::SpiCrc)};
        w[i++] = {address(Reg::ioControl1), 3, Ad7124Detail::ioControl1Value(Config::PowerSwitch)};
        w[i++] = {address(Reg::config0),
                  2,
                  Ad7124Detail::configValue(Config::Polarity,
                                            Config::ReferenceBuffers,
                                            Config::InputBuffers,
                                            Config::Reference,
                                            Config::Gain)};
        w[i++] = {address(Reg::filter0),
                  3,
                  Ad7124Detail::filterValue(Config::Filter,
                                            Config::Reject60Hz,
                                            Config::PostFilter,
                                            Config::FilterSelect)};
        for(std::size_t c = 0; c < ChannelCount; ++c) {
            w[i++]
              = {address(Reg::channel0, c), 2, Ad7124Detail::channelValue(Config::Channels[c])};
        }
        // Last: it switches the part to continuous conversion through the sequence.
        w[i++] = {address(Reg::adcControl),
                  2,
                  Ad7124Detail::adcControlValue(Config::Power, Config::Reference)};
        return w;
    }();

    /// Whether the bring-up runs the internal calibration: a gain above 1.
    static constexpr bool Calibrates = Config::Gain != Ad7124Gain::x1;

    /// OFFSET_0 to its default, the full-scale calibration, the zero-scale one.
    static constexpr std::array<Ad7124Detail::Write, 3> CalibrationWrites{
      {{Ad7124Detail::address(Reg::offset0), 3, 0x800000},
       {Ad7124Detail::address(Reg::adcControl),
        2,
        Ad7124Detail::adcControlCalibration(Config::Power,
                                            Config::Reference,
                                            Ad7124Detail::CalibrationMode::internalFullScale)},
       {Ad7124Detail::address(Reg::adcControl),
        2,
        Ad7124Detail::adcControlCalibration(Config::Power,
                                            Config::Reference,
                                            Ad7124Detail::CalibrationMode::internalZeroScale)}}
    };

private:
    enum class State : std::uint8_t {
        startup,
        startupWait,
        reset,
        resetWait,
        retry,
        settle,
        id,
        idWait,
        clearPor,
        clearPorWait,
        clearPorGap,
        configure,
        configureWait,
        calibrate,
        calibrateWait,
        calibratePoll,
        calibratePollWait,
        checkError,
        checkErrorWait,
        wait,
        status,
        statusWait,
        data,
        dataWait,
        error,
        errorWait,
    };

    static constexpr std::uint32_t AbsentKey = rateLimitKey(2);
    static constexpr std::uint32_t ErrorKey  = rateLimitKey(3);
    static constexpr std::uint32_t PorKey    = rateLimitKey(4);
    static constexpr std::uint32_t CrcKey    = rateLimitKey(5);

    /// A failed frame or a wrong id: start over after the base's retry delay.
    void fail_(typename Clock::time_point now) {
        valid_    = false;
        crcOn_    = false;
        porPolls_ = 0;
        this->reportFailure();
        wakeAt_ = now + this->retryDelay();
        state_  = State::retry;
    }

    bool write_(Ad7124Detail::Write const& w) {
        this->tx_[0] = std::byte{w.reg};
        for(std::size_t i = 0; i < w.size; ++i) {
            auto const shift = 8U * static_cast<unsigned>(w.size - 1U - i);
            this->tx_[1 + i] = std::byte{static_cast<std::uint8_t>((w.value >> shift) & 0xFFU)};
        }
        std::size_t n = 1U + w.size;
        if(crcOn_) {
            this->tx_[n]
              = std::byte{Ad7124Detail::crc(std::span<std::byte const>{this->tx_}.first(n))};
            ++n;
        }
        return this->submit(std::span<std::byte const>{this->tx_}.first(n), {});
    }

    /// `n` data bytes, plus the CRC byte behind them once it is on.
    bool read_(Reg         reg,
               std::size_t n) {
        auto const cmd
          = static_cast<std::uint8_t>(Ad7124Detail::address(reg) | Ad7124Detail::ReadBit);
        return this->readRegister(cmd, crcOn_ ? n + 1 : n);
    }

    /// The CRC over the command byte, the `n` data bytes and the CRC byte itself is zero
    /// when the frame is intact. Always true while the CRC is off.
    [[nodiscard]] bool checkCrc_(std::size_t n) {
        if(!crcOn_) { return true; }
        if(Ad7124Detail::crc(frameForCrc_(n)) == 0) { return true; }
        ++crcErrors_;
        KVASIR_LOG_LIMITED(log_.allow(CrcKey, Clock::now()),
                           UC_LOG_W,
                           "ad7124: SPI CRC error on register {}",
                           std::to_integer<std::uint8_t>(this->tx_[0]) & 0x3FU);
        return false;
    }

    /// The bytes the CRC covers on a read: the command byte as sent, then the data and the
    /// CRC as received. They live in two buffers, so they are put side by side in rx_[0],
    /// which received nothing meaningful during the command byte.
    [[nodiscard]] std::span<std::byte const> frameForCrc_(std::size_t n) {
        this->rx_[0] = this->tx_[0];
        return std::span<std::byte const>{this->rx_}.first(n + 2);
    }

    [[nodiscard]] std::uint32_t be24_() const {
        return Bytes{std::span<std::byte const>{this->rx_}}.be24(1);
    }

    void decode_() {
        auto const channel = static_cast<std::uint8_t>(std::to_integer<std::uint8_t>(this->rx_[4])
                                                       & Ad7124Detail::ChannelMask);
        if(channel >= ChannelCount) { return; }
        if(pending_) { ++missed_; }
        pending_ = Conversion{channel, Ad7124Detail::toSigned(be24_(), Config::Polarity)};
        valid_   = true;
        ++conversions_;
    }

    State                      state_{State::startup};
    typename Clock::time_point wakeAt_{};
    std::size_t                configStep_{};
    std::size_t                calibrationStep_{};
    std::uint32_t              porPolls_{};
    typename Clock::time_point deadline_{};
    std::uint8_t               status_{};
    std::uint8_t               id_{};
    bool                       crcOn_{false};
    bool                       valid_{false};
    std::optional<Conversion>  pending_{};
    std::uint32_t              conversions_{};
    std::uint32_t              missed_{};
    std::uint32_t              errorRegister_{};
    std::uint32_t              errorReports_{};
    std::uint32_t              crcErrors_{};
    RateLimiter<Clock>         log_{};
};

}   // namespace Kvasir::SPI
