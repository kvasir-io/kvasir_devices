#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Es8311Detail {

    /// Which side of the serial port drives the clocks.
    ///
    /// `codecMaster`: the codec generates BCLK and LRCK from MCLK, and the host only supplies
    /// MCLK and shifts data. `codecSlave`: the host generates all three, which is what a
    /// general I2S controller does. This description always takes the converters' clock from
    /// the MCLK pin (REG01 MCLK_SEL 0) and divides it (see Coefficients below); the part can
    /// also take it from BCLK (MCLK_SEL 1), which Linux es8311.c uses in slave mode without
    /// an MCLK, but that is not offered here.
    enum class Role : std::uint8_t { codecMaster, codecSlave };

    /// Serial audio data word length, REG09/REG0A bits 4:2 (SDP_IN_WL / SDP_OUT_WL).
    enum class WordLength : std::uint8_t {
        bits24 = 0,
        bits20 = 1,
        bits18 = 2,
        bits16 = 3,
        bits32 = 4
    };

    /// One row of the clock-manager coefficients: what REG02..REG05 must hold for a given
    /// (MCLK, sample rate) pair. Taken from the `coeff_div[]` table of the vendor example
    /// driver (es8311.c), which is the only published form of it -- the datasheet documents
    /// the register *fields* but not the values that pair with each clock. Only the 6.144 MHz
    /// and 12.288 MHz master clocks are carried here; other clocks need rows of their own.
    ///
    /// The vendor rows also carry the master-mode LRCK and BCLK dividers, the same for every
    /// row. Those divide the MCLK pin (REG06..REG08), not the pre-divided clock, so they are
    /// right only where MCLK is 256 fs; the codec works them out from MCLK / fs instead.
    struct Coefficients {
        std::uint32_t mclkHz{};
        std::uint32_t sampleRate{};
        std::uint8_t  preDiv{};     ///< REG02 bits 7:5, as (preDiv - 1)
        std::uint8_t  preMulti{};   ///< REG02 bits 4:3
        std::uint8_t  adcDiv{};     ///< REG05 bits 7:4, as (adcDiv - 1)
        std::uint8_t  dacDiv{};     ///< REG05 bits 3:0, as (dacDiv - 1)
        std::uint8_t  fsMode{};     ///< REG03 bit 6
        std::uint8_t  adcOsr{};     ///< REG03 bits 5:0
        std::uint8_t  dacOsr{};     ///< REG04 bits 6:0
    };

    inline constexpr std::array Coeffs{
      //          mclk     rate  pre  mul  adc  dac   fs  aOsr  dOsr
      Coefficients{ 6144000,  8000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{ 6144000, 12000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{ 6144000, 16000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{ 6144000, 24000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{ 6144000, 32000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{ 6144000, 48000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{12288000,  8000, 0x06, 0x00, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{12288000, 12000, 0x04, 0x00, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{12288000, 16000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{12288000, 24000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{12288000, 32000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x10, 0x10},
      Coefficients{12288000, 48000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x10, 0x10},
    };

    [[noreturn]] void no_ES8311_BCLK_divider_for_this_MCLK_rate_and_word_length();

    /// REG06 DIV_BCLK for an MCLK-to-BCLK ratio: 1..20 are the ratio less one, the codes above
    /// that name the ratios the datasheet lists (REGISTER 0X06).
    consteval std::uint8_t bclkCodeFor(std::uint32_t ratio) {
        if(ratio >= 1 && ratio <= 20) { return static_cast<std::uint8_t>(ratio - 1); }
        constexpr std::array<std::uint32_t, 12>
          Listed{22, 24, 25, 30, 32, 33, 34, 36, 44, 48, 66, 72};
        for(std::size_t i = 0; i < Listed.size(); ++i) {
            if(Listed[i] == ratio) { return static_cast<std::uint8_t>(20 + i); }
        }
        no_ES8311_BCLK_divider_for_this_MCLK_rate_and_word_length();
    }

    // Declared and never defined: calling it is not a constant expression, so a consteval
    // function that reaches one fails to compile and the diagnostic names it. `throw` would
    // say it more directly but freestanding builds run without exceptions, and a
    // static_assert cannot see a function's arguments.
    [[noreturn]] void no_ES8311_coefficients_for_this_MCLK_and_sample_rate();

    [[noreturn]] void ES8311_DAC_volume_out_of_range_minus95dB_to_plus32dB();

    /// The row for a clock pair, or a compile error naming the pair that has none. The vendor
    /// driver reports the same condition only at run time ("Unable to configure sample
    /// rate").
    consteval Coefficients coefficientsFor(Hertz mclk,
                                           Hertz sampleRate) {
        for(auto const& c : Coeffs) {
            if(Units::hertz(c.mclkHz) == mclk && Units::hertz(c.sampleRate) == sampleRate) {
                return c;
            }
        }
        no_ES8311_coefficients_for_this_MCLK_and_sample_rate();
    }

    /// DAC volume, REG32: 0.5 dB a step with 0xBF as 0 dB, 0x00 muting to -95.5 dB and 0xFF
    /// reaching +32 dB. A helper because "half a decibel below unity" is how a datasheet talks
    /// about it and `0xBD` is not. Rounded to the nearest step: -0.3 dB is 0xBE, not 0xBF.
    consteval std::uint8_t volumeForDb(double db) {
        auto const half  = db * 2.0;
        auto const steps = static_cast<int>(half >= 0.0 ? half + 0.5 : half - 0.5);
        auto const step  = 0xBF + steps;
        if(step < 0 || step > 0xFF) { ES8311_DAC_volume_out_of_range_minus95dB_to_plus32dB(); }
        return static_cast<std::uint8_t>(step);
    }

    /// The Mute group's value: the DSM and DEM mutes set, or the output playing.
    enum class Muting : std::uint8_t { muted, playing };
}   // namespace Es8311Detail

/// What an Es8311 is configured with, as a struct a board derives from and overrides:
///
///     struct Codec : Kvasir::I2C::Chips::Es8311Config {
///         static constexpr Hertz Mclk       = Units::hertz(12'288'000);
///         static constexpr Hertz SampleRate = Units::hertz(48'000);
///     };
///     using Chip = Kvasir::I2C::Chips::Es8311Codec<Codec>;
///
/// `SampleRate` is the one member that may be left out: it is then `Mclk / 256`, which is
/// the rate the part reaches from that clock without its clock doubler (24 kHz from
/// 6.144 MHz, 48 kHz from 12.288 MHz). `Addr` is the I2C address the CE pin gives.
struct Es8311Config {
    static constexpr Hertz                    Mclk       = Units::hertz(6'144'000);
    static constexpr Es8311Detail::Role       SerialRole = Es8311Detail::Role::codecMaster;
    static constexpr Es8311Detail::WordLength Width      = Es8311Detail::WordLength::bits16;
    static constexpr Address7                 Addr       = 0x18;
};

namespace Es8311Detail {
    /// The positional form's parameters as a config struct, for the `Es8311<...>` alias.
    template<Hertz Mclk_, Hertz Rate, Role SerialRole_, WordLength Width_, Address7 Addr_>
    struct Positional : Es8311Config {
        static constexpr Hertz      Mclk       = Mclk_;
        static constexpr Hertz      SampleRate = Rate;
        static constexpr Role       SerialRole = SerialRole_;
        static constexpr WordLength Width      = Width_;
        static constexpr Address7   Addr       = Addr_;
    };

    template<typename Cfg>
    consteval Hertz sampleRateOf() {
        if constexpr(requires { Cfg::SampleRate; }) {
            return Cfg::SampleRate;
        } else {
            return Cfg::Mclk / 256;
        }
    }
}   // namespace Es8311Detail

/// Everest Semiconductor ES8311 low-power mono audio codec, from the datasheet Revision 10.0,
/// January 2021.
///
/// Address 0b0011'00x where x is the CE pin (datasheet "Control Interface"), so 0x18 with CE
/// low and 0x19 with it high. One-byte registers. Chip ID 0xFD reads 0x83 and 0xFE reads 0x11.
///
/// This is the control plane only. Audio samples do not pass through here: they go over I2S,
/// and the codec needs MCLK running before its converters do anything. A description brings
/// the part up and sets its volume; something else has to be clocking it.
///
/// Two vendor-driver discrepancies against the datasheet, not reproduced here:
///
///   * its mic gain enum (`ES8311_MIC_GAIN_*`), 6 dB a step to 42 dB, is REG16's ADC_SCALE
///     (bits 2:0, 0 to 42 dB), not the analogue PGA: REG14 PGAGAIN (bits 3:0) is 3 dB a step,
///     0 dB to 30 dB (datasheet REGISTERS 0X14 and 0X16). The mic is documented below but not
///     driven, so this only matters to whoever wires it;
///   * it maps a 0..100 "volume" onto REG32 as `(v * 256 / 100) - 1`, which makes 73 mean
///     -3 dB by arithmetic accident. REG32 is a plain 0.5 dB ladder and `Es8311Detail::
///     volumeForDb()` says so.
///
/// The microphone path is deliberately unimplemented. Its registers, for whoever wants it:
/// REG14 = 0x1A selects Mic1p-Mic1n with PGA gain, REG16 the ADC gain scale, REG17 the ADC
/// digital volume, REG0A the ADC serial-port format.
///
/// Configured by a struct derived from `Es8311Config` (`Es8311Codec<Cfg>`), or positionally
/// through the `Es8311<Mclk, SampleRate, SerialRole, Width, Addr>` alias below.
template<typename Cfg = Es8311Config>
struct Es8311Codec {
    static_assert(std::derived_from<Cfg,
                                    Es8311Config>,
                  "derive the configuration from Kvasir::I2C::Chips::Es8311Config");

    static constexpr Hertz                    Mclk       = Cfg::Mclk;
    static constexpr Hertz                    SampleRate = Es8311Detail::sampleRateOf<Cfg>();
    static constexpr Es8311Detail::Role       SerialRole = Cfg::SerialRole;
    static constexpr Es8311Detail::WordLength Width      = Cfg::Width;
    static constexpr Address7                 Addr       = Cfg::Addr;

    static constexpr std::string_view        Name    = "ES8311";
    static constexpr Address7                Address = Addr;
    static constexpr std::array<Address7, 2> Addresses{0x18, 0x19};
    static constexpr std::size_t             RegisterBytes = 1;

    static_assert(Addr == 0x18 || Addr == 0x19,
                  "the CE pin gives 0x18 or 0x19");

    static constexpr auto Coeff = Es8311Detail::coefficientsFor(Mclk, SampleRate);

    /// REG02's MULT_PRE multiplies the pre-divided master clock by 2, 4 or 8. The datasheet
    /// defines it as that alone, with PATHSEL (bit 2, the clock doubler path) as a separate
    /// field and no condition on it beyond a pre-divided clock above 1 MHz at 3.3 V for the
    /// x4 and x8 codes; the vendor example and Linux es8311.c both set MULT_PRE with PATHSEL at
    /// its reset value. Until that is tried on a part here, the rows that need it are refused
    /// rather than risk a rate a half, a quarter or an eighth of the one asked for. For 48 kHz, feed the part 12.288 MHz; for a 6.144 MHz master clock, 24 kHz
    /// is the top of what it gives without MULT_PRE.
    static_assert(Coeff.preMulti == 0,
                  "this (MCLK, sample rate) pair needs the ES8311's clock doubler (REG02 "
                  "MULT_PRE), which this description does not configure -- the part would run "
                  "at a half, a quarter or an eighth of the rate asked for. Use a row with "
                  "MULT_PRE 0: 24 kHz from a 6.144 MHz master clock, or 48 kHz from 12.288 MHz");

    static constexpr bool CodecIsMaster = SerialRole == Es8311Detail::Role::codecMaster;

    /// The master-mode dividers, both of the MCLK pin (REGISTERS 0X06..0X08): LRCK is
    /// MCLK / (DIV_LRCK + 1), so DIV_LRCK is MCLK / fs - 1; BCLK carries two slots a frame, 16
    /// bits each for 16-bit words and 32 for the longer ones. A slave ignores both.
    static constexpr std::uint32_t MclkPerFrame
      = static_cast<std::uint32_t>(Units::value(Mclk) / Units::value(SampleRate));
    static_assert(MclkPerFrame * Units::value(SampleRate) == Units::value(Mclk),
                  "MCLK is a whole multiple of the sample rate");
    static constexpr std::uint16_t LrckDiv = static_cast<std::uint16_t>(MclkPerFrame - 1U);
    static constexpr std::uint32_t BitsPerFrame
      = Width == Es8311Detail::WordLength::bits16 ? 32U : 64U;
    static constexpr std::uint8_t BclkCode = Es8311Detail::bclkCodeFor(
      MclkPerFrame % BitsPerFrame == 0 ? MclkPerFrame / BitsPerFrame : 0U);

    /// REG00 after the resets are released: CSM_ON (bit 7) always, MSC (bit 6) only when the
    /// codec drives the clocks.
    static constexpr std::uint8_t Reg00Run
      = static_cast<std::uint8_t>(0x80U | (CodecIsMaster ? 0x40U : 0x00U));

    /// REG09, the DAC-side serial port: word length in bits 4:2, format I2S in bits 1:0, and
    /// SDP_IN_SEL (bit 7) left at 0 so the left-channel slot feeds the DAC.
    static constexpr std::uint8_t Reg09
      = static_cast<std::uint8_t>(static_cast<std::uint8_t>(Width) << 2U);

    /// Bring-up, in the vendor's order, with the microphone writes left out.
    ///
    /// The 20 ms after the reset is the vendor's and is not in the datasheet's power-up
    /// sequence; it costs nothing and a codec that has not settled answers its own ID.
    static constexpr std::array Init{
      // Two single-register reads, not one two-byte one: the ES8311's pointer does not
      // auto-increment, so a two-byte read would return 0xFD twice.
      Step::read({.reg = 0xFD, .count = 1, .offset = 0}),   // CHIP_ID1 -> setup()
      Step::read({.reg = 0xFE, .count = 1, .offset = 1}),   // CHIP_ID2
      Step::identify(),
      Step::write({.reg     = 0x00,
                   .payload = {0x1F},
                   .delay   = std::chrono::milliseconds{20}}),   // assert every reset, then wait
      Step::write({.reg = 0x00, .payload = {0x00}}),             // release them
      Step::write({.reg = 0x00, .payload = {0x80}}),             // CSM_ON: the state machine runs
      Step::write(
        {.reg = 0x01, .payload = {0x3F}}),   // MCLK from the MCLK pin, every clock domain on
      // The clock manager: REG02..REG05 straight out of the coefficient row, REG06..REG08 the
      // master-mode dividers.
      Step::write({.reg     = 0x02,
                   .payload = {static_cast<std::uint8_t>(((Coeff.preDiv - 1U) << 5U)
                                                         | (Coeff.preMulti << 3U))}}),
      Step::write(
        {.reg = 0x03, .payload = {static_cast<std::uint8_t>((Coeff.fsMode << 6U) | Coeff.adcOsr)}}),
      Step::write({.reg = 0x04, .payload = {Coeff.dacOsr}}),
      Step::write({.reg     = 0x05,
                   .payload = {static_cast<std::uint8_t>(((Coeff.adcDiv - 1U) << 4U)
                                                         | (Coeff.dacDiv - 1U))}}),
      Step::write({.reg = 0x06, .payload = {BclkCode}}),
      Step::write({.reg = 0x07, .payload = {static_cast<std::uint8_t>(LrckDiv >> 8U)}}),
      Step::write({.reg = 0x08, .payload = {static_cast<std::uint8_t>(LrckDiv & 0xFFU)}}),
      Step::write({.reg = 0x00, .payload = {Reg00Run}}),   // and now master, if Role says so
      Step::write({.reg = 0x09, .payload = {Reg09}}),      // Width-bit I2S into the DAC
      // Analogue. REG0D powers the reference and bias and starts VMID charging; REG0E enables
      // the PGA and modulator; REG12 powers the DAC; REG13 bit 4 (HPSW) picks the headphone
      // driver over the line-out default.
      Step::write({.reg = 0x0D, .payload = {0x01}}),
      Step::write({.reg = 0x0E, .payload = {0x02}}),
      Step::write({.reg = 0x12, .payload = {0x00}}),
      Step::write({.reg = 0x13, .payload = {0x10}}),
      Step::write({.reg     = 0x1C,
                   .payload = {0x6A}}),   // ADC equaliser bypassed, DC offset cancelled digitally
      Step::write({.reg = 0x37, .payload = {0x08}}),   // DAC equaliser bypassed
      // Muted, and quiet, until the application says otherwise: the volume the part comes up
      // at is unknown to the description, and a loud first sample is not a good default.
      Step::write({.reg = 0x31, .payload = {0x60}}),
      Step::write({.reg = 0x32, .payload = {Es8311Detail::volumeForDb(-30.0)}}),
    };

    struct State {
        std::uint8_t deviceIdHigh{};   ///< CHIP_ID1 (0xFD), 0x83
        std::uint8_t deviceIdLow{};    ///< CHIP_ID2 (0xFE), 0x11
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.deviceIdHigh = data.u8(0);
        state.deviceIdLow  = data.u8(1);
        return state.deviceIdHigh == 0x83 && state.deviceIdLow == 0x11;
    }

    /// DAC volume, REG32. 0.5 dB a step, 0xBF is 0 dB; `Es8311Detail::volumeForDb()` converts.
    struct Volume {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Es8311Detail::volumeForDb(-30.0);

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{value};
            return Step::writeBuffer({.reg = 0x32, .offset = 0, .count = 1});
        }
    };

    /// REG31 bits 6 and 5, the DSM and DEM mutes together. Muted at bring-up, so an
    /// application that never touches this makes no sound rather than an undefined one.
    struct Mute {
        using Value                        = Es8311Detail::Muting;
        static constexpr std::size_t Bytes = 1;
        static constexpr Value       Initial{Value::muted};

        [[nodiscard]] static constexpr Step encode(Value const&         muting,
                                                   std::span<std::byte> buffer) {
            buffer[0] = muting == Value::muted ? std::byte{0x60} : std::byte{0x00};
            return Step::writeBuffer({.reg = 0x31, .offset = 0, .count = 1});
        }
    };

    /// DAC volume ramp, REG37 bits 7:4: 0 is off, n is 0.25 dB per 2^(n+1) LRCK, so the
    /// higher the number the slower the fade. Bit 3 keeps the DAC equaliser bypassed, matching
    /// the bring-up. Takes the edge off starting and stopping a tone.
    struct Fade {
        using Value                        = std::uint8_t;   ///< 0..15
        static constexpr std::size_t Bytes = 1;
        static constexpr Value       Initial{0};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{static_cast<std::uint8_t>(((value & 0x0FU) << 4U) | 0x08U)};
            return Step::writeBuffer({.reg = 0x37, .offset = 0, .count = 1});
        }
    };

    /// A read-back of the registers that decide whether the output is audible, so a silent
    /// output with a running stream can be diagnosed from outside.
    /// Ten single-register reads, not one block: this part's pointer does not auto-increment.
    struct Status {
        static constexpr auto Period = std::chrono::milliseconds{1000};

        static constexpr std::array Steps{
          Step::read({.reg = 0x00, .count = 1, .offset = 0}),   // CSM_ON, master/slave, resets
          Step::read({.reg = 0x01, .count = 1, .offset = 1}),   // the clock domains
          Step::read({.reg    = 0x09,
                      .count  = 1,
                      .offset = 2}),   // serial port: word length, format, SDP_IN_MUTE
          Step::read({.reg = 0x0D, .count = 1, .offset = 3}),   // analogue power-down bits
          Step::read({.reg = 0x0E, .count = 1, .offset = 4}),   // PGA, modulator, VROI
          Step::read({.reg = 0x12, .count = 1, .offset = 5}),   // DAC power
          Step::read({.reg = 0x13, .count = 1, .offset = 6}),   // HPSW: headphone drive or line out
          Step::read({.reg = 0x31, .count = 1, .offset = 7}),   // the DAC mutes
          Step::read({.reg = 0x32, .count = 1, .offset = 8}),   // DAC volume
          Step::read(
            {.reg = 0x37, .count = 1, .offset = 9}),   // fade, and the DAC equaliser bypass
        };

        struct Sample {
            std::uint8_t reg00{}, reg01{}, reg09{}, reg0D{}, reg0E{};
            std::uint8_t reg12{}, reg13{}, reg31{}, reg32{}, reg37{};

            /// Everything that has to be true for a sample to reach the pin, in one place.
            [[nodiscard]] constexpr bool csmOn() const { return (reg00 & 0x80U) != 0; }

            [[nodiscard]] constexpr bool isMaster() const { return (reg00 & 0x40U) != 0; }

            [[nodiscard]] constexpr bool serialPortMuted() const { return (reg09 & 0x40U) != 0; }

            [[nodiscard]] constexpr bool dacMuted() const { return (reg31 & 0x60U) != 0; }

            [[nodiscard]] constexpr bool dacPowered() const { return reg12 == 0x00; }

            [[nodiscard]] constexpr bool analogUp() const { return (reg0D & 0x80U) == 0; }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {data.u8(0),
                    data.u8(1),
                    data.u8(2),
                    data.u8(3),
                    data.u8(4),
                    data.u8(5),
                    data.u8(6),
                    data.u8(7),
                    data.u8(8),
                    data.u8(9)};
        }
    };

    using Reads  = List<Status>;
    using Writes = List<Volume, Mute, Fade>;
};

/// The positional form: `Es8311<Units::hertz(6'144'000), Units::hertz(24'000),
/// Role::codecSlave>`. `SampleRate` defaults to `Mclk / 256`, the rate that clock gives
/// without the doubler.
template<Hertz                    Mclk       = Es8311Config::Mclk,
         Hertz                    SampleRate = Mclk / 256,
         Es8311Detail::Role       SerialRole = Es8311Config::SerialRole,
         Es8311Detail::WordLength Width      = Es8311Config::Width,
         Address7                 Addr       = Es8311Config::Addr>
using Es8311 = Es8311Codec<Es8311Detail::Positional<Mclk, SampleRate, SerialRole, Width, Addr>>;

}   // namespace Kvasir::I2C::Chips
