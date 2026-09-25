#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace As7341Detail {
    /// CFG1 AGAIN (4:0): 0.5x, and each step doubles up to 512x. 11..31 are reserved.
    enum class Gain : std::uint8_t {
        x0_5 = 0,
        x1   = 1,
        x2   = 2,
        x4   = 3,
        x8   = 4,
        x16  = 5,
        x32  = 6,
        x64  = 7,
        x128 = 8,
        x256 = 9,
        x512 = 10,
    };

    /// ATIME and ASTEP: the integration time is (ATIME + 1) x (ASTEP + 1) x 2.78 us.
    struct Codes {
        std::int64_t atime{};
        std::int64_t astep{};
    };

    /// (ATIME + 1) x (ASTEP + 1) x 2.78 us, rounded up to whole microseconds.
    [[nodiscard]] constexpr std::chrono::microseconds integrationTime(Codes c) {
        return std::chrono::microseconds{((c.atime + 1) * (c.astep + 1) * 278 + 99) / 100};
    }

    /// The codes for `t` with the smallest ATIME whose ASTEP reaches it: ASTEP + 1 is the nearest
    /// whole number of (ATIME + 1) x 2.78 us steps. ASTEP is -1 when `t` is below half the shortest
    /// step and 65535 (reserved) when not even ATIME 255 reaches it.
    [[nodiscard]] constexpr Codes codesFor(std::chrono::microseconds t) {
        auto const hundredths = t.count() * 100;   // in 0.01 us
        for(std::int64_t atime = 0; atime <= 255; ++atime) {
            auto const step  = (atime + 1) * 278;
            auto const steps = (hundredths + step / 2) / step;
            if(steps <= 65535) { return {atime, steps - 1}; }
        }
        return {255, 65535};
    }
}   // namespace As7341Detail

/// ams AS7341 11-channel spectral sensor. One-byte pointer: ENABLE 0x80 (PON 0, SP_EN 1,
/// WEN 3, SMUXEN 4, FDEN 6), ATIME 0x81, ID 0x92 (the chip id is in bits 7:2 and reads
/// 0x09), STATUS 0x93, ASTATUS 0x94, the six 16-bit little-endian ADC results from
/// CH0_DATA_L 0x95, STATUS2 0xA3 (AVALID bit 6), CFG1 0xAA (AGAIN 4:0), CFG6 0xAF
/// (SMUX_CMD 4:3) and ASTEP 0xCA/0xCB. Integration time is
/// (ATIME + 1) x (ASTEP + 1) x 2.78 us.
///
/// The part has eight visible channels plus clear, NIR and flicker, but only six ADCs, so a
/// full spectrum needs two integration cycles with the photodiode multiplexer (SMUX)
/// remapped between them. The SMUX is twenty bytes of RAM at 0x00..0x13 written while
/// SP_EN is low, with CFG6 set to "write config" first and SMUXEN then latching it. The two
/// mappings are not in the datasheet -- ams publishes them in an application note and
/// reference driver -- and the ones used here are that reference mapping:
///
///     F1F4_Clear_NIR: ADC0=F1 ADC1=F2 ADC2=F3 ADC3=F4 ADC4=Clear ADC5=NIR
///     F5F8_Clear_NIR: ADC0=F5 ADC1=F6 ADC2=F7 ADC3=F8 ADC4=Clear ADC5=NIR
///
/// One read group runs both passes back to back, so a Sample is a whole spectrum taken
/// under one set of conditions rather than two halves a caller has to pair up. The twenty
/// SMUX bytes go out as three writes because a Step's inline payload is eight bytes.
///
/// The flicker-detection channel and the FIFO are not driven here: flicker detection is a
/// separate engine with its own timing and its own status register, and it does not belong
/// in a cyclic spectral read.
///
/// The integration time comes from `Timing`, in one of two forms:
///   IntegrationTime   a duration (50 ms when Timing sets neither form): the smallest ATIME
///                     whose ASTEP reaches it, ASTEP rounded to the nearest step
///   Atime, Astep      the two codes as the datasheet gives them (29 and 599 is its 50 ms)
///     struct Datasheet { static constexpr unsigned Atime = 29, Astep = 599; };
///     struct Short     { static constexpr auto IntegrationTime = std::chrono::milliseconds{20}; };
/// `IntegrationTime` is what the codes come to.
///
/// Fixed address 0x39 -- which the APDS-9960 also uses, so the two cannot share a bus.
/// The Sample is raw counts with no lux, so the gain written at run time (GainSetting)
/// needs no scale here.
template<As7341Detail::Gain Gain = As7341Detail::Gain::x256, typename Timing = DefaultTiming>
struct As7341 {
    static constexpr std::string_view Name = "AS7341";
    /// ams AS7341. AS7341.md:1363..1370, ID register 0x92: part number 001001b in 7:2.
    static constexpr std::array Identity{
      RegisterCheck{"id", 0x92, 1, true, 0xFC, 0x24},
    };
    static constexpr Address7                Address = 0x39;
    static constexpr std::array<Address7, 1> Addresses{0x39};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr bool ByTime  = requires { Timing::IntegrationTime; };
    static constexpr bool ByCodes = requires { Timing::Atime; } || requires { Timing::Astep; };

    static_assert(!(ByTime && ByCodes),
                  "Timing sets IntegrationTime or Atime and Astep, not both");
    static_assert(!ByCodes || (requires { Timing::Atime; } && requires { Timing::Astep; }),
                  "Timing sets Atime and Astep together");

    static constexpr As7341Detail::Codes Codes = [] {
        if constexpr(ByCodes) {
            return As7341Detail::Codes{static_cast<std::int64_t>(Timing::Atime),
                                       static_cast<std::int64_t>(Timing::Astep)};
        } else if constexpr(ByTime) {
            return As7341Detail::codesFor(
              Kvasir::asDuration<std::chrono::microseconds>(Timing::IntegrationTime));
        } else {
            return As7341Detail::codesFor(std::chrono::milliseconds{50});
        }
    }();

    static_assert(Codes.atime >= 0 && Codes.atime <= 255,
                  "ATIME is one byte");
    static_assert(Codes.astep >= 0,
                  "Timing::IntegrationTime is below half the shortest step (1.39 us)");
    static_assert(Codes.atime + Codes.astep > 0,
                  "ATIME and ASTEP may not both be 0 (datasheet, ATIME register)");
    static_assert(Codes.astep <= 65534,
                  "ASTEP is 16 bits and 65535 is reserved (the longest integration time is "
                  "256 x 65535 x 2.78 us, 46.6 s)");

    static constexpr std::uint8_t  Atime = static_cast<std::uint8_t>(Codes.atime);
    static constexpr std::uint16_t Astep = static_cast<std::uint16_t>(Codes.astep);

    /// The integration time the codes give, rounded up to whole microseconds.
    static constexpr std::chrono::microseconds IntegrationTime
      = As7341Detail::integrationTime(Codes);

    /// The same in whole milliseconds, rounded up, for the waits.
    static constexpr auto IntegrationWait
      = std::chrono::ceil<std::chrono::milliseconds>(IntegrationTime);

    /// Before the first integration after SP_EN, the spectral engine auto-zeroes (AZ_CONFIG
    /// reset 255, "only before first measurement cycle"), 15 ms typical (AZ_CONFIG register
    /// text): each pass starts from SP_EN = 0, so each pass pays it, with margin.
    static constexpr auto AutoZeroWait = std::chrono::milliseconds{30};

    static constexpr std::uint8_t Pon    = 0x01;
    static constexpr std::uint8_t SpEn   = 0x02;
    static constexpr std::uint8_t SmuxEn = 0x10;

    /// The reference SMUX mappings, twenty bytes each.
    static constexpr std::array<std::uint8_t, 20> SmuxLow{0x30, 0x01, 0x00, 0x00, 0x00, 0x42, 0x00,
                                                          0x00, 0x50, 0x00, 0x00, 0x00, 0x20, 0x04,
                                                          0x00, 0x30, 0x01, 0x50, 0x00, 0x06};
    static constexpr std::array<std::uint8_t, 20> SmuxHigh{0x00, 0x00, 0x00, 0x40, 0x02, 0x00, 0x10,
                                                           0x03, 0x50, 0x10, 0x03, 0x00, 0x00, 0x00,
                                                           0x24, 0x00, 0x00, 0x50, 0x00, 0x06};

    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    static constexpr std::array Init{
      // CFG0: REG_BANK 0, so every register from 0x80 up is reachable -- earlier firmware may
      // have left it at 1 for 0x60..0x74 (CFG0 register) -- and LOW_POWER off
      Step::write({.reg = 0xA9, .payload = {0x00}}
      ),
      Step::write({.reg = 0x80, .payload = {Pon}}
      ), // power on
      Step::write({.reg = 0x81, .payload = {Atime}}
      ),
      Step::write({.reg     = 0xCA,
                   .payload = {static_cast<std::uint8_t>(Astep & 0xFF),
                               static_cast<std::uint8_t>(Astep >> 8)}}
      ), // ASTEP, low then high
      Step::write({.reg     = 0xAA,
                   .payload = {static_cast<std::uint8_t>(Gain)},
                   .delay   = std::chrono::milliseconds{10}}
      ), // CFG1: AGAIN
    };

    struct State {
        std::uint8_t deviceId{};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint8_t>(ids[0]);
        // the chip id sits in bits 7:2 and reads 0x09
    }

    struct Spectrum {
        static constexpr auto Period
          = 2 * (AutoZeroWait + IntegrationWait) + std::chrono::milliseconds{100};

        /// One pass: stop, tell CFG6 a SMUX write follows, the twenty bytes in three
        /// chunks, latch them, auto-zero and integrate, then collect the six ADCs. ENABLE is
        /// read back once SMUXEN has had its time, and STATUS2 before the ADCs: SMUXEN clears
        /// itself once the SMUX command is done (ENABLE register), AVALID says the
        /// measurement has completed (STATUS2), and decode() runs the pass again when either
        /// says not yet -- Adafruit's library polls both.
        static constexpr std::uint8_t EnableAt  = 24;
        static constexpr std::uint8_t Status2At = 26;

        static constexpr auto Steps = [] {
            std::array<Step, 20> s{};
            std::size_t          i    = 0;
            auto const           pass = [&](std::array<std::uint8_t, 20> const& smux,
                                            std::uint8_t                        into,
                                            std::uint8_t                        passIndex) {
                s[i++] = Step::write(
                  {.reg = 0x80, .payload = {Pon}});   // SP_EN low while the SMUX changes
                s[i++] = Step::write(
                  {.reg = 0xAF, .payload = {0x10}});   // CFG6: SMUX_CMD = write config
                s[i++] = Step::write({
                  .reg = 0x00,
                  .payload
                  = {smux[0], smux[1], smux[2], smux[3], smux[4], smux[5], smux[6], smux[7]}
                });
                s[i++] = Step::write({
                  .reg     = 0x08,
                  .payload = {smux[8],
                              smux[9],
                              smux[10],
                              smux[11],
                              smux[12],
                              smux[13],
                              smux[14],
                              smux[15]}
                });
                s[i++] = Step::write({
                  .reg     = 0x10,
                  .payload = {smux[16], smux[17], smux[18], smux[19]}
                });
                s[i++] = Step::write({.reg     = 0x80,
                                      .payload = {static_cast<std::uint8_t>(Pon | SmuxEn)},
                                      .delay   = std::chrono::milliseconds{5}});
                s[i++] = Step::read(
                  {.reg    = 0x80,
                   .count  = 1,
                   .offset = static_cast<std::uint8_t>(EnableAt + passIndex)});   // SMUXEN
                s[i++] = Step::write(
                  {.reg     = 0x80,
                   .payload = {static_cast<std::uint8_t>(Pon | SpEn)},
                   .delay   = AutoZeroWait + IntegrationWait + std::chrono::milliseconds{10}});
                s[i++] = Step::read(
                  {.reg    = 0xA3,
                   .count  = 1,
                   .offset = static_cast<std::uint8_t>(Status2At + passIndex)});   // AVALID
                s[i++] = Step::read({.reg = 0x95, .count = 12, .offset = into});
            };
            pass(SmuxLow, 0, 0);
            pass(SmuxHigh, 12, 1);
            return s;
        }();

        struct Sample {
            /// F1 415 nm, F2 445, F3 480, F4 515, F5 555, F6 590, F7 630, F8 680 nm.
            std::array<std::uint16_t, 8> f{};
            std::uint16_t                clear{};
            std::uint16_t                nir{};

            [[nodiscard]] constexpr std::uint16_t violet() const { return f[0]; }

            [[nodiscard]] constexpr std::uint16_t blue() const { return f[2]; }

            [[nodiscard]] constexpr std::uint16_t green() const { return f[4]; }

            [[nodiscard]] constexpr std::uint16_t red() const { return f[6]; }
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            bool const smuxDone = ((data.u8(EnableAt) | data.u8(EnableAt + 1)) & SmuxEn) == 0;
            bool const valid
              = (data.u8(Status2At) & 0x40U) != 0 && (data.u8(Status2At + 1) & 0x40U) != 0;
            if(!smuxDone || !valid) {
                return Outcome<Sample>::retry(std::chrono::milliseconds{10});
            }
            Sample sample{};
            for(std::size_t c = 0; c < 4; ++c) {
                sample.f[c]     = data.le16(2 * c);        // F1..F4 from the first pass
                sample.f[c + 4] = data.le16(12 + 2 * c);   // F5..F8 from the second
            }
            // clear and NIR are measured in both passes; the second is the fresher one
            sample.clear = data.le16(20);
            sample.nir   = data.le16(22);
            return Outcome<Sample>::ok(sample);
        }
    };

    /// AGAIN, so the range can be changed without rebuilding.
    struct GainSetting {
        using Value                          = As7341Detail::Gain;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Gain;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0xAA, .offset = 0, .count = 1});
        }
    };

    using Reads  = List<Spectrum>;
    using Writes = List<GainSetting>;
};

}   // namespace Kvasir::I2C::Chips
