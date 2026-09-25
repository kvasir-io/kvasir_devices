#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Iis2dulpxDetail {
    /// CTRL5 FS (1:0): +-2, +-4, +-8, +-16 g.
    enum class FullScale : std::uint8_t { g2 = 0, g4 = 1, g8 = 2, g16 = 3 };

    /// CTRL5 BW (3:2): the anti-aliasing filter at ODR/2, /4, /8, /16.
    enum class Bandwidth : std::uint8_t { div2 = 0, div4 = 1, div8 = 2, div16 = 3 };

    /// The ODR as ST's driver numbers it (`iis2dulpx_md_t::odr`): the low nibble is CTRL5's
    /// ODR field, bit 4 selects high performance (CTRL3 HP_EN), and 0x01..0x03 are the
    /// ultra-low-power rates.
    enum class Odr : std::uint8_t {
        off    = 0x00,
        ulp1_6 = 0x01,
        ulp3   = 0x02,
        ulp25  = 0x03,
        lp6    = 0x04,
        lp12_5 = 0x05,
        lp25   = 0x06,
        lp50   = 0x07,
        lp100  = 0x08,
        lp200  = 0x09,
        lp400  = 0x0A,
        lp800  = 0x0B,
        hp6    = 0x14,
        hp12_5 = 0x15,
        hp25   = 0x16,
        hp50   = 0x17,
        hp100  = 0x18,
        hp200  = 0x19,
        hp400  = 0x1A,
        hp800  = 0x1B,
    };

    /// CTRL5 with its BW field replaced by one the rate allows, the way ST's
    /// `iis2dulpx_mode_set` does: the ultra-low-power rates have no bandwidth selection (00),
    /// and in low-power mode below 50 Hz (Table 39) 6 Hz allows only 11, 12.5 Hz 10 or 11, and
    /// 25 Hz 01, 10 or 11 -- a narrower filter than asked for, never a wider one.
    /// `highPerformance` is CTRL3's HP_EN; the other rates take any BW.
    [[nodiscard]] constexpr std::uint8_t allowedCtrl5(std::uint8_t ctrl5,
                                                      bool         highPerformance) {
        auto const odr = static_cast<unsigned>(ctrl5 >> 4U);
        auto const bw  = static_cast<unsigned>((ctrl5 >> 2U) & 0x03U);
        auto       fit = bw;
        if(odr >= 0x1 && odr <= 0x3) {
            fit = 0;
        } else if(!highPerformance && odr == 0x4) {
            fit = 3;
        } else if(!highPerformance && odr == 0x5) {
            fit = bw < 2 ? 2 : bw;
        } else if(!highPerformance && odr == 0x6) {
            fit = bw < 1 ? 1 : bw;
        }
        return static_cast<std::uint8_t>((ctrl5 & 0xF3U) | (fit << 2U));
    }
}   // namespace Iis2dulpxDetail

/// STMicroelectronics IIS2DULPX ultra-low-power 3-axis accelerometer. One-byte pointer:
/// WHO_AM_I 0x0F (0x47), CTRL1 0x10 (WU_Z/Y/X_EN 2:0, DRDY_PULSED 3, IF_ADD_INC 4,
/// SW_RESET 5, INT1_ON_RES 6, SMART_POWER_EN 7), CTRL2 0x11 (interrupt routing), CTRL3 0x12
/// (ST_SIGN_X 0, ST_SIGN_Y 1, HP_EN 2, the INT2 routing above), CTRL4 0x13 (BOOT 0), CTRL5
/// 0x14 (FS 1:0, BW 3:2, ODR 7:4), STATUS 0x25, acceleration 0x28..0x2D little endian, the
/// temperature / analog-hub result at 0x2E..0x2F, and EN_DEVICE_CONFIG 0x3E (SOFT_PD bit 0,
/// the SPI way out of deep power-down).
/// Those eight output bytes are contiguous, so one burst read with IF_ADD_INC set collects
/// acceleration and temperature together.
///
/// The part powers up in deep power-down. On I2C the way out is its address alone: the part
/// NAKs it and starts a power-up of 25 ms at most, after which it acknowledges (3.3.1.1;
/// SOFT_PD is the SPI way, 3.3.1.2). So the bring-up starts with the WHO_AM_I read, whose NAKs
/// WakeRetries puts on the wire again until the part is up, then resets, and sets
/// IF_ADD_INC, HP_EN when the ODR is a high-performance one (CTRL3 bit 2, as ST's
/// `iis2dulpx_mode_set` sets it for an ODR code with bit 4 set), BDU (CTRL4 bit 5, as ST's
/// `iis2dulpx_init_set` sets it; it takes effect in the low-power and ultra-low-power modes,
/// note 1 of Table 34), and CTRL5 with a bandwidth the rate allows (allowedCtrl5).
///
/// A read takes STATUS first: DRDY (bit 0) is 1 from a new sample until an output MSB has been
/// read (Table 68), so a frame without it -- the slow rates' polls between samples, and the
/// first poll after CTRL5, before the first sample -- is Outcome::unchanged(), not a
/// sample.
///
/// 0x18, or 0x19 with SA0 high. (ST's own header states the 8-bit forms, 0x31 and 0x33.)
///
/// Sensitivity is 0.061, 0.122, 0.244 and 0.488 mg/LSB for the four full scales (Table 2),
/// and temperature 0.045 degC per LSB of the 12-bit value with 0 at 25 degC (Table 5), which is
/// `lsb / 355.5 + 25` degC for the left-justified 16-bit word.
///
/// The finite state machine, machine learning core, FIFO and Qvar sensing channel are not
/// driven here. They are separately programmed engines with their own memory banks, which
/// is a driver rather than a description; plain acceleration and temperature are cyclic and
/// belong here.
///
/// The Motion group's period follows from the ODR (OdrPeriod), and from a rate written to
/// Config at run time too. Config writes CTRL5 only: a run-time change between low-power and
/// high-performance rates would also need CTRL3's HP_EN, which stays as the bring-up set it.
template<Iis2dulpxDetail::FullScale Fs      = Iis2dulpxDetail::FullScale::g2,
         Iis2dulpxDetail::Odr       OdrCode = Iis2dulpxDetail::Odr::hp100,
         Iis2dulpxDetail::Bandwidth Bw      = Iis2dulpxDetail::Bandwidth::div2>
struct Iis2dulpx {
    static constexpr std::string_view Name = "IIS2DULPX";
    /// ST IIS2DULPX. IIS2DULPX.md:1601, WHO_AM_I (0Fh) "fixed at 47h".
    /// Configuration: CTRL4 (13h) BDU set, so the two bytes of a sample belong together
    /// (IIS2DULPX.md:1713..1725).
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0F, 1, true, 0xFF, 0x47},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"bdu", 0x13, 1, true, 0x20, 0x20},
    };
    static constexpr Address7    Address       = 0x18;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 2> Addresses{0x18, 0x19};

    static constexpr unsigned FsCode = static_cast<unsigned>(Fs);
    static constexpr unsigned OdrRaw = static_cast<unsigned>(OdrCode);
    static constexpr unsigned BwCode = static_cast<unsigned>(Bw);

    static_assert((OdrRaw & 0x0FU) >= 0x01 && (OdrRaw & 0x0FU) <= 0x0B,
                  "an ODR field from 0x1 (1.6 Hz, ultra low power) to 0xB (800 Hz); 0 is off");

    /// The sampling period each ODR field asks for, rounded up to whole milliseconds so the
    /// part is never read faster than it converts: off, the ultra-low-power 1.6, 3 and 25 Hz,
    /// then 6, 12.5, 25, 50, 100, 200, 400 and 800 Hz.
    static constexpr std::array<std::chrono::milliseconds, 12> OdrPeriod{
      std::chrono::milliseconds{0},
      std::chrono::milliseconds{625},
      std::chrono::milliseconds{334},
      std::chrono::milliseconds{40},
      std::chrono::milliseconds{167},
      std::chrono::milliseconds{80},
      std::chrono::milliseconds{40},
      std::chrono::milliseconds{20},
      std::chrono::milliseconds{10},
      std::chrono::milliseconds{5},
      std::chrono::milliseconds{3},
      std::chrono::milliseconds{2}};

    static constexpr std::chrono::milliseconds OutputPeriod = OdrPeriod[OdrRaw & 0x0FU];

    /// Per LSB: 0.061, 0.122, 0.244 and 0.488 mg.
    static constexpr std::array<MicroG, 4> AccelPerCount{Units::microG(61),
                                                         Units::microG(122),
                                                         Units::microG(244),
                                                         Units::microG(488)};

    /// IF_ADD_INC, so a burst read walks the output registers.
    static constexpr std::uint8_t Ctrl1 = 0x10;
    /// ST puts the mode bits above the four ODR bits: 0x10 in the code is high performance,
    /// which is CTRL3's HP_EN (bit 2).
    static constexpr bool         HighPerformance = (OdrRaw & 0x30U) == 0x10U;
    static constexpr std::uint8_t Ctrl5           = Iis2dulpxDetail::allowedCtrl5(
      static_cast<std::uint8_t>(((OdrRaw & 0x0FU) << 4) | (BwCode << 2) | FsCode),
      HighPerformance);
    static constexpr std::uint8_t Ctrl3 = HighPerformance ? 0x04 : 0x00;

    static constexpr auto StartupDelay = std::chrono::milliseconds{25};

    /// Out of deep power-down the part NAKs its address and powers up, 25 ms at most.
    static constexpr std::uint8_t WakeRetries    = 5;
    static constexpr auto         WakeRetryDelay = std::chrono::milliseconds{10};

    static constexpr std::array Init{
      Step::write({.reg     = 0x10,
                   .payload = {0x20},
                   .delay   = std::chrono::milliseconds{25}}),   // SW_RESET, then the boot time
      Step::write({.reg = 0x10, .payload = {Ctrl1}}),            // IF_ADD_INC for the burst read
      Step::write({.reg = 0x12, .payload = {Ctrl3}}),            // HP_EN for a high-performance ODR
      Step::write({.reg = 0x13, .payload = {0x20}}),             // CTRL4: BDU
      Step::write({.reg     = 0x14,
                   .payload = {Ctrl5},
                   .delay   = std::chrono::milliseconds{25}}),   // ODR, bandwidth and full scale
    };

    struct State {
        std::uint8_t deviceId{};   ///< WHO_AM_I (0x0F)
        std::uint8_t fs{FsCode};   ///< the full scale CTRL5 holds: what decode() multiplies by
        std::uint8_t odr{static_cast<std::uint8_t>(OdrRaw & 0x0FU)};   ///< and its ODR field
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint8_t>(ids[0]);
        state.fs       = static_cast<std::uint8_t>(FsCode);   // what the Init script wrote
        state.odr      = static_cast<std::uint8_t>(OdrRaw & 0x0FU);
    }

    struct Motion {
        static constexpr auto Period = OutputPeriod;
        /// The bus completion of every sample is kept: an accelerometer on a fast period is
        /// the part whose missed ticks are worth counting (Device::takeGaps<Motion>()).
        static constexpr bool Timestamped = true;

        /// The rate CTRL5 holds now, which a Config write changes; `Period` is the template's,
        /// which the bus load and the nominal rates are computed from.
        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            auto const field = static_cast<std::size_t>(state.odr & 0x0FU);
            auto const p
              = field < OdrPeriod.size() ? OdrPeriod[field] : std::chrono::milliseconds::zero();
            return p != std::chrono::milliseconds::zero() ? p : OutputPeriod;
        }

        /// STATUS, then 0x28..0x2F in one go: three axes then the temperature word.
        static constexpr std::array Steps{Step::read({.reg = 0x25, .count = 1, .offset = 8}),
                                          Step::read({.reg = 0x28, .count = 8, .offset = 0})};

        struct Sample {
            MicroG    x{};
            MicroG    y{};
            MicroG    z{};
            MilliDegC temperature{};   ///< 0.001 degC
        };

        [[nodiscard]] static constexpr std::int32_t accel(std::int16_t raw,
                                                          unsigned     fs) {
            return static_cast<std::int32_t>(static_cast<std::int64_t>(raw)
                                             * Units::value(AccelPerCount[fs & 0x03U]));
        }

        /// At the full scale the part holds now, which a Config write changes (applied()).
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if((data.u8(8) & 0x01U) == 0) { return Outcome<Sample>::unchanged(); }   // DRDY
            Sample sample{};
            sample.x = Units::microG(accel(data.s16le(0), state.fs));
            sample.y = Units::microG(accel(data.s16le(2), state.fs));
            sample.z = Units::microG(accel(data.s16le(4), state.fs));
            // lsb / 355.5 + 25 degC, in millidegrees and without the float
            sample.temperature = Units::milliDegC(
              static_cast<std::int32_t>(static_cast<std::int64_t>(data.s16le(6)) * 10000 / 3555)
              + 25000);
            return Outcome<Sample>::ok(sample);
        }
    };

    /// CTRL5, so the range or rate can be changed at run time. A new full scale is what decode()
    /// multiplies by from the write's completion on, and a new rate the period Motion is read at.
    struct Config {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Ctrl5;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0]
              = static_cast<std::byte>(Iis2dulpxDetail::allowedCtrl5(value, HighPerformance));
            return Step::writeBuffer({.reg = 0x14, .offset = 0, .count = 1});
        }

        /// The full scale is FS, bits 1:0; the rate ODR, bits 7:4.
        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.fs  = static_cast<std::uint8_t>(value & 0x03U);
            state.odr = static_cast<std::uint8_t>(value >> 4U);
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<Config>;
};

}   // namespace Kvasir::I2C::Chips
