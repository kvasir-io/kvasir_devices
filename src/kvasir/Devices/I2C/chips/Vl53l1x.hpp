#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {
namespace Vl53l1xDetail {
    /// The first register the default configuration is written to.
    constexpr std::uint16_t ConfigFirst = 0x002D;

    /// `VL51L1X_DEFAULT_CONFIGURATION` from ST's Ultra Lite Driver (STSW-IMG009,
    /// `VL53L1X_api.c`): registers 0x2D..0x87 in order, 91 bytes. Taken byte for byte from
    /// ULD 3.5.4 (the gschorcht/riot_st_vl53l1x_uld_api mirror, "Copyright (c) 2023
    /// STMicroelectronics") and checked identical to ULD 3.4.0 (linkjumper/VL53L1X_ULD_API)
    /// and to the copies Adafruit_VL53L1X and stm32duino/VL53L1X ship. SparkFun's copy
    /// differs in three bytes (0x2E and 0x2F = 0x01, pads pulled up to AVDD; 0x5F = 0xDB)
    /// and is not what this is.
    ///
    /// The bytes ST annotates:
    ///   0x2D  fast-plus mode off          0x2E, 0x2F  I2C and GPIO pads at 1.8 V
    ///   0x30  GPIO_HV_MUX__CTRL: bits 3:0 must be 0x1, bit 4 clear = active-high interrupt
    ///   0x31  GPIO__TIO_HV_STATUS         0x46  interrupt on "new sample ready"
    ///   0x64  sigma threshold (14.2 mm)   0x66  minimum count rate (9.7 MCPS)
    ///   0x6C  inter-measurement period (32 bit, oscillator units)
    ///   0x72, 0x74  distance thresholds   0x7F  ROI centre 199   0x80  ROI 16 x 16
    ///   0x86  interrupt clear             0x87  mode start (0 here: ranging off)
    constexpr std::array<std::uint8_t, 91> DefaultConfiguration{
      0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,   // 0x2D..0x34
      0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,   // 0x35..0x3C
      0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,   // 0x3D..0x44
      0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,   // 0x45..0x4C
      0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,   // 0x4D..0x54
      0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,   // 0x55..0x5C
      0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,   // 0x5D..0x64
      0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,   // 0x65..0x6C
      0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,   // 0x6D..0x74
      0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,   // 0x75..0x7C
      0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,   // 0x7D..0x84
      0x01, 0x00, 0x00,                                 // 0x85..0x87
    };

    /// A Step carries at most eight inline bytes (Step::InlineBytes, enforced by
    /// wellFormed()), and an Init script has no buffer to take a longer payload from, so the
    /// configuration goes out as twelve writes of consecutive registers.
    constexpr std::size_t ConfigChunks
      = (DefaultConfiguration.size() + Step::InlineBytes - 1) / Step::InlineBytes;

    constexpr Step configChunk(std::size_t i) {
        auto const first = i * Step::InlineBytes;
        auto const n     = std::min(Step::InlineBytes, DefaultConfiguration.size() - first);
        Step       step
          = Step::write({.reg = static_cast<std::uint16_t>(ConfigFirst + first), .payload = {}});
        step.count = static_cast<std::uint8_t>(n);
        for(std::size_t k = 0; k < n; ++k) { step.bytes[k] = DefaultConfiguration[first + k]; }
        return step;
    }

    /// ULD's `status_rtn`: RESULT__RANGE_STATUS bits 4:0 to the range status the ULD API
    /// reports. 255 is "no mapping".
    constexpr std::array<std::uint8_t, 24> StatusMap{255, 255, 255, 5,   2,   4,   1,   7,
                                                     3,   0,   255, 255, 9,   13,  255, 255,
                                                     255, 255, 10,  6,   255, 255, 11,  12};
}   // namespace Vl53l1xDetail

/// ST VL53L1X time-of-flight ranging sensor (the Adafruit STEMMA QT breakout among others),
/// 0x29. Sixteen-bit register indices, big-endian on the wire. There is no public register
/// map: ST's Ultra Lite Driver (STSW-IMG009, `VL53L1X_api.c` / `VL53L1X_api.h`) is the
/// reference, and every register and sequence here is the ULD's.
///
///   IDENTIFICATION__MODEL_ID 0x010F (16 bit, 0xEACC)   FIRMWARE__SYSTEM_STATUS 0x00E5
///   VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008       GPIO_HV_MUX__CTRL 0x0030
///   GPIO__TIO_HV_STATUS 0x0031                         SYSTEM__INTERRUPT_CLEAR 0x0086
///   SYSTEM__MODE_START 0x0087 (0x40 ranging, 0x00 stop) RESULT__RANGE_STATUS 0x0089
///   RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 0x008C      RESULT__AMBIENT_COUNT_RATE_MCPS_SD 0x0090
///   RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
///   RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 0x0098
///
/// The bring-up is ULD's `VL53L1X_SensorInit()` followed by `VL53L1X_StartRanging()`: the
/// 91-byte default configuration (Vl53l1xDetail::DefaultConfiguration, twelve writes), one
/// ranging to let the part run its VHV calibration, interrupt clear, stop, then
/// VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x09 ("two bounds VHV") and 0x000B = 0 ("start
/// VHV from the previous temperature"), and ranging started for good.
///
/// ULD waits for that first ranging by polling the data-ready flag. An Init script cannot
/// poll (a check step belongs to a read group), so the wait is a fixed `FirstRanging`:
/// twice the default 100 ms timing budget. The model ID and the boot flag are read before
/// any of it; setup() accepts the part when the ID is 0xEACC and FIRMWARE__SYSTEM_STATUS
/// bit 0 (ULD: "1: booted") is set. ULD polls that flag until the part has booted; here
/// `StartupDelay` stands in for the poll, and a part that has not booted NAKs the
/// configuration writes and is brought up again from the start.
///
/// Ranging runs continuously at the defaults the configuration carries: long distance mode,
/// 100 ms timing budget, 100 ms inter-measurement period (`TimingBudget`,
/// `InterMeasurement` below, which the Ranging period follows), full 16 x 16 SPAD ROI. A read is
/// ULD's `CheckForDataReady()` / `GetResult()` / `ClearInterrupt()`: GPIO_HV_MUX__CTRL and
/// GPIO__TIO_HV_STATUS in one two-byte read (the interrupt polarity is bit 4 of the first,
/// active high when clear, and the flag is bit 0 of the second), a check step that runs the
/// group again until the flag says a result is there, the seventeen result bytes
/// 0x0089..0x0099, and the interrupt clear that arms the next one.
///
/// Distance mode, timing budget and inter-measurement period are neither write groups nor
/// template parameters: ULD's setters rewrite eight registers from per-mode tables
/// (`VL53L1X_SetTimingBudgetInMs`) and scale the inter-measurement period by the oscillator
/// calibration read back from the part (`VL53L1X_SetInterMeasurementInMs`), which is a
/// driver rather than an Init script. They stay what the default configuration carries. The
/// carried inter-measurement register is a raw 0x0F89 in oscillator units, so the 100 ms is
/// nominal: the real period is off by as much as the part's OSC_CALIBRATE_VAL (0x00DE) is off
/// the value that figure assumes. The Ranging period below is a poll, not a lock to it.
/// The ROI is one register pair and is a write group.
///
/// Before the configuration the part is soft reset -- SOFT_RESET (0x0000) = 0, then 1 -- the
/// way ULD and Linux vl53l1x-i2c.c do it without an XSHUT line: a warm part is otherwise
/// still ranging with whatever earlier firmware configured. That comes after the model ID
/// and boot flag have been read and checked (Step::identify), so a part that only shares
/// 0x29 -- a TCS34725, a TSL2591 -- is not written to.
///
/// The part's address can be moved (I2C_SLAVE__DEVICE_ADDRESS 0x0001, volatile), but only by
/// talking to it at 0x29 first, which this description does not do.
struct Vl53l1x {
    static constexpr std::string_view Name = "VL53L1X";
    /// ST VL53L1X. VL53L1X.md:844..845, Model ID 0x010F = 0xEA and Module Type 0x0110 = 0xCC; the
    /// register address is two bytes.
    static constexpr std::array Identity{
      RegisterCheck{   "model-id", 0x010F, 1, true, 0xFF, 0xEA, RegisterCheck::None, 2},
      RegisterCheck{"module-type", 0x0110, 1, true, 0xFF, 0xCC, RegisterCheck::None, 2},
    };
    static constexpr Address7    Address       = 0x29;
    static constexpr std::size_t RegisterBytes = 2;

    static constexpr std::array<Address7, 1> Addresses{0x29};

    /// ULD polls the boot flag; the datasheet's boot time is well under this.
    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    /// What the default configuration carries (ULD's SensorInit defaults): the timing budget
    /// and the inter-measurement period, both 100 ms. Not changeable here, see above.
    static constexpr std::chrono::milliseconds TimingBudget{100};
    static constexpr std::chrono::milliseconds InterMeasurement{100};

    /// The first ranging, for the VHV calibration: two default timing budgets.
    static constexpr std::chrono::milliseconds FirstRanging = 2 * TimingBudget;

    static constexpr std::array<Step, 1 + Vl53l1xDetail::ConfigChunks + 11> Init = [] {
        std::array<Step, 1 + Vl53l1xDetail::ConfigChunks + 11> s{};
        std::size_t                                            n = 0;
        s[n++] = Step::read({.reg = 0x00E5, .count = 1, .offset = 0});   // FIRMWARE__SYSTEM_STATUS
        s[n++] = Step::identify();
        s[n++] = Step::write({.reg     = 0x0000,
                              .payload = {0x00},
                              .delay   = std::chrono::milliseconds{1}});   // SOFT_RESET
        s[n++] = Step::write({.reg = 0x0000, .payload = {0x01}, .delay = StartupDelay});   // boot
        for(std::size_t i = 0; i < Vl53l1xDetail::ConfigChunks; ++i) {
            s[n++] = Vl53l1xDetail::configChunk(i);   // 0x2D..0x87
        }
        s[n++] = Step::write({.reg = 0x0086, .payload = {0x01}});   // SYSTEM__INTERRUPT_CLEAR
        s[n++] = Step::write({.reg     = 0x0087,
                              .payload = {0x40},
                              .delay   = FirstRanging});   // SYSTEM__MODE_START: range once
        s[n++] = Step::write({.reg = 0x0086, .payload = {0x01}});
        s[n++] = Step::write({.reg = 0x0087, .payload = {0x00}});   // stop
        s[n++] = Step::write(
          {.reg = 0x0008, .payload = {0x09}});   // VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND
        s[n++]
          = Step::write({.reg = 0x000B, .payload = {0x00}});   // VHV from the previous temperature
        s[n++] = Step::write({.reg = 0x0086, .payload = {0x01}});
        s[n++] = Step::write({.reg = 0x0087, .payload = {0x40}});   // ranging, continuous
        return s;
    }();

    struct State {
        std::uint16_t modelId{};     ///< IDENTIFICATION__MODEL_ID (0x010F)
        std::uint8_t  bootState{};   ///< FIRMWARE__SYSTEM_STATUS (0x00E5)
    };

    /// `state().modelId` is model ID and module type as the engine read them for the Identity.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.modelId = static_cast<std::uint16_t>((ids[0] << 8U) | ids[1]);
    }

    /// The identity is the engine's; what is left is whether the firmware has booted.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.bootState = data.u8(0);
        return (state.bootState & 0x01U) != 0;
    }

    struct Ranging {
        /// One result per inter-measurement period (100 ms).
        static constexpr auto Period = InterMeasurement;

        /// Retried up to the engine's eight times, so a poll that lands just after the
        /// last clear still finds the next result (120 ms > one period) rather than being
        /// rejected.
        static constexpr std::chrono::milliseconds RetryAfter{15};

        static constexpr std::array Steps{
          Step::read(
            {.reg = 0x0030, .count = 2, .offset = 0}),   // GPIO_HV_MUX__CTRL, GPIO__TIO_HV_STATUS
          Step::check(RetryAfter),
          Step::read({.reg    = 0x0089,
                      .count  = 17,
                      .offset = 2}),   // RESULT__RANGE_STATUS .. peak signal rate
          Step::write({.reg = 0x0086, .payload = {0x01}}),   // SYSTEM__INTERRUPT_CLEAR
        };

        /// ULD `CheckForDataReady()`: the flag equals the polarity, which is active high
        /// when GPIO_HV_MUX__CTRL bit 4 is clear.
        [[nodiscard]] static constexpr bool ready(Bytes data) {
            bool const activeHigh = (data.u8(0) & 0x10U) == 0;
            bool const flag       = (data.u8(1) & 0x01U) != 0;
            return flag == activeHigh;
        }

        struct Sample {
            MilliMetre   distance{};
            std::uint8_t status{0xFF};   ///< ULD range status, see valid()
            /// The peak signal count rate (crosstalk corrected) and the ambient count rate.
            /// ULD reports them in kcps -- thousands of SPAD counts a second -- and a count a
            /// second is a hertz, so these are Hertz holding kcps x 1000: 1 kcps = 1000 Hz.
            Hertz signalRate{};
            Hertz ambientRate{};

            /// ULD range status 0: a good measurement. UM2510 names 1 sigma failure, 2
            /// signal failure, 4 phase out of bounds and 7 wraparound; the rest are ULD's
            /// status_rtn mapping as is. The distance of a frame with any other status is
            /// what the part reported but is not usable as a range: the sample carries it
            /// with the status, and this says whether it can be trusted.
            [[nodiscard]] constexpr bool valid() const { return status == 0; }
        };

        /// ULD `GetResult()` over the seventeen bytes from 0x0089 (at offset 2 here):
        /// status [0] bits 4:0, ambient [7..8] x 8, distance [13..14], signal [15..16] x 8.
        /// A raw status ULD has no mapping for (its 255) is a frame this description cannot
        /// interpret and is rejected.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            auto const result = data.sub(2, 17);
            auto const raw    = static_cast<std::uint8_t>(result.u8(0) & 0x1FU);
            auto const status = raw < Vl53l1xDetail::StatusMap.size()
                                ? Vl53l1xDetail::StatusMap[raw]
                                : std::uint8_t{0xFF};
            if(status == 0xFF) { return Outcome<Sample>::reject(); }
            Sample sample{};
            sample.status = status;
            sample.ambientRate
              = Units::hertz(static_cast<std::uint32_t>(result.be16(7)) * 8U * 1000U);
            sample.distance = Units::milliMetre(result.be16(13));
            sample.signalRate
              = Units::hertz(static_cast<std::uint32_t>(result.be16(15)) * 8U * 1000U);
            return Outcome<Sample>::ok(sample);
        }
    };

    /// ROI_CONFIG__USER_ROI_CENTRE_SPAD 0x007F and ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE
    /// 0x0080 in one write, as ULD `SetROI()` puts them: the size clamped to 4..16 SPADs a
    /// side (ULD: "the smallest acceptable ROI size = 4") and encoded (height - 1) << 4 |
    /// (width - 1), and the centre forced to 199 once either side is over 10. ULD takes the
    /// centre for a smaller ROI from ROI_CONFIG__MODE_ROI_CENTRE_SPAD 0x013E, the part's own
    /// optical centre; here it is the Value's, 199 unless the application says otherwise.
    struct Roi {
        struct Value {
            std::uint8_t width{16};
            std::uint8_t height{16};
            std::uint8_t centre{199};
        };

        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr std::uint8_t side(std::uint8_t spads) {
            return std::clamp(spads, std::uint8_t{4}, std::uint8_t{16});
        }

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            auto const w = side(value.width);
            auto const h = side(value.height);
            buffer[0]    = static_cast<std::byte>(w > 10 || h > 10 ? 199U : value.centre);
            buffer[1]    = static_cast<std::byte>(((h - 1U) << 4) | (w - 1U));
            return Step::writeBuffer({.reg = 0x007F, .offset = 0, .count = 2});
        }
    };

    using Reads  = List<Ranging>;
    using Writes = List<Roi>;
};

}   // namespace Kvasir::I2C::Chips
