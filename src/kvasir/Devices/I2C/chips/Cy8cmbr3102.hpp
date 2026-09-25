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

namespace Cy8cmbr3102Detail {
    /// CSx_SENSITIVITY in SENSITIVITY0 (TRM 1.5.5): 50 counts per 0.1, 0.2, 0.3 or 0.4 pF.
    enum class Sensitivity : std::uint8_t {
        counts500PerPf = 0,
        counts250PerPf = 1,
        counts167PerPf = 2,
        counts125PerPf = 3,
    };

    /// PROXIMITY_ARST and BUTTON_SLD_ARST in DEVICE_CFG2 (TRM 1.5.60): the sensor's baseline
    /// is reset to its raw count after it has been "on" this long.
    enum class AutoReset : std::uint8_t { off = 0, after5s = 1, after20s = 2 };

    /// SPO0 in SPO_CFG (TRM 1.5.57). On the CY8CMBR3102 SPO0 is the CS1 pin (TRM 1.5.1).
    enum class Spo0 : std::uint8_t {
        disabled      = 0,
        sensor        = 1,
        shield        = 2,
        buzzer        = 3,
        hostInterrupt = 4,
        gpo           = 5,
    };

    /// CMD_OP_CODE values for CTRL_CMD (TRM 1.5.80) that are commands rather than the
    /// configuration save, which is Provision's.
    enum class Operation : std::uint8_t {
        deepSleep    = 7,     ///< stop scanning; any address match wakes the part
        clearLatched = 8,     ///< LATCHED_BUTTON_STAT / LATCHED_PROX_STAT to 0
        resetAlpPs0  = 9,     ///< reset the advanced low-pass filter of proximity sensor 0
        resetAlpPs1  = 10,    ///< the same for proximity sensor 1
        reset        = 255,   ///< software reset: back through Boot
    };

    /// CTRL_CMD_ERR (TRM 1.5.82): what the last command came to. The log prints it by name.
    enum class CommandError : std::uint8_t {
        none             = 0,
        flashWriteFailed = 253,
        crcMismatch      = 254,
        invalidCommand   = 255,
    };

    /// The configuration CRC. The TRM says only "CCITT CRC16 checksum for all data from offset
    /// 0 to 125" (1.5.77) and prints no example. Seed 0xFFFF and polynomial 0x1021, MSB first,
    /// no final XOR, fed a nibble at a time high nibble first, is Cypress's host API
    /// (CY8CMBR3xxx_CalculateCrc in its CY8CMBR3xxx_CRC.c) -- verify on a part: a Provision whose
    /// CRC the part disagrees with is refused with CTRL_CMD_ERR 254, which the next bring-up
    /// reports as State::lastError == CommandError::crcMismatch. The nibble step multiplies
    /// the polynomial by the index; 0x1021 * i for i < 16 has no carries, so it is the
    /// carry-less product and the whole is CRC-16/CCITT-FALSE, whose published check value
    /// over "123456789" is 0x29B1 (asserted below).
    [[nodiscard]] constexpr std::uint16_t crc16(std::span<std::uint8_t const> data) {
        auto const nibble = [](std::uint8_t value, std::uint16_t remainder) {
            auto const index = static_cast<std::uint16_t>((value & 0x0FU) ^ (remainder >> 12U));
            return static_cast<std::uint16_t>((0x1021U * index)
                                              ^ (static_cast<unsigned>(remainder) << 4U));
        };
        std::uint16_t seed = 0xFFFF;
        for(auto const byte : data) {
            seed = nibble(static_cast<std::uint8_t>(byte >> 4U), seed);
            seed = nibble(byte, seed);
        }
        return seed;
    }

    static_assert(crc16(std::array<std::uint8_t,
                                   9>{'1',
                                      '2',
                                      '3',
                                      '4',
                                      '5',
                                      '6',
                                      '7',
                                      '8',
                                      '9'})
                  == 0x29B1);

    /// A moisture reading, 0..100 %, from a raw count between the application's own dry and
    /// wet references (the plate in air and in water, or dry and saturated soil). Linear and
    /// clamped; either reference may be the larger. The part knows nothing of soil: this is a
    /// calibration the application owns, measures on its own plate and stores itself.
    [[nodiscard]] constexpr Units::Percent moisture(std::uint16_t raw,
                                                    std::uint16_t dry,
                                                    std::uint16_t wet) {
        if(dry == wet) { return Units::percent(0U); }
        auto const rising = wet > dry;
        auto const lo     = rising ? dry : wet;
        auto const hi     = rising ? wet : dry;
        auto const r      = raw < lo ? lo : (raw > hi ? hi : raw);
        auto const span   = static_cast<std::uint32_t>(hi - lo);
        auto const along  = static_cast<std::uint32_t>(rising ? r - lo : hi - r);
        return Units::percent((along * 100U + span / 2U) / span);
    }
}   // namespace Cy8cmbr3102Detail

/// Infineon (Cypress) CY8CMBR3102 CapSense Express controller, two capacitive inputs, as on
/// the SparkFun Qwiic Soil Moisture Sensor (SEN-30480: one plate on CS0, the LED on GPO0).
/// Datasheet 001-85330 Rev *Q ("DS" below) and the registers TRM 001-91082 Rev *E ("TRM").
///
/// **Register map** (TRM 1.5, little endian throughout, 1.3). 0x00..0x7F is the configuration
/// (0x00..0x7D) and its CRC (CONFIG_CRC 0x7E, 1.5.77); 0x80..0x87 take commands, executed
/// within TI2C_LATENCY_MAX = 50 ms of the ACK (DS AC table); 0x88..0xFB report status. Used
/// here: SENSOR_ID 0x82 (1.5.79), CTRL_CMD 0x86 (1.5.80), CTRL_CMD_ERR 0x89 (1.5.82),
/// SYSTEM_STATUS 0x8A (1.5.83), FAMILY_ID 0x8F = 154 = 0x9A (1.5.85), DEVICE_ID 0x90 (1.5.86;
/// 2561 = 0x0A01 for the 3102 in 1.4.1, where the converted table splits it as "256|1"), and
/// the debug block SYNC_COUNTER1 0xDB, DEBUG_SENSOR_ID 0xDC, DEBUG_CP 0xDD,
/// DEBUG_DIFFERENCE_COUNT0 0xDE, DEBUG_BASELINE0 0xE0, DEBUG_RAW_COUNT0 0xE2,
/// DEBUG_AVG_RAW_COUNT0 0xE4, SYNC_COUNTER2 0xE7 (1.5.121..1.5.128).
///
/// **The configuration lives in flash, and nothing here writes it on its own.** A register
/// written in 0x00..0x7E takes effect only after CTRL_CMD 2 (SAVE_CHECK_CRC: the part computes
/// the CRC over 0..125, compares it with CONFIG_CRC and saves both if they match) and a reset
/// (TRM 1.5, 1.5.80). The whole 128-byte block this description wants is `Configuration`,
/// built at compile time from the template parameters with its CRC; the bring-up reads the
/// stored CONFIG_CRC and `state().configured` says whether the part already holds exactly
/// that. When it does not, the application requests `Provision` once (`set<Provision>(0)`):
/// it writes the block and CRC, saves (DS: NAKs until done, 220 ms typ) and resets. It is
/// Transient -- never replayed after a reset -- because every save spends a flash write.
/// SparkFun's library rewrites and saves on every begin(); this does not.
///
/// **It NAKs while it wakes.** In any low-power state the part NAKs the address match, wakes
/// on it, and NAKs until it is active; the host "is expected to retry the transaction until it
/// receives an ACK" (DS, I2C Communication Guidelines 2). `WakeRetries` puts a NAKed
/// transaction on the wire again five times (SparkFun's count) before it counts as a NAK.
/// The datasheet gives no wake-up time; the 10 ms between tries is not a datasheet figure.
///
/// **Reading.** SENSOR_ID selects the sensor the DEBUG_ registers report; it is 255 (none)
/// after every reset. `Moisture` writes it and reads the block 0xDB..0xE7 in one burst. The
/// data between SYNC_COUNTER1 and SYNC_COUNTER2 is consistent only when the two are equal
/// (1.5.128): decode asks for a retry otherwise, and also while DEBUG_SENSOR_ID does not yet
/// name the sensor (after a reset, or within the 50 ms a command register takes). DEBUG_CP,
/// the total capacitance in whole pF, is described as "updated on each scan refresh" and, in
/// the same section, as "updated whenever there is a change in value of SENSOR_ID" (1.5.123
/// -- verify). SparkFun selects another sensor and back before reading it; `Capacitance` is
/// that sequence, on demand, for when the pF figure matters. The raw count is the signal a
/// soil probe wants; `moisture()` turns it into a percentage against the application's own
/// dry and wet counts.
///
/// **Defaults** are SparkFun's defaultMoistureSensorInit() on top of the 3102's factory
/// defaults (TRM 1.4.1, which its saveDefaultConfig() table matches byte for byte): CS0 on, CS1
/// off and SPO0 (the CS1 pin) a host-controlled GPO driving the LED (GPO_CFG 0x0D: host control,
/// DC, strong drive, active high), 500 counts/pF, 100 ms refresh, button and proximity auto
/// reset after 5 s, automatic thresholds, IIR and median filters, system diagnostics on,
/// address 0x37 (DS, I2C Slave Address; I2C_ADDR 1.5.62 takes 8..119). Registers the 3102
/// does not have (FINGER_THRESHOLD2..15, sliders, buzzer, the gaps) are written as 0; the CRC
/// covers whatever the block holds, so their value only has to be consistent.
///
/// `Addr` is both the address this description talks to and the one the block stores. Only
/// `SetAddress` changes the part's address, and after it the *type* has to change too.
/// `Timing` may name `Refresh`, the REFRESH_INTERVAL written to the block (100 ms), and
/// `Period`, how often Moisture is read (the refresh).
template<Address7                       Addr     = 0x37,
         std::uint8_t                   SensorId = 0,
         Cy8cmbr3102Detail::Sensitivity Sens     = Cy8cmbr3102Detail::Sensitivity::counts500PerPf,
         Cy8cmbr3102Detail::AutoReset   Arst     = Cy8cmbr3102Detail::AutoReset::after5s,
         Cy8cmbr3102Detail::Spo0        Spo0Mode = Cy8cmbr3102Detail::Spo0::gpo,
         typename Timing                         = DefaultTiming>
struct Cy8cmbr3102 {
    static constexpr std::string_view        Name          = "CY8CMBR3102";
    static constexpr Address7                Address       = Addr;
    static constexpr std::size_t             RegisterBytes = 1;
    static constexpr std::array<Address7, 1> Addresses{Addr};

    /// TI2CBOOT: power to I2C ready, 15 ms max (DS AC table).
    static constexpr auto StartupDelay = std::chrono::milliseconds{15};

    /// A NAK is put on the wire again this often before it counts (DS guideline 2).
    static constexpr std::uint8_t WakeRetries    = 5;
    static constexpr auto         WakeRetryDelay = std::chrono::milliseconds{10};

    using Sensitivity  = Cy8cmbr3102Detail::Sensitivity;
    using AutoReset    = Cy8cmbr3102Detail::AutoReset;
    using Spo0         = Cy8cmbr3102Detail::Spo0;
    using Operation    = Cy8cmbr3102Detail::Operation;
    using CommandError = Cy8cmbr3102Detail::CommandError;

    static_assert(SensorId <= 1,
                  "the CY8CMBR3102 has CS0 and CS1");
    static_assert(SensorId == 0 || Spo0Mode == Spo0::sensor,
                  "CS1 is the SPO0 pin: it is a sensor only with SPO0 set to sensor (TRM 1.5.1)");
    static constexpr std::chrono::milliseconds Refresh = [] {
        if constexpr(requires { Timing::Refresh; }) {
            return Kvasir::asDuration(Timing::Refresh);
        } else {
            return std::chrono::milliseconds{100};
        }
    }();
    static constexpr std::chrono::milliseconds ReadPeriod = [] {
        if constexpr(requires { Timing::Period; }) {
            return Kvasir::asDuration(Timing::Period);
        } else {
            return Refresh;
        }
    }();

    static_assert(Refresh >= std::chrono::milliseconds{20}
                    && Refresh <= std::chrono::milliseconds{500}
                    && Refresh % std::chrono::milliseconds{20} == std::chrono::milliseconds::zero(),
                  "REFRESH_INTERVAL is 1..25 steps of 20 ms (TRM 1.5.63)");
    static_assert(ReadPeriod > std::chrono::milliseconds::zero(),
                  "a cyclic read needs a period");

    // -- registers --------------------------------------------------------------------------
    static constexpr std::uint8_t RegSensorEn     = 0x00;
    static constexpr std::uint8_t RegSensitivity0 = 0x08;
    static constexpr std::uint8_t RegGpoCfg       = 0x40;
    static constexpr std::uint8_t RegSpoCfg       = 0x4C;
    static constexpr std::uint8_t RegDeviceCfg2   = 0x4F;
    static constexpr std::uint8_t RegI2cAddr      = 0x51;
    static constexpr std::uint8_t RegRefreshCtrl  = 0x52;
    static constexpr std::uint8_t RegConfigCrc    = 0x7E;
    static constexpr std::uint8_t RegSensorId     = 0x82;
    static constexpr std::uint8_t RegCtrlCmd      = 0x86;
    static constexpr std::uint8_t RegCtrlCmdErr   = 0x89;
    static constexpr std::uint8_t RegFamilyId     = 0x8F;
    static constexpr std::uint8_t RegDebugBlock   = 0xDB;   ///< SYNC_COUNTER1 .. SYNC_COUNTER2

    static constexpr std::uint8_t  FamilyId = 0x9A;     ///< 154 (TRM 1.5.85)
    static constexpr std::uint16_t DeviceId = 0x0A01;   ///< 2561, the 3102 (TRM 1.4.1)

    static constexpr std::uint8_t CmdSaveCheckCrc = 2;   ///< TRM 1.5.80 (3 is CALC_CRC)

    /// 220 ms typ for the save (DS note 20, guideline 4), no maximum given: the margin is ours.
    static constexpr std::chrono::milliseconds SaveTime{500};
    /// TBOOT_SYS, reset to first scan with system diagnostics on, 900 ms max (DS AC table).
    static constexpr std::chrono::milliseconds BootTime{900};
    /// TI2C_LATENCY_MAX (DS AC table).
    static constexpr std::chrono::milliseconds CommandLatency{50};
    /// STATE_TIMEOUT, written to the configuration block in whole seconds (TRM 1.5.64).
    static constexpr std::chrono::seconds StateTimeout{10};

    static constexpr std::size_t ConfigBytes = 126;   ///< offsets 0..125, under the CRC
    static constexpr std::size_t BlockBytes  = 128;   ///< and CONFIG_CRC behind them

    using Block = std::array<std::uint8_t, BlockBytes>;

    /// The configuration block for an address, CRC included.
    [[nodiscard]] static constexpr Block configurationFor(std::uint8_t address) {
        Block      b{};
        auto const le16 = [&](std::size_t at, std::uint16_t v) {
            b[at]     = static_cast<std::uint8_t>(v & 0xFFU);
            b[at + 1] = static_cast<std::uint8_t>(v >> 8U);
        };
        le16(RegSensorEn, static_cast<std::uint16_t>(1U << SensorId));   // SENSOR_EN 1.5.1
        // FSS_EN 0x02, TOGGLE_EN 0x04, LED_ON_EN 0x06: 0
        b[RegSensitivity0]
          = static_cast<std::uint8_t>(static_cast<unsigned>(Sens) << (2U * SensorId));
        b[0x0C] = 128;     // BASE_THRESHOLD0 (1.5.9)
        b[0x0D] = 128;     // BASE_THRESHOLD1
        b[0x1C] = 3;       // SENSOR_DEBOUNCE (1.5.25)
        b[0x1D] = 12;      // BUTTON_HYS (1.5.26)
        b[0x1F] = 50;      // BUTTON_LBR
        b[0x20] = 51;      // BUTTON_NNT
        b[0x21] = 51;      // BUTTON_NT
        b[0x27] = 0x80;    // PROX_CFG: ALP filter enabled (1.5.31)
        b[0x28] = 5;       // PROX_CFG2: ALP_FILTER_K medium
        le16(0x2A, 512);   // PROX_TOUCH_TH0 (1.5.33)
        le16(0x2C, 512);   // PROX_TOUCH_TH1
        b[0x30] = 5;       // PROX_HYS
        b[0x32] = 50;      // PROX_LBR
        b[0x33] = 20;      // PROX_NNT
        b[0x34] = 20;      // PROX_NT
        b[0x35] = 30;      // PROX_POSITIVE_TH0
        b[0x36] = 30;      // PROX_POSITIVE_TH1
        b[0x39] = 30;      // PROX_NEGATIVE_TH0
        b[0x3A] = 30;      // PROX_NEGATIVE_TH1
        // GPO_CFG (1.5.48): host-controlled DC, strong drive, active high while SPO0 is a GPO
        b[RegGpoCfg] = Spo0Mode == Spo0::gpo ? std::uint8_t{0x0D} : std::uint8_t{0x00};
        b[0x41]      = 0x0F;   // PWM_DUTYCYCLE_CFG0 (1.5.49)
        b[RegSpoCfg] = static_cast<std::uint8_t>(Spo0Mode);
        b[0x4D]      = 0x03;   // DEVICE_CFG0: IIR and median filters (1.5.58)
        b[0x4E]      = 0x01;   // DEVICE_CFG1: system diagnostics (1.5.59)
        // DEVICE_CFG2 (1.5.60): both auto-resets, automatic thresholds (ATH_EN, bit 3)
        b[RegDeviceCfg2] = static_cast<std::uint8_t>((static_cast<unsigned>(Arst) << 6U)
                                                     | (static_cast<unsigned>(Arst) << 4U) | 0x08U);
        // DEVICE_CFG3 0x50: 0, the internally regulated 1.8..5.5 V supply (1.5.61)
        b[RegI2cAddr] = static_cast<std::uint8_t>(address & 0x7FU);
        b[RegRefreshCtrl]
          = static_cast<std::uint8_t>(Refresh / std::chrono::milliseconds{20});        // 1.5.63
        b[0x55] = static_cast<std::uint8_t>(StateTimeout / std::chrono::seconds{1});   // 1.5.64
        le16(RegConfigCrc, crc(b));
        return b;
    }

    /// The CRC of a block's configuration bytes (0..125).
    [[nodiscard]] static constexpr std::uint16_t crc(Block const& block) {
        return Cy8cmbr3102Detail::crc16(std::span<std::uint8_t const>{block}.first(ConfigBytes));
    }

    static constexpr Block         Configuration = configurationFor(Addr);
    static constexpr std::uint16_t ConfigCrc     = crc(Configuration);

    /// CTRL_CMD_ERR and SYSTEM_STATUS, FAMILY_ID and DEVICE_ID, then CONFIG_CRC.
    static constexpr std::array Init{
      Step::read({.reg = RegCtrlCmdErr, .count = 2, .offset = 0}),
      Step::read({.reg = RegFamilyId, .count = 3, .offset = 2}),
      Step::read({.reg = RegConfigCrc, .count = 2, .offset = 5}),
    };

    struct State {
        std::uint8_t  familyId{};
        std::uint16_t deviceId{};
        std::uint16_t storedCrc{};                     ///< CONFIG_CRC as the part holds it
        CommandError  lastError{CommandError::none};   ///< CTRL_CMD_ERR at the bring-up
        bool          factoryDefaults{};               ///< SYSTEM_STATUS F_DEFAULT
        bool          configured{};   ///< the part holds Configuration: nothing to provision
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.lastError       = static_cast<CommandError>(data.u8(0));
        state.factoryDefaults = (data.u8(1) & 0x01U) != 0;
        state.familyId        = data.u8(2);
        state.deviceId        = data.le16(3);
        state.storedCrc       = data.le16(5);
        state.configured      = state.storedCrc == ConfigCrc;
        return state.familyId == FamilyId && state.deviceId == DeviceId;
    }

    /// What the debug block says about the selected sensor.
    struct Sample {
        Units::PicoFarad capacitance{};   ///< DEBUG_CP, whole pF (see Capacitance)
        std::uint16_t    raw{};           ///< DEBUG_RAW_COUNT0
        std::uint16_t    baseline{};      ///< DEBUG_BASELINE0
        std::uint16_t    difference{};    ///< DEBUG_DIFFERENCE_COUNT0

        /// 0..100 % between the application's dry and wet raw counts (Detail::moisture).
        [[nodiscard]] constexpr Units::Percent moisture(std::uint16_t dry,
                                                        std::uint16_t wet) const {
            return Cy8cmbr3102Detail::moisture(raw, dry, wet);
        }
    };

    /// The 13 bytes from SYNC_COUNTER1: counter, DEBUG_SENSOR_ID, DEBUG_CP, difference,
    /// baseline, raw and average raw (LE16 each), 0xE6, SYNC_COUNTER2.
    static constexpr std::uint8_t DebugBytes = 13;

    [[nodiscard]] static constexpr Outcome<Sample> decodeDebug(Bytes data) {
        if((data.u8(0) & 0x0FU) != (data.u8(12) & 0x0FU)) {
            return Outcome<Sample>::retry(
              std::chrono::milliseconds{5});   // the part updated the block under the read
        }
        if(data.u8(1) != SensorId) {
            return Outcome<Sample>::retry(CommandLatency);   // SENSOR_ID not taken yet
        }
        return Outcome<Sample>::ok({.capacitance = Units::picoFarad(data.u8(2)),
                                    .raw         = data.le16(7),
                                    .baseline    = data.le16(5),
                                    .difference  = data.le16(3)});
    }

    /// Cyclic: select the sensor (idempotent, and what brings the selection back after a
    /// reset) and read the debug block.
    struct Moisture {
        static constexpr auto       Period = ReadPeriod;
        static constexpr std::array Steps{
          Step::write({.reg = RegSensorId, .payload = {SensorId}}),
          Step::read({.reg = RegDebugBlock, .count = DebugBytes, .offset = 0}),
        };

        using Sample = Cy8cmbr3102::Sample;

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            return decodeDebug(data);
        }
    };

    /// On demand: SparkFun's way to a fresh DEBUG_CP -- select the other sensor, then this
    /// one, each given the command latency, then the block. A disabled sensor is a valid
    /// SENSOR_ID (0 .. sensors - 1, TRM 1.5.79).
    struct Capacitance {
        static constexpr std::array Steps{
          Step::write({.reg     = RegSensorId,
                       .payload = {static_cast<std::uint8_t>(SensorId == 0 ? 1 : 0)},
                       .delay   = CommandLatency}),
          Step::write({.reg = RegSensorId, .payload = {SensorId}, .delay = CommandLatency}),
          Step::read({.reg = RegDebugBlock, .count = DebugBytes, .offset = 0}),
        };

        using Sample = Cy8cmbr3102::Sample;

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            return decodeDebug(data);
        }
    };

    /// The block and CRC, SAVE_CHECK_CRC, then a software reset (TRM 1.5, 1.5.80). The Value is
    /// ignored. Transient: a flash write, never replayed. Whether the save took is not known
    /// from the write: Cypress' host API and SparkFun's library both read CTRL_CMD and
    /// CTRL_CMD_ERR after the save before they reset, and a write script cannot. A refused save
    /// leaves the flash as it was, which the reset then loads again, so the reset does no harm;
    /// `request<Stored>()` after the write reads CTRL_CMD_ERR and CONFIG_CRC from the part as
    /// it came back up, and says whether the configuration is there.
    struct Provision {
        using Value                            = std::uint8_t;
        static constexpr std::size_t Bytes     = BlockBytes;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr std::array<Step,
                                                  3>
        encode(Value const&,
               std::span<std::byte> buffer) {
            for(std::size_t i = 0; i < BlockBytes; ++i) {
                buffer[i] = static_cast<std::byte>(Configuration[i]);
            }
            return {
              Step::writeBuffer({.reg = 0x00, .offset = 0, .count = BlockBytes}),
              Step::write({.reg = RegCtrlCmd, .payload = {CmdSaveCheckCrc}, .delay = SaveTime}),
              Step::write({.reg     = RegCtrlCmd,
                           .payload = {static_cast<std::uint8_t>(Operation::reset)},
                           .delay   = BootTime}),
            };
        }
    };

    /// On demand: CTRL_CMD_ERR and CONFIG_CRC as the part holds them now -- after a Provision,
    /// whether the save took.
    struct Stored {
        static constexpr std::array Steps{
          Step::read({.reg = RegCtrlCmdErr, .count = 1, .offset = 0}),
          Step::read({.reg = RegConfigCrc, .count = 2, .offset = 1}),
        };

        struct Sample {
            CommandError  lastError{CommandError::none};   ///< CTRL_CMD_ERR
            std::uint16_t storedCrc{};                     ///< CONFIG_CRC
            bool          configured{};                    ///< it is ConfigCrc
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {.lastError  = static_cast<CommandError>(data.u8(0)),
                    .storedCrc  = data.le16(1),
                    .configured = data.le16(1) == ConfigCrc};
        }
    };

    /// A command (TRM 1.5.80). Transient. After `deepSleep` any transaction wakes the part
    /// again, so park the cyclic read first (`period<Moisture>(0ms)`).
    struct Command {
        using Value                            = Operation;
        static constexpr std::size_t Bytes     = 1;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer(
              {.reg    = RegCtrlCmd,
               .offset = 0,
               .count  = 1,
               .delay  = value == Operation::reset ? BootTime : CommandLatency});
        }
    };

    /// !!! CHANGES THE PART'S ADDRESS IN FLASH. !!! Writes the configuration block with
    /// I2C_ADDR = the Value (and its CRC), saves and resets; from then on the part answers
    /// only at the new address, and this Device -- whose address is compiled in -- loses it
    /// (it NAKs and goes absent). Rebuild with `Cy8cmbr3102<NewAddress, ...>` (or a Config
    /// `At<NewAddress>` plus a matching `Addr`, since the block stores `Addr`) before the part
    /// is powered again, or it will be provisioned back. A value outside 8..119 (TRM 1.5.62)
    /// writes `Addr` instead. Transient: a flash write, never replayed.
    struct SetAddress {
        using Value                            = std::uint8_t;
        static constexpr std::size_t Bytes     = BlockBytes;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr std::array<Step,
                                                  3>
        encode(Value const&         value,
               std::span<std::byte> buffer) {
            auto const block
              = configurationFor(value >= 0x08 && value <= 0x77 ? value : Addr.value);
            for(std::size_t i = 0; i < BlockBytes; ++i) {
                buffer[i] = static_cast<std::byte>(block[i]);
            }
            return {
              Step::writeBuffer({.reg = 0x00, .offset = 0, .count = BlockBytes}),
              Step::write({.reg = RegCtrlCmd, .payload = {CmdSaveCheckCrc}, .delay = SaveTime}),
              Step::write({.reg     = RegCtrlCmd,
                           .payload = {static_cast<std::uint8_t>(Operation::reset)},
                           .delay   = BootTime}),
            };
        }
    };

    using Primary = Moisture;
    using Reads   = List<Moisture, Capacitance, Stored>;
    using Writes  = List<Provision, Command, SetAddress>;
};

/// The SparkFun Qwiic Soil Moisture Sensor (SEN-30480): the defaults above are its setup.
using SparkFunSoilMoisture = Cy8cmbr3102<>;

}   // namespace Kvasir::I2C::Chips
