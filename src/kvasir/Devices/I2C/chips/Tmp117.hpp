#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Tmp117Detail {
    /// A temperature as the word the result, limit and offset registers share: 7.8125 m degC,
    /// which is 125/16 of a millidegree, an LSB.
    [[nodiscard]] constexpr std::uint16_t toRaw(MilliDegC t) {
        return static_cast<std::uint16_t>(static_cast<std::int16_t>(Units::value(t) * 16 / 125));
    }

    /// 00 continuous, 01 shutdown, 11 one-shot (10 reads back as 00).
    enum class Mode : std::uint8_t { continuous = 0, shutdown = 1, oneShot = 3 };

    /// Conversions accumulated and averaged before the result register updates.
    enum class Averaging : std::uint8_t { none = 0, eight = 1, thirtyTwo = 2, sixtyFour = 3 };

    /// The conversion cycle, CONV[2:0] (Table 7-7). Named by the cycle at AVG = 00 and from
    /// 250 ms on; the averaging stretches the short ones: code 0 is 15.5 ms, 125 ms, 500 ms or
    /// 1 s and code 1 is 125 ms, 125 ms, 500 ms or 1 s for AVG 00, 01, 10 and 11.
    enum class Cycle : std::uint8_t {
        ms15_5 = 0,
        ms125  = 1,
        ms250  = 2,
        ms500  = 3,
        s1     = 4,   ///< the reset value
        s4     = 5,
        s8     = 6,
        s16    = 7,
    };

    /// Bit 4 T/nA: how the ALERT pin follows the limits.
    enum class AlertMode : std::uint8_t { alert, therm };

    /// Bit 3 POL: the ALERT pin polarity.
    enum class Polarity : std::uint8_t { activeLow, activeHigh };

    /// Bit 2 DR/Alert: what the ALERT pin reports.
    enum class AlertPin : std::uint8_t { limits, dataReady };

    /// The configuration register as fields. Out here because a default argument may not be
    /// used from inside the class that encloses it.
    struct Config {
        Mode      mode{Mode::continuous};
        Cycle     cycle{Cycle::s1};
        Averaging averaging{Averaging::eight};
        AlertMode alertMode{AlertMode::alert};
        Polarity  alertPolarity{Polarity::activeLow};
        AlertPin  alertPin{AlertPin::limits};

        constexpr Config() = default;

        constexpr Config(Mode      m,
                         Cycle     c        = Cycle::s1,
                         Averaging a        = Averaging::eight,
                         AlertMode am       = AlertMode::alert,
                         Polarity  polarity = Polarity::activeLow,
                         AlertPin  pin      = AlertPin::limits)
          : mode{m}
          , cycle{c}
          , averaging{a}
          , alertMode{am}
          , alertPolarity{polarity}
          , alertPin{pin} {}
    };
}   // namespace Tmp117Detail

/// Texas Instruments TMP117 and TMP119 high-accuracy temperature sensors. One-byte
/// pointer, 16-bit big-endian registers: temperature 0x00 (two's complement, 7.8125 m degC
/// per LSB), configuration 0x01 (reset 0x0220: continuous, 1 s cycle, 8 averages), THIGH
/// 0x02 (reset 0x6000), TLOW 0x03 (0x8000), temperature offset 0x07, device id 0x0F. The
/// two parts share a register map. The id register holds the revision in 15:12 and DID 117h in
/// 11:0 on both (TMP117 7.6.11, TMP119 8.5.11: revision 0 and 2 in the current datasheets), so
/// bring-up accepts any revision of DID 117h and records the whole word in State.
///
/// Config bits: 15 HIGH_Alert, 14 LOW_Alert and 13 Data_Ready are read-only flags
/// cleared by reading the register, 12 EEPROM_Busy, 11:10 MOD, 9:7 CONV, 6:5 AVG, 4
/// therm/alert, 3 alert polarity, 2 alert-is-data-ready. Every read takes the configuration
/// and then the temperature -- a read of either clears Data_Ready, so the flag has to come
/// first -- and a frame whose Data_Ready is clear is `unchanged`: the
/// part has not converted since the last read (a 16 s cycle, or shutdown), so the sample
/// stands and seq does not step. The EEPROM is deliberately left alone: it is locked on
/// reset, holds a NIST-traceable unique id, and programming it is a one-shot procedure
/// that does not belong in a cyclic driver; for the same reason the Offset group has no
/// Initial, because TEMP_OFFSET is EEPROM-backed and a factory or application trim would
/// be zeroed after every bring-up. The configuration and the limits are EEPROM-backed too
/// (Table 7-3), yet Config does carry an Initial -- the reset configuration, 0220h -- so a
/// configuration programmed into the EEPROM is written over at every bring-up; a board that
/// relies on one leaves Config unset only by removing the Initial. 0x48..0x4B by ADD0.
struct Tmp117 {
    static constexpr std::string_view Name = "TMP117";
    /// TI TMP117. TMP117.md:1292..1315, "Device ID Register (address = 0Fh) [reset = 0117h]": DID 117h
    /// in 11:0, the revision above it.
    static constexpr std::array Identity{
      RegisterCheck{"id", 0x0F, 2, true, 0x0FFF, 0x0117},
    };
    static constexpr Address7    Address       = 0x48;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 4> Addresses{0x48, 0x49, 0x4A, 0x4B};

    /// 1.5 ms to load the EEPROM into the register map after power-up; writes during that
    /// window are ignored, so the first thing done here is a read. The same load follows a
    /// general-call reset at run time, which StartupDelay does not cover, so the configuration
    /// register is read too and a part whose EEPROM_Busy (bit 12) is still set is not brought
    /// up yet: the Config write after the bring-up would be ignored (7.3.1, "Power Up").
    static constexpr auto StartupDelay = std::chrono::milliseconds{5};

    static constexpr std::array Init{Step::read({.reg = 0x01, .count = 2, .offset = 0})};

    struct State : Groups::DeviceId {
        bool eepromBusy{};   ///< EEPROM_Busy when the bring-up read the configuration
    };

    /// `state().deviceId` is DEVICE_ID as the engine read it for the Identity above.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    /// The identity is the engine's; what is left to decide is whether the part is ready.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.eepromBusy = (data.be16(0) & 0x1000U) != 0;
        return !state.eepromBusy;
    }

    using Mode      = Tmp117Detail::Mode;
    using Averaging = Tmp117Detail::Averaging;
    using Cycle     = Tmp117Detail::Cycle;
    using AlertMode = Tmp117Detail::AlertMode;
    using Polarity  = Tmp117Detail::Polarity;
    using AlertPin  = Tmp117Detail::AlertPin;

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{Step::read({.reg = 0x01, .count = 2, .offset = 0}),
                                          Step::read({.reg = 0x00, .count = 2, .offset = 2})};

        struct Sample {
            MilliDegC temperature{};   ///< 0.001 degC
            bool      highAlert{};     ///< HIGH_Alert, as read with the temperature
            bool      lowAlert{};      ///< LOW_Alert
        };

        /// 7.8125 m degC per LSB is exactly 125/16 of a millidegree. Reading either the
        /// temperature or the configuration clears Data_Ready (bit 13), so the configuration
        /// is read first; the flag clear means no conversion since the last read: the
        /// previous sample stands.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes         data,
                                                              Sample const& previous) {
            auto const config = data.be16(0);
            if((config & 0x2000U) == 0) {
                static_cast<void>(previous);
                return Outcome<Sample>::unchanged();
            }
            Sample sample{};
            sample.temperature
              = Units::milliDegC(static_cast<std::int32_t>(data.s16be(2)) * 125 / 16);
            sample.highAlert = (config & 0x8000U) != 0;
            sample.lowAlert  = (config & 0x4000U) != 0;
            return Outcome<Sample>::ok(sample);
        }
    };

    struct Config {
        using Value                        = Tmp117Detail::Config;
        static constexpr std::size_t Bytes = 2;

        /// The reset configuration (0x0220), put back after every bring-up.
        /// A one-shot mode starts a conversion with every write, so the same value again is a
        /// new command: every set goes out.
        static constexpr bool  AlwaysWrite = true;
        static constexpr Value Initial{Mode::continuous, Cycle::s1, Averaging::eight};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            std::uint16_t raw = 0;
            raw = static_cast<std::uint16_t>(raw | (static_cast<unsigned>(value.mode) << 10));
            raw = static_cast<std::uint16_t>(raw | (static_cast<unsigned>(value.cycle) << 7));
            raw = static_cast<std::uint16_t>(raw | (static_cast<unsigned>(value.averaging) << 5));
            if(value.alertMode == AlertMode::therm) {
                raw = static_cast<std::uint16_t>(raw | (1U << 4));
            }
            if(value.alertPolarity == Polarity::activeHigh) {
                raw = static_cast<std::uint16_t>(raw | (1U << 3));
            }
            if(value.alertPin == AlertPin::dataReady) {
                raw = static_cast<std::uint16_t>(raw | (1U << 2));
            }
            putBe16(buffer, 0, raw);
            return Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 2});
        }
    };

    /// THIGH then TLOW, in the same format as the result register.
    struct Limits {
        using Value                        = MilliDegC;
        static constexpr std::size_t Items = 2;
        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr Step encode(Value const&         limit,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            putBe16(buffer, 0, Tmp117Detail::toRaw(limit));
            return Step::writeBuffer(
              {.reg = static_cast<std::uint16_t>(0x02 + item), .offset = 0, .count = 2});
        }
    };

    /// Added to every conversion result, same format again (0x07). No Initial: the register
    /// is EEPROM-backed, so the part keeps its trim until the application sets one.
    struct Offset {
        using Value                        = MilliDegC;
        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr Step encode(Value const&         offset,
                                                   std::span<std::byte> buffer) {
            putBe16(buffer, 0, Tmp117Detail::toRaw(offset));
            return Step::writeBuffer({.reg = 0x07, .offset = 0, .count = 2});
        }
    };

    /// Limits items, in register order. Note this is the opposite of the TMP1075's: on the
    /// TMP117 register 0x02 is the *high* limit and 0x03 the low one.
    static constexpr std::size_t High = 0;
    static constexpr std::size_t Low  = 1;

    using Reads  = List<Temperature>;
    using Writes = List<Config, Limits, Offset>;
};

/// Register-compatible with the TMP117; the tighter-accuracy part of the same family.
struct Tmp119 : Tmp117 {
    static constexpr std::string_view Name = "TMP119";
    /// TI TMP119. TMP119.md:1551, "Device ID Register (address = 0Fh) [reset = 2117h]". The register
    /// map on :1196 says 0117h for the same register -- the TMP117's value, carried over: the part on
    /// the bench reads 2117h (2026-09-18).
    static constexpr std::array Identity{
      RegisterCheck{"id", 0x0F, 2, true, 0xFFFF, 0x2117},
    };
};

}   // namespace Kvasir::I2C::Chips
