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

namespace Tmp1075Detail {
    enum class Rate : std::uint8_t { ms27 = 0, ms55 = 1, ms110 = 2, ms220 = 3 };

    enum class Faults : std::uint8_t { one = 0, two = 1, three = 2, four = 3 };

    /// Bit 15 OS: a write with `start` begins a single conversion.
    enum class OneShot : std::uint8_t { idle, start };

    /// Bit 10 POL: the ALERT pin polarity.
    enum class Polarity : std::uint8_t { activeLow, activeHigh };

    /// Bit 9 TM: how ALERT behaves.
    enum class AlertMode : std::uint8_t { comparator, interrupt };

    /// Bit 8 SD: converting continuously or shut down.
    enum class Power : std::uint8_t { active, shutdown };

    /// The configuration register (0x01) as fields. Out here because a default member
    /// initializer may not be used from inside the class that encloses it.
    struct Config {
        OneShot   oneShot{OneShot::idle};
        Rate      rate{Rate::ms27};
        Faults    faults{Faults::one};
        Polarity  alertPolarity{Polarity::activeLow};
        AlertMode alertMode{AlertMode::comparator};
        Power     power{Power::active};
    };
}   // namespace Tmp1075Detail

/// Texas Instruments TMP1075 temperature sensor. One-byte pointer, 16-bit big-endian
/// registers: temperature 0x00 (12 bits left-aligned, 0.0625 degC/LSB), configuration
/// 0x01 (reset 0x00FF), the alert limits TLOW 0x02 (reset 0x4B00, 75 degC) and THIGH 0x03
/// (reset 0x5000, 80 degC), device id DIEID 0x0F (7500h). Configuration bits: 15 OS
/// (one-shot), 14:13 conversion rate, 12:11 fault queue, 10 alert polarity, 9 thermostat
/// mode, 8 shutdown. Bring-up reads DIEID and rejects anything else, which leaves the
/// TMP1075N out: it has no device id register (Table 7-5 note). A2 is tied high or low, A1
/// and A0 may also go to SDA or SCL, which gives the 32 addresses 0x40..0x5F of Table 7-2.
struct Tmp1075 {
    static constexpr std::string_view Name = "TMP1075";
    /// TI TMP1075. TMP1075.md:931: DIEID (0Fh) is 7500h (not on the TMP1075N, :935).
    static constexpr std::array Identity{
      RegisterCheck{"die-id", 0x0F, 2, true, 0xFFFF, 0x7500},
    };
    static constexpr Address7    Address       = 0x48;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 32> Addresses{
      0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A,
      0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55,
      0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F};

    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

    using State = Groups::DeviceId;

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    using Rate      = Tmp1075Detail::Rate;
    using Faults    = Tmp1075Detail::Faults;
    using OneShot   = Tmp1075Detail::OneShot;
    using Polarity  = Tmp1075Detail::Polarity;
    using AlertMode = Tmp1075Detail::AlertMode;
    using Power     = Tmp1075Detail::Power;

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{500};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2, .offset = 0})};

        struct Sample {
            CentiDegC temperature{};   ///< 0.01 degC
        };

        /// 12 bits left-aligned in a 16-bit word, 1/16 degC each; the low four bits always
        /// read zero, so 0xFFFF is a floating bus and not -0.0625 degC.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if(data.be16(0) == 0xFFFF) { return Outcome<Sample>::reject(); }
            return Outcome<Sample>::ok(
              {Units::centiDegC(static_cast<std::int32_t>(data.s16be(0) >> 4) * 25 / 4)});
        }
    };

    /// The whole configuration register as named fields, so one of them can be changed
    /// without disturbing the rest: dev.modify<Config>([](auto& c) { c.power = Power::shutdown; }).
    struct Config {
        using Value = Tmp1075Detail::Config;

        static constexpr std::size_t Bytes = 2;

        /// All fields at their reset values, put back after every bring-up. The register resets
        /// to 0x00FF; its low byte is unused and written as 0.
        /// A one-shot mode starts a conversion with every write, so the same value again is a
        /// new command: every set goes out.
        static constexpr bool  AlwaysWrite = true;
        static constexpr Value Initial{};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            std::uint16_t raw = 0;
            if(value.oneShot == OneShot::start) {
                raw = static_cast<std::uint16_t>(raw | (1U << 15));
            }
            raw = static_cast<std::uint16_t>(raw | (static_cast<unsigned>(value.rate) << 13));
            raw = static_cast<std::uint16_t>(raw | (static_cast<unsigned>(value.faults) << 11));
            if(value.alertPolarity == Polarity::activeHigh) {
                raw = static_cast<std::uint16_t>(raw | (1U << 10));
            }
            if(value.alertMode == AlertMode::interrupt) {
                raw = static_cast<std::uint16_t>(raw | (1U << 9));
            }
            if(value.power == Power::shutdown) {
                raw = static_cast<std::uint16_t>(raw | (1U << 8));
            }
            putBe16(buffer, 0, raw);
            return Step::writeBuffer({.reg = 0x01, .offset = 0, .count = 2});
        }
    };

    /// TLOW then THIGH: set<Limits>(Tmp1075::Low, Units::centiDegC(7500)).
    struct Limits {
        using Value                        = CentiDegC;
        static constexpr std::size_t Items = 2;
        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr Step encode(Value const&         limit,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            // Clamped to the part's -55 .. +125 degC: the 12-bit register holds -128 .. +127.94
            // degC, and past that a setpoint would wrap to the other sign.
            auto const raw        = Units::value(limit);
            auto const centi      = raw < -5500 ? -5500 : raw > 12500 ? 12500 : raw;
            auto const sixteenths = static_cast<std::int16_t>(centi * 4 / 25);
            putBe16(buffer,
                    0,
                    static_cast<std::uint16_t>(static_cast<std::uint16_t>(sixteenths) << 4));
            return Step::writeBuffer(
              {.reg = static_cast<std::uint16_t>(0x02 + item), .offset = 0, .count = 2});
        }
    };

    /// Limits items, in register order. Note this is the opposite of the TMP117's: on the
    /// TMP1075 register 0x02 is the *low* limit and 0x03 the high one.
    static constexpr std::size_t Low  = 0;
    static constexpr std::size_t High = 1;

    using Reads  = List<Temperature>;
    using Writes = List<Config, Limits>;
};

}   // namespace Kvasir::I2C::Chips
