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

namespace Adt7420Detail {
    enum class Mode : std::uint8_t { continuous = 0, oneShot = 1, oneSps = 2, shutdown = 3 };

    enum class Faults : std::uint8_t { one = 0, two = 1, three = 2, four = 3 };

    /// Bit 7: 13-bit (1/16 degC) or 16-bit (1/128 degC) conversions.
    enum class Resolution : std::uint8_t { bits13, bits16 };

    /// Bit 4: how INT and CT behave.
    enum class AlertMode : std::uint8_t { interrupt, comparator };

    /// Bits 3 and 2: the INT and CT pin polarity.
    enum class Polarity : std::uint8_t { activeLow, activeHigh };

    /// The configuration register (0x03) as fields. Out here because a default member
    /// initializer may not be used from inside the class that encloses it.
    struct Config {
        Resolution resolution{Resolution::bits13};
        Mode       mode{Mode::continuous};
        AlertMode  alertMode{AlertMode::interrupt};
        Polarity   intPolarity{Polarity::activeLow};
        Polarity   ctPolarity{Polarity::activeLow};
        Faults     faults{Faults::one};
    };

    /// What a bring-up read, and what the temperature format follows: the resolution bit
    /// the configuration register holds now, which a Config write changes (applied()).
    struct State {
        std::uint8_t deviceId{};     ///< ID register (0x0B): 0xCB
        Resolution   resolution{};   ///< the resolution the part converts at now
    };
}   // namespace Adt7420Detail

/// Analog Devices ADT7420 temperature sensor. One-byte pointer: temperature 0x00 (16 bit,
/// big endian), configuration 0x03 (reset 0x00), the setpoints THIGH 0x04 (reset
/// 0x2000, 64 degC), TLOW 0x06 (0x0500, 10 degC), TCRIT 0x08 (0x4980, 147 degC), the
/// 4-bit hysteresis THYST 0x0A (0x05, 5 degC) and the ID register 0x0B (0xCB: manufacturer
/// ID 11001 in bits 7:3, revision 011 in bits 2:0, Table 19). Configuration bits: 7
/// resolution, 6:5 operation mode, 4 comparator/interrupt, 3 INT polarity, 2 CT polarity,
/// 1:0 fault queue.
///
/// Bring-up reads the ID register and rejects anything but 0xCB. The temperature format
/// follows bit 7 of the configuration, so decode scales from the State: `Res` is the
/// resolution the Config group's Initial puts in the part, and a Config written at run time
/// with the other resolution changes what decode divides by from the write's completion on.
/// `Adt7420<>` is the 13-bit reset default, 1/16 degC; `Adt7420<Resolution::bits16>` is 16-bit,
/// 1/128 degC.
/// Setpoints are always the 13-bit format, left-aligned by three. 0x48..0x4B by A1 A0.
template<Adt7420Detail::Resolution Res = Adt7420Detail::Resolution::bits13>
struct Adt7420 {
    static constexpr std::string_view Name = "ADT7420";
    /// Analog Devices ADT7420. ADT7420.md:592, :739: ID (0x0B) is 0xCB, the manufacturer in 7:3 and
    /// the silicon revision below it.
    static constexpr std::array Identity{
      RegisterCheck{"id", 0x0B, 1, true, 0xF8, 0xC8},
    };
    static constexpr Address7    Address       = 0x48;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 4> Addresses{0x48, 0x49, 0x4A, 0x4B};

    static constexpr auto StartupDelay = std::chrono::milliseconds{250};

    using Mode       = Adt7420Detail::Mode;
    using Faults     = Adt7420Detail::Faults;
    using Resolution = Adt7420Detail::Resolution;
    using AlertMode  = Adt7420Detail::AlertMode;
    using Polarity   = Adt7420Detail::Polarity;
    using State      = Adt7420Detail::State;

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId   = static_cast<std::uint8_t>(ids[0]);
        state.resolution = Res;   // what Config::Initial writes right after
        // Table 19: the manufacturer ID 11001 in bits 7:3; bits 2:0 are the silicon revision
    }

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{500};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2, .offset = 0})};

        struct Sample {
            CentiDegC temperature{};   ///< 0.01 degC
        };

        /// At the resolution the part holds now. 0xFFFF is what a floating bus reads. In the
        /// 13-bit format it is no temperature (-1/16 degC with all three flag bits set, which
        /// the part cannot report together) and is rejected; in the 16-bit format it is
        /// -1/128 degC, a real reading, and is kept.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if(state.resolution == Resolution::bits13 && data.be16(0) == 0xFFFF) {
                return Outcome<Sample>::reject();
            }
            auto const raw = data.s16be(0);
            if(state.resolution == Resolution::bits16) {
                return Outcome<Sample>::ok(
                  {Units::centiDegC(static_cast<std::int32_t>(raw) * 25 / 32)});   // 1/128 degC
            }
            return Outcome<Sample>::ok(
              {Units::centiDegC(static_cast<std::int32_t>(raw >> 3) * 25 / 4)});   // 1/16 degC
        }
    };

    struct Config {
        using Value = Adt7420Detail::Config;

        static constexpr std::size_t Bytes = 1;

        /// A one-shot mode starts a conversion with every write, so the same value again is a
        /// new command: every set goes out.
        static constexpr bool  AlwaysWrite = true;
        static constexpr Value Initial{.resolution = Res};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            std::uint8_t raw = 0;
            if(value.resolution == Resolution::bits16) {
                raw = static_cast<std::uint8_t>(raw | 0x80U);
            }
            raw = static_cast<std::uint8_t>(raw | (static_cast<unsigned>(value.mode) << 5));
            if(value.alertMode == AlertMode::comparator) {
                raw = static_cast<std::uint8_t>(raw | 0x10U);
            }
            if(value.intPolarity == Polarity::activeHigh) {
                raw = static_cast<std::uint8_t>(raw | 0x08U);
            }
            if(value.ctPolarity == Polarity::activeHigh) {
                raw = static_cast<std::uint8_t>(raw | 0x04U);
            }
            raw       = static_cast<std::uint8_t>(raw | static_cast<unsigned>(value.faults));
            buffer[0] = static_cast<std::byte>(raw);
            return Step::writeBuffer({.reg = 0x03, .offset = 0, .count = 1});
        }

        /// The resolution bit is what Temperature::decode scales by from now on.
        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.resolution = value.resolution;
        }
    };

    /// THIGH, TLOW then TCRIT, always the 13-bit format (1/16 degC).
    struct Limits {
        using Value                        = CentiDegC;
        static constexpr std::size_t Items = 3;
        static constexpr std::size_t Bytes = 2;

        static constexpr std::array<std::uint8_t, 3> Registers{0x04, 0x06, 0x08};

        [[nodiscard]] static constexpr Step encode(Value const&         limit,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            // Clamped to the part's -55 .. +150 degC, as Linux adt7x10.c does: past +255.94 degC
            // the 13-bit register value would wrap to a negative setpoint.
            auto const raw        = Units::value(limit);
            auto const centi      = raw < -5500 ? -5500 : raw > 15000 ? 15000 : raw;
            auto const sixteenths = static_cast<std::int16_t>(centi * 4 / 25);
            putBe16(buffer,
                    0,
                    static_cast<std::uint16_t>(static_cast<std::uint16_t>(sixteenths) << 3));
            return Step::writeBuffer({.reg = Registers[item], .offset = 0, .count = 2});
        }
    };

    /// Whole degrees, 0..15.
    struct Hysteresis {
        using Value                          = DegC;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Units::degC(5);

        [[nodiscard]] static constexpr Step encode(Value const&         hysteresis,
                                                   std::span<std::byte> buffer) {
            auto const degrees = Units::value(hysteresis);
            auto const clamped = degrees > 15 ? 15 : degrees < 0 ? 0 : degrees;
            buffer[0]          = static_cast<std::byte>(clamped);
            return Step::writeBuffer({.reg = 0x0A, .offset = 0, .count = 1});
        }
    };

    static constexpr std::size_t High     = 0;
    static constexpr std::size_t Low      = 1;
    static constexpr std::size_t Critical = 2;

    using Reads  = List<Temperature>;
    using Writes = List<Config, Limits, Hysteresis>;
};

}   // namespace Kvasir::I2C::Chips
