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

namespace Tsl2591Detail {
    /// CONFIG AGAIN (5:4): 1x, 25x, 428x, 9876x (the datasheet's typical values).
    enum class Gain : std::uint8_t { low = 0, medium = 1, high = 2, maximum = 3 };

    /// CONFIG ATIME (2:0): 100 ms a step.
    enum class Integration : std::uint8_t {
        ms100 = 0,
        ms200 = 1,
        ms300 = 2,
        ms400 = 3,
        ms500 = 4,
        ms600 = 5
    };

    inline constexpr std::array<std::uint16_t, 4> GainValue{1, 25, 428, 9876};

    [[nodiscard]] constexpr std::chrono::milliseconds integrationTime(std::uint8_t integration) {
        return std::chrono::milliseconds{100U * ((integration & 0x07U) + 1U)};
    }

    /// The count a channel saturates at: 37888 at 100 ms, the full 16 bits otherwise
    /// (datasheet, ALS characteristics "ADC counts per step" and the CONFIG ATIME table).
    [[nodiscard]] constexpr std::uint16_t saturation(std::uint8_t integration) {
        return (integration & 0x07U) == 0 ? 37888 : 65535;
    }
}   // namespace Tsl2591Detail

/// ams TSL2591 high-dynamic-range light sensor. Every register access is prefixed by a
/// COMMAND byte -- bit 7 set, bits 6:5 = 01 for normal operation -- so the register address
/// on the wire is 0xA0 | reg: ENABLE 0x00 (bit 0 PON, bit 1 AEN), CONFIG 0x01 (AGAIN in
/// 5:4, ATIME in 2:0, SRESET in 7), package id 0x11, device id 0x12 (0x50), STATUS 0x13
/// (AVALID bit 0), and the two 16-bit little-endian channels at 0x14 (CH0, full spectrum)
/// and 0x16 (CH1, infrared). Fixed address 0x29.
///
/// Gain and integration time are template parameters, the Initial of the Config write group:
/// the application can change them at run time, and every Sample carries the pair it was
/// taken under, which its lux() divides by. `Tsl2591<>` is medium gain (25x) over 100 ms.
/// The datasheet gives no lux formula. This is the counts-per-lux form Adafruit's library uses
/// and Linux tsl2591.c took from it: CPL = ATIME_ms x AGAIN / 408, lux = (CH0 - CH1) x
/// (1 - CH1/CH0) / CPL, which is (CH0 - CH1)^2 x 408 / (CH0 x ATIME_ms x AGAIN) and so exact in
/// 64-bit integers -- where Linux computes CPL in integers first, which is 0 at 1x gain and
/// 100 ms and divides by it.
///
/// STATUS is read in the same burst as the channels and a frame without AVALID is rejected.
template<Tsl2591Detail::Gain        Gain            = Tsl2591Detail::Gain::medium,
         Tsl2591Detail::Integration IntegrationTime = Tsl2591Detail::Integration::ms100>
struct Tsl2591 {
    static constexpr std::string_view Name = "TSL2591";
    /// ams TSL2591. TSL2591.md:488..494, ID register 0x12 = 0x50. The part is addressed through its
    /// command register: 0xA0 (CMD, normal operation) | 0x12.
    static constexpr std::array Identity{
      RegisterCheck{"id", 0xA0 | 0x12, 1, true, 0xFF, 0x50},
    };
    static constexpr Address7                Address = 0x29;
    static constexpr std::array<Address7, 1> Addresses{0x29};
    static constexpr std::size_t             RegisterBytes = 1;

    /// A register address on the wire: COMMAND bit set, normal transaction.
    [[nodiscard]] static constexpr std::uint16_t cmd(std::uint8_t reg) {
        return static_cast<std::uint16_t>(0xA0U | reg);
    }

    static constexpr std::uint8_t GainCode        = static_cast<std::uint8_t>(Gain);
    static constexpr std::uint8_t IntegrationCode = static_cast<std::uint8_t>(IntegrationTime);
    static constexpr std::uint8_t Config
      = static_cast<std::uint8_t>((GainCode << 4) | IntegrationCode);

    /// 1x, 25x, 428x, 9876x (the datasheet's typical values).
    static constexpr std::array<std::uint16_t, 4> GainValue = Tsl2591Detail::GainValue;
    static constexpr std::uint16_t                AGain     = GainValue[GainCode];
    static constexpr auto Atime = Tsl2591Detail::integrationTime(IntegrationCode);

    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    static constexpr std::array Init{
      Step::write({.reg = cmd(0x01), .payload = {Config}}),
      Step::write(
        {.reg     = cmd(0x00),
         .payload = {0x03},
         .delay
         = Atime + std::chrono::milliseconds{20}}),   // PON | AEN, then the first integration
    };

    struct State {
        std::uint8_t deviceId{};
        std::uint8_t gain{GainCode};                 ///< AGAIN as CONFIG holds it now
        std::uint8_t integration{IntegrationCode};   ///< ATIME
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId    = static_cast<std::uint8_t>(ids[0]);
        state.gain        = GainCode;   // what the Init script wrote
        state.integration = IntegrationCode;
    }

    struct Light {
        static constexpr auto Period = std::chrono::milliseconds{500};

        /// A new result every integration time; never faster than the part converts.
        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            auto const p = Tsl2591Detail::integrationTime(state.integration);
            return p > std::chrono::milliseconds{500} ? p : std::chrono::milliseconds{500};
        }

        /// STATUS, then CH0 and CH1 (each 16-bit little endian) in one burst.
        static constexpr std::array Steps{Step::read({.reg = cmd(0x13), .count = 5, .offset = 0})};

        struct Sample {
            std::uint16_t full{};          ///< CH0: visible + infrared
            std::uint16_t infrared{};      ///< CH1
            std::uint8_t  gain{};          ///< AGAIN the frame was taken at
            std::uint8_t  integration{};   ///< ATIME

            [[nodiscard]] constexpr std::uint16_t visible() const {
                return full > infrared ? static_cast<std::uint16_t>(full - infrared) : 0;
            }

            /// Saturated at the integration time's limit; the reading below is meaningless there.
            [[nodiscard]] constexpr bool saturated() const {
                auto const limit = Tsl2591Detail::saturation(integration);
                return full >= limit || infrared >= limit;
            }

            [[nodiscard]] constexpr MilliLux lux() const {
                if(full == 0 || saturated()) { return Units::milliLux(0); }
                auto const v = static_cast<std::uint64_t>(visible());
                auto const den
                  = static_cast<std::uint64_t>(full)
                  * static_cast<std::uint64_t>(Tsl2591Detail::integrationTime(integration).count())
                  * Tsl2591Detail::GainValue[gain & 0x03U];
                return Units::milliLux(v * v * 408ULL * 1000ULL / den);
            }
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if((data.u8(0) & 0x01U) == 0) { return Outcome<Sample>::reject(); }   // AVALID
            return Outcome<Sample>::ok({data.le16(1), data.le16(3), state.gain, state.integration});
        }
    };

    /// CONFIG: AGAIN in 5:4 and ATIME in 2:0, changeable at run time; a Sample decoded after
    /// the write carries the new pair. AVALID says an integration has completed since AEN was
    /// set, not since this write, so it does not tell an old-setting frame from a new one: the
    /// write is followed by two integration times at the new setting instead, the part's other
    /// groups off the wire meanwhile (Linux tsl2591.c sleeps an integration time and polls
    /// AVALID).
    struct Configuration {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Config;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer(
              {.reg    = cmd(0x01),
               .offset = 0,
               .count  = 1,
               .delay = 2 * Tsl2591Detail::integrationTime(static_cast<std::uint8_t>(value & 0x07U))
                      + std::chrono::milliseconds{10}});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gain        = static_cast<std::uint8_t>((value >> 4U) & 0x03U);
            state.integration = static_cast<std::uint8_t>(value & 0x07U);
        }
    };

    /// ENABLE: PON alone powers the oscillator, PON | AEN also runs the ALS.
    struct Enable {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = 0x03;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = cmd(0x00), .offset = 0, .count = 1});
        }
    };

    using Reads  = List<Light>;
    using Writes = List<Configuration, Enable>;
};

}   // namespace Kvasir::I2C::Chips
