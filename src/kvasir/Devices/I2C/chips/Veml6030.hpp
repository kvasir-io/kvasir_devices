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

namespace Veml6030Detail {
    /// ALS_GAIN bits (ALS_CONF[12:11]) and ALS_IT bits (ALS_CONF[9:6]).
    enum class Gain : std::uint8_t { x1 = 0b00, x2 = 0b01, eighth = 0b10, quarter = 0b11 };

    enum class Integration : std::uint8_t {
        ms25  = 0b1100,
        ms50  = 0b1000,
        ms100 = 0b0000,
        ms200 = 0b0001,
        ms400 = 0b0010,
        ms800 = 0b0011
    };

    [[nodiscard]] constexpr std::chrono::milliseconds integrationTime(Integration integration) {
        switch(integration) {
        case Integration::ms25:  return std::chrono::milliseconds{25};
        case Integration::ms50:  return std::chrono::milliseconds{50};
        case Integration::ms100: return std::chrono::milliseconds{100};
        case Integration::ms200: return std::chrono::milliseconds{200};
        case Integration::ms400: return std::chrono::milliseconds{400};
        case Integration::ms800: return std::chrono::milliseconds{800};
        }
        return std::chrono::milliseconds{100};
    }

    [[nodiscard]] constexpr int gainTimes8(Gain g) {
        switch(g) {
        case Gain::x1:      return 8;
        case Gain::x2:      return 16;
        case Gain::eighth:  return 1;
        case Gain::quarter: return 2;
        }
        return 8;
    }

    /// Light per count: 0.0042 lx at gain x2 / 800 ms (datasheet 84366, "Digital resolution"
    /// and the resolution table), doubling as gain or integration halves. 4200 ulx x 16 x 800
    /// divides by every gain x 8 and
    /// integration time pair, so this is exact.
    [[nodiscard]] constexpr MicroLux resolution(Gain        gain,
                                                Integration integration) {
        return Units::microLux(
          4200 * 16 * 800
          / (gainTimes8(gain) * static_cast<int>(integrationTime(integration).count())));
    }

    /// The two fields as an ALS_CONF word holds them.
    [[nodiscard]] constexpr Gain gainOf(std::uint16_t conf) {
        return static_cast<Gain>((conf >> 11U) & 0x03U);
    }

    [[nodiscard]] constexpr Integration integrationOf(std::uint16_t conf) {
        return static_cast<Integration>((conf >> 6U) & 0x0FU);
    }

    /// The ALS_CONF word for a gain and an integration time, everything else 0: persistence
    /// 1, interrupt off, power on. What Config::Initial is, and what an application that
    /// changes range at run time sets Config to.
    [[nodiscard]] constexpr std::uint16_t conf(Gain        gain,
                                               Integration integration) {
        return static_cast<std::uint16_t>((static_cast<std::uint16_t>(gain) << 11U)
                                          | (static_cast<std::uint16_t>(integration) << 6U));
    }
}   // namespace Veml6030Detail

/// Vishay VEML6030 ambient light sensor (datasheet 84366). One-byte command code, 16-bit
/// little-endian registers: 00h ALS_CONF, 03h power saving, 04h ALS, 05h WHITE, 07h ID (low
/// byte 81h). Bring-up: read the ID, write power saving off (03h = 0: a part left in PSM
/// by earlier firmware refreshes only every 600 to 4800 ms, which the read period does not
/// allow for), write ALS_CONF (gain, integration, persistence 1, interrupt off, power on),
/// wait an integration time; then ALS and WHITE, every 200 ms at 100 ms of integration and
/// an integration time plus 100 ms otherwise.
///
/// Gain and integration are the Initial of the Config write group, so they can be changed at
/// run time; each Sample carries the resolution it was taken at, which lux() multiplies by.
///
/// Above 1000 lx the part's response is not linear, and Vishay's application note ("Designing
/// the VEML6030 into an application") corrects the lux figure for the gain 1/8 and 1/4
/// settings with the polynomial
///
///     lux' = 6.0135e-13 lux^4 - 9.3924e-9 lux^3 + 8.1488e-5 lux^2 + 1.0023 lux
///
/// It is not applied here: it is a float polynomial that an integer decode cannot carry
/// without a 128-bit intermediate, and the application, which owns the float, can apply it
/// to lux() once at the edge when its readings exceed 1000 lx.
/// 0x10 with ADDR low, 0x48 with ADDR high.
template<Veml6030Detail::Gain        G  = Veml6030Detail::Gain::eighth,
         Veml6030Detail::Integration IT = Veml6030Detail::Integration::ms100>
struct Veml6030 {
    using Gain        = Veml6030Detail::Gain;
    using Integration = Veml6030Detail::Integration;

    static constexpr std::string_view Name = "VEML6030";
    /// Vishay VEML6030. VEML6030.md:480..487: command code 07h, low byte first; the low byte is the
    /// device ID code 81h, the high byte follows the address option.
    static constexpr std::array Identity{
      RegisterCheck{"id", 0x07, 2, false, 0x00FF, 0x0081},
    };
    static constexpr Address7                Address = 0x10;
    static constexpr std::array<Address7, 2> Addresses{0x10, 0x48};
    static constexpr std::size_t             RegisterBytes = 1;
    static constexpr auto StartupDelay = std::chrono::milliseconds{3};   // >= 2.5 ms

    static constexpr MicroLux Resolution = Veml6030Detail::resolution(G, IT);

    static constexpr std::uint16_t Conf = Veml6030Detail::conf(G, IT);

    static constexpr std::array Init{
      Step::write({.reg = 0x03, .payload = {0x00, 0x00}}
      ),
      Step::write(
        {.reg     = 0x00,
                   .payload = {static_cast<std::uint8_t>(Conf & 0xFF), static_cast<std::uint8_t>(Conf >> 8)},
                   .delay   = Veml6030Detail::integrationTime(IT) + std::chrono::milliseconds{50}}
      ),
    };

    struct State {
        std::uint16_t deviceId{};
        Gain          gain{G};   ///< ALS_CONF's gain and integration now
        Integration   integration{IT};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId    = static_cast<std::uint16_t>(ids[0]);
        state.gain        = G;
        state.integration = IT;
    }

    /// The integration time plus 100 ms, never under 200 ms.
    [[nodiscard]] static constexpr std::chrono::milliseconds readPeriod(Integration integration) {
        auto const p
          = Veml6030Detail::integrationTime(integration) + std::chrono::milliseconds{100};
        return p < std::chrono::milliseconds{200} ? std::chrono::milliseconds{200} : p;
    }

    struct Light {
        static constexpr auto       Period = readPeriod(IT);
        static constexpr std::array Steps{Step::read({.reg = 0x04, .count = 2, .offset = 0}),
                                          Step::read({.reg = 0x05, .count = 2, .offset = 2})};

        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            return readPeriod(state.integration);
        }

        struct Sample {
            std::uint16_t als{};
            std::uint16_t white{};
            MicroLux      resolution{Resolution};   ///< light per count the frame was taken at

            [[nodiscard]] constexpr MilliLux lux() const { return toLight(als); }

            [[nodiscard]] constexpr MilliLux whiteLux() const { return toLight(white); }

            [[nodiscard]] constexpr MilliLux toLight(std::uint16_t count) const {
                return Units::milliLux(static_cast<std::uint64_t>(count) * Units::value(resolution)
                                       / 1000U);
            }

            [[nodiscard]] constexpr bool saturated() const { return als >= 0xFFF0; }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes        data,
                                                     State const& state) {
            return {data.le16(0),
                    data.le16(2),
                    Veml6030Detail::resolution(state.gain, state.integration)};
        }
    };

    /// ALS_CONF, so gain and integration can be changed at run time: the word as the register
    /// holds it (gain 12:11, integration 9:6, persistence 5:4, interrupt 1, shutdown 0). The
    /// write is followed by two integration times at the new setting, the part's other groups
    /// off the wire meanwhile: a result read sooner was integrated under the old setting, and
    /// applied() has already switched the resolution it is scaled by.
    struct Config {
        using Value                          = std::uint16_t;
        static constexpr std::size_t Bytes   = 2;
        static constexpr Value       Initial = Conf;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            putLe16(buffer, 0, value);
            return Step::writeBuffer(
              {.reg    = 0x00,
               .offset = 0,
               .count  = 2,
               .delay  = 2 * Veml6030Detail::integrationTime(Veml6030Detail::integrationOf(value))
                       + std::chrono::milliseconds{10}});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gain        = Veml6030Detail::gainOf(value);
            state.integration = Veml6030Detail::integrationOf(value);
        }
    };

    using Reads  = List<Light>;
    using Writes = List<Config>;
};

/// Vishay VEML7700 (datasheet 84286): the VEML6030's register set at the fixed address
/// 0x10, ID low byte 0x81 as well (high byte 0xC4 for this address option).
template<Veml6030Detail::Gain        G  = Veml6030Detail::Gain::eighth,
         Veml6030Detail::Integration IT = Veml6030Detail::Integration::ms100>
struct Veml7700 : Veml6030<G, IT> {
    static constexpr std::string_view Name = "VEML7700";
    // Identity: the VEML6030's, inherited. The VEML7700's data sheet documents no ID register; the
    // description has always asked it for the VEML6030's 81h, and still does.
    static constexpr Address7                Address = 0x10;
    static constexpr std::array<Address7, 1> Addresses{0x10};
};

}   // namespace Kvasir::I2C::Chips
