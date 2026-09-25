#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Tcs34725Detail {
    /// ATIME: the integration time counts down from 256 in 2.4 ms cycles; the datasheet's
    /// named values.
    enum class Integration : std::uint8_t {
        ms2_4 = 0xFF,   ///< 1 cycle
        ms24  = 0xF6,   ///< 10 cycles
        ms101 = 0xD5,   ///< 43 cycles, 103 ms
        ms154 = 0xC0,   ///< 64 cycles
        ms700 = 0x00,   ///< 256 cycles, 614 ms (the datasheet names it 700 ms)
    };

    /// CONTROL AGAIN (1:0): 1x, 4x, 16x, 60x.
    enum class Gain : std::uint8_t { x1 = 0, x4 = 1, x16 = 2, x60 = 3 };

    /// The integration time of an ATIME value, rounded up to whole milliseconds.
    [[nodiscard]] constexpr std::chrono::milliseconds integrationTime(std::uint8_t atime) {
        return std::chrono::milliseconds{((256U - atime) * 24U + 9U) / 10U};
    }
}   // namespace Tcs34725Detail

/// ams / TAOS TCS34725 colour sensor (TAOS135). Fixed address 0x29. Every register access
/// goes through the command register: bit 7 set, type 00 (repeated byte) or 01
/// (auto-increment, 0xA0) in bits 6:5, the register in bits 4:0. Bring-up: ID 0x12 = 0x44
/// (TCS34725) or 0x4D (TCS34727); ENABLE 0x00 = PON (2.4 ms), ATIME 0x01 = 0xD5 (43
/// cycles, 103 ms), CONTROL 0x0F = 0x01 (4x gain), ENABLE = PON | AEN. Data: nine bytes
/// from STATUS 0x13 with auto-increment: AVALID (bit 0), then clear, red, green, blue,
/// little-endian. A frame without AVALID -- the part has not completed an integration since
/// it was enabled -- is rejected.
///
/// Integration time and gain are the Initial of the two write groups, so they can be changed
/// at run time; the Colour period follows the integration time written. A result integrated
/// under the old setting would be decoded with the new one, which applied() has already put
/// in State, and AVALID does not tell the two apart -- it stays set once an integration has
/// completed. So an ATIME write is followed by two integration times at the new ATIME, the
/// part's other groups off the wire, and a gain write -- which does not know the integration
/// time -- restarts the RGBC cycle: AEN off and on again, so AVALID is clear until an
/// integration at the new gain completes (the same state machine as the TSL2591's, whose
/// AVALID is "since the AEN bit was asserted").
template<Tcs34725Detail::Integration Integration = Tcs34725Detail::Integration::ms101,
         Tcs34725Detail::Gain        Gain        = Tcs34725Detail::Gain::x4>
struct Tcs34725X {
    static constexpr std::string_view Name = "TCS34725";
    /// ams TCS3472x. TCS34725.md:848..858: ID (0x12) is 0x44 for the TCS34721/34725 and 0x4D for the
    /// TCS34723/34727. Through the command register: CMD, bit 7 (:632).
    static constexpr std::array Identity{
      RegisterCheck{"id", 0x80 | 0x12, 1, true, 0xFF, 0x44, 0x4D},
    };
    static constexpr Address7                Address = 0x29;
    static constexpr std::array<Address7, 1> Addresses{0x29};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::uint8_t Cmd     = 0x80;
    static constexpr std::uint8_t CmdAuto = 0xA0;

    static constexpr std::uint8_t              Atime   = static_cast<std::uint8_t>(Integration);
    static constexpr std::uint8_t              Control = static_cast<std::uint8_t>(Gain);
    static constexpr std::chrono::milliseconds IntegrationTime
      = Tcs34725Detail::integrationTime(Atime);

    static constexpr std::array Init{
      Step::write(
        {.reg = Cmd | 0x00, .payload = {0x01}, .delay = std::chrono::milliseconds{3}}),   // PON
      Step::write({.reg = Cmd | 0x01, .payload = {Atime}}),
      Step::write({.reg = Cmd | 0x0F, .payload = {Control}}),
      Step::write({.reg     = Cmd | 0x00,
                   .payload = {0x03},
                   .delay   = IntegrationTime + std::chrono::milliseconds{10}}),   // PON | AEN
    };

    struct State {
        std::uint8_t deviceId{};
        std::uint8_t atime{Atime};
        std::uint8_t gain{Control};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint8_t>(ids[0]);
        state.atime    = Atime;
        state.gain     = Control;
    }

    /// Twice the integration time, so every poll finds a new result; never under 100 ms.
    [[nodiscard]] static constexpr std::chrono::milliseconds readPeriod(std::uint8_t atime) {
        auto const p = 2 * Tcs34725Detail::integrationTime(atime);
        return p < std::chrono::milliseconds{100} ? std::chrono::milliseconds{100} : p;
    }

    struct Colour {
        static constexpr auto       Period = readPeriod(Atime);
        static constexpr std::array Steps{
          Step::read({.reg = CmdAuto | 0x13, .count = 9})};   // STATUS, then the channels

        [[nodiscard]] static constexpr std::chrono::milliseconds period(State const& state) {
            return readPeriod(state.atime);
        }

        struct Sample {
            std::uint16_t clear{}, red{}, green{}, blue{};
            std::uint8_t  atime{};   ///< ATIME the frame was integrated with
            std::uint8_t  gain{};    ///< AGAIN, 0..3
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if((data.u8(0) & 0x01U) == 0) { return Outcome<Sample>::reject(); }   // AVALID
            return Outcome<Sample>::ok(
              {data.le16(1), data.le16(3), data.le16(5), data.le16(7), state.atime, state.gain});
        }
    };

    /// ATIME, changeable at run time.
    struct IntegrationSetting {
        using Value                          = Tcs34725Detail::Integration;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Integration;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer(
              {.reg    = Cmd | 0x01,
               .offset = 0,
               .count  = 1,
               .delay  = 2 * Tcs34725Detail::integrationTime(static_cast<std::uint8_t>(value))
                       + std::chrono::milliseconds{10}});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.atime = static_cast<std::uint8_t>(value);
        }
    };

    /// CONTROL: AGAIN in 1:0.
    struct GainSetting {
        using Value                          = Tcs34725Detail::Gain;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Gain;

        [[nodiscard]] static constexpr std::array<Step,
                                                  3>
        encode(Value const&         value,
               std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return {Step::writeBuffer({.reg = Cmd | 0x0F, .offset = 0, .count = 1}),
                    Step::write({.reg = Cmd | 0x00, .payload = {0x01}}),    // PON: RGBC stopped
                    Step::write({.reg = Cmd | 0x00, .payload = {0x03}})};   // PON | AEN again
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gain = static_cast<std::uint8_t>(value);
        }
    };

    using Reads  = List<Colour>;
    using Writes = List<IntegrationSetting, GainSetting>;
};

/// The part at 101 ms and 4x gain.
using Tcs34725 = Tcs34725X<>;

}   // namespace Kvasir::I2C::Chips
