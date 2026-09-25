#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Texas Instruments TMP102 (SBOS397I). One-byte pointer, 16-bit big-endian registers:
/// 00 temperature (12 bit, left aligned: value >> 4 at 0.0625 degC), 01 configuration
/// (reset 60A0h: 12 bit, 4 Hz, continuous), which is written at bring-up so a warm part
/// is in a known state. 0x48..0x4B by the ADD0 pin.
///
/// The part has no id register, and its addresses are shared with the TMP117, TMP1075,
/// ADS1x15 and LM75, so the configuration is read before anything is written to it: R1:R0
/// (bits 14:13) are read-only 11 and bits 3:0 read 0 (Tables 6-10 and 6-11), which is the
/// check Linux tmp102.c makes too. The temperature register reads 0 degC until the first
/// conversion after the write is done (10 ms typical, 15 ms at most, 6.3.1 and the
/// characteristics table), so the first read waits 35 ms, as Linux does.
struct Tmp102 {
    static constexpr std::string_view        Name    = "TMP102";
    static constexpr Address7                Address = 0x48;
    static constexpr std::array<Address7, 4> Addresses{0x48, 0x49, 0x4A, 0x4B};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::array Init{
      Step::read({.reg = 0x01,              .count = 2,                            .offset = 0}
      ),
      Step::identify(),
      Step::write({.reg = 0x01, .payload = {0x60, 0xA0}, .delay = std::chrono::milliseconds{35}}
      ),
    };

    struct State {
        std::uint16_t configuration{};   ///< as the part held it before the write
    };

    /// Checked before the write (Step::identify), so a part that fails it is not written.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.configuration = data.be16(0);
        return (state.configuration & 0x600FU) == 0x6000U;
    }

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{250};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2})};

        struct Sample {
            CentiDegC temperature{};   ///< 0.01 degC, from 0.0625 degC steps
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {Units::centiDegC(static_cast<std::int32_t>(data.s16be(0) >> 4) * 25 / 4)};
        }
    };

    using Reads = List<Temperature>;
};

/// LM75B (TI SNOSC66 / NXP): 0x48..0x4F, pointer 00 temperature (11 bit on the LM75B, 9 on
/// the LM75A -- both left aligned, 0.125 degC per LSB after >> 5, the A's low bits zero),
/// 01 configuration (00h: continuous). Same shape as the TMP102 with a coarser number. The
/// temperature register holds no conversion until the first one after power-up is done
/// (the LM75B converts every 100 ms), so the first read waits for one.
struct Lm75 {
    static constexpr std::string_view Name          = "LM75";
    static constexpr Address7         Address       = 0x48;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array<Address7, 8>
      Addresses{0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F};

    static constexpr std::array Init{
      Step::write({.reg = 0x01, .payload = {0x00}, .delay = std::chrono::milliseconds{110}})};

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{250};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2})};

        struct Sample {
            CentiDegC temperature{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {Units::centiDegC(static_cast<std::int32_t>(data.s16be(0) >> 5) * 25 / 2)};
        }
    };

    using Reads = List<Temperature>;
};

}   // namespace Kvasir::I2C::Chips
