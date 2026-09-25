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

/// NXP MMA8451Q 14-bit accelerometer (MMA8451Q data sheet rev 8.1). One-byte registers,
/// burst reads auto-increment. Bring-up: CTRL_REG2 (0x2B) RST, then 10 ms; WHO_AM_I (0x0D)
/// = 0x1A; CTRL_REG1 (0x2A) = 0 (standby: the configuration registers are writable only
/// there); XYZ_DATA_CFG (0x0E) FS = 0 (2 g); CTRL_REG2 MODS = 2 (high resolution);
/// CTRL_REG1 = 0x19 (100 Hz, active). Data (6.1): STATUS 0x00 (ZYXDR bit 3) then six bytes
/// from OUT_X_MSB 0x01 in one burst, each axis a 14-bit left-justified two's complement in
/// 16 bits, big-endian: 4096 counts/g at 2 g. A frame without ZYXDR is Outcome::unchanged();
/// seven bytes of 0xFF (every overwrite flag set and minus one count on every axis at once)
/// is a bus that answered nothing and is rejected.
///
/// The reset write is the first step and stands alone before its delay: the part does not
/// respond during the reset, so a transaction that follows it too soon is NAKed -- which the
/// engine would take for an absent part -- and the write itself may be NAKed when the part is
/// mid-reset from a power glitch, in which case the bring-up is repeated from the start.
/// The full scale is not a write group: XYZ_DATA_CFG is writable in standby only, which is
/// three transactions (standby, FS, active), a driver rather than an encode().
/// 0x1D with SA0 high (the Adafruit breakout), 0x1C low.
struct Mma8451 {
    static constexpr std::string_view Name = "MMA8451";
    /// NXP MMA8451Q. MMA8451Q.md:742, :1048: WHO_AM_I (0x0D) is 0x1A.
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0D, 1, true, 0xFF, 0x1A},
    };
    static constexpr Address7                Address = 0x1D;
    static constexpr std::array<Address7, 2> Addresses{0x1C, 0x1D};
    static constexpr std::size_t             RegisterBytes = 1;

    /// WHO_AM_I first, and checked before anything is written (Step::identify): 0x1C and 0x1D
    /// are shared with other parts, such as the ADXL345's alternate address.
    static constexpr std::array Init{
      Step::write({.reg = 0x2B, .payload = {0x40}}),   // RST
      Step::wait(std::chrono::milliseconds{10}),       // the reset; nothing answers during it
      Step::write({.reg = 0x2A, .payload = {0x00}}),
      Step::write({.reg = 0x0E, .payload = {0x00}}),
      Step::write({.reg = 0x2B, .payload = {0x02}}),
      Step::write({.reg = 0x2A, .payload = {0x19}}),
    };

    using State = Groups::DeviceId;

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    struct Motion {
        static constexpr auto       Period = std::chrono::milliseconds{20};
        static constexpr std::array Steps{
          Step::read({.reg = 0x00, .count = 7})};   // STATUS, OUT_X_MSB .. OUT_Z_LSB

        struct Sample {
            MicroG                      x{}, y{}, z{};
            std::array<std::int16_t, 3> raw{};   ///< 4096 counts per g
        };

        /// 4096 counts per g is 1'000'000 / 4096 = 15625 / 64 ug a count, exact.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data,
                                                              Sample const&) {
            bool allOnes = true;
            for(std::size_t i = 0; i < 7; ++i) { allOnes = allOnes && data.u8(i) == 0xFF; }
            if(allOnes) { return Outcome<Sample>::reject(); }
            if((data.u8(0) & 0x08U) == 0) { return Outcome<Sample>::unchanged(); }   // ZYXDR
            Sample sample{};
            for(std::size_t i = 0; i < 3; ++i) {
                sample.raw[i] = static_cast<std::int16_t>(data.s16be(1 + 2 * i) >> 2);
            }
            auto const g = [&](std::size_t i) { return Units::microG(sample.raw[i] * 15625 / 64); };
            sample.x     = g(0);
            sample.y     = g(1);
            sample.z     = g(2);
            return Outcome<Sample>::ok(sample);
        }
    };

    using Reads = List<Motion>;
};

}   // namespace Kvasir::I2C::Chips
