#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// ams AS5600 magnetic rotary encoder (DS000365 v1-06). Fixed address 0x36, one-byte
/// pointer. Output registers: STATUS 0x0B (MD bit 5 magnet detected, ML bit 4 too weak, MH
/// bit 3 too strong), RAW ANGLE 0x0C/0x0D and ANGLE 0x0E/0x0F (12 bit, big-endian), AGC
/// 0x1A, MAGNITUDE 0x1B/0x1C. The angle is read every 20 ms, the gain and magnitude every
/// 500 ms.
///
/// The pointer increments after each byte, except that ANGLE, RAW ANGLE and MAGNITUDE hold it
/// on their own word -- a read addressed to one of them wraps back to its high byte -- and
/// that only when the read was addressed to the word's high byte ("Automatic Increment of the
/// Address Pointer"). A read addressed to STATUS walks on through RAW ANGLE and ANGLE. STATUS
/// + RAW ANGLE and ANGLE are still two reads, each addressed where it starts, which holds
/// whichever way a given part treats the pointer.
struct As5600 {
    static constexpr std::string_view        Name    = "AS5600";
    static constexpr Address7                Address = 0x36;
    static constexpr std::array<Address7, 1> Addresses{0x36};
    static constexpr std::size_t             RegisterBytes = 1;

    struct Angle {
        static constexpr auto       Period = std::chrono::milliseconds{20};
        static constexpr std::array Steps{
          Step::read({.reg = 0x0B, .count = 3, .offset = 0}),    // STATUS, RAW ANGLE
          Step::read({.reg = 0x0E, .count = 2, .offset = 3})};   // ANGLE

        struct Sample {
            CentiDegree   angle{};         ///< RAW ANGLE: 4096 counts over the full turn
            std::uint16_t raw{};           ///< RAW ANGLE as read, 0..4095
            std::uint16_t scaledAngle{};   ///< ANGLE: 0..4095 over the ZPOS/MPOS/MANG range
            bool          magnet{};
            bool          tooWeak{};
            bool          tooStrong{};
        };

        /// A frame with MH and ML both set, or with bits above the twelve of either angle
        /// word, is not one the part produces.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            auto const status = data.u8(0);
            auto const raw    = data.be16(1);
            auto const scaled = data.be16(3);
            if((status & 0x18U) == 0x18U || (raw & 0xF000U) != 0 || (scaled & 0xF000U) != 0) {
                return Outcome<Sample>::reject();
            }
            Sample sample{};
            sample.magnet      = (status & 0x20U) != 0;
            sample.tooWeak     = (status & 0x10U) != 0;
            sample.tooStrong   = (status & 0x08U) != 0;
            sample.raw         = raw;
            sample.scaledAngle = scaled;
            sample.angle = Units::centiDegree(static_cast<std::int32_t>(sample.raw) * 36000 / 4096);
            return Outcome<Sample>::ok(sample);
        }
    };

    struct Gain {
        static constexpr auto       Period = std::chrono::milliseconds{500};
        static constexpr std::array Steps{
          Step::read({.reg = 0x1A, .count = 1, .offset = 0}),    // AGC
          Step::read({.reg = 0x1B, .count = 2, .offset = 1})};   // MAGNITUDE

        struct Sample {
            std::uint8_t  agc{};
            std::uint16_t magnitude{};
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            auto const magnitude = data.be16(1);
            if((magnitude & 0xF000U) != 0) { return Outcome<Sample>::reject(); }
            return Outcome<Sample>::ok({data.u8(0), magnitude});
        }
    };

    using Reads = List<Angle, Gain>;
};

}   // namespace Kvasir::I2C::Chips
