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

namespace Lis3dhDetail {
    /// CTRL_REG4 FS (5:4): +-2, +-4, +-8, +-16 g.
    enum class Range : std::uint8_t { g2 = 0, g4 = 1, g8 = 2, g16 = 3 };

    /// Micro-g per digit of the twelve-bit high-resolution reading (Table 3: 1, 2, 4 and
    /// 12 mg/digit).
    inline constexpr std::array<MicroG, 4> AccelPerDigit{Units::microG(1000),
                                                         Units::microG(2000),
                                                         Units::microG(4000),
                                                         Units::microG(12000)};
}   // namespace Lis3dhDetail

/// ST LIS3DH (Doc ID 17530). One-byte sub-address; bit 7 set makes a multi-byte read
/// auto-increment (5.1.1). Bring-up: WHO_AM_I 0x0F = 0x33; CTRL_REG1 0x20 = 0x57 (100 Hz,
/// X Y Z on); CTRL_REG4 0x23 = 0x88 (BDU, high resolution, 2 g: 1 mg per digit after
/// >> 4, Table 3; Table 9 calls LPen 0 with HR 1 "normal mode", but CTRL_REG4's HR bit is
/// "high-resolution output mode", and Linux st_accel runs it as the
/// 12-bit mode). Data: six bytes from 0x28 | 0x80, little-endian, left aligned 12 bit.
/// The range is the Initial of the Range write group and can be changed at run time.
/// 0x18 (SA0 low) or 0x19.
template<Lis3dhDetail::Range Range = Lis3dhDetail::Range::g2>
struct Lis3dhX {
    static constexpr std::string_view Name = "LIS3DH";
    /// ST LIS3DH. LIS3DH.md:1059: WHO_AM_I (0Fh) is 33h.
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x0F, 1, true, 0xFF, 0x33},
    };
    static constexpr Address7                Address = 0x18;
    static constexpr std::array<Address7, 2> Addresses{0x18, 0x19};
    static constexpr std::size_t             RegisterBytes = 1;

    /// Table 5 gives a turn-on time of 1 ms at 100 Hz; the 5 ms wait after power-up is margin.
    static constexpr auto StartupDelay = std::chrono::milliseconds{5};

    static constexpr std::uint8_t RangeCode = static_cast<std::uint8_t>(Range);
    /// BDU, HR, the range.
    static constexpr std::uint8_t Ctrl4 = static_cast<std::uint8_t>(0x88U | (RangeCode << 4U));

    static constexpr std::array Init{
      Step::write({.reg = 0x20, .payload = {0x57}}),
      Step::write({.reg = 0x23, .payload = {Ctrl4}}),
    };

    struct State : Groups::DeviceId {
        std::uint8_t range{RangeCode};   ///< CTRL_REG4 FS as written now
    };

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.range    = RangeCode;
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    struct Motion {
        static constexpr auto       Period = std::chrono::milliseconds{20};
        static constexpr std::array Steps{Step::read({.reg = 0x80 | 0x28, .count = 6})};

        struct Sample {
            MicroG x{}, y{}, z{};
        };

        /// Left aligned: the twelve significant bits sit in 15:4.
        [[nodiscard]] static constexpr Sample decode(Bytes        data,
                                                     State const& state) {
            auto const perDigit = Units::value(Lis3dhDetail::AccelPerDigit[state.range & 0x03U]);
            auto const axis     = [&](std::size_t i) {
                return Units::microG(static_cast<std::int32_t>(data.s16le(2 * i) >> 4) * perDigit);
            };
            return {axis(0), axis(1), axis(2)};
        }
    };

    /// CTRL_REG4's FS (5:4), with BDU and HR kept; changeable at run time.
    struct RangeSetting {
        using Value                          = Lis3dhDetail::Range;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Range;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(0x88U | (static_cast<unsigned>(value) << 4U));
            return Step::writeBuffer({.reg = 0x23, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.range = static_cast<std::uint8_t>(value);
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<RangeSetting>;
};

/// The part at +-2 g.
using Lis3dh = Lis3dhX<>;

}   // namespace Kvasir::I2C::Chips
