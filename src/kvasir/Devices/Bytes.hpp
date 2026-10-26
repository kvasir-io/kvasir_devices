#pragma once

#include "kvasir/Util/Crc.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

/// Reading numbers out of the bytes a device sent (on any bus: the I2C descriptions and the
/// SPI drivers share this): big- and little-endian words of 16, 24
/// and 32 bits, signed and unsigned, sign extension from an arbitrary width, packed BCD, and
/// the CRC-8s these parts use (named presets of the SDK's Kvasir::Crc::Engine). Once, and
/// constexpr.
namespace Kvasir {

struct Bytes {
    std::span<std::byte const> bytes{};

    constexpr Bytes() = default;

    constexpr explicit Bytes(std::span<std::byte const> span) : bytes{span} {}

    template<std::size_t N>
    constexpr explicit Bytes(std::array<std::byte,
                                        N> const& array)
      : bytes{array} {}

    [[nodiscard]] constexpr std::size_t size() const { return bytes.size(); }

    [[nodiscard]] constexpr Bytes sub(std::size_t offset,
                                      std::size_t count) const {
        return Bytes{bytes.subspan(offset, count)};
    }

    /// One byte as a std::byte, for a caller assembling a frame of its own.
    [[nodiscard]] constexpr std::byte at(std::size_t i) const { return bytes[i]; }

    [[nodiscard]] constexpr std::uint8_t u8(std::size_t i) const {
        return static_cast<std::uint8_t>(bytes[i]);
    }

    [[nodiscard]] constexpr std::int8_t s8(std::size_t i) const {
        return static_cast<std::int8_t>(u8(i));
    }

    [[nodiscard]] constexpr std::uint16_t be16(std::size_t i) const {
        return static_cast<std::uint16_t>((u8(i) << 8) | u8(i + 1));
    }

    [[nodiscard]] constexpr std::uint16_t le16(std::size_t i) const {
        return static_cast<std::uint16_t>(u8(i) | (u8(i + 1) << 8));
    }

    [[nodiscard]] constexpr std::int16_t s16be(std::size_t i) const {
        return static_cast<std::int16_t>(be16(i));
    }

    [[nodiscard]] constexpr std::int16_t s16le(std::size_t i) const {
        return static_cast<std::int16_t>(le16(i));
    }

    [[nodiscard]] constexpr std::uint32_t be24(std::size_t i) const {
        return (static_cast<std::uint32_t>(u8(i)) << 16)
             | (static_cast<std::uint32_t>(u8(i + 1)) << 8) | u8(i + 2);
    }

    [[nodiscard]] constexpr std::uint32_t le24(std::size_t i) const {
        return u8(i) | (static_cast<std::uint32_t>(u8(i + 1)) << 8)
             | (static_cast<std::uint32_t>(u8(i + 2)) << 16);
    }

    [[nodiscard]] constexpr std::uint32_t be32(std::size_t i) const {
        return (be24(i) << 8) | u8(i + 3);
    }

    [[nodiscard]] constexpr std::uint32_t le32(std::size_t i) const {
        return le24(i) | (static_cast<std::uint32_t>(u8(i + 3)) << 24);
    }

    /// Two BCD digits, the high nibble masked by `mask` first (an RTC's flag bits).
    [[nodiscard]] constexpr std::uint8_t bcd(std::size_t  i,
                                             std::uint8_t mask = 0xFF) const {
        auto const value = static_cast<std::uint8_t>(u8(i) & mask);
        return static_cast<std::uint8_t>((value >> 4) * 10 + (value & 0x0F));
    }

    /// Sign-extend an n-bit two's complement value.
    [[nodiscard]] static constexpr std::int32_t signExtend(std::uint32_t value,
                                                           unsigned      bits) {
        auto const m = 1U << (bits - 1);
        return static_cast<std::int32_t>((value ^ m)) - static_cast<std::int32_t>(m);
    }
};

constexpr std::uint8_t toBcd(std::uint8_t value) {
    return static_cast<std::uint8_t>(((value / 10) << 4) | (value % 10));
}

/// The CRC-8 Sensirion's parts use (SHT3x Table 20, also SHT4x, SHTC3, SGP30/40, SCD4x)
/// and Aosong copied for the AHT20: CRC-8/NRSC-5, polynomial 0x31, init 0xFF. CRC(0xBEEF) =
/// 0x92. (The HTU21D / Si7021 use the same polynomial from 0x00: Kvasir::Crc::Crc8Htu21d.)
namespace Sensirion {
    [[nodiscard]] constexpr std::uint8_t crc8(Bytes data) {
        return Crc::Crc8Sensirion<>::compute(data.bytes);
    }

    static_assert(crc8(Bytes{
                    std::array{std::byte{0xBE},
                               std::byte{0xEF}}
    })
                  == 0x92);

    /// A 16-bit word followed by its CRC, as every Sensirion result is framed.
    [[nodiscard]] constexpr bool wordOk(Bytes       data,
                                        std::size_t i) {
        return crc8(data.sub(i, 2)) == data.u8(i + 2);
    }

    /// The signal conversions the SHT3x (4.13), SHTC3 (5.11) and SCD4x share, in hundredths and integer arithmetic:
    /// T = -45 + 175 x S / (2^16 - 1) degC and RH = 100 x S / (2^16 - 1) %RH (the SCD4x sheet's read_measurement
    /// table, md l.433-434, has them as text; the SHT3x/SHTC3 sheets as pictures). The SHT4x's humidity is another
    /// formula (-6 + 125 x S / (2^16 - 1), clamped): its own.
    [[nodiscard]] constexpr std::int32_t centiDegC(std::uint16_t raw) {
        return -4500 + static_cast<std::int32_t>((17500LL * raw) / 65535);
    }

    [[nodiscard]] constexpr std::uint32_t centiPercentRh(std::uint16_t raw) {
        return static_cast<std::uint32_t>((10000ULL * raw) / 65535);
    }

    /// The three bytes a 16-bit command argument is sent as.
    constexpr std::array<std::uint8_t,
                         3>
    framed(std::uint16_t word) {
        std::array<std::byte, 2> w{static_cast<std::byte>(word >> 8),
                                   static_cast<std::byte>(word & 0xFF)};
        return {static_cast<std::uint8_t>(w[0]), static_cast<std::uint8_t>(w[1]), crc8(Bytes{w})};
    }
}   // namespace Sensirion

/// SMBus PEC (MLX90614): CRC-8/SMBUS, polynomial 0x07, init 0, over the whole frame including
/// the address bytes.
[[nodiscard]] constexpr std::uint8_t crc8Smbus(Bytes data) {
    return Crc::Crc8Smbus<>::compute(data.bytes);
}

/// The Dallas/Maxim CRC-8 (CRC-8/MAXIM-DOW: x^8 + x^5 + x^4 + 1, reflected, init 0): 1-Wire
/// ROM codes and DS18B20 scratchpads. Maxim AN27's example ROM 02 1C B8 01 00 00 00 has the
/// CRC A2.
namespace Dallas {
    [[nodiscard]] constexpr std::uint8_t crc8(Bytes data) {
        return Crc::Crc8MaximDow<>::compute(data.bytes);
    }

    static_assert(crc8(Bytes{
                    std::array{std::byte{0x02},
                               std::byte{0x1C},
                               std::byte{0xB8},
                               std::byte{0x01},
                               std::byte{0x00},
                               std::byte{0x00},
                               std::byte{0x00}}
    })
                  == 0xA2);
}   // namespace Dallas

/// The two bytes of a 16-bit word, high first / low first: what every two-byte register
/// encode() spells out by hand otherwise.
constexpr void putBe16(std::span<std::byte> out,
                       std::size_t          i,
                       std::uint16_t        value) {
    out[i]     = static_cast<std::byte>(value >> 8);
    out[i + 1] = static_cast<std::byte>(value & 0xFF);
}

constexpr void putLe16(std::span<std::byte> out,
                       std::size_t          i,
                       std::uint16_t        value) {
    out[i]     = static_cast<std::byte>(value & 0xFF);
    out[i + 1] = static_cast<std::byte>(value >> 8);
}

/// The low three bytes of `value`, high first (a 24-bit flash address).
constexpr void putBe24(std::span<std::byte> out,
                       std::size_t          i,
                       std::uint32_t        value) {
    out[i]     = static_cast<std::byte>((value >> 16) & 0xFF);
    out[i + 1] = static_cast<std::byte>((value >> 8) & 0xFF);
    out[i + 2] = static_cast<std::byte>(value & 0xFF);
}

/// The four bytes of `value`, high first (an SD command argument).
constexpr void putBe32(std::span<std::byte> out,
                       std::size_t          i,
                       std::uint32_t        value) {
    out[i] = static_cast<std::byte>(value >> 24);
    putBe24(out, i + 1, value);
}

}   // namespace Kvasir
