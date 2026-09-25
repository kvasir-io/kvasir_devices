#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

/// Reading numbers out of the bytes a device sent (on any bus: the I2C descriptions and the
/// SPI drivers share this): big- and little-endian words of 16, 24
/// and 32 bits, signed and unsigned, sign extension from an arbitrary width, packed BCD, and
/// the two CRC-8 variants these parts use. Once, and constexpr.
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

/// CRC-8, MSB first, no reflection and no final XOR. The two polynomials these parts use
/// differ only in that constant, so they are one function.
template<std::uint8_t Polynomial>
[[nodiscard]] constexpr std::uint8_t crc8(Bytes        data,
                                          std::uint8_t init) {
    std::uint8_t crc = init;
    for(std::size_t i = 0; i < data.size(); ++i) {
        crc ^= data.u8(i);
        for(int k = 0; k < 8; ++k) {
            auto const shifted = static_cast<unsigned>(crc) << 1U;
            crc = (crc & 0x80U) != 0 ? static_cast<std::uint8_t>(shifted ^ Polynomial)
                                     : static_cast<std::uint8_t>(shifted);
        }
    }
    return crc;
}

/// The CRC-8 Sensirion's parts use (SHT3x Table 20, also SHT4x, SHTC3, SGP30/40, SCD4x)
/// and Aosong copied for the AHT20: polynomial 0x31, init 0xFF. CRC(0xBEEF) = 0x92.
namespace Sensirion {
    /// `init` 0xFF is Sensirion's; the HTU21D / Si7021 use the same polynomial from 0x00.
    [[nodiscard]] constexpr std::uint8_t crc8(Bytes        data,
                                              std::uint8_t init = 0xFF) {
        return Kvasir::crc8<0x31U>(data, init);
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

    /// The three bytes a 16-bit command argument is sent as.
    constexpr std::array<std::uint8_t,
                         3>
    framed(std::uint16_t word) {
        std::array<std::byte, 2> w{static_cast<std::byte>(word >> 8),
                                   static_cast<std::byte>(word & 0xFF)};
        return {static_cast<std::uint8_t>(w[0]), static_cast<std::uint8_t>(w[1]), crc8(Bytes{w})};
    }
}   // namespace Sensirion

/// SMBus PEC (MLX90614): CRC-8 polynomial 0x07, init 0, over the whole frame including
/// the address bytes.
[[nodiscard]] constexpr std::uint8_t crc8Smbus(Bytes        data,
                                               std::uint8_t init = 0) {
    return crc8<0x07U>(data, init);
}

/// CRC-8 reflected (LSB first), no final XOR: the Dallas/Maxim CRC every 1-Wire ROM and
/// scratchpad carries is `crc8Reflected<0x8C>(data, 0)`. Bit-serial, so it needs no table.
template<std::uint8_t ReflectedPolynomial>
[[nodiscard]] constexpr std::uint8_t crc8Reflected(Bytes        data,
                                                   std::uint8_t init) {
    std::uint8_t crc = init;
    for(std::size_t i = 0; i < data.size(); ++i) {
        crc ^= data.u8(i);
        for(int k = 0; k < 8; ++k) {
            auto const shifted = static_cast<std::uint8_t>(crc >> 1U);
            crc = (crc & 0x01U) != 0 ? static_cast<std::uint8_t>(shifted ^ ReflectedPolynomial)
                                     : shifted;
        }
    }
    return crc;
}

/// The Dallas/Maxim CRC-8 (polynomial x^8 + x^5 + x^4 + 1, reflected 0x8C), init 0: 1-Wire
/// ROM codes and DS18B20 scratchpads. Maxim AN27's example ROM 02 1C B8 01 00 00 00 has the
/// CRC A2.
namespace Dallas {
    [[nodiscard]] constexpr std::uint8_t crc8(Bytes        data,
                                              std::uint8_t init = 0) {
        return Kvasir::crc8Reflected<0x8CU>(data, init);
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

}   // namespace Kvasir
