#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace Kvasir::I2C {

namespace AddressDetail {
    // Declared and never defined: reaching one while constructing an Address7 is not a constant
    // expression, so the construction fails to compile and the diagnostic names the problem.
    void i2cAddressAbove0x7FIsAn8BitAddressUseThe7BitOne();
    void i2cAddressIsReserved0x00To0x07Or0x78To0x7FUseAddress7Reserved();
}   // namespace AddressDetail

/// A 7-bit I2C address, checked where it is written. Only a constant makes one, so a datasheet's
/// 8-bit form (0x90 for 0x48) or an address in the two reserved blocks is a compile error, and a
/// register or a channel number cannot turn into an address at run time:
///
///     static constexpr Address7 Address = 0x48;
///     static constexpr std::array<Address7, 2> Addresses{0x48, 0x49};
///     template<Address7 Addr> struct Part;        // Part<0x20>
///
/// A part that really sits in a reserved block says so with `Address7::reserved(0x78)`. It reads
/// back as the address byte (`std::uint8_t`), which is what the bus, the logs and the scans use.
struct Address7 {
    std::uint8_t value;

    template<std::integral T>
    consteval Address7(T address)   // implicit: `Address = 0x48`
      : value{static_cast<std::uint8_t>(address)} {
        if(std::cmp_less(address, 0) || std::cmp_greater(address, 0x7F)) {
            AddressDetail::i2cAddressAbove0x7FIsAn8BitAddressUseThe7BitOne();
        }
        if(std::cmp_less(address, 0x08) || std::cmp_greater(address, 0x77)) {
            AddressDetail::i2cAddressIsReserved0x00To0x07Or0x78To0x7FUseAddress7Reserved();
        }
    }

    /// An address in 0x00..0x07 or 0x78..0x7F: the general call, or a part whose factory address
    /// is there anyway.
    template<std::integral T>
    [[nodiscard]] static consteval Address7 reserved(T address) {
        if(std::cmp_less(address, 0) || std::cmp_greater(address, 0x7F)) {
            AddressDetail::i2cAddressAbove0x7FIsAn8BitAddressUseThe7BitOne();
        }
        return Address7{Unchecked{}, static_cast<std::uint8_t>(address)};
    }

    constexpr operator std::uint8_t() const { return value; }

private:
    struct Unchecked {};

    constexpr Address7(Unchecked,
                       std::uint8_t address)
      : value{address} {}
};

/// The `N` consecutive addresses from `First` on, for a family whose address pins pick one of a
/// contiguous run (INA219: 0x40..0x4F is `addressRun<0x40, 16>()`). Every address of the run is
/// checked as `Address7` checks one, so a run that reaches a reserved block is a compile error.
template<int         First,
         std::size_t N>
[[nodiscard]] consteval std::array<Address7,
                                   N>
addressRun() {
    static_assert(N > 0, "an address run holds at least one address");
    return []<std::size_t... I>(std::index_sequence<I...>) consteval {
        return std::array<Address7, N>{Address7{First + static_cast<int>(I)}...};
    }(std::make_index_sequence<N>{});
}

}   // namespace Kvasir::I2C
