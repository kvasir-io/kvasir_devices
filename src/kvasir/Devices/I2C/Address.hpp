#pragma once

#include <concepts>
#include <cstdint>
#include <utility>

namespace Kvasir::I2C {

namespace AddressDetail {
    // Declared and never defined: reaching one while constructing an Address7 is not a constant
    // expression, so the construction fails to compile and the diagnostic names the problem.
    void I2C_address_above_0x7F_is_an_8_bit_address_use_the_7_bit_one();
    void I2C_address_is_reserved_0x00_to_0x07_or_0x78_to_0x7F_use_Address7_reserved();
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
    consteval Address7(T address)   // NOLINT(google-explicit-constructor): `Address = 0x48`
      : value{static_cast<std::uint8_t>(address)} {
        if(std::cmp_less(address, 0) || std::cmp_greater(address, 0x7F)) {
            AddressDetail::I2C_address_above_0x7F_is_an_8_bit_address_use_the_7_bit_one();
        }
        if(std::cmp_less(address, 0x08) || std::cmp_greater(address, 0x77)) {
            AddressDetail::
              I2C_address_is_reserved_0x00_to_0x07_or_0x78_to_0x7F_use_Address7_reserved();
        }
    }

    /// An address in 0x00..0x07 or 0x78..0x7F: the general call, or a part whose factory address
    /// is there anyway.
    template<std::integral T>
    [[nodiscard]] static consteval Address7 reserved(T address) {
        if(std::cmp_less(address, 0) || std::cmp_greater(address, 0x7F)) {
            AddressDetail::I2C_address_above_0x7F_is_an_8_bit_address_use_the_7_bit_one();
        }
        return Address7{Unchecked{}, static_cast<std::uint8_t>(address)};
    }

    constexpr operator std::uint8_t() const {   // NOLINT(google-explicit-constructor)
        return value;
    }

private:
    struct Unchecked {};

    constexpr Address7(Unchecked,
                       std::uint8_t address)
      : value{address} {}
};

}   // namespace Kvasir::I2C
