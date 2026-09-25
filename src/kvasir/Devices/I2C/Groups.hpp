#pragma once

#include "Device.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

/// Common shapes of write/read groups, so a description does not spell them out: a one-byte
/// register a write group owns, and the identification register a bring-up reads first.
namespace Kvasir::I2C::Groups {

/// A one-byte register the application writes. No Initial: the part keeps what it came up with
/// until something is set, and a value set is put back after a later bring-up.
template<std::uint16_t Reg>
struct Byte {
    using Value                        = std::uint8_t;
    static constexpr std::size_t Bytes = 1;

    [[nodiscard]] static constexpr Step encode(Value const&         value,
                                               std::span<std::byte> buffer) {
        buffer[0] = static_cast<std::byte>(value);
        return Step::writeBuffer({.reg = Reg, .offset = 0, .count = 1});
    }
};

/// A one-byte register that holds `Init` from every bring-up on, unless the application set it.
template<std::uint16_t Reg, std::uint8_t Init>
struct InitialByte : Byte<Reg> {
    static constexpr typename Byte<Reg>::Value Initial = Init;
};

/// A 16-bit big-endian register that holds `Init` from every bring-up on, unless the
/// application set it: the TI monitors' ADC_CONFIG and SHUNT_CAL words.
template<std::uint16_t Reg, std::uint16_t Init>
struct Word {
    using Value                          = std::uint16_t;
    static constexpr std::size_t Bytes   = 2;
    static constexpr Value       Initial = Init;

    [[nodiscard]] static constexpr Step encode(Value const&         value,
                                               std::span<std::byte> buffer) {
        putBe16(buffer, 0, value);
        return Step::writeBuffer({.reg = Reg, .offset = 0, .count = 2});
    }
};

/// An InitialByte read back `Timing::VerifyDelay` after every write and every
/// `Timing::VerifyInterval` after that, and written again when it does not match (Device.hpp's
/// verify). Both are std::chrono durations:
///
///     struct Checked {
///         static constexpr std::chrono::milliseconds VerifyDelay{100};
///         static constexpr std::chrono::milliseconds VerifyInterval{1000};
///     };
///     using Enable = Groups::VerifiedByte<0x80, 0x00, Checked>;
template<std::uint16_t Reg, std::uint8_t Init, typename Timing>
struct VerifiedByte : InitialByte<Reg, Init> {
    static constexpr std::chrono::milliseconds VerifyDelay    = Timing::VerifyDelay;
    static constexpr std::chrono::milliseconds VerifyInterval = Timing::VerifyInterval;
};

/// The State of a description that publishes its identity register and nothing else: filled by its
/// `identified()` hook from what the engine read for the description's `Identity` (Device.hpp).
struct DeviceId {
    std::uint16_t deviceId{};
};

}   // namespace Kvasir::I2C::Groups
