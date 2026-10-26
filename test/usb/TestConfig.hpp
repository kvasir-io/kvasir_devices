#pragma once

#include <cstdint>
#include <kvasir/Devices/USB/Descriptors.hpp>
#include <support/FakeClock.hpp>

/// What the USB tests share: their clock, the config members every test device has, and the
/// bmRequestType values and descriptor wValues the host sends, by name.
namespace Kvasir::Test::Usb {

using Clock = FakeClock;

/// A test's Config derives from this and adds its product (productString, productId) and
/// whatever its mixins read (sendBufferSize, ...); a member it redefines hides this one.
struct TestConfig {
    static constexpr auto manufacturerString = "Kvasir host test";
    static constexpr auto serialNumberString = "1";
    static constexpr auto productVersionBcd  = 0x0100;
    static constexpr auto vendorId           = 0x1209;
};

using Setup = Kvasir::USB::SetupPacket;

/// bmRequestType from its three parts: the inverse of SetupPacket's direction() (bit 7), type()
/// (bits 6..5) and recipient() (bits 4..0).
constexpr std::uint8_t requestType(Setup::Direction d,
                                   Setup::Type      t,
                                   Setup::Recipient r) {
    return static_cast<std::uint8_t>((std::to_underlying(d) << 7) | (std::to_underlying(t) << 5)
                                     | std::to_underlying(r));
}

inline constexpr auto toDevice
  = requestType(Setup::Direction::hostToDevice, Setup::Type::standard, Setup::Recipient::device);
inline constexpr auto fromDevice
  = requestType(Setup::Direction::deviceToHost, Setup::Type::standard, Setup::Recipient::device);
inline constexpr auto toInterface
  = requestType(Setup::Direction::hostToDevice, Setup::Type::standard, Setup::Recipient::interface);
inline constexpr auto fromInterface
  = requestType(Setup::Direction::deviceToHost, Setup::Type::standard, Setup::Recipient::interface);
inline constexpr auto toEndpoint
  = requestType(Setup::Direction::hostToDevice, Setup::Type::standard, Setup::Recipient::endpoint);
inline constexpr auto fromEndpoint
  = requestType(Setup::Direction::deviceToHost, Setup::Type::standard, Setup::Recipient::endpoint);
inline constexpr auto classToInterface
  = requestType(Setup::Direction::hostToDevice, Setup::Type::classT, Setup::Recipient::interface);
inline constexpr auto classFromInterface
  = requestType(Setup::Direction::deviceToHost, Setup::Type::classT, Setup::Recipient::interface);
inline constexpr auto vendorToInterface
  = requestType(Setup::Direction::hostToDevice, Setup::Type::vendor, Setup::Recipient::interface);
inline constexpr auto vendorFromDevice
  = requestType(Setup::Direction::deviceToHost, Setup::Type::vendor, Setup::Recipient::device);

static_assert(toDevice == 0x00 && fromDevice == 0x80 && toInterface == 0x01 && fromInterface == 0x81
              && toEndpoint == 0x02 && fromEndpoint == 0x82 && classToInterface == 0x21
              && classFromInterface == 0xA1 && vendorToInterface == 0x41
              && vendorFromDevice == 0xC0);

/// wValue of GET_DESCRIPTOR: the type in the high byte, the index in the low one.
constexpr std::uint16_t descriptorValue(std::uint8_t type,
                                        std::uint8_t index = 0) {
    return static_cast<std::uint16_t>((type << 8) | index);
}

constexpr std::uint16_t descriptorValue(Kvasir::USB::DescriptorType type,
                                        std::uint8_t                index = 0) {
    return descriptorValue(std::to_underlying(type), index);
}
}   // namespace Kvasir::Test::Usb
