#pragma once

#include "Descriptors.hpp"
#include "Mixins.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <utility>

// Microsoft OS 2.0 descriptors: Windows (8.1+) binds WinUSB to the vendor-specific interfaces
// without an INF. For a device with bcdUSB >= 2.01 it reads the BOS descriptor, finds the
// MS OS 2.0 platform capability and fetches the descriptor set with the vendor request named
// there.
//
// Windows remembers per VID/PID/bcdDevice that it has asked. For a device it has seen before,
// bump bcdDevice or delete HKLM\SYSTEM\CurrentControlSet\Control\usbflags\<VIDPIDREV>.
namespace Kvasir::USB::WinUsb {
namespace MsOs20 {
    static constexpr std::uint16_t BcdUSB          = 0x0201;
    static constexpr std::uint32_t WindowsVersion  = 0x06030000;   // Windows 8.1
    static constexpr std::uint16_t DescriptorIndex = 7;            // MS_OS_20_DESCRIPTOR_INDEX

    enum class DescriptorType : std::uint16_t {
        setHeader                 = 0,
        subsetHeaderConfiguration = 1,
        subsetHeaderFunction      = 2,
        featureCompatibleId       = 3,
        featureRegProperty        = 4,
    };

    enum class RegistryType : std::uint16_t { multiSz = 7 };

    struct [[gnu::packed]] SetHeader {
        std::uint16_t  wLength{sizeof(SetHeader)};
        DescriptorType wDescriptorType{DescriptorType::setHeader};
        std::uint32_t  dwWindowsVersion{WindowsVersion};
        std::uint16_t  wTotalLength;
    };

    struct [[gnu::packed]] ConfigurationSubset {
        std::uint16_t  wLength{sizeof(ConfigurationSubset)};
        DescriptorType wDescriptorType{DescriptorType::subsetHeaderConfiguration};
        std::uint8_t   bConfigurationValue{0};   // Windows reads this as the index, not the value
        std::uint8_t   bReserved{};
        std::uint16_t  wTotalLength;
    };

    struct [[gnu::packed]] FunctionSubset {
        std::uint16_t  wLength{sizeof(FunctionSubset)};
        DescriptorType wDescriptorType{DescriptorType::subsetHeaderFunction};
        std::uint8_t   bFirstInterface;
        std::uint8_t   bReserved{};
        std::uint16_t  wSubsetLength;
    };

    struct [[gnu::packed]] CompatibleId {
        std::uint16_t  wLength{sizeof(CompatibleId)};
        DescriptorType wDescriptorType{DescriptorType::featureCompatibleId};
        char           CompatibleID[8]{"WINUSB"};
        char           SubCompatibleID[8]{};
    };

    // DeviceInterfaceGUIDs as REG_MULTI_SZ holding one GUID
    struct [[gnu::packed]] DeviceInterfaceGuids {
        static constexpr std::size_t GuidLength = 38;   // {xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}

        std::uint16_t  wLength{sizeof(DeviceInterfaceGuids)};
        DescriptorType wDescriptorType{DescriptorType::featureRegProperty};
        RegistryType   wPropertyDataType{RegistryType::multiSz};
        std::uint16_t  wPropertyNameLength{sizeof(PropertyName)};
        char16_t       PropertyName[21]{u"DeviceInterfaceGUIDs"};
        std::uint16_t  wPropertyDataLength{sizeof(PropertyData)};
        char16_t       PropertyData[GuidLength + 2]{};   // GUID, NUL, list-terminating NUL
    };

    struct [[gnu::packed]] Features {
        CompatibleId         compatibleId{};
        DeviceInterfaceGuids deviceInterfaceGuids{};
    };

    struct [[gnu::packed]] Function {
        FunctionSubset subset;
        Features       features;
    };

    struct [[gnu::packed]] PlatformCapability
      : USB::detail::DescriptorBase<PlatformCapability, USB::DescriptorType::deviceCapability> {
        std::uint8_t  bDevCapabilityType{0x05};   // PLATFORM
        std::uint8_t  bReserved{};
        std::uint8_t  PlatformCapabilityUUID[16]{// {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
                                                 0xDF,
                                                 0x60,
                                                 0xDD,
                                                 0xD8,
                                                 0x89,
                                                 0x45,
                                                 0xC7,
                                                 0x4C,
                                                 0x9C,
                                                 0xD2,
                                                 0x65,
                                                 0x9D,
                                                 0x9E,
                                                 0x64,
                                                 0x8A,
                                                 0x9F};
        std::uint32_t dwWindowsVersion{WindowsVersion};
        std::uint16_t wMSOSDescriptorSetTotalLength;
        std::uint8_t  bMS_VendorCode;
        std::uint8_t  bAltEnumCode{};
    };

    static_assert(sizeof(SetHeader) == 10);
    static_assert(sizeof(ConfigurationSubset) == 8);
    static_assert(sizeof(FunctionSubset) == 8);
    static_assert(sizeof(CompatibleId) == 20);
    static_assert(sizeof(DeviceInterfaceGuids) == 132);
    static_assert(sizeof(PlatformCapability) == 28);

    constexpr bool isGuid(std::string_view guid) {
        if(guid.size() != DeviceInterfaceGuids::GuidLength || guid.front() != '{'
           || guid.back() != '}')
        {
            return false;
        }
        for(std::size_t i = 1; i < guid.size() - 1; ++i) {
            char const c    = guid[i];
            bool const dash = i == 9 || i == 14 || i == 19 || i == 24;
            bool const hexDigit
              = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
            if(dash ? c != '-' : !hexDigit) { return false; }
        }
        return true;
    }

    constexpr Features makeFeatures(std::string_view guid) {
        Features features{};
        // element by element: std::ranges::copy would take a pointer into the packed struct
        for(std::size_t i = 0; i < guid.size(); ++i) {
            features.deviceInterfaceGuids.PropertyData[i] = static_cast<char16_t>(guid[i]);
        }
        return features;
    }

    struct VendorInterfaces {
        std::array<std::uint8_t, 16> numbers{};
        std::size_t                  count{};
    };

    // Interfaces no Windows inbox driver claims: class 255, alternate setting 0.
    constexpr VendorInterfaces findVendorInterfaces(std::span<std::byte const> config) {
        VendorInterfaces found{};
        USB::Descriptors::detail::forEachDescriptor(config, [&](std::span<std::byte const> bytes) {
            if(bytes.size() != sizeof(USB::Descriptors::Interface)
               || bytes[1] != std::byte{std::to_underlying(USB::DescriptorType::interface)})
            {
                return;
            }
            std::array<std::byte, sizeof(USB::Descriptors::Interface)> raw{};
            std::ranges::copy(bytes, raw.begin());
            auto const descriptor = std::bit_cast<USB::Descriptors::Interface>(raw);
            if(descriptor.bAlternateSetting == 0 && descriptor.bInterfaceClass == 0xFF) {
                found.numbers[found.count++] = descriptor.bInterfaceNumber;
            }
        });
        return found;
    }

    // A composite device describes each function in its own subset, a single-interface
    // device carries the features directly.
    template<bool        Composite,
             std::size_t InterfaceCount>
    consteval auto makeDescriptorSetArray(std::array<std::uint8_t,
                                                     InterfaceCount> const& interfaces,
                                          std::string_view                  guid) {
        auto const features = makeFeatures(guid);
        if constexpr(Composite) {
            std::array<std::byte, InterfaceCount * sizeof(Function)> functions{};
            for(std::size_t i = 0; i < InterfaceCount; ++i) {
                auto const bytes = std::bit_cast<std::array<std::byte, sizeof(Function)>>(Function{
                  .subset{.bFirstInterface = interfaces[i], .wSubsetLength = sizeof(Function)},
                  .features = features
                });
                std::ranges::copy(bytes,
                                  std::span{functions}.subspan(i * sizeof(Function)).begin());
            }
            std::uint16_t const configurationLength
              = sizeof(ConfigurationSubset) + functions.size();
            return USB::Descriptors::detail::generateArray(
              SetHeader{.wTotalLength = sizeof(SetHeader) + configurationLength},
              ConfigurationSubset{.wTotalLength = configurationLength},
              functions);
        } else {
            return USB::Descriptors::detail::generateArray(
              SetHeader{.wTotalLength = sizeof(SetHeader) + sizeof(Features)},
              features);
        }
    }

    consteval auto makeBosDescriptorArray(std::uint16_t descriptorSetLength,
                                          std::uint8_t  vendorCode) {
        return USB::Descriptors::detail::generateArray(
          USB::Descriptors::Bos{.wTotalLength
                                = sizeof(USB::Descriptors::Bos) + sizeof(PlatformCapability),
                                .bNumDeviceCaps = 1},
          PlatformCapability{.wMSOSDescriptorSetTotalLength = descriptorSetLength,
                             .bMS_VendorCode                = vendorCode});
    }
}   // namespace MsOs20

// Binds WinUSB to every vendor-specific interface of the device; has no interface or endpoint
// of its own. All those interfaces share one DeviceInterfaceGUID, applications tell them apart
// by the MI_xx part of the device path.
//
// Optional Config:
//   WinUsbDeviceInterfaceGuid  "{...}", give each product its own
//   MsOs20VendorCode           bRequest of the descriptor set request
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber>
struct Mixin {
private:
    friend Derived;
    friend struct Kvasir::USB::detail::MixinTraits;

    static constexpr std::size_t   InterfaceCount = 0;
    static constexpr std::size_t   EndpointCount  = 0;
    static constexpr std::uint16_t BcdUSB         = MsOs20::BcdUSB;

    static constexpr std::array<std::byte, 0> InterfaceDescriptor{};

    static constexpr std::string_view DeviceInterfaceGuid = [] {
        if constexpr(requires { Config::WinUsbDeviceInterfaceGuid; }) {
            return std::string_view{Config::WinUsbDeviceInterfaceGuid};
        } else {
            return std::string_view{"{2B8E7A3C-5B0D-4E61-9C7A-6E3F1D2A4B58}"};
        }
    }();

    static constexpr std::uint8_t VendorCode = [] {
        if constexpr(requires { Config::MsOs20VendorCode; }) {
            return static_cast<std::uint8_t>(Config::MsOs20VendorCode);
        } else {
            return std::uint8_t{0x57};
        }
    }();

    static_assert(
      MsOs20::isGuid(DeviceInterfaceGuid),
      "WinUsbDeviceInterfaceGuid must look like {xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}");

    // Needs the whole configuration descriptor, so only instantiated from a callback body.
    struct Tables {
        static constexpr std::span<std::byte const> ConfigDescriptor = Derived::configDescriptor();

        static constexpr auto Found = MsOs20::findVendorInterfaces(ConfigDescriptor);
        static_assert(Found.count > 0,
                      "WinUsb::Mixin on a device without a vendor interface");

        static constexpr auto Interfaces = [] {
            std::array<std::uint8_t, Found.count> interfaces{};
            std::ranges::copy(std::span{Found.numbers}.first(Found.count), interfaces.begin());
            return interfaces;
        }();

        static constexpr bool Composite = [] {
            std::array<std::byte, sizeof(USB::Descriptors::Configuration)> raw{};
            std::ranges::copy(ConfigDescriptor.first(raw.size()), raw.begin());
            return std::bit_cast<USB::Descriptors::Configuration>(raw).bNumInterfaces > 1;
        }();

        static constexpr auto DescriptorSet
          = MsOs20::makeDescriptorSetArray<Composite>(Interfaces, DeviceInterfaceGuid);

        static constexpr auto Bos
          = MsOs20::makeBosDescriptorArray(static_cast<std::uint16_t>(DescriptorSet.size()),
                                           VendorCode);
    };

    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        if(pkt.recipient() != SetupPacket::Recipient::device
           || pkt.direction() != SetupPacket::Direction::deviceToHost)
        {
            return false;
        }
        if(pkt.type() == SetupPacket::Type::standard
           && pkt.bRequest == SetupPacket::Request::getDescriptor
           && pkt.descriptorType() == DescriptorType::bos)
        {
            return Derived::ep0INDataPhase(Tables::Bos, pkt.wLength);
        }
        if(pkt.type() == SetupPacket::Type::vendor && std::to_underlying(pkt.bRequest) == VendorCode
           && pkt.wValue == 0 && pkt.wIndex == MsOs20::DescriptorIndex)
        {
            return Derived::ep0INDataPhase(Tables::DescriptorSet, pkt.wLength);
        }
        return false;
    }
};
}   // namespace Kvasir::USB::WinUsb
