#pragma once

#include "BulkEndpoints.hpp"
#include "Descriptors.hpp"
#include "Device.hpp"
#include "Mixins.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir::USB::SimpleBulk {
namespace Descriptors {
    /// One vendor bulk interface: one IN endpoint, plus the matching OUT endpoint when the
    /// interface is bidirectional. bInterfaceProtocol tells a host the device's interfaces apart.
    template<std::uint8_t InterfaceID,
             std::uint8_t DataEndpointID,
             bool         Bidirectional,
             std::uint8_t Protocol>
    consteval auto makeInterfaceDescriptorArrays() {
        constexpr USB::Descriptors::Interface InterfaceDescriptor{
          .bInterfaceNumber{InterfaceID},
          .bAlternateSetting{0},
          .bNumEndpoints{Bidirectional ? std::uint8_t{2} : std::uint8_t{1}},
          .bInterfaceClass{255},
          .bInterfaceSubClass{0},
          .bInterfaceProtocol{Protocol}};

        constexpr USB::Descriptors::Endpoint DataInEndpointDescriptor{
          .bEndpointAddress{makeEndpointAddress(EndpointDirection::In, DataEndpointID)},
          .bmAttributes{EndpointTransferType::Bulk},
          .wMaxPacketSize{detail::MaxPacketSize}};

        constexpr USB::Descriptors::Endpoint DataOutEndpointDescriptor{
          .bEndpointAddress{makeEndpointAddress(EndpointDirection::Out, DataEndpointID)},
          .bmAttributes{EndpointTransferType::Bulk},
          .wMaxPacketSize{detail::MaxPacketSize}};

        if constexpr(Bidirectional) {
            return USB::Descriptors::detail::generateArray(InterfaceDescriptor,
                                                           DataOutEndpointDescriptor,
                                                           DataInEndpointDescriptor);
        } else {
            return USB::Descriptors::detail::generateArray(InterfaceDescriptor,
                                                           DataInEndpointDescriptor);
        }
    }
}   // namespace Descriptors

/// A vendor bulk interface, in the shape the application needs:
///
///   Framed          send() is one message, and the host reads exactly that message. Without it
///                   the endpoint carries a byte stream: write() what there is room for, flush()
///                   where the host's transfer should end.
///   Bidirectional   an OUT endpoint too, whose data waits in getRecvBuffer() and whose queue
///                   NAKs the host when it is full.
///   Protocol        bInterfaceProtocol, to tell a device's interfaces apart on the host.
///
/// A device with several of these addresses them by their type rather than through the device:
///
///     using Commands = Kvasir::USB::SimpleBulk::Mixin<Clock, Config, Usb, 0, 1>;
///     Commands::send(payload);
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t  FirstInterfaceNumber,
         std::size_t  FirstEndpointNumber,
         bool         Framed        = true,
         bool         Bidirectional = true,
         std::uint8_t Protocol      = 0>
struct Mixin {
private:
    friend Derived;
    friend struct Kvasir::USB::detail::MixinTraits;

    using Self = Mixin<Clock,
                       Config,
                       Derived,
                       FirstInterfaceNumber,
                       FirstEndpointNumber,
                       Framed,
                       Bidirectional,
                       Protocol>;

    static constexpr std::size_t DataEndpointNumber = FirstEndpointNumber;
    static constexpr std::size_t InterfaceCount     = 1;
    static constexpr std::size_t EndpointCount      = 1;

    static constexpr auto InterfaceDescriptor
      = Descriptors::makeInterfaceDescriptorArrays<FirstInterfaceNumber,
                                                   DataEndpointNumber,
                                                   Bidirectional,
                                                   Protocol>();

    using DataEndpointHandler = Kvasir::USB::detail::BulkDataAdapter<Clock,
                                                                     Config,
                                                                     Derived,
                                                                     Self,
                                                                     FirstInterfaceNumber,
                                                                     DataEndpointNumber,
                                                                     Framed,
                                                                     Bidirectional>;

    // Callbacks
    static void SetupEndpointsCallback() { DataEndpointHandler::SetupEndpointsCallback(); }

    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        return detail::handleSetInterface<Derived>(pkt,
                                                   FirstInterfaceNumber,
                                                   DataEndpointHandler::restart)
            || DataEndpointHandler::SetupPacketRequestCallback(pkt);
    }

    static bool EndpointHandlerCallback(std::size_t epNum,
                                        bool        in) {
        return DataEndpointHandler::EndpointHandlerCallback(epNum, in);
    }

    static bool AbortDoneCallback(std::size_t epNum,
                                  bool        in) {
        return DataEndpointHandler::AbortDoneCallback(epNum, in);
    }

    static void ResetCallback() { DataEndpointHandler::ResetCallback(); }

    static void ConfiguredCallback(std::uint8_t configuration) {
        DataEndpointHandler::ConfiguredCallback(configuration);
    }

public:
    /// Connected means configured: a vendor bulk interface has no "open" the way CDC-ACM has DTR.
    static bool isConnected() { return Derived::isConfigured(); }

    static bool isSendReady() { return DataEndpointHandler::isSendReady() && isConnected(); }

    static bool send(std::span<std::byte const> data) { return DataEndpointHandler::send(data); }

    static std::size_t write(std::span<std::byte const> data)
        requires(!Framed)
    {
        return DataEndpointHandler::write(data);
    }

    static std::size_t writeAvailable() { return DataEndpointHandler::writeAvailable(); }

    static void flush()
        requires(!Framed)
    {
        DataEndpointHandler::flush();
    }

    static auto& getRecvBuffer()
        requires Bidirectional
    {
        return DataEndpointHandler::getRecvBuffer();
    }

    /// Where the IN side stands, for a test or a log that wants to report it.
    static auto sendDiagnostics() { return DataEndpointHandler::sendDiagnostics(); }
};

/// A byte stream in both directions.
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber>
using StreamMixin
  = Mixin<Clock, Config, Derived, FirstInterfaceNumber, FirstEndpointNumber, false, true, 4>;

/// Device-to-host only, in messages.
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber>
using InMixin
  = Mixin<Clock, Config, Derived, FirstInterfaceNumber, FirstEndpointNumber, true, false, 2>;
}   // namespace Kvasir::USB::SimpleBulk

namespace Kvasir::USB {
/// A vendor bulk device: SimpleBulk::Mixin as interface 0, plus whatever further mixins. Device
/// class EF/02/01: a composite device.
template<template<typename, typename> class BackendT,
         typename Clock,
         typename Config,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
using Bulk = Device<BackendT,
                    Clock,
                    Config,
                    DeviceClass::Miscellaneous,
                    DeviceClass::CommonClass,
                    0,   // FirstInterfaceNumber
                    1,   // FirstEndpointNumber
                    SimpleBulk::Mixin,
                    Mixins...>;
}   // namespace Kvasir::USB
