#pragma once

#include "../Log.hpp"
#include "Backend.hpp"
#include "BulkEndpoints.hpp"
#include "Descriptors.hpp"
#include "Device.hpp"
#include "InterruptEndpoint.hpp"
#include "Mixins.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string_view>
#include <utility>

namespace Kvasir::USB::CDC {

// Class-specific requests (CDC PSTN subclass).
enum class Request : std::uint8_t {
    setLineCoding       = 0x20,
    getLineCoding       = 0x21,
    setControlLineState = 0x22,
};

struct [[gnu::packed]] LineCoding {
    std::uint32_t dwDTERate{9600};
    std::uint8_t  bCharFormat{};
    std::uint8_t  bParityType{};
    std::uint8_t  bDataBits{8};
};

namespace Descriptors {
    struct [[gnu::packed]] Header
      : Detail::InterfaceDescriptorBase<Header, DescriptorSubType::cdcHeader> {
        std::uint16_t bcdCDC{0x0120};
    };

    struct [[gnu::packed]] Union
      : Detail::InterfaceDescriptorBase<Union, DescriptorSubType::cdcUnion> {
        std::uint8_t bMasterInterface0;
        std::uint8_t bSlaveInterface0;
    };

    struct [[gnu::packed]] CallManagement
      : Detail::InterfaceDescriptorBase<CallManagement, DescriptorSubType::cdcCallManagement> {
        std::uint8_t bmCapabilities;
        std::uint8_t bDataInterface;
    };
}   // namespace Descriptors

namespace ACM {
    namespace Descriptors {
        struct [[gnu::packed]] Descriptor
          : Detail::InterfaceDescriptorBase<Descriptor, DescriptorSubType::cdcAcmDescriptor> {
            // SET_LINE_CODING, GET_LINE_CODING and SET_CONTROL_LINE_STATE are supported
            std::uint8_t bmCapabilities{0x02};
        };

        template<std::uint8_t ManagementInterfaceId,
                 std::uint8_t DataInterfaceId,
                 std::uint8_t ManagementEndpointId,
                 std::uint8_t DataEndpointID>
        consteval auto makeInterfaceDescriptorArrays() {
            constexpr USB::Descriptors::InterfaceAssociation cdcAcmInterfaceAssociation{
              .bFirstInterface{ManagementInterfaceId},
              .bInterfaceCount{2},
              .bFunctionClass{2},
              .bFunctionSubClass{2},
              .bFunctionProtocol{0}};

            constexpr USB::Descriptors::Interface cdcManagementInterfaceDescriptor{
              .bInterfaceNumber{ManagementInterfaceId},
              .bAlternateSetting{0},
              .bNumEndpoints{1},
              .bInterfaceClass{std::to_underlying(DeviceClass::communication)},
              .bInterfaceSubClass{2},
              .bInterfaceProtocol{0}};

            constexpr USB::Descriptors::Interface acmInterfaceDescriptor{
              .bInterfaceNumber{DataInterfaceId},
              .bAlternateSetting{0},
              .bNumEndpoints{2},
              .bInterfaceClass{std::to_underlying(DeviceClass::cdcData)},
              .bInterfaceSubClass{0},
              .bInterfaceProtocol{0}};

            constexpr USB::CDC::Descriptors::Header cdcHeaderDescriptor{};

            constexpr USB::CDC::ACM::Descriptors::Descriptor cdcAcmDescriptor{};

            constexpr USB::CDC::Descriptors::Union cdcUnionDescriptor{
              .bMasterInterface0{ManagementInterfaceId},
              .bSlaveInterface0{DataInterfaceId}};

            constexpr USB::CDC::Descriptors::CallManagement cdcCallManagementDescriptor{
              .bmCapabilities{0x0},
              .bDataInterface{DataInterfaceId}};

            // CDC Management endpoint packet size
            // A SERIAL_STATE notification is 10 bytes (8 of header, 2 of state): one packet.
            static constexpr std::uint16_t managementEndpointPacketSize = 16;
            // Interrupt endpoint polling interval (in ms, max 255 to minimize polling)
            static constexpr std::uint8_t interruptEndpointInterval = 255;

            constexpr auto cdcManagementEndpointDescriptor
              = USB::Descriptors::endpoint(EndpointDirection::in,
                                           ManagementEndpointId,
                                           EndpointTransferType::interrupt,
                                           managementEndpointPacketSize,
                                           interruptEndpointInterval);
            constexpr auto acmDataInEndpointDescriptor
              = USB::Descriptors::endpoint(EndpointDirection::in,
                                           DataEndpointID,
                                           EndpointTransferType::bulk);
            constexpr auto acmDataOutEndpointDescriptor
              = USB::Descriptors::endpoint(EndpointDirection::out,
                                           DataEndpointID,
                                           EndpointTransferType::bulk);

            constexpr auto cdcInterface
              = USB::Descriptors::generateArray(cdcManagementInterfaceDescriptor,
                                                cdcHeaderDescriptor,
                                                cdcCallManagementDescriptor,
                                                cdcAcmDescriptor,
                                                cdcUnionDescriptor,
                                                cdcManagementEndpointDescriptor);
            constexpr auto acmInterface
              = USB::Descriptors::generateArray(acmInterfaceDescriptor,
                                                acmDataOutEndpointDescriptor,
                                                acmDataInEndpointDescriptor);

            constexpr auto interface = USB::Descriptors::generateArray(cdcAcmInterfaceAssociation,
                                                                       cdcInterface,
                                                                       acmInterface);

            return interface;
        }
    }   // namespace Descriptors

    // CDC-ACM Mixin - provides CDC/ACM functionality
    template<typename Clock,
             typename Config,
             typename Derived,
             std::size_t FirstInterfaceNumber,
             std::size_t FirstEndpointNumber>
    struct Mixin {
    private:
        friend Derived;
        friend struct Kvasir::USB::MixinTraits;   // Grants access to helper functions

        using Self = Mixin<Clock, Config, Derived, FirstInterfaceNumber, FirstEndpointNumber>;

        // CDC-ACM uses 2 interfaces (management + data)
        static constexpr std::size_t interfaceCount = 2;
        static constexpr std::size_t endpointCount  = 2;

        static constexpr std::size_t serialStateSize          = 10;
        static constexpr std::size_t managementInterface      = FirstInterfaceNumber;
        static constexpr std::size_t dataInterface            = FirstInterfaceNumber + 1;
        static constexpr std::size_t managementEndpointNumber = FirstEndpointNumber;
        static constexpr std::size_t dataEndpointNumber       = FirstEndpointNumber + 1;

        // Interface descriptors for both CDC management and ACM data interfaces
        static constexpr auto interfaceDescriptor
          = USB::CDC::ACM::Descriptors::makeInterfaceDescriptorArrays<managementInterface,
                                                                      dataInterface,
                                                                      managementEndpointNumber,
                                                                      dataEndpointNumber>();

        using DataEndpointHandler = Kvasir::USB::Detail::BulkDataAdapter<Clock,
                                                                         Config,
                                                                         Derived,
                                                                         Self,
                                                                         dataInterface,
                                                                         dataEndpointNumber,
                                                                         true,
                                                                         true>;

        // The notification endpoint: SERIAL_STATE goes out on it (sendSerialState()).
        using Notification
          = Detail::InterruptInEndpoint<Derived, managementEndpointNumber, serialStateSize>;

        // Callbacks
        static void setupEndpointsCallback() {
            Notification::setup();
            DataEndpointHandler::setupEndpointsCallback();
        }

        static bool setupPacketRequestCallback(SetupPacket const& pkt) {
            using Direction = SetupPacket::Direction;
            using Recipient = SetupPacket::Recipient;
            using Type      = SetupPacket::Type;

            // A new request ends a SET_LINE_CODING whose data stage never came.
            lineCodingExpected = false;

            if(pkt.type() == Type::classT && pkt.recipient() == Recipient::interface) {
                if(pkt.wIndex == managementInterface) {
                    switch(static_cast<Request>(std::to_underlying(pkt.bRequest))) {
                    case Request::setControlLineState:
                        {
                            if(pkt.direction() == Direction::hostToDevice) {
                                using namespace std::string_view_literals;
                                Derived::acknowledgeSetupRequest();
                                connected = (pkt.wValue & 0x01) != 0;
                                UC_LOG_D("CDC-ACM: Control line state - DTR={}, RTS={}",
                                         (pkt.wValue & 0x01) ? "on"sv : "off"sv,
                                         (pkt.wValue & 0x02) ? "on"sv : "off"sv);
                                return true;
                            }
                        }
                        break;
                    case Request::getLineCoding:
                        {
                            if(pkt.direction() == Direction::deviceToHost) {
                                UC_LOG_D("CDC-ACM: Get line coding request");
                                return Derived::ep0INDataPhase(
                                  std::as_bytes(std::span{std::addressof(lineCoding), 1}),
                                  pkt.wLength);
                            }
                        }
                        break;
                    case Request::setLineCoding:
                        {
                            if(pkt.direction() == Direction::hostToDevice
                               && pkt.wLength == sizeof(USB::CDC::LineCoding))
                            {
                                lineCodingExpected = true;
                                Derived::ep0OUTDataPhase(sizeof(USB::CDC::LineCoding));
                                UC_LOG_D("CDC-ACM: Set line coding request");
                                return true;
                            }
                        }
                        break;
                    default: break;
                    }
                }
                return false;
            }
            return handleEndpointRequest<Derived, Notification>(pkt)
                || Detail::handleSetInterface<Derived, &Notification::restart>(pkt,
                                                                               managementInterface)
                || Detail::handleSetInterface<Derived, &DataEndpointHandler::restart>(pkt,
                                                                                      dataInterface)
                || DataEndpointHandler::setupPacketRequestCallback(pkt);
        }

        static bool endpointHandlerCallback(std::size_t epNum,
                                            bool        in) {
            // Handle line coding data stage on EP0
            if(epNum == 0 && !in && lineCodingExpected) {
                std::array<std::byte, sizeof(lineCoding)> tempBuffer{};
                lineCodingExpected = false;
                if(!Derived::ep0OUTGetData(tempBuffer)) {
                    Derived::stallControlRequest();
                    return true;
                }
                std::memcpy(std::addressof(lineCoding), tempBuffer.data(), tempBuffer.size());
                Derived::acknowledgeSetupRequest();
                UC_LOG_D(
                  "CDC-ACM: Line coding set - {} baud, {} data bits, parity={}, stop "
                  "bits={}",
                  lineCoding.dwDTERate,
                  lineCoding.bDataBits,
                  lineCoding.bParityType,
                  lineCoding.bCharFormat);
                return true;
            }
            if(epNum == managementEndpointNumber && in) {
                Notification::bufferDone();
                return true;
            }
            return DataEndpointHandler::endpointHandlerCallback(epNum, in);
        }

        static bool abortDoneCallback(std::size_t epNum,
                                      bool        in) {
            if(epNum == managementEndpointNumber && in) {
                Notification::abortDone();
                return true;
            }
            return DataEndpointHandler::abortDoneCallback(epNum, in);
        }

        static void resetCallback() {
            lineCodingExpected = false;
            connected          = false;
            Notification::busReset();
            DataEndpointHandler::resetCallback();
        }

        static void configuredCallback(std::uint8_t configuration) {
            if(configuration == 0) { connected = false; }
            Notification::configured();
            DataEndpointHandler::configuredCallback(configuration);
        }

        // State
        static inline std::atomic<bool>    connected          = false;
        static inline bool                 lineCodingExpected = false;
        static inline USB::CDC::LineCoding lineCoding{};

    public:
        // Public API
        static bool isConnected() { return connected; }

        // The data endpoint
        static bool isSendReady() { return DataEndpointHandler::isSendReady() && isConnected(); }

        static auto& getRecvBuffer() { return DataEndpointHandler::getRecvBuffer(); }

        static bool send(std::span<std::byte const> data) {
            return DataEndpointHandler::send(data);
        }

        static std::size_t writeAvailable() { return DataEndpointHandler::writeAvailable(); }

        /// The bits of a SERIAL_STATE notification (PSTN 1.2, 6.5.4, table 31): what a UART's
        /// modem and error lines would say.
        struct SerialState {
            static constexpr std::uint16_t dcd     = 1U << 0U;   // bRxCarrier
            static constexpr std::uint16_t dsr     = 1U << 1U;   // bTxCarrier
            static constexpr std::uint16_t break_  = 1U << 2U;
            static constexpr std::uint16_t ring    = 1U << 3U;
            static constexpr std::uint16_t framing = 1U << 4U;
            static constexpr std::uint16_t parity  = 1U << 5U;
            static constexpr std::uint16_t overrun = 1U << 6U;
        };

        /// Tells the host the state of the lines, on the notification endpoint. False while the
        /// last notification has not gone out yet, or while the device is not configured.
        static bool sendSerialState(std::uint16_t state) {
            std::array<std::byte, serialStateSize> const notification{
              std::byte{0xA1},   // bmRequestType: class, interface, device to host
              std::byte{0x20},   // bNotification: SERIAL_STATE
              std::byte{0},
              std::byte{0},   // wValue
              static_cast<std::byte>(managementInterface),
              std::byte{0},   // wIndex: the interface
              std::byte{2},
              std::byte{0},   // wLength
              static_cast<std::byte>(state & 0xffU),
              static_cast<std::byte>(state >> 8U)};
            return Notification::send(notification);
        }
    };

}   // namespace ACM
}   // namespace Kvasir::USB::CDC

namespace Kvasir::USB {
/// A CDC-ACM device (a serial port on the host), plus whatever further mixins. Device class
/// EF/02/01, because the two CDC interfaces are tied together by an interface association.
///
///     using Usb = Kvasir::USB::CdcAcm<HW::UsbBackend, Clock, UsbConfig>;
///     Usb::isConnected();  Usb::isSendReady();  Usb::send(bytes);  Usb::getRecvBuffer().popInto(b);
template<template<typename, typename> class BackendT,
         typename Clock,
         typename Config,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
using CdcAcm = Device<BackendT,
                      Clock,
                      Config,
                      DeviceClass::miscellaneous,
                      DeviceClass::communication,
                      0,   // FirstInterfaceNumber
                      1,   // FirstEndpointNumber
                      CDC::ACM::Mixin,
                      Mixins...>;
}   // namespace Kvasir::USB
