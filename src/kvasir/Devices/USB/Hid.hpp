#pragma once

#include "../Log.hpp"
#include "BulkEndpoints.hpp"
#include "Descriptors.hpp"
#include "Device.hpp"
#include "InterruptEndpoint.hpp"
#include "Mixins.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

/// A HID interface: input reports on an interrupt IN endpoint, everything else on endpoint 0.
/// No boot protocol, no interrupt OUT endpoint: output and feature reports arrive as SET_REPORT.
///
/// What the device is comes from a traits struct of the application's:
///
///     struct Knob {
///         static constexpr auto ReportDescriptor = std::to_array<std::uint8_t>({0x06, ...});
///         static constexpr std::size_t  InputReportSize = 8;    // the largest, id byte included
///         static constexpr std::uint8_t PollIntervalMs  = 10;   // optional, 10
///
///         // optional: the host wrote an output or feature report (type 2 or 3). From the USB
///         // interrupt; false refuses it (STALL).
///         static bool setReport(std::uint8_t type, std::uint8_t id, std::span<std::byte const>);
///         // optional: the host asks for a report over endpoint 0; returns its length, 0 refuses.
///         static std::size_t getReport(std::uint8_t type, std::uint8_t id, std::span<std::byte>);
///     };
///
///     template<typename C, typename Cfg, typename D, std::size_t I, std::size_t E>
///     using KnobHid = Kvasir::USB::HID::Mixin<C, Cfg, D, I, E, Knob>;
///     using Usb     = Kvasir::USB::Hid<HW::UsbBackend, Clock, UsbConfig, KnobHid>;
///
///     if(Usb::isReportReady()) { Usb::sendReport(bytes); }
///
/// An input report goes out once, when the host next polls; sendReport() is false while the last
/// one still waits. (HID 1.11: Device Class Definition for Human Interface Devices, 6.2 and 7.)
namespace Kvasir::USB::HID {

// bRequest of the class requests (HID 1.11, 7.2).
enum class Request : std::uint8_t {
    getReport   = 0x01,
    getIdle     = 0x02,
    getProtocol = 0x03,
    setReport   = 0x09,
    setIdle     = 0x0A,
    setProtocol = 0x0B,
};

inline constexpr std::uint8_t HidDescriptorType    = 0x21;
inline constexpr std::uint8_t ReportDescriptorType = 0x22;

template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber,
         typename Traits>
struct Mixin {
private:
    friend Derived;
    friend struct Kvasir::USB::detail::MixinTraits;

    static constexpr std::size_t InterfaceCount = 1;
    static constexpr std::size_t EndpointCount  = 1;

    static constexpr std::size_t ReportDescriptorSize = Traits::ReportDescriptor.size();
    static constexpr std::size_t InputReportSize      = Traits::InputReportSize;
    static_assert(InputReportSize != 0 && InputReportSize <= MaxPacketSize,
                  "an input report has to fit one 64-byte packet");

    static constexpr std::uint8_t PollIntervalMs = [] {
        if constexpr(requires { Traits::PollIntervalMs; }) {
            return static_cast<std::uint8_t>(Traits::PollIntervalMs);
        } else {
            return std::uint8_t{10};
        }
    }();

    // Interface, the HID descriptor (HID 1.11, 6.2.1: bcdHID 1.11, no country, one class
    // descriptor - the report descriptor and its length), the endpoint.
    static constexpr auto InterfaceDescriptor = USB::Descriptors::detail::generateArray(
      USB::Descriptors::Interface{.bInterfaceNumber   = FirstInterfaceNumber,
                                  .bAlternateSetting  = 0,
                                  .bNumEndpoints      = 1,
                                  .bInterfaceClass    = 3,
                                  .bInterfaceSubClass = 0,
                                  .bInterfaceProtocol = 0},
      std::array<std::byte, 9>{std::byte{9},
                               std::byte{HidDescriptorType},
                               std::byte{0x11},
                               std::byte{0x01},
                               std::byte{0},
                               std::byte{1},
                               std::byte{ReportDescriptorType},
                               static_cast<std::byte>(ReportDescriptorSize & 0xff),
                               static_cast<std::byte>(ReportDescriptorSize >> 8)},
      USB::Descriptors::Endpoint{
        .bEndpointAddress{makeEndpointAddress(EndpointDirection::In,
                                              static_cast<std::uint8_t>(FirstEndpointNumber))},
        .bmAttributes{EndpointTransferType::Interrupt},
        .wMaxPacketSize{static_cast<std::uint16_t>(InputReportSize)},
        .bInterval{PollIntervalMs}});

    static constexpr auto ReportDescriptorBytes = [] {
        std::array<std::byte, ReportDescriptorSize> bytes{};
        for(std::size_t i = 0; i != bytes.size(); ++i) {
            bytes[i] = static_cast<std::byte>(Traits::ReportDescriptor[i]);
        }
        return bytes;
    }();

    using In = detail::InterruptInEndpoint<Derived, FirstEndpointNumber, InputReportSize>;

    // State
    static inline std::uint8_t                         idleRate{};
    static inline std::array<std::byte, MaxPacketSize> controlReport{};
    static inline std::uint8_t                         pendingType{};
    static inline std::uint8_t                         pendingId{};
    static inline std::uint16_t                        pendingLength{};   // 0: no SET_REPORT open

    // Callbacks
    static void SetupEndpointsCallback() { In::setup(); }

    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        using Direction = SetupPacket::Direction;
        bool const in   = pkt.direction() == Direction::deviceToHost;

        // A new request ends a SET_REPORT whose data stage never came.
        pendingLength = 0;

        if(pkt.recipient()
             == SetupPacket::Recipient::interface && pkt.wIndex == FirstInterfaceNumber)
        {
            if(pkt.type() == SetupPacket::Type::standard) {
                // GET_DESCRIPTOR(report) goes to the interface (HID 1.11, 7.1.1).
                if(in && pkt.bRequest == SetupPacket::Request::getDescriptor
                   && (pkt.wValue >> 8) == ReportDescriptorType)
                {
                    return Derived::ep0INDataPhase(ReportDescriptorBytes, pkt.wLength);
                }
                return false;
            }
            if(pkt.type() == SetupPacket::Type::classT) { return classRequest(pkt, in); }
            return false;
        }
        return detail::handleEndpointRequest<Derived, In>(pkt);
    }

    static bool classRequest(SetupPacket const& pkt,
                             bool               in) {
        auto const type = static_cast<std::uint8_t>(pkt.wValue >> 8);
        auto const id   = static_cast<std::uint8_t>(pkt.wValue & 0xff);
        switch(static_cast<Request>(std::to_underlying(pkt.bRequest))) {
        case Request::setIdle:
            if(in) { return false; }
            idleRate = type;   // the upper byte is the rate here; kept, not acted on
            Derived::acknowledgeSetupRequest();
            return true;
        case Request::getIdle:
            if(!in) { return false; }
            controlReport[0] = std::byte{idleRate};
            return Derived::ep0INDataPhase(std::span{controlReport}.first(1), pkt.wLength);
        case Request::getReport:
            if constexpr(requires(std::span<std::byte> out) { Traits::getReport(type, id, out); }) {
                if(!in) { return false; }
                std::size_t const n = Traits::getReport(type, id, std::span{controlReport});
                if(n == 0 || n > controlReport.size()) { return false; }
                return Derived::ep0INDataPhase(std::span{controlReport}.first(n), pkt.wLength);
            } else {
                return false;
            }
        case Request::setReport:
            if constexpr(requires(std::span<std::byte const> data) {
                             Traits::setReport(type, id, data);
                         })
            {
                if(in || pkt.wLength == 0 || pkt.wLength > MaxPacketSize) { return false; }
                pendingType   = type;
                pendingId     = id;
                pendingLength = pkt.wLength;
                Derived::ep0OUTDataPhase(pkt.wLength);
                return true;
            } else {
                return false;
            }
        default: return false;   // no boot protocol: GET/SET_PROTOCOL are refused
        }
    }

    static bool EndpointHandlerCallback(std::size_t epNum,
                                        bool        in) {
        // The data stage of a SET_REPORT, on endpoint 0.
        if(epNum == 0 && !in && pendingLength != 0) {
            std::size_t const length = std::exchange(pendingLength, std::uint16_t{0});
            bool              taken  = false;
            if constexpr(requires(std::span<std::byte const> data) {
                             Traits::setReport(pendingType, pendingId, data);
                         })
            {
                auto const data = std::span{controlReport}.first(length);
                taken           = Derived::ep0OUTGetData(data)
                               && Traits::setReport(pendingType, pendingId, std::span<std::byte const>{data});
            }
            if(taken) {
                Derived::acknowledgeSetupRequest();
            } else {
                Derived::stallControlRequest();
            }
            return true;
        }
        if(epNum == FirstEndpointNumber && in) {
            In::bufferDone();
            return true;
        }
        return false;
    }

    static bool AbortDoneCallback(std::size_t epNum,
                                  bool        in) {
        if(epNum != FirstEndpointNumber || !in) { return false; }
        In::abortDone();
        return true;
    }

    static void ResetCallback() {
        pendingLength = 0;
        idleRate      = 0;
        In::busReset();
    }

    static void ConfiguredCallback(std::uint8_t) { In::configured(); }

public:
    /// Whether sendReport() would take a report now.
    static bool isReportReady() { return In::ready(); }

    /// An input report (with its id byte first, if the descriptor uses ids), for the host's next
    /// poll. False while the last one still waits, or while the device is not configured.
    static bool sendReport(std::span<std::byte const> report) { return In::send(report); }
};
}   // namespace Kvasir::USB::HID

namespace Kvasir::USB {
/// A device whose first interface is a HID one. Device class 0: the class is the interface's.
///
///     template<typename C, typename Cfg, typename D, std::size_t I, std::size_t E>
///     using KnobHid = Kvasir::USB::HID::Mixin<C, Cfg, D, I, E, Knob>;
///     using Usb     = Kvasir::USB::Hid<HW::UsbBackend, Clock, UsbConfig, KnobHid>;
template<template<typename, typename> class BackendT,
         typename Clock,
         typename Config,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
using Hid = Device<BackendT,
                   Clock,
                   Config,
                   DeviceClass::UseInterfaceClass,
                   DeviceClass::UseInterfaceClass,
                   0,   // FirstInterfaceNumber
                   1,   // FirstEndpointNumber
                   Mixins...>;
}   // namespace Kvasir::USB
