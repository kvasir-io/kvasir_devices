#pragma once

#include "../Log.hpp"
#include "Backend.hpp"
#include "Config.hpp"
#include "ControlTransfer.hpp"
#include "Descriptors.hpp"
#include "Mixins.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <kvasir/Util/RateLimiter.hpp>
#include <kvasir/Util/using_literals.hpp>
#include <span>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

/// A USB full-speed device: endpoint 0 (the standard requests, the control transfer's stages, the
/// descriptors) and the mixins that make up its interfaces. The controller is the backend's
/// (Backend.hpp); this file touches no register.
///
///     struct UsbConfig {
///         static constexpr auto ManufacturerString = "...";   // or a callable, for a runtime one
///         static constexpr auto ProductString      = "...";
///         static constexpr auto SerialNumberString = [] { return Kvasir::serialNumberString(); };
///         static constexpr auto ProductVersionBCD  = 0x0100;
///         static constexpr auto VendorID           = 0x....;
///         static constexpr auto ProductID          = 0x....;
///     };
///     using Usb = Kvasir::USB::CdcAcm<HW::UsbBackend, Clock, UsbConfig>;   // CdcAcm.hpp
///
/// and Usb in the Startup list. Config.hpp has the optional config members.
namespace Kvasir::USB {
namespace detail {
    // The backend's Startup members, passed on where it has them.
    template<typename B>
    struct ForwardProvides {};

    template<typename B>
        requires requires { typename B::Provides; }
    struct ForwardProvides<B> {
        using Provides = typename B::Provides;
    };

    template<typename B>
    struct ForwardClaims {};

    template<typename B>
        requires requires { typename B::Claims; }
    struct ForwardClaims<B> {
        using Claims = typename B::Claims;
    };

    template<typename B>
    struct ForwardPowerClockEnable {};

    template<typename B>
        requires requires { B::powerClockEnable; }
    struct ForwardPowerClockEnable<B> {
        static constexpr auto powerClockEnable = B::powerClockEnable;
    };

    template<typename B>
    struct ForwardPinConfig {};

    template<typename B>
        requires requires { B::initStepPinConfig; }
    struct ForwardPinConfig<B> {
        static constexpr auto initStepPinConfig = B::initStepPinConfig;
    };

    template<typename B>
    struct ForwardPeripheryConfig {};

    template<typename B>
        requires requires { B::initStepPeripheryConfig; }
    struct ForwardPeripheryConfig<B> {
        static constexpr auto initStepPeripheryConfig = B::initStepPeripheryConfig;
    };

    template<typename B>
    struct ForwardInterruptConfig {};

    template<typename B>
        requires requires { B::initStepInterruptConfig; }
    struct ForwardInterruptConfig<B> {
        static constexpr auto initStepInterruptConfig = B::initStepInterruptConfig;
    };

    template<typename B>
    struct BackendStartup
      : ForwardProvides<B>
      , ForwardClaims<B>
      , ForwardPowerClockEnable<B>
      , ForwardPinConfig<B>
      , ForwardPeripheryConfig<B>
      , ForwardInterruptConfig<B> {};
}   // namespace detail

template<template<typename, typename> class BackendT,
         typename Clock,
         typename ConfigT,
         DeviceClass Class,
         DeviceClass SubClass,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
struct Device
  : detail::MixinTraits::MixinBases<Clock,
                                    ConfigT,
                                    Device<BackendT,
                                           Clock,
                                           ConfigT,
                                           Class,
                                           SubClass,
                                           FirstInterfaceNumber,
                                           FirstEndpointNumber,
                                           Mixins...>,
                                    FirstInterfaceNumber,
                                    FirstEndpointNumber,
                                    std::make_index_sequence<sizeof...(Mixins)>,
                                    Mixins...>
  , detail::BackendStartup<BackendT<Clock, ConfigT>> {
    /// The chip's half. Its endpoints carry whatever more the chip can do than Backend.hpp asks.
    using Backend = BackendT<Clock, ConfigT>;

private:
    using Self = Device<BackendT,
                        Clock,
                        ConfigT,
                        Class,
                        SubClass,
                        FirstInterfaceNumber,
                        FirstEndpointNumber,
                        Mixins...>;

    using MixinsBase = detail::MixinTraits::MixinBases<Clock,
                                                       ConfigT,
                                                       Self,
                                                       FirstInterfaceNumber,
                                                       FirstEndpointNumber,
                                                       std::make_index_sequence<sizeof...(Mixins)>,
                                                       Mixins...>;

    using SetupPacket = Kvasir::USB::SetupPacket;
    using Traits      = ConfigTraits<ConfigT>;

    static_assert(detail::Controller<Backend>,
                  "the backend lacks part of what kvasir/Devices/USB/Backend.hpp lists for the "
                  "controller");
    static_assert(Backend::MaxPacketSize == MaxPacketSize,
                  "the backend's MaxPacketSize differs from the 64 bytes the descriptors promise");
    static_assert(1
                      + detail::MixinTraits::countMixinEndpoints<Clock,
                                                                 ConfigT,
                                                                 Device,
                                                                 Mixins...>()
                    <= Backend::EndpointCount,
                  "the mixins use more endpoint numbers than the controller has");

    using EP0_IN =
      typename Backend::template Endpoint<0, EndpointDirection::In, EndpointTransferType::Control>;
    using EP0_OUT =
      typename Backend::template Endpoint<0, EndpointDirection::Out, EndpointTransferType::Control>;
    static_assert(detail::InEndpoint<EP0_IN> && detail::OutEndpoint<EP0_OUT>,
                  "the backend's endpoints lack part of what kvasir/Devices/USB/Backend.hpp lists");

    static constexpr std::tuple DescriptorStrings{
      []() {
          if constexpr(std::is_invocable_v<decltype(ConfigT::ManufacturerString)>) {
              return Kvasir::USB::RuntimeDescriptorString{ConfigT::ManufacturerString};
          } else {
              return Kvasir::USB::DescriptorString{SC_LIFT(ConfigT::ManufacturerString)};
          }
      }(),
      []() {
          if constexpr(std::is_invocable_v<decltype(ConfigT::ProductString)>) {
              return Kvasir::USB::RuntimeDescriptorString{ConfigT::ProductString};
          } else {
              return Kvasir::USB::DescriptorString{SC_LIFT(ConfigT::ProductString)};
          }
      }(),
      []() {
          if constexpr(std::is_invocable_v<decltype(ConfigT::SerialNumberString)>) {
              return Kvasir::USB::RuntimeDescriptorString{ConfigT::SerialNumberString};
          } else {
              return Kvasir::USB::DescriptorString{SC_LIFT(ConfigT::SerialNumberString)};
          }
      }()};

    static constexpr auto DeviceDescriptor{USB::Descriptors::makeDeviceDescriptorArray<
      ConfigT::ProductVersionBCD,
      ConfigT::VendorID,
      ConfigT::ProductID,
      1,
      2,
      3,
      Class,
      SubClass,
      detail::MixinTraits::maxBcdUSB<Clock, ConfigT, Self, Mixins...>()>()};

    static constexpr auto ConfigDescriptor = []() {
        constexpr std::size_t MixinInterfaceCount
          = detail::MixinTraits::countMixinInterfaces<Clock, ConfigT, Self, Mixins...>();

        constexpr auto mixinDescriptors
          = detail::MixinTraits::assembleMixinDescriptors<Clock,
                                                          ConfigT,
                                                          Self,
                                                          FirstInterfaceNumber,
                                                          FirstEndpointNumber,
                                                          Mixins...>();

        return Descriptors::makeConfigDescriptorArray(
          Traits::BusPower,
          Traits::BusPowered,
          static_cast<std::uint8_t>(MixinInterfaceCount),
          mixinDescriptors);
    }();

    // State
    static inline std::atomic<std::uint8_t>  configuration{};
    static inline std::uint8_t               deviceBusAddr{};
    static inline bool                       pendingAddressSet{false};
    static inline std::span<std::byte const> remainingControlData{};
    static inline bool                       endOfTransferPending{false};
    static inline detail::EP0ControlState    ep0_ctrl{};
    static inline std::uint8_t               isrMaskDepth{};
    static inline std::atomic<bool>          suspended{false};

    enum class Fault : std::uint8_t {
        unhandledBufferDone = 1,
        unhandledAbortDone,
        unhandledSetup,
        invalidOutData,
        outReadNotInDataPhase,
    };
    // Fault logging goes through this: a misbehaving host repeats these per packet, from inside
    // the ISR.
    static inline Kvasir::RateLimiter<Clock, Kvasir::RateLimiterConfig{.burst = 8}> faultLog_{};

    // What the backend's dispatchEvents reports to.
    struct Sink {
        static void busReset() { handleBusReset(); }

        static void setup(SetupPacket const& pkt) { handleSetupPacket(pkt); }

        static void transferComplete(std::size_t ep_num,
                                     bool        in) {
            handleBufferDone(ep_num, in);
        }

        static void cancelComplete(std::size_t ep_num,
                                   bool        in) {
            handleAbortDone(ep_num, in);
        }

        // A backend that can tell reports both; the bus going quiet for 3 ms, and waking up.
        static void suspend() { handleSuspend(true); }

        static void resume() { handleSuspend(false); }

        static void startOfFrame([[maybe_unused]] std::uint16_t frame) {
            if constexpr(Traits::UseSof) { ConfigT::StartOfFrameCallback(frame); }
        }
    };

    static void onIsr() { Backend::template dispatchEvents<Sink>(); }

    static void endpointConfig() {
        EP0_IN::setupEndpoint();
        EP0_OUT::setupEndpoint();
        MixinsBase::callSetupEndpoints();
    }

    static void handleSuspend(bool isSuspended) {
        if(suspended.exchange(isSuspended) == isSuspended) { return; }
        MixinsBase::callSuspended(isSuspended);
        if constexpr(requires { ConfigT::SuspendCallback(isSuspended); }) {
            ConfigT::SuspendCallback(isSuspended);
        }
    }

    static void handleBusReset() {
        ep0_ctrl.reset();
        suspended = false;
        // Some controllers forget their endpoints on a bus reset (the SAM one all but endpoint
        // 0, and every endpoint's interrupts); setting them up again costs the others nothing.
        endpointConfig();
        deviceBusAddr        = 0;
        pendingAddressSet    = false;
        configuration        = 0;
        remainingControlData = {};
        endOfTransferPending = false;
        EP0_IN::reset();
        EP0_OUT::reset();

        Backend::setAddress(0);

        MixinsBase::callReset();
        UC_LOG_I("USB: Bus reset detected");
    }

    static bool EndpointHandler(std::size_t ep_num,
                                bool        in) {
        if(ep_num == 0 && in) {
            if(pendingAddressSet) {
                UC_LOG_I("USB: Setting device address to {}", deviceBusAddr);
                // The status stage went out from the old address; from here on the new one.
                Backend::setAddress(deviceBusAddr);
                pendingAddressSet = false;
                ep0_ctrl.transition(ControlStage::Idle);
                return true;
            }
            if(!remainingControlData.empty() || endOfTransferPending) {
                auto const chunk = remainingControlData.first(
                  std::min(remainingControlData.size(), MaxPacketSize));
                remainingControlData = remainingControlData.subspan(chunk.size());
                if(chunk.empty()) { endOfTransferPending = false; }
                if(remainingControlData.empty() && !endOfTransferPending) {
                    ep0IN<true>(chunk);
                } else {
                    ep0IN<false>(chunk);
                }
                return true;
            }
            if(ep0_ctrl.stage() == ControlStage::Data) {
                ep0_ctrl.transition(ControlStage::Status);
                ep0OUT<true>(0);
                return true;
            }
            if(ep0_ctrl.stage() == ControlStage::Status) {
                ep0_ctrl.transition(ControlStage::Idle);
                return true;
            }
            return false;
        }
        if(ep_num == 0 && !in) {
            if(ep0_ctrl.stage() == ControlStage::Status) {
                ep0_ctrl.transition(ControlStage::Idle);
                return true;
            }
        }
        return false;
    }

    static void handleBufferDone(std::size_t ep_num,
                                 bool        in) {
        using namespace std::string_view_literals;
        if(EndpointHandler(ep_num, in)) { return; }
        if(!MixinsBase::callEndpointHandler(ep_num, in)) {
            KVASIR_LOG_LIMITED(
              faultLog_.allow(Kvasir::rateLimitKey(Fault::unhandledBufferDone, ep_num, in)),
              UC_LOG_W,
              "USB: Unhandled endpoint buffer done (EP{} {})",
              ep_num,
              in ? "IN"sv : "OUT"sv);
        }
    }

    static void handleAbortDone(std::size_t ep_num,
                                bool        in) {
        using namespace std::string_view_literals;
        if(!MixinsBase::callAbortDone(ep_num, in)) {
            KVASIR_LOG_LIMITED(
              faultLog_.allow(Kvasir::rateLimitKey(Fault::unhandledAbortDone, ep_num, in)),
              UC_LOG_W,
              "USB: Unhandled endpoint abort done (EP{} {})",
              ep_num,
              in ? "IN"sv : "OUT"sv);
        }
    }

    static bool handleDeviceDescriptor(SetupPacket const& pkt) {
        return ep0INDataPhase(Self::DeviceDescriptor, pkt.wLength);
    }

    static bool handleConfigDescriptor(SetupPacket const& pkt) {
        // The only configuration has index 0.
        if((pkt.wValue & 0xff) != 0) { return false; }
        return ep0INDataPhase(Self::ConfigDescriptor, pkt.wLength);
    }

    // A string given as a literal is a finished descriptor in flash; only one that is made at
    // run time (a serial number read from the chip) needs a buffer to be put together in.
    template<std::size_t I>
    static constexpr bool IsRuntimeString = requires { std::get<I>(DescriptorStrings).f; };

    static constexpr std::size_t StringCount = std::tuple_size_v<decltype(DescriptorStrings)>;

    static constexpr bool AnyRuntimeString = []<std::size_t... Ns>(std::index_sequence<Ns...>) {
        return (IsRuntimeString<Ns> || ...);
    }(std::make_index_sequence<StringCount>{});

    static inline std::array<std::byte, AnyRuntimeString ? 256 : 0> stringDescriptorBuffer{};

    template<std::size_t I>
    static constexpr auto FlashString = [] {
        constexpr auto const&                         text = std::get<I>(DescriptorStrings);
        std::array<std::byte, 2 + text.buffer.size()> descriptor{};
        descriptor[0] = static_cast<std::byte>(descriptor.size());
        descriptor[1] = static_cast<std::byte>(DescriptorType::string);
        std::copy(text.begin(), text.end(), descriptor.begin() + 2);
        return descriptor;
    }();

    template<std::size_t I>
    static bool sendString(SetupPacket const& pkt) {
        if constexpr(IsRuntimeString<I>) {
            auto&      buffer = stringDescriptorBuffer;
            auto const start  = buffer.begin() + 2;
            auto const end
              = insertStringDescriptor(std::get<I>(DescriptorStrings).get(), start, buffer.end());
            if(end == start) { return false; }
            auto const len = static_cast<std::size_t>(std::distance(buffer.begin(), end));
            buffer[0]      = std::byte(len);
            buffer[1]      = std::byte(DescriptorType::string);
            return ep0INDataPhase(std::span{buffer.data(), len}, pkt.wLength);
        } else {
            return ep0INDataPhase(FlashString<I>, pkt.wLength);
        }
    }

    static bool handleStringDescriptor(SetupPacket const& pkt) {
        // String 0: the languages, of which there is one (English US).
        static constexpr std::array<std::byte, 4> Languages{std::byte{4},
                                                            std::byte(DescriptorType::string),
                                                            std::byte{0x09},
                                                            std::byte{0x04}};

        auto const index = static_cast<std::size_t>(pkt.wValue & 0xff);

        // Windows probes for a Microsoft OS 1.0 descriptor on every enumeration.
        static constexpr std::size_t MsOs10StringIndex = 0xEE;
        if(index == MsOs10StringIndex) { return stallQuietly(); }

        if(index == 0) { return ep0INDataPhase(Languages, pkt.wLength); }

        bool handled = false;
        [&]<std::size_t... Ns>(std::index_sequence<Ns...>) {
            static_cast<void>(((Ns + 1 == index && (handled = sendString<Ns>(pkt), true)) || ...));
        }(std::make_index_sequence<StringCount>{});
        return handled;
    }

    // Descriptors hosts routinely ask for that a full-speed device does not have; the
    // STALL is the expected answer, so it is not logged.
    static bool stallQuietly() {
        stallControlRequest();
        return true;
    }

    static bool handleGetDescriptor(SetupPacket const& pkt) {
        switch(pkt.descriptorType()) {
        case DescriptorType::device:                  return handleDeviceDescriptor(pkt);
        case DescriptorType::configuration:           return handleConfigDescriptor(pkt);
        case DescriptorType::string:                  return handleStringDescriptor(pkt);
        case DescriptorType::deviceQualifier:
        case DescriptorType::otherSpeedConfiguration:
        case DescriptorType::debug:                   return stallQuietly();
        default:                                      return false;
        }
    }

    static bool handleSetupPacketDeviceIn(SetupPacket const& pkt) {
        switch(pkt.bRequest) {
        case SetupPacket::Request::getDescriptor: return handleGetDescriptor(pkt);

        case SetupPacket::Request::getStatus:
            {
                std::array<std::byte, 2> const status{std::byte{Traits::BusPowered ? 0 : 1}};
                return ep0INDataPhase(status, pkt.wLength);
            }

        case SetupPacket::Request::getConfiguration:
            {
                std::array<std::byte, 1> const value{std::byte{configuration.load()}};
                return ep0INDataPhase(value, pkt.wLength);
            }

        default: return false;
        }
    }

    static bool handleSetupPacketDeviceOut(SetupPacket const& pkt) {
        switch(pkt.bRequest) {
        case SetupPacket::Request::setAddress:
            acknowledgeSetupRequest();
            // Set address is special: send 0-length status packet first with address 0
            deviceBusAddr     = pkt.wValue & 0x7f;
            pendingAddressSet = true;
            return true;

        case SetupPacket::Request::setConfiguration:
            using namespace std::string_view_literals;
            // The only configuration is 1.
            if((pkt.wValue & 0xff) > 1) { return false; }
            acknowledgeSetupRequest();
            configuration = static_cast<std::uint8_t>(pkt.wValue & 0xff);

            MixinsBase::callConfigured(configuration);
            UC_LOG_I("USB: Device {} (config={})",
                     configuration == 0 ? "unconfigured"sv : "configured"sv,
                     configuration.load());
            return true;

        default: return false;
        }
    }

    static constexpr std::size_t TotalInterfaceCount
      = detail::MixinTraits::countMixinInterfaces<Clock, ConfigT, Self, Mixins...>();

    // Standard requests no mixin claimed: interfaces without endpoints of their own always
    // run alternate setting 0, and EP0 is never halted.
    static bool handleStandardFallback(SetupPacket const& pkt) {
        using Request   = SetupPacket::Request;
        using Recipient = SetupPacket::Recipient;
        bool const in   = pkt.direction() == SetupPacket::Direction::deviceToHost;

        if(pkt.recipient() == Recipient::interface) {
            if(configuration == 0 || pkt.wIndex >= TotalInterfaceCount) { return false; }
            switch(pkt.bRequest) {
            case Request::getStatus:
                {
                    if(!in) { return false; }
                    static constexpr std::array<std::byte, 2> Status{};
                    return ep0INDataPhase(Status, pkt.wLength);
                }
            case Request::getInterface:
                {
                    if(!in) { return false; }
                    static constexpr std::array<std::byte, 1> AlternateSetting{};
                    return ep0INDataPhase(AlternateSetting, pkt.wLength);
                }
            case Request::setInterface:
                if(in || pkt.wValue != 0) { return false; }
                acknowledgeSetupRequest();
                return true;
            default: return false;
            }
        }
        if(pkt.recipient() == Recipient::endpoint && (pkt.wIndex & 0x7f) == 0
           && pkt.bRequest == Request::getStatus && in)
        {
            static constexpr std::array<std::byte, 2> Status{};
            return ep0INDataPhase(Status, pkt.wLength);
        }
        return false;
    }

    static void handleSetupPacket(SetupPacket const& pkt) {
        remainingControlData = {};
        endOfTransferPending = false;
        ep0_ctrl.transition(ControlStage::Setup);

        // Whatever the last transfer left armed is dropped, and the data stage starts at DATA1.
        Backend::beginControlTransfer();

        bool handled = false;

        if(pkt.type() == SetupPacket::Type::standard
           && pkt.recipient() == SetupPacket::Recipient::device)
        {
            if(pkt.direction() == SetupPacket::Direction::hostToDevice) {
                handled = handleSetupPacketDeviceOut(pkt);
            } else {
                handled = handleSetupPacketDeviceIn(pkt);
            }
        }
        if(!handled) { handled = MixinsBase::callSetupPacketRequest(pkt); }
        if(!handled && pkt.type() == SetupPacket::Type::standard) {
            handled = handleStandardFallback(pkt);
        }

        // Centralized error handling - USB 2.0 spec requires STALL for unsupported requests
        if(!handled) {
            KVASIR_LOG_LIMITED(
              faultLog_.allow(
                Kvasir::rateLimitKey(Fault::unhandledSetup, pkt.bmRequestType, pkt.bRequest)),
              UC_LOG_W,
              "USB: STALL - Unhandled setup packet: {}",
              pkt);
            stallControlRequest();
        }
    }

    template<bool Last>
    static void ep0IN(std::span<std::byte const> data) {
        EP0_IN::template tryTransfer<Last>(data);
    }

    template<bool Last>
    static void ep0OUT(std::size_t size) {
        EP0_OUT::template armReceive<Last>(size);
    }

public:
    // We can make them private with c++26 friend pack indexing
    //Mixin API
    // Multi-packet IN data stage, truncated to wLength. data must outlive the transfer.
    static bool ep0INDataPhase(std::span<std::byte const> data,
                               std::uint16_t              wLength) {
        // wLength 0: no data stage, the status stage follows directly.
        if(wLength == 0) {
            acknowledgeSetupRequest();
            return true;
        }
        data = data.first(std::min<std::size_t>(data.size(), wLength));
        // A short reply that ends on a packet boundary needs a zero-length packet.
        bool const endsOnPacketBoundary
          = !data.empty() && data.size() < wLength && data.size() % MaxPacketSize == 0;
        auto const first = data.first(std::min(data.size(), MaxPacketSize));

        ep0_ctrl.transition(ControlStage::Data);
        remainingControlData = data.subspan(first.size());
        endOfTransferPending = endsOnPacketBoundary;
        if(remainingControlData.empty() && !endOfTransferPending) {
            ep0IN<true>(first);
        } else {
            ep0IN<false>(first);
        }
        return true;
    }

    // Whole configuration descriptor, for mixins describing other mixins' interfaces.
    // Only usable where the device type is complete, i.e. inside a callback body.
    static constexpr std::span<std::byte const> configDescriptor() { return ConfigDescriptor; }

    static void ep0OUTDataPhase(std::size_t size) {
        assert(MaxPacketSize >= size);
        ep0_ctrl.transition(ControlStage::Data);
        ep0OUT<true>(size);
    }

    static bool ep0OUTGetData(std::span<std::byte> data) {
        if(ep0_ctrl.stage() == ControlStage::Data) {
            std::size_t const len = EP0_OUT::readCurrentBuffer(data);
            if(len != data.size()) {
                KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::invalidOutData)),
                                   UC_LOG_E,
                                   "USB: Invalid out data (received={}, expected={})",
                                   len,
                                   data.size());
                return false;
            }
            return true;
        } else {
            KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::outReadNotInDataPhase)),
                               UC_LOG_E,
                               "USB: out read attempted while not in data phase");
            return false;
        }
    }

    static void acknowledgeSetupRequest() {
        ep0_ctrl.transition(ControlStage::Status);
        ep0IN<true>(std::span<std::byte const>{});
    }

    // Ends the current control request with a STALL, from the setup or the data stage.
    static void stallControlRequest() {
        EP0_IN::stall();
        EP0_OUT::stall();
        ep0_ctrl.transition(ControlStage::Stall);
    }

    // Runs f with the USB interrupt masked, for thread code touching endpoint state the
    // interrupt also changes. Nests; safe from inside the interrupt too.
    template<typename F>
    static decltype(auto) withIsrMasked(F&& f) {
        struct Guard {
            Guard() {
                if(isrMaskDepth++ == 0) { Backend::maskInterrupt(); }
            }

            ~Guard() {
                if(--isrMaskDepth == 0) { Backend::unmaskInterrupt(); }
            }

            Guard(Guard const&)            = delete;
            Guard& operator=(Guard const&) = delete;
        };

        Guard const guard{};
        return std::forward<F>(f)();
    }

public:
    //Kvasir Callbacks
    using Isr = typename Backend::template Isr<&Device::onIsr>;

    static constexpr auto runtimeInit = []() {
        Backend::prepare();
        endpointConfig();
        Backend::connect();
    };

public:
    //Public API
    static bool isConfigured() { return configuration != 0; }

    /// Whether the bus is suspended: no traffic for 3 ms, the host asleep or the cable out. Only as
    /// good as the backend's report; one that reports nothing leaves it false.
    static bool isSuspended() { return suspended; }

    /// Off the bus and on again, for a device that wants the host to enumerate it afresh, or a
    /// self-powered one that watches VBUS itself. Where the backend can (Backend.hpp: disconnect()).
    static void disconnect()
        requires requires { Backend::disconnect(); }
    {
        withIsrMasked([] { Backend::disconnect(); });
    }

    static void connect() { Backend::connect(); }

    /// The controller's own checks, a log line each (Backend.hpp: selfTest()): what to call from
    /// main() on a board that does not enumerate, before reaching for a scope. True where the
    /// backend has none.
    static bool selfTest() {
        if constexpr(requires { Backend::selfTest(); }) {
            return Backend::selfTest();
        } else {
            return true;
        }
    }

    /// The device's I-th mixin, as the type that holds its state: `Usb::Function<1>::send(...)`
    /// for a device that has several with the same member names.
    template<std::size_t I>
    using Function = typename MixinsBase::template Function<I>;

    /// The interrupt's body, for a host test to run: on a target the vector table calls it.
    static void runInterrupt() { onIsr(); }
};
}   // namespace Kvasir::USB
