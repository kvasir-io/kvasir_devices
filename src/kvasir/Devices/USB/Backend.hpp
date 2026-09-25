#pragma once

#include "Descriptors.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

/// What a chip package supplies so that Kvasir::USB::Device runs on its USB controller.
///
/// A backend is a class template over the application's clock and config struct,
///
///     template<typename Clock, typename ConfigT> struct Backend;
///
/// all static like every Kvasir peripheral, and handed to the device as that template
/// (`Kvasir::USB::CdcAcm<HW::UsbBackend, Clock, UsbConfig>`). Nothing in this library includes a
/// chip header: everything below arrives through that one parameter.
///
/// THE CONTROLLER
///
///     static constexpr std::size_t MaxPacketSize;   // has to be USB::MaxPacketSize (64)
///     static constexpr std::size_t EndpointCount;   // endpoint numbers it has, endpoint 0 included
///     static constexpr bool        AsyncCancel;     // see "Taking packets back"
///
///     template<auto Handler> using Isr;             // the vector table entries that run Handler
///     using Provides; using Claims;                 // Startup resources        (each optional)
///     static constexpr auto powerClockEnable, initStepPinConfig, initStepPeripheryConfig,
///                           initStepInterruptConfig;   // Startup register lists (each optional)
///
///     static void prepare();        // once, before any endpoint is set up: the controller ready,
///                                   // its interrupts chosen, the device not yet on the bus
///     static void connect();        // the interrupt enabled and the pull-up on
///     static void disconnect();     // optional: the pull-up off again
///     static bool selfTest();       // optional: what the controller can check about itself
///                                   // (clocks on, out of reset, calibration programmed,
///                                   // attached), one log line per check, true = all passed
///     static void setAddress(std::uint8_t);   // the device calls it when SET_ADDRESS's status
///                                   // stage is over, and with 0 on a bus reset
///     static void beginControlTransfer();     // a SETUP arrived: whatever endpoint 0 still has
///                                   // armed is dropped, a stall on it ends, and the next packet
///                                   // in either direction is DATA1
///     static void maskInterrupt();  static void unmaskInterrupt();   // not nesting; the device
///                                   // counts (Device::withIsrMasked)
///
///     template<typename Sink> static void dispatchEvents();
///
/// dispatchEvents is the body of the interrupt: the device's handler calls it, and the backend
/// reads its flags, acknowledges them and tells the sink (static members, so it all inlines):
///
///     Sink::busReset()                         nothing else is reported from the same interrupt
///     Sink::setup(SetupPacket const&)          after the completions that came before it
///     Sink::transferComplete(ep, in)           one packet went out or came in
///     Sink::cancelComplete(ep, in)             AsyncCancel only
///     Sink::startOfFrame(std::uint16_t)        only when the config asks (ConfigTraits::UseSof)
///     Sink::suspend() / Sink::resume()         where the controller can tell; optional
///
/// Controller faults (CRC, timeouts, ...) the backend logs itself; they are not the protocol's.
///
/// AN ENDPOINT
///
///     template<std::size_t N, EndpointDirection Dir, EndpointTransferType Type> struct Endpoint;
///
/// One type per endpoint number and direction, whoever names it: its state is static. Packets are
/// at most MaxPacketSize, and the backend copies - a span handed over is free again on return.
///
///     static constexpr std::size_t QueueDepth;   // packets that can be with the controller at once
///     static constexpr bool        AsyncCancel;  // the controller's
///     using FreeBuffers = std::array<bool, ...>; // one flag per hardware buffer (>= QueueDepth)
///     static FreeBuffers freeBuffers();          // which of them are free, asked of the hardware
///     static std::size_t armedBuffers();         // how many are not
///
///     static void setupEndpoint();               // after prepare(), and again on every bus
///                                                // reset (some controllers forget), so it has
///                                                // to leave the endpoint idle: nothing armed,
///                                                // DATA0 next, not stalled
///     template<bool Last> static bool tryTransfer(std::span<std::byte const>);              // IN
///     template<bool Last> static bool tryTransfer(std::span<std::byte const>,
///                                                 FreeBuffers const&);                      // IN
///     template<bool Last = false> static bool armReceive(std::size_t maxSize);              // OUT
///     static std::size_t readCurrentBuffer(std::span<std::byte>);                           // OUT
///
/// readCurrentBuffer is called at most once per transferComplete, before the endpoint is armed
/// again. The data toggle is the backend's business entirely: the device never says when a
/// packet is over, only when the toggle has to start again (resetDataToggle, reset).
///
///     static void stall();  static void clearStall();
///     static void resetDataToggle();             // next packet DATA0
///     static void reset();                       // bus reset: toggle and buffer selection
///
/// More than a packet per arm. A controller that splits an IN transfer into packets by itself
/// says how much one arm may carry, and takes the bytes where they are instead of copying them:
///
///     static constexpr std::size_t MaxTransfer;  // optional; a multiple of MaxPacketSize
///     template<bool Last> static bool tryTransferInPlace(std::span<std::byte const>,
///                                                        FreeBuffers const&);
///         // optional, IN: up to MaxTransfer bytes in word-aligned RAM that stays as it is until
///         // transferComplete or a cancel. One transferComplete per transfer, not per packet.
///     static std::size_t sentOfCancelled();
///         // with it: how much of the first transfer a cancel took back had already gone out
///
/// The bulk IN endpoint uses them where they exist. The OUT counterpart, for a controller that
/// gathers the packets of a transfer by itself and whose cancel is done on return:
///
///     static constexpr std::size_t MaxReceive;   // optional; a multiple of MaxPacketSize
///     static bool armReceiveInto(std::span<std::byte>);
///         // optional, OUT: a multiple of MaxPacketSize, up to MaxReceive, in word-aligned RAM
///         // that is the controller's until transferComplete, takeBackReceive() or a cancel.
///         // One transferComplete per transfer: when it is full or a short packet ends it. The
///         // controller may dirty up to two bytes behind a short packet (the SAM one writes its
///         // CRC there), still inside what was armed.
///     static std::size_t received();        // in transferComplete: how long that transfer was
///     static std::size_t receivedSoFar();   // while armed: what has come - a hint, not a fence
///     static std::optional<std::size_t> takeBackReceive();
///         // ends the arm and says what had come and was acknowledged; nothing if the transfer
///         // completed meanwhile, whose transferComplete then follows as usual
///
/// A transfer of several packets only completes when it is full or a short packet ends it, so
/// what a host wrote as exactly one full packet waits in the controller (cdc_acm and libusb send
/// no zero-length packet behind it). The bulk OUT endpoint therefore takes a transfer back that
/// has stopped growing, and one that a halt clear interrupts, with what it holds.
///
/// Taking packets back. cancel() takes whatever is armed away from the controller.
///
///     AsyncCancel = false    auto cancel() -> std::size_t;   done on return. IN: how many armed
///                            packets were never sent, which the caller may hand over again.
///     AsyncCancel = true     void cancel();                  a request. The controller answers
///                            with Sink::cancelComplete, and there
///                            std::size_t cancelComplete();   finishes it and returns that count.
///
/// Anything more a chip's endpoint can do stays public on its type and is the application's to
/// use at its own risk; EndpointOf names the type.
namespace Kvasir::USB {

template<template<typename, typename> class BackendT,
         typename Clock,
         typename ConfigT,
         DeviceClass Class,
         DeviceClass SubClass,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
struct Device;

namespace detail {
    // A mixin is a base of the device and so is instantiated while the device is incomplete:
    // the backend is read off the device's template arguments, not out of its body.
    template<typename DeviceT>
    struct BackendOf;

    template<template<typename, typename> class BackendT,
             typename Clock,
             typename ConfigT,
             DeviceClass Class,
             DeviceClass SubClass,
             std::size_t FirstInterfaceNumber,
             std::size_t FirstEndpointNumber,
             template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
    struct BackendOf<Device<BackendT,
                            Clock,
                            ConfigT,
                            Class,
                            SubClass,
                            FirstInterfaceNumber,
                            FirstEndpointNumber,
                            Mixins...>> {
        using type = BackendT<Clock, ConfigT>;
    };
}   // namespace detail

/// The backend of a device, usable from a mixin (where the device is still incomplete).
template<typename DeviceT>
using BackendOf = typename detail::BackendOf<DeviceT>::type;

/// One of a device's endpoints.
template<typename DeviceT, std::size_t Number, EndpointDirection Dir, EndpointTransferType Type>
using EndpointOf = typename BackendOf<DeviceT>::template Endpoint<Number, Dir, Type>;

namespace detail {
    template<typename E>
    concept EndpointCommon = requires {
        { E::QueueDepth } -> std::convertible_to<std::size_t>;
        { E::AsyncCancel } -> std::convertible_to<bool>;
        typename E::FreeBuffers;
        { E::freeBuffers() } -> std::same_as<typename E::FreeBuffers>;
        { E::armedBuffers() } -> std::convertible_to<std::size_t>;
        E::setupEndpoint();
        E::stall();
        E::clearStall();
        E::resetDataToggle();
        E::reset();
        E::cancel();
    };

    template<typename E>
    concept EndpointCancel = (E::AsyncCancel && requires {
                                 { E::cancelComplete() } -> std::convertible_to<std::size_t>;
                             }) || (!E::AsyncCancel && requires {
                                 { E::cancel() } -> std::convertible_to<std::size_t>;
                             });

    template<typename E>
    concept InEndpoint
      = EndpointCommon<E> && EndpointCancel<E>
     && requires(std::span<std::byte const> data, typename E::FreeBuffers const& free) {
            { E::template tryTransfer<true>(data) } -> std::same_as<bool>;
            { E::template tryTransfer<false>(data, free) } -> std::same_as<bool>;
        };

    template<typename E>
    concept OutEndpoint
      = EndpointCommon<E> && EndpointCancel<E> && requires(std::span<std::byte> data) {
            { E::template armReceive<true>(std::size_t{}) } -> std::same_as<bool>;
            { E::readCurrentBuffer(data) } -> std::convertible_to<std::size_t>;
        };

    template<typename B>
    concept Controller = requires(std::uint8_t address) {
        { B::MaxPacketSize } -> std::convertible_to<std::size_t>;
        { B::EndpointCount } -> std::convertible_to<std::size_t>;
        { B::AsyncCancel } -> std::convertible_to<bool>;
        B::prepare();
        B::connect();
        B::setAddress(address);
        B::beginControlTransfer();
        B::maskInterrupt();
        B::unmaskInterrupt();
    };
}   // namespace detail
}   // namespace Kvasir::USB
