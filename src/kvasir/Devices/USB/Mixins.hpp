#pragma once

#include "Descriptors.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <utility>

namespace Kvasir::USB::detail {

// Accessor struct for mixin traits - allows friend access to private members
struct MixinTraits {
    // Forward declaration of MixinBases
    template<typename Clock,
             typename Config,
             typename Derived,
             std::size_t BaseInterface,
             std::size_t BaseEndpoint,
             typename Indices,
             template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
    struct MixinBases;

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval auto getMixinInterfaceCounts() {
        if constexpr(sizeof...(Mixins) > 0) {
            return std::array<std::size_t, sizeof...(Mixins)>{
              Mixins<Clock, Config, Derived, 0, 0>::InterfaceCount...};
        } else {
            return std::array<std::size_t, 0>{};
        }
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::size_t getInterfaceOffset(std::size_t mixin_index) {
        constexpr auto counts = getMixinInterfaceCounts<Clock, Config, Derived, Mixins...>();
        std::size_t    offset = 0;
        for(std::size_t i = 0; i < mixin_index; ++i) { offset += counts[i]; }
        return offset;
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval auto getMixinEndpointCounts() {
        if constexpr(sizeof...(Mixins) > 0) {
            return std::array<std::size_t, sizeof...(Mixins)>{
              Mixins<Clock, Config, Derived, 0, 0>::EndpointCount...};
        } else {
            return std::array<std::size_t, 0>{};
        }
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::size_t getEndpointOffset(std::size_t mixin_index) {
        constexpr auto counts = getMixinEndpointCounts<Clock, Config, Derived, Mixins...>();
        std::size_t    offset = 0;
        for(std::size_t i = 0; i < mixin_index; ++i) { offset += counts[i]; }
        return offset;
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::size_t countMixinInterfaces() {
        return (Mixins<Clock, Config, Derived, 0, 0>::InterfaceCount + ... + 0);
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::size_t countMixinEndpoints() {
        return (Mixins<Clock, Config, Derived, 0, 0>::EndpointCount + ... + 0);
    }

    // A mixin may require a newer bcdUSB than 2.00 (e.g. 2.01 for a BOS descriptor).
    template<typename Mixin>
    static consteval std::uint16_t getMixinBcdUSB() {
        if constexpr(requires { Mixin::BcdUSB; }) {
            return Mixin::BcdUSB;
        } else {
            return Kvasir::USB::detail::bcdUSB;
        }
    }

    template<typename Clock,
             typename Config,
             typename Derived,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval std::uint16_t maxBcdUSB() {
        return std::max(
          {Kvasir::USB::detail::bcdUSB, getMixinBcdUSB<Mixins<Clock, Config, Derived, 0, 0>>()...});
    }

    // Helper wrapper for passing mixin packs
    template<template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
    struct MixinPack {};

    // Helper to concatenate mixin descriptors at compile-time
    template<typename Clock,
             typename Config,
             typename Derived,
             std::size_t BaseInterface,
             std::size_t BaseEndpoint,
             std::size_t... Is,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval auto assembleMixinDescriptors(std::index_sequence<Is...>,
                                                   MixinPack<Mixins...>) {
        auto extractIfPresent = []<std::size_t I,
                                   template<typename, typename, typename, std::size_t, std::size_t>
                                   class Mixin>() {
            using InstantiatedMixin = Mixin<
              Clock,
              Config,
              Derived,
              BaseInterface + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(I),
              BaseEndpoint + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(I)>;
            return Descriptors::detail::generateArray(InstantiatedMixin::InterfaceDescriptor);
        };

        if constexpr(sizeof...(Mixins) == 0) {
            return std::array<std::byte, 0>{};
        } else {
            return Descriptors::detail::generateArray(
              extractIfPresent.template operator()<Is, Mixins>()...);
        }
    }

    // Wrapper for easier use - directly calls the index_sequence version
    template<typename Clock,
             typename Config,
             typename Derived,
             std::size_t BaseInterface,
             std::size_t BaseEndpoint,
             template<typename,
                      typename,
                      typename,
                      std::size_t,
                      std::size_t> class... Mixins>
    static consteval auto assembleMixinDescriptors() {
        return assembleMixinDescriptors<Clock, Config, Derived, BaseInterface, BaseEndpoint>(
          std::make_index_sequence<sizeof...(Mixins)>{},
          MixinPack<Mixins...>{});
    }
};

// MixinBases specialization - inherits from all mixins
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t BaseInterface,
         std::size_t BaseEndpoint,
         std::size_t... Is,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
struct MixinTraits::MixinBases<Clock,
                               Config,
                               Derived,
                               BaseInterface,
                               BaseEndpoint,
                               std::index_sequence<Is...>,
                               Mixins...>
  : Mixins<Clock,
           Config,
           Derived,
           BaseInterface + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(Is),
           BaseEndpoint
             + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(Is)>... {
    // Mixin number I where the device put it: with its first interface and endpoint number.
    template<template<typename, typename, typename, std::size_t, std::size_t> class M,
             std::size_t I>
    using Placed
      = M<Clock,
          Config,
          Derived,
          BaseInterface + MixinTraits::getInterfaceOffset<Clock, Config, Derived, Mixins...>(I),
          BaseEndpoint + MixinTraits::getEndpointOffset<Clock, Config, Derived, Mixins...>(I)>;

    /// The device's I-th mixin as the type that holds its state, for an application that has
    /// several with the same member names (`Usb::Function<1>::send(...)`). The numbers are the
    /// device's, so they cannot be got wrong.
    template<std::size_t I>
    using Function = std::tuple_element_t<I, std::tuple<Placed<Mixins, Is>...>>;

    // A mixin has the callbacks it needs and no others:
    //
    //     static void SetupEndpointsCallback();
    //     static bool SetupPacketRequestCallback(SetupPacket const&);     // true: handled
    //     static bool EndpointHandlerCallback(std::size_t ep, bool in);   // true: mine
    //     static bool AbortDoneCallback(std::size_t ep, bool in);         // true: mine; only a
    //                                          // backend with AsyncCancel ever reports one
    //     static void ResetCallback();
    //     static void ConfiguredCallback(std::uint8_t configuration);
    //     static void SuspendCallback(bool suspended);
    template<typename M>
    static void setupEndpoints() {
        if constexpr(requires { M::SetupEndpointsCallback(); }) { M::SetupEndpointsCallback(); }
    }

    template<typename M>
    static bool endpointHandler(std::size_t epNum,
                                bool        in) {
        if constexpr(requires { M::EndpointHandlerCallback(epNum, in); }) {
            return M::EndpointHandlerCallback(epNum, in);
        } else {
            return false;
        }
    }

    template<typename M>
    static bool abortDone(std::size_t epNum,
                          bool        in) {
        if constexpr(requires { M::AbortDoneCallback(epNum, in); }) {
            return M::AbortDoneCallback(epNum, in);
        } else {
            return false;
        }
    }

    template<typename M>
    static void reset() {
        if constexpr(requires { M::ResetCallback(); }) { M::ResetCallback(); }
    }

    template<typename M>
    static void configured(std::uint8_t configuration) {
        if constexpr(requires { M::ConfiguredCallback(configuration); }) {
            M::ConfiguredCallback(configuration);
        }
    }

    template<typename M>
    static void suspended(bool isSuspended) {
        if constexpr(requires { M::SuspendCallback(isSuspended); }) {
            M::SuspendCallback(isSuspended);
        }
    }

    template<typename M>
    static bool setupPacketRequest(SetupPacket const& pkt) {
        if constexpr(requires { M::SetupPacketRequestCallback(pkt); }) {
            return M::SetupPacketRequestCallback(pkt);
        } else {
            return false;
        }
    }

    // Helper methods to dispatch to all mixins
    static void callSetupEndpoints() { (setupEndpoints<Placed<Mixins, Is>>(), ...); }

    static bool callEndpointHandler([[maybe_unused]] std::size_t epNum,
                                    [[maybe_unused]] bool        in) {
        return (endpointHandler<Placed<Mixins, Is>>(epNum, in) || ... || false);
    }

    static bool callAbortDone([[maybe_unused]] std::size_t epNum,
                              [[maybe_unused]] bool        in) {
        return (abortDone<Placed<Mixins, Is>>(epNum, in) || ... || false);
    }

    static void callReset() { (reset<Placed<Mixins, Is>>(), ...); }

    static void callConfigured([[maybe_unused]] std::uint8_t configuration) {
        (configured<Placed<Mixins, Is>>(configuration), ...);
    }

    static void callSuspended([[maybe_unused]] bool isSuspended) {
        (suspended<Placed<Mixins, Is>>(isSuspended), ...);
    }

    static bool callSetupPacketRequest([[maybe_unused]] SetupPacket const& pkt) {
        return (setupPacketRequest<Placed<Mixins, Is>>(pkt) || ... || false);
    }
};

}   // namespace Kvasir::USB::detail
