#pragma once

#include "Descriptors.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <utility>

namespace Kvasir::USB {

// The one friend a mixin names (`friend struct Kvasir::USB::MixinTraits;`): what the device reads
// out of its mixins - their interface and endpoint counts, descriptors and callbacks - is private
// to each mixin, and Layout below is a member of this struct.
struct MixinTraits {
    // A mixin may require a newer bcdUSB than 2.00 (e.g. 2.01 for a BOS descriptor).
    template<typename Mixin>
    static consteval std::uint16_t bcdUSBOf() {
        if constexpr(requires { Mixin::BcdUSB; }) {
            return Mixin::BcdUSB;
        } else {
            return Kvasir::USB::detail::bcdUSB;
        }
    }

    // Element i: the sum of the counts before it; the last element is the total.
    template<std::size_t N>
    static consteval std::array<std::size_t,
                                N + 1>
    prefixSums(std::array<std::size_t,
                          N> const& counts) {
        std::array<std::size_t, N + 1> sums{};
        for(std::size_t i = 0; i != N; ++i) { sums[i + 1] = sums[i] + counts[i]; }
        return sums;
    }

    /// Where a device's mixins sit: each mixin's first interface and endpoint number after the
    /// ones before it, computed once. Indices is std::make_index_sequence<sizeof...(Mixins)>.
    template<typename Clock,
             typename Config,
             typename Derived,
             typename Indices,
             template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
    struct Layout;
};

template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t... Is,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
struct MixinTraits::Layout<Clock, Config, Derived, std::index_sequence<Is...>, Mixins...> {
    // The counts and bcdUSB do not depend on where a mixin sits: read at numbers 0, 0.
    static constexpr std::array<std::size_t, sizeof...(Mixins)> InterfaceCounts{
      Mixins<Clock, Config, Derived, 0, 0>::InterfaceCount...};
    static constexpr std::array<std::size_t, sizeof...(Mixins)> EndpointCounts{
      Mixins<Clock, Config, Derived, 0, 0>::EndpointCount...};

    static constexpr auto InterfaceOffsets = MixinTraits::prefixSums(InterfaceCounts);
    static constexpr auto EndpointOffsets  = MixinTraits::prefixSums(EndpointCounts);

    static constexpr std::size_t InterfaceCount = InterfaceOffsets.back();
    static constexpr std::size_t EndpointCount  = EndpointOffsets.back();

    static constexpr std::uint16_t BcdUSB
      = std::max({Kvasir::USB::detail::bcdUSB,
                  MixinTraits::bcdUSBOf<Mixins<Clock, Config, Derived, 0, 0>>()...});

    /// The mixins placed from the device's first interface and endpoint number on; the device
    /// derives from this.
    template<std::size_t BaseInterface, std::size_t BaseEndpoint>
    struct Bases
      : Mixins<Clock,
               Config,
               Derived,
               BaseInterface + InterfaceOffsets[Is],
               BaseEndpoint + EndpointOffsets[Is]>... {
        using Placed = std::tuple<Mixins<Clock,
                                         Config,
                                         Derived,
                                         BaseInterface + InterfaceOffsets[Is],
                                         BaseEndpoint + EndpointOffsets[Is]>...>;

        /// The device's I-th mixin as the type that holds its state, for an application that has
        /// several with the same member names (`Usb::Function<1>::send(...)`). The numbers are the
        /// device's, so they cannot be got wrong.
        template<std::size_t I>
        using Function = std::tuple_element_t<I, Placed>;

        // The mixins' interface blocks end to end, in the order of the mixins.
        static consteval auto interfaceDescriptors() {
            if constexpr(sizeof...(Mixins) == 0) {
                return std::array<std::byte, 0>{};
            } else {
                return Descriptors::generateArray(
                  Descriptors::generateArray(Function<Is>::InterfaceDescriptor)...);
            }
        }

        // A mixin has the callbacks it needs and no others:
        //
        //     static void setupEndpointsCallback();
        //     static bool setupPacketRequestCallback(SetupPacket const&);     // true: handled
        //     static bool endpointHandlerCallback(std::size_t ep, bool in);   // true: mine
        //     static bool abortDoneCallback(std::size_t ep, bool in);         // true: mine; only a
        //                                          // backend with asyncCancel ever reports one
        //     static void resetCallback();
        //     static void configuredCallback(std::uint8_t configuration);
        //     static void suspendCallback(bool suspended);
        template<typename M>
        static void setupEndpoints() {
            if constexpr(requires { M::setupEndpointsCallback(); }) { M::setupEndpointsCallback(); }
        }

        template<typename M>
        static bool endpointHandler(std::size_t epNum,
                                    bool        in) {
            if constexpr(requires { M::endpointHandlerCallback(epNum, in); }) {
                return M::endpointHandlerCallback(epNum, in);
            } else {
                return false;
            }
        }

        template<typename M>
        static bool abortDone(std::size_t epNum,
                              bool        in) {
            if constexpr(requires { M::abortDoneCallback(epNum, in); }) {
                return M::abortDoneCallback(epNum, in);
            } else {
                return false;
            }
        }

        template<typename M>
        static void reset() {
            if constexpr(requires { M::resetCallback(); }) { M::resetCallback(); }
        }

        template<typename M>
        static void configured(std::uint8_t configuration) {
            if constexpr(requires { M::configuredCallback(configuration); }) {
                M::configuredCallback(configuration);
            }
        }

        template<typename M>
        static void suspended(bool isSuspended) {
            if constexpr(requires { M::suspendCallback(isSuspended); }) {
                M::suspendCallback(isSuspended);
            }
        }

        template<typename M>
        static bool setupPacketRequest(SetupPacket const& pkt) {
            if constexpr(requires { M::setupPacketRequestCallback(pkt); }) {
                return M::setupPacketRequestCallback(pkt);
            } else {
                return false;
            }
        }

        // Helper methods to dispatch to all mixins
        static void callSetupEndpoints() { (setupEndpoints<Function<Is>>(), ...); }

        static bool callEndpointHandler([[maybe_unused]] std::size_t epNum,
                                        [[maybe_unused]] bool        in) {
            return (endpointHandler<Function<Is>>(epNum, in) || ... || false);
        }

        static bool callAbortDone([[maybe_unused]] std::size_t epNum,
                                  [[maybe_unused]] bool        in) {
            return (abortDone<Function<Is>>(epNum, in) || ... || false);
        }

        static void callReset() { (reset<Function<Is>>(), ...); }

        static void callConfigured([[maybe_unused]] std::uint8_t configuration) {
            (configured<Function<Is>>(configuration), ...);
        }

        static void callSuspended([[maybe_unused]] bool isSuspended) {
            (suspended<Function<Is>>(isSuspended), ...);
        }

        static bool callSetupPacketRequest([[maybe_unused]] SetupPacket const& pkt) {
            return (setupPacketRequest<Function<Is>>(pkt) || ... || false);
        }
    };
};

}   // namespace Kvasir::USB
