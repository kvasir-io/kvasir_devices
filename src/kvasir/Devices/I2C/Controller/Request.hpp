#pragma once
// The request types of a queued I2C master (QueueCore.hpp, and the chip wrappers around it: chip_rp_common
// I2CQueued.hpp, chip_atsam_common Sercom_I2CQueued.hpp), in `Kvasir::I2C::Queue`. One set for every chip: a device
// written against `Master::Request` / `Master::Result` sees the same struct on each. The results are BusTypes.hpp's
// `Bus::Outcome` (failed, nack, ok) and, on a bus that tracks requests, `Bus::TrackedOutcome` (+ cancelled, timedOut).
#include "../../BusTypes.hpp"
#include "../../Features.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace Kvasir { namespace I2C { namespace Queue {

    /// What every request does with its prefix and send data, once for both request types: an empty base whose
    /// members take the request as their object (deducing this), so the fields stay the request's own direct members -
    /// a designated initialiser (`.address`, `.receiveData`, `.callback`) names only those - and the request keeps
    /// its size (the empty base takes no byte).
    struct PrefixedSend {
        static constexpr std::size_t MaxPrefix = 2;

        template<typename Self,
                 std::integral... B>
            requires(sizeof...(B) <= MaxPrefix)
        constexpr void setPrefix(this Self& self,
                                 B... b) {
            self.prefix      = {std::byte{static_cast<std::uint8_t>(b)}...};
            self.prefixBytes = sizeof...(B);
        }

        /// Everything that goes out after the address, prefix first.
        template<typename Self>
        [[nodiscard]] constexpr std::size_t sendBytes(this Self const& self) {
            return self.prefixBytes + self.sendData.size();
        }

        template<typename Self>
        [[nodiscard]] constexpr std::byte sendByte(this Self const& self,
                                                   std::size_t      i) {
            return i < self.prefixBytes ? self.prefix[i] : self.sendData[i - self.prefixBytes];
        }
    };

    template<std::size_t CallbackSize>
    struct I2CRequest : PrefixedSend {
        std::uint8_t address{};
        /// Sent ahead of sendData in the same write, with no START or STOP between them: a
        /// register address, an EEPROM memory address, a display's control byte. Up to two
        /// bytes, kept in the padding behind `address`, so the request stays the size it was
        /// (a gather list would need storage that outlives the queued request, for the one or
        /// two bytes every user has).
        std::uint8_t                                               prefixBytes{};
        std::array<std::byte, MaxPrefix>                           prefix{};
        std::span<std::byte const>                                 sendData{};
        std::span<std::byte>                                       receiveData{};
        StaticFunction<void(::Kvasir::Bus::Outcome), CallbackSize> callback{};
    };

    static_assert(sizeof(void*) != 4 || sizeof(I2CRequest<16>) == 40,
                  "the prefix must stay in the padding behind address");

    /// The request of a bus with I2CConfig::perDeviceClock: the device's clock set-up (SCL counts, BAUD) rides
    /// along. A type of its own, not a parameter of I2CRequest, so that a bus without the feature
    /// keeps the very same type (and names: a sanitize image hashes them).
    ///
    /// `BusDefault` is the timing of the bus's own rate: a request nobody gave a timing - the bus
    /// scan's probes, a raw request of the application's - goes out at baudRate, never at the
    /// all-zero counts a value-initialised member would be.
    template<std::size_t CallbackSize, typename Timing, Timing BusDefault>
    struct I2CTimedRequest : I2CRequest<CallbackSize> {
        Timing timing{BusDefault};
    };

    /// The request of a bus with `queueFeatures.cancel` / `.deadlines` (kvasir_devices BusTypes.hpp): I2CRequest's
    /// fields with the tracked result (Bus::TrackedOutcome), a ticket, the tombstone flag and an optional deadline. A type of its own, so a
    /// bus without the features keeps I2CRequest and its names.
    template<std::size_t CallbackSize, typename TimePoint, bool Deadlines>
    struct I2CTrackedRequest : PrefixedSend {
        std::uint8_t                                                              address{};
        std::uint8_t                                                              prefixBytes{};
        std::array<std::byte, MaxPrefix>                                          prefix{};
        std::span<std::byte const>                                                sendData{};
        std::span<std::byte>                                                      receiveData{};
        StaticFunction<void(::Kvasir::Bus::TrackedOutcome), CallbackSize>         callback{};
        std::uint16_t                                                             ticket{};
        bool                                                                      cancelled{};
        [[no_unique_address]] Kvasir::detail::IfFeature<Deadlines, TimePoint, 30> deadline{
          TimePoint::max()};
    };

    template<std::size_t CallbackSize,
             typename TimePoint,
             bool Deadlines,
             typename Timing,
             Timing BusDefault>
    struct I2CTimedTrackedRequest : I2CTrackedRequest<CallbackSize, TimePoint, Deadlines> {
        Timing timing{BusDefault};
    };

    /// The request type of a bus with these features: the one lattice both chips use.
    template<std::size_t CallbackSize,
             typename TimePoint,
             bool Tracked,
             bool Deadlines,
             bool PerDeviceClock,
             typename Timing,
             Timing BusDefault>
    using QueuedRequestFor = std::conditional_t<
      Tracked,
      std::conditional_t<
        PerDeviceClock,
        I2CTimedTrackedRequest<CallbackSize, TimePoint, Deadlines, Timing, BusDefault>,
        I2CTrackedRequest<CallbackSize, TimePoint, Deadlines>>,
      std::conditional_t<PerDeviceClock,
                         I2CTimedRequest<CallbackSize, Timing, BusDefault>,
                         I2CRequest<CallbackSize>>>;
}}}   // namespace Kvasir::I2C::Queue
