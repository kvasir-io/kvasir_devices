#pragma once

#include "Backend.hpp"
#include "Descriptors.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace Kvasir::USB::detail {

/// An interrupt IN endpoint: one packet at a time, handed over when the host next polls. What a
/// HID report or a CDC notification goes out on.
///
/// send() takes a packet if none is waiting and says so; one that was taken goes out exactly once,
/// unless the endpoint is restarted first (a halt clear, a reconfiguration, a bus reset), which
/// drops it: what an interrupt endpoint reports is a state, and a stale one is not worth
/// repeating.
///
/// It has the members handleEndpointRequest() asks of an endpoint (Address, halted, halt(),
/// clearHalt()), and the owner forwards its callbacks: setup(), bufferDone(), abortDone(),
/// busReset(), configured().
template<typename Derived, std::size_t EndpointNumber, std::size_t MaxSize = MaxPacketSize>
struct InterruptInEndpoint {
    static_assert(MaxSize <= MaxPacketSize,
                  "one packet per report");

    using EP
      = EndpointOf<Derived, EndpointNumber, EndpointDirection::In, EndpointTransferType::Interrupt>;

    static constexpr std::uint8_t Address
      = makeEndpointAddress(EndpointDirection::In, static_cast<std::uint8_t>(EndpointNumber));

    static inline std::array<std::byte, MaxSize> packet{};
    static inline bool                           halted{false};
    static inline bool                           restarting{false};   // a cancel is on its way

    static void setup() { EP::setupEndpoint(); }

    /// Whether send() would take a packet now.
    static bool ready() {
        return Derived::withIsrMasked([] {
            return Derived::isConfigured() && !halted && !restarting && EP::armedBuffers() == 0;
        });
    }

    /// The packet, for the host's next poll. False: not configured, halted, or the last one has
    /// not gone out yet.
    static bool send(std::span<std::byte const> data) {
        if(data.size() > MaxSize) { return false; }
        return Derived::withIsrMasked([&] {
            if(!Derived::isConfigured() || halted || restarting || EP::armedBuffers() != 0) {
                return false;
            }
            std::copy(data.begin(), data.end(), packet.begin());
            return EP::template tryTransfer<true>(std::span{packet}.first(data.size()));
        });
    }

    // Takes back whatever waits, and starts the data toggle over.
    static void restart() {
        if constexpr(EP::AsyncCancel) {
            restarting = true;
            EP::cancel();
        } else {
            static_cast<void>(EP::cancel());
            EP::resetDataToggle();
        }
    }

    static void bufferDone() {}

    static void abortDone() {
        if constexpr(EP::AsyncCancel) {
            static_cast<void>(EP::cancelComplete());
            EP::resetDataToggle();
            restarting = false;
        }
    }

    static void busReset() {
        halted     = false;
        restarting = false;
        static_cast<void>(EP::cancel());
        EP::reset();
    }

    // A (re)configuration also ends a halt (USB 2.0, 9.4.5).
    static void configured() {
        if(std::exchange(halted, false)) { EP::clearStall(); }
        restart();
    }

    static void halt() {
        halted = true;
        EP::stall();
    }

    static void clearHalt() {
        halted = false;
        EP::clearStall();
        restart();
    }
};
}   // namespace Kvasir::USB::detail
