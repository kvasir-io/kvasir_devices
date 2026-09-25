#pragma once

// Must come before any driver header: the uc_log macros as counters.
#include <support/LogStubs.hpp>
//
#include "../i2c/Check.hpp"
#include "FakeUsbBackend.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <support/FakeClock.hpp>
#include <vector>

/// The host a test plays against a device on the fake controller: control transfers the way a
/// host controller runs them (SETUP, the data stage in 64-byte packets until a short one, the
/// status stage the other way), bulk reads and writes, enumeration. Every step runs the device's
/// interrupt, so by the time a call returns the device has reacted.
namespace Kvasir::Test {

using Clock = FakeClock;

template<typename Fake, typename Device>
struct UsbHost {
    using Setup     = Kvasir::USB::SetupPacket;
    using Handshake = typename Fake::Handshake;

    static constexpr std::size_t MaxPacket = Fake::MaxPacket;

    /// The interrupt, until the controller has nothing more to say (a bus reset is an interrupt of
    /// its own, and a cancel the device asks for in one is answered in the next).
    static void pump() {
        Fake::inInterrupt = true;
        for(int i = 0; i != 16 && !Fake::events.empty(); ++i) { Device::runInterrupt(); }
        Fake::inInterrupt = false;
    }

    /// Power on: what Startup does with the device, then the host's reset.
    static void plugIn() {
        Fake::resetAll();
        Fake::isr = [] { Device::runInterrupt(); };
        Clock::reset();
        Device::runtimeInit();
        busReset();
    }

    static void busReset() {
        Fake::hostBusReset();
        pump();
    }

    static Setup makeSetup(std::uint8_t  bmRequestType,
                           std::uint8_t  bRequest,
                           std::uint16_t wValue,
                           std::uint16_t wIndex,
                           std::uint16_t wLength) {
        Setup s{};
        s.bmRequestType = bmRequestType;
        s.bRequest      = static_cast<Setup::Request>(bRequest);
        s.wValue        = wValue;
        s.wIndex        = wIndex;
        s.wLength       = wLength;
        return s;
    }

    /// What one control transfer came to.
    struct ControlResult {
        bool                   stalled{false};
        bool                   timedOut{false};   // the device NAKed where it had to answer
        std::vector<std::byte> data{};
        std::vector<bool>      data1{};   // the PID of each data packet, for the toggle checks

        bool ok() const { return !stalled && !timedOut; }
    };

    /// A control read: SETUP, IN packets until a short one or wLength bytes, a zero-length OUT.
    static ControlResult controlIn(Setup const& setup) {
        ControlResult r{};
        Fake::hostSetup(setup);
        pump();
        while(r.data.size() < setup.wLength) {
            auto in = Fake::hostIn(0);
            if(in.handshake == Handshake::stall) {
                r.stalled = true;
                return r;
            }
            if(in.handshake == Handshake::nak) {
                r.timedOut = true;
                return r;
            }
            std::size_t const n = in.packet.data.size();
            r.data.insert(r.data.end(), in.packet.data.begin(), in.packet.data.end());
            r.data1.push_back(in.packet.data1);
            pump();
            if(n < MaxPacket) { break; }
        }
        auto const status = Fake::hostOut(0, {});
        if(status == Handshake::stall) { r.stalled = true; }
        if(status == Handshake::nak) { r.timedOut = true; }
        pump();
        return r;
    }

    /// A control write: SETUP, the data as OUT packets, a zero-length IN.
    static ControlResult controlOut(Setup const&               setup,
                                    std::span<std::byte const> data = {}) {
        ControlResult r{};
        Fake::hostSetup(setup);
        pump();
        while(!data.empty()) {
            auto const chunk = data.first(std::min(data.size(), MaxPacket));
            auto const hs    = Fake::hostOut(0, chunk);
            if(hs == Handshake::stall) {
                r.stalled = true;
                return r;
            }
            if(hs == Handshake::nak) {
                r.timedOut = true;
                return r;
            }
            data = data.subspan(chunk.size());
            pump();
        }
        auto in = Fake::hostIn(0);
        if(in.handshake == Handshake::stall) { r.stalled = true; }
        if(in.handshake == Handshake::nak) { r.timedOut = true; }
        if(in.handshake == Handshake::ack && !in.packet.data.empty()) { r.timedOut = true; }
        pump();
        return r;
    }

    // ---- the standard requests ----------------------------------------------------------------

    static ControlResult getDescriptor(std::uint8_t  type,
                                       std::uint8_t  index,
                                       std::uint16_t length,
                                       std::uint16_t language = 0) {
        return controlIn(
          makeSetup(0x80, 6, static_cast<std::uint16_t>((type << 8) | index), language, length));
    }

    static ControlResult setAddress(std::uint8_t address) {
        return controlOut(makeSetup(0x00, 5, address, 0, 0));
    }

    static ControlResult setConfiguration(std::uint8_t value) {
        return controlOut(makeSetup(0x00, 9, value, 0, 0));
    }

    static ControlResult clearHalt(std::uint8_t endpointAddress) {
        return controlOut(makeSetup(0x02, 1, 0, endpointAddress, 0));
    }

    static ControlResult setHalt(std::uint8_t endpointAddress) {
        return controlOut(makeSetup(0x02, 3, 0, endpointAddress, 0));
    }

    /// What a host does with a new device, as far as a test needs it.
    static bool enumerate(std::uint8_t address = 7) {
        return getDescriptor(1, 0, 64).ok() && setAddress(address).ok()
            && getDescriptor(1, 0, 18).ok() && getDescriptor(2, 0, 9).ok()
            && setConfiguration(1).ok();
    }

    // ---- bulk ---------------------------------------------------------------------------------

    /// One IN transfer: packets until a short one, or until the device has nothing more armed.
    struct BulkResult {
        std::vector<std::byte> data{};
        std::vector<bool>      data1{};
        bool                   endedShort{false};
        bool                   stalled{false};
    };

    static BulkResult bulkIn(std::size_t ep,
                             std::size_t maxBytes = 1U << 20) {
        BulkResult r{};
        while(r.data.size() < maxBytes) {
            auto in = Fake::hostIn(ep);
            if(in.handshake == Handshake::stall) {
                r.stalled = true;
                return r;
            }
            if(in.handshake == Handshake::nak) { return r; }
            std::size_t const n = in.packet.data.size();
            r.data.insert(r.data.end(), in.packet.data.begin(), in.packet.data.end());
            r.data1.push_back(in.packet.data1);
            pump();
            if(n < MaxPacket) {
                r.endedShort = true;
                return r;
            }
        }
        return r;
    }

    /// OUT packets until all of it went or the device NAKs; returns how many bytes it took.
    static std::size_t bulkOut(std::size_t                ep,
                               std::span<std::byte const> data) {
        std::size_t sent{};
        while(sent < data.size()) {
            auto const chunk = data.subspan(sent, std::min(data.size() - sent, MaxPacket));
            if(Fake::hostOut(ep, chunk) != Handshake::ack) { return sent; }
            sent += chunk.size();
            pump();
        }
        return sent;
    }
};

inline std::vector<std::byte> pattern(std::size_t n,
                                      std::size_t seed = 0) {
    std::vector<std::byte> v(n);
    for(std::size_t i = 0; i != n; ++i) { v[i] = static_cast<std::byte>((i * 7 + seed) & 0xff); }
    return v;
}
}   // namespace Kvasir::Test
