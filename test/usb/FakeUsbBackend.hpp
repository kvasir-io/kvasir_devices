#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <kvasir/Devices/USB/Backend.hpp>
#include <kvasir/Devices/USB/Config.hpp>
#include <kvasir/Devices/USB/Descriptors.hpp>
#include <optional>
#include <span>
#include <vector>

/// A USB controller for host tests of the USB device (src/kvasir/Devices/USB), in the shape
/// Backend.hpp asks of a chip package - and with the *host* on its other side: the test plays
/// the host through hostSetup / hostIn / hostOut / hostBusReset, which do what the wire would and
/// queue the events a controller would raise. `Device::runInterrupt()` then is the interrupt, the
/// way FakeBus::complete() is for the I2C engine.
///
/// Two controllers are worth testing against, and the template parameters pick one:
///
///   Depth = 1, Async = false   one packet per endpoint, a cancel is done on return, the data
///                              toggle is the hardware's: a SAM D21
///   Depth = 2, Async = true    two IN packets armed at once, a cancel is a request the
///                              controller answers later: an RP2040 / RP2350
///   Transfer > 64              one arm carries a run of packets which the controller splits and
///                              gathers by itself, reading and writing the caller's memory in
///                              place: what the SAM controller can do (Endpoint::MaxTransfer,
///                              tryTransferInPlace). The fake reads an IN
///                              transfer's bytes when the host asks for them, not when it is
///                              armed - so a caller that reuses the memory too early is caught.
///
/// The data toggle is kept the way hardware keeps it - it flips when a packet goes over the
/// wire - so what the host sees (Packet::data1) tells whether the device reset it when it had to.
///
/// The fake also watches the contract: `violations` counts what a backend may rely on and the
/// device did not do (arming an endpoint that was never set up, a packet larger than the
/// endpoint's, an unmask without a mask, ...). A test ends by checking it is 0.
namespace Kvasir::Test {

template<std::size_t Depth, bool Async, std::size_t Transfer = 64, typename Tag = void>
struct FakeUsb {
    static constexpr std::size_t MaxPacket = Kvasir::USB::MaxPacketSize;
    static constexpr std::size_t Endpoints = 8;

    /// What the host got for an IN token, or gave with an OUT token.
    enum class Handshake : std::uint8_t { ack, nak, stall };

    struct Packet {
        std::vector<std::byte> data;
        bool                   data1{};
    };

    struct InResult {
        Handshake handshake{Handshake::nak};
        Packet    packet{};
    };

    struct Event {
        enum class Kind : std::uint8_t {
            busReset,
            setup,
            transferComplete,
            cancelComplete,
            sof,
            suspend,
            resume
        };

        Kind                     kind{};
        std::size_t              ep{};
        bool                     in{};
        Kvasir::USB::SetupPacket setup{};
        std::uint16_t            frame{};
    };

    static constexpr bool InPlace = Transfer > MaxPacket;

    // One arm of an IN endpoint: a packet, or a run of them that goes out 64 bytes at a time.
    struct ArmedIn {
        std::vector<std::byte> copy{};      // the bytes, where the backend copied them
        std::byte const*       inPlace{};   // or where the caller keeps them
        std::size_t            size{};
        std::size_t            sent{};

        std::byte const* data() const { return inPlace != nullptr ? inPlace : copy.data(); }
    };

    struct EndpointState {
        std::deque<ArmedIn>        armedIn{};      // IN: transfers with the controller
        std::optional<std::size_t> armedOut{};     // OUT: armed for up to this many bytes
        std::byte*                 outInPlace{};   // OUT: where the caller wants them, if in place
        std::vector<std::byte>     received{};     // OUT: what came in
        std::size_t                sentOfCancelled{};
        bool                       hasReceived{false};
        bool                       stalled{false};
        bool                       data1{false};   // the toggle of the next packet
        bool                       set{false};     // setupEndpoint() was called
        bool                       cancelPending{false};
        Kvasir::USB::EndpointTransferType type{};

        void clear() { *this = EndpointState{.set = set, .type = type}; }
    };

    // [endpoint number][0 = OUT, 1 = IN]
    static inline std::array<std::array<EndpointState, 2>, Endpoints> endpoints{};
    static inline std::deque<Event>                                   events{};
    static inline std::uint8_t                                        address{};
    static inline bool                                                prepared{false};
    static inline bool                                                connected{false};
    static inline int                                                 maskDepth{};
    static inline int                                                 violations{};
    static inline int                                                 controlTransfersBegun{};
    static inline std::vector<std::uint8_t>                           addressHistory{};

    /// The interrupt as hardware raises it: not when the test next pumps, but the moment it can
    /// run - right before the application masks it and right after it unmasks it again, which
    /// are the two places where thread code and the interrupt can disagree about an endpoint.
    /// A test sets `isr` (UsbHost does) and turns `eagerInterrupts` on.
    static inline void (*isr)(){};
    static inline bool eagerInterrupts{false};
    static inline bool inInterrupt{false};

    static void raiseIfPending() {
        if(!eagerInterrupts || isr == nullptr || inInterrupt || maskDepth != 0) { return; }
        inInterrupt = true;
        for(int i = 0; i != 16 && !events.empty(); ++i) { isr(); }
        inInterrupt = false;
    }

    static EndpointState& state(std::size_t ep,
                                bool        in) {
        return endpoints[ep][in ? 1 : 0];
    }

    static void resetAll() {
        endpoints = {};
        events.clear();
        address               = 0;
        prepared              = false;
        connected             = false;
        maskDepth             = 0;
        violations            = 0;
        controlTransfersBegun = 0;
        addressHistory.clear();
    }

    // ---- the host's side of the wire ----------------------------------------------------------

    /// A SETUP token and its 8 bytes. A controller always takes it, whatever endpoint 0 is doing.
    static void hostSetup(Kvasir::USB::SetupPacket const& pkt) {
        events.push_back(Event{.kind = Event::Kind::setup, .setup = pkt});
    }

    /// An IN token. ACK: the packet the device had armed; it is gone from the controller and the
    /// device is told. NAK: nothing armed. STALL: the endpoint is halted.
    static InResult hostIn(std::size_t ep) {
        auto& e = state(ep, true);
        if(e.stalled) { return {.handshake = Handshake::stall}; }
        if(e.armedIn.empty() || e.cancelPending) { return {.handshake = Handshake::nak}; }
        auto&             t = e.armedIn.front();
        std::size_t const n = std::min(MaxPacket, t.size - t.sent);
        InResult          r{
          .handshake = Handshake::ack,
          .packet = Packet{.data = {t.data() + t.sent, t.data() + t.sent + n}, .data1 = e.data1}
        };
        t.sent += n;
        e.data1 = !e.data1;
        // The transfer is over with its last byte - or, a zero-length one, with its only packet.
        if(t.sent == t.size) {
            e.armedIn.pop_front();
            events.push_back(Event{.kind = Event::Kind::transferComplete, .ep = ep, .in = true});
        }
        return r;
    }

    /// An OUT token with its data. NAK while the endpoint is not armed: the host tries again.
    static Handshake hostOut(std::size_t                ep,
                             std::span<std::byte const> data) {
        auto& e = state(ep, false);
        if(e.stalled) {
            // What the SAM D21 does, against its data sheet ("the incoming data is discarded",
            // 32.6.2.7; seen on the bench 2026-09-20): a transfer that is still open takes the
            // packet and counts it, and the host gets its STALL all the same. A device that
            // leaves one armed while halted ends up with bytes nobody acknowledged.
            if(e.outInPlace != nullptr && e.armedOut
               && e.received.size() + data.size() <= *e.armedOut)
            {
                std::copy(data.begin(), data.end(), e.outInPlace + e.received.size());
                e.received.insert(e.received.end(), data.begin(), data.end());
            }
            return Handshake::stall;
        }
        if(!e.armedOut || e.cancelPending) { return Handshake::nak; }
        if(e.received.size() + data.size() > *e.armedOut && *e.armedOut != 0) { ++violations; }
        if(e.outInPlace != nullptr) {
            // As the SAM controller does it: every packet straight into the caller's memory, and
            // behind a short one its two CRC bytes, where they still fit the packet's 64 bytes.
            std::byte* const at = e.outInPlace + e.received.size();
            std::copy(data.begin(), data.end(), at);
            for(std::size_t i = data.size(); i < std::min(data.size() + 2, MaxPacket); ++i) {
                at[i] = std::byte{0xC5};
            }
        }
        e.received.insert(e.received.end(), data.begin(), data.end());
        e.data1 = !e.data1;
        // Over with a short packet, or when what was asked for has come.
        if(data.size() < MaxPacket || e.received.size() >= *e.armedOut) {
            e.hasReceived = true;
            e.armedOut.reset();
            events.push_back(Event{.kind = Event::Kind::transferComplete, .ep = ep, .in = false});
        }
        return Handshake::ack;
    }

    /// A bus reset: whatever was pending is gone, and the controller forgets its address.
    static void hostBusReset() {
        events.clear();
        for(auto& ep : endpoints) {
            for(auto& e : ep) { e.clear(); }
        }
        events.push_back(Event{.kind = Event::Kind::busReset});
    }

    /// The bus goes quiet (3 ms without traffic), and wakes up again.
    static void hostSuspend() { events.push_back(Event{.kind = Event::Kind::suspend}); }

    static void hostResume() { events.push_back(Event{.kind = Event::Kind::resume}); }

    static void hostStartOfFrame(std::uint16_t frame) {
        events.push_back(Event{.kind = Event::Kind::sof, .frame = frame});
    }

    // ---- the chip package's side: Backend.hpp --------------------------------------------------

    template<typename Clock, typename ConfigT>
    struct Backend {
        static constexpr std::size_t MaxPacketSize = MaxPacket;
        static constexpr std::size_t EndpointCount = Endpoints;
        static constexpr bool        AsyncCancel   = Async;

        /// No vector table on a host: the test calls Device::runInterrupt().
        template<auto Handler>
        struct Isr {
            static constexpr auto handler = Handler;
        };

        static void prepare() { prepared = true; }

        static void connect() {
            if(!prepared) { ++violations; }
            connected = true;
        }

        static void disconnect() { connected = false; }

        static void setAddress(std::uint8_t a) {
            address = a;
            addressHistory.push_back(a);
        }

        static void beginControlTransfer() {
            ++controlTransfersBegun;
            for(bool const in : {false, true}) {
                auto& e = state(0, in);
                e.armedIn.clear();
                e.armedOut.reset();
                e.hasReceived   = false;
                e.stalled       = false;
                e.cancelPending = false;
                e.data1         = true;
            }
        }

        static void maskInterrupt() {
            raiseIfPending();
            if(maskDepth != 0) { ++violations; }   // the device counts, the backend need not
            ++maskDepth;
        }

        static void unmaskInterrupt() {
            if(maskDepth != 1) { ++violations; }
            --maskDepth;
            raiseIfPending();
        }

        /// One interrupt: everything pending, in the order it happened - but a bus reset on its
        /// own, as Backend.hpp says.
        template<typename Sink>
        static void dispatchEvents() {
            while(!events.empty()) {
                Event const ev = events.front();
                events.pop_front();
                switch(ev.kind) {
                case Event::Kind::busReset:         Sink::busReset(); return;
                case Event::Kind::setup:            Sink::setup(ev.setup); break;
                case Event::Kind::transferComplete: Sink::transferComplete(ev.ep, ev.in); break;
                case Event::Kind::cancelComplete:
                    if constexpr(Async) { Sink::cancelComplete(ev.ep, ev.in); }
                    break;
                case Event::Kind::suspend: Sink::suspend(); break;
                case Event::Kind::resume:  Sink::resume(); break;
                case Event::Kind::sof:
                    if constexpr(Kvasir::USB::ConfigTraits<ConfigT>::UseSof) {
                        Sink::startOfFrame(ev.frame);
                    }
                    break;
                }
            }
        }

        template<std::size_t                       N,
                 Kvasir::USB::EndpointDirection    Dir,
                 Kvasir::USB::EndpointTransferType Type>
        struct Endpoint {
            static constexpr bool IsIn = Dir == Kvasir::USB::EndpointDirection::In;

            static constexpr std::size_t QueueDepth  = IsIn && N != 0 ? Depth : 1;
            static constexpr bool        AsyncCancel = Async;
            // Endpoint 0 stays at a packet, as on a real controller of this kind.
            static constexpr std::size_t MaxTransfer = N != 0 ? Transfer : MaxPacket;

            using FreeBuffers = std::array<bool, QueueDepth>;

            static EndpointState& self() { return state(N, IsIn); }

            static std::size_t armedBuffers() {
                return IsIn ? self().armedIn.size() : (self().armedOut ? 1U : 0U);
            }

            static FreeBuffers freeBuffers() {
                FreeBuffers       free{};
                std::size_t const armed = armedBuffers();
                for(std::size_t i = 0; i != QueueDepth; ++i) { free[i] = i >= armed; }
                return free;
            }

            static void setupEndpoint() {
                self()      = EndpointState{};
                self().set  = true;
                self().type = Type;
            }

            template<bool Last>
            static bool tryTransfer(std::span<std::byte const> data) {
                return tryTransfer<Last>(data, freeBuffers());
            }

            template<bool Last>
            static bool tryTransfer(std::span<std::byte const> data,
                                    FreeBuffers const&) {
                static_assert(IsIn);
                auto& e = self();
                if(!e.set || data.size() > MaxPacket) { ++violations; }
                if(e.armedIn.size() >= QueueDepth || e.cancelPending) { return false; }
                e.armedIn.push_back(ArmedIn{
                  .copy = {data.begin(), data.end()},
                  .size = data.size()
                });
                return true;
            }

            // The caller's memory, read when the host asks: it has to stay as it is until then.
            template<bool Last>
            static bool tryTransferInPlace(std::span<std::byte const> data,
                                           FreeBuffers const&)
                requires(InPlace && N != 0)
            {
                static_assert(IsIn);
                auto& e = self();
                if(!e.set || data.size() > MaxTransfer) { ++violations; }
                if(e.armedIn.size() >= QueueDepth || e.cancelPending) { return false; }
                e.armedIn.push_back(ArmedIn{.inPlace = data.data(), .size = data.size()});
                return true;
            }

            static std::size_t sentOfCancelled()
                requires(InPlace && N != 0)
            {
                return self().sentOfCancelled;
            }

            template<bool Last = false>
            static bool armReceive(std::size_t maxSize) {
                static_assert(!IsIn);
                auto& e = self();
                if(!e.set || maxSize > MaxPacket) { ++violations; }
                if(e.armedOut || e.cancelPending) { return false; }
                e.armedOut    = maxSize;
                e.outInPlace  = nullptr;
                e.hasReceived = false;
                e.received.clear();
                return true;
            }

            // A whole transfer into the caller's memory, packet by packet as they come; only
            // where cancelling is done on return.
            static constexpr std::size_t MaxReceive = Transfer;

            static bool armReceiveInto(std::span<std::byte> dest)
                requires(InPlace && !Async && N != 0)
            {
                static_assert(!IsIn);
                auto& e = self();
                if(!e.set || dest.empty() || dest.size() > MaxReceive
                   || dest.size() % MaxPacket != 0
                   || reinterpret_cast<std::uintptr_t>(dest.data()) % 4 != 0)
                {
                    ++violations;
                }
                // Like the SAM controller: not while a finished transfer is still to be reported,
                // whose bytes the new one would overwrite.
                bool const unreported = std::ranges::any_of(events, [](Event const& ev) {
                    return ev.kind == Event::Kind::transferComplete && ev.ep == N && !ev.in;
                });
                if(e.armedOut || unreported) { return false; }
                e.armedOut    = dest.size();
                e.outInPlace  = dest.data();
                e.hasReceived = false;
                e.received.clear();
                return true;
            }

            static std::size_t received()
                requires(InPlace && !Async && N != 0)
            {
                if(!self().hasReceived) { ++violations; }
                return self().received.size();
            }

            static std::size_t receivedSoFar()
                requires(InPlace && !Async && N != 0)
            {
                return self().armedOut ? self().received.size() : 0;
            }

            static std::optional<std::size_t> takeBackReceive()
                requires(InPlace && !Async && N != 0)
            {
                auto& e = self();
                if(!e.armedOut) { return std::nullopt; }   // completed: its event is on its way
                e.armedOut.reset();
                e.outInPlace = nullptr;
                return e.received.size();
            }

            static std::size_t readCurrentBuffer(std::span<std::byte> dest) {
                static_assert(!IsIn);
                auto& e = self();
                if(!e.hasReceived) { return 0; }
                std::size_t const n = std::min(dest.size(), e.received.size());
                std::copy_n(e.received.begin(), n, dest.begin());
                return n;
            }

            static void stall() { self().stalled = true; }

            static void clearStall() { self().stalled = false; }

            static void resetDataToggle() { self().data1 = false; }

            static void reset() { self().data1 = false; }

            static std::size_t takeBack() {
                auto&             e = self();
                std::size_t const n = armedBuffers();
                e.sentOfCancelled   = e.armedIn.empty() ? 0 : e.armedIn.front().sent;
                e.armedIn.clear();
                e.armedOut.reset();
                return n;
            }

            static auto cancel() {
                if constexpr(Async) {
                    if(!self().cancelPending) {
                        self().cancelPending = true;
                        events.push_back(
                          Event{.kind = Event::Kind::cancelComplete, .ep = N, .in = IsIn});
                    }
                } else {
                    return takeBack();
                }
            }

            static std::size_t cancelComplete() {
                static_assert(Async);
                if(!self().cancelPending) { ++violations; }
                self().cancelPending = false;
                return takeBack();
            }
        };
    };
};
}   // namespace Kvasir::Test
