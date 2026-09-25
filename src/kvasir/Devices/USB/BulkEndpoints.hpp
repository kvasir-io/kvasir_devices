#pragma once

#include "../Log.hpp"
#include "Backend.hpp"
#include "Descriptors.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <functional>
#include <kvasir/Atomic/Queue.hpp>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

namespace Kvasir::USB::detail {

// Standard requests addressed to one endpoint: GET_STATUS and SET/CLEAR_FEATURE(ENDPOINT_HALT).
// Endpoint provides Address, halted, halt() and clearHalt().
template<typename Derived,
         typename Endpoint>
bool handleEndpointRequest(SetupPacket const& pkt) {
    using Direction = SetupPacket::Direction;
    using Request   = SetupPacket::Request;

    if(pkt.type() != SetupPacket::Type::standard
       || pkt.recipient() != SetupPacket::Recipient::endpoint || pkt.wIndex != Endpoint::Address)
    {
        return false;
    }
    switch(pkt.bRequest) {
    case Request::getStatus:
        {
            if(pkt.direction() != Direction::deviceToHost) { return false; }
            std::array<std::byte, 2> const status{static_cast<std::byte>(Endpoint::halted)};
            return Derived::ep0INDataPhase(status, pkt.wLength);
        }
    case Request::setFeature:
    case Request::clearFeature:
        {
            if(pkt.direction() != Direction::hostToDevice || pkt.wValue != 0 /* ENDPOINT_HALT */) {
                return false;
            }
            if(pkt.bRequest == Request::setFeature) {
                Endpoint::halt();
            } else {
                Endpoint::clearHalt();
            }
            Derived::acknowledgeSetupRequest();
            using namespace std::string_view_literals;
            UC_LOG_I("USB: EP{:#04x} halt {}",
                     Endpoint::Address,
                     pkt.bRequest == Request::setFeature ? "set"sv : "cleared"sv);
            return true;
        }
    default: return false;
    }
}

// SET_INTERFACE on one interface: alternate setting 0 is the only one, and selecting it resets the
// interface's endpoints (restart).
template<typename Derived,
         typename Restart>
bool handleSetInterface(SetupPacket const& pkt,
                        std::size_t        interfaceNumber,
                        Restart            restart) {
    bool const toInterface = pkt.recipient() == SetupPacket::Recipient::interface;
    if(pkt.type() != SetupPacket::Type::standard || !toInterface
       || pkt.direction() != SetupPacket::Direction::hostToDevice
       || pkt.bRequest != SetupPacket::Request::setInterface || pkt.wIndex != interfaceNumber
       || pkt.wValue != 0)
    {
        return false;
    }
    restart();
    Derived::acknowledgeSetupRequest();
    return true;
}

// How much one arm of an endpoint may carry: a packet, unless the backend says more
// (Endpoint::MaxTransfer, Backend.hpp). A whole number of packets.
template<typename EP>
inline constexpr std::size_t MaxTransferOf = [] {
    if constexpr(requires { EP::MaxTransfer; }) {
        return static_cast<std::size_t>(EP::MaxTransfer);
    } else {
        return MaxPacketSize;
    }
}();

// A size the host sees as "more follows": a whole number of full packets.
constexpr bool endsOnPacketBoundary(std::size_t size) {
    return size != 0 && size % MaxPacketSize == 0;
}

// The IN half of a bulk function.
//
//   Framed = true    send() is one message. A message that is empty or fills its last packet is
//                    followed by a zero-length packet, so the host's read returns exactly that
//                    message. Message lengths are queued next to the ring, so the next message can
//                    be written while the current one is going out.
//   Framed = false   a byte stream; flush() ends the host's transfer.
//
// The application fills the ring, the interrupt empties it (single producer, single consumer).
// As many packets as the controller takes at once (EP::QueueDepth) are kept armed, so where that
// is more than one the host never has to wait for the interrupt between two packets.
//
// Stopping takes the endpoint's packets back and resets its data toggle; sending resumes once the
// controller has let go of them - at once, or in abortDone() where cancelling is a request
// (EP::AsyncCancel). A halt clear hands the unsent packets over again and keeps the ring: nothing
// lost, nothing repeated. SET_CONFIGURATION, SET_INTERFACE and a bus reset drop everything.
template<typename Derived,
         std::size_t EndpointNumber,
         std::size_t MaxMessageSize,
         bool        Framed,
         std::size_t RingSize_ = 0>
struct BulkInEndpoint {
    using EP
      = EndpointOf<Derived, EndpointNumber, EndpointDirection::In, EndpointTransferType::Bulk>;

    static constexpr std::uint8_t Address
      = makeEndpointAddress(EndpointDirection::In, static_cast<std::uint8_t>(EndpointNumber));

    // Framed: room for one message being sent and one being written - unless the config says
    // otherwise (Config::sendRingSize), down to one message: send() then waits for the last one.
    static constexpr std::size_t RingSize
      = RingSize_ != 0 ? RingSize_ : (Framed ? 2 * MaxMessageSize + 1 : MaxMessageSize);
    static_assert(RingSize > MaxMessageSize || !Framed,
                  "the send ring has to hold a whole message (a ring of N holds N - 1 bytes)");

    // What one arm carries: a packet on most controllers, a run of them where the controller
    // splits a transfer into packets itself.
    static constexpr std::size_t TransferSize = MaxTransferOf<EP>;
    static_assert(TransferSize % MaxPacketSize == 0 && TransferSize != 0);

    // How many messages may be queued at once.
    static constexpr std::size_t MaxQueuedMessages = 4;

    // How many packets may be with the controller.
    static constexpr std::uint8_t BuffersPerEndpoint = static_cast<std::uint8_t>(EP::QueueDepth);

    using RingType  = Kvasir::Atomic::Queue<std::byte, RingSize>;
    using SizeQueue = Kvasir::Atomic::
      Queue<std::uint32_t, MaxQueuedMessages + 1, Kvasir::Atomic::OverFlowPolicyIgnore>;

    static inline RingType ring{};

    // One staging buffer per packet with the controller: it stays there until it is sent, for a
    // replay.
    // Word aligned: a backend that takes a transfer in place lets its controller read from here.
    alignas(4) static inline std::array<std::array<std::byte,
                                                   TransferSize>,
                                        BuffersPerEndpoint> packets{};
    static inline std::array<std::uint16_t, BuffersPerEndpoint> packetSizes{};
    // How much of a slot went out before a cancel took the rest back (only a controller that
    // splits a transfer into packets ever sends part of one).
    static inline std::array<std::uint16_t, BuffersPerEndpoint> packetOffsets{};
    static inline std::uint8_t nextSlot{};   // where the next packet is put together

    // The packet put together in packets[nextSlot] and not yet handed over.
    static inline std::size_t stagedSize{};
    static inline bool        stagedEndsMessage{false};
    static inline bool        halted{false};
    static inline bool        restartPending{false};
    static inline bool        replayUnsent{false};   // for abortDone(): hand the unsent over again

    // Byte stream only: flush() was called, and whether the last packet was full - only then
    // does the host need a zero-length packet to see the end.
    static inline bool endOfTransferPending{false};
    static inline bool lastPacketFull{false};

    // Framed only: the length of each queued message, and what is left of the one going out.
    // The message stays open until the packet that ends the host's transfer is handed over.
    static inline std::conditional_t<Framed, SizeQueue, std::monostate> messageSizes{};
    static inline std::uint32_t                                         messageBytesLeft{};
    static inline bool                                                  messageOpen{false};

    // Hands the next packet to the controller. Runs in the interrupt, or from the application with
    // the interrupt masked.
    static void armNext() {
        if(halted || restartPending || !Derived::isConfigured()) { return; }
        // Asked of the controller, not counted: two completions can raise a single interrupt.
        auto const free = EP::freeBuffers();
        if(std::ranges::none_of(free, std::identity{})) { return; }
        if(stagedSize == 0) {
            if constexpr(Framed) {
                if(!messageOpen) {
                    if(!messageSizes.pop_into(messageBytesLeft)) { return; }
                    messageOpen = true;
                }
            }
            std::size_t size = std::min(ring.size(), TransferSize);
            if constexpr(Framed) { size = std::min<std::size_t>(size, messageBytesLeft); }
            if(size == 0) {
                endTransfer(free);
                return;
            }
            std::span<std::byte> chunk{packets[nextSlot].data(), size};
            if(!ring.pop_into(chunk)) { return; }
            stagedSize = size;
            if constexpr(Framed) {
                messageBytesLeft -= static_cast<std::uint32_t>(size);
                stagedEndsMessage = messageBytesLeft == 0;
            }
        }
        if(handOver(stagedSize, free)) {
            if constexpr(Framed) {
                // A full last packet leaves the message open until a zero-length packet follows.
                if(stagedEndsMessage && !endsOnPacketBoundary(stagedSize)) { messageOpen = false; }
            } else {
                lastPacketFull = endsOnPacketBoundary(stagedSize);
            }
            stagedSize = 0;
            // The controller has its copy, so the slot is free for the next packet.
            armNext();
        }
    }

    // With nothing left to send: the host sees the end of a transfer in a short packet, so an
    // empty message, or a transfer whose last packet was full, takes a zero-length packet.
    static void endTransfer(typename EP::FreeBuffers const& free) {
        if constexpr(Framed) {
            if(messageOpen && handOver(0, free)) {
                messageOpen = false;
                armNext();
            }
        } else {
            if(!endOfTransferPending) { return; }
            if(lastPacketFull && !handOver(0, free)) { return; }
            endOfTransferPending = false;
            lastPacketFull       = false;
        }
    }

    // Hands what is in the current slot to the controller and moves on to the other slot.
    // In place where the backend can (the slot stays as it is until the transfer is over or taken
    // back, which is what tryTransferInPlace asks), copied otherwise.
    static bool arm(std::span<std::byte const>      data,
                    typename EP::FreeBuffers const& free) {
        if constexpr(requires { EP::template tryTransferInPlace<true>(data, free); }) {
            return EP::template tryTransferInPlace<true>(data, free);
        } else {
            return EP::template tryTransfer<true>(data, free);
        }
    }

    static bool handOver(std::size_t                     size,
                         typename EP::FreeBuffers const& free) {
        if(!arm(std::span{packets[nextSlot]}.first(size), free)) { return false; }
        packetSizes[nextSlot]   = static_cast<std::uint16_t>(size);
        packetOffsets[nextSlot] = 0;
        nextSlot                = static_cast<std::uint8_t>((nextSlot + 1) % BuffersPerEndpoint);
        return true;
    }

    // The transfers an abort took back, in the order they were armed. Of the first, a controller
    // that splits transfers into packets may already have sent some: those are not sent again.
    static void handOverAgain(std::uint8_t count,
                              std::size_t  alreadySent) {
        for(std::uint8_t i = 0; i != count; ++i) {
            std::uint8_t const slot = static_cast<std::uint8_t>(
              (nextSlot + BuffersPerEndpoint - count + i) % BuffersPerEndpoint);
            // What an earlier cancel found sent stays sent.
            std::size_t const skip
              = std::min<std::size_t>(packetOffsets[slot] + (i == 0 ? alreadySent : 0),
                                      packetSizes[slot]);
            packetOffsets[slot] = static_cast<std::uint16_t>(skip);
            if(!arm(std::span{packets[slot]}.first(packetSizes[slot]).subspan(skip),
                    EP::freeBuffers()))
            {
                return;
            }
        }
    }

    static void bufferDone() { armNext(); }

    // Consumer-side flush: safe against the application filling the ring.
    static void dropQueued() {
        std::byte discard{};
        while(ring.pop_into(discard)) {}
        if constexpr(Framed) {
            std::uint32_t size{};
            while(messageSizes.pop_into(size)) {}
            messageOpen      = false;
            messageBytesLeft = 0;
        }
        stagedSize           = 0;
        endOfTransferPending = false;
        lastPacketFull       = false;
    }

    // Takes the endpoint's packets back from the controller.
    static void abortTransfers(bool keepQueued) {
        restartPending = true;
        replayUnsent   = keepQueued;
        if constexpr(EP::AsyncCancel) {
            EP::cancel();
            if(!keepQueued) { dropQueued(); }
        } else {
            auto const unsent = EP::cancel();
            if(!keepQueued) { dropQueued(); }
            finishCancel(unsent);
        }
    }

    // SET_CONFIGURATION and SET_INTERFACE: everything in flight is dropped, and a halt ends - "the
    // Halt feature is reset to zero after either a SetConfiguration() or SetInterface() request
    // even if the requested configuration or interface is the same" (USB 2.0, 9.4.5).
    static void stop() {
        if(std::exchange(halted, false)) { EP::clearStall(); }
        abortTransfers(false);
    }

    static void busReset() {
        halted         = false;
        restartPending = false;
        replayUnsent   = false;
        dropQueued();
        static_cast<void>(EP::cancel());
        EP::reset();
    }

    // The controller has let go: unsent is how many armed packets it never sent.
    static void finishCancel(std::size_t unsent) {
        std::size_t const alreadySent = [] {
            if constexpr(requires { EP::sentOfCancelled(); }) {
                return EP::sentOfCancelled();
            } else {
                return std::size_t{0};
            }
        }();
        EP::resetDataToggle();
        auto const replay
          = static_cast<std::uint8_t>(std::exchange(replayUnsent, false) ? unsent : 0);
        if(std::exchange(restartPending, false) && Derived::isConfigured() && !halted) {
            handOverAgain(replay, alreadySent);
            armNext();
        }
    }

    // Only where cancelling is a request (EP::AsyncCancel): the controller's answer.
    static void abortDone() {
        if constexpr(EP::AsyncCancel) { finishCancel(EP::cancelComplete()); }
    }

    static void halt() {
        halted = true;
        EP::stall();
    }

    static void clearHalt() {
        halted = false;
        EP::clearStall();
        abortTransfers(true);
    }

    // ---- what the application calls ----------------------------------------------------------

    static std::size_t writeAvailable() { return ring.max_size() - ring.size(); }

    // A byte stream: takes as much as the ring has room for and returns how much that was.
    static std::size_t write(std::span<std::byte const> data)
        requires(!Framed)
    {
        auto const chunk = data.first(std::min(writeAvailable(), data.size()));
        if(!chunk.empty()) { ring.push(chunk); }
        Derived::withIsrMasked([] { armNext(); });
        return chunk.size();
    }

    // Ends the host's current transfer once everything written so far has gone out.
    static void flush()
        requires(!Framed)
    {
        Derived::withIsrMasked([] {
            endOfTransferPending = true;
            armNext();
        });
    }

    // Whether a whole message fits right now.
    static bool isSendReady() {
        if(halted || !Derived::isConfigured()) { return false; }
        if constexpr(Framed) {
            return writeAvailable() >= MaxMessageSize && messageSizes.size() < MaxQueuedMessages;
        } else {
            return writeAvailable() != 0;
        }
    }

    // One message, all of it or none of it.
    static bool send(std::span<std::byte const> data) {
        if(data.size() > MaxMessageSize) { return false; }
        return Derived::withIsrMasked([&] {
            if constexpr(Framed) {
                if(writeAvailable() < data.size() || messageSizes.size() >= MaxQueuedMessages) {
                    return false;
                }
                if(!data.empty()) { ring.push(data); }
                // The bytes are in the ring before the interrupt learns of the message.
                messageSizes.push(static_cast<std::uint32_t>(data.size()));
            } else {
                if(writeAvailable() < data.size()) { return false; }
                if(!data.empty()) { ring.push(data); }
            }
            armNext();
            return true;
        });
    }
};

// The OUT half of a bulk function: a receive queue with back-pressure. The endpoint is armed only
// while the queue has room for a packet; otherwise the host is NAKed until the application reads.
//
// A halt clear keeps the queue: those bytes were acknowledged. A reconfiguration or a bus reset
// empties it, but the interrupt (the producer) only requests that; the reading side does it, and
// the endpoint is not armed again before.
template<typename Clock, typename Derived, std::size_t EndpointNumber, std::size_t RecvBufferSize>
struct BulkOutEndpoint {
    using EP
      = EndpointOf<Derived, EndpointNumber, EndpointDirection::Out, EndpointTransferType::Bulk>;

    static constexpr std::uint8_t Address
      = makeEndpointAddress(EndpointDirection::Out, static_cast<std::uint8_t>(EndpointNumber));

    using QueueType = Kvasir::Atomic::Queue<std::byte, RecvBufferSize>;

    // Where the controller gathers the packets of a transfer by itself (Backend.hpp,
    // armReceiveInto) it gets a whole transfer's room at a time: one interrupt and one copy into
    // the queue for up to MaxReceive bytes, instead of one of each per packet.
    //
    // Such a transfer only completes with a short packet or when it is full, so a host that
    // writes exactly one full packet (cdc_acm does, without a zero-length one) would wait for its
    // echo forever, and what was acknowledged into a half-filled transfer would be lost to a halt
    // clear (found by the host tests, before any hardware). Hence:
    //   - the reading side takes a transfer back that holds bytes and has not grown for
    //     StalledAfter, with what it holds (service());
    //   - after that the endpoint arms packet by packet, where nothing can wait, until
    //     FullPacketsForTransfers full packets in a row say a stream is coming again;
    //   - a halt clear takes what the transfer holds into the queue before it cancels.
    static constexpr bool ReceivesInPlace
      = !EP::AsyncCancel && requires(std::span<std::byte> dest) {
            { EP::armReceiveInto(dest) } -> std::same_as<bool>;
            { EP::received() } -> std::convertible_to<std::size_t>;
            { EP::receivedSoFar() } -> std::convertible_to<std::size_t>;
            { EP::takeBackReceive() } -> std::same_as<std::optional<std::size_t>>;
        };

    static constexpr std::size_t MaxReceive = [] {
        if constexpr(ReceivesInPlace) {
            return static_cast<std::size_t>(EP::MaxReceive);
        } else {
            return MaxPacketSize;
        }
    }();
    static_assert(MaxReceive % MaxPacketSize == 0 && MaxReceive != 0);

    // Short on purpose: taking a transfer back too early costs nothing but a few packets armed
    // one by one, while every message of whole packets waits this long for its last ones.
    // Measured on the SAM D21 (usb_playground, 2026-09-20, MiB/s host to device / echo of 1 KiB
    // messages): 2 ms 0.847 / 0.218, 700 us 0.845 / 0.318, 300 us 0.774 / 0.348; packet by
    // packet it was 0.486 / 0.341.
    static constexpr auto         StalledAfter            = std::chrono::microseconds{700};
    static constexpr std::uint8_t FullPacketsForTransfers = 4;

    // Word aligned, the controller's while a transfer is armed into it. Empty where packets are
    // read out of the controller one by one.
    alignas(4) static inline std::array<std::byte,
                                        ReceivesInPlace ? MaxReceive : 0> staging{};
    // Of the transfer armed into staging, else 0. The interrupt's, and looked at by the reading
    // side without it masked (takeBackStalled()).
    static inline std::atomic<std::size_t> armedSize{};
    static inline std::uint8_t packetArmsLeft{};   // full packets to see before transfers again
    // The reading side's alone: what the armed transfer held when it last looked, and when that
    // changed. A new transfer that happens to hold as much is taken back early - harmless, the
    // bytes are only handed up sooner.
    static inline std::size_t                seenSoFar{};
    static inline typename Clock::time_point seenAt{};

    static inline QueueType         queue{};
    static inline std::atomic<bool> paused{false};
    static inline std::atomic<bool> flushPending{false};
    static inline bool              halted{false};
    static inline bool              restartPending{false};

    static bool hasRoom() { return queue.max_size() - queue.size() >= MaxPacketSize; }

    static void arm() {
        if(!flushPending && hasRoom()) {
            paused = false;
            if constexpr(ReceivesInPlace) {
                // As much as the queue can take afterwards, in whole packets.
                std::size_t const room = queue.max_size() - queue.size();
                std::size_t const size = packetArmsLeft != 0
                                         ? MaxPacketSize
                                         : std::min(MaxReceive, room - room % MaxPacketSize);
                if(EP::armReceiveInto(std::span{staging}.first(size))) { armedSize = size; }
            } else {
                EP::armReceive(MaxPacketSize);
            }
        } else {
            paused = true;
        }
    }

    // What the armed transfer holds goes into the queue (acknowledged bytes) or is dropped with
    // it. Interrupt masked, or in the interrupt.
    static void takeBack(bool keep) {
        if constexpr(ReceivesInPlace) {
            if(armedSize == 0) { return; }
            auto const held = EP::takeBackReceive();
            // Completed meanwhile: that is bufferDone()'s to take, with its completion.
            if(!held) { return; }
            armedSize = 0;
            if(keep) {
                // It fits: arm() sized the transfer by the queue's room, and the reading side only
                // makes more.
                assert(*held <= queue.max_size() - queue.size());
                queue.push(std::span{staging}.first(*held));
            }
        }
    }

    static void abortTransfers(bool keepQueued) {
        restartPending = true;
        paused         = false;
        if(!keepQueued) { flushPending = true; }
        takeBack(keepQueued);
        static_cast<void>(EP::cancel());
        if constexpr(!EP::AsyncCancel) { finishCancel(); }
    }

    // SET_CONFIGURATION and SET_INTERFACE: everything in flight is dropped, and a halt ends - "the
    // Halt feature is reset to zero after either a SetConfiguration() or SetInterface() request
    // even if the requested configuration or interface is the same" (USB 2.0, 9.4.5).
    static void stop() {
        if(std::exchange(halted, false)) { EP::clearStall(); }
        abortTransfers(false);
    }

    static void busReset() {
        halted         = false;
        paused         = false;
        restartPending = false;
        flushPending   = true;
        takeBack(false);
        static_cast<void>(EP::cancel());
        EP::reset();
    }

    // The controller has let go; only now is the data toggle the endpoint's to set.
    static void finishCancel() {
        EP::resetDataToggle();
        if(std::exchange(restartPending, false) && Derived::isConfigured() && !halted) { arm(); }
    }

    // Only where cancelling is a request (EP::AsyncCancel): the controller's answer.
    static void abortDone() {
        if constexpr(EP::AsyncCancel) {
            static_cast<void>(EP::cancelComplete());
            finishCancel();
        }
    }

    // Nothing stays armed through a halt: the SAM D21 answers STALL and still writes the packet
    // into a transfer that is open, and counts it (seen on the bench, 2026-09-20, against its
    // data sheet) - bytes the host was told were refused. What came before the halt is kept.
    static void halt() {
        halted = true;
        takeBack(true);
        EP::stall();
    }

    static void clearHalt() {
        halted = false;
        EP::clearStall();
        abortTransfers(true);
    }

    static void bufferDone() {
        if constexpr(ReceivesInPlace) {
            // A completion of a transfer that was taken back meanwhile has nothing to add.
            if(armedSize != 0) {
                armedSize             = 0;
                std::size_t const len = std::min<std::size_t>(EP::received(), staging.size());
                queue.push(std::span{staging}.first(len));
                if(packetArmsLeft != 0 && len == MaxPacketSize) { --packetArmsLeft; }
            }
            // A transfer that completed just as the halt came: the halt clear arms again.
            if(halted) { return; }
        } else {
            std::array<std::byte, MaxPacketSize> tempBuffer{};
            std::size_t const                    len = EP::readCurrentBuffer(tempBuffer);
            queue.push(std::span{tempBuffer.data(), len});
        }
        arm();
    }

    // The reading side: a transfer that holds bytes and has stopped growing is what a host leaves
    // behind that wrote whole packets and no short one. Looked at without the interrupt masked -
    // a hint - and taken back with it masked.
    static void takeBackStalled() {
        if constexpr(ReceivesInPlace) {
            if(armedSize <= MaxPacketSize) { return; }
            std::size_t const soFar = EP::receivedSoFar();
            auto const        now   = Clock::now();
            if(soFar != seenSoFar) {
                seenSoFar = soFar;
                seenAt    = now;
                return;
            }
            if(soFar == 0 || now - seenAt < StalledAfter) { return; }
            Derived::withIsrMasked([] {
                if(armedSize == 0 || restartPending || flushPending) { return; }
                // While halted too: what the transfer holds was acknowledged before the halt,
                // and the queue's bytes stay readable through one. The halt clear arms again.
                takeBack(true);
                packetArmsLeft = FullPacketsForTransfers;
                if(!halted) { arm(); }
            });
            seenSoFar = 0;
        }
    }

    // The reading side, before every access: performs a requested flush, and arms a paused
    // endpoint again once there is room.
    static void service() {
        if(flushPending.load()) {
            std::byte discard{};
            while(queue.pop_into(discard)) {}
            flushPending = false;
        }
        takeBackStalled();
        if(!paused.load()) { return; }
        Derived::withIsrMasked([] {
            if(paused && !halted && !restartPending && !flushPending && Derived::isConfigured()
               && hasRoom())
            {
                arm();
            }
        });
    }
};

// What getRecvBuffer() hands out: the receive queue's reading side (a single consumer), which
// flushes the queue when the endpoint was stopped and lets a paused endpoint take data again.
template<typename Out>
struct RecvQueue {
    bool pop_into(std::byte& out) {
        Out::service();
        bool const got = Out::queue.pop_into(out);
        if(got) { Out::service(); }
        return got;
    }

    template<typename Range>
    bool pop_into(Range& range) {
        Out::service();
        bool const got = Out::queue.pop_into(range);
        if(got) { Out::service(); }
        return got;
    }

    std::size_t size() const {
        Out::service();
        return Out::queue.size();
    }

    bool empty() const { return size() == 0; }

    constexpr std::size_t max_size() const { return Out::queue.max_size(); }
};

// A bulk function: BulkInEndpoint, plus BulkOutEndpoint when Bidirectional.
// Config::SendBufferSize sizes the send ring (framed: the largest message),
// Config::RecvBufferSize the receive queue.
//
// A device with several bulk functions sizes each by its interface number instead, where RAM is
// short - any of these, the plain member or 4096 where one is missing:
//
//     static constexpr std::size_t sendBufferSize(std::size_t interfaceNumber);
//     static constexpr std::size_t recvBufferSize(std::size_t interfaceNumber);
//     static constexpr std::size_t sendRingSize(std::size_t interfaceNumber);   // framed: the ring,
//         // by default room for two messages; at least one message + 1
template<typename Clock,
         typename Config,
         typename Derived,
         typename Mixin,
         std::size_t InterfaceNumber,
         std::size_t EndpointNumber,
         bool        Framed,
         bool        Bidirectional>
struct BulkDataAdapter {
private:
    friend Mixin;

    static constexpr std::size_t SendBufferSize = [] {
        if constexpr(requires { Config::sendBufferSize(InterfaceNumber); }) {
            return static_cast<std::size_t>(Config::sendBufferSize(InterfaceNumber));
        } else if constexpr(requires { Config::SendBufferSize; }) {
            return static_cast<std::size_t>(Config::SendBufferSize);
        } else {
            return std::size_t{4096};
        }
    }();

    static constexpr std::size_t RecvBufferSize = [] {
        if constexpr(requires { Config::recvBufferSize(InterfaceNumber); }) {
            return static_cast<std::size_t>(Config::recvBufferSize(InterfaceNumber));
        } else if constexpr(requires { Config::RecvBufferSize; }) {
            return static_cast<std::size_t>(Config::RecvBufferSize);
        } else {
            return std::size_t{4096};
        }
    }();

    static constexpr std::size_t SendRingSize = [] {
        if constexpr(requires { Config::sendRingSize(InterfaceNumber); }) {
            return static_cast<std::size_t>(Config::sendRingSize(InterfaceNumber));
        } else {
            return std::size_t{0};   // BulkInEndpoint's default
        }
    }();

    using In  = BulkInEndpoint<Derived, EndpointNumber, SendBufferSize, Framed, SendRingSize>;
    using Out = BulkOutEndpoint<Clock,
                                Derived,
                                EndpointNumber,
                                Bidirectional ? RecvBufferSize : std::size_t{2}>;

    static inline RecvQueue<Out> recvQueue{};

    // Callbacks (called by MixinBases)
    static bool EndpointHandlerCallback(std::size_t epNum,
                                        bool        in) {
        if(epNum != EndpointNumber) { return false; }
        if(in) {
            In::bufferDone();
            return true;
        }
        if constexpr(Bidirectional) {
            Out::bufferDone();
            return true;
        }
        return false;
    }

    static bool AbortDoneCallback(std::size_t epNum,
                                  bool        in) {
        if(epNum != EndpointNumber) { return false; }
        if(in) {
            In::abortDone();
            return true;
        }
        if constexpr(Bidirectional) {
            Out::abortDone();
            return true;
        }
        return false;
    }

    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        if(handleEndpointRequest<Derived, In>(pkt)) { return true; }
        if constexpr(Bidirectional) { return handleEndpointRequest<Derived, Out>(pkt); }
        return false;
    }

    static void ResetCallback() {
        In::busReset();
        if constexpr(Bidirectional) { Out::busReset(); }
    }

    // SET_CONFIGURATION resets the data toggles whatever the value.
    static void ConfiguredCallback(std::uint8_t) { restart(); }

    static void SetupEndpointsCallback() {
        In::EP::setupEndpoint();
        if constexpr(Bidirectional) { Out::EP::setupEndpoint(); }
    }

public:
    // SET_INTERFACE on the owning interface: the same as a reconfiguration.
    static void restart() {
        In::stop();
        if constexpr(Bidirectional) { Out::stop(); }
    }

    // ---- what the application calls ----------------------------------------------------------

    /// Whether send() would take a whole message (framed) or anything at all (stream).
    static bool isSendReady() { return In::isSendReady(); }

    /// One message (framed) or a block of the stream: all of it or none of it.
    static bool send(std::span<std::byte const> data) { return In::send(data); }

    /// Byte stream only: takes as much as there is room for and says how much that was.
    static std::size_t write(std::span<std::byte const> data)
        requires(!Framed)
    {
        return In::write(data);
    }

    static std::size_t writeAvailable() { return In::writeAvailable(); }

    /// Byte stream only: ends the host's current transfer once everything written so far has gone
    /// out. A framed endpoint does that at the end of every message by itself.
    static void flush()
        requires(!Framed)
    {
        In::flush();
    }

    static auto& getRecvBuffer()
        requires Bidirectional
    {
        return recvQueue;
    }

    /// Where the IN side stands, for diagnostics.
    struct SendDiagnostics {
        std::uint32_t queuedBytes;   // written, still waiting in the ring
        std::uint16_t heldBytes;     // taken out of the ring, waiting for a free buffer
        bool          armed;         // a packet is with the controller
        bool          restarting;    // waiting for an abort to finish
        bool          halted;
    };

    static SendDiagnostics sendDiagnostics() {
        return SendDiagnostics{.queuedBytes = static_cast<std::uint32_t>(In::ring.size()),
                               .heldBytes   = static_cast<std::uint16_t>(In::stagedSize),
                               .armed       = In::EP::armedBuffers() != 0,
                               .restarting  = In::restartPending,
                               .halted      = In::halted};
    }
};

}   // namespace Kvasir::USB::detail
