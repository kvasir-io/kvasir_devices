// A random host and a random application against a model of what has to come out: messages on a
// framed bulk IN endpoint arrive whole, in order, once; bytes written to the OUT endpoint are read
// back in order, once - across halts, halt clears, SET_INTERFACE, SET_CONFIGURATION and bus
// resets at any point, which may only ever drop what the device is allowed to drop. Against every
// kind of controller, with the interrupt raised at the test's pace and at hardware's (at the
// edges of every masked section).
#include "UsbHost.hpp"
//
#include <chrono>
#include <deque>
#include <kvasir/Devices/USB/SimpleBulk.hpp>
#include <random>

using namespace Kvasir::Test;

namespace {
struct Config {
    static constexpr auto        ManufacturerString = "Kvasir host test";
    static constexpr auto        ProductString      = "fuzz";
    static constexpr auto        SerialNumberString = "1";
    static constexpr auto        ProductVersionBCD  = 0x0100;
    static constexpr auto        VendorID           = 0x1209;
    static constexpr auto        ProductID          = 0x0005;
    static constexpr std::size_t SendBufferSize     = 400;
    static constexpr std::size_t RecvBufferSize     = 300;
};

constexpr std::size_t  Ep    = 1;
constexpr std::uint8_t InEp  = 0x81;
constexpr std::uint8_t OutEp = 0x01;

/// What has to come out. A struct rather than locals captured by a lambda: clang 23's lifetime analysis takes a
/// container captured by reference as a borrow of its contents, so every later clear() looked like a use after
/// invalidation (-Wlifetime-safety-invalidation).
struct Model {
    std::deque<std::vector<std::byte>> expectedMessages;   // sent, not yet fully read
    std::vector<std::byte>             reading;            // the transfer the host is in
    std::deque<std::byte>              expectedBytes;      // acknowledged to the host, not yet read
    bool                               inHalted{};
    bool                               outHalted{};

    void dropInFlight() {
        expectedMessages.clear();
        reading.clear();
        expectedBytes.clear();
        inHalted = outHalted = false;
    }
};

template<typename Fake>
bool run(char const*   name,
         bool          eager,
         std::uint32_t seed,
         int           steps) {
    using Device = Kvasir::USB::Bulk<Fake::template Backend, Clock, Config>;
    using Host   = UsbHost<Fake, Device>;

    std::mt19937 rng{seed};
    auto const   pick
      = [&](std::size_t n) { return std::uniform_int_distribution<std::size_t>{0, n - 1}(rng); };

    Model       model;
    std::size_t counter = 0;
    int         before  = failures;

    auto const bringUp = [&] {
        check(Host::enumerate(), "enumerated");
        model.dropInFlight();
        check(Device::getRecvBuffer().empty(), "the application polls: the OUT endpoint is armed");
    };

    Host::plugIn();
    Fake::eagerInterrupts = eager;
    bringUp();

    for(int step = 0; step != steps && failures == before; ++step) {
        switch(pick(12)) {
        case 0:
        case 1:
            {   // the application sends a message
                std::size_t const size = pick(5) == 0 ? pick(3) * 64 : pick(399);
                auto const        msg  = pattern(size, ++counter);
                if(Device::isSendReady() && Device::send(msg)) {
                    model.expectedMessages.push_back(msg);
                }
                break;
            }
        case 2:
        case 3:
        case 4:
            {   // the host reads a packet
                auto in = Fake::hostIn(Ep);
                Host::pump();
                if(model.inHalted) {
                    check(in.handshake == Fake::Handshake::stall, "a halted endpoint reads STALL");
                    break;
                }
                if(in.handshake != Fake::Handshake::ack) { break; }
                model.reading.insert(model.reading.end(),
                                     in.packet.data.begin(),
                                     in.packet.data.end());
                if(in.packet.data.size() < Fake::MaxPacket) {
                    check(!model.expectedMessages.empty()
                            && model.reading == model.expectedMessages.front(),
                          "a transfer is the oldest message, whole");
                    if(!model.expectedMessages.empty()) { model.expectedMessages.pop_front(); }
                    model.reading.clear();
                }
                break;
            }
        case 5:
        case 6:
            {   // the host writes a packet
                // Full packets often: they are what a controller that gathers a transfer by
                // itself keeps to itself until a short one follows.
                auto const data = pattern(pick(3) == 0 ? 64 : 1 + pick(64), ++counter);
                auto const hs   = Fake::hostOut(Ep, data);
                Host::pump();
                if(hs == Fake::Handshake::ack) {
                    model.expectedBytes.insert(model.expectedBytes.end(), data.begin(), data.end());
                }
                check(hs != Fake::Handshake::stall || model.outHalted, "STALL only while halted");
                break;
            }
        case 7:
        case 8:
            {   // the application reads some
                std::size_t n = pick(200);
                std::byte   b{};
                while(n-- != 0 && Device::getRecvBuffer().pop_into(b)) {
                    check(!model.expectedBytes.empty() && model.expectedBytes.front() == b,
                          "a byte read is the oldest byte written");
                    if(!model.expectedBytes.empty()) { model.expectedBytes.pop_front(); }
                }
                break;
            }
        case 9:
            {   // a halt, or its clear: nothing is lost across either
                bool const in     = pick(2) == 0;
                bool&      halted = in ? model.inHalted : model.outHalted;
                check(
                  (halted ? Host::clearHalt(in ? InEp : OutEp) : Host::setHalt(in ? InEp : OutEp))
                    .ok(),
                  "halt feature");
                halted = !halted;
                break;
            }
        case 10:
            if(pick(4) == 0) {   // what the device may drop on
                if(pick(2) == 0) {
                    check(Host::setConfiguration(1).ok(), "SET_CONFIGURATION again");
                } else {
                    check(Host::controlOut(Host::makeSetup(0x01, 11, 0, 0, 0)).ok(),
                          "SET_INTERFACE");
                }
                model.dropInFlight();
                check(Device::getRecvBuffer().empty(), "received data is dropped with it");
            }
            break;
        default:
            if(pick(40) == 0) {
                Host::busReset();
                bringUp();
            }
            break;
        }
    }

    // Everything still owed arrives.
    if(model.inHalted) { check(Host::clearHalt(InEp).ok(), "halt cleared"); }
    for(int i = 0; i != 64 && !model.expectedMessages.empty() && failures == before; ++i) {
        auto const r = Host::bulkIn(Ep);
        model.reading.insert(model.reading.end(), r.data.begin(), r.data.end());
        if(r.endedShort) {
            check(model.reading == model.expectedMessages.front(),
                  "a queued message arrives in the end");
            model.expectedMessages.pop_front();
            model.reading.clear();
        }
    }
    check(model.expectedMessages.empty(), "no message is left over");
    std::byte b{};
    // Whole packets a controller still holds come up once the reading side has looked twice,
    // a while apart.
    for(int look = 0; look != 3; ++look) {
        while(Device::getRecvBuffer().pop_into(b)) {
            check(!model.expectedBytes.empty() && model.expectedBytes.front() == b,
                  "the rest of the bytes, in order");
            if(!model.expectedBytes.empty()) { model.expectedBytes.pop_front(); }
        }
        Clock::advance(std::chrono::milliseconds{3});
    }
    check(model.expectedBytes.empty(), "no acknowledged byte is missing");
    checkEq(Fake::violations, 0, "the backend contract was kept");

    Fake::eagerInterrupts = false;
    if(failures != before) {
        std::printf("   ^ %s, %s interrupts, seed %u\n", name, eager ? "eager" : "pumped", seed);
    }
    return failures == before;
}

template<typename Fake>
void sweep(char const* name) {
    testCase(name);
    for(bool const eager : {false, true}) {
        for(std::uint32_t seed = 1; seed <= 40; ++seed) {
            if(!run<Fake>(name, eager, seed, 3000)) { return; }
        }
    }
}
}   // namespace

int main() {
    sweep<FakeUsb<1, false>>("one packet per endpoint, cancel done on return");
    sweep<FakeUsb<2, true>>("two IN packets armed, cancel answered later");
    sweep<FakeUsb<1, false, 256>>("transfers of up to four packets, in place");
    sweep<FakeUsb<2, true, 256>>(
      "two transfers of up to four packets armed, cancel answered later");
    return finish();
}
