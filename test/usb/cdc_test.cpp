// The CDC-ACM class on the fake controller: its class requests, messages out through the bulk IN
// endpoint (packets, the closing zero-length packet, the data toggle), data in through the bulk
// OUT endpoint with back-pressure, and what a halt, a reconfiguration and a bus reset do to data
// in flight. Against both kinds of controller (FakeUsbBackend.hpp).
#include "UsbHost.hpp"
//
#include <kvasir/Devices/USB/CdcAcm.hpp>

using namespace Kvasir::Test;

namespace {
struct Config {
    static constexpr auto ManufacturerString = "Kvasir host test";
    static constexpr auto ProductString      = "cdc";
    static constexpr auto SerialNumberString = "1";
    static constexpr auto ProductVersionBCD  = 0x0100;
    static constexpr auto VendorID           = 0x1209;
    static constexpr auto ProductID          = 0x0002;
    // Small, so that back-pressure is reached with a handful of packets.
    static constexpr std::size_t SendBufferSize = 512;
    static constexpr std::size_t RecvBufferSize = 192;
};

// CdcAcm: interface 0 management (endpoint 1 IN, interrupt), interface 1 data (endpoint 2).
constexpr std::size_t  DataEp  = 2;
constexpr std::uint8_t DataIn  = 0x82;
constexpr std::uint8_t DataOut = 0x02;

template<typename Device>
std::vector<std::byte> drain() {
    std::vector<std::byte> got;
    std::byte              b{};
    while(Device::getRecvBuffer().pop_into(b)) { got.push_back(b); }
    return got;
}

template<typename Fake>
void run(char const* name) {
    using Device = Kvasir::USB::CdcAcm<Fake::template Backend, Clock, Config>;
    using Host   = UsbHost<Fake, Device>;
    std::printf("== %s\n", name);

    auto const open = [] {
        Host::plugIn();
        check(Host::enumerate(), "enumerated");
        check(Host::controlOut(Host::makeSetup(0x21, 0x22, 0x0003, 0, 0)).ok(), "DTR and RTS on");
        // The application polls its receive queue, and that is what arms the OUT endpoint after
        // a (re)configuration: the reading side empties the queue first (BulkOutEndpoint).
        check(Device::getRecvBuffer().empty(), "nothing received yet");
    };

    testCase("the port opens with SET_CONTROL_LINE_STATE, and closes with it");
    Host::plugIn();
    check(Host::enumerate(), "enumerated");
    check(!Device::isConnected() && !Device::isSendReady(), "configured is not yet open");
    check(Host::controlOut(Host::makeSetup(0x21, 0x22, 0x0001, 0, 0)).ok(), "DTR on");
    check(Device::isConnected() && Device::isSendReady(), "open");
    check(Host::controlOut(Host::makeSetup(0x21, 0x22, 0x0000, 0, 0)).ok(), "DTR off");
    check(!Device::isConnected(), "closed");

    testCase("SET_LINE_CODING has a data stage, and GET_LINE_CODING returns it");
    {
        std::array<std::byte, 7> const coding{std::byte{0x00},
                                              std::byte{0xC2},
                                              std::byte{0x01},
                                              std::byte{0x00},   // 115200
                                              std::byte{0},
                                              std::byte{0},
                                              std::byte{8}};
        check(Host::controlOut(Host::makeSetup(0x21, 0x20, 0, 0, 7), coding).ok(), "set");
        auto const r = Host::controlIn(Host::makeSetup(0xA1, 0x21, 0, 0, 7));
        check(r.ok(), "get");
        check(std::ranges::equal(r.data, coding), "what was set");
        check(Host::controlOut(Host::makeSetup(0x21, 0x20, 0, 0, 3)).stalled,
              "a line coding of the wrong length is refused");
    }

    testCase("SERIAL_STATE goes out on the notification endpoint");
    {
        using State = typename Device::SerialState;
        check(Device::sendSerialState(State::Dcd | State::Dsr), "taken");
        check(!Device::sendSerialState(0), "one at a time");
        auto const in = Fake::hostIn(1);
        check(in.handshake == Fake::Handshake::ack && in.packet.data.size() == 10, "10 bytes");
        check(std::to_integer<int>(in.packet.data[0]) == 0xA1
                && std::to_integer<int>(in.packet.data[1]) == 0x20
                && std::to_integer<int>(in.packet.data[8]) == 0x03,
              "a SERIAL_STATE notification with DCD and DSR");
        Host::pump();
        check(Device::sendSerialState(0), "and the next one is taken");
        check(Fake::hostIn(1).handshake == Fake::Handshake::ack, "and read");
        Host::pump();
    }

    testCase("a message is one transfer: short packet, or a zero-length one after a full packet");
    {
        open();
        auto const small = pattern(10);
        check(Device::send(small), "10 bytes");
        auto r = Host::bulkIn(DataEp);
        check(r.data == small && r.endedShort, "arrive as one short packet");
        check(!r.data1[0], "DATA0 first after SET_CONFIGURATION");

        auto const full = pattern(64, 1);
        check(Device::send(full), "64 bytes");
        r = Host::bulkIn(DataEp);
        check(r.data == full && r.endedShort, "arrive");
        checkEq(r.data1.size(), 2U, "as a full packet and a zero-length one");

        auto const big = pattern(200, 2);
        check(Device::send(big), "200 bytes");
        r = Host::bulkIn(DataEp);
        check(r.data == big && r.endedShort, "arrive");
        checkEq(r.data1.size(), 4U, "64 + 64 + 64 + 8");
        bool alternates = true;
        for(std::size_t i = 1; i < r.data1.size(); ++i) {
            alternates = alternates && r.data1[i] != r.data1[i - 1];
        }
        check(alternates, "with the data toggle alternating");

        check(Device::send({}), "an empty message");
        r = Host::bulkIn(DataEp);
        check(r.data.empty() && r.endedShort, "is a zero-length packet");

        check(Device::send(small) && Device::send(full) && Device::send(small), "three queued");
        check(Host::bulkIn(DataEp).data == small, "first");
        check(Host::bulkIn(DataEp).data == full, "second");
        check(Host::bulkIn(DataEp).data == small, "third");
        check(!Device::send(pattern(513)), "a message larger than the send buffer is refused");
    }

    testCase("received data waits in the queue, and a full queue NAKs the host");
    {
        open();
        auto const data = pattern(100, 3);
        checkEq(Host::bulkOut(DataEp, data), 100U, "100 bytes taken");
        check(drain<Device>() == data, "and read back");

        auto const        flood = pattern(64 * 8, 4);
        std::size_t const taken = Host::bulkOut(DataEp, flood);
        check(taken >= 128 && taken < flood.size(), "a 192-byte queue takes some, then NAKs");
        auto              got  = drain<Device>();
        std::size_t const rest = Host::bulkOut(DataEp, std::span{flood}.subspan(taken));
        check(rest > 0, "reading makes room, and the host's retry is taken");
        auto const more = drain<Device>();
        got.insert(got.end(), more.begin(), more.end());
        check(std::ranges::equal(got, std::span{flood}.first(taken + rest)),
              "nothing lost, nothing twice");
    }

    testCase("a halt on the IN endpoint: STALL, and the transfer goes on after the clear");
    {
        open();
        auto const msg = pattern(300, 5);
        check(Device::send(msg), "300 bytes queued");
        auto first = Fake::hostIn(DataEp);
        check(first.handshake == Fake::Handshake::ack, "the first packet");
        Host::pump();
        check(Host::setHalt(DataIn).ok(), "SET_FEATURE(ENDPOINT_HALT)");
        check(Fake::hostIn(DataEp).handshake == Fake::Handshake::stall, "reads STALL");
        auto const st = Host::controlIn(Host::makeSetup(0x82, 0, 0, DataIn, 2));
        check(st.ok() && std::to_integer<int>(st.data[0]) == 1, "GET_STATUS says halted");
        check(Host::clearHalt(DataIn).ok(), "CLEAR_FEATURE(ENDPOINT_HALT)");
        auto const rest = Host::bulkIn(DataEp);
        check(!rest.data1.empty() && !rest.data1[0], "the toggle starts over at DATA0");
        std::vector<std::byte> all = first.packet.data;
        all.insert(all.end(), rest.data.begin(), rest.data.end());
        check(all == msg, "nothing lost, nothing repeated");
    }

    testCase("a halt on the OUT endpoint keeps what was acknowledged");
    {
        open();
        auto const data = pattern(64, 6);
        checkEq(Host::bulkOut(DataEp, data), 64U, "one packet taken");
        check(Host::setHalt(DataOut).ok(), "halted");
        check(Fake::hostOut(DataEp, data) == Fake::Handshake::stall, "writes STALL");
        check(Host::clearHalt(DataOut).ok(), "cleared");
        checkEq(Host::bulkOut(DataEp, data), 64U, "and taken again");
        checkEq(drain<Device>().size(), 128U, "both packets are in the queue");
    }

    testCase("SET_CONFIGURATION again drops what was in flight, both ways");
    {
        open();
        check(Device::send(pattern(300, 7)), "300 bytes queued");
        check(Fake::hostIn(DataEp).handshake == Fake::Handshake::ack, "one packet goes");
        Host::pump();
        checkEq(Host::bulkOut(DataEp, pattern(64, 8)), 64U, "one packet comes");
        check(Host::setConfiguration(1).ok(), "SET_CONFIGURATION(1) again");
        check(Host::bulkIn(DataEp).data.empty(), "the rest of the message is gone");
        check(drain<Device>().empty(), "and so is the received packet");
        auto const fresh = pattern(70, 9);
        check(Device::send(fresh), "a new message");
        auto const r = Host::bulkIn(DataEp);
        check(r.data == fresh && !r.data1[0], "arrives whole, from DATA0");
        checkEq(Host::bulkOut(DataEp, fresh), 70U, "and the OUT endpoint takes data again");
        check(drain<Device>() == fresh, "which is what is read");
    }

    testCase("a bus reset with data in flight");
    {
        open();
        check(Device::send(pattern(300, 10)), "300 bytes queued");
        check(Fake::hostIn(DataEp).handshake == Fake::Handshake::ack, "one packet goes");
        Host::pump();
        Host::busReset();
        check(!Device::isConnected() && !Device::isSendReady(), "closed and unconfigured");
        check(Host::enumerate(), "enumerates again");
        check(Host::controlOut(Host::makeSetup(0x21, 0x22, 0x0001, 0, 0)).ok(), "opens again");
        check(Host::bulkIn(DataEp).data.empty(), "nothing of the old message is left");
        auto const fresh = pattern(130, 11);
        check(Device::send(fresh), "a new message");
        check(Host::bulkIn(DataEp).data == fresh, "arrives whole");
    }

    checkEq(Fake::violations, 0, "the device kept to the backend contract throughout");
}
}   // namespace

int main() {
    run<FakeUsb<1, false>>("one packet per endpoint, cancel done on return");
    run<FakeUsb<2, true>>("two IN packets armed, cancel answered later");
    run<FakeUsb<1, false, 256>>("transfers of up to four packets, in place, cancel done on return");
    return finish();
}
