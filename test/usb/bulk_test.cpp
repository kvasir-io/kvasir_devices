// A vendor bulk device with a byte-stream interface and the WinUSB descriptors: write() and
// flush(), the BOS and MS OS 2.0 descriptors, SET_INTERFACE, and the picotool-style reset
// interface with a chip's actions plugged in. Against both kinds of controller.
#include "UsbHost.hpp"
//
#include <chrono>
#include <kvasir/Devices/USB/SimpleBulk.hpp>
#include <kvasir/Devices/USB/VendorReset.hpp>
#include <kvasir/Devices/USB/WinUsb.hpp>
#include <vector>

using namespace Kvasir::Test;

namespace {
struct Config {
    static constexpr auto        ManufacturerString = "Kvasir host test";
    static constexpr auto        ProductString      = "bulk";
    static constexpr auto        SerialNumberString = "1";
    static constexpr auto        ProductVersionBCD  = 0x0100;
    static constexpr auto        VendorID           = 0x1209;
    static constexpr auto        ProductID          = 0x0003;
    static constexpr std::size_t SendBufferSize     = 256;
    static constexpr std::size_t RecvBufferSize     = 256;
};

struct ResetActions {
    static inline int bootloaders{};
    static inline int reboots{};

    static void bootloader() { ++bootloaders; }

    static void reboot() { ++reboots; }
};

template<typename C, typename Cfg, typename D, std::size_t I, std::size_t E>
using Reset = Kvasir::USB::VendorReset::Mixin<C, Cfg, D, I, E, ResetActions>;

template<typename Fake>
void run(char const* name) {
    // interface 0: messages on endpoint 1; interface 1: the byte stream on endpoint 2;
    // interface 2: the reset interface; WinUsb has no interface of its own.
    using Device = Kvasir::USB::Bulk<Fake::template Backend,
                                     Clock,
                                     Config,
                                     Kvasir::USB::SimpleBulk::StreamMixin,
                                     Reset,
                                     Kvasir::USB::WinUsb::Mixin>;
    using Host   = UsbHost<Fake, Device>;
    using Data   = Kvasir::USB::SimpleBulk::Mixin<Clock, Config, Device, 0, 1>;
    using Stream = Kvasir::USB::SimpleBulk::StreamMixin<Clock, Config, Device, 1, 2>;
    std::printf("== %s\n", name);

    testCase("WinUSB raises bcdUSB to 2.01 and adds the BOS and MS OS 2.0 descriptors");
    Host::plugIn();
    check(Host::enumerate(), "enumerated");
    {
        auto const dev = Host::getDescriptor(1, 0, 18);
        check(dev.ok() && std::to_integer<int>(dev.data[2]) == 0x01
                && std::to_integer<int>(dev.data[3]) == 0x02,
              "bcdUSB 0x0201");
        auto const bos = Host::getDescriptor(0x0F, 0, 255);
        check(bos.ok() && bos.data.size() > 5, "a BOS descriptor");
        auto const msos = Host::controlIn(Host::makeSetup(0xC0, 0x57, 0, 7, 1024));
        check(msos.ok() && msos.data.size() > 10, "the MS OS 2.0 descriptor set");
    }

    testCase("messages and the byte stream side by side, each on its own endpoint");
    {
        auto const msg = pattern(100);
        check(Data::isSendReady() && Data::send(msg), "a message on interface 0");
        auto const bytes = pattern(64, 1);
        checkEq(Stream::write(bytes), 64U, "64 bytes into the stream");
        check(Host::bulkIn(1).data == msg, "the message");
        auto r = Host::bulkIn(2);
        check(r.data == bytes && !r.endedShort, "the stream's packet, and the transfer stays open");
        Stream::flush();
        r = Host::bulkIn(2);
        check(r.data.empty() && r.endedShort, "flush() after a full packet: a zero-length one");
        checkEq(Stream::write(pattern(10, 2)), 10U, "10 more");
        Stream::flush();
        r = Host::bulkIn(2);
        check(r.data.size() == 10 && r.data1.size() == 1, "a short packet needs none");
        Stream::flush();
        check(Host::bulkIn(2).data1.empty(), "and a flush with nothing written sends nothing");
        std::size_t const room = Stream::writeAvailable();
        check(room > 0 && room <= 256, "the send buffer's room");
        checkEq(Stream::write(pattern(1000, 3)), room, "write() takes what there is room for");
        checkEq(Host::bulkIn(2, room).data.size(), room, "which the host reads");
    }

    testCase("SET_INTERFACE restarts that interface only");
    {
        check(Data::send(pattern(200, 4)), "a message in flight on interface 0");
        checkEq(Stream::write(pattern(100, 5)), 100U, "and stream bytes on interface 1");
        check(Host::controlOut(Host::makeSetup(0x01, 11, 0, 1, 0)).ok(), "SET_INTERFACE(1, 0)");
        check(Host::bulkIn(2).data.empty(), "the stream's bytes are dropped");
        check(Host::bulkIn(1).data == pattern(200, 4), "the message is not");
        check(Host::controlOut(Host::makeSetup(0x01, 11, 1, 1, 0)).stalled,
              "there is no alternate setting 1");
        auto const alt = Host::controlIn(Host::makeSetup(0x81, 10, 0, 1, 1));
        check(alt.ok() && std::to_integer<int>(alt.data[0]) == 0, "GET_INTERFACE: 0");
    }

    testCase("the reset interface runs the chip's actions, and only for its own requests");
    {
        ResetActions::bootloaders = 0;
        ResetActions::reboots     = 0;
        check(Host::controlOut(Host::makeSetup(0x01, 1, 0, 2, 0)).stalled,
              "a standard request to it is not a reboot");
        checkEq(ResetActions::bootloaders + ResetActions::reboots, 0, "nothing ran");
        Fake::hostSetup(Host::makeSetup(0x41, 0x01, 0, 2, 0));
        Host::pump();
        checkEq(ResetActions::bootloaders, 1, "request 1: the bootloader");
        Fake::hostSetup(Host::makeSetup(0x41, 0x02, 0, 2, 0));
        Host::pump();
        checkEq(ResetActions::reboots, 1, "request 2: a reboot");
    }

    // Host to device. Where the controller gathers a transfer by itself, whole packets without a
    // short one behind them wait in it: the reading side has to get them all the same, in order,
    // once, and a halt clear must not lose what was acknowledged.
    auto readAll = [] {
        std::vector<std::byte> got;
        std::byte              b{};
        while(Data::getRecvBuffer().pop_into(b)) { got.push_back(b); }
        return got;
    };
    auto readWithin = [&](std::chrono::milliseconds time) {
        auto got = readAll();
        for(auto waited = std::chrono::milliseconds{0}; waited < time; ++waited) {
            Clock::advance(std::chrono::milliseconds{1});
            auto const more = readAll();
            got.insert(got.end(), more.begin(), more.end());
        }
        return got;
    };

    testCase("host to device: what the host wrote arrives, whatever its length");
    {
        check(readWithin(std::chrono::milliseconds{5}).empty(), "nothing is waiting");
        for(std::size_t const length : {1U, 63U, 64U, 65U, 128U, 192U, 256U, 257U, 600U}) {
            // The queue is smaller than the longest of these: the host is held back until the
            // application has read, so both take turns.
            auto const             data = pattern(length, static_cast<unsigned>(length));
            std::vector<std::byte> got;
            std::size_t            sent = 0;
            for(int turn = 0; turn != 40 && got.size() != length; ++turn) {
                sent += Host::bulkOut(1, std::span{data}.subspan(sent));
                auto const more = readWithin(std::chrono::milliseconds{3});
                got.insert(got.end(), more.begin(), more.end());
            }
            checkEq(sent, length, "the device takes all of it");
            check(got == data, "and hands it up, whole and in order");
        }
    }

    testCase("host to device: a write that ends in a short packet needs no waiting");
    {
        auto const data = pattern(100, 77);
        checkEq(Host::bulkOut(1, data), 100U, "taken");
        check(readAll() == data, "there at once");
    }

    testCase("host to device: whole packets, then more - order is kept across the take-back");
    {
        auto const first  = pattern(128, 31);
        auto const second = pattern(70, 32);
        checkEq(Host::bulkOut(1, first), 128U, "two full packets");
        auto got = readWithin(std::chrono::milliseconds{1});   // looked at, not yet given up on
        checkEq(Host::bulkOut(1, second), 70U, "a full and a short one behind them");
        auto const rest = readWithin(std::chrono::milliseconds{5});
        got.insert(got.end(), rest.begin(), rest.end());
        auto expected = first;
        expected.insert(expected.end(), second.begin(), second.end());
        check(got == expected, "198 bytes, in order, once");
    }

    testCase("host to device: a nearly full queue - a transfer is never larger than its room");
    {
        // The queue holds 255 bytes. What is armed has to fit behind what is already there, in
        // whole packets, also when it is taken back half filled.
        auto const first = pattern(100, 61);
        checkEq(Host::bulkOut(1, first), 100U, "100 bytes queued and left there: room for 155");
        auto const second = pattern(64, 62);
        checkEq(Host::bulkOut(1, second), 64U, "a full packet into the two that fit");
        // Looked at twice without reading: taken back into the queue, which leaves room for 91.
        // (Where packets are armed one by one it is there at once.)
        static_cast<void>(Data::getRecvBuffer().size());
        Clock::advance(std::chrono::milliseconds{3});
        checkEq(Data::getRecvBuffer().size(), 164U, "both are in the queue");
        auto const third = pattern(64, 63);
        checkEq(Host::bulkOut(1, third), 64U, "one more packet fits");
        checkEq(Host::bulkOut(1, pattern(64, 64)), 0U, "and then none: room for 27");
        auto expected = first;
        expected.insert(expected.end(), second.begin(), second.end());
        expected.insert(expected.end(), third.begin(), third.end());
        check(readWithin(std::chrono::milliseconds{5}) == expected, "228 bytes, in order, once");
        auto const after = pattern(30, 65);
        checkEq(Host::bulkOut(1, after), 30U, "read empty, the endpoint takes data again");
        check(readAll() == after, "which arrives");
    }

    testCase("host to device: a halt clear keeps what was acknowledged");
    {
        auto const data = pattern(128, 41);
        checkEq(Host::bulkOut(1, data), 128U, "two full packets, acknowledged");
        check(Host::setHalt(0x01).ok(), "SET_FEATURE(ENDPOINT_HALT)");
        check(Host::clearHalt(0x01).ok(), "CLEAR_FEATURE(ENDPOINT_HALT)");
        check(readWithin(std::chrono::milliseconds{5}) == data, "they are read all the same");
        auto const after = pattern(10, 42);
        checkEq(Host::bulkOut(1, after), 10U, "and the endpoint takes data again");
        check(readAll() == after, "which arrives");
    }

    testCase("host to device: a reconfiguration drops what the controller still held");
    {
        checkEq(Host::bulkOut(1, pattern(64, 51)), 64U, "one full packet, acknowledged");
        check(Host::controlOut(Host::makeSetup(0x00, 9, 1, 0, 0)).ok(), "SET_CONFIGURATION(1)");
        check(readWithin(std::chrono::milliseconds{5}).empty(), "gone, like the queue");
        auto const after = pattern(30, 52);
        checkEq(Host::bulkOut(1, after), 30U, "new data is taken");
        check(readAll() == after, "and arrives alone");
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
