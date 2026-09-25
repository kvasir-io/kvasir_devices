// The HID class on the fake controller: its descriptors, input reports over the interrupt
// endpoint, SET_REPORT and GET_REPORT over endpoint 0, SET_IDLE, a halt on the endpoint - and the
// device-level additions that came with it: suspend and resume, disconnect(), Function<I>, string
// descriptors. Against both kinds of controller.
#include "UsbHost.hpp"
//
#include <kvasir/Devices/USB/Hid.hpp>
#include <kvasir/Devices/USB/SimpleBulk.hpp>

using namespace Kvasir::Test;

namespace {
struct Knob {
    // Vendor usage page, one 4-byte input report and one 2-byte output report, no report ids.
    static constexpr auto ReportDescriptor = std::to_array<std::uint8_t>(
      {0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08,
       0x95, 0x04, 0x09, 0x02, 0x81, 0x02, 0x95, 0x02, 0x09, 0x03, 0x91, 0x02, 0xC0});
    static constexpr std::size_t  InputReportSize = 4;
    static constexpr std::uint8_t PollIntervalMs  = 5;

    static inline std::vector<std::byte> lastOutput{};
    static inline bool                   accept{true};

    static bool setReport(std::uint8_t type,
                          std::uint8_t,
                          std::span<std::byte const> data) {
        if(type != 2 || !accept) { return false; }
        lastOutput.assign(data.begin(), data.end());
        return true;
    }

    static std::size_t getReport(std::uint8_t type,
                                 std::uint8_t,
                                 std::span<std::byte> out) {
        if(type != 1) { return 0; }
        for(std::size_t i = 0; i != 4; ++i) { out[i] = static_cast<std::byte>(0xA0 + i); }
        return 4;
    }
};

template<typename C, typename Cfg, typename D, std::size_t I, std::size_t E>
using KnobHid = Kvasir::USB::HID::Mixin<C, Cfg, D, I, E, Knob>;

struct Config {
    static constexpr auto        ManufacturerString = "Kvasir host test";
    static constexpr auto        ProductString      = "hid";
    static constexpr auto        SerialNumberString = "1";
    static constexpr auto        ProductVersionBCD  = 0x0100;
    static constexpr auto        VendorID           = 0x1209;
    static constexpr auto        ProductID          = 0x0004;
    static constexpr std::size_t SendBufferSize     = 256;
    static constexpr std::size_t RecvBufferSize     = 256;

    static inline int suspends{};
    static inline int resumes{};

    static void SuspendCallback(bool suspended) { ++(suspended ? suspends : resumes); }
};

template<typename Fake>
void run(char const* name) {
    // interface 0: the HID one, endpoint 1; interface 1: a vendor bulk one, endpoint 2.
    using Device = Kvasir::USB::
      Hid<Fake::template Backend, Clock, Config, KnobHid, Kvasir::USB::SimpleBulk::Mixin>;
    using Host = UsbHost<Fake, Device>;
    using Hid  = typename Device::template Function<0>;
    using Bulk = typename Device::template Function<1>;
    std::printf("== %s\n", name);

    testCase("a HID device: class 0 at the device, 3 at the interface, and its descriptors");
    Host::plugIn();
    check(Host::enumerate(), "enumerated");
    {
        auto const dev = Host::getDescriptor(1, 0, 18);
        check(dev.ok() && std::to_integer<int>(dev.data[4]) == 0
                && std::to_integer<int>(dev.data[6]) == 0,
              "bDeviceClass and bDeviceProtocol 0");
        auto const cfg = Host::getDescriptor(2, 0, 255);
        check(cfg.ok(), "the configuration");
        // 9 configuration + 9 interface, then the HID descriptor
        check(cfg.data.size() > 27 && std::to_integer<int>(cfg.data[18]) == 9
                && std::to_integer<int>(cfg.data[19]) == 0x21,
              "the HID descriptor follows the interface");
        checkEq(std::to_integer<unsigned>(cfg.data[25]),
                Knob::ReportDescriptor.size(),
                "with the report descriptor's length");
        auto const report = Host::controlIn(Host::makeSetup(0x81, 6, 0x2200, 0, 255));
        check(report.ok() && report.data.size() == Knob::ReportDescriptor.size(),
              "GET_DESCRIPTOR(report) to the interface");
        check(std::to_integer<int>(report.data[0]) == 0x06, "and it is the traits' bytes");
        check(Host::controlIn(Host::makeSetup(0x81, 6, 0x2200, 1, 255)).stalled,
              "not from the bulk interface");
    }

    testCase("an input report goes out once, when the host polls");
    {
        auto const report = pattern(4, 7);
        check(Hid::isReportReady() && Hid::sendReport(report), "taken");
        check(!Hid::isReportReady() && !Hid::sendReport(report), "the next one waits for it");
        auto const in = Fake::hostIn(1);
        check(in.handshake == Fake::Handshake::ack && in.packet.data == report,
              "the host reads it");
        Host::pump();
        check(Fake::hostIn(1).handshake == Fake::Handshake::nak, "and nothing until the next");
        check(Hid::isReportReady() && Hid::sendReport(pattern(4, 8)), "which is taken again");
        check(Fake::hostIn(1).packet.data == pattern(4, 8), "and read");
        Host::pump();
        check(!Hid::sendReport(pattern(5)), "a report larger than the endpoint is refused");
    }

    testCase("SET_REPORT, GET_REPORT and SET_IDLE on endpoint 0");
    {
        auto const out = pattern(2, 9);
        check(Host::controlOut(Host::makeSetup(0x21, 0x09, 0x0200, 0, 2), out).ok(), "SET_REPORT");
        check(Knob::lastOutput == out, "reaches the application");
        Knob::accept = false;
        check(Host::controlOut(Host::makeSetup(0x21, 0x09, 0x0200, 0, 2), out).stalled,
              "one it refuses is a STALL");
        Knob::accept  = true;
        auto const in = Host::controlIn(Host::makeSetup(0xA1, 0x01, 0x0100, 0, 4));
        check(in.ok() && in.data.size() == 4 && std::to_integer<int>(in.data[0]) == 0xA0,
              "GET_REPORT(input)");
        check(Host::controlIn(Host::makeSetup(0xA1, 0x01, 0x0300, 0, 4)).stalled,
              "GET_REPORT of a type the application has none of");
        check(Host::controlOut(Host::makeSetup(0x21, 0x0A, 0x7D00, 0, 0)).ok(), "SET_IDLE");
        auto const idle = Host::controlIn(Host::makeSetup(0xA1, 0x02, 0, 0, 1));
        check(idle.ok() && std::to_integer<int>(idle.data[0]) == 0x7D, "GET_IDLE returns it");
        check(Host::controlOut(Host::makeSetup(0x21, 0x0B, 0, 0, 0)).stalled,
              "SET_PROTOCOL: no boot protocol");
    }

    testCase("a halt on the report endpoint drops the waiting report and restarts the toggle");
    {
        check(Hid::sendReport(pattern(4, 1)), "one report sent");
        check(Fake::hostIn(1).handshake == Fake::Handshake::ack, "and read: DATA0");
        Host::pump();
        check(Hid::sendReport(pattern(4, 2)), "a second one waits");
        check(Host::setHalt(0x81).ok(), "halted");
        check(Fake::hostIn(1).handshake == Fake::Handshake::stall, "reads STALL");
        check(!Hid::isReportReady(), "and takes no report");
        check(Host::clearHalt(0x81).ok(), "cleared");
        check(Fake::hostIn(1).handshake == Fake::Handshake::nak, "the stale report is gone");
        check(Hid::isReportReady() && Hid::sendReport(pattern(4, 3)), "a new one is taken");
        auto const in = Fake::hostIn(1);
        check(in.packet.data == pattern(4, 3) && !in.packet.data1, "and goes out as DATA0");
        Host::pump();
    }

    testCase("Function<I> names each mixin where the device put it");
    {
        auto const msg = pattern(70, 4);
        check(Bulk::isSendReady() && Bulk::send(msg), "the bulk interface, by index");
        check(Host::bulkIn(2).data == msg, "is the one on endpoint 2");
    }

    testCase("suspend and resume reach the application, and a bus reset ends a suspend");
    {
        Config::suspends = Config::resumes = 0;
        check(!Device::isSuspended(), "awake");
        Fake::hostSuspend();
        Host::pump();
        check(Device::isSuspended() && Config::suspends == 1, "suspended");
        Fake::hostSuspend();
        Host::pump();
        checkEq(Config::suspends, 1, "reported once");
        Fake::hostResume();
        Host::pump();
        check(!Device::isSuspended() && Config::resumes == 1, "resumed");
        Fake::hostSuspend();
        Host::pump();
        Host::busReset();
        check(!Device::isSuspended(), "a bus reset is activity");
    }

    testCase("disconnect() and connect()");
    {
        check(Fake::connected, "on the bus");
        Device::disconnect();
        check(!Fake::connected, "off");
        Device::connect();
        check(Fake::connected, "and on again");
        Host::busReset();
        check(Host::enumerate(), "the host enumerates it afresh");
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
