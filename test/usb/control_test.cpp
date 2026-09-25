// Endpoint 0: enumeration, the data stage's packets and its zero-length packet, STALLs, the timing
// of SET_ADDRESS, a SETUP that cuts a transfer short, a bus reset in the middle of one. Against
// both kinds of controller (FakeUsbBackend.hpp).
#include "UsbHost.hpp"
//
#include <kvasir/Devices/USB/CdcAcm.hpp>
#include <kvasir/Devices/USB/WinUsb.hpp>

using namespace Kvasir::Test;

namespace {
struct Config {
    static constexpr auto ManufacturerString = "Kvasir host test";
    // 31 characters: a 64-byte string descriptor, exactly one full packet.
    static constexpr auto ProductString      = "0123456789012345678901234567890";
    static constexpr auto SerialNumberString = [] { return std::string_view{"SN-42"}; };
    static constexpr auto ProductVersionBCD  = 0x0123;
    static constexpr auto VendorID           = 0x1209;
    static constexpr auto ProductID          = 0x0001;
};

std::uint16_t le16(std::vector<std::byte> const& v,
                   std::size_t                   at) {
    return static_cast<std::uint16_t>(std::to_integer<unsigned>(v[at])
                                      | (std::to_integer<unsigned>(v[at + 1]) << 8));
}

template<typename Fake>
void run(char const* name) {
    using Device = Kvasir::USB::CdcAcm<Fake::template Backend, Clock, Config>;
    using Host   = UsbHost<Fake, Device>;
    std::printf("== %s\n", name);

    testCase("the device comes up connected, and a host enumerates it");
    Host::plugIn();
    check(Fake::prepared && Fake::connected, "prepare() then connect()");
    check(!Device::isConfigured(), "not configured before SET_CONFIGURATION");
    check(Host::enumerate(9), "enumeration");
    check(Device::isConfigured(), "configured");
    checkEq(Fake::address, 9, "the address the host gave");

    testCase("the device descriptor");
    {
        auto const r = Host::getDescriptor(1, 0, 64);
        check(r.ok(), "answered");
        checkEq(r.data.size(), 18U, "18 bytes although 64 were asked for");
        checkEq(std::to_integer<int>(r.data[7]), 64, "bMaxPacketSize0");
        checkEq(le16(r.data, 8), 0x1209, "idVendor");
        checkEq(le16(r.data, 10), 0x0001, "idProduct");
        checkEq(le16(r.data, 12), 0x0123, "bcdDevice");
        check(r.data1.size() == 1 && r.data1[0], "the data stage starts with DATA1");
    }

    testCase("a configuration descriptor longer than a packet, and cut to wLength");
    {
        auto const head = Host::getDescriptor(2, 0, 9);
        check(head.ok(), "the 9-byte header");
        checkEq(head.data.size(), 9U, "exactly wLength");
        std::uint16_t const total = le16(head.data, 2);
        check(total > 64, "this configuration takes more than one packet");
        auto const all = Host::getDescriptor(2, 0, 255);
        check(all.ok(), "the whole of it");
        checkEq(all.data.size(), total, "wTotalLength bytes");
        check(all.data1.size() == 2 && all.data1[0] && !all.data1[1], "DATA1 then DATA0");
        check(Host::getDescriptor(2, 1, 255).stalled, "there is no configuration index 1");
    }

    testCase("a reply that ends on a packet boundary is closed by a zero-length packet");
    {
        auto const r = Host::getDescriptor(3, 2, 255, 0x0409);
        check(r.ok(), "the product string");
        checkEq(r.data.size(), 64U, "2 + 31 * 2 bytes");
        checkEq(r.data1.size(), 2U, "the full packet and the empty one");
        auto const exact = Host::getDescriptor(3, 2, 64, 0x0409);
        checkEq(exact.data1.size(), 1U, "none when the host asked for exactly that much");
        auto const serial = Host::getDescriptor(3, 3, 255, 0x0409);
        checkEq(serial.data.size(), 12U, "the runtime serial number string: 2 + 5 * 2");
        check(Host::getDescriptor(3, 9, 255, 0x0409).stalled, "no string 9");
    }

    testCase("what the device does not have is a STALL, and the next request is answered");
    {
        check(Host::getDescriptor(6, 0, 10).stalled, "device qualifier: full speed only");
        check(Host::controlIn(Host::makeSetup(0xC0, 0x55, 0, 0, 4)).stalled, "a vendor request");
        check(Host::controlOut(Host::makeSetup(0x00, 9, 5, 0, 0)).stalled, "SET_CONFIGURATION(5)");
        check(Device::isConfigured(), "which left the configuration alone");
        auto const status = Host::controlIn(Host::makeSetup(0x80, 0, 0, 0, 2));
        check(status.ok() && status.data.size() == 2, "GET_STATUS after the STALLs");
        auto const cfg = Host::controlIn(Host::makeSetup(0x80, 8, 0, 0, 1));
        check(cfg.ok() && std::to_integer<int>(cfg.data[0]) == 1, "GET_CONFIGURATION");
    }

    testCase("SET_ADDRESS takes effect after its status stage, not before");
    {
        Host::plugIn();
        Fake::hostSetup(Host::makeSetup(0x00, 5, 23, 0, 0));
        Host::pump();
        checkEq(Fake::address, 0, "still address 0 while the status stage is outstanding");
        auto const in = Fake::hostIn(0);
        check(in.handshake == Fake::Handshake::ack && in.packet.data.empty(), "the status stage");
        checkEq(Fake::address, 0, "and until the device has seen it complete");
        Host::pump();
        checkEq(Fake::address, 23, "then the new address");
        Host::busReset();
        checkEq(Fake::address, 0, "a bus reset takes it away again");
    }

    testCase("a SETUP in the middle of a data stage starts over");
    {
        Host::plugIn();
        Fake::hostSetup(Host::makeSetup(0x80, 6, 0x0200, 0, 255));
        Host::pump();
        check(Fake::hostIn(0).handshake == Fake::Handshake::ack, "the first packet");
        Host::pump();
        // the host gives up here and asks for something else
        auto const r = Host::getDescriptor(1, 0, 18);
        check(r.ok(), "the new request is answered");
        checkEq(r.data.size(), 18U, "with its own data, none of the old");
        check(r.data1[0], "from DATA1");
    }

    testCase("a bus reset in the middle of a data stage");
    {
        Host::plugIn();
        check(Host::enumerate(), "enumerated");
        Fake::hostSetup(Host::makeSetup(0x80, 6, 0x0200, 0, 255));
        Host::pump();
        Host::busReset();
        check(!Device::isConfigured(), "unconfigured");
        checkEq(Fake::address, 0, "address 0");
        check(Host::enumerate(), "and it enumerates again");
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
