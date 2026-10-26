// Endpoint 0: enumeration, the data stage's packets and its zero-length packet, STALLs, the timing
// of SET_ADDRESS, a SETUP that cuts a transfer short, a bus reset in the middle of one. Against
// both kinds of controller (FakeUsbBackend.hpp).
#include "UsbHost.hpp"
//
#include <kvasir/Devices/USB/CdcAcm.hpp>
#include <kvasir/Devices/USB/WinUsb.hpp>

using namespace Kvasir::Test;
using namespace Kvasir::Test::Usb;
using DT = Kvasir::USB::DescriptorType;

namespace {
struct Config : TestConfig {
    // 31 characters: a 64-byte string descriptor, exactly one full packet.
    static constexpr auto productString      = "0123456789012345678901234567890";
    static constexpr auto serialNumberString = [] { return std::string_view{"SN-42"}; };
    static constexpr auto productVersionBcd  = 0x0123;
    static constexpr auto productId          = 0x0001;
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
        auto const r = Host::getDescriptor(DT::device, 0, 64);
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
        auto const head = Host::getDescriptor(DT::configuration, 0, 9);
        check(head.ok(), "the 9-byte header");
        checkEq(head.data.size(), 9U, "exactly wLength");
        std::uint16_t const total = le16(head.data, 2);
        check(total > 64, "this configuration takes more than one packet");
        auto const all = Host::getDescriptor(DT::configuration, 0, 255);
        check(all.ok(), "the whole of it");
        checkEq(all.data.size(), total, "wTotalLength bytes");
        check(all.data1.size() == 2 && all.data1[0] && !all.data1[1], "DATA1 then DATA0");
        check(Host::getDescriptor(DT::configuration, 1, 255).stalled,
              "there is no configuration index 1");
    }

    testCase("a reply that ends on a packet boundary is closed by a zero-length packet");
    {
        auto const r = Host::getDescriptor(DT::string, 2, 255, 0x0409);
        check(r.ok(), "the product string");
        checkEq(r.data.size(), 64U, "2 + 31 * 2 bytes");
        checkEq(r.data1.size(), 2U, "the full packet and the empty one");
        auto const exact = Host::getDescriptor(DT::string, 2, 64, 0x0409);
        checkEq(exact.data1.size(), 1U, "none when the host asked for exactly that much");
        auto const serial = Host::getDescriptor(DT::string, 3, 255, 0x0409);
        checkEq(serial.data.size(), 12U, "the runtime serial number string: 2 + 5 * 2");
        check(Host::getDescriptor(DT::string, 9, 255, 0x0409).stalled, "no string 9");
    }

    testCase("what the device does not have is a STALL, and the next request is answered");
    {
        check(Host::getDescriptor(DT::deviceQualifier, 0, 10).stalled,
              "device qualifier: full speed only");
        check(
          Host::controlIn(Host::makeSetup(vendorFromDevice, std::uint8_t{0x55}, 0, 0, 4)).stalled,
          "a vendor request");
        check(Host::controlOut(Host::makeSetup(toDevice, Setup::Request::setConfiguration, 5, 0, 0))
                .stalled,
              "SET_CONFIGURATION(5)");
        check(Device::isConfigured(), "which left the configuration alone");
        auto const status
          = Host::controlIn(Host::makeSetup(fromDevice, Setup::Request::getStatus, 0, 0, 2));
        check(status.ok() && status.data.size() == 2, "GET_STATUS after the STALLs");
        auto const cfg
          = Host::controlIn(Host::makeSetup(fromDevice, Setup::Request::getConfiguration, 0, 0, 1));
        check(cfg.ok() && std::to_integer<int>(cfg.data[0]) == 1, "GET_CONFIGURATION");
    }

    testCase("SET_ADDRESS takes effect after its status stage, not before");
    {
        Host::plugIn();
        Fake::hostSetup(Host::makeSetup(toDevice, Setup::Request::setAddress, 23, 0, 0));
        Host::pump();
        checkEq(Fake::address, 0, "still address 0 while the status stage is outstanding");
        auto const in = Fake::hostIn(0);
        check(in.handshake == Fake::Handshake::ack && in.packet.data.empty(), "the status stage");
        checkEq(Fake::address, 0, "and until the device has seen it complete");
        Host::pump();
        checkEq(Fake::address, 23, "then the new address");
        Host::busReset();
        checkEq(Fake::address, 0, "a bus reset takes it away again");
        check(Fake::addressHistory == std::vector<std::uint8_t>{0, 23, 0},
              "written three times: 0 at the plug-in's bus reset, 23 once after the status stage, "
              "0 at the next reset");
    }

    testCase("a SETUP in the middle of a data stage starts over");
    {
        Host::plugIn();
        Fake::hostSetup(Host::makeSetup(fromDevice,
                                        Setup::Request::getDescriptor,
                                        descriptorValue(DT::configuration),
                                        0,
                                        255));
        Host::pump();
        check(Fake::hostIn(0).handshake == Fake::Handshake::ack, "the first packet");
        Host::pump();
        // the host gives up here and asks for something else
        auto const r = Host::getDescriptor(DT::device, 0, 18);
        check(r.ok(), "the new request is answered");
        checkEq(r.data.size(), 18U, "with its own data, none of the old");
        check(r.data1[0], "from DATA1");
        checkEq(Fake::controlTransfersBegun, 2, "each SETUP began a control transfer, once");
    }

    testCase("a bus reset in the middle of a data stage");
    {
        Host::plugIn();
        check(Host::enumerate(), "enumerated");
        Fake::hostSetup(Host::makeSetup(fromDevice,
                                        Setup::Request::getDescriptor,
                                        descriptorValue(DT::configuration),
                                        0,
                                        255));
        Host::pump();
        Host::busReset();
        check(!Device::isConfigured(), "unconfigured");
        checkEq(Fake::address, 0, "address 0");
        check(Host::enumerate(), "and it enumerates again");
    }

    checkEq(Fake::violations, 0, "the device kept to the backend contract throughout");
}

/// A configuration that asks for the start-of-frame callback (ConfigTraits::UseSof).
struct SofConfig : Config {
    static inline std::vector<std::uint16_t> frames{};

    static void startOfFrameCallback(std::uint16_t frame) { frames.push_back(frame); }
};

template<typename Fake>
void startOfFrame(char const* name) {
    std::printf("== %s\n", name);
    {
        testCase(
          "SOF: a configuration with startOfFrameCallback gets every frame number, in order");
        using Device = Kvasir::USB::CdcAcm<Fake::template Backend, Clock, SofConfig>;
        using Host   = UsbHost<Fake, Device>;
        SofConfig::frames.clear();
        Host::plugIn();
        check(Host::enumerate(), "enumerated");
        Fake::hostStartOfFrame(0x123);
        Fake::hostStartOfFrame(0x124);
        Host::pump();
        Fake::hostStartOfFrame(0x7FF);
        Host::pump();
        check(SofConfig::frames == std::vector<std::uint16_t>{0x123, 0x124, 0x7FF},
              "0x123, 0x124, 0x7FF");
        check(Host::getDescriptor(DT::device, 0, 18).ok(), "and endpoint 0 answers between them");
        checkEq(Fake::violations, 0, "within the backend contract");
    }
    {
        testCase("SOF: without the callback the frames are dropped, and nothing else changes");
        using Device = Kvasir::USB::CdcAcm<Fake::template Backend, Clock, Config>;
        using Host   = UsbHost<Fake, Device>;
        SofConfig::frames.clear();
        Host::plugIn();
        check(Host::enumerate(), "enumerated");
        Fake::hostStartOfFrame(0x10);
        Host::pump();
        check(Fake::events.empty(), "the frame was taken off the controller");
        check(SofConfig::frames.empty(), "and reached no callback");
        check(Device::isConfigured() && Host::getDescriptor(DT::device, 0, 18).ok(),
              "still configured, still answering");
    }
}
}   // namespace

int main() {
    run<FakeUsb<1, false>>("one packet per endpoint, cancel done on return");
    run<FakeUsb<2, true>>("two IN packets armed, cancel answered later");
    run<FakeUsb<1, false, 256>>("transfers of up to four packets, in place, cancel done on return");
    startOfFrame<FakeUsb<1, false>>("start of frame, one packet per endpoint");
    startOfFrame<FakeUsb<2, true>>("start of frame, two IN packets armed");
    return finish();
}
