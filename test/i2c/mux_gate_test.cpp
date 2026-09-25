/// A device behind an I2C switch: the gate, the arbiter and its three policies, against two
/// parts at one address that are never on the wire together.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Mux.hpp>
#include <kvasir/Devices/I2C/chips/Sht3x.hpp>
#include <kvasir/Devices/I2C/chips/Tca9548a.hpp>
#include <map>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

// -- an I2C switch: two devices at the same address, never on the wire together ------------

namespace MuxTest {

    using MuxDev = Dev<Chips::Tca9548a>;

    // The gate carries its channel in the type (the clash check below keys on that) and its
    // switch as a member, so nothing here needs static storage.
    using Left  = Device<FakeBus,
                         FakeClock,
                         Chips::Sht3x,
                         DefaultConfig,
                         NoReset,
                         Kvasir::I2C::MuxGate<MuxDev, 0>>;
    using Right = Device<FakeBus,
                         FakeClock,
                         Chips::Sht3x,
                         DefaultConfig,
                         NoReset,
                         Kvasir::I2C::MuxGate<MuxDev, 3>>;

    /// A chip whose data-ready flag is polled by a check step. With the flag never set the
    /// run is rejected after the engine's retries -- and that exit has to let the switch go
    /// like every other, or the channel is held for good.
    struct FlagChip {
        static constexpr std::string_view Name          = "FLAG";
        static constexpr Address7         Address       = 0x60;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Data {
            static constexpr auto       Period = std::chrono::milliseconds{100};
            static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0}),
                                              Step::check(10ms),
                                              Step::read({.reg = 0x01, .count = 1, .offset = 1})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr bool ready(Bytes data) { return (data.u8(0) & 1U) != 0; }

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(1); }
        };

        using Reads = List<Data>;
    };

    using Flagged = Device<FakeBus,
                           FakeClock,
                           FlagChip,
                           DefaultConfig,
                           NoReset,
                           Kvasir::I2C::MuxGate<MuxDev, 0>>;

    /// A part that is always due: a one-byte read every millisecond, on channel 0.
    struct BusyChip {
        static constexpr std::string_view Name          = "BUSY";
        static constexpr Address7         Address       = 0x61;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Data {
            static constexpr auto       Period = std::chrono::milliseconds{1};
            static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        using Reads = List<Data>;
    };

    /// A part on a 10 ms period, on channel 3, handled after the busy one every turn.
    struct TenMsChip {
        static constexpr std::string_view Name          = "TENMS";
        static constexpr Address7         Address       = 0x62;
        static constexpr std::size_t      RegisterBytes = 1;

        struct Data {
            static constexpr auto       Period = std::chrono::milliseconds{10};
            static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 1, .offset = 0})};
            using Sample = std::uint8_t;

            [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
        };

        using Reads = List<Data>;
    };

    using Busy  = Device<FakeBus,
                         FakeClock,
                         BusyChip,
                         DefaultConfig,
                         NoReset,
                         Kvasir::I2C::MuxGate<MuxDev, 0>>;
    using TenMs = Device<FakeBus,
                         FakeClock,
                         TenMsChip,
                         DefaultConfig,
                         NoReset,
                         Kvasir::I2C::MuxGate<MuxDev, 3>>;

    // The address check keys on (address, gate), so the same part on two channels is legal --
    // and two on the *same* channel is not.
    static_assert(detail::addressesDistinct<Left,
                                            Right>(),
                  "0x44 twice, but on channels 0 and 3");
    static_assert(!detail::addressesDistinct<Left,
                                             Left>(),
                  "0x44 twice on channel 0");
    static_assert(!detail::addressesDistinct<Dev<Chips::Sht3x>,
                                             Dev<Chips::Sht3x>>(),
                  "0x44 twice on the bare bus");
    static_assert(!detail::addressesDistinct<Dev<Chips::Sht3x>,
                                             Left>(),
                  "0x44 on the bare bus and on channel 0: the one in front is on every "
                  "channel's wire, and answers with the one behind");

    /// A part in front of the switch that must not be heard behind it: its gate claims the
    /// switch closed (MuxFrontGate).
    using InFront = Device<FakeBus,
                           FakeClock,
                           TenMsChip,
                           DefaultConfig,
                           NoReset,
                           Kvasir::I2C::MuxFrontGate<MuxDev>>;

    // It is a part of segment 0, like an ungated one, and the Bus binds its gate to the
    // switch's arbiter all the same.
    using FrontBus = Kvasir::I2C::Bus<FakeBus, FakeClock, MuxDev, InFront, Left, Right>;
    static_assert(FrontBus::segmentOf<InFront>() == 0,
                  "in front of the switch");
    static_assert(FrontBus::segmentOf<Left>() == 1 && FrontBus::segmentOf<Right>() == 4,
                  "the channels keep their segments");
    static_assert(FrontBus::Switches == 1,
                  "one switch, whoever comes first in the list");
    // And a Bus whose only gated part is the one in front still has its switch and an arbiter.
    using OnlyFrontBus = Kvasir::I2C::Bus<FakeBus, FakeClock, MuxDev, InFront>;
    static_assert(OnlyFrontBus::Switches == 1 && OnlyFrontBus::segmentOf<InFront>() == 0,
                  "the switch is found by the gate, not by a segment behind it");
    static_assert(std::is_same_v<OnlyFrontBus::SwitchDeviceAt<0>,
                                 MuxDev>,
                  "and it is this one");

    // A gate costs nothing when there is none.
    static_assert(sizeof(Dev<Chips::Sht3x>)
                    == sizeof(Device<FakeBus,
                                     FakeClock,
                                     Chips::Sht3x>),
                  "NoGate is not a member");

}   // namespace MuxTest

void muxGate() {
    using namespace MuxTest;

    // The switch, the arbiter and the two devices behind them are locals: a gate keeps a
    // pointer to the switch it is on rather than naming it, so none of this is a global.
    Kvasir::I2C::MuxArbiter arbiter{};
    MuxDev                  mux{};
    Left                    left{};
    Right                   right{};
    left.gate().bind(mux, arbiter);
    right.gate().bind(mux, arbiter);

    testCase("Mux: a gated device waits for its channel, and the switch write comes first");
    fresh();
    SwitchWire wire1{};
    FakeBus::respond = std::ref(wire1);
    for(int i = 0; i < 300; ++i) { turn(mux, left); }

    // Every transaction to 0x44 must be preceded by a write of 0x01 to the switch at 0x70,
    // with no other channel selection in between: that is the whole contract.
    std::uint8_t selected = 0xFF;
    std::size_t  gatedOk = 0, gatedBad = 0;
    for(auto const& t : FakeBus::log) {
        if(!t.isBus()) { continue; }
        if(t.address == Chips::Tca9548a::Address && t.isWrite() && t.sent.size() == 1) {
            selected = t.sent[0];
        } else if(t.address == Chips::Sht3x::Address) {
            if(selected == 0x01) {
                ++gatedOk;
            } else {
                ++gatedBad;
            }
        }
    }
    check(gatedOk > 0, "the gated device did talk");
    checkEq(gatedBad, std::size_t{0}, "and never before its channel was selected");
    checkEq(selected, 0x01, "the switch is left on channel 0");

    testCase("Mux: two channels take the switch in turn, never mid-script");
    fresh();
    SwitchWire wire2{};
    FakeBus::respond = std::ref(wire2);
    for(int i = 0; i < 1500; ++i) { turn(mux, left, right); }
    selected           = 0xFF;
    std::size_t leftOk = 0, rightOk = 0, wrong = 0;
    for(auto const& t : FakeBus::log) {
        if(!t.isBus()) { continue; }
        if(t.address == Chips::Tca9548a::Address && t.isWrite() && t.sent.size() == 1) {
            selected = t.sent[0];
        } else if(t.address == Chips::Sht3x::Address) {
            if(selected == 0x01) {
                ++leftOk;
            } else if(selected == 0x08) {
                ++rightOk;
            } else {
                ++wrong;
            }
        }
    }
    check(leftOk > 0 && rightOk > 0, "both channels got their turn");
    checkEq(wrong, std::size_t{0}, "and no transaction went out on an unselected switch");

    testCase("Mux: a part in front with a MuxFrontGate talks only while the switch is closed");
    fresh();
    {
        FrontBus   bus{};
        SwitchWire wire{};
        FakeBus::respond = std::ref(wire);
        for(int i = 0; i < 1500; ++i) { turn(bus); }
        selected            = 0xFF;
        std::size_t frontOk = 0, frontHeard = 0, behindOk = 0, behindBad = 0;
        for(auto const& t : FakeBus::log) {
            if(!t.isBus()) { continue; }
            if(t.address == Chips::Tca9548a::Address && t.isWrite() && t.sent.size() == 1) {
                selected = t.sent[0];
            } else if(t.address == TenMsChip::Address) {
                if(selected == 0x00) {
                    ++frontOk;
                } else {
                    ++frontHeard;
                }
            } else if(t.address == Chips::Sht3x::Address) {
                if(selected == 0x01 || selected == 0x08) {
                    ++behindOk;
                } else {
                    ++behindBad;
                }
            }
        }
        check(frontOk > 0, "the part in front did talk");
        checkEq(frontHeard, std::size_t{0}, "and never with a channel open");
        check(behindOk > 0, "the parts behind the switch still got their turns");
        checkEq(behindBad, std::size_t{0}, "each on its own channel");
        check(bus.get<InFront>().samples() > 0, "and the part in front measured");
    }

    testCase("Mux: a device waiting out a measurement lets the other channel have the switch");
    fresh();
    // Words of 0x0000 with their Sensirion CRC (0x81), so both parts pass their bring-up
    // and get to measure: zeros alone fail the CRC and leave them repeating the bring-up.
    SwitchWire wireCrc{.behind = crcZeros};
    FakeBus::respond = std::ref(wireCrc);
    for(int i = 0; i < 2500; ++i) { turn(mux, left, right); }
    // An SHT3x measurement is the command 0x2400, 16 ms, then the six-byte read. With the gate
    // let go for the wait, the other channel's transactions land between the two -- each
    // still behind its own switch write, which the case above checks for a whole transcript.
    {
        std::uint8_t sel          = 0xFF;
        bool         leftMeasures = false;
        bool         rightSlips   = false;
        std::size_t  measured     = 0;
        std::size_t  between      = 0;
        for(auto const& t : FakeBus::log) {
            if(!t.isBus()) { continue; }
            if(t.address == Chips::Tca9548a::Address && t.isWrite() && t.sent.size() == 1) {
                sel = t.sent[0];
                continue;
            }
            if(t.address != Chips::Sht3x::Address) { continue; }
            if(sel == 0x01) {
                if(t.isWrite() && t.sent == std::vector<std::uint8_t>{0x24, 0x00}) {
                    leftMeasures = true;
                    rightSlips   = false;
                } else if(t.isRead() && t.recvLen == 6 && leftMeasures) {
                    ++measured;
                    if(rightSlips) { ++between; }
                    leftMeasures = false;
                }
            } else if(sel == 0x08 && leftMeasures) {
                rightSlips = true;
            }
        }
        check(measured > 0, "the first channel's part measured");
        check(between > 0, "and the other channel talked while it waited for the result");
    }

    testCase("Mux: a gate nobody bound puts nothing on the wire");
    fresh();
    SwitchWire wire3{};
    FakeBus::respond = std::ref(wire3);
    {
        // Never bound to a switch: claim() can only answer "not yet", so the device stays
        // silent rather than talking to whatever the switch happens to be pointing at. It
        // says so once through UC_LOG_W; what is checked here is the silence.
        Left orphan{};
        check(!orphan.gate().bound(), "the gate knows it was never bound");
        runFor(orphan, 300ms);
        std::size_t spoke = 0;
        for(auto const& t : FakeBus::log) {
            if(t.isBus() && t.address == Chips::Sht3x::Address) { ++spoke; }
        }
        checkEq(spoke, std::size_t{0}, "and nothing of its went out");
    }

    testCase("Mux: a check step rejected after its retries lets the switch go");
    fresh();
    SwitchWire wire4{};
    FakeBus::respond = std::ref(wire4);
    {
        Kvasir::I2C::MuxArbiter arbiter2{};
        MuxDev                  mux2{};
        Flagged                 flagged{};
        Right                   right2{};
        flagged.gate().bind(mux2, arbiter2);
        right2.gate().bind(mux2, arbiter2);
        // The other channel is up first (an empty Init costs the flagged chip a turn), so
        // what it got *before* the first rejection proves nothing: only the transcript
        // from that point on does.
        std::size_t fromReject = 0;
        bool        rejected   = false;
        for(int i = 0; i < 1500; ++i) {
            turn(mux2, flagged, right2);
            if(!rejected && flagged.rejected<MuxTest::FlagChip::Data>() > 0) {
                rejected   = true;
                fromReject = FakeBus::log.size();
            }
        }
        check(rejected, "the flag never came, so the run was rejected");
        selected             = 0xFF;
        std::size_t rightGot = 0;
        for(std::size_t i = 0; i < FakeBus::log.size(); ++i) {
            auto const& t = FakeBus::log[i];
            if(!t.isBus()) { continue; }
            if(t.address == Chips::Tca9548a::Address && t.isWrite() && t.sent.size() == 1) {
                selected = t.sent[0];
            } else if(i >= fromReject && t.address == Chips::Sht3x::Address && selected == 0x08) {
                ++rightGot;
            }
        }
        check(rightGot > 0, "and the other channel got the switch after the rejection");
    }

    testCase("Mux: a channel that always wants the switch does not starve the next one");
    fresh();
    SwitchWire wire5{};
    FakeBus::respond = std::ref(wire5);
    {
        // The busy part is handled first every turn and is due again the moment its read is
        // done; the 10 ms part asks every turn too, and over two seconds is owed 200 reads. The
        // switch is first come, first served, which does not starve it: the busy part only
        // learns its read is done on its next turn, and the other channel has claimed the
        // switch by then. Checks that property (all 200 are read).
        Kvasir::I2C::MuxArbiter arbiter3{};
        MuxDev                  mux3{};
        Busy                    busy{};
        TenMs                   tenMs{};
        busy.gate().bind(mux3, arbiter3);
        tenMs.gate().bind(mux3, arbiter3);
        for(int i = 0; i < 2000; ++i) { turn(mux3, busy, tenMs); }
        check(busy.samples() > 0, "the busy part is read");
        check(tenMs.samples() >= 150,
              "and the 10 ms part gets at least three quarters of its reads");
        if(failures != 0) {
            std::printf("    busy %u, 10 ms %u of 200\n", busy.samples(), tenMs.samples());
        }
    }

    testCase("Mux: a part that never got the switch cannot free it for another");
    fresh();
    SwitchWire wire6{};
    FakeBus::respond = std::ref(wire6);
    {
        Kvasir::I2C::MuxArbiter         arbiter4{};
        MuxDev                          mux4{};
        Kvasir::I2C::MuxGate<MuxDev, 0> holder{};
        Kvasir::I2C::MuxGate<MuxDev, 0> idle{};
        Kvasir::I2C::MuxGate<MuxDev, 3> other{};
        holder.bind(mux4, arbiter4);
        idle.bind(mux4, arbiter4);
        other.bind(mux4, arbiter4);
        check(settle(holder, mux4), "one part on channel 0 gets the switch");
        check(!other.claim(), "channel 3 waits for it");
        idle.release();   // the other part of channel 0 never claimed it: its run ended first
        checkEq(arbiter4.holder,
                std::uint8_t{0},
                "a release by a part without the switch frees nothing");
        check(!other.claim(), "so channel 3 still waits");
        holder.release();
        checkEq(arbiter4.holder, MuxArbiter::NoHolder, "the holder's own release frees it");
        check(settle(other, mux4), "and channel 3 gets it");
    }

    testCase("Mux: a wait is counted once, however many turns it lasts");
    fresh();
    SwitchWire wire7{};
    FakeBus::respond = std::ref(wire7);
    {
        Kvasir::I2C::MuxArbiter         arbiter8{};
        MuxDev                          mux8{};
        Kvasir::I2C::MuxGate<MuxDev, 0> holder{};
        Kvasir::I2C::MuxGate<MuxDev, 3> waiter{};
        holder.bind(mux8, arbiter8);
        waiter.bind(mux8, arbiter8);
        check(settle(holder, mux8), "channel 0 gets the switch");
        for(int i = 0; i < 10; ++i) { check(!waiter.claim(), "channel 3 waits"); }
        checkEq(arbiter8.waited[3][0], std::uint32_t{1}, "ten refused claims are one wait");
        holder.release();
        waiter.release();   // its run ended while it waited: so did the wait
        check(settle(holder, mux8), "channel 0 again");
        check(!waiter.claim(), "channel 3 waits again");
        checkEq(arbiter8.waited[3][0],
                std::uint32_t{2},
                "a new run that has to wait is a second wait");
        holder.release();
        check(settle(waiter, mux8), "and gets the switch");
        checkEq(arbiter8.waited[3][0], std::uint32_t{2}, "getting it counts no wait");
    }

    testCase(
      "Mux: ShareChannel lets a second part of the channel go, and the first release frees it");
    fresh();
    SwitchWire wire8{};
    FakeBus::respond = std::ref(wire8);
    {
        Kvasir::I2C::MuxArbiter         arbiter5{};
        MuxDev                          mux5{};
        Kvasir::I2C::MuxGate<MuxDev, 0> first{};
        Kvasir::I2C::MuxGate<MuxDev, 0> second{};
        Kvasir::I2C::MuxGate<MuxDev, 3> other{};
        first.bind(mux5, arbiter5);
        second.bind(mux5, arbiter5);
        other.bind(mux5, arbiter5);
        check(settle(first, mux5), "the first part gets channel 0");
        check(second.claim(), "a second part of channel 0 goes at once");
        first.release();
        checkEq(arbiter5.holder, MuxArbiter::NoHolder, "the first release frees the switch");
        check(settle(other, mux5), "channel 3 takes it");
        second.release();
        checkEq(arbiter5.holder,
                std::uint8_t{3},
                "the second part's hold ended with the first release");
    }

    testCase("Mux: HoldChannel keeps the switch for a channel until its last part lets go");
    fresh();
    SwitchWire wire9{};
    FakeBus::respond = std::ref(wire9);
    {
        Kvasir::I2C::BasicMuxArbiter<HoldChannel>    arbiter6{};
        MuxDev                                       mux6{};
        Kvasir::I2C::MuxGate<MuxDev, 0, HoldChannel> first{};
        Kvasir::I2C::MuxGate<MuxDev, 0, HoldChannel> second{};
        Kvasir::I2C::MuxGate<MuxDev, 3, HoldChannel> other{};
        first.bind(mux6, arbiter6);
        second.bind(mux6, arbiter6);
        other.bind(mux6, arbiter6);
        check(settle(first, mux6), "the first part gets channel 0");
        check(second.claim(), "a second part of channel 0 joins it");
        check(!other.claim(), "channel 3 waits");
        first.release();
        check(!other.claim(), "and still waits while the second part is mid-script");
        checkEq(arbiter6.holders, std::uint8_t{1}, "one part of channel 0 still holds it");
        check(first.claim(),
              "a part of the holding channel may join again, even while channel 3 waits");
        first.release();
        second.release();
        checkEq(arbiter6.holder,
                BasicMuxArbiter<HoldChannel>::NoHolder,
                "free after the last release");
        check(settle(other, mux6), "and channel 3 gets it");
    }

    testCase("Mux: DrainChannel lets no part join while another channel waits");
    fresh();
    SwitchWire wire10{};
    FakeBus::respond = std::ref(wire10);
    {
        Kvasir::I2C::BasicMuxArbiter<DrainChannel>    arbiter7{};
        MuxDev                                        mux7{};
        Kvasir::I2C::MuxGate<MuxDev, 0, DrainChannel> first{};
        Kvasir::I2C::MuxGate<MuxDev, 0, DrainChannel> second{};
        Kvasir::I2C::MuxGate<MuxDev, 3, DrainChannel> other{};
        first.bind(mux7, arbiter7);
        second.bind(mux7, arbiter7);
        other.bind(mux7, arbiter7);
        check(settle(first, mux7), "the first part gets channel 0");
        check(second.claim(), "with nobody waiting, a second part of channel 0 joins");
        check(!other.claim(), "channel 3 waits");
        first.release();
        check(!first.claim(), "now no part of channel 0 joins");
        second.release();
        check(settle(other, mux7), "the last release hands the switch to channel 3");
    }

    testCase("Mux: DrainChannel forgets a wait its client gave up");
    fresh();
    SwitchWire wire11{};
    FakeBus::respond = std::ref(wire11);
    {
        Kvasir::I2C::BasicMuxArbiter<DrainChannel>    arbiter8{};
        MuxDev                                        mux8{};
        Kvasir::I2C::MuxGate<MuxDev, 0, DrainChannel> first{};
        Kvasir::I2C::MuxGate<MuxDev, 0, DrainChannel> second{};
        Kvasir::I2C::MuxGate<MuxDev, 3, DrainChannel> other{};
        first.bind(mux8, arbiter8);
        second.bind(mux8, arbiter8);
        other.bind(mux8, arbiter8);
        check(settle(first, mux8), "the first part gets channel 0");
        check(!other.claim(), "channel 3 waits");
        other.release();   // and gives up, as a scan that timed out does
        checkEq(arbiter8.waiting, std::uint8_t{0}, "the wait is gone from the arbiter");
        check(second.claim(), "so a second part of channel 0 joins again");
    }
    if(failures != 0) { dump(); }
}

/// Transactions to `address` in the transcript from entry `from` on.
std::size_t transactionsTo(std::uint8_t address,
                           std::size_t  from = 0) {
    std::size_t n = 0;
    for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
        if(FakeBus::log[i].isBus() && FakeBus::log[i].address == address) { ++n; }
    }
    return n;
}

void muxFaults() {
    using namespace MuxTest;
    using Channels = Chips::Tca9548a::Channels;

    testCase("Mux: a switch that browns out is set again within a verify interval");
    fresh();
    SwitchWire wire{.behind = crcZeros};
    FakeBus::respond = std::ref(wire);
    Kvasir::I2C::MuxArbiter arbiter{};
    MuxDev                  mux{};
    Left                    left{};
    left.gate().bind(mux, arbiter);
    check(runUntil(
            all(mux, left),
            [&] { return left.valid(); },
            2s),
          "the part behind channel 0 is up");
    checkEq(wire.control[MuxDev::Address], 0x01, "with the switch on its channel");
    checkEq(mux.mismatches<Channels>(), 0U, "and every read-back so far matched");
    // The switch loses its register -- a brown-out -- while the engine believes channel 0.
    wire.control[MuxDev::Address] = 0x00;
    auto const from               = FakeBus::log.size();
    check(runUntil(
            all(mux, left),
            [&] { return wire.control[MuxDev::Address] == 0x01; },
            600ms),
          "written again within the 500 ms verify interval");
    check(mux.mismatches<Channels>() >= 1, "because the read-back caught it");
    check(hasWrite({0x01}, from), "with the channel byte the engine holds");

    testCase("Mux: a switch that NAKs parks, its parts wait, and both come back after a probe");
    fresh();
    bool       muxThere = false;
    SwitchWire wire2{.behind = crcZeros};
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(a == MuxDev::Address && !muxThere) { return FakeBus::Result::notAcknowledged; }
        return wire2(a, s, r);
    };
    Kvasir::I2C::MuxArbiter arbiter2{};
    MuxDev                  mux2{};
    Left                    left2{};
    left2.gate().bind(mux2, arbiter2);
    check(runUntil(
            all(mux2, left2),
            [&] { return mux2.absent(); },
            3s),
          "the switch is parked after its NAKs");
    checkEq(transactionsTo(Chips::Sht3x::Address),
            std::size_t{0},
            "the part behind it never got the switch, so it never talked");
    check(left2.link() == Link::starting, "and is starting, not absent: nothing was asked of it");
    runFor(all(mux2, left2), 500ms);
    checkEq(transactionsTo(Chips::Sht3x::Address),
            std::size_t{0},
            "and still nothing while the switch is away");
    muxThere = true;
    check(runUntil(
            all(mux2, left2),
            [&] { return left2.valid(); },
            5s),
          "up once the switch answered its probe and was set for the channel");
    check(!mux2.absent() && mux2.answering(), "the switch is back");
    {
        std::uint8_t selected = 0xFF;
        std::size_t  bad      = 0;
        for(auto const& tr : FakeBus::log) {
            if(!tr.isBus()) { continue; }
            if(tr.address == MuxDev::Address && tr.isWrite() && tr.sent.size() == 1) {
                selected = tr.sent[0];
            } else if(tr.address == Chips::Sht3x::Address && selected != 0x01) {
                ++bad;
            }
        }
        checkEq(bad,
                std::size_t{0},
                "and every transaction to the part came behind a switch write");
    }

    testCase("Mux: a NAK in the middle of a script lets the switch go at once");
    fresh();
    bool       nakRead = false;
    SwitchWire wire3{.behind = crcZeros};
    FakeBus::respond = [&](std::uint8_t a, std::span<std::byte const> s, std::span<std::byte> r) {
        if(a == Chips::Sht3x::Address && nakRead && !r.empty()) {
            nakRead = false;
            return FakeBus::Result::notAcknowledged;
        }
        return wire3(a, s, r);
    };
    Kvasir::I2C::MuxArbiter         arbiter3{};
    MuxDev                          mux3{};
    Left                            left3{};
    Kvasir::I2C::MuxGate<MuxDev, 3> other{};
    left3.gate().bind(mux3, arbiter3);
    other.bind(mux3, arbiter3);
    check(runUntil(all(mux3, left3), [&] { return left3.valid(); }, 2s), "up and measuring");
    // The next result read (the second step of a measurement) is NAKed: the turn it is
    // submitted in completes it, and the part learns of it on its next turn.
    nakRead = true;
    check(runUntil(all(mux3, left3), [&] { return !nakRead; }, 2s), "the result read was NAKed");
    checkEq(arbiter3.holder, std::uint8_t{0}, "channel 0 held the switch for that read");
    left3.handler();   // takes the NAK
    checkEq(arbiter3.holder,
            Kvasir::I2C::MuxArbiter::NoHolder,
            "the NAK ended the run, and the run let the switch go");
    check(settle(other, mux3), "so the other channel gets it at once");
    other.release();
    checkEq(left3.errors(), 1U, "one error for the part");
    check(runUntil(
            all(mux3, left3),
            [&] { return left3.seq() > 0 && left3.answering(); },
            2s),
          "which carries on");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    muxGate();
    muxFaults();
    return finish();
}
