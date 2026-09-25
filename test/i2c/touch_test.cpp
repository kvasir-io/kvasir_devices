/// The touch controllers behind Kvasir::I2C::Touch::Controller: reset, identification, the
/// frames, the poll cadence, the INT line, and one lost read.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/TouchController.hpp>
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

// -- the touch controllers ------------------------------------------------------------------

namespace TouchFakes {   // not "Touch": that is Kvasir::I2C::Touch, visible here
    namespace TD = Kvasir::I2C::Chips::TouchDetail;

    /// A touch controller on the wire: the registers its Init script reads, one settable data
    /// frame, and the writes it was sent.
    struct Model {
        std::uint8_t                                       address{};
        std::size_t                                        regBytes{1};
        std::uint16_t                                      dataReg{};
        std::map<std::uint16_t, std::vector<std::uint8_t>> regs{};
        std::vector<std::uint8_t>                          frame{};
        std::vector<std::vector<std::uint8_t>>             writes{};
        int                                                nak{0};   ///< NAK the next n requests

        FakeBus::Result operator()(std::uint8_t               addr,
                                   std::span<std::byte const> sent,
                                   std::span<std::byte>       recv) {
            if(addr != address) { return FakeBus::Result::notAcknowledged; }
            if(nak > 0) {
                --nak;
                return FakeBus::Result::notAcknowledged;
            }
            std::uint16_t reg = 0;
            for(std::size_t i = 0; i < regBytes && i < sent.size(); ++i) {
                reg = static_cast<std::uint16_t>((reg << 8) | static_cast<std::uint8_t>(sent[i]));
            }
            if(recv.empty()) {
                std::vector<std::uint8_t> w;
                for(auto const b : sent) { w.push_back(static_cast<std::uint8_t>(b)); }
                writes.push_back(w);
                return FakeBus::Result::succeeded;
            }
            auto const& src = reg == dataReg ? frame : regs[reg];
            for(std::size_t i = 0; i < recv.size(); ++i) {
                recv[i] = static_cast<std::byte>(i < src.size() ? src[i] : 0);
            }
            return FakeBus::Result::succeeded;
        }
    };

    struct Cfg : Kvasir::I2C::Touch::Defaults {
        static constexpr std::uint16_t Width  = 466;
        static constexpr std::uint16_t Height = 466;
    };

    struct FlippedCfg : Cfg {
        static constexpr auto FlipX = Kvasir::I2C::Touch::Mirror::mirrored;
        static constexpr auto FlipY = Kvasir::I2C::Touch::Mirror::mirrored;
    };

    struct SwappedCfg : Kvasir::I2C::Touch::Defaults {
        static constexpr std::uint16_t Width  = 200;
        static constexpr std::uint16_t Height = 100;
        static constexpr auto          SwapXY = Kvasir::I2C::Touch::Axes::swapped;
    };

    template<typename Ctrl, typename C = Cfg>
    using TouchDev = Kvasir::I2C::Touch::Controller<FakeBus, FakeClock, Ctrl, C, FakeResetLine>;

    // The transform is pure and Config-driven, so it is checked at compile time.
    static_assert(TouchDev<TD::Cst9217>::transform(10,
                                                   20)
                  == std::pair<std::uint16_t,
                               std::uint16_t>{10,
                                              20});
    static_assert(TouchDev<TD::Cst9217,
                           FlippedCfg>::transform(0,
                                                  0)
                  == std::pair<std::uint16_t,
                               std::uint16_t>{465,
                                              465});
    static_assert(TouchDev<TD::Cst9217,
                           FlippedCfg>::transform(9999,
                                                  0)
                    == std::pair<std::uint16_t,
                                 std::uint16_t>{0,
                                                465},
                  "clamped before the mirror");
    static_assert(TouchDev<TD::Cst9217,
                           SwappedCfg>::transform(30,
                                                  150)
                    == std::pair<std::uint16_t,
                                 std::uint16_t>{150,
                                                30},
                  "swapped, then clamped to the swapped extents");

    /// The reset edges and the transactions, as text, for a failing case.
    std::string wire() {
        std::string out;
        for(auto const& t : FakeBus::log) {
            if(t.kind == Transaction::Kind::hold) {
                out += " [hold]";
                continue;
            }
            if(t.kind == Transaction::Kind::release) {
                out += " [release]";
                continue;
            }
            out += t.isRead() ? " R" : " W";
            for(auto const b : t.sent) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "%02x", b);
                out += buf;
            }
        }
        return out;
    }
}   // namespace TouchFakes

void touch() {
    using namespace TouchFakes;

    testCase("CST9217: reset, init script, identification");
    fresh();
    Model m9217{0x5A, 2, 0xD000};
    m9217.regs[0xD1FC] = {0x20, 0x4E, 0xCA, 0x00};   // checkcode
    m9217.regs[0xD1F8] = {0xD2, 0x01, 0xD2, 0x01};   // 466 x 466
    m9217.regs[0xD204] = {0x08, 0x00, 0x17, 0x92};   // project id, 0x9217
    m9217.frame
      = {0xFF, 0xFF, 0xFF, 0xFF, 0x08, 0x00, 0x08, 0x00, 0xD2, 0x01, 0xD2, 0x01, 0x20, 0x4E, 0xCA};
    FakeBus::respond = std::ref(m9217);
    TouchDev<TD::Cst9217> t9217{};
    check(runUntil(t9217, [&] { return t9217.answering(); }, 500ms), "up");
    check(FakeBus::log.size() >= 3 && FakeBus::log[0].kind == Transaction::Kind::hold
            && FakeBus::log[1].kind == Transaction::Kind::release,
          "the reset line is pulsed before any transaction");
    check(FakeBus::log[1].at - FakeBus::log[0].at >= TD::Cst9217::ResetLow, "held for ResetLow");
    check(FakeBus::log[2].at - FakeBus::log[1].at >= TD::Cst9217::ResetSettle, "then ResetSettle");
    check(FakeBus::log[2].address == 0x5A, "at 0x5A");
    check(hasWrite({0xD1, 0x01}), "the vendor's address-only init write");
    check(t9217.chipInfo().ok && t9217.chipInfo().chip == 0x9217, "identified");
    checkEq(t9217.chipInfo().width, std::uint16_t{466}, "and its own resolution");
    check(t9217.identified(), "the policy accepts the chip");
    if(failures != 0) { std::printf("    wire:%s\n", wire().c_str()); }

    testCase("CST9217: the resting frame is no finger, and says nothing new");
    check(runUntil(t9217, [&] { return t9217.quiet() >= 2; }, 500ms), "quiet frames counted");
    checkEq(t9217.reports(), 0U, "nothing reported: the glass was never touched");
    checkEq(t9217.latest().seq, 0U, "so seq never stepped");
    checkEq(t9217.malformed(), 0U, "and the resting frame is not malformed");

    testCase("CST9217: a finger, a move, then the lift frame");
    // status 6 (touching), x = 0x12 << 4 | 0x3 = 0x123 (291), y = 0x1C << 4 | 0x2 = 0x1C2 (450)
    m9217.frame
      = {0x06, 0x12, 0x1C, 0x32, 0x00, 0x01, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    check(runUntil(t9217, [&] { return t9217.latest().points != 0; }, 500ms), "a finger");
    checkEq(t9217.latest().x, std::uint16_t{0x123}, "x from three nibbles");
    checkEq(t9217.latest().y, std::uint16_t{0x1C2}, "y from three nibbles");
    checkEq(t9217.latest().points, std::uint8_t{1}, "one finger");
    check(t9217.latest().at != FakeClock::time_point{}, "stamped at the bus completion");
    auto const heldSeq = t9217.latest().seq;
    check(runUntil(
            t9217,
            [&] { return t9217.latest().seq > heldSeq; },
            200ms),
          "a motionless finger still reports, so a page can time a hold");
    // the lift: acked, finger still in the slot, status nibble 0
    m9217.frame[0] = 0x00;
    check(runUntil(t9217, [&] { return t9217.latest().points == 0; }, 500ms), "the lift");
    checkEq(t9217.latest().x, std::uint16_t{0x123}, "a release keeps where the finger left");
    auto const liftSeq     = t9217.latest().seq;
    auto const quietBefore = t9217.quiet();
    runFor(t9217, 500ms);
    checkEq(t9217.latest().seq, liftSeq, "and the frames after it say nothing new");
    check(t9217.quiet() > quietBefore, "counted as quiet, not as reports");

    testCase("CST9217: poll cadence follows the reading");
    fresh();
    FakeBus::respond = std::ref(m9217);
    m9217.frame
      = {0xFF, 0xFF, 0xFF, 0xFF, 0x08, 0x00, 0x08, 0x00, 0xD2, 0x01, 0xD2, 0x01, 0x20, 0x4E, 0xCA};
    TouchDev<TD::Cst9217> tc{};
    check(runUntil(tc, [&] { return tc.answering(); }, 500ms), "up");
    auto reads = [](std::size_t from) {
        std::size_t n = 0;
        for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
            if(FakeBus::log[i].isRead()) { ++n; }
        }
        return n;
    };
    auto base = FakeBus::log.size();
    runFor(tc, 1s);
    auto const idle = reads(base);
    check(idle >= 9 && idle <= 11, "about ten reads a second with the glass idle");
    m9217.frame
      = {0x06, 0x12, 0x1C, 0x32, 0x00, 0x01, 0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    check(runUntil(tc, [&] { return tc.latest().points != 0; }, 500ms), "a finger goes down");
    base = FakeBus::log.size();
    runFor(tc, 1s);
    auto const held = reads(base);
    check(held >= 18 && held <= 21, "and about twenty a second while it is down");

    testCase("CST9217: the INT line asks for a read between polls");
    base = FakeBus::log.size();
    turn(tc);
    tc.interrupt();
    turn(tc);
    turn(tc);
    check(reads(base) >= 1, "a read went out without waiting for the period");

    testCase("GT911: the stale bit, and the acknowledge write after every frame read");
    fresh();
    Model mgt{0x5D, 2, 0x814E};
    mgt.regs[0x8140] = {'9', '1', '1', 0x00, 0x60, 0x10};
    mgt.regs[0x8048] = {0xD2, 0x01, 0xD2, 0x01};
    mgt.frame        = {0x00, 0, 0, 0, 0, 0, 0, 0, 0};   // bit 7 clear: nothing new
    FakeBus::respond = std::ref(mgt);
    TouchDev<TD::Gt911> gt{};
    check(runUntil(gt, [&] { return gt.answering(); }, 500ms), "up");
    check(gt.chipInfo().ok && gt.chipInfo().chip == 0x393131, "identified by its ASCII id");
    check(runUntil(gt, [&] { return gt.quiet() >= 2; }, 500ms), "the stale frames are quiet");
    checkEq(gt.malformed(), 0U, "a stale frame is not a fault");
    checkEq(gt.reports(), 0U, "and not a report");
    check(std::ranges::none_of(
            mgt.writes,
            [](auto const& w) { return w == std::vector<std::uint8_t>{0x81, 0x4E, 0x00}; }),
          "no 00 to 814E after a stale read: it could clear a frame that became ready since");
    auto const acksBefore = mgt.writes.size();
    // a finger: status 0x81 (new data, one point), x = 0x0123, y = 0x01C2 little-endian
    mgt.frame = {0x81, 0x00, 0x23, 0x01, 0xC2, 0x01, 0x00, 0x00, 0x00};
    check(runUntil(gt, [&] { return gt.latest().points != 0; }, 500ms), "a finger");
    checkEq(gt.latest().x, std::uint16_t{0x123}, "x little-endian");
    checkEq(gt.latest().y, std::uint16_t{0x1C2}, "y little-endian");
    check(mgt.writes.size() > acksBefore
            && std::ranges::any_of(
              mgt.writes,
              [](auto const& w) { return w == std::vector<std::uint8_t>{0x81, 0x4E, 0x00}; }),
          "814E is written 00 after a frame, or the chip never raises INT again");

    testCase("GT911: a frame that cannot be right is counted and changes nothing");
    mgt.frame         = {0x8F, 0, 0, 0, 0, 0, 0, 0, 0};   // fifteen fingers on a five-point chip
    auto const wasSeq = gt.latest().seq;
    check(runUntil(gt, [&] { return gt.malformed() >= 1; }, 500ms), "counted as malformed");
    checkEq(gt.latest().seq, wasSeq, "and nothing was published");

    testCase("CST816S and FT6x36: bring-up and one finger");
    fresh();
    Model m816{0x15, 1, 0x02};
    m816.regs[0xA7]  = {0xB4};
    m816.regs[0xA9]  = {0x03};
    m816.frame       = {0x01, 0x01, 0x23, 0x01, 0xC2};   // one finger, event down
    FakeBus::respond = std::ref(m816);
    TouchDev<TD::Cst816s> t816{};
    check(runUntil(t816, [&] { return t816.answering(); }, 800ms), "up");
    check(hasWrite({0xFE, 0x01}) && hasWrite({0xFA, 0x60}), "DisAutoSleep and IrqCtl");
    check(t816.chipInfo().ok && t816.chipInfo().chip == 0xB4, "a CST816S");
    check(runUntil(t816, [&] { return t816.latest().points != 0; }, 500ms), "a finger");
    checkEq(t816.latest().x, std::uint16_t{0x123}, "x");
    checkEq(t816.latest().y, std::uint16_t{0x1C2}, "y");

    fresh();
    Model m636{0x38, 1, 0x02};
    m636.regs[0xA3]  = {0x36, 0x00, 0x00, 0x10, 0x00, 0x11};   // chip 0x36, vendor 0x11
    m636.frame       = {0x01, 0x01, 0x23, 0x01, 0xC2, 0x00, 0x00};
    FakeBus::respond = std::ref(m636);
    TouchDev<TD::Ft6x36> t636{};
    check(runUntil(t636, [&] { return t636.answering(); }, 1s), "up");
    check(t636.chipInfo().ok && t636.chipInfo().extra == 0x11, "identified by the vendor id");
    // an FT5x06-style panel: a panel maker's model code in A8, a FocalTech chip id in A3
    static_assert([] {
        auto const panel = TD::Ft6x36::identify(Bytes{frame(0x64, 0x00, 0x00, 0x10, 0x00, 0x43)});
        auto const other = TD::Ft6x36::identify(Bytes{frame(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF)});
        return panel.ok && !other.ok;
    }());
    check(runUntil(t636, [&] { return t636.latest().points != 0; }, 500ms), "a finger");
    checkEq(t636.latest().x, std::uint16_t{0x123}, "x");

    testCase("touch: one lost data read costs one report, not the bring-up");
    auto const beforeErr   = t636.errors();
    auto const beforeSeq   = t636.latest().seq;
    auto const holdsBefore = std::ranges::count_if(FakeBus::log, [](auto const& t) {
        return t.kind == Transaction::Kind::hold;
    });
    m636.nak               = 1;
    check(runUntil(t636, [&] { return t636.errors() > beforeErr; }, 500ms), "the NAK was counted");
    check(t636.answering(), "the controller is not taken down for one lost frame");
    check(std::ranges::count_if(FakeBus::log,
                                [](auto const& t) { return t.kind == Transaction::Kind::hold; })
            == holdsBefore,
          "and its reset line was not pulsed");
    check(runUntil(
            t636,
            [&] { return t636.latest().seq > beforeSeq; },
            500ms),
          "the next read reports as usual");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    touch();
    return finish();
}
