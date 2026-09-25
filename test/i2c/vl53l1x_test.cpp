/// The VL53L1X description against the fake bus: the ULD bring-up on the wire in ULD's order,
/// the default configuration split into writes of consecutive registers, the data-ready poll,
/// the result decode and the interrupt clear.
#include "Harness.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Vl53l1x.hpp>
#include <span>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

using Tof = Device<FakeBus, FakeClock, Chips::Vl53l1x>;

// The engine carries the configuration eight bytes at a time: nothing over Step::InlineBytes.
static_assert(Tof::MaxPayload == Step::InlineBytes,
              "no write over eight bytes");
static_assert(
  Tof::InitSteps.size() == 27,
  "the engine's two identity reads and their comparison, then the description's own: the "
  "boot state, the identify step, the soft reset pair, twelve config writes, eight ULD steps");
static_assert(Tof::RegisterBytes == 2,
              "sixteen-bit register indices");

/// Index of the first write whose bytes are exactly `bytes`, from `from`; log size if none.
std::size_t readsOf(std::uint16_t reg) {
    std::size_t n = 0;
    for(auto const& t : FakeBus::log) {
        if(t.isRead()
           && t.sent
                == std::vector<std::uint8_t>{static_cast<std::uint8_t>(reg >> 8),
                                             static_cast<std::uint8_t>(reg & 0xFF)})
        {
            ++n;
        }
    }
    return n;
}

void vl53l1x() {
    testCase("VL53L1X");
    fresh();
    RegisterModel<2> m{0x29};
    m.set(0x010F, {0xEA, 0xCC});   // model id
    m.set(0x00E5, {0x01});         // booted
    m.set(0x0031, {0x03});         // new sample flag (bit 0) set
    // RESULT__RANGE_STATUS 0x0089 .. 0x0099: raw status 9 (ULD 0), 0x20 SPADs, ambient 0x0010,
    // distance 1234 mm, signal 0x0100
    m.set(0x0089,
          {0x09,
           0x00,
           0x00,
           0x20,
           0x00,
           0x00,
           0x00,
           0x00,
           0x10,
           0x00,
           0x00,
           0x00,
           0x00,
           0x04,
           0xD2,
           0x01,
           0x00});
    m.readOnly = {0x010F, 0x0110, 0x00E5, 0x0031};
    for(std::uint32_t r = 0x0089; r <= 0x0099; ++r) { m.readOnly.push_back(r); }
    FakeBus::respond = std::ref(m);

    Tof d{};
    check(runUntil(d, [&] { return d.valid(); }, 2s), "first sample");
    check(d.identified(), "model id 0xEACC and booted");
    checkEq(d.state().modelId, 0xEACC, "model id kept");
    {
        auto const low  = findWrite({0x00, 0x00, 0x00});
        auto const high = findWrite({0x00, 0x00, 0x01}, low);
        auto const conf = findWrite({0x00, 0x87, 0x40}, high);
        check(low < high && high < conf && conf < FakeBus::log.size(),
              "SOFT_RESET 0 then 1, before the configuration and ranging");
        check(low > 1 && FakeBus::log[0].isRead() && FakeBus::log[1].isRead(),
              "and only after the model id and boot flag were read");
    }

    // the configuration: 0x2D..0x87 in writes of at most eight consecutive registers
    {
        std::array<int, 91> got{};
        got.fill(-1);
        std::size_t chunks = 0;
        for(auto const& t : FakeBus::log) {
            if(!t.isWrite() || t.sent.size() < 4) { continue; }   // register + > 1 byte
            auto const reg = static_cast<std::uint16_t>((t.sent[0] << 8) | t.sent[1]);
            if(reg < 0x2D || reg > 0x87) { continue; }
            ++chunks;
            check(t.sent.size() - 2 <= 8, "a config write carries at most eight bytes");
            for(std::size_t i = 2; i < t.sent.size(); ++i) { got[reg - 0x2D + i - 2] = t.sent[i]; }
        }
        checkEq(chunks, std::size_t{12}, "twelve config writes");
        bool same = true;
        for(std::size_t i = 0; i < got.size(); ++i) {
            same = same && got[i] == Chips::Vl53l1xDetail::DefaultConfiguration[i];
        }
        check(same, "every byte of ULD's default configuration, at its register");
    }

    // ULD's order: range once, clear, stop, the VHV settings, then ranging for good
    auto const firstStart = findWrite({0x00, 0x87, 0x40});
    auto const stop       = findWrite({0x00, 0x87, 0x00}, firstStart);
    auto const vhv        = findWrite({0x00, 0x08, 0x09}, stop);
    auto const vhvTemp    = findWrite({0x00, 0x0B, 0x00}, vhv);
    auto const start      = findWrite({0x00, 0x87, 0x40}, vhvTemp);
    check(firstStart < stop && stop < vhv && vhv < vhvTemp && vhvTemp < start
            && start < FakeBus::log.size(),
          "start, stop, VHV bound 0x09, 0x000B = 0, start");
    check(FakeBus::log[stop].at - FakeBus::log[firstStart].at >= Chips::Vl53l1x::FirstRanging,
          "and the first ranging is given its time");

    check(d.latest().distance == Kvasir::Units::milliMetre(1234),
          "the model's frame reached the decode: distance, big-endian at 0x0096");
    checkEq(d.latest().signalRate,
            Units::hertz(2048000U),
            "signal 0x0100 from the end of the result read at offset 2: a shifted buffer "
            "would give 0x04D2 or 0x0000");
    // GPIO_HV_MUX__CTRL 0x00, TIO_HV_STATUS 0x03, then the seventeen result bytes above: raw
    // status 9 is ULD's 0, ambient 0x0010 and signal 0x0100 x 8 kcps as hertz, 1234 mm
    static_assert([] {
        auto const f   = frame(0x00,
                               0x03,
                               0x09,
                               0x00,
                               0x00,
                               0x20,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x10,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x04,
                               0xD2,
                               0x01,
                               0x00);
        auto const got = Chips::Vl53l1x::Ranging::decode(Bytes{f});
        return isOk(got) && got.value.distance == Kvasir::Units::milliMetre(1234)
            && equal(got.value.status, 0) && got.value.valid()
            && equal(got.value.ambientRate, Units::hertz(128000U))
            && equal(got.value.signalRate, Units::hertz(2048000U));
    }());
    check(findWrite({0x00, 0x86, 0x01}, start) < FakeBus::log.size(),
          "the interrupt cleared after the result");

    // another status: raw 4 is ULD's 2, signal failure
    m.readOnly.clear();
    m.set(0x0089, {0x04});
    m.readOnly   = {0x010F, 0x0110, 0x00E5, 0x0031, 0x0089};
    auto const n = d.samples();
    check(runUntil(d, [&] { return d.samples() > n; }, 500ms), "next sample");
    checkEq(d.latest().status, 2, "the new frame reached the decode: raw 4 is signal failure");
    // the same frame with raw status 4: ULD's 2, signal failure, a sample that is not valid
    static_assert([] {
        auto const f   = frame(0x00,
                               0x03,
                               0x04,
                               0x00,
                               0x00,
                               0x20,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x10,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x04,
                               0xD2,
                               0x01,
                               0x00);
        auto const got = Chips::Vl53l1x::Ranging::decode(Bytes{f});
        return isOk(got) && equal(got.value.status, 2) && !got.value.valid();
    }());
    // a raw status ULD has no mapping for (0 is its 255) is a frame that cannot be read: rejected
    static_assert([] {
        auto const f = frame(0x00,
                             0x03,
                             0x00,
                             0x00,
                             0x00,
                             0x20,
                             0x00,
                             0x00,
                             0x00,
                             0x00,
                             0x10,
                             0x00,
                             0x00,
                             0x00,
                             0x00,
                             0x04,
                             0xD2,
                             0x01,
                             0x00);
        return isReject(Chips::Vl53l1x::Ranging::decode(Bytes{f}));
    }());

    // no data-ready flag: the result is never read and the run is rejected
    m.readOnly.clear();
    m.set(0x0031, {0x02});
    m.readOnly         = {0x010F, 0x0110, 0x00E5, 0x0031};
    auto const results = readsOf(0x0089);
    auto const samples = d.samples();
    runFor(d, 600ms);
    check(d.rejected() > 0, "rejected after the retries");
    checkEq(d.samples(), samples, "no sample");
    checkEq(readsOf(0x0089), results, "the result is not read without the flag");

    // ROI
    auto const from = FakeBus::log.size();
    d.set<Chips::Vl53l1x::Roi>({8, 8, 199});
    check(runUntil(d, [&] { return !d.pending(); }, 200ms), "ROI written");
    check(findWrite({0x00, 0x7F, 0xC7, 0x77}, from) < FakeBus::log.size(), "8 x 8 about 199");
    auto const from2 = FakeBus::log.size();
    d.set<Chips::Vl53l1x::Roi>({20, 2, 0x55});
    check(runUntil(d, [&] { return !d.pending(); }, 200ms), "ROI written again");
    check(findWrite({0x00, 0x7F, 0xC7, 0x3F}, from2) < FakeBus::log.size(),
          "16 x 4 after clamping, centre forced to 199");

    // a wrong model id: the bring-up is turned down, nothing else runs, and it is tried again
    // a second later -- and a part that gets its id right by then comes up
    fresh();
    RegisterModel<2> other{0x29};
    other.set(0x010F, {0xEE, 0xAA});
    other.set(0x00E5, {0x01});
    other.set(0x0031, {0x03});   // data ready
    other.set(0x0089, {0x09});   // range status 9: ULD's 0
    other.readOnly = {0x010F, 0x0110, 0x00E5, 0x0031};
    for(std::uint32_t r = 0x0089; r <= 0x0099; ++r) { other.readOnly.push_back(r); }
    FakeBus::respond = std::ref(other);
    Tof o{};
    check(runUntil(
            o,
            [&] { return o.unidentified() == 1; },
            3s),
          "the bring-up read the id and turned it down");
    check(!o.identified() && o.link() == Link::starting,
          "0xEEAA is not a VL53L1X: starting, not answering");
    {
        auto const reads  = other.reads;
        auto const writes = other.writes;
        runFor(o, 900ms);
        check(other.reads == reads && other.writes == writes,
              "no ranging: nothing is sent to it meanwhile");
        checkEq(o.samples(), 0U, "and no sample");
        other.set(0x010F, {0xEA, 0xCC});   // it was still booting: the id is right now
        check(runUntil(
                o,
                [&] { return other.reads > reads; },
                300ms),
              "the bring-up is tried again a second later");
    }
    check(runUntil(o, [&] { return o.valid(); }, 3s), "and the part comes up and ranges");
    check(o.identified(), "identified");
    checkEq(o.unidentified(), 1U, "turned down once");

    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    vl53l1x();
    return finish();
}
