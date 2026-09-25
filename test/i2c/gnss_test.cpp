/// The u-blox M8 on its DDC port (chips/SamM8q.hpp) against a model of the receiver's three
/// registers and its stream, and the UBX framer (I2C/Ubx.hpp) over what the Stream group reads.
#include "Harness.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Ubx.hpp>
#include <kvasir/Devices/I2C/chips/SamM8q.hpp>
#include <span>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

namespace M8 = Chips::UbloxM8Detail;

/// The receiver's DDC port (protocol spec 11.5): a write of one byte sets the register pointer,
/// a longer one is a message; a read walks 0xFD (the waiting count, high byte), 0xFE (low byte)
/// and then stays at 0xFF, which hands out the stream, 0xFF when it is empty.
struct M8Model {
    std::uint8_t                           address{0x42};
    std::uint8_t                           pointer{0xFF};
    std::deque<std::uint8_t>               stream{};
    std::vector<std::vector<std::uint8_t>> messages{};

    void queue(std::span<std::uint8_t const> bytes) {
        for(auto const b : bytes) { stream.push_back(b); }
    }

    FakeBusResult operator()(std::uint8_t               addr,
                             std::span<std::byte const> sent,
                             std::span<std::byte>       recv) {
        if(addr != address) { return FakeBusResult::notAcknowledged; }
        if(sent.size() == 1) {
            pointer = static_cast<std::uint8_t>(sent[0]);
        } else if(sent.size() >= 2) {
            std::vector<std::uint8_t> m;
            for(auto const b : sent) { m.push_back(static_cast<std::uint8_t>(b)); }
            messages.push_back(m);
            return FakeBusResult::succeeded;
        }
        auto const count = static_cast<std::uint16_t>(stream.size());
        for(auto& b : recv) {
            if(pointer == 0xFD) {
                b       = std::byte{static_cast<std::uint8_t>(count >> 8U)};
                pointer = 0xFE;
            } else if(pointer == 0xFE) {
                b       = std::byte{static_cast<std::uint8_t>(count & 0xFFU)};
                pointer = 0xFF;
            } else if(pointer == 0xFF && !stream.empty()) {
                b = std::byte{stream.front()};
                stream.pop_front();
            } else {
                b = std::byte{0xFF};
            }
        }
        return FakeBusResult::succeeded;
    }
};

/// A UBX-NAV-PVT payload (32.17.17.1), little endian, with every field decodeNavPvt reads set.
constexpr std::array<std::uint8_t,
                     92>
navPvtPayload() {
    std::array<std::uint8_t, 92> p{};
    auto const                   put = [&](std::size_t at, std::uint32_t v, std::size_t n) {
        for(std::size_t i = 0; i < n; ++i) { p[at + i] = static_cast<std::uint8_t>(v >> (8U * i)); }
    };
    put(0, 123456000, 4);                                 // iTOW
    put(4, 2026, 2);                                      // year
    p[6]  = 9;                                            // month
    p[7]  = 16;                                           // day
    p[8]  = 12;                                           // hour
    p[9]  = 34;                                           // min
    p[10] = 56;                                           // sec
    p[11] = 0x07;                                         // validDate, validTime, fullyResolved
    put(12, 25, 4);                                       // tAcc 25 ns
    put(16, static_cast<std::uint32_t>(-123456789), 4);   // nano
    p[20] = 3;                                            // 3-D fix
    p[21] = 0x01;                                         // gnssFixOK
    p[23] = 11;                                           // numSV
    put(24, 85417009, 4);                                 // lon  8.5417009 deg
    put(28, static_cast<std::uint32_t>(-473769005), 4);   // lat -47.3769005 deg
    put(32, 500123, 4);                                   // height 500.123 m
    put(36, 452000, 4);                                   // hMSL 452 m
    put(40, 1500, 4);                                     // hAcc 1.5 m
    put(44, 0xFFFFFFFFU, 4);                              // vAcc: past int32
    put(60, 1234, 4);                                     // gSpeed 1.234 m/s
    put(64, 12345678, 4);                                 // headMot 123.45678 deg
    put(76, 135, 2);                                      // pDOP 1.35
    return p;
}

constexpr std::array<std::byte,
                     92>
asBytes(std::array<std::uint8_t,
                   92> const& p) {
    std::array<std::byte, 92> b{};
    for(std::size_t i = 0; i < p.size(); ++i) { b[i] = std::byte{p[i]}; }
    return b;
}

static_assert([] {
    auto const bytes = asBytes(navPvtPayload());
    auto const n     = M8::decodeNavPvt(bytes);
    using namespace std::chrono;
    return n.iTow == 123456000ms && n.date == year{2026} / September / day{16} && n.hour == 12h
        && n.minute == 34min && n.second == 56s && n.nano == nanoseconds{-123456789}
        && n.tAcc == 25ns && n.validDate && n.validTime && n.fullyResolved
        && n.fixType == M8::FixType::fix3d && n.fixOk && !n.invalidLlh && n.numSv == 11
        && equal(n.lon, 8541700)     // 1e-7 degree / 10, truncated
        && equal(n.lat, -47376900)   // towards zero
        && equal(n.height, 500123) && equal(n.hMsl, 452000) && equal(n.hAcc, 1500)
        && equal(n.vAcc, 2147483647)   // the U4 clamped
        && equal(n.gSpeed, 1234) && equal(n.headMot, 12345) && n.pDop == 135;
}());

static_assert(
  [] {
      auto const bytes = asBytes(navPvtPayload());
      auto const n     = M8::decodeNavPvt(std::span<std::byte const>{bytes}.first(91));
      return n.fixType == M8::FixType::noFix && !n.validDate && !n.fixOk && n.numSv == 0;
  }(),
  "a payload short of 92 bytes is no solution");

constexpr auto PortFrame = std::array<std::uint8_t, 28>{
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x96};
constexpr auto MsgFrame
  = std::array<std::uint8_t, 11>{0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
constexpr auto RateFrame = std::array<std::uint8_t, 14>{0xB5,
                                                        0x62,
                                                        0x06,
                                                        0x08,
                                                        0x06,
                                                        0x00,
                                                        0xE8,
                                                        0x03,
                                                        0x01,
                                                        0x00,
                                                        0x01,
                                                        0x00,
                                                        0x01,
                                                        0x39};

std::vector<std::uint8_t> vec(std::span<std::uint8_t const> s) { return {s.begin(), s.end()}; }

/// The reads in the transcript from `from` on: register and length.
std::vector<std::pair<std::uint8_t,
                      std::size_t>>
reads(std::size_t from = 0) {
    std::vector<std::pair<std::uint8_t, std::size_t>> r;
    for(std::size_t i = from; i < FakeBus::log.size(); ++i) {
        auto const& t = FakeBus::log[i];
        if(t.isRead()) { r.emplace_back(t.sent.empty() ? 0 : t.sent[0], t.recvLen); }
    }
    return r;
}

void bringUp() {
    testCase("SAM-M8Q: bring-up is CFG-PRT (DDC, UBX only), CFG-MSG (NAV-PVT), CFG-RATE");
    fresh();
    M8Model m{};
    FakeBus::respond = std::ref(m);
    Dev<Chips::SamM8q<>> d{};
    check(runUntil(d, [&] { return d.answering() && !d.pending(); }, 500ms), "configured");
    check(
      m.messages
        == std::vector<std::vector<std::uint8_t>>{vec(PortFrame), vec(MsgFrame), vec(RateFrame)},
      "the three frames, in order, with their checksums");
    check(writes() == m.messages, "and nothing else was written");

    testCase("SAM-M8Q: nothing waiting is the count alone, no stream read, no sample");
    runFor(d, 500ms);
    auto const r = reads();
    check(r.size() >= 8, "polled every 50 ms");
    bool onlyCounts = true;
    for(auto const& [reg, len] : r) { onlyCounts = onlyCounts && reg == 0xFD && len == 2; }
    check(onlyCounts, "every read is 0xFD, two bytes");
    checkEq(d.samples(), 0U, "no sample");
    check(!d.valid(), "and not valid");
    checkEq(d.errors(), 0U, "nothing failed");

    testCase("SAM-M8Q: Command sends a prebuilt frame once, and not again after a reset");
    m.messages.clear();
    check(d.set<Chips::SamM8q<>::Command>(Chips::SamM8q<>::Frame::of(ubxFrame(0x0A, 0x04))), "set");
    check(runUntil(d, [&] { return !d.pending(); }, 100ms), "sent");
    check(m.messages
            == std::vector<std::vector<std::uint8_t>>{{0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34}},
          "the MON-VER poll");
    m.messages.clear();
    m.address = 0x00;   // gone
    check(runUntil(d, [&] { return d.absent(); }, 1s), "parked");
    m.address = 0x42;
    check(runUntil(d, [&] { return d.answering() && !d.pending(); }, 3s), "back");
    check(
      m.messages
        == std::vector<std::vector<std::uint8_t>>{vec(PortFrame), vec(MsgFrame), vec(RateFrame)},
      "the configuration again, the command not");
    if(failures != 0) { dump(); }
}

void stream() {
    testCase("SAM-M8Q: a stream read is the two-byte count, then that many bytes of 0xFF");
    fresh();
    M8Model m{};
    FakeBus::respond = std::ref(m);
    Dev<Chips::SamM8q<>> d{};
    check(runUntil(d, [&] { return d.answering() && !d.pending(); }, 500ms), "configured");
    std::vector<std::uint8_t> text(300, 'x');
    m.queue(text);
    auto const from = FakeBus::log.size();
    check(runUntil(d, [&] { return d.fresh(); }, 200ms), "a chunk");
    check(reads(from) == std::vector<std::pair<std::uint8_t, std::size_t>>{{0xFD, 2}, {0xFF, 128}},
          "0xFD for two, then 0xFF for 128 of the 300");
    checkEq(d.latest().length, 128U, "a full chunk");
    checkEq(d.latest().available, 300U, "the count as the part said it");
    check(runUntil(d, [&] { return d.fresh(); }, 200ms), "the next");
    check(runUntil(d, [&] { return d.fresh(); }, 200ms), "and the last");
    checkEq(d.latest().length, 44U, "the 44 left");
    checkEq(d.latest().available, 44U, "of 44");
    auto const at = FakeBus::log.size();
    runFor(d, 200ms);
    check(!d.fresh(), "then nothing");
    for(auto const& [reg, len] : reads(at)) {
        check(reg == 0xFD, "only counts once it is drained");
    }
    if(failures != 0) { dump(); }
}

void navPvt() {
    testCase("SAM-M8Q: a NAV-PVT split across two reads, reassembled by the framer and decoded");
    fresh();
    M8Model m{};
    FakeBus::respond = std::ref(m);
    Dev<Chips::SamM8q<64>> d{};
    check(runUntil(d, [&] { return d.answering() && !d.pending(); }, 500ms), "configured");

    Chips::Ubx ubx{};
    int        pvts   = 0;
    int        chunks = 0;
    M8::NavPvt last{};
    auto const pump = [&](int turns) {
        for(int i = 0; i < turns; ++i) {
            turn(d);
            if(d.fresh()) {
                ++chunks;
                ubx.feed(d.latest().data(), [&](UbxMessage const& msg) {
                    if(msg.is(M8::NavPvtClass, M8::NavPvtId)) {
                        ++pvts;
                        last = M8::decodeNavPvt(msg.payload);
                    }
                });
            }
        }
    };
    std::vector<std::uint8_t> nmea{'$', 'G', 'N', 'G', 'G', 'A', '*', '0', '0', '\r', '\n'};
    m.queue(nmea);   // what the part sent before the port was configured
    m.queue(ubxFrame(M8::NavPvtClass, M8::NavPvtId, navPvtPayload()));
    pump(300);
    checkEq(chunks, 2, "64 + 47 bytes: two chunks");
    checkEq(pvts, 1, "one NAV-PVT");
    checkEq(ubx.frames, 1U, "one frame");
    checkEq(ubx.bad(), 0U, "nothing bad");
    check(last.fixType == M8::FixType::fix3d && last.numSv == 11, "a 3-D fix on eleven");
    check(equal(last.lat, -47376900) && equal(last.lon, 8541700), "at the position");

    testCase("UBX: a bad checksum is counted, not handed on, and the next frame still is");
    auto broken = ubxFrame(M8::NavPvtClass, M8::NavPvtId, navPvtPayload());
    broken[30] ^= 0x01U;   // a payload byte
    m.queue(broken);
    m.queue(ubxFrame(0x05, 0x01, std::array<std::uint8_t, 2>{0x06, 0x08}));   // ACK-ACK CFG-RATE
    pump(300);
    checkEq(ubx.badChecksum, 1U, "one bad checksum");
    checkEq(pvts, 1, "not handed on");
    checkEq(ubx.frames, 2U, "the ACK after it was");

    testCase("UBX: a frame longer than MaxPayload is counted and dropped");
    Kvasir::I2C::Ubx<16> small{};
    auto const           frame = ubxFrame(M8::NavPvtClass, M8::NavPvtId, navPvtPayload());
    std::array<std::byte, frame.size()> bytes{};
    for(std::size_t i = 0; i < frame.size(); ++i) { bytes[i] = std::byte{frame[i]}; }
    int handed = 0;
    small.feed(bytes, [&](UbxMessage const&) { ++handed; });
    checkEq(small.oversize, 1U, "oversize");
    checkEq(handed, 0, "nothing handed on");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    bringUp();
    stream();
    navPvt();
    return finish();
}
