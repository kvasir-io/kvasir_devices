/// The TLV493D-A1B6 description against a model of the part: a read register file that
/// always answers from 0x00, a write register file that always takes 0x10..0x13, and one
/// conversion per request in master controlled mode. Its own translation unit: the model of
/// the part is large.
#include "Harness.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Tlv493d.hpp>
#include <span>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

using Sensor    = Device<FakeBus, FakeClock, Chips::Tlv493d>;
using Field     = Chips::Tlv493d::Field;
using Configure = Chips::Tlv493d::Configure;

long long nanoTesla(Units::NanoTesla q) {
    return q.numerical_value_in(mp_units::si::nano<mp_units::si::tesla>);
}

long long milliDegC(Units::MilliDegC q) {
    return q.numerical_value_in(mp_units::si::milli<mp_units::si::degree_Celsius>);
}

/// Infineon's calcParity, as written in their library: P set, every write byte XORed into
/// one, the bits of that folded, the result's LSB into P.
std::uint8_t infineonParity(std::array<std::uint8_t,
                                       4> w) {
    w[1]           = static_cast<std::uint8_t>(w[1] | 0x80U);
    std::uint8_t y = 0;
    for(auto const b : w) { y = static_cast<std::uint8_t>(y ^ b); }
    y = static_cast<std::uint8_t>(y ^ (y >> 1U));
    y = static_cast<std::uint8_t>(y ^ (y >> 2U));
    y = static_cast<std::uint8_t>(y ^ (y >> 4U));
    return static_cast<std::uint8_t>(y & 0x01U);
}

struct Tlv493dModel {
    std::uint8_t                             address{0x5E};
    std::array<std::uint8_t, 10>             reg{};
    std::array<std::uint8_t, 4>              mod{};
    std::vector<std::array<std::uint8_t, 4>> written{};
    std::int16_t                             bx{};
    std::int16_t                             by{};
    std::int16_t                             bz{};
    std::int16_t                             temperature{340};
    bool                                     converts{true};
    bool                                     parityError{};
    int                                      conversions{};
    int midConversion{};   ///< frame reads answered with CH = 1

    /// A conversion here lasts 1.5 ms, not the part's ~270 us: this model sees time in the
    /// simulation's 1 ms turns, so "the next transaction a turn after the one that started a
    /// conversion" is what landing inside it looks like.
    static constexpr std::chrono::microseconds Conversion{1500};
    FakeClock::time_point                      busyUntil{};
    int insideConversion{};   ///< transactions while converting

    [[nodiscard]] bool masterControlled() const { return (mod[1] & 0x03U) == 0x03U; }

    void convert() {
        if(!converts) { return; }
        ++conversions;
        busyUntil      = FakeClock::now() + Conversion;
        auto const x   = static_cast<std::uint16_t>(bx) & 0x0FFFU;
        auto const y   = static_cast<std::uint16_t>(by) & 0x0FFFU;
        auto const z   = static_cast<std::uint16_t>(bz) & 0x0FFFU;
        auto const t   = static_cast<std::uint16_t>(temperature) & 0x0FFFU;
        auto const frm = ((static_cast<unsigned>(reg[3]) >> 2U) + 1U) & 0x03U;
        reg[0]         = static_cast<std::uint8_t>(x >> 4U);
        reg[1]         = static_cast<std::uint8_t>(y >> 4U);
        reg[2]         = static_cast<std::uint8_t>(z >> 4U);
        reg[3]         = static_cast<std::uint8_t>((t >> 8U) << 4U | frm << 2U);
        reg[4]         = static_cast<std::uint8_t>((x & 0x0FU) << 4U | (y & 0x0FU));
        reg[5]         = static_cast<std::uint8_t>((reg[5] & 0xE0U) | 0x10U | (z & 0x0FU));   // PD
        reg[6]         = static_cast<std::uint8_t>(t & 0xFFU);
    }

    FakeBus::Result operator()(std::uint8_t               addr,
                               std::span<std::byte const> sent,
                               std::span<std::byte>       recv) {
        if(addr != address) { return FakeBus::Result::notAcknowledged; }
        if(FakeClock::now() < busyUntil) { ++insideConversion; }
        if(!sent.empty()) {
            if(sent.size() > mod.size()) { return FakeBus::Result::failed; }
            int ones = 0;
            for(std::size_t i = 0; i < sent.size(); ++i) {
                mod[i] = static_cast<std::uint8_t>(sent[i]);
                ones += std::popcount(mod[i]);
            }
            written.push_back(mod);
            if((mod[3] & 0x20U) != 0 && (ones & 1) == 0) { parityError = true; }
            if(masterControlled()) { convert(); }
        }
        if(!recv.empty()) {
            if(recv.size() > reg.size()) { return FakeBus::Result::failed; }
            for(std::size_t i = 0; i < recv.size(); ++i) {
                recv[i] = static_cast<std::byte>(reg[i]);
            }
            if(recv.size() == 7 && midConversion > 0) {
                recv[3] = static_cast<std::byte>(reg[3] | 0x01U);
                --midConversion;
            }
            // A readout starts the next conversion (Infineon: MASTERCONTROLLEDMODE).
            if(masterControlled()) { convert(); }
        }
        return FakeBus::Result::succeeded;
    }
};

Tlv493dModel part() {
    Tlv493dModel m{};
    m.reg[5] = 0x20;   // FF; every count 0 until the first conversion
    m.reg[7] = 0x9B;   // factory bits 4:3 = 11, the rest not ours
    m.reg[8] = 0xA5;
    m.reg[9] = 0xE7;   // factory bits 4:0 = 00111
    m.bx     = -241;   // Table 10's example: 1111 0000 1111
    m.by     = 2047;
    m.bz     = -2048;
    return m;
}

/// Turns until the part is configured, then until `readouts` more frames have been sampled:
/// a readout returns the conversion the one before it started, so what the model was set to
/// shows one readout later.
bool configuredAndSampled(Sensor&             d,
                          Tlv493dModel const& m,
                          std::uint32_t       readouts) {
    if(!runUntil(d, [&] { return m.masterControlled(); }, 3s)) { return false; }
    auto const from = d.samples<Field>();
    return runUntil(d, [&] { return d.samples<Field>() >= from + readouts; }, 500ms);
}

void parity() {
    testCase("TLV493D parity against Infineon's calcParity");
    int mismatches = 0;
    for(unsigned f1 = 0; f1 < 256; ++f1) {
        for(unsigned f3 = 0; f3 < 256; ++f3) {
            for(unsigned const f2 : {0x00U, 0x5AU, 0xFFU}) {
                auto const c    = Chips::Tlv493d::configuration(static_cast<std::uint8_t>(f1),
                                                                static_cast<std::uint8_t>(f2),
                                                                static_cast<std::uint8_t>(f3));
                int        ones = 0;
                for(auto const b : c) { ones += std::popcount(b); }
                auto const p = static_cast<std::uint8_t>(c[1] >> 7U);
                if((ones & 1) != 1 || p != infineonParity(c)) { ++mismatches; }
            }
        }
    }
    checkEq(mismatches, 0, "odd over 32 bits, the same P Infineon computes");
}

void bringUp() {
    testCase("TLV493D bring-up, configuration and decode");
    fresh();
    auto m           = part();
    FakeBus::respond = std::ref(m);
    Sensor d{};
    check(runUntil(d, [&] { return m.masterControlled(); }, 3s), "configured");
    checkEq(d.samples<Field>(),
            0U,
            "nothing sampled before the first conversion: the power-on frame is all zero counts");
    check(d.rejected<Field>() > 0, "its readouts ran out of retries instead");
    check(configuredAndSampled(d, m, 2), "then sampled");
    check(d.identified(), "FF set, T clear");
    check(!FakeBus::log.empty() && FakeBus::log.front().isRead()
            && FakeBus::log.front().recvLen == 10,
          "a bare ten-byte read first, nothing written before it");
    check(!m.written.empty(), "configured");
    if(!m.written.empty()) {
        auto const w = m.written.front();
        check(w == std::array<std::uint8_t, 4>{0x00, 0x1B, 0xA5, 0x67},
              "0x10 = 0, MOD1 = P 0 | factory 11 | FAST LOW, 0x12 = factory, MOD2 = LP PT | 00111");
        check((w[1] & 0x04U) == 0, "INT off");
        check((w[1] & 0x60U) == 0, "IICAddr 00: stays at 0x5E");
        check((w[3] & 0x80U) == 0, "temperature on");
    }
    check(!m.parityError, "every write odd");
    checkEq(d.rejected<Configure>(),
            1U,
            "the first configuration run had no factory bits to build from and wrote nothing");
    checkEq(nanoTesla(d.latest<Field>().x), -241 * 98'000, "the model's frame reached the decode");
    // Bx 0xF0F, By 0x7FF, Bz 0x800, T 0x154 (340), FRM 1, CH 0; the first frame, after nothing
    static_assert([] {
        auto const f   = frame(0xF0, 0x7F, 0x80, 0x14, 0xFF, 0x30, 0x54);
        auto const got = Field::decode(Bytes{f}, Field::Sample{});
        return isOk(got)
            && equal(got.value.x, Units::nanoTesla(-241 * 98'000))     // Table 10: -23.6 mT
            && equal(got.value.y, Units::nanoTesla(2047 * 98'000))     // full scale positive
            && equal(got.value.z, Units::nanoTesla(-2048 * 98'000))    // full scale negative
            && equal(got.value.temperature, Units::milliDegC(25000))   // 340 counts is 25 degC
            && equal(got.value.frame, 1);
    }());

    // The same frame with PD clear, which is what the A1B6 on the i2c_testing bench reads in every
    // master-controlled readout (doc/hwtest-findings.md there, 2026-09-18): a frame all the same.
    static_assert([] {
        auto const f   = frame(0xF0, 0x7F, 0x80, 0x14, 0xFF, 0x20, 0x54);
        auto const got = Field::decode(Bytes{f}, Field::Sample{});
        return isOk(got) && equal(got.value.x, Units::nanoTesla(-241 * 98'000))
            && equal(got.value.frame, 1);
    }());
    // What the part holds before it has converted anything: every count 0, which would be
    // -349 degC. Rejected, whatever PD says; and CH not 00 is still a frame to read again.
    static_assert(!isOk(
      Field::decode(Bytes{frame(0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00)}, Field::Sample{})));
    static_assert(!isOk(
      Field::decode(Bytes{frame(0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00)}, Field::Sample{})));
    static_assert(!isOk(
      Field::decode(Bytes{frame(0xF0, 0x7F, 0x80, 0x15, 0xFF, 0x30, 0x54)}, Field::Sample{})));

    // The field is a readout alone: between two configuration runs nothing is written.
    auto const writesNow  = m.written.size();
    auto const samplesNow = d.samples<Field>();
    for(int i = 0; i < 200 && m.written.size() == writesNow; ++i) { turn(d); }
    check(d.samples<Field>() >= samplesNow + 2 || m.written.size() == writesNow,
          "several readouts go by without a write");

    m.temperature     = 440;
    m.bx              = 10;
    auto const before = d.samples<Field>();
    check(runUntil(
            d,
            [&] { return d.samples<Field>() >= before + 2; },
            200ms),
          "two readouts on: the second carries the new conversion");
    checkEq(milliDegC(d.latest<Field>().temperature),
            135000,
            "the new conversion reached the decode");
    // Bx 0x00A, T 0x1B8 (440): 100 counts at 1.1 degC over 25 degC, 10 counts of field
    static_assert([] {
        auto const f   = frame(0x00, 0x7F, 0x80, 0x18, 0xAF, 0x30, 0xB8);
        auto const got = Field::decode(Bytes{f}, Field::Sample{});
        return isOk(got) && equal(got.value.temperature, Units::milliDegC(135000))
            && equal(got.value.x, Units::nanoTesla(980'000));
    }());
    auto const writes = m.written.size();
    check(runUntil(
            d,
            [&] { return m.written.size() > writes; },
            600ms),
          "configured again within half a second");
    check(!m.parityError, "still odd");
    if(failures != 0) { dump(); }
}

void frames() {
    testCase("TLV493D mid-conversion and stale frames");
    fresh();
    auto m           = part();
    FakeBus::respond = std::ref(m);
    Sensor d{};
    check(configuredAndSampled(d, m, 2), "configured, then sampled");
    auto const rejectedBefore = d.rejected<Field>();

    m.midConversion   = 1;
    m.bz              = 100;
    auto const before = d.samples<Field>();
    check(runUntil(
            d,
            [&] { return d.samples<Field>() >= before + 2; },
            200ms),
          "a CH != 0 frame is read again");
    checkEq(nanoTesla(d.latest<Field>().z), 9'800'000, "the frame after it");
    // CH != 0 (bit 0 of byte 3): mid-conversion, read again a millisecond later; the frame
    // after it has Bz 0x064, 100 counts
    static_assert([] {
        auto const busy = frame(0xF0, 0x7F, 0x06, 0x15, 0xFF, 0x34, 0x54);
        auto const noPd = frame(0xF0, 0x7F, 0x06, 0x14, 0xFF, 0x24, 0x54);
        auto const test = frame(0xF0, 0x7F, 0x06, 0x14, 0xFF, 0x74, 0x54);
        auto const done = frame(0xF0, 0x7F, 0x06, 0x14, 0xFF, 0x34, 0x54);
        auto const r    = Field::decode(Bytes{busy}, Field::Sample{});
        auto const rPd  = Field::decode(Bytes{noPd}, Field::Sample{});
        auto const got  = Field::decode(Bytes{done}, Field::Sample{});
        // PD clear is not a reason to read again: the bench's A1B6 never sets it in this mode.
        return isRetry(r) && r.retryAfter == 1ms && isOk(rPd)
            && isReject(Field::decode(Bytes{test}, Field::Sample{})) && isOk(got)
            && equal(got.value.z, Units::nanoTesla(9'800'000));
    }());
    checkEq(d.rejected<Field>(), rejectedBefore, "and not counted as a rejection");

    // The last frame read before the part stops has already started one more conversion,
    // so the first frame after that is still new; what follows it is not. Counted from here:
    // unchanged() only covers frames since the part started converting.
    auto const unchangedBefore = d.unchanged<Field>();
    m.converts                 = false;
    check(runUntil(
            d,
            [&] { return d.unchanged<Field>() > unchangedBefore; },
            300ms),
          "a part that stopped converting");
    auto const seq    = d.seq<Field>();
    auto const quiet  = d.unchanged<Field>();
    auto const writes = m.written.size();
    check(runUntil(
            d,
            [&] { return d.unchanged<Field>() >= quiet + 2; },
            300ms),
          "keeps answering with the same frame");
    checkEq(d.seq<Field>(), seq, "does not step seq");
    // the same field, temperature and FRM as the sample before it: a part that did not convert
    static_assert([] {
        auto const f    = frame(0xF0, 0x7F, 0x80, 0x14, 0xFF, 0x30, 0x54);
        auto const prev = Field::decode(Bytes{f}, Field::Sample{}).value;
        auto const next = frame(0xF0, 0x7F, 0x80, 0x18, 0xFF, 0x30, 0x54);   // FRM 2
        return isUnchanged(Field::decode(Bytes{f}, prev)) && isOk(Field::decode(Bytes{next}, prev));
    }());
    check(runUntil(
            d,
            [&] { return m.written.size() > writes; },
            600ms),
          "and is configured again meanwhile, twice a second");
    checkEq(d.rejected<Field>(), rejectedBefore, "nor count as rejections");

    m.converts = true;
    m.by       = -5;
    check(runUntil(
            d,
            [&] { return d.seq<Field>() >= seq + 2; },
            200ms),
          "and is back once it converts");
    checkEq(nanoTesla(d.latest<Field>().y), -490'000, "-5 counts");
    // By 0xFFB, -5 counts
    static_assert([] {
        auto const f   = frame(0xF0, 0xFF, 0x80, 0x14, 0xFB, 0x30, 0x54);
        auto const got = Field::decode(Bytes{f}, Field::Sample{});
        return isOk(got) && equal(got.value.y, Units::nanoTesla(-490'000));
    }());
    if(failures != 0) { dump(); }
}

void conversionGaps() {
    testCase("TLV493D nothing lands inside a conversion");
    fresh();
    auto m           = part();
    FakeBus::respond = std::ref(m);
    Sensor d{};
    check(configuredAndSampled(d, m, 2), "configured, then sampled");
    auto const writes  = m.written.size();
    auto const samples = d.samples<Field>();
    runFor(d, 3s);
    check(m.written.size() >= writes + 5, "configured again, twice a second");
    check(d.samples<Field>() >= samples + 50, "and sampled all along");
    checkEq(m.insideConversion,
            0,
            "no readout or write while the one before it has a conversion running: the "
            "Configure run's write after its readout, nor a readout after either");
    if(failures != 0) { dump(); }
}

void corruptedFactoryRead() {
    testCase("TLV493D a corrupted factory read is never written");
    fresh();
    auto m           = part();
    FakeBus::respond = std::ref(m);
    Sensor d{};
    check(configuredAndSampled(d, m, 1), "configured, then sampled");
    auto const writesBefore = m.written.size();
    // One ten-byte read answers with a flipped factory bit, as a bus error would.
    int  corrupt = 1;
    auto inner   = std::ref(m);
    FakeBus::respond
      = [&](std::uint8_t addr, std::span<std::byte const> sent, std::span<std::byte> recv) {
            auto const r = inner(addr, sent, recv);
            if(recv.size() == 10 && corrupt > 0) {
                recv[8] = static_cast<std::byte>(static_cast<std::uint8_t>(recv[8]) ^ 0x01U);
                --corrupt;
            }
            return r;
        };
    check(runUntil(
            d,
            [&] { return m.written.size() >= writesBefore + 2; },
            1500ms),
          "configuration goes on");
    check(corrupt == 0, "and the corrupted read did happen");
    bool onlyGood = true;
    for(std::size_t i = writesBefore; i < m.written.size(); ++i) {
        onlyGood = onlyGood && m.written[i] == std::array<std::uint8_t, 4>{0x00, 0x1B, 0xA5, 0x67};
    }
    check(onlyGood, "every configuration written carries the real factory bits");
    check(!m.parityError, "and odd parity");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    parity();
    bringUp();
    frames();
    conversionGaps();
    corruptedFactoryRead();
    return finish();
}
