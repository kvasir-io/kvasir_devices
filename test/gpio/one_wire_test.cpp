/// The bit-banged 1-Wire master (OneWire.hpp) against a model of a slave on a simulated open-drain line: reset and
/// presence, no part on the line, write slots (LSB first), read slots, write then read, the transaction deadline, one
/// transaction at a time, and the CRC-8. The model checks every slot against the MAX31820's timing limits (Maxim
/// MAX31820 data sheet, AC electrical characteristics, other/MAX31820.md l.132-141): tSLOT >= 60 us, tREC >= 1 us,
/// tLOW0 60-120 us, tLOW1 1-15 us, tRDV 15 us max (the master's sample), tRSTL and tRSTH >= 480 us, the presence pulse
/// tPDHIGH 15-60 us after the release and tPDLOW 60-240 us long.
#include <algorithm>
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/Bytes.hpp>
#include <kvasir/Devices/OneWire.hpp>
#include <span>
#include <string>
#include <support/FakeClock.hpp>
#include <utility>
#include <vector>

using namespace Kvasir::Test;

namespace Kvasir::Test::OneWireLine {

using Us = std::chrono::microseconds;

/// Simulated time only: a delay winds the clock (rule: no test reads the real clock).
struct Clock : Kvasir::Test::FakeClock {
    template<typename Rep,
             typename Period>
    static void delay(std::chrono::duration<Rep,
                                            Period> d) {
        advance(d);
    }
};

[[nodiscard]] inline Us now() { return Clock::now().time_since_epoch(); }

/// A 1-Wire slave: decodes the master's low pulses (reset, write-1, write-0, read slot) from their lengths, answers
/// the reset with a presence pulse, takes `expectWrite` bytes, then sends `answer` LSB first in the read slots.
struct Slave {
    bool                      present{true};
    std::size_t               expectWrite{};
    std::vector<std::uint8_t> answer;

    // what it saw
    int                       resets{};
    std::vector<std::uint8_t> written;
    int                       readSlots{};
    std::vector<std::string>  errors;

    void masterLow(Us t) {
        if(haveRise_ && t - rise_ < Us{1}) { error("tREC under 1 us"); }
        if(afterReset_) {
            if(t - rise_ < Us{480}) { error("tRSTH under 480 us"); }
        } else if(haveFall_ && t - fall_ < Us{60}) {
            error("tSLOT under 60 us");
        }
        afterReset_ = false;
        fall_       = t;
        haveFall_   = true;
        drive0_     = false;
        if(phase_ == Phase::answer) {
            ++readSlots;
            if(answerBit_ < answer.size() * 8) {
                auto const bit = (answer[answerBit_ / 8] >> (answerBit_ % 8)) & 1U;
                ++answerBit_;
                drive0_ = bit == 0;   // hold the line low past the master's sample
            } else {
                error("a read slot with nothing left to send");
            }
        }
    }

    void masterReleased(Us t) {
        auto const low = t - fall_;
        rise_          = t;
        haveRise_      = true;
        if(low >= Us{480}) {
            ++resets;
            afterReset_ = true;
            haveFall_   = false;
            bit_        = 0;
            byte_       = 0;
            answerBit_  = 0;
            presenceAt_ = t + Us{30};   // tPDHIGH 15-60 us
            phase_ = !present ? Phase::idle : expectWrite == 0 ? Phase::answer : Phase::command;
            return;
        }
        if(low > Us{120} || (low > Us{15} && low < Us{60})) {
            error("a low pulse of " + std::to_string(low.count()) + " us is no slot");
            return;
        }
        if(phase_ == Phase::answer) {
            if(low > Us{15}) { error("a write slot where the slave sends"); }
            return;
        }
        if(phase_ != Phase::command) { return; }
        auto const one = low <= Us{15};
        if(one && low < Us{1}) { error("tLOW1 under 1 us"); }
        byte_ = static_cast<std::uint8_t>(byte_ | (one ? 1U << bit_ : 0U));
        if(++bit_ == 8) {
            written.push_back(byte_);
            bit_  = 0;
            byte_ = 0;
            if(written.size() == expectWrite) { phase_ = Phase::answer; }
        }
    }

    /// The master samples the line at `t`.
    void sampled(Us t) {
        if(phase_ == Phase::answer && haveFall_ && t - fall_ > Us{15}) {
            error("read sample " + std::to_string((t - fall_).count())
                  + " us after the fall: tRDV is 15 us max");
        }
    }

    [[nodiscard]] bool pullsLow(Us t) const {
        // the presence pulse: tPDLOW 60-240 us
        if(present && t >= presenceAt_ && t < presenceAt_ + Us{120}) { return true; }
        return drive0_ && t < fall_ + Us{30};
    }

    // the decoder's state
    enum class Phase : std::uint8_t { idle, command, answer };
    Phase        phase_{Phase::idle};
    Us           fall_{};
    Us           rise_{};
    Us           presenceAt_{Us{-1'000'000}};
    bool         haveFall_{};
    bool         haveRise_{};
    bool         afterReset_{};
    bool         drive0_{};
    unsigned     bit_{};
    std::uint8_t byte_{};
    std::size_t  answerBit_{};

    void error(std::string what) { errors.push_back(std::move(what)); }
};

/// The master's pin (DQ) and the slave on the line.
struct Dq {};

struct PinOp {
    enum class Kind : std::uint8_t { input, output, clear, read };
    Kind kind{};
};

constexpr PinOp makeInput(Dq) { return {PinOp::Kind::input}; }

constexpr PinOp makeOutput(Dq) { return {PinOp::Kind::output}; }

constexpr PinOp clear(Dq) { return {PinOp::Kind::clear}; }

constexpr PinOp read(Dq) { return {PinOp::Kind::read}; }

struct Line {
    static inline Slave* slave{};
    static inline bool   output{};
    static inline bool   latch{true};

    [[nodiscard]] static bool released() { return !output || latch; }
};

/// ADL target of the driver's `apply(makeInput(Pin{}))`, `apply(makeOutput(Pin{}), clear(Pin{}))` and
/// `apply(read(Pin{}))`: the slave is told of every change of the master's side; a read is the wired-AND of master,
/// slave and pull-up.
template<std::same_as<PinOp>... Ops>
bool apply(PinOp first,
           Ops... rest) {
    bool       level  = false;
    bool const before = Line::released();
    for(PinOp const o : {first, rest...}) {
        switch(o.kind) {
        case PinOp::Kind::input:  Line::output = false; break;
        case PinOp::Kind::output: Line::output = true; break;
        case PinOp::Kind::clear:  Line::latch = false; break;
        case PinOp::Kind::read:
            if(Line::slave != nullptr) { Line::slave->sampled(now()); }
            level = Line::released() && (Line::slave == nullptr || !Line::slave->pullsLow(now()));
            break;
        }
    }
    if(Line::slave != nullptr && Line::released() != before) {
        if(Line::released()) {
            Line::slave->masterReleased(now());
        } else {
            Line::slave->masterLow(now());
        }
    }
    return level;
}

}   // namespace Kvasir::Test::OneWireLine

namespace {
using Kvasir::Test::OneWireLine::Clock;
using Kvasir::Test::OneWireLine::Dq;
using Kvasir::Test::OneWireLine::Line;
using Kvasir::Test::OneWireLine::Slave;
using Bus = Kvasir::OneWire<Clock, Dq, 16>;
using Op  = Bus::OperationState;

void fresh(Slave& s) {
    Clock::reset();
    Clock::advance(std::chrono::milliseconds{1});
    Line::output = false;
    Line::latch  = true;
    Line::slave  = &s;
}

/// turn() every microsecond until the transaction is no longer ongoing (or `limit` simulated time passed).
Op run(std::chrono::microseconds limit = std::chrono::milliseconds{50}) {
    auto const end = Clock::now() + limit;
    while(Clock::now() < end) {
        Clock::advance(std::chrono::microseconds{1});
        Bus::turn();
        if(auto const op = Bus::operationState(Clock::now()); op != Op::ongoing) { return op; }
    }
    return Op::ongoing;
}

void checkClean(Slave const& s) {
    for(auto const& e : s.errors) { check(false, e); }
}

/// The end of a case: the line lets go of the case's slave, a local that is gone after it.
void detach() { Line::slave = nullptr; }

template<std::size_t N>
std::array<std::byte,
           N>
bytes(std::array<std::uint8_t,
                 N> const& v) {
    std::array<std::byte, N> out{};
    for(std::size_t i = 0; i < N; ++i) { out[i] = std::byte{v[i]}; }
    return out;
}

void resetAndWrite() {
    testCase("1 reset, presence, then the bytes out LSB first, every slot inside the limits");
    Slave s{};
    s.expectWrite = 3;
    fresh(s);
    check(Bus::acquire(), "acquire an idle bus");
    Bus::send(Clock::now(), bytes(std::array<std::uint8_t, 3>{0xCC, 0x44, 0x01}));
    checkEq(static_cast<int>(run()), static_cast<int>(Op::succeeded), "send succeeds");
    checkEq(s.resets, 1, "one reset pulse");
    check(s.written == std::vector<std::uint8_t>{0xCC, 0x44, 0x01},
          "the slave decoded 0xCC 0x44 0x01");
    checkEq(s.readSlots, 0, "no read slot after a send");
    checkClean(s);
    Bus::release();
    detach();
}

void noPresence() {
    testCase("2 no part on the line: no presence pulse, the transaction fails, nothing is sent");
    Slave s{};
    s.present     = false;
    s.expectWrite = 1;
    fresh(s);
    check(Bus::acquire(), "acquire");
    Bus::send(Clock::now(), bytes(std::array<std::uint8_t, 1>{0xCC}));
    checkEq(static_cast<int>(run()), static_cast<int>(Op::failed), "send fails without presence");
    check(s.written.empty(), "no write slot after a missing presence pulse");
    checkClean(s);
    Bus::release();
    detach();
}

void writeThenRead() {
    testCase("3 write then read: a scratchpad with its CRC comes back byte for byte");
    std::array<std::uint8_t, 9> pad{0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0F, 0x10, 0x00};
    auto const                  head = bytes(pad);
    pad[8]                           = Kvasir::Dallas::crc8(Kvasir::Bytes{
      std::span<std::byte const>{head.data(), 8}
    });
    Slave s{};
    s.expectWrite = 2;
    s.answer      = {pad.begin(), pad.end()};
    fresh(s);
    check(Bus::acquire(), "acquire");
    Bus::sendReceive(Clock::now(), bytes(std::array<std::uint8_t, 2>{0xCC, 0xBE}), 9);
    checkEq(static_cast<int>(run()), static_cast<int>(Op::succeeded), "sendReceive succeeds");
    check(s.written == std::vector<std::uint8_t>{0xCC, 0xBE}, "Skip ROM, Read Scratchpad out");
    checkEq(s.readSlots, 72, "72 read slots");
    std::array<std::byte, 9> got{};
    Bus::getReceivedBytes(got);
    check(got == bytes(pad), "the scratchpad as the slave sent it");
    checkEq(Bus::crc(std::span<std::byte const>{got}), 0, "CRC-8 over data and CRC byte is 0");
    checkClean(s);
    Bus::release();
    detach();
}

void readOnly() {
    testCase("4 receive only: reset, then read slots alone; zeros and ones both decoded");
    Slave s{};
    s.expectWrite = 0;
    s.answer      = {0x00, 0xFF, 0xA5};
    fresh(s);
    check(Bus::acquire(), "acquire");
    Bus::receive(Clock::now(), 3);
    checkEq(static_cast<int>(run()), static_cast<int>(Op::succeeded), "receive succeeds");
    std::array<std::byte, 3> got{};
    Bus::getReceivedBytes(got.begin(), got.end());
    check(got == bytes(std::array<std::uint8_t, 3>{0x00, 0xFF, 0xA5}), "0x00 0xFF 0xA5");
    checkClean(s);
    Bus::release();
    detach();
}

void deadlineAndOwnership() {
    testCase("5 one transaction at a time; the deadline fails one that is still running");
    Slave s{};
    s.expectWrite = 1;
    fresh(s);
    check(Bus::acquire(), "acquire");
    check(!Bus::acquire(), "a second acquire while held is refused");
    Bus::send(Clock::now(), bytes(std::array<std::uint8_t, 1>{0xCC}));
    checkEq(static_cast<int>(Bus::operationState(Clock::now())),
            static_cast<int>(Op::ongoing),
            "ongoing at once");
    // 100 ms + 2 ms per bit of one byte: 116 ms
    checkEq(static_cast<int>(Bus::operationState(Clock::now() + std::chrono::milliseconds{115})),
            static_cast<int>(Op::ongoing),
            "still ongoing just inside the deadline");
    checkEq(static_cast<int>(Bus::operationState(Clock::now() + std::chrono::milliseconds{117})),
            static_cast<int>(Op::failed),
            "failed past the deadline");
    checkEq(static_cast<int>(Bus::operationState(Clock::now())),
            static_cast<int>(Op::failed),
            "and stays failed");
    Bus::release();
    check(Bus::acquire(), "acquire again after release");
    Bus::release();
    detach();
}

/// The Dallas/Maxim CRC-8 bit by bit (x^8 + x^5 + x^4 + 1, reflected, init 0): the reference.
std::uint8_t crcReference(std::span<std::uint8_t const> data) {
    std::uint8_t c = 0;
    for(auto b : data) {
        for(int i = 0; i < 8; ++i) {
            auto const mix = static_cast<std::uint8_t>((c ^ b) & 1U);
            c              = static_cast<std::uint8_t>(c >> 1U);
            if(mix != 0) { c = static_cast<std::uint8_t>(c ^ 0x8CU); }
            b = static_cast<std::uint8_t>(b >> 1U);
        }
    }
    return c;
}

void crc() {
    testCase("6 crc(): the bitwise reference over 256 patterns, both overloads, 0 over data + CRC");
    std::uint32_t x = 0x12345678;
    for(int n = 0; n < 256; ++n) {
        std::array<std::uint8_t, 8> d{};
        for(auto& v : d) {
            x = x * 1664525U + 1013904223U;
            v = static_cast<std::uint8_t>(x >> 24U);
        }
        auto const want = crcReference(d);
        auto const b    = bytes(d);
        checkEq(Bus::crc(std::span<std::byte const>{b}), want, "crc(span)");
        checkEq(Bus::crc(d.begin(), d.end()), want, "crc(first, last)");
        std::array<std::uint8_t, 9> withCrc{};
        std::ranges::copy(d, withCrc.begin());
        withCrc[8] = want;
        checkEq(Bus::crc(withCrc.begin(), withCrc.end()), 0, "0 over data + its CRC");
    }
}
}   // namespace

int main() {
    resetAndWrite();
    noPresence();
    writeThenRead();
    readOnly();
    deadlineAndOwnership();
    crc();
    return finish();
}
