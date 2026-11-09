/// The bit-banged 1-Wire master (OneWire.hpp) against a model of a slave on a simulated open-drain line: reset and
/// presence, no part on the line, write slots (LSB first), read slots, write then read, the transaction deadline, one
/// transaction at a time, the CRC-8, and an interrupt after every timed wait. The model checks every slot against the MAX31820's timing limits (Maxim
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
#include <kvasir/Atomic/CriticalSection.hpp>
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

/// An interrupt of `length` that runs at the end of every timed wait while `armed` - the worst moment for a bit-banged
/// slot - unless the master masked interrupts there (a criticalSection: the policy below counts them).
struct Interrupt {
    static inline bool armed{};
    static inline Us   length{
      126};   // an ADC handler with a soft-float PID update on a 48 MHz Cortex-M0+
    static inline int every{1};   // at the end of every `every`-th timed wait
    static inline int waits{};
    static inline int masked{};
    static inline int ran{};
    static inline int heldOff{};
};

/// This test's critical-section policy: no lock (one thread), it says whether interrupts are masked.
struct MaskCounting {
    template<typename Tag,
             typename F>
    static decltype(auto) run(F&& f) {
        ++Interrupt::masked;

        struct Unmask {
            ~Unmask() { --Interrupt::masked; }
        } const unmask{};

        return std::forward<F>(f)();
    }

    template<typename Tag,
             typename F,
             typename Pred>
    static decltype(auto) run(F&&    f,
                              Pred&& pred) {
        while(!pred()) {}
        return run<Tag>(std::forward<F>(f));
    }
};

/// Simulated time only: a delay winds the clock (rule: no test reads the real clock).
struct Clock : Kvasir::Test::FakeClock {
    template<typename Rep,
             typename Period>
    static void delay(std::chrono::duration<Rep,
                                            Period> d) {
        advance(d);
        if(!Interrupt::armed || ++Interrupt::waits % Interrupt::every != 0) { return; }
        if(Interrupt::masked == 0) {
            ++Interrupt::ran;
            advance(Interrupt::length);
        } else {
            ++Interrupt::heldOff;   // it runs once the section ends, after the slot's timed part
        }
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
    std::size_t               errorsAtLastReset{};

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
            errorsAtLastReset  = errors.size();
            writtenSinceReset_ = 0;   // a reset starts the command over (a master's restart)
            afterReset_        = true;
            haveFall_          = false;
            bit_               = 0;
            byte_              = 0;
            answerBit_         = 0;
            presenceAt_        = t + Us{30};   // tPDHIGH 15-60 us
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
            if(++writtenSinceReset_ == expectWrite) { phase_ = Phase::answer; }
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
    std::size_t  writtenSinceReset_{};

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

template<>
struct Kvasir::Atomic::CriticalSectionPolicy<Kvasir::Tag::User> {
    using type = Kvasir::Test::OneWireLine::MaskCounting;
};

namespace {
using Kvasir::Test::OneWireLine::Clock;
using Kvasir::Test::OneWireLine::Dq;
using Kvasir::Test::OneWireLine::Line;
using Kvasir::Test::OneWireLine::Slave;
using Bus = Kvasir::OneWire<Clock, Dq, 16>;
using Op  = Bus::OperationState;

/// The checked mode (OneWireSlotTiming::checked): nothing masked, every timed part measured.
struct CheckedConfig : Kvasir::OneWireDefaults {
    static constexpr auto slotTiming = Kvasir::OneWireSlotTiming::checked;
};

using CheckedBus = Kvasir::OneWire<Clock, Dq, 16, CheckedConfig>;

/// A SlotGate closed until `openFrom`; it notes the time it was first asked.
struct TestGate {
    static inline Kvasir::Test::OneWireLine::Us openFrom{};
    static inline int                           closedAnswers{};

    static bool open(std::chrono::microseconds) {
        bool const o = Kvasir::Test::OneWireLine::now() >= openFrom;
        if(!o) { ++closedAnswers; }
        return o;
    }
};

struct GatedConfig : CheckedConfig {
    using SlotGate = TestGate;
};

using GatedBus = Kvasir::OneWire<Clock, Dq, 16, GatedConfig>;

void fresh(Slave& s) {
    Clock::reset();
    Clock::advance(std::chrono::milliseconds{1});
    Line::output = false;
    Line::latch  = true;
    Line::slave  = &s;
}

/// turn() every microsecond until the transaction is no longer ongoing (or `limit` simulated time passed).
template<typename B = Bus>
typename B::OperationState run(std::chrono::microseconds limit = std::chrono::milliseconds{50}) {
    using O        = typename B::OperationState;
    auto const end = Clock::now() + limit;
    while(Clock::now() < end) {
        Clock::advance(std::chrono::microseconds{1});
        B::turn();
        if(auto const op = B::operationState(Clock::now()); op != O::ongoing) { return op; }
    }
    return O::ongoing;
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

void interruptAfterEveryWait() {
    testCase(
      "7 an interrupt after every timed wait (126 us): the presence is seen, no slot "
      "stretched past its limit, write and read bytes intact");
    using Kvasir::Test::OneWireLine::Interrupt;
    std::array<std::uint8_t, 9> pad{0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0F, 0x10, 0x00};
    auto const                  head = bytes(pad);
    pad[8]                           = Kvasir::Dallas::crc8(Kvasir::Bytes{
      std::span<std::byte const>{head.data(), 8}
    });
    Slave s{};
    s.expectWrite = 2;
    s.answer      = {pad.begin(), pad.end()};
    fresh(s);
    Interrupt::armed   = true;
    Interrupt::ran     = 0;
    Interrupt::heldOff = 0;
    check(Bus::acquire(), "acquire");
    Bus::sendReceive(Clock::now(), bytes(std::array<std::uint8_t, 2>{0xCC, 0xBE}), 9);
    auto const op    = run(std::chrono::milliseconds{200});
    Interrupt::armed = false;
    checkEq(static_cast<int>(op), static_cast<int>(Op::succeeded), "sendReceive succeeds");
    check(s.written == std::vector<std::uint8_t>{0xCC, 0xBE}, "Skip ROM, Read Scratchpad out");
    if(op == Op::succeeded && s.readSlots == 72) {
        std::array<std::byte, 9> got{};
        Bus::getReceivedBytes(got);
        check(got == bytes(pad), "the scratchpad as the slave sent it");
    } else {
        check(false, "72 read slots and a finished transaction, to compare the scratchpad");
    }
    checkClean(s);
    checkEq(Interrupt::ran, 0, "no interrupt inside a timed part of a slot");
    check(Interrupt::heldOff > 0, "the interrupt was there: held off until the timed parts ended");
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

/// Read Scratchpad's answer: 8 bytes and their CRC.
std::array<std::uint8_t,
           9>
scratchpad() {
    std::array<std::uint8_t, 9> pad{0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0F, 0x10, 0x00};
    auto const                  head = bytes(pad);
    pad[8]                           = Kvasir::Dallas::crc8(Kvasir::Bytes{
      std::span<std::byte const>{head.data(), 8}
    });
    return pad;
}

void checkedEveryWaitStretched() {
    testCase(
      "8 checked, an interrupt (126 us) after every timed wait: every part comes out too long - "
      "restarted "
      "timingRetries times, then failed; nothing masked, nothing handed out");
    using Kvasir::Test::OneWireLine::Interrupt;
    auto const pad = scratchpad();
    Slave      s{};
    s.expectWrite = 2;
    s.answer      = {pad.begin(), pad.end()};
    fresh(s);
    Interrupt::armed   = true;
    Interrupt::every   = 1;
    Interrupt::ran     = 0;
    Interrupt::heldOff = 0;
    auto const faults  = CheckedBus::timingFaults();
    check(CheckedBus::acquire(), "acquire");
    CheckedBus::sendReceive(Clock::now(), bytes(std::array<std::uint8_t, 2>{0xCC, 0xBE}), 9);
    auto const op    = run<CheckedBus>(std::chrono::milliseconds{200});
    Interrupt::armed = false;
    checkEq(static_cast<int>(op), static_cast<int>(CheckedBus::OperationState::failed), "fails");
    checkEq(static_cast<int>(CheckedBus::timingFaults() - faults),
            CheckedBus::timingRetries + 1,
            "one fault per try: the first and timingRetries restarts");
    checkEq(Interrupt::heldOff, 0, "nothing masked");
    check(Interrupt::ran > 0, "the interrupts ran inside the parts");
    checkEq(s.resets, CheckedBus::timingRetries + 1, "a reset pulse for every try");
    check(s.written.empty(), "no byte reached the slave: each try ended at its first part");
    CheckedBus::release();
    detach();
}

void checkedSomeStretched() {
    testCase(
      "9 checked, a 30 us interrupt in the first try's 20th timed wait (a read slot's low pulse): "
      "that try "
      "restarts, the next one is clean - it succeeds intact, the slave sees nothing out of its "
      "limits after the "
      "restart");
    using Kvasir::Test::OneWireLine::Interrupt;
    auto const pad = scratchpad();
    Slave      s{};
    s.expectWrite = 2;
    s.answer      = {pad.begin(), pad.end()};
    fresh(s);
    Interrupt::armed = true;
    Interrupt::length
      = Kvasir::Test::OneWireLine::Us{30};   // about a station's ADC handler, integer
    Interrupt::every = 200;                  // more waits than a try has: one hit, then none
    Interrupt::waits
      = 180;   // the first at the try's 20th wait: presence 1 + 16 write bits, then read slots
    Interrupt::ran    = 0;
    auto const faults = CheckedBus::timingFaults();
    check(CheckedBus::acquire(), "acquire");
    CheckedBus::sendReceive(Clock::now(), bytes(std::array<std::uint8_t, 2>{0xCC, 0xBE}), 9);
    auto const op     = run<CheckedBus>(std::chrono::milliseconds{200});
    Interrupt::armed  = false;
    Interrupt::every  = 1;
    Interrupt::length = Kvasir::Test::OneWireLine::Us{126};
    checkEq(static_cast<int>(op),
            static_cast<int>(CheckedBus::OperationState::succeeded),
            "succeeds");
    auto const restarts = CheckedBus::timingFaults() - faults;
    checkEq(static_cast<int>(restarts), 1, "the one stretched read slot restarted it");
    checkEq(Interrupt::ran, 1, "the interrupt ran, inside the slot");
    check(!s.errors.empty() && s.errors.size() == s.errorsAtLastReset,
          "the slave saw the stretched slot out of its limits, and nothing out of them after the "
          "restart's reset");
    if(op == CheckedBus::OperationState::succeeded) {
        std::array<std::byte, 9> got{};
        CheckedBus::getReceivedBytes(got);
        check(got == bytes(pad), "the scratchpad as the slave sent it");
    }
    checkEq(s.resets, static_cast<int>(restarts) + 1, "one reset per try");
    check(s.written.size() >= 2
            && std::vector<std::uint8_t>(s.written.end() - 2, s.written.end())
                 == std::vector<std::uint8_t>{0xCC, 0xBE},
          "the last try sent Skip ROM, Read Scratchpad");
    CheckedBus::release();
    detach();
}

void gateHoldsSlots() {
    testCase(
      "10 a closed SlotGate: no slot starts while it is closed - the reset pulse just stays low - "
      "then the "
      "transaction runs as usual");
    auto const pad = scratchpad();
    Slave      s{};
    s.expectWrite = 2;
    s.answer      = {pad.begin(), pad.end()};
    fresh(s);
    auto const start        = Kvasir::Test::OneWireLine::now();
    TestGate::openFrom      = start + Kvasir::Test::OneWireLine::Us{700};
    TestGate::closedAnswers = 0;
    check(GatedBus::acquire(), "acquire");
    GatedBus::sendReceive(Clock::now(), bytes(std::array<std::uint8_t, 2>{0xCC, 0xBE}), 9);
    auto const op = run<GatedBus>(std::chrono::milliseconds{200});
    checkEq(static_cast<int>(op),
            static_cast<int>(GatedBus::OperationState::succeeded),
            "succeeds");
    check(TestGate::closedAnswers > 0, "the gate was asked while closed");
    checkEq(s.resets, 1, "one reset pulse - longer than 480 us, no second one");
    check(s.written == std::vector<std::uint8_t>{0xCC, 0xBE}, "the bytes out");
    std::array<std::byte, 9> got{};
    GatedBus::getReceivedBytes(got);
    check(got == bytes(pad), "the scratchpad in");
    checkClean(s);
    GatedBus::release();
    detach();
}

}   // namespace

int main() {
    resetAndWrite();
    noPresence();
    writeThenRead();
    readOnly();
    deadlineAndOwnership();
    crc();
    interruptAfterEveryWait();
    checkedEveryWaitStretched();
    checkedSomeStretched();
    gateHoldsSlots();
    return finish();
}
