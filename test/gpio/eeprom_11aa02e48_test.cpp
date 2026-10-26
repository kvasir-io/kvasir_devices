/// 11AA02E48 (UNI/O EUI-48 EEPROM) against a model of the part on a simulated line: a read, the
/// line rule, no part, a corrupted half bit, no log line inside the timed read, the retries, and
/// another bit period. Data sheet: Microchip DS20002122D.
#include "UnioModel.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kd/Check.hpp>
#include <kd/ScopedHook.hpp>
#include <kvasir/Devices/11AA02E48.hpp>
#include <optional>

using namespace Kvasir::Test;
using Kvasir::Test::Gpio::Line;
using Kvasir::Test::Gpio::Scio;
using Kvasir::Test::Gpio::UnioClock;
using Kvasir::Test::Gpio::UnioModel;

namespace {

using Mac = std::optional<std::array<std::byte, 6>>;

void fresh() {
    UnioClock::reset();
    UnioClock::advance(std::chrono::milliseconds{1});
    Line::reset();
    Log::reset();
}

bool same(Mac const&           got,
          std::array<std::uint8_t,
                     6> const& want) {
    if(!got) { return false; }
    return std::ranges::equal(*got, want, [](std::byte a, std::uint8_t b) {
        return std::to_integer<std::uint8_t>(a) == b;
    });
}

template<typename Timing = Kvasir::Eeprom11AA02E48::DefaultTiming>
Mac readOnce(UnioModel& m) {
    ScopedHook const attached{Line::model, &m};
    auto const       mac = Kvasir::Eeprom11AA02E48::readMacBlocking<UnioClock, Scio, Timing>();
    m.settle(Line::now());
    return mac;
}

Mac readRetry(UnioModel&  m,
              std::size_t attempts) {
    ScopedHook const attached{Line::model, &m};
    auto const       mac = Kvasir::Eeprom11AA02E48::readMacBlockingRetry<UnioClock, Scio>(attempts);
    m.settle(Line::now());
    return mac;
}

void read() {
    testCase("1 read: the EUI-48 comes back");
    fresh();
    UnioModel  m{};
    auto const mac = readOnce(m);
    check(same(mac, m.eui), "the model's EUI-48");
    checkEq(m.headers, 1, "one start header");
    checkEq(m.commands, 1, "one READ, ended by NoMAK + SAK");
    checkEq(m.address, 0x00FA, "read from 0xFA (DS 7.2)");
    checkEq(m.teUs, 50U, "T_E = 2 x halfBit");
    checkEq(m.maks.size(), 6U, "six data bytes");
    check(m.maks.size() == 6
            && std::all_of(m.maks.begin(), m.maks.begin() + 5, [](bool b) { return b; })
            && !m.maks[5],
          "MAK after bytes 1-5, NoMAK after byte 6 (DS 4.1)");
    check(m.errors.empty(), m.errors.empty() ? "" : m.errors.front().c_str());
    checkEq(Log::warnings, 0, "no warning");
}

void lineRule() {
    testCase("2 the line rule: released in every slot the part owns");
    fresh();
    UnioModel  m{};
    auto const mac = readOnce(m);
    check(mac.has_value(), "read");
    checkEq(m.contention, 0, "the master never drove low against the part");
    check(Line::released(), "the line is released after the read");
}

void noPart() {
    testCase("3 no part: nothing, and one warning");
    fresh();
    UnioModel m{};
    m.present      = false;
    auto const mac = readOnce(m);
    check(!mac.has_value(), "no EUI");
    checkEq(m.headers, 1, "the master sent its header");
    checkEq(Log::warnings, 1, "one warning for the read, not one per bad bit");
    check(Line::released(), "the line is released after the read");
}

void corruptedHalfBit() {
    testCase("4 a half bit without its middle edge in byte 3");
    fresh();
    UnioModel m{};
    m.corrupt      = {2, 4};
    auto const mac = readOnce(m);
    check(!mac.has_value(), "the read is refused");
    checkEq(Log::warnings, 1, "one warning");
    checkEq(m.commands, 1, "the part finished its READ");
}

void missingSak() {
    testCase("4b no SAK after data byte 4");
    fresh();
    UnioModel m{};
    m.withholdSak  = 3;
    auto const mac = readOnce(m);
    check(!mac.has_value(), "the read is refused (DS 3.5: a NoSAK after any byte is an error)");
    checkEq(m.commands, 1, "the part finished its READ");
}

void noLogInTimedSection() {
    testCase("5 no log line between two half bits");
    fresh();
    UnioModel m{};
    m.corrupt      = {2, 4};
    auto const mac = readOnce(m);
    check(!mac.has_value(), "the read is refused");
    check(!m.warningsAtOp.empty(), "the model saw the operations");
    check(std::ranges::all_of(m.warningsAtOp, [](int w) { return w == 0; }),
          "Log::warnings did not move before the last pin operation (the final release)");
    checkEq(Log::warnings, 1, "the warning came after it");
}

void retries() {
    testCase("6 retries");
    {
        fresh();
        UnioModel m{};
        m.failFirstReads = 1;
        auto const mac   = readRetry(m, 3);
        check(same(mac, m.eui), "the second read checks out");
        checkEq(m.headers, 2, "two commands sent");
        checkEq(m.commands, 1, "one of them finished");
    }
    {
        fresh();
        UnioModel  m{};
        auto const mac = readRetry(m, 0);
        check(!mac.has_value(), "zero attempts: nothing");
        checkEq(m.headers, 0, "and nothing on the wire");
        checkEq(Line::ops, 0, "not one pin operation");
    }
    {
        fresh();
        UnioModel m{};
        m.present      = false;
        auto const mac = readRetry(m, 2);
        check(!mac.has_value(), "no part: nothing");
        checkEq(m.headers, 2, "two attempts");
        checkEq(Log::warnings, 2, "one warning per attempt");
    }
}

struct FastTiming {
    static constexpr std::chrono::microseconds halfBit{10};
};

void otherTiming() {
    testCase("7 halfBit 10 us: the part learns T_E = 20 us");
    fresh();
    UnioModel  m{};
    auto const mac = readOnce<FastTiming>(m);
    check(same(mac, m.eui), "the model's EUI-48");
    checkEq(m.teUs, 20U, "T_E from the header");
    check(m.errors.empty(), m.errors.empty() ? "" : m.errors.front().c_str());
}

}   // namespace

int main() {
    read();
    lineRule();
    noPart();
    corruptedHalfBit();
    missingSak();
    noLogInTimedSection();
    retries();
    otherTiming();
    return finish();
}
