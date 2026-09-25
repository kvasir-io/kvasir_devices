#pragma once

/// What every I2C test shares: the fake bus and clock, the checks, one loop turn, a run until a
/// predicate holds or a duration is up, the transcript helpers, the responders most cases want
/// and the device models a command-style chip needs. Include it first: it brings the log stubs
/// in before any driver.
#include <algorithm>
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <map>
#include <span>
#include <tuple>
#include <vector>

// clang-format off
#include <support/LogStubs.hpp>
#include "FakeBus.hpp"
#include "Check.hpp"
// clang-format on
#include <kvasir/Devices/Bytes.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Quantities.hpp>

namespace Kvasir::Test {

using namespace std::chrono_literals;

/// The helpers below work on the untagged FakeBus unless they are given the tags of the buses
/// to work on, first: `turn<I2c0, I2c1>(left, right)` completes both buses in the one turn,
/// `writes<I2c1>()` is I2C1's transcript. The tags are an explicit pack in front of the
/// deduced parameters, so a call without them works on the untagged bus.
namespace HarnessImpl {
    template<typename... Tags>
    void complete() {
        if constexpr(sizeof...(Tags) == 0) {
            FakeBus::complete();
        } else {
            (FakeBusFor<Tags>::complete(), ...);
        }
    }
}   // namespace HarnessImpl

/// One loop turn: every handler, the bus (every tagged bus) completes what was submitted, a
/// millisecond passes.
template<typename... Tags,
         typename... Ds>
void turn(Ds&... ds) {
    (ds.handler(), ...);
    HarnessImpl::complete<Tags...>();
    FakeClock::current += 1ms;
}

/// Several devices handled as one, for runUntil() and runFor(): `runUntil(all(mux, left), ...)`.
template<typename... Ds>
struct All {
    std::tuple<Ds&...> devices;

    void handler() {
        std::apply([](auto&... d) { (d.handler(), ...); }, devices);
    }
};

template<typename... Ds>
All<Ds...> all(Ds&... ds) {
    return {std::tie(ds...)};
}

/// Turns until `done()` holds or `maxTurns` have gone by; whether it held.
template<typename... Tags,
         typename D,
         typename P>
bool runUntil(D&& d,
              P   done,
              int maxTurns) {
    for(int i = 0; i < maxTurns; ++i) {
        if(done()) { return true; }
        turn<Tags...>(d);
    }
    return done();
}

/// Turns until `done()` holds or the clock has moved `within`; whether it held.
template<typename... Tags,
         typename D,
         typename P,
         typename Rep,
         typename Period>
bool runUntil(D&&                           d,
              P                             done,
              std::chrono::duration<Rep,
                                    Period> within) {
    auto const until = FakeClock::now() + std::chrono::duration_cast<FakeClock::duration>(within);
    while(!done()) {
        if(FakeClock::now() >= until) { return false; }
        turn<Tags...>(d);
    }
    return true;
}

/// Turns until the clock has moved `span`.
template<typename... Tags,
         typename D,
         typename Rep,
         typename Period>
void runFor(D&&                           d,
            std::chrono::duration<Rep,
                                  Period> span) {
    auto const until = FakeClock::now() + std::chrono::duration_cast<FakeClock::duration>(span);
    while(FakeClock::now() < until) { turn<Tags...>(d); }
}

/// The transcript's writes from entry `from` on, as byte vectors.
template<typename Tag = void>
std::vector<std::vector<std::uint8_t>> writes(std::size_t from = 0) {
    auto const&                            log = FakeBusFor<Tag>::log;
    std::vector<std::vector<std::uint8_t>> w;
    for(std::size_t i = from; i < log.size(); ++i) {
        if(log[i].isWrite()) { w.push_back(log[i].sent); }
    }
    return w;
}

template<typename Tag = void>
bool hasWrite(std::vector<std::uint8_t> const& bytes,
              std::size_t                      from = 0) {
    for(auto const& w : writes<Tag>(from)) {
        if(w == bytes) { return true; }
    }
    return false;
}

/// Where in the transcript the first write of exactly `bytes` at or after `from` is; the
/// transcript's size when there is none.
template<typename Tag = void>
std::size_t findWrite(std::vector<std::uint8_t> const& bytes,
                      std::size_t                      from = 0) {
    auto const& log = FakeBusFor<Tag>::log;
    for(std::size_t i = from; i < log.size(); ++i) {
        if(log[i].isWrite() && log[i].sent == bytes) { return i; }
    }
    return log.size();
}

/// The transcript, for a failing case.
template<typename Tag = void>
void dump() {
    for(auto const& t : FakeBusFor<Tag>::log) {
        std::printf("    %s %02x:", t.isRead() ? "R" : "W", t.address);
        for(auto const b : t.sent) { std::printf(" %02x", b); }
        if(t.isRead()) { std::printf(" -> %zu", t.recvLen); }
        std::printf("\n");
    }
}

/// A clean bus (every tagged bus), the clock a second past its epoch, the log counters at zero.
template<typename... Tags>
void fresh() {
    if constexpr(sizeof...(Tags) == 0) {
        FakeBus::reset();
    } else {
        (FakeBusFor<Tags>::reset(), ...);
    }
    FakeClock::current = FakeClock::time_point{} + 1s;
    Log::reset();
}

template<typename C, typename Cfg = Kvasir::I2C::DefaultConfig>
using Dev = Kvasir::I2C::Device<FakeBus, FakeClock, C, Cfg>;

/// A device on the bus tagged `Tag`.
template<typename Tag, typename C, typename Cfg = Kvasir::I2C::DefaultConfig>
using DevOn = Kvasir::I2C::Device<FakeBusFor<Tag>, FakeClock, C, Cfg>;

// -- compile-time decode checks -----------------------------------------------------------
//
// A chip's decode is constexpr, so a fixed frame in and the Sample out is a static_assert next
// to the runtime case that runs the same part through the engine:
//     static_assert([] {
//         auto const f = frame(0x04, 0xB0);
//         auto const s = Chips::Bh1750::Measurement::decode(Bytes{f});
//         return equal(s.raw, 1200U) && equal(s.lux(), 1000000U);
//     }());

/// Bytes as a decode sees them: `frame(0x04, 0xB0)`.
template<typename... B>
    requires(std::integral<B> && ...)
constexpr std::array<std::byte,
                     sizeof...(B)>
frame(B... b) {
    return {static_cast<std::byte>(b)...};
}

/// What a decode that returns an Outcome made of the frame.
template<typename S>
constexpr bool isOk(Kvasir::I2C::Outcome<S> const& o) {
    return o.kind == Kvasir::I2C::Outcome<S>::Kind::ok;
}

template<typename S>
constexpr bool isReject(Kvasir::I2C::Outcome<S> const& o) {
    return o.kind == Kvasir::I2C::Outcome<S>::Kind::reject;
}

template<typename S>
constexpr bool isRetry(Kvasir::I2C::Outcome<S> const& o) {
    return o.kind == Kvasir::I2C::Outcome<S>::Kind::retry;
}

template<typename S>
constexpr bool isUnchanged(Kvasir::I2C::Outcome<S> const& o) {
    return o.kind == Kvasir::I2C::Outcome<S>::Kind::unchanged;
}

/// A temperature quantity back as the hundredths of a degree the checks are written in.
template<typename Q>
constexpr std::int32_t centiOf(Q q) {
    return Kvasir::Units::CentiDegC{q}.numerical_value_in(
      mp_units::si::centi<mp_units::si::degree_Celsius>);
}

// -- responders ---------------------------------------------------------------------------

/// Acks everything at any address and answers every read with zeros.
inline FakeBusResult zeros(std::uint8_t,
                           std::span<std::byte const>,
                           std::span<std::byte> recv) {
    for(auto& b : recv) { b = std::byte{0}; }
    return FakeBusResult::succeeded;
}

/// Nothing there: every request is NAKed.
inline FakeBusResult alwaysNak(std::uint8_t,
                               std::span<std::byte const>,
                               std::span<std::byte>) {
    return FakeBusResult::notAcknowledged;
}

/// A broken wire: every request fails without a NAK.
inline FakeBusResult alwaysFault(std::uint8_t,
                                 std::span<std::byte const>,
                                 std::span<std::byte>) {
    return FakeBusResult::failed;
}

/// Sensirion words of zero with their CRC (0x81), which the other parts do not mind: what a
/// bus of Sensirion and register parts answers so that every one of them comes up.
inline FakeBusResult crcZeros(std::uint8_t,
                              std::span<std::byte const>,
                              std::span<std::byte> recv) {
    for(std::size_t i = 0; i < recv.size(); ++i) {
        recv[i] = std::byte{static_cast<std::uint8_t>(i % 3 == 2 ? 0x81 : 0x00)};
    }
    return FakeBusResult::succeeded;
}

/// The switches on the wire (TCA9548As at `switches`): each keeps its control byte and
/// answers the engine's read-back (Chips::Tca9548a::Channels verifies) with it; everything
/// else acks and answers `behind`, zeros unless a case says otherwise.
struct SwitchWire {
    std::vector<std::uint8_t>            switches{0x70};
    std::map<std::uint8_t, std::uint8_t> control{};
    FakeBusResponder                     behind{zeros};

    FakeBusResult operator()(std::uint8_t               addr,
                             std::span<std::byte const> sent,
                             std::span<std::byte>       recv) {
        if(std::find(switches.begin(), switches.end(), addr) == switches.end()) {
            return behind(addr, sent, recv);
        }
        auto& c = control[addr];
        if(!sent.empty()) { c = static_cast<std::uint8_t>(sent[0]); }
        for(auto& b : recv) { b = std::byte{c}; }
        return FakeBusResult::succeeded;
    }
};

/// Claims `gate` until the switch has been written for its channel, running the mux
/// meanwhile; whether it got there.
template<typename Gate,
         typename Mux>
bool settle(Gate& gate,
            Mux&  mux) {
    for(int i = 0; i < 50 && !gate.claim(); ++i) { turn(mux); }
    return gate.claim();
}

// -- device models ------------------------------------------------------------------------

inline std::vector<std::uint8_t> withCrc(std::uint16_t word) {
    auto const f = Sensirion::framed(word);
    return {f[0], f[1], f[2]};
}

/// Sensirion words, each with its CRC, back to back.
inline std::vector<std::uint8_t> words(std::initializer_list<std::uint16_t> ws) {
    std::vector<std::uint8_t> out;
    for(auto const w : ws) {
        auto const f = withCrc(w);
        out.insert(out.end(), f.begin(), f.end());
    }
    return out;
}

/// A command-style device (no registers): `reply` answers the next bare read after the
/// command `cmd`; `onCommand` sees every command.
struct CommandModel {
    std::uint8_t                                       address{};
    std::map<std::uint16_t, std::vector<std::uint8_t>> replies{};
    std::function<void(std::uint16_t)>                 onCommand{};
    std::vector<std::uint16_t>                         commands{};
    std::uint16_t                                      last{};

    explicit CommandModel(std::uint8_t a) : address{a} {}

    FakeBusResult operator()(std::uint8_t               addr,
                             std::span<std::byte const> sent,
                             std::span<std::byte>       recv) {
        if(addr != address) { return FakeBusResult::notAcknowledged; }
        if(!sent.empty()) {
            last = sent.size() >= 2
                   ? static_cast<std::uint16_t>((static_cast<std::uint8_t>(sent[0]) << 8)
                                                | static_cast<std::uint8_t>(sent[1]))
                   : static_cast<std::uint8_t>(sent[0]);
            commands.push_back(last);
            if(onCommand) { onCommand(last); }
            return FakeBusResult::succeeded;
        }
        auto const it = replies.find(last);
        if(it == replies.end() || it->second.size() != recv.size()) {
            return FakeBusResult::notAcknowledged;
        }
        for(std::size_t i = 0; i < recv.size(); ++i) {
            recv[i] = static_cast<std::byte>(it->second[i]);
        }
        return FakeBusResult::succeeded;
    }
};

}   // namespace Kvasir::Test
