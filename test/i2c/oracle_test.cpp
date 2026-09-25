// Every description's `Identity` -- what its data sheet says tells the part from any other
// (RegisterCheck.hpp) -- against the engine's identity stage, without a part.
//
// For every description that has one, a fake part is built out of the Identity alone: it answers
// the data sheet's value at the data sheet's register, and zeros everywhere else. It has to get
// through the stage, each identity register read exactly once; the same part with its identity
// bits inverted must not, must never count as identified or answering, and must not be written
// to. A second accepted value (`also`: the other half of a family) is a part of its own.
//
// The same arrays are read off real parts by i2c_testing's hardware test. A value that is wrong
// there is a data sheet, or a transcription of it, that disagrees with the silicon.
#include "Harness.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/Every.hpp>
#include <span>
#include <string>
#include <type_traits>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

/// A register part that knows only what the description's Identity says. A read of several
/// bytes walks on from the register it starts at: through the bytes of a multi-byte register,
/// then to the next register address, as both the byte-addressed and the word-addressed parts do.
template<typename Chip>
struct IdentityPart {
    enum class Says : std::uint8_t { expect, also, wrong };

    Says          says{Says::expect};
    std::uint32_t pointer{};

    [[nodiscard]] static constexpr std::size_t regBytes() { return Chip::Identity[0].regBytes; }

    FakeBusResult operator()(std::uint8_t,
                             std::span<std::byte const> sent,
                             std::span<std::byte>       recv) {
        if(sent.size() >= regBytes()) {
            pointer = 0;
            for(std::size_t i = 0; i < regBytes(); ++i) {
                pointer = (pointer << 8U) | static_cast<std::uint8_t>(sent[i]);
            }
        }
        std::size_t at  = 0;
        auto        reg = pointer;
        while(at < recv.size()) {
            RegisterCheck const* hit = nullptr;
            for(auto const& c : Chip::Identity) {
                if(c.reg == reg) { hit = &c; }
            }
            if(hit == nullptr) {
                recv[at++] = std::byte{0};
            } else {
                auto value = hit->expect;
                if(says == Says::also && hit->also != RegisterCheck::None) { value = hit->also; }
                if(says == Says::wrong) { value = hit->expect ^ hit->mask; }
                for(std::size_t b = 0; b < hit->width && at < recv.size(); ++b) {
                    auto const shift = hit->bigEndian ? 8U * (hit->width - 1U - b) : 8U * b;
                    recv[at++]       = static_cast<std::byte>((value >> shift) & 0xFFU);
                }
            }
            ++reg;
        }
        return FakeBusResult::succeeded;
    }
};

/// The description's own Init reads a register its Identity names: a second read of what the
/// engine has read already, and a second place for the value to live.
template<typename Chip>
constexpr bool readsIdentityItself() {
    if constexpr(requires { std::span<Step const>{Chip::Init}; }) {
        for(auto const& s : Chip::Init) {
            if(s.kind != Step::Kind::read || !s.hasRegister || s.regFromBuffer) { continue; }
            for(auto const& c : Chip::Identity) {
                // a block read that starts below the register and runs over it counts too
                if(c.reg >= s.reg && c.reg < static_cast<unsigned>(s.reg) + s.count) {
                    return true;
                }
            }
        }
    }
    return false;
}

template<typename Chip>
constexpr bool hasAlso() {
    for(auto const& c : Chip::Identity) {
        if(c.also != RegisterCheck::None) { return true; }
    }
    return false;
}

/// Set by the chips that still read an identity register themselves: the list of work left.
std::string readTwice{};

template<typename Chip>
void identityFromTheDataSheet() {
    using D = Device<FakeBus, FakeClock, Chip>;
    if constexpr(D::HasIdentity) {
        using Part = IdentityPart<Chip>;
        std::string const name{Chip::Name};

        auto const passes = [&](typename Part::Says says, std::string const& what) {
            fresh();
            Part part{.says = says};
            FakeBus::respond = std::ref(part);
            D d{};
            check(runUntil(d, [&] { return d.identityMatched(); }, 5s), (name + what).c_str());
            for(std::size_t i = 0; i < Chip::Identity.size(); ++i) {
                auto const& c = Chip::Identity[i];
                auto const  wanted
                  = says == Part::Says::also && c.also != RegisterCheck::None ? c.also : c.expect;
                checkEq(d.identity(i) & c.mask,
                        wanted,
                        (name + ": identity(i) is what was read").c_str());
            }
        };

        passes(Part::Says::expect, ": the part the data sheet describes passes the identity stage");
        // Read once: the engine's read is the only one.
        if(readsIdentityItself<Chip>()) { readTwice += name + " "; }
        if constexpr(hasAlso<Chip>()) {
            passes(Part::Says::also, ": and so does the other part of the family");
        }
        {
            fresh();
            Part part{.says = Part::Says::wrong};
            FakeBus::respond = std::ref(part);
            D d{};
            runFor(d, 3s);
            std::size_t writes = 0;
            for(auto const& x : FakeBus::log) { writes += x.isWrite() ? 1U : 0U; }
            check(
              !d.identityMatched() && !d.identified() && !d.answering() && d.unidentified() >= 1,
              (name + ": a part with another identity is never identified or answering").c_str());
            checkEq(writes, std::size_t{0}, (name + ": and is not written to").c_str());
        }
    }
}

template<typename L>
struct ForEach;

template<typename... Chips>
struct ForEach<List<Chips...>> {
    static void run() { (identityFromTheDataSheet<Chips>(), ...); }
};

}   // namespace

int main() {
    testCase("identity: every description's data sheet identity, through the engine's stage");
    ForEach<Chips::Every>::run();
    check(readTwice.empty(),
          ("an identity register is read by the description too: " + readTwice).c_str());
    return finish();
}
