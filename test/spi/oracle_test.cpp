/// Every SPI description's `Identity` (SPI/chips/Every.hpp) through the engine's identity stage - the
/// I2C oracle test for SPI. A fake part answering the Identity must pass; with the bits inverted it
/// must never be identified nor written to (on SPI nothing else keeps a driver off a foreign part).
#include "Harness.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kd/Check.hpp>
#include <kvasir/Devices/SPI/chips/Every.hpp>
#include <optional>
#include <span>
#include <string>
#include <support/FakeClock.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Engine::RegisterCheck;
using Kvasir::Test::Spi::Pins;

namespace {

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;

/// The description as it is, with an identified() hook that records what the engine hands it
/// and passes it on: the engine keeps no copy of the identity (it lives in the State only), so
/// the values handed to the hook are what the oracle test judges.
template<typename Chip>
struct Seen {
    static inline std::vector<std::uint32_t> ids{};
};

template<typename Chip>
struct Spy : Chip {
    template<typename State>
    static void identified(std::span<std::uint32_t const> ids,
                           State&                         state) {
        Seen<Chip>::ids.assign(ids.begin(), ids.end());
        if constexpr(requires { Chip::identified(ids, state); }) { Chip::identified(ids, state); }
    }
};

/// A part that knows only the Identity, in the description's own command bytes.
template<typename Chip>
struct IdentityPart {
    enum class Says : std::uint8_t { expect, also, wrong };

    Says          says{Says::expect};
    std::size_t   writes{};
    std::uint32_t seenRises{};
    bool          haveCommand{};
    bool          reading{};
    std::size_t   dummy{};
    unsigned      reg{};
    std::size_t   byteInReg{};

    /// Which register a command byte names. On SPI a register address may lose a bit to R/W (the
    /// BME280 carries 7 of 8: 0xD0 and 0x50 read alike), so the Identity's registers are tried first.
    static std::optional<std::pair<bool,
                                   unsigned>>
    decode(std::uint8_t c) {
        for(auto const& id : Chip::Identity) {
            if(Kvasir::SPI::detail::readCommand<Chip>(static_cast<std::uint8_t>(id.reg)) == c) {
                return std::pair{true, unsigned{id.reg}};
            }
        }
        for(unsigned r = 0; r < 256U; ++r) {
            if(Kvasir::SPI::detail::readCommand<Chip>(static_cast<std::uint8_t>(r)) == c) {
                return std::pair{true, r};
            }
        }
        for(unsigned r = 0; r < 256U; ++r) {
            if(Kvasir::SPI::detail::writeCommand<Chip>(static_cast<std::uint8_t>(r)) == c) {
                return std::pair{false, r};
            }
        }
        return std::nullopt;
    }

    [[nodiscard]] std::uint8_t valueByte() const {
        for(auto const& c : Chip::Identity) {
            if(c.reg != reg) { continue; }
            auto value = c.expect;
            if(says == Says::also && c.also != RegisterCheck::None) { value = c.also; }
            if(says == Says::wrong) { value = c.expect ^ c.mask; }
            auto const shift = c.bigEndian ? 8U * (c.width - 1U - byteInReg) : 8U * byteInReg;
            return static_cast<std::uint8_t>((value >> shift) & 0xFFU);
        }
        return 0;
    }

    std::uint8_t exchange(std::uint8_t mosi) {
        if(Pins::rises[Spi::Cs::id] != seenRises) {
            seenRises   = Pins::rises[Spi::Cs::id];
            haveCommand = false;
        }
        if(!haveCommand) {
            haveCommand  = true;
            auto const d = decode(mosi);
            reading      = d && d->first;
            reg          = d ? d->second : 0U;
            byteInReg    = 0;
            dummy        = Kvasir::SPI::detail::dummyBytes<Chip>();
            if(!reading) { ++writes; }
            return 0xFF;
        }
        if(!reading) { return 0xFF; }
        if(dummy != 0) {
            --dummy;
            return 0xFF;
        }
        auto const  b     = valueByte();
        std::size_t width = 1;
        for(auto const& c : Chip::Identity) {
            if(c.reg == reg) { width = c.width; }
        }
        if(++byteInReg >= width) {
            byteInReg = 0;
            ++reg;
        }
        return b;
    }
};

template<typename Chip>
constexpr bool hasAlso() {
    for(auto const& c : Chip::Identity) {
        if(c.also != RegisterCheck::None) { return true; }
    }
    return false;
}

/// 100 us turns, the clock last; the part answers on the byte.
constexpr Spi::Loop loop{.step = 100us, .order = Spi::ClockOrder::last};

template<typename D,
         typename Pred>
bool runUntil(D&                        d,
              Pred                      pred,
              std::chrono::milliseconds limit) {
    return Spi::Harness::runUntil<Bus>(d, pred, limit, loop);
}

template<typename Part>
void fresh(Part& part) {
    Spi::Harness::fresh<Bus>([&part](std::uint8_t m) { return part.exchange(m); }, 0s);
}

std::size_t withIdentity{};

template<typename Chip>
void identityFromTheDataSheet() {
    using D = Kvasir::SPI::Device<Bus, FakeClock, Chip, Spi::Cs>;
    std::string const name{Chip::Name};
    if constexpr(D::HasIdentity) {
        ++withIdentity;
        using Part = IdentityPart<Chip>;

        auto const passes = [&](typename Part::Says says, std::string const& what) {
            Part part{.says = says};
            fresh(part);
            Seen<Chip>::ids.clear();
            Kvasir::SPI::Device<Bus, FakeClock, Spy<Chip>, Spi::Cs> d{};
            check(runUntil(d, [&] { return d.identityMatched(); }, 2s), name + what);
            checkEq(Seen<Chip>::ids.size(),
                    Chip::Identity.size(),
                    name + ": identified() is handed every value");
            for(std::size_t i = 0; i < Chip::Identity.size() && i < Seen<Chip>::ids.size(); ++i) {
                auto const& c = Chip::Identity[i];
                auto const  wanted
                  = says == Part::Says::also && c.also != RegisterCheck::None ? c.also : c.expect;
                checkEq(Seen<Chip>::ids[i] & c.mask,
                        wanted,
                        name + ": identified() is handed what was read");
            }
            Bus::reset();   // what is still queued completes into this device, not the next case's
        };

        passes(Part::Says::expect, ": the part the data sheet describes passes the identity stage");
        if constexpr(hasAlso<Chip>()) {
            passes(Part::Says::also, ": and so does the other part of the family");
        }
        {
            Part part{.says = Part::Says::wrong};
            fresh(part);
            D          d{};
            auto const end = FakeClock::now() + 3s;
            runUntil(d, [&] { return FakeClock::now() >= end; }, 4s);
            check(!d.identityMatched() && !d.identified() && !d.answering()
                    && d.unidentified() >= 1,
                  name + ": a part with another identity is never identified or answering");
            checkEq(part.writes, std::size_t{0}, name + ": and is not written to");
            Bus::reset();
        }
    } else {
        // No identity register: the description judges its Init's read-back, or is write-only and says so.
        constexpr bool judges    = requires { &Chip::setup; };
        constexpr bool writeOnly = requires { Chip::WriteOnly; };
        check(judges || writeOnly, name + ": no Identity, and no setup() that judges a read-back");
    }
}

template<typename L>
struct ForEach;

template<typename... Chips>
struct ForEach<Kvasir::Engine::List<Chips...>> {
    static void run() { (identityFromTheDataSheet<Chips>(), ...); }
};

}   // namespace

int main() {
    testCase("identity: every SPI description's data sheet identity, through the engine's stage");
    ForEach<Kvasir::SPI::Chips::Every>::run();
    check(withIdentity >= 3, "the BME280, BMP280 and MPU-9250 have one");
    return finish();
}
