#pragma once

#include "Concepts.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C {

/// The chips a build knows, as one type: what a bus scan can say about an address. Every
/// chip whose `Address` or `Addresses` include it is named, so 0x48 reads "VEML6030 /
/// ADS1115 / TMP102 / LM75 / INA219", and the drivers then tell which it is. Two forms of
/// the answer, both built by the compiler: `hint(a)` is the one preformatted line of up to
/// `Width` characters (a crowded address -- 0x48 has a dozen candidates -- outgrows a short
/// one and ends in "~"), and `namesAt(a)` the names one by one, never cut.
///
/// `Catalogue<Chips...>` is this at a Width of 160; `SizedCatalogue<Width, Chips...>` for
/// another.
template<std::size_t Width, typename... Cs>
struct SizedCatalogue {
    static_assert(Width >= 8,
                  "a hint needs room for at least a name");

    struct Entry {
        std::array<char, Width + 1> text{};
        std::size_t                 length{};

        constexpr void append(std::string_view name) {
            if(length != 0) { append_(" / "); }
            append_(name);
        }

        [[nodiscard]] constexpr std::string_view view() const { return {text.data(), length}; }

    private:
        constexpr void append_(std::string_view part) {
            for(auto const c : part) {
                if(length + 1 >= Width) {
                    text[Width - 1] = '~';   // truncated
                    length          = Width;
                    return;
                }
                text[length++] = c;
            }
        }
    };

    /// Whether chip C may answer at an address: its default `Address`, or any of the
    /// `Addresses` it can be strapped to.
    template<typename C>
    [[nodiscard]] static constexpr bool answersAt(std::uint8_t address) {
        if(static_cast<std::uint8_t>(C::address) == address) { return true; }
        if constexpr(Detail::HasAddresses<C>) {
            for(auto const a : C::addresses) {
                if(a == address) { return true; }
            }
        }
        return false;
    }

    static constexpr std::size_t addressCount = 0x78;

    /// The hints as the compiler puts them together, a fixed-width Entry an address. Only ever
    /// read in constant expressions (Packed below is what goes into the image): 120 entries of
    /// Width + 5 bytes, mostly zeros, were 20 KB of flash in every firmware with a bus scan before
    /// Packed.
    /// Which of the chips answers at each address, folded once over chips x addresses: the tables
    /// below read it instead of folding again.
    static constexpr std::array<std::array<bool, sizeof...(Cs)>, addressCount> answers = [] {
        std::array<std::array<bool, sizeof...(Cs)>, addressCount> t{};
        for(std::size_t a = 0; a < addressCount; ++a) {
            std::size_t c = 0;
            ((t[a][c++] = answersAt<Cs>(static_cast<std::uint8_t>(a))), ...);
            static_cast<void>(c);
        }
        return t;
    }();

    static constexpr std::array<Entry, addressCount> table = [] {
        std::array<Entry, addressCount> t{};
        for(std::size_t a = 0; a < t.size(); ++a) {
            std::size_t c = 0;
            ((answers[a][c++] ? t[a].append(Cs::name) : void()), ...);
            static_cast<void>(c);
        }
        return t;
    }();

    /// The characters of all hints together.
    static constexpr std::size_t hintBytes = [] {
        std::size_t n = 0;
        for(auto const& e : table) { n += e.length; }
        return n;
    }();

    /// The hints back to back, and where each address's starts: what is in the image.
    struct PackedHints {
        std::array<char, hintBytes == 0 ? 1 : hintBytes> text{};
        std::array<std::uint16_t, addressCount + 1>      start{};
    };

    static constexpr PackedHints packed = [] {
        PackedHints   p{};
        std::uint16_t n = 0;
        for(std::size_t a = 0; a < addressCount; ++a) {
            p.start[a] = n;
            for(std::size_t i = 0; i < table[a].length; ++i) { p.text[n++] = table[a].text[i]; }
        }
        p.start[addressCount] = n;
        return p;
    }();
    static_assert(hintBytes <= 0xFFFF);

    /// How many (address, chip) pairs there are in all: the size of the name table.
    static constexpr std::size_t entryCount = [] {
        std::size_t n = 0;
        for(auto const& at : answers) {
            for(bool const answersHere : at) { n += answersHere ? 1U : 0U; }
        }
        return n;
    }();

    /// Every name, address by address; `Offsets[a]` is where address a's names start.
    static constexpr std::array<std::string_view, entryCount == 0 ? 1 : entryCount> names = [] {
        std::array<std::string_view, entryCount == 0 ? 1 : entryCount> namesHere{};
        std::size_t                                                    n = 0;
        for(std::size_t a = 0; a < addressCount; ++a) {
            std::size_t c = 0;
            ((answers[a][c++] ? void(namesHere[n++] = Cs::name) : void()), ...);
            static_cast<void>(c);
        }
        return namesHere;
    }();

    static constexpr std::array<std::uint16_t, addressCount + 1> offsets = [] {
        std::array<std::uint16_t, addressCount + 1> o{};
        std::uint16_t                               n = 0;
        for(std::size_t a = 0; a < addressCount; ++a) {
            o[a] = n;
            for(bool const answersHere : answers[a]) {
                n = static_cast<std::uint16_t>(n + (answersHere ? 1U : 0U));
            }
        }
        o[addressCount] = n;
        return o;
    }();

    /// What answers at an address in this build: the names, or "unknown".
    [[nodiscard]] static constexpr std::string_view hint(std::uint8_t address) {
        if(address >= addressCount || packed.start[address] == packed.start[address + 1]) {
            return "unknown";
        }
        return {packed.text.data() + packed.start[address],
                static_cast<std::size_t>(packed.start[address + 1] - packed.start[address])};
    }

    /// The same, one name at a time and never truncated; empty for an address nothing
    /// in this build answers at.
    [[nodiscard]] static constexpr std::span<std::string_view const> namesAt(std::uint8_t address) {
        if(address >= addressCount) { return {}; }
        return std::span<std::string_view const>{names}.subspan(
          offsets[address],
          static_cast<std::size_t>(offsets[address + 1] - offsets[address]));
    }
};

template<typename... Cs>
using Catalogue = SizedCatalogue<160, Cs...>;

}   // namespace Kvasir::I2C
