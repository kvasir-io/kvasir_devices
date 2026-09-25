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
/// one and ends in "~"), and `names(a)` the names one by one, never cut.
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
        if(static_cast<std::uint8_t>(C::Address) == address) { return true; }
        if constexpr(detail::HasAddresses<C>) {
            for(auto const a : C::Addresses) {
                if(a == address) { return true; }
            }
        }
        return false;
    }

    static constexpr std::size_t AddressCount = 0x78;

    /// The hints as the compiler puts them together, a fixed-width Entry an address. Only ever
    /// read in constant expressions (Packed below is what goes into the image): 120 entries of
    /// Width + 5 bytes, mostly zeros, were 20 KB of flash in every firmware with a bus scan until
    /// 2026-09-20.
    static constexpr std::array<Entry, AddressCount> Table = [] {
        std::array<Entry, AddressCount> t{};
        for(std::size_t a = 0; a < t.size(); ++a) {
            ((answersAt<Cs>(static_cast<std::uint8_t>(a)) ? t[a].append(Cs::Name) : void()), ...);
        }
        return t;
    }();

    /// The characters of all hints together.
    static constexpr std::size_t HintBytes = [] {
        std::size_t n = 0;
        for(auto const& e : Table) { n += e.length; }
        return n;
    }();

    /// The hints back to back, and where each address's starts: what is in the image.
    struct PackedHints {
        std::array<char, HintBytes == 0 ? 1 : HintBytes> text{};
        std::array<std::uint16_t, AddressCount + 1>      start{};
    };

    static constexpr PackedHints Packed = [] {
        PackedHints   p{};
        std::uint16_t n = 0;
        for(std::size_t a = 0; a < AddressCount; ++a) {
            p.start[a] = n;
            for(std::size_t i = 0; i < Table[a].length; ++i) { p.text[n++] = Table[a].text[i]; }
        }
        p.start[AddressCount] = n;
        return p;
    }();
    static_assert(HintBytes <= 0xFFFF);

    /// How many (address, chip) pairs there are in all: the size of the name table.
    static constexpr std::size_t EntryCount = [] {
        std::size_t n = 0;
        for(std::size_t a = 0; a < AddressCount; ++a) {
            ((answersAt<Cs>(static_cast<std::uint8_t>(a)) ? void(++n) : void()), ...);
        }
        return n;
    }();

    /// Every name, address by address; `Offsets[a]` is where address a's names start.
    static constexpr std::array<std::string_view, EntryCount == 0 ? 1 : EntryCount> Names = [] {
        std::array<std::string_view, EntryCount == 0 ? 1 : EntryCount> names{};
        std::size_t                                                    n = 0;
        for(std::size_t a = 0; a < AddressCount; ++a) {
            ((answersAt<Cs>(static_cast<std::uint8_t>(a)) ? void(names[n++] = Cs::Name) : void()),
             ...);
        }
        return names;
    }();

    static constexpr std::array<std::uint16_t, AddressCount + 1> Offsets = [] {
        std::array<std::uint16_t, AddressCount + 1> o{};
        std::uint16_t                               n = 0;
        for(std::size_t a = 0; a < AddressCount; ++a) {
            o[a] = n;
            ((answersAt<Cs>(static_cast<std::uint8_t>(a)) ? void(++n) : void()), ...);
        }
        o[AddressCount] = n;
        return o;
    }();

    /// What answers at an address in this build: the names, or "unknown".
    [[nodiscard]] static constexpr std::string_view hint(std::uint8_t address) {
        if(address >= AddressCount || Packed.start[address] == Packed.start[address + 1]) {
            return "unknown";
        }
        return {Packed.text.data() + Packed.start[address],
                static_cast<std::size_t>(Packed.start[address + 1] - Packed.start[address])};
    }

    /// The same, one name at a time and never truncated; empty for an address nothing
    /// in this build answers at.
    [[nodiscard]] static constexpr std::span<std::string_view const> names(std::uint8_t address) {
        if(address >= AddressCount) { return {}; }
        return std::span<std::string_view const>{Names}.subspan(
          Offsets[address],
          static_cast<std::size_t>(Offsets[address + 1] - Offsets[address]));
    }
};

template<typename... Cs>
using Catalogue = SizedCatalogue<160, Cs...>;

}   // namespace Kvasir::I2C
