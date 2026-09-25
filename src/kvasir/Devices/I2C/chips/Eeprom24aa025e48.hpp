#pragma once

#include "../Device.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Microchip 24AA025E48 2-Kbit EEPROM with a factory-programmed EUI-48 node address. A
/// one-byte word address; the lower 128 bytes are the user area and the upper half is
/// write protected, with the EUI-48 at 0xFA..0xFF (Linux at24.c's 24aa025e48 entry maps the
/// MAC from offset 0, which is the user area, not the node address). A page write must not cross a
/// `PageBytes` boundary and takes up to 5 ms, which is the delay on the write step.
/// 0x50..0x57 by A2..A0; the 6-lead SOT-23 has no A2 pin and answers at 0x50..0x53 only.
///
/// The EUI-48 is read once during bring-up and kept in State, so `state().eui48` has it
/// from the moment the part is up. Reads are variable length:
/// `request<Block>({address, length})`, clamped to the end of the 256-byte array; writes are
/// `set<Page>({address, data, length})`, and only as much of the data goes out as fits
/// before the end of the page the address is in and the end of the user area
/// (`Page::fits()`, which the application can check first -- a write group cannot refuse a
/// value, so a page that would not fit is trimmed, and one entirely outside the user area
/// sends the address alone).
template<std::size_t PageBytes = 16, std::size_t ChunkBytes = 16>
struct Eeprom24aa025e48 {
    static constexpr std::string_view Name          = "24AA025E48";
    static constexpr Address7         Address       = 0x50;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array<Address7, 8>
      Addresses{0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};

    /// The part's write buffer is 16 bytes; a write past its page boundary wraps to the start
    /// of the same page and overwrites it (6.2), so a page here must fit the part's whole.
    static_assert(PageBytes >= 1 && 16 % PageBytes == 0,
                  "a page divides the 24AA025E48's 16-byte page: 1, 2, 4, 8 or 16");
    static_assert(ChunkBytes >= 1 && ChunkBytes <= 128,
                  "a chunk is 1..128 bytes");

    /// The whole array, readable.
    static constexpr std::size_t Capacity = 256;

    /// Everything at or above this is write protected.
    static constexpr std::uint8_t UserAreaBytes = 0x80;

    static constexpr std::array Init{Step::read({.reg = 0xFA, .count = 6, .offset = 0})};

    struct State {
        std::array<std::uint8_t, 6> eui48{};
    };

    /// The EUI-48 is factory programmed and never all zeroes or all ones, which is the only
    /// thing worth checking about it.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        bool allZero = true;
        bool allOnes = true;
        for(std::size_t i = 0; i < 6; ++i) {
            state.eui48[i] = data.u8(i);
            if(state.eui48[i] != 0x00) { allZero = false; }
            if(state.eui48[i] != 0xFF) { allOnes = false; }
        }
        return !allZero && !allOnes;
    }

    struct Block {
        struct Request {
            std::uint8_t address{};
            std::uint8_t length{ChunkBytes};
        };

        static constexpr std::array Steps{
          Step::readIndirect({.regOffset = 0, .count = ChunkBytes, .offset = 1})};

        /// How many bytes a request reads: its length, at most a chunk, and never past the
        /// end of the array (the part would wrap to address 0).
        [[nodiscard]] static constexpr std::size_t fits(Request const& r) {
            return std::min({static_cast<std::size_t>(r.length),
                             ChunkBytes,
                             Capacity - static_cast<std::size_t>(r.address)});
        }

        [[nodiscard]] static constexpr std::size_t prepare(Request const&       r,
                                                           std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(r.address);
            return fits(r);
        }

        struct Sample {
            std::uint8_t                         address{};
            std::uint8_t                         length{};
            std::array<std::uint8_t, ChunkBytes> data{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.address = data.u8(0);
            sample.length  = static_cast<std::uint8_t>(data.size() - 1);
            for(std::size_t i = 0; i < sample.length; ++i) { sample.data[i] = data.u8(1 + i); }
            return sample;
        }
    };

    /// One page write: an address in the user area, the bytes, and how many of them.
    struct PageData {
        std::uint8_t                        address{};
        std::array<std::uint8_t, PageBytes> data{};
        std::uint8_t                        length{PageBytes};
    };

    struct Page {
        using Value = PageData;

        static constexpr std::size_t Bytes = PageBytes;

        /// A page write is a one-shot command: replaying it after a reset would burn another
        /// of the part's finite write cycles.
        static constexpr bool Transient = true;

        /// How many of the value's bytes one page write takes: none outside the user area,
        /// else up to the end of the page the address is in (the part wraps within the page)
        /// and the end of the user area.
        [[nodiscard]] static constexpr std::uint8_t fits(Value const& value) {
            if(value.address >= UserAreaBytes) { return 0; }
            auto const address = static_cast<std::size_t>(value.address);
            auto const n       = std::min({static_cast<std::size_t>(value.length),
                                           PageBytes - address % PageBytes,
                                           static_cast<std::size_t>(UserAreaBytes) - address});
            return static_cast<std::uint8_t>(n);
        }

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            auto const n = fits(value);
            for(std::size_t i = 0; i < n; ++i) { buffer[i] = std::byte{value.data[i]}; }
            return Step::writeBuffer(
              {.reg    = value.address,
               .offset = 0,
               .count  = n,
               .delay  = std::chrono::milliseconds{5}});   // the 5 ms write cycle
        }
    };

    using Reads  = List<Block>;
    using Writes = List<Page>;
};

}   // namespace Kvasir::I2C::Chips
