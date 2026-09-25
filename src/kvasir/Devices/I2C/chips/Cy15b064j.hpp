#pragma once

#include "../Device.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Infineon CY15B064J 64-Kbit (8K x 8) automotive F-RAM. A two-byte word address, then the
/// data; a random read is the address followed by a repeated START. Unlike an EEPROM there
/// is no page boundary and no write cycle at all -- a write completes at bus speed -- so a
/// write step carries no delay and there is no busy poll.
///
/// `ChunkBytes` is the most one transaction moves, and both groups are variable length:
/// `request<Block>({address, length})` reads that many bytes, `set<Store>({address, data,
/// length})` writes them. Both are clamped to `Capacity`: a read past the end is shortened
/// (an address at or past it reads the last byte), a write past the end is trimmed to what
/// fits and one entirely past it sends the address alone -- `Block::fits()` and
/// `Store::fits()` say how much would go, for an application that wants to check first.
template<std::size_t ChunkBytes = 64>
struct Cy15b064j {
    static constexpr std::string_view Name          = "CY15B064J";
    static constexpr Address7         Address       = 0x50;
    static constexpr std::size_t      RegisterBytes = 2;

    static constexpr std::array<Address7, 8>
      Addresses{0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};

    static_assert(ChunkBytes >= 1 && ChunkBytes <= 128,
                  "a chunk is 1..128 bytes");

    /// The whole 8 KB address space; there is no protected region.
    static constexpr std::uint16_t Capacity = 0x2000;

    /// A one-byte read from address zero: the bring-up transaction, and the probe an absent
    /// part is looked for with.
    static constexpr std::array Init{Step::read({.reg = 0x0000, .count = 1, .offset = 0})};

    struct Block {
        struct Request {
            std::uint16_t address{};
            std::uint8_t  length{ChunkBytes};
        };

        /// The address goes at the front of the buffer and the data behind it.
        static constexpr std::array Steps{
          Step::readIndirect({.regOffset = 0, .count = ChunkBytes, .offset = 2})};

        /// The address a request reads from: clamped to the last byte of the array.
        [[nodiscard]] static constexpr std::uint16_t start(Request const& r) {
            return r.address < Capacity ? r.address : static_cast<std::uint16_t>(Capacity - 1);
        }

        /// How many bytes it reads: its length, at most a chunk, and not past the end.
        [[nodiscard]] static constexpr std::size_t fits(Request const& r) {
            return std::min({static_cast<std::size_t>(r.length),
                             ChunkBytes,
                             static_cast<std::size_t>(Capacity) - start(r)});
        }

        /// Returns how many bytes to read, so one description serves every length.
        [[nodiscard]] static constexpr std::size_t prepare(Request const&       r,
                                                           std::span<std::byte> buffer) {
            putBe16(buffer, 0, start(r));
            return fits(r);
        }

        struct Sample {
            std::uint16_t                        address{};
            std::uint8_t                         length{};
            std::array<std::uint8_t, ChunkBytes> data{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.address = data.be16(0);
            sample.length  = static_cast<std::uint8_t>(data.size() - 2);
            for(std::size_t i = 0; i < sample.length; ++i) { sample.data[i] = data.u8(2 + i); }
            return sample;
        }
    };

    /// One write: where, the bytes, and how many of them.
    struct Chunk {
        std::uint16_t                        address{};
        std::array<std::uint8_t, ChunkBytes> data{};
        std::uint8_t                         length{ChunkBytes};
    };

    struct Store {
        using Value = Chunk;

        static constexpr std::size_t Bytes = ChunkBytes;

        /// Writing memory is a one-shot command, not a state to restore after a reset.
        static constexpr bool Transient = true;

        /// How many of the value's bytes go out: none from an address past the end, else up
        /// to a chunk and the end of the array.
        [[nodiscard]] static constexpr std::uint8_t fits(Value const& value) {
            if(value.address >= Capacity) { return 0; }
            auto const n = std::min({static_cast<std::size_t>(value.length),
                                     ChunkBytes,
                                     static_cast<std::size_t>(Capacity) - value.address});
            return static_cast<std::uint8_t>(n);
        }

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            auto const n = fits(value);
            for(std::size_t i = 0; i < n; ++i) { buffer[i] = std::byte{value.data[i]}; }
            return Step::writeBuffer({.reg    = value.address,
                                      .offset = 0,
                                      .count  = n});   // F-RAM: no write cycle to wait out
        }
    };

    using Reads  = List<Block>;
    using Writes = List<Store>;
};

}   // namespace Kvasir::I2C::Chips
