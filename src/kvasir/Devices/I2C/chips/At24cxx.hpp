#pragma once

#include "../Device.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Atmel / Microchip AT24C32 and AT24C64 serial EEPROM (doc0336), the onsemi CAT24C512
/// and its AT24C512 equivalent, and the smaller AT24C0x family with a one-byte address
/// (`AddressBytes` 1). A page is `PageBytes` (128 on the C512, 32 on the C32/C64, 8 on
/// the C02, 16 on the C08/C16). Random read: the address bytes, then a repeated START and
/// the data ("Read Operations"); page write: the address and up to a page of data that
/// must not cross a page boundary, then the internally timed write cycle, 10 ms max (tWR).
/// There is no cyclic reading: `request<ReadPage>({ address })` reads a page,
/// `set<WritePage>({ address, data, length })` writes one -- as much of it as fits before
/// the end of the page the address is in and the end of the memory (`WritePage::fits`).
/// 0x50..0x57 by A2..A0.
///
/// The write cycle is `Timing::WriteTime` long (10 ms, the 2.7 V and 5 V tWR; a part run at
/// 1.8 V needs 20 ms: `struct Slow { static constexpr auto WriteTime = 20ms; };`). A part
/// still in its write cycle does not acknowledge its address ("All inputs are disabled during
/// this write cycle"), so a NAK is also put back on the wire a few times, 5 ms apart, rather
/// than counted -- Linux at24.c polls the same way for up to 25 ms.
///
/// A one-byte address reaches 256 bytes. The AT24C08 and AT24C16 spend one or two of the
/// device-address bits on the upper 256-byte blocks (they answer at several addresses,
/// one per block), which this description does not model: `MemoryBytes` above 256 needs
/// `AddressBytes` 2.
template<std::size_t PageBytes    = 32,
         std::size_t AddressBytes = 2,
         std::size_t MemoryBytes  = 4096,
         typename Timing          = DefaultTiming>
struct At24cxx {
    static constexpr std::string_view Name          = "AT24Cxx";
    static constexpr Address7         Address       = 0x50;
    static constexpr std::size_t      RegisterBytes = AddressBytes;

    static constexpr std::chrono::milliseconds WriteTime = [] {
        if constexpr(requires { Timing::WriteTime; }) {
            return Kvasir::asDuration(Timing::WriteTime);
        } else {
            return std::chrono::milliseconds{10};
        }
    }();

    static constexpr std::uint8_t WakeRetries    = 4;
    static constexpr auto         WakeRetryDelay = std::chrono::milliseconds{5};

    static constexpr std::array<Address7, 8>
      Addresses{0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};

    static_assert(AddressBytes == 1 || AddressBytes == 2,
                  "the word address is one or two bytes");
    static_assert(PageBytes >= 1 && PageBytes <= 128,
                  "a page is 1..128 bytes");
    static_assert(MemoryBytes >= PageBytes && MemoryBytes <= 65536 && MemoryBytes % PageBytes == 0,
                  "the memory is whole pages, and the address bytes reach 65536 at most");
    static_assert(AddressBytes == 2 || MemoryBytes <= 256,
                  "a one-byte word address reaches 256 bytes; the AT24C08/C16 select the blocks "
                  "above that with device-address bits, which this description does not model");

    struct ReadPage {
        struct Request {
            std::uint16_t address{};
        };

        static constexpr void prepare(Request const&       r,
                                      std::span<std::byte> buffer) {
            if constexpr(AddressBytes == 2) {
                putBe16(buffer, 0, r.address);
            } else {
                buffer[0] = static_cast<std::byte>(r.address & 0xFF);
            }
        }

        static constexpr std::array Steps{
          Step::readIndirect({.regOffset = 0, .count = PageBytes, .offset = 2})};

        struct Sample {
            std::uint16_t                       address{};
            std::array<std::uint8_t, PageBytes> data{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.address = AddressBytes == 2 ? data.be16(0) : data.u8(0);
            for(std::size_t i = 0; i < PageBytes; ++i) { sample.data[i] = data.u8(2 + i); }
            return sample;
        }
    };

    /// One page write: where, the bytes, and how many of them.
    struct PageData {
        std::uint16_t                       address{};
        std::array<std::uint8_t, PageBytes> data{};
        std::uint8_t                        length{PageBytes};
    };

    struct WritePage {
        using Value = PageData;

        static constexpr std::size_t Bytes = PageBytes;

        /// A page write is a one-shot command: replaying it after a reset would burn
        /// another of the part's finite write cycles.
        static constexpr bool Transient = true;

        /// How many of the value's bytes one page write takes: a write may not cross a page
        /// boundary (the part would wrap within the page), nor run past the end of the memory.
        [[nodiscard]] static constexpr std::uint8_t fits(Value const& value) {
            if(value.address >= MemoryBytes) { return 0; }
            std::size_t n = value.length;
            n             = std::min(n, PageBytes - value.address % PageBytes);
            n             = std::min(n, MemoryBytes - value.address);
            return static_cast<std::uint8_t>(n);
        }

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            auto const n = fits(value);
            for(std::size_t i = 0; i < n; ++i) { buffer[i] = std::byte{value.data[i]}; }
            return Step::writeBuffer(
              {.reg = value.address, .offset = 0, .count = n, .delay = WriteTime});
        }
    };

    using Reads  = List<ReadPage>;
    using Writes = List<WritePage>;
};

using At24c32 = At24cxx<32, 2, 4096>;
using At24c02 = At24cxx<8, 1, 256>;

/// 512 kbit (65536 bytes, 0x0000..0xFFFF) with a 128-byte page: the CAT24C512 and the
/// AT24C512. Its whole memory is behind one I2C address -- unlike the 1 Mbit parts, which
/// spend an address bit on the upper half -- so A2..A0 still give eight of them.
using At24c512 = At24cxx<128, 2, 65536>;

}   // namespace Kvasir::I2C::Chips
