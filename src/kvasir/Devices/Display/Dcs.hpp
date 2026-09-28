#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

/// The MIPI DCS command set, and a compact form for vendor init tables.
namespace Kvasir { namespace Display { namespace Dcs {

    /// The shared user command set (CO5300 7.4, ST77916 13.2, ST77922 13.2).
    struct Cmd {
        static constexpr std::uint8_t Nop       = 0x00;
        static constexpr std::uint8_t Swreset   = 0x01;
        static constexpr std::uint8_t Rddid     = 0x04;   // ID1, ID2, ID3
        static constexpr std::uint8_t Rddpm     = 0x0A;   // power mode
        static constexpr std::uint8_t Rddcolmod = 0x0C;   // pixel format read-back
        static constexpr std::uint8_t Slpin     = 0x10;
        static constexpr std::uint8_t Slpout    = 0x11;
        static constexpr std::uint8_t Noron     = 0x13;
        static constexpr std::uint8_t Invoff    = 0x20;
        static constexpr std::uint8_t Invon     = 0x21;
        static constexpr std::uint8_t Dispoff   = 0x28;
        static constexpr std::uint8_t Dispon    = 0x29;
        static constexpr std::uint8_t Caset     = 0x2A;
        static constexpr std::uint8_t Raset     = 0x2B;
        static constexpr std::uint8_t Ramwr     = 0x2C;
        static constexpr std::uint8_t Teoff     = 0x34;
        static constexpr std::uint8_t Teon      = 0x35;
        static constexpr std::uint8_t Madctl    = 0x36;
        static constexpr std::uint8_t Colmod    = 0x3A;
        static constexpr std::uint8_t Ramwrc    = 0x3C;
        static constexpr std::uint8_t Wrdisbv   = 0x51;   // brightness, DBV[7:0]
        static constexpr std::uint8_t Wrctrld   = 0x53;   // gates brightness control
    };

    /// MADCTL (36h) bits.
    struct Madctl {
        static constexpr std::uint8_t RowOrder    = 0x80;   // MY: bottom to top
        static constexpr std::uint8_t ColumnOrder = 0x40;   // MX: right to left
        static constexpr std::uint8_t Exchange    = 0x20;   // MV: rows and columns swapped
        static constexpr std::uint8_t ScanOrder   = 0x10;   // ML: refresh bottom to top
        static constexpr std::uint8_t Bgr         = 0x08;   // panel wired BGR
        static constexpr std::uint8_t Horizontal  = 0x04;   // MH: refresh right to left
    };

    struct Step {
        std::uint8_t                  cmd{};
        std::span<std::uint8_t const> params{};
        std::uint8_t                  delayMs{};   ///< wait after sending, before the next
    };

    /// A vendor init table as a byte stream, `cmd, count, params..., delayMs` per step: a
    /// fraction of the flash of fixed-size structs. wellFormed() makes a miscounted entry a
    /// build error.
    class Script {
    public:
        constexpr Script() = default;

        constexpr explicit Script(std::span<std::uint8_t const> bytes) : bytes_{bytes} {}

        [[nodiscard]] static constexpr bool wellFormed(std::span<std::uint8_t const> bytes) {
            std::size_t pos = 0;
            while(pos < bytes.size()) {
                if(pos + 2 > bytes.size()) { return false; }
                std::size_t const count = bytes[pos + 1];
                pos += 2 + count + 1;
                if(pos > bytes.size()) { return false; }
            }
            return true;
        }

        [[nodiscard]] constexpr std::size_t size() const { return bytes_.size(); }

        [[nodiscard]] constexpr bool empty() const { return bytes_.empty(); }

        [[nodiscard]] constexpr std::size_t steps() const {
            std::size_t n   = 0;
            std::size_t pos = 0;
            while(pos < bytes_.size()) {
                pos += 3U + std::size_t{bytes_[pos + 1]};
                ++n;
            }
            return n;
        }

        /// The step at byte offset `pos`, advancing `pos`; false at the end.
        [[nodiscard]] constexpr bool next(std::size_t& pos,
                                          Step&        out) const {
            if(pos >= bytes_.size()) { return false; }
            std::size_t const count = bytes_[pos + 1];
            out.cmd                 = bytes_[pos];
            out.params              = bytes_.subspan(pos + 2, count);
            out.delayMs             = bytes_[pos + 2 + count];
            pos += 3 + count;
            return true;
        }

    private:
        std::span<std::uint8_t const> bytes_{};
    };

}}}   // namespace Kvasir::Display::Dcs
