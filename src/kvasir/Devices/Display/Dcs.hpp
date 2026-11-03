#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

/// The MIPI DCS command set, and a compact form for vendor init tables.
namespace Kvasir { namespace Display { namespace Dcs {

    /// The shared user command set (CO5300 7.4, ST77916 13.2, ST77922 13.2).
    struct Cmd {
        static constexpr std::uint8_t nop       = 0x00;
        static constexpr std::uint8_t swreset   = 0x01;
        static constexpr std::uint8_t rddid     = 0x04;   // ID1, ID2, ID3
        static constexpr std::uint8_t rddpm     = 0x0A;   // power mode
        static constexpr std::uint8_t rddcolmod = 0x0C;   // pixel format read-back
        static constexpr std::uint8_t slpin     = 0x10;
        static constexpr std::uint8_t slpout    = 0x11;
        static constexpr std::uint8_t noron     = 0x13;
        static constexpr std::uint8_t invoff    = 0x20;
        static constexpr std::uint8_t invon     = 0x21;
        static constexpr std::uint8_t dispoff   = 0x28;
        static constexpr std::uint8_t dispon    = 0x29;
        static constexpr std::uint8_t caset     = 0x2A;
        static constexpr std::uint8_t raset     = 0x2B;
        static constexpr std::uint8_t ramwr     = 0x2C;
        static constexpr std::uint8_t teoff     = 0x34;
        static constexpr std::uint8_t teon      = 0x35;
        static constexpr std::uint8_t madctl    = 0x36;
        static constexpr std::uint8_t colmod    = 0x3A;
        static constexpr std::uint8_t ramwrc    = 0x3C;
        static constexpr std::uint8_t wrdisbv   = 0x51;   // brightness, DBV[7:0]
        static constexpr std::uint8_t wrctrld   = 0x53;   // gates brightness control
    };

    /// MADCTL (36h) bits.
    struct Madctl {
        static constexpr std::uint8_t rowOrder    = 0x80;   // MY: bottom to top
        static constexpr std::uint8_t columnOrder = 0x40;   // MX: right to left
        static constexpr std::uint8_t exchange    = 0x20;   // MV: rows and columns swapped
        static constexpr std::uint8_t scanOrder   = 0x10;   // ML: refresh bottom to top
        static constexpr std::uint8_t bgr         = 0x08;   // panel wired BGR
        static constexpr std::uint8_t horizontal  = 0x04;   // MH: refresh right to left
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
        std::span<std::uint8_t const> bytes_;
    };

}}}   // namespace Kvasir::Display::Dcs
