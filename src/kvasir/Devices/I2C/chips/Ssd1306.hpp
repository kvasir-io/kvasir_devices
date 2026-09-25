#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Ssd1306Detail {

    /// The command set the three families share (SSD1306 rev 1.1 section 9, SSD1309,
    /// SH1106 v2.3).
    struct Cmd {
        static constexpr std::uint8_t ColumnLow  = 0x00;   // | low nibble
        static constexpr std::uint8_t ColumnHigh = 0x10;   // | high nibble
        static constexpr std::uint8_t AddressMode
          = 0x20;   // SSD1306/1309: 00 horizontal, 01 vertical, 02 page
        static constexpr std::uint8_t StartLine     = 0x40;   // | line
        static constexpr std::uint8_t Contrast      = 0x81;
        static constexpr std::uint8_t ChargePump    = 0x8D;   // SSD1306: 14h on, 10h off
        static constexpr std::uint8_t SegmentRemap  = 0xA0;   // A1h: column 127 -> SEG0
        static constexpr std::uint8_t AllOnResume   = 0xA4;
        static constexpr std::uint8_t Normal        = 0xA6;
        static constexpr std::uint8_t Inverse       = 0xA7;
        static constexpr std::uint8_t Multiplex     = 0xA8;
        static constexpr std::uint8_t DcDc          = 0xAD;   // SH1106: 8Bh on, 8Ah off
        static constexpr std::uint8_t DisplayOff    = 0xAE;
        static constexpr std::uint8_t DisplayOn     = 0xAF;
        static constexpr std::uint8_t Page          = 0xB0;   // | page
        static constexpr std::uint8_t ScanNormal    = 0xC0;
        static constexpr std::uint8_t ScanRemap     = 0xC8;   // COM[N-1] -> COM0: flips
        static constexpr std::uint8_t DisplayOffset = 0xD3;
        static constexpr std::uint8_t ClockDivide   = 0xD5;
        static constexpr std::uint8_t Precharge     = 0xD9;
        static constexpr std::uint8_t ComPins       = 0xDA;
        static constexpr std::uint8_t VcomDeselect  = 0xDB;
    };

    /// The control byte that opens a transaction: 0x00 says "commands follow to the STOP",
    /// 0x40 "RAM data follows" (SSD1306 8.1.5.2, SH1106 "I2C-bus Interface").
    static constexpr std::uint8_t CommandStream = 0x00;
    static constexpr std::uint8_t DataStream    = 0x40;

    /// What differs between the controllers.
    template<typename C>
    concept Controller = requires {
        { C::Name } -> std::convertible_to<std::string_view>;
        { C::RamColumns } -> std::convertible_to<int>;
        { C::MaxRows } -> std::convertible_to<int>;
        { C::HasAddressMode } -> std::convertible_to<bool>;          // has the 20h command
        { C::ChargePumpOn } -> std::convertible_to<std::uint16_t>;   // 0: none; else cmd<<8|arg
        { C::PrechargeInternal } -> std::convertible_to<std::uint8_t>;
        { C::PrechargeExternal } -> std::convertible_to<std::uint8_t>;
        { C::VcomDeselect } -> std::convertible_to<std::uint8_t>;
    };

    /// Solomon Systech SSD1306, 128 x 64, the 0.96" modules. Internal charge pump
    /// (8Dh 14h, "Charge Pump Setting"), pre-charge F1h with it (common practice, as in
    /// Adafruit's library: the datasheet gives only the D9h reset default 22h, for external
    /// VCC, and the application note does not send D9h; Linux ssd130x.c uses 22h for every
    /// variant). VCOMH deselect 40h is Adafruit's value and outside the ones the DBh table
    /// documents (00h, 20h, 30h); Linux sends 20h.
    struct Ssd1306C {
        static constexpr std::string_view Name              = "SSD1306";
        static constexpr int              RamColumns        = 128;
        static constexpr int              MaxRows           = 64;
        static constexpr bool             HasAddressMode    = true;
        static constexpr std::uint16_t    ChargePumpOn      = 0x8D14;
        static constexpr std::uint8_t     PrechargeInternal = 0xF1;
        static constexpr std::uint8_t     PrechargeExternal = 0x22;
        static constexpr std::uint8_t     VcomDeselect      = 0x40;
    };

    /// The SSD1306's successor, found on many current small modules (for example those
    /// marked NFP1315-xx). Same command set and RAM layout; 14h is the SSD1306-compatible
    /// charge pump setting (the SSD1315 adds 15h for a higher VCC). Its own policy so a log
    /// line says which chip a module has, and so a difference between them has a place to go.
    struct Ssd1315C : Ssd1306C {
        static constexpr std::string_view Name = "SSD1315";
    };

    /// SSD1309, 128 x 64, the 2.42" modules. No charge pump: VCC is external, which is why
    /// its datasheet has no 8Dh.
    struct Ssd1309C {
        static constexpr std::string_view Name              = "SSD1309";
        static constexpr int              RamColumns        = 128;
        static constexpr int              MaxRows           = 64;
        static constexpr bool             HasAddressMode    = true;
        static constexpr std::uint16_t    ChargePumpOn      = 0;
        static constexpr std::uint8_t     PrechargeInternal = 0x22;
        static constexpr std::uint8_t     PrechargeExternal = 0x22;
        static constexpr std::uint8_t     VcomDeselect      = 0x34;
    };

    /// Sino Wealth SH1106, 132 x 64, the 1.3" modules. RAM is 132 columns wide and a 128 px
    /// panel is usually centred: set ColumnOffset 2. Page addressing only, DC-DC on by
    /// ADh 8Bh ("Set DC-DC OFF/ON").
    struct Sh1106C {
        static constexpr std::string_view Name              = "SH1106";
        static constexpr int              RamColumns        = 132;
        static constexpr int              MaxRows           = 64;
        static constexpr bool             HasAddressMode    = false;
        static constexpr std::uint16_t    ChargePumpOn      = 0xAD8B;
        static constexpr std::uint8_t     PrechargeInternal = 0x22;
        static constexpr std::uint8_t     PrechargeExternal = 0x22;
        static constexpr std::uint8_t     VcomDeselect      = 0x35;
    };

    /// The knobs a panel may leave out, with what they are then; a panel derives from this
    /// and redeclares what it sets. ComPins has no fixed default (it follows the height) and
    /// stays a lambda in the description.
    /// Segment remap (x) or COM scan direction (y): as wired, or mirrored.
    enum class Mirror : std::uint8_t { none, mirrored };

    /// Where the panel's VCC comes from: the controller's charge pump or the module.
    enum class Vcc : std::uint8_t { chargePump, external };

    /// The Power group's value: the display on (AFh) or off (AEh).
    enum class Display : std::uint8_t { off, on };

    /// The Invert group's value: the RAM shown as it is (A6h) or inverted (A7h).
    enum class Inversion : std::uint8_t { normal, inverted };

    struct PanelDefaults {
        static constexpr int          ColumnOffset  = 0;              ///< panel column 0 in RAM
        static constexpr Mirror       FlipX         = Mirror::none;   ///< segment remap
        static constexpr Mirror       FlipY         = Mirror::none;   ///< COM scan direction
        static constexpr std::uint8_t Contrast      = 0x7F;           ///< 81h
        static constexpr Vcc          Supply        = Vcc::chargePump;
        static constexpr std::uint8_t DisplayOffset = 0;   ///< D3h
    };

    /// The panel the catalogue names, and what an unqualified Ssd1306<> is: the 0.96" module.
    struct Panel128x64 : PanelDefaults {
        static constexpr int      Width   = 128;
        static constexpr int      Height  = 64;
        static constexpr Address7 Address = 0x3C;
    };

    /// One command and its arguments: the unit the bring-up script is packed in, because a
    /// command must never be separated from its argument by a STOP.
    struct Unit {
        std::array<std::uint8_t, 2> b{};
        std::uint8_t                n{};
    };

    /// At most this many units in a bring-up; the builder asserts it does not overflow.
    static constexpr std::size_t MaxUnits = 16;

    struct Script {
        std::array<Unit, MaxUnits> units{};
        std::size_t                n{};

        constexpr void push(std::uint8_t a) {
            units[n++] = Unit{
              {a, 0},
              1
            };
        }

        constexpr void push(std::uint8_t a,
                            std::uint8_t b) {
            units[n++] = Unit{
              {a, b},
              2
            };
        }
    };

}   // namespace Ssd1306Detail

/// A monochrome OLED on I2C -- SSD1306, SSD1315, SSD1309 or SH1106 -- as a description.
/// The three families share their command set and their RAM layout: a byte is eight
/// vertically adjacent pixels of one column, bit 0 topmost, and the bytes run column by
/// column across a "page" of eight rows. A framebuffer stored in that layout is written
/// with no repacking.
///
/// From the SSD1306 rev 1.1, SSD1309 and SH1106 v2.3 datasheets.
///
/// Protocol: a write is the address, then a control byte and bytes -- 0x00 "commands
/// follow", 0x40 "RAM data follows", to the STOP. One transaction per command batch, and
/// two per page of RAM: the page and column window, then the page's bytes. Page addressing
/// on all four, which is all the SH1106 has, so there is one code path.
///
/// `set<Page>(p, bytes)` writes one page; the write group's values *are* the frame, so a
/// canvas can draw into them (`items<Page>()` and `touch<Page>(p)`) and only the pages that
/// changed go on the wire. The bring-up leaves the display off, `Page::Initial` blanks
/// every page and `Power::Initial` turns it on -- in that order, because write groups are
/// served in list order -- so the random RAM a cold controller holds is never shown.
///
/// The panel policy derives from `Ssd1306Detail::PanelDefaults`:
///   Width, Height     the panel; Height a multiple of 8 the controller can drive
///   Address           0x3C (SA0 low, the usual) or 0x3D
///   ColumnOffset (0)  where panel column 0 sits in RAM (2 on a 128 px SH1106)
///   FlipX, FlipY      (none) mount orientation, by segment remap and COM scan
///   Contrast (0x7F)   81h
///   Supply            (chargePump) Vcc::external: the module supplies VCC, no charge pump
///   ComPins           DAh: how the COM lines are wired to the glass. 12h (alternative)
///                     above 32 rows, 02h (sequential) for the 128 x 32 modules. The small
///                     SSD1315 panels (72 x 40, 64 x 48, 64 x 32) all want 12h, so a
///                     64 x 32 one has to say so.
///   DisplayOffset (0) D3h: the COM line the first RAM row is shown on
template<Ssd1306Detail::Controller Ctrl, typename Panel = Ssd1306Detail::Panel128x64>
struct MonoOled {
    using Cmd        = Ssd1306Detail::Cmd;
    using Controller = Ctrl;

    static_assert(std::derived_from<Panel,
                                    Ssd1306Detail::PanelDefaults>,
                  "derive the panel from Kvasir::I2C::Chips::Ssd1306Detail::PanelDefaults");

    static constexpr std::string_view Name          = Ctrl::Name;
    static constexpr Address7         Address       = Panel::Address;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array<Address7, 2> Addresses{0x3C, 0x3D};

    /// Only used when the device is given a Reset line; most I2C modules tie RES# to VDD
    /// through an RC and have none.
    static constexpr auto ResetLow    = std::chrono::milliseconds{10};
    static constexpr auto ResetSettle = std::chrono::milliseconds{10};

    static constexpr int Width  = static_cast<int>(Panel::Width);
    static constexpr int Height = static_cast<int>(Panel::Height);
    static constexpr int Pages  = Height / 8;

    /// 12h: alternative COM pin configuration, what a 64 row panel is wired for; 02h:
    /// sequential, for the 128 x 32 modules (SSD1306 10.1.18). The one default that depends
    /// on another member.
    static constexpr std::uint8_t ComPins = [] {
        if constexpr(requires { Panel::ComPins; }) {
            return static_cast<std::uint8_t>(Panel::ComPins);
        } else {
            return Height > 32 ? std::uint8_t{0x12} : std::uint8_t{0x02};
        }
    }();

    static_assert(Height % 8 == 0 && Height >= 16 && Height <= Ctrl::MaxRows,
                  "the height must be a multiple of 8 the controller can drive");
    static_assert(Width > 0 && Panel::ColumnOffset + Width <= Ctrl::RamColumns,
                  "the panel does not fit the controller's RAM columns");
    static_assert(Panel::DisplayOffset < Ctrl::MaxRows,
                  "D3h takes 0..MaxRows-1");
    static_assert(Address == 0x3C || Address == 0x3D,
                  "these controllers answer at 0x3C or 0x3D");
    static_assert(Pages <= 32,
                  "one dirty bit per page, and a write group has at most 32 items");

private:
    /// The power-on configuration as a list of commands, each with its arguments, built at
    /// compile time from the controller and the panel. It ends with the display still off:
    /// Power turns it on once a blank frame has been written.
    static constexpr auto Commands = [] {
        Ssd1306Detail::Script s{};
        s.push(Cmd::DisplayOff);
        // divide by 1, oscillator mid-range: the SSD1306's reset value (the SSD1309 resets to
        // 70h, the SH1106 to 50h), and what these modules are run at
        s.push(Cmd::ClockDivide, 0x80);
        s.push(Cmd::Multiplex, static_cast<std::uint8_t>(Height - 1));
        s.push(Cmd::DisplayOffset, Panel::DisplayOffset);
        s.push(static_cast<std::uint8_t>(Cmd::StartLine | 0x00));
        if constexpr(Ctrl::ChargePumpOn != 0) {
            if constexpr(Panel::Supply == Ssd1306Detail::Vcc::chargePump) {
                s.push(static_cast<std::uint8_t>(Ctrl::ChargePumpOn >> 8U),
                       static_cast<std::uint8_t>(Ctrl::ChargePumpOn & 0xFFU));
            }
        }
        if constexpr(Ctrl::HasAddressMode) {
            s.push(Cmd::AddressMode, 0x02);   // page addressing, which is all the SH1106 has
        }
        s.push(static_cast<std::uint8_t>(
          Cmd::SegmentRemap | (Panel::FlipX == Ssd1306Detail::Mirror::mirrored ? 0x01U : 0x00U)));
        s.push(Panel::FlipY == Ssd1306Detail::Mirror::mirrored ? Cmd::ScanRemap : Cmd::ScanNormal);
        s.push(Cmd::ComPins, ComPins);
        s.push(Cmd::Contrast, Panel::Contrast);
        s.push(Cmd::Precharge,
               Panel::Supply == Ssd1306Detail::Vcc::external ? Ctrl::PrechargeExternal
                                                             : Ctrl::PrechargeInternal);
        s.push(Cmd::VcomDeselect, Ctrl::VcomDeselect);
        s.push(Cmd::AllOnResume);
        s.push(Cmd::Normal);
        return s;
    }();

    static_assert(Commands.n <= Ssd1306Detail::MaxUnits,
                  "the bring-up script outgrew Ssd1306Detail::MaxUnits");

    /// How many transactions the commands pack into: a control byte and up to seven command
    /// bytes each (Step::InlineBytes is 8), never splitting a command from its argument.
    static constexpr auto Packing = [] {
        struct Chunk {
            std::size_t first{};
            std::size_t last{};   // one past
        };

        struct P {
            std::array<Chunk, Ssd1306Detail::MaxUnits> chunks{};
            std::size_t                                n{};
        } p{};

        std::size_t i = 0;
        while(i < Commands.n) {
            std::size_t bytes = 0;
            std::size_t j     = i;
            while(j < Commands.n && bytes + Commands.units[j].n <= Step::InlineBytes - 1) {
                bytes += Commands.units[j].n;
                ++j;
            }
            p.chunks[p.n++] = Chunk{i, j};
            i               = j;
        }
        return p;
    }();

public:
    static constexpr auto Init = [] {
        std::array<Step, Packing.n> a{};
        for(std::size_t c = 0; c < Packing.n; ++c) {
            Step st{};
            st.kind        = Step::Kind::write;
            st.hasRegister = false;
            st.bytes[0]    = Ssd1306Detail::CommandStream;
            std::uint8_t k = 1;
            for(std::size_t u = Packing.chunks[c].first; u < Packing.chunks[c].last; ++u) {
                for(std::uint8_t b = 0; b < Commands.units[u].n; ++b) {
                    st.bytes[k++] = Commands.units[u].b[b];
                }
            }
            st.count = k;
            a[c]     = st;
        }
        return a;
    }();

    /// One page of the display RAM: eight rows, one byte per column, bit 0 the topmost.
    /// Two transactions -- the page and column window, then the bytes -- because the
    /// control byte cannot change inside one.
    struct Page {
        using Value = std::array<std::uint8_t, static_cast<std::size_t>(Width)>;
        static constexpr std::size_t Items = static_cast<std::size_t>(Pages);
        static constexpr std::size_t Bytes = 4 + 1 + static_cast<std::size_t>(Width);
        /// A blank frame at the first bring-up. After a later reset the pages carry what is
        /// on the glass, because Initial is only installed while nothing has been written.
        static constexpr Value Initial{};

        [[nodiscard]] static constexpr std::array<Step,
                                                  2>
        encode(Value const&         value,
               std::size_t          page,
               std::span<std::byte> buffer) {
            auto const col = static_cast<unsigned>(Panel::ColumnOffset);
            buffer[0]      = std::byte{Ssd1306Detail::CommandStream};
            buffer[1]      = static_cast<std::byte>(Cmd::Page | static_cast<unsigned>(page));
            buffer[2]      = static_cast<std::byte>(Cmd::ColumnLow | (col & 0xFU));
            buffer[3]      = static_cast<std::byte>(Cmd::ColumnHigh | ((col >> 4U) & 0xFU));
            buffer[4]      = std::byte{Ssd1306Detail::DataStream};
            for(std::size_t i = 0; i < static_cast<std::size_t>(Width); ++i) {
                buffer[5 + i] = static_cast<std::byte>(value[i]);
            }
            return {
              Step::commandBuffer({.offset = 0, .count = 4}),
              Step::commandBuffer({.offset = 4, .count = static_cast<std::uint8_t>(1 + Width)})};
        }
    };

    /// The display on or off (AFh / AEh). On after the first blank frame; see the note
    /// above about the order write groups are served in.
    struct Power {
        using Value                          = Ssd1306Detail::Display;
        static constexpr std::size_t Bytes   = 2;
        static constexpr Value       Initial = Value::on;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{Ssd1306Detail::CommandStream};
            buffer[1] = std::byte{value == Value::on ? Cmd::DisplayOn : Cmd::DisplayOff};
            return Step::commandBuffer({.offset = 0, .count = 2});
        }
    };

    /// 81h: the segment current, 0..255. The bring-up sets the panel's, so this group only
    /// costs traffic when the application changes it.
    struct Contrast {
        using Value                        = std::uint8_t;
        static constexpr std::size_t Bytes = 3;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{Ssd1306Detail::CommandStream};
            buffer[1] = std::byte{Cmd::Contrast};
            buffer[2] = static_cast<std::byte>(value);
            return Step::commandBuffer({.offset = 0, .count = 3});
        }
    };

    /// A7h / A6h: show the RAM inverted, without redrawing it.
    struct Invert {
        using Value                        = Ssd1306Detail::Inversion;
        static constexpr std::size_t Bytes = 2;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{Ssd1306Detail::CommandStream};
            buffer[1] = std::byte{value == Value::inverted ? Cmd::Inverse : Cmd::Normal};
            return Step::commandBuffer({.offset = 0, .count = 2});
        }
    };

    // Page first: at every bring-up the frame goes out before Power turns the panel on.
    using Writes = List<Page, Power, Contrast, Invert>;

    // No Reads: these controllers are written, never asked. The first Init transaction is
    // what the base probes an absent module with.
};

template<typename Panel = Ssd1306Detail::Panel128x64>
using Ssd1306 = MonoOled<Ssd1306Detail::Ssd1306C, Panel>;

template<typename Panel = Ssd1306Detail::Panel128x64>
using Ssd1315 = MonoOled<Ssd1306Detail::Ssd1315C, Panel>;

template<typename Panel = Ssd1306Detail::Panel128x64>
using Ssd1309 = MonoOled<Ssd1306Detail::Ssd1309C, Panel>;

template<typename Panel = Ssd1306Detail::Panel128x64>
using Sh1106 = MonoOled<Ssd1306Detail::Sh1106C, Panel>;

}   // namespace Kvasir::I2C::Chips
