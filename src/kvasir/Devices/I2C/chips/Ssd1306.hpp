#pragma once

#include "../../Engine/Description.hpp"
#include "../Address.hpp"
#include "kvasir/Util/Config.hpp"

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
        static constexpr std::uint8_t columnLow  = 0x00;   // | low nibble
        static constexpr std::uint8_t columnHigh = 0x10;   // | high nibble
        static constexpr std::uint8_t addressMode
          = 0x20;   // SSD1306/1309: 00 horizontal, 01 vertical, 02 page
        static constexpr std::uint8_t startLine     = 0x40;   // | line
        static constexpr std::uint8_t contrast      = 0x81;
        static constexpr std::uint8_t chargePump    = 0x8D;   // SSD1306: 14h on, 10h off
        static constexpr std::uint8_t segmentRemap  = 0xA0;   // A1h: column 127 -> SEG0
        static constexpr std::uint8_t allOnResume   = 0xA4;
        static constexpr std::uint8_t normal        = 0xA6;
        static constexpr std::uint8_t inverse       = 0xA7;
        static constexpr std::uint8_t multiplex     = 0xA8;
        static constexpr std::uint8_t dcDc          = 0xAD;   // SH1106: 8Bh on, 8Ah off
        static constexpr std::uint8_t displayOff    = 0xAE;
        static constexpr std::uint8_t displayOn     = 0xAF;
        static constexpr std::uint8_t page          = 0xB0;   // | page
        static constexpr std::uint8_t scanNormal    = 0xC0;
        static constexpr std::uint8_t scanRemap     = 0xC8;   // COM[N-1] -> COM0: flips
        static constexpr std::uint8_t displayOffset = 0xD3;
        static constexpr std::uint8_t clockDivide   = 0xD5;
        static constexpr std::uint8_t precharge     = 0xD9;
        static constexpr std::uint8_t comPins       = 0xDA;
        static constexpr std::uint8_t vcomDeselect  = 0xDB;
    };

    /// The control byte that opens a transaction: 0x00 says "commands follow to the STOP",
    /// 0x40 "RAM data follows" (SSD1306 8.1.5.2, SH1106 "I2C-bus Interface").
    static constexpr std::uint8_t commandStream = 0x00;
    static constexpr std::uint8_t dataStream    = 0x40;

    /// What differs between the controllers.
    template<typename C>
    concept Controller = requires {
        { C::name } -> std::convertible_to<std::string_view>;
        { C::ramColumns } -> std::convertible_to<int>;
        { C::maxRows } -> std::convertible_to<int>;
        { C::hasAddressMode } -> std::convertible_to<bool>;          // has the 20h command
        { C::chargePumpOn } -> std::convertible_to<std::uint16_t>;   // 0: none; else cmd<<8|arg
        { C::prechargeInternal } -> std::convertible_to<std::uint8_t>;
        { C::prechargeExternal } -> std::convertible_to<std::uint8_t>;
        { C::vcomDeselect } -> std::convertible_to<std::uint8_t>;
    };

    /// Solomon Systech SSD1306, 128 x 64, the 0.96" modules. Internal charge pump
    /// (8Dh 14h, "Charge Pump Setting"), pre-charge F1h with it (common practice, as in
    /// Adafruit's library: the datasheet gives only the D9h reset default 22h, for external
    /// VCC, and the application note does not send D9h; Linux ssd130x.c uses 22h for every
    /// variant). VCOMH deselect 40h is Adafruit's value and outside the ones the DBh table
    /// documents (00h, 20h, 30h); Linux sends 20h.
    struct Ssd1306C {
        static constexpr std::string_view name              = "SSD1306";
        static constexpr int              ramColumns        = 128;
        static constexpr int              maxRows           = 64;
        static constexpr bool             hasAddressMode    = true;
        static constexpr std::uint16_t    chargePumpOn      = 0x8D14;
        static constexpr std::uint8_t     prechargeInternal = 0xF1;
        static constexpr std::uint8_t     prechargeExternal = 0x22;
        static constexpr std::uint8_t     vcomDeselect      = 0x40;
    };

    /// The SSD1306's successor, found on many current small modules (for example those
    /// marked NFP1315-xx). Same command set and RAM layout; 14h is the SSD1306-compatible
    /// charge pump setting (the SSD1315 adds 15h for a higher VCC). Its own policy so a log
    /// line says which chip a module has, and so a difference between them has a place to go.
    struct Ssd1315C : Ssd1306C {
        static constexpr std::string_view name = "SSD1315";
    };

    /// SSD1309, 128 x 64, the 2.42" modules. No charge pump: VCC is external, which is why
    /// its datasheet has no 8Dh.
    struct Ssd1309C {
        static constexpr std::string_view name              = "SSD1309";
        static constexpr int              ramColumns        = 128;
        static constexpr int              maxRows           = 64;
        static constexpr bool             hasAddressMode    = true;
        static constexpr std::uint16_t    chargePumpOn      = 0;
        static constexpr std::uint8_t     prechargeInternal = 0x22;
        static constexpr std::uint8_t     prechargeExternal = 0x22;
        static constexpr std::uint8_t     vcomDeselect      = 0x34;
    };

    /// Sino Wealth SH1106, 132 x 64, the 1.3" modules. RAM is 132 columns wide and a 128 px
    /// panel is usually centred: set columnOffset 2. Page addressing only, DC-DC on by
    /// ADh 8Bh ("Set DC-DC OFF/ON").
    struct Sh1106C {
        static constexpr std::string_view name              = "SH1106";
        static constexpr int              ramColumns        = 132;
        static constexpr int              maxRows           = 64;
        static constexpr bool             hasAddressMode    = false;
        static constexpr std::uint16_t    chargePumpOn      = 0xAD8B;
        static constexpr std::uint8_t     prechargeInternal = 0x22;
        static constexpr std::uint8_t     prechargeExternal = 0x22;
        static constexpr std::uint8_t     vcomDeselect      = 0x35;
    };

    /// The knobs a panel may leave out, with what they are then; a panel derives from this
    /// and redeclares what it sets. comPins has no fixed default (it follows the height) and
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
        static constexpr int          columnOffset  = 0;              ///< panel column 0 in RAM
        static constexpr Mirror       flipX         = Mirror::none;   ///< segment remap
        static constexpr Mirror       flipY         = Mirror::none;   ///< COM scan direction
        static constexpr std::uint8_t contrast      = 0x7F;           ///< 81h
        static constexpr Vcc          supply        = Vcc::chargePump;
        static constexpr std::uint8_t displayOffset = 0;   ///< D3h
    };

    /// The panel the catalogue names, and what an unqualified Ssd1306<> is: the 0.96" module.
    struct Panel128x64 : PanelDefaults {
        static constexpr int      width   = 128;
        static constexpr int      height  = 64;
        static constexpr Address7 address = 0x3C;
    };

    /// One command and its arguments: the unit the bring-up script is packed in, because a
    /// command must never be separated from its argument by a STOP.
    struct Unit {
        std::array<std::uint8_t, 2> b{};
        std::uint8_t                n{};
    };

    /// At most this many units in a bring-up; the builder asserts it does not overflow.
    static constexpr std::size_t maxUnits = 16;

    struct Script {
        std::array<Unit, maxUnits> units{};
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
///   columnOffset (0)  where panel column 0 sits in RAM (2 on a 128 px SH1106)
///   flipX, flipY      (none) mount orientation, by segment remap and COM scan
///   contrast (0x7F)   81h
///   supply            (chargePump) Vcc::external: the module supplies VCC, no charge pump
///   comPins           DAh: how the COM lines are wired to the glass. 12h (alternative)
///                     above 32 rows, 02h (sequential) for the 128 x 32 modules. The small
///                     SSD1315 panels (72 x 40, 64 x 48, 64 x 32) all want 12h, so a
///                     64 x 32 one has to say so.
///   displayOffset (0) D3h: the COM line the first RAM row is shown on
template<Ssd1306Detail::Controller Ctrl, typename Panel = Ssd1306Detail::Panel128x64>
struct MonoOled {
    using Cmd        = Ssd1306Detail::Cmd;
    using Controller = Ctrl;

    static_assert(std::derived_from<Panel,
                                    Ssd1306Detail::PanelDefaults>,
                  "derive the panel from Kvasir::I2C::Chips::Ssd1306Detail::PanelDefaults");

    static constexpr std::string_view name          = Ctrl::name;
    static constexpr Address7         address       = Panel::address;
    static constexpr std::size_t      registerBytes = 0;

    static constexpr std::array<Address7, 2> addresses{0x3C, 0x3D};

    /// Only used when the device is given a Reset line; most I2C modules tie RES# to VDD
    /// through an RC and have none.
    static constexpr auto resetLow    = std::chrono::milliseconds{10};
    static constexpr auto resetSettle = std::chrono::milliseconds{10};

    static constexpr int width  = static_cast<int>(Panel::width);
    static constexpr int height = static_cast<int>(Panel::height);
    static constexpr int pages  = height / 8;

    /// 12h: alternative COM pin configuration, what a 64 row panel is wired for; 02h:
    /// sequential, for the 128 x 32 modules (SSD1306 10.1.18). The one default that depends
    /// on another member.
    static constexpr std::uint8_t comPins = static_cast<std::uint8_t>(
      KVASIR_CONFIG_OR(Panel, comPins, height > 32 ? std::uint8_t{0x12} : std::uint8_t{0x02}));

    static_assert(height % 8 == 0 && height >= 16 && height <= Ctrl::maxRows,
                  "the height must be a multiple of 8 the controller can drive");
    static_assert(width > 0 && Panel::columnOffset + width <= Ctrl::ramColumns,
                  "the panel does not fit the controller's RAM columns");
    static_assert(Panel::displayOffset < Ctrl::maxRows,
                  "D3h takes 0..MaxRows-1");
    static_assert(address == 0x3C || address == 0x3D,
                  "these controllers answer at 0x3C or 0x3D");
    static_assert(pages <= 32,
                  "one dirty bit per page, and a write group has at most 32 items");

private:
    /// The power-on configuration as a list of commands, each with its arguments, built at
    /// compile time from the controller and the panel. It ends with the display still off:
    /// Power turns it on once a blank frame has been written.
    static constexpr auto commands = [] {
        Ssd1306Detail::Script s{};
        s.push(Cmd::displayOff);
        // divide by 1, oscillator mid-range: the SSD1306's reset value (the SSD1309 resets to
        // 70h, the SH1106 to 50h), and what these modules are run at
        s.push(Cmd::clockDivide, 0x80);
        s.push(Cmd::multiplex, static_cast<std::uint8_t>(height - 1));
        s.push(Cmd::displayOffset, Panel::displayOffset);
        s.push(static_cast<std::uint8_t>(Cmd::startLine | 0x00));
        if constexpr(Ctrl::chargePumpOn != 0) {
            if constexpr(Panel::supply == Ssd1306Detail::Vcc::chargePump) {
                s.push(static_cast<std::uint8_t>(Ctrl::chargePumpOn >> 8U),
                       static_cast<std::uint8_t>(Ctrl::chargePumpOn & 0xFFU));
            }
        }
        if constexpr(Ctrl::hasAddressMode) {
            s.push(Cmd::addressMode, 0x02);   // page addressing, which is all the SH1106 has
        }
        s.push(static_cast<std::uint8_t>(
          Cmd::segmentRemap | (Panel::flipX == Ssd1306Detail::Mirror::mirrored ? 0x01U : 0x00U)));
        s.push(Panel::flipY == Ssd1306Detail::Mirror::mirrored ? Cmd::scanRemap : Cmd::scanNormal);
        s.push(Cmd::comPins, comPins);
        s.push(Cmd::contrast, Panel::contrast);
        s.push(Cmd::precharge,
               Panel::supply == Ssd1306Detail::Vcc::external ? Ctrl::prechargeExternal
                                                             : Ctrl::prechargeInternal);
        s.push(Cmd::vcomDeselect, Ctrl::vcomDeselect);
        s.push(Cmd::allOnResume);
        s.push(Cmd::normal);
        return s;
    }();

    static_assert(commands.n <= Ssd1306Detail::maxUnits,
                  "the bring-up script outgrew Ssd1306Detail::MaxUnits");

    /// How many transactions the commands pack into: a control byte and up to seven command
    /// bytes each (Step::InlineBytes is 8), never splitting a command from its argument.
    static constexpr auto packing = [] {
        struct Chunk {
            std::size_t first{};
            std::size_t last{};   // one past
        };

        struct P {
            std::array<Chunk, Ssd1306Detail::maxUnits> chunks{};
            std::size_t                                n{};
        } p{};

        std::size_t i = 0;
        while(i < commands.n) {
            std::size_t bytes = 0;
            std::size_t j     = i;
            while(j < commands.n && bytes + commands.units[j].n <= Engine::Step::inlineBytes - 1) {
                bytes += commands.units[j].n;
                ++j;
            }
            p.chunks[p.n++] = Chunk{i, j};
            i               = j;
        }
        return p;
    }();

public:
    static constexpr auto init = [] {
        std::array<Engine::Step, packing.n> a{};
        for(std::size_t c = 0; c < packing.n; ++c) {
            Engine::Step st{};
            st.kind        = Engine::Step::Kind::write;
            st.hasRegister = false;
            st.bytes[0]    = Ssd1306Detail::commandStream;
            std::uint8_t k = 1;
            for(std::size_t u = packing.chunks[c].first; u < packing.chunks[c].last; ++u) {
                for(std::uint8_t b = 0; b < commands.units[u].n; ++b) {
                    st.bytes[k++] = commands.units[u].b[b];
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
        using Value = std::array<std::uint8_t, static_cast<std::size_t>(width)>;
        static constexpr std::size_t items = static_cast<std::size_t>(pages);
        static constexpr std::size_t bytes = 4 + 1 + static_cast<std::size_t>(width);
        /// A blank frame at the first bring-up. After a later reset the pages carry what is
        /// on the glass, because Initial is only installed while nothing has been written.
        static constexpr Value initial{};

        [[nodiscard]] static constexpr std::array<Engine::Step,
                                                  2>
        encode(Value const&         value,
               std::size_t          page,
               std::span<std::byte> buffer) {
            auto const col = static_cast<unsigned>(Panel::columnOffset);
            buffer[0]      = std::byte{Ssd1306Detail::commandStream};
            buffer[1]      = static_cast<std::byte>(Cmd::page | static_cast<unsigned>(page));
            buffer[2]      = static_cast<std::byte>(Cmd::columnLow | (col & 0xFU));
            buffer[3]      = static_cast<std::byte>(Cmd::columnHigh | ((col >> 4U) & 0xFU));
            buffer[4]      = std::byte{Ssd1306Detail::dataStream};
            for(std::size_t i = 0; i < static_cast<std::size_t>(width); ++i) {
                buffer[5 + i] = static_cast<std::byte>(value[i]);
            }
            return {Engine::Step::commandBuffer({.offset = 0, .count = 4}),
                    Engine::Step::commandBuffer(
                      {.offset = 4, .count = static_cast<std::uint8_t>(1 + width)})};
        }
    };

    /// The display on or off (AFh / AEh). On after the first blank frame; see the note
    /// above about the order write groups are served in.
    struct Power {
        using Value                          = Ssd1306Detail::Display;
        static constexpr std::size_t bytes   = 2;
        static constexpr Value       initial = Value::on;

        [[nodiscard]] static constexpr Engine::Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
            buffer[0] = std::byte{Ssd1306Detail::commandStream};
            buffer[1] = std::byte{value == Value::on ? Cmd::displayOn : Cmd::displayOff};
            return Engine::Step::commandBuffer({.offset = 0, .count = 2});
        }
    };

    /// 81h: the segment current, 0..255. The bring-up sets the panel's, so this group only
    /// costs traffic when the application changes it.
    struct Contrast {
        using Value                        = std::uint8_t;
        static constexpr std::size_t bytes = 3;

        [[nodiscard]] static constexpr Engine::Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
            buffer[0] = std::byte{Ssd1306Detail::commandStream};
            buffer[1] = std::byte{Cmd::contrast};
            buffer[2] = static_cast<std::byte>(value);
            return Engine::Step::commandBuffer({.offset = 0, .count = 3});
        }
    };

    /// A7h / A6h: show the RAM inverted, without redrawing it.
    struct Invert {
        using Value                        = Ssd1306Detail::Inversion;
        static constexpr std::size_t bytes = 2;

        [[nodiscard]] static constexpr Engine::Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
            buffer[0] = std::byte{Ssd1306Detail::commandStream};
            buffer[1] = std::byte{value == Value::inverted ? Cmd::inverse : Cmd::normal};
            return Engine::Step::commandBuffer({.offset = 0, .count = 2});
        }
    };

    // Page first: at every bring-up the frame goes out before Power turns the panel on.
    using Writes = Engine::List<Page, Power, Contrast, Invert>;

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
