#pragma once

#include "../Lines.hpp"
#include "../ResetLine.hpp"
#include "Dcs.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <string_view>

/// Solomon Systech SSD1680 black/white e-paper controller over SPI with D/C and BUSY lines,
/// as a gfx::MonoPanel: the driver owns the frame as page RAM, a gfx::MonoFrame draws
/// straight into it, and only a frame whose pages actually changed is sent and refreshed.
///
///     using Epd = Kvasir::Display::Ssd1680<Clock, Spi, Kvasir::GpioReset<HW::Pin::rst>,
///                                          Kvasir::GpioDataCommand<HW::Pin::dc>,
///                                          Kvasir::GpioBusy<HW::Pin::busy>, EpdConfig>;
///     gfx::MonoFrame<Epd, gfx::MonoCrispTraits> screen{};
///     ...
///     Epd::handler();                                   // once per loop turn
///     screen.frame([&](auto& c) { draw(c); });          // false while a refresh runs
///
/// The canvas is landscape: `Width` is the gate lines (296), `Height` the source lines (128);
/// a portrait canvas would be 37 pages, more than MonoCanvasView's 32-bit dirty mask. The
/// frame is repacked into the controller's layout `ChunkRows` gate lines at a time while it
/// is sent. Every command waits for BUSY low.
namespace Kvasir { namespace Display {

    /// An asynchronous, zero-copy send and its state.
    template<typename S>
    concept SpiBus = requires(std::span<std::byte const> const& bytes) {
        { S::send_nocopy(bytes) };
        { S::operationState() == S::OperationState::succeeded } -> std::convertible_to<bool>;
    };

    /// SSD1680 datasheet, 10: Command Table.
    struct Ssd1680Cmd {
        static constexpr std::uint8_t DriverOutputControl   = 0x01;
        static constexpr std::uint8_t DeepSleep             = 0x10;
        static constexpr std::uint8_t DataEntryMode         = 0x11;
        static constexpr std::uint8_t SwReset               = 0x12;
        static constexpr std::uint8_t MasterActivation      = 0x20;
        static constexpr std::uint8_t DisplayUpdateControl1 = 0x21;
        static constexpr std::uint8_t DisplayUpdateControl2 = 0x22;
        static constexpr std::uint8_t WriteRamBw            = 0x24;
        static constexpr std::uint8_t WriteLut              = 0x32;
        static constexpr std::uint8_t WriteDisplayOption    = 0x37;
        static constexpr std::uint8_t BorderWaveform        = 0x3C;
        static constexpr std::uint8_t RamXRange             = 0x44;
        static constexpr std::uint8_t RamYRange             = 0x45;
        static constexpr std::uint8_t RamXCounter           = 0x4E;
        static constexpr std::uint8_t RamYCounter           = 0x4F;
    };

    /// Defaults for a panel config; a config derives from this and redeclares what it sets.
    struct Ssd1680Defaults {
        /// Source lines: a multiple of 8, at most 176; gate lines at most 296.
        static constexpr int SourceLines = 128;
        static constexpr int GateLines   = 296;

        /// Rotated by 180 degrees.
        static constexpr bool Flip = false;

        /// Partial refresh after one full refresh at bring-up: fast, no flash, ghosting
        /// that clean() clears.
        static constexpr bool Partial = false;

        /// Gate lines per SPI transfer (`ChunkRows * SourceLines / 8` bytes of RAM).
        static constexpr int ChunkRows = 16;

        /// A full refresh takes about three seconds.
        static constexpr unsigned BusyTimeoutMs = 10'000;
    };

    namespace ssd1680_detail {
        /// A canvas byte has its top pixel in bit 0, a source byte its first pixel in bit 7.
        inline constexpr std::array<std::uint8_t, 256> Reverse = [] {
            std::array<std::uint8_t, 256> r{};
            for(unsigned v = 0; v < 256U; ++v) {
                unsigned out = 0;
                for(unsigned b = 0; b < 8U; ++b) {
                    if(((v >> b) & 1U) != 0) { out |= 1U << (7U - b); }
                }
                r[v] = static_cast<std::uint8_t>(out);
            }
            return r;
        }();

        /// Five LUTs of twelve groups, the group timings, frame rates, XON.
        inline constexpr std::array<std::uint8_t, 153> PartialLut{
          // clang-format off
            // LUT 0..4, VS phase A B C D per group
            0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            // TP[A] TP[B] SR[AB] TP[C] TP[D] SR[CD] RP, groups 0..11
            0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
            0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            // FR, groups 0..11
            0x44, 0x44, 0x22, 0x22, 0x22, 0x22,
            // XON
            0x00, 0x00, 0x00,
          // clang-format on
        };

        /// Byte streams laid end to end, for a Dcs::Script assembled from parts.
        template<std::size_t... Ns>
        constexpr std::array<std::uint8_t,
                             (Ns + ... + 0)>
        join(std::array<std::uint8_t,
                        Ns> const&... parts) {
            std::array<std::uint8_t, (Ns + ... + 0)> out{};
            std::size_t                              i = 0;
            for(auto const part : {std::span<std::uint8_t const>{parts}...}) {
                for(auto const b : part) { out[i++] = b; }
            }
            return out;
        }

        [[nodiscard]] constexpr std::uint8_t lo(int v) {
            return static_cast<std::uint8_t>(static_cast<unsigned>(v) & 0xFFU);
        }

        [[nodiscard]] constexpr std::uint8_t hi(int v) {
            return static_cast<std::uint8_t>((static_cast<unsigned>(v) >> 8U) & 0xFFU);
        }
    }   // namespace ssd1680_detail

    template<typename Clock,
             SpiBus          Spi,
             ResetLine       Reset,
             DataCommandLine Dc,
             BusyLine        Busy,
             typename Config_ = Ssd1680Defaults>
    struct Ssd1680 {
        static_assert(std::derived_from<Config_,
                                        Ssd1680Defaults>,
                      "derive the panel config from Kvasir::Display::Ssd1680Defaults");
        using Config = Config_;

        static constexpr std::string_view Name = "SSD1680";

        static constexpr int Width    = Config::GateLines;
        static constexpr int Height   = Config::SourceLines;
        static constexpr int Pages    = (Height + 7) / 8;
        static constexpr int RowBytes = Config::SourceLines / 8;

        static_assert(Config::SourceLines > 0 && Config::SourceLines <= 176
                        && Config::SourceLines % 8 == 0,
                      "the SSD1680 drives up to 176 source lines, in whole bytes");
        static_assert(Config::GateLines > 0 && Config::GateLines <= 296,
                      "the SSD1680 drives up to 296 gate lines");
        static_assert(Pages <= 32,
                      "MonoCanvasView tracks dirty pages in 32 bits");
        static_assert(Config::ChunkRows > 0,
                      "at least one gate line per transfer");

        using PageBytes = std::array<std::uint8_t, static_cast<std::size_t>(Width)>;
        using PageRam   = std::span<PageBytes, static_cast<std::size_t>(Pages)>;

        static constexpr unsigned ResetPulseMs     = 10;
        static constexpr unsigned ResetToCommandMs = 10;

        /// BUSY is believed only this long after a transfer: a command that has not raised it
        /// yet must not look finished.
        static constexpr unsigned BusySettleMs = 2;

    private:
        using u8 = std::uint8_t;
        using ms = std::chrono::milliseconds;
        using tp = typename Clock::time_point;

        static constexpr int LastGate = Config::GateLines - 1;

        // Dcs::Script steps; the delay byte is unused, BUSY is waited for.
        static constexpr std::array<u8, 18> CommonBytes{
          Ssd1680Cmd::SwReset,
          0,
          0,
          Ssd1680Cmd::DriverOutputControl,
          3,
          ssd1680_detail::lo(LastGate),
          ssd1680_detail::hi(LastGate),
          0x00,
          0,
          Ssd1680Cmd::DataEntryMode,
          1,
          0x03,   // X increment, Y increment, counter moves in X
          0,
          Ssd1680Cmd::DisplayUpdateControl1,
          2,
          0x00,
          0x80,
          0};

        static constexpr std::array<u8, 8> FullBytes{Ssd1680Cmd::BorderWaveform,
                                                     1,
                                                     0x05,
                                                     0,
                                                     Ssd1680Cmd::DisplayUpdateControl2,
                                                     1,
                                                     0xF7,
                                                     0};

        static constexpr auto PartialBytes
          = ssd1680_detail::join(std::array<u8, 2>{Ssd1680Cmd::WriteLut, 153},
                                 ssd1680_detail::PartialLut,
                                 std::array<u8, 1>{0},
                                 std::array<u8, 13>{Ssd1680Cmd::WriteDisplayOption,
                                                    10,
                                                    0x00,
                                                    0x00,
                                                    0x00,
                                                    0x00,
                                                    0x00,
                                                    0x40,
                                                    0x00,
                                                    0x00,
                                                    0x00,
                                                    0x00,
                                                    0},
                                 std::array<u8, 15>{Ssd1680Cmd::BorderWaveform,
                                                    1,
                                                    0x80,
                                                    0,
                                                    Ssd1680Cmd::DisplayUpdateControl2,
                                                    1,
                                                    0xC0,
                                                    0,
                                                    Ssd1680Cmd::MasterActivation,
                                                    0,
                                                    0,
                                                    Ssd1680Cmd::DisplayUpdateControl2,
                                                    1,
                                                    0x0C,
                                                    0});

        static constexpr std::array<u8, 21> WindowBytes{Ssd1680Cmd::RamXRange,
                                                        2,
                                                        0x00,
                                                        static_cast<u8>(RowBytes - 1),
                                                        0,
                                                        Ssd1680Cmd::RamYRange,
                                                        4,
                                                        0x00,
                                                        0x00,
                                                        ssd1680_detail::lo(LastGate),
                                                        ssd1680_detail::hi(LastGate),
                                                        0,
                                                        Ssd1680Cmd::RamXCounter,
                                                        1,
                                                        0x00,
                                                        0,
                                                        Ssd1680Cmd::RamYCounter,
                                                        2,
                                                        0x00,
                                                        0x00,
                                                        0};

        static constexpr std::array<u8, 3> ActivateBytes{Ssd1680Cmd::MasterActivation, 0, 0};

        static_assert(Dcs::Script::wellFormed(CommonBytes) && Dcs::Script::wellFormed(FullBytes)
                      && Dcs::Script::wellFormed(PartialBytes)
                      && Dcs::Script::wellFormed(WindowBytes)
                      && Dcs::Script::wellFormed(ActivateBytes));

        enum class Kind : u8 {
            script,     ///< send a Dcs::Script, each command after BUSY
            ram,        ///< write RAM: the command, then the frame repacked in chunks
            waitIdle,   ///< BUSY low: the refresh the activation started is over
        };

        struct Item {
            Kind        kind{};
            Dcs::Script script{};
        };

        static constexpr Item Common{Kind::script, Dcs::Script{CommonBytes}};
        static constexpr Item Full{Kind::script, Dcs::Script{FullBytes}};
        static constexpr Item PartialInit{Kind::script, Dcs::Script{PartialBytes}};
        static constexpr Item Window{Kind::script, Dcs::Script{WindowBytes}};
        static constexpr Item Ram{Kind::ram, {}};
        static constexpr Item Activate{Kind::script, Dcs::Script{ActivateBytes}};
        static constexpr Item WaitIdle{Kind::waitIdle, {}};

        static constexpr std::array<Item, 4> Refresh{Window, Ram, Activate, WaitIdle};

        /// Also clean(): a full refresh, then with Partial the partial waveform.
        static constexpr std::array<Item, Config::Partial ? 12 : 6> BringUp = [] {
            std::array<Item, Config::Partial ? 12 : 6> p{};
            std::size_t                                i = 0;
            for(auto const& it : {Common, Full, Window, Ram, Activate, WaitIdle}) { p[i++] = it; }
            if constexpr(Config::Partial) {
                for(auto const& it : {Common, PartialInit, Window, Ram, Activate, WaitIdle}) {
                    p[i++] = it;
                }
            }
            return p;
        }();

        enum class State : u8 { resetAssert, resetRelease, running, ready, failed };

        enum class Tx : u8 { idle, command, params, data };

        static constexpr std::size_t ChunkBytes
          = static_cast<std::size_t>(Config::ChunkRows) * static_cast<std::size_t>(RowBytes);

        // Zero-initialised (nothing copied from flash); a zero frame is white.
        static inline State                             state_{State::resetAssert};
        static inline Tx                                tx_{Tx::idle};
        static inline Kvasir::ResetPulse<Reset, Clock>  reset_{};
        static inline std::span<Item const>             program_{};
        static inline std::size_t                       item_{};
        static inline std::size_t                       pos_{};   ///< script offset, or gate line
        static inline bool                              ramStarted_{};
        static inline bool                              cleanRequested_{};
        static inline bool                              drawnWhileBusy_{};
        static inline std::span<u8 const>               params_{};
        static inline tp                                settled_{};
        static inline std::uint32_t                     refreshes_{};
        static inline std::array<std::byte, 1>          command_{};
        static inline std::array<std::byte, ChunkBytes> chunk_{};
        static inline std::array<PageBytes, std::size_t(Pages)>     ram_{};
        static inline std::array<std::uint32_t, std::size_t(Pages)> hashes_{};

        /// Stands in for a copy of what was last sent.
        [[nodiscard]] static std::uint32_t hash(PageBytes const& page) {
            std::uint32_t h = 2166136261U;
            for(auto const b : page) {
                h ^= b;
                h *= 16777619U;
            }
            return h;
        }

        [[nodiscard]] static bool differs(std::uint32_t pages) {
            for(std::size_t p = 0; p < std::size_t(Pages); ++p) {
                if(((pages >> p) & 1U) != 0 && hash(ram_[p]) != hashes_[p]) { return true; }
            }
            return false;
        }

        static void start(std::span<Item const> program) {
            for(std::size_t p = 0; p < std::size_t(Pages); ++p) { hashes_[p] = hash(ram_[p]); }
            program_    = program;
            item_       = 0;
            pos_        = 0;
            ramStarted_ = false;
            state_      = State::running;
        }

        static void finishItem() {
            ++item_;
            pos_        = 0;
            ramStarted_ = false;
            if(item_ >= program_.size()) {
                state_ = State::ready;
                ++refreshes_;
            }
        }

        static void send(std::span<std::byte const> const& bytes) { Spi::send_nocopy(bytes); }

        static void command(u8                  cmd,
                            std::span<u8 const> params) {
            Dc::command();
            command_[0] = std::byte{cmd};
            params_     = params;
            tx_         = Tx::command;
            send(std::span<std::byte const>{command_});
        }

        [[nodiscard]] static bool controllerIdle(tp now) {
            if(now < settled_) { return false; }
            if(Busy::busy()) {
                if(now > settled_ + ms{Config::BusyTimeoutMs}) { state_ = State::failed; }
                return false;
            }
            return true;
        }

        /// Gate lines [first, first + rows): source bytes MSB first, 1 for white.
        static void repack(int first,
                           int rows) {
            std::size_t i = 0;
            for(int y = first; y < first + rows; ++y) {
                if constexpr(Config::Flip) {
                    // Page and bit order both run backwards: each byte stays as it is.
                    auto const x = static_cast<std::size_t>(y);
                    for(std::size_t k = 0; k < std::size_t(RowBytes); ++k) {
                        auto const v = ram_[std::size_t(Pages) - 1 - k][x];
                        chunk_[i++]  = static_cast<std::byte>(static_cast<u8>(~v));
                    }
                } else {
                    auto const x = static_cast<std::size_t>(Width - 1 - y);
                    for(std::size_t k = 0; k < std::size_t(RowBytes); ++k) {
                        auto const v = ssd1680_detail::Reverse[ram_[k][x]];
                        chunk_[i++]  = static_cast<std::byte>(static_cast<u8>(~v));
                    }
                }
            }
        }

        static void run(tp now) {
            if(tx_ != Tx::idle) {
                auto const st = Spi::operationState();
                if constexpr(requires { Spi::OperationState::failed; }) {
                    if(st == Spi::OperationState::failed) {
                        tx_    = Tx::idle;
                        state_ = State::failed;
                        return;
                    }
                }
                if(!(st == Spi::OperationState::succeeded)) { return; }
                settled_ = now + ms{BusySettleMs};
                if(tx_ == Tx::command) {
                    Dc::data();
                    if(!params_.empty()) {
                        tx_ = Tx::params;
                        send(std::as_bytes(params_));
                        return;
                    }
                }
                tx_ = Tx::idle;
            }

            auto const& item = program_[item_];
            switch(item.kind) {
            case Kind::script:
                {
                    auto      pos = pos_;
                    Dcs::Step step{};
                    if(!item.script.next(pos, step)) {
                        finishItem();
                        return;
                    }
                    if(!controllerIdle(now)) { return; }
                    pos_ = pos;
                    command(step.cmd, step.params);
                }
                break;

            case Kind::ram:
                if(!ramStarted_) {
                    if(!controllerIdle(now)) { return; }
                    ramStarted_ = true;
                    command(Ssd1680Cmd::WriteRamBw, {});
                    return;
                }
                if(pos_ >= std::size_t(Width)) {
                    finishItem();
                    return;
                }
                {
                    auto const rows = std::min(Config::ChunkRows, Width - static_cast<int>(pos_));
                    repack(static_cast<int>(pos_), rows);
                    pos_ += static_cast<std::size_t>(rows);
                    tx_ = Tx::data;
                    send(std::span<std::byte const>{chunk_}.first(static_cast<std::size_t>(rows)
                                                                  * std::size_t(RowBytes)));
                }
                break;

            case Kind::waitIdle:
                if(controllerIdle(now)) { finishItem(); }
                break;
            }
        }

    public:
        static void restart() {
            state_          = State::resetAssert;
            tx_             = Tx::idle;
            cleanRequested_ = false;
            drawnWhileBusy_ = false;
            if constexpr(requires { Spi::restart(); }) { Spi::restart(); }
        }

        /// A frame may be drawn.
        [[nodiscard]] static bool ready() { return state_ == State::ready && !cleanRequested_; }

        /// BUSY timed out or the master failed; nothing is sent until restart().
        [[nodiscard]] static bool failed() { return state_ == State::failed; }

        [[nodiscard]] static std::uint32_t refreshes() { return refreshes_; }

        [[nodiscard]] static PageRam pageRam() { return PageRam{ram_}; }

        /// The pages drawn into: refreshed only if one differs from what was last sent.
        static void pagesChanged(std::uint32_t pages) {
            if(state_ == State::ready) {
                if(differs(pages)) { start(Refresh); }
            } else if(pages != 0) {
                drawnWhileBusy_ = true;
            }
        }

        /// A full refresh at the next opportunity: clears partial-refresh ghosting.
        static void clean() { cleanRequested_ = true; }

        /// From the main loop.
        static void handler() {
            auto const now = Clock::now();
            switch(state_) {
            case State::resetAssert:
                reset_.begin(now, ms{ResetPulseMs}, ms{ResetToCommandMs});
                state_ = State::resetRelease;
                break;

            case State::resetRelease:
                if(reset_.done(now)) {
                    settled_ = now;
                    start(BringUp);
                }
                break;

            case State::running: run(now); break;

            case State::ready:
                if(cleanRequested_) {
                    cleanRequested_ = false;
                    drawnWhileBusy_ = false;
                    start(BringUp);
                } else if(drawnWhileBusy_) {
                    drawnWhileBusy_ = false;
                    if(differs(0xFFFF'FFFFU)) { start(Refresh); }
                }
                break;

            case State::failed: break;
            }
        }
    };

}}   // namespace Kvasir::Display
