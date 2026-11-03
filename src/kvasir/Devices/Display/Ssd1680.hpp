#pragma once

#include "../Duration.hpp"
#include "../Lines.hpp"
#include "../Log.hpp"
#include "../Quantities.hpp"
#include "../ResetLine.hpp"
#include "Bus.hpp"
#include "Dcs.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Link.hpp"
#include "kvasir/Util/Retry.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <string_view>

/// Solomon Systech SSD1680 black/white e-paper controller on a Display::DataBus (4-wire SPI with
/// D/C: Display::SpiDcsBus) and a BUSY line, as a gfx::MonoPanel: the driver owns the frame as page
/// RAM, a gfx::MonoFrame draws straight into it, and only a frame whose pages actually changed is
/// sent and refreshed.
///
///     using EpdBus = Kvasir::Display::SpiDcsBus<Spi, Clock, HW::Pin::cs, HW::Pin::dc, BusConfig>;
///     using Epd    = Kvasir::Display::Ssd1680<Clock, EpdBus, Kvasir::GpioReset<HW::Pin::rst>,
///                                             Kvasir::GpioBusy<HW::Pin::busy>, EpdConfig>;
///     gfx::MonoFrame<Epd, gfx::MonoCrispTraits> screen{};
///     ...
///     Epd::turn();                                      // once per loop turn
///     screen.frame([&](auto& c) { draw(c); });          // false while a refresh runs
///
/// The canvas is landscape: `Width` is the gate lines (296), `Height` the source lines (128);
/// a portrait canvas would be 37 pages, more than MonoCanvasView's 32-bit dirty mask. The
/// frame is repacked into the controller's layout `chunkRows` gate lines at a time while it
/// is sent. Every command waits for BUSY low, and for the one before it to be off the bus (a
/// command is one transfer: its byte with D/C low, its parameters with D/C high; a RAM chunk one
/// with D/C high). Nothing waits in a loop: turn() polls.
///
/// A failure (BUSY high past `busyTimeout`, a failed transfer) is logged once and counted
/// (`failures()`); `retryAfter` later the panel is reset and brought up again, which resends
/// the page RAM, so the picture that was showing comes back. While it keeps failing the pause
/// doubles up to ten minutes (Kvasir::Retry); a bring-up that completes starts it over.
namespace Kvasir { namespace Display {

    /// SSD1680 datasheet, 10: Command Table.
    struct Ssd1680Cmd {
        static constexpr std::uint8_t driverOutputControl   = 0x01;
        static constexpr std::uint8_t deepSleep             = 0x10;
        static constexpr std::uint8_t dataEntryMode         = 0x11;
        static constexpr std::uint8_t swReset               = 0x12;
        static constexpr std::uint8_t masterActivation      = 0x20;
        static constexpr std::uint8_t displayUpdateControl1 = 0x21;
        static constexpr std::uint8_t displayUpdateControl2 = 0x22;
        static constexpr std::uint8_t writeRamBw            = 0x24;
        static constexpr std::uint8_t writeLut              = 0x32;
        static constexpr std::uint8_t writeDisplayOption    = 0x37;
        static constexpr std::uint8_t borderWaveform        = 0x3C;
        static constexpr std::uint8_t ramXRange             = 0x44;
        static constexpr std::uint8_t ramYRange             = 0x45;
        static constexpr std::uint8_t ramXCounter           = 0x4E;
        static constexpr std::uint8_t ramYCounter           = 0x4F;
    };

    /// Defaults for a panel config; a config derives from this and redeclares what it sets.
    struct Ssd1680Defaults {
        /// Source lines: a multiple of 8, at most 176; gate lines at most 296.
        static constexpr int sourceLines = 128;
        static constexpr int gateLines   = 296;

        /// Rotated by 180 degrees.
        static constexpr bool flip = false;

        /// Partial refresh after one full refresh at bring-up: fast, no flash, ghosting
        /// that clean() clears.
        static constexpr bool partial = false;

        /// Gate lines per SPI transfer (`chunkRows * sourceLines / 8` bytes of RAM).
        static constexpr int chunkRows = 16;

        /// A full refresh takes about three seconds.
        static constexpr std::chrono::milliseconds busyTimeout{10'000};

        /// After a failure the panel is reset and brought up again this much later (doubling
        /// while it keeps failing); above 0. A bring-up is a full refresh, about three seconds
        /// of flashing, and a stuck BUSY already costs busyTimeout per attempt: hence the long
        /// pause.
        static constexpr std::chrono::milliseconds retryAfter{10'000};
    };

    namespace ssd1680_detail {
        /// A canvas byte has its top pixel in bit 0, a source byte its first pixel in bit 7.
        inline constexpr std::array<std::uint8_t, 256> reverse = [] {
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
        inline constexpr std::array<std::uint8_t, 153> partialLut{
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

        /// What made the panel fail.
        enum class Failure : std::uint8_t { busyTimeout, transfer };
    }   // namespace ssd1680_detail

    template<typename Clock,
             DataBus   Bus,
             ResetLine Reset,
             BusyLine  Busy,
             typename UserConfig = Ssd1680Defaults>
    struct Ssd1680 {
        static_assert(std::derived_from<UserConfig,
                                        Ssd1680Defaults>,
                      "derive the panel config from Kvasir::Display::Ssd1680Defaults");
        using Config = UserConfig;

        static constexpr std::chrono::milliseconds retryAfter
          = Kvasir::asDuration(Config::retryAfter);
        static constexpr std::chrono::milliseconds busyTimeout
          = Kvasir::asDuration(Config::busyTimeout);
        static_assert(retryAfter > std::chrono::milliseconds::zero(),
                      "retryAfter is the pause before a failed panel is brought up again");
        static_assert(busyTimeout > std::chrono::milliseconds::zero(),
                      "busyTimeout is how long BUSY may stay high; a full refresh takes about 3 s");

        /// SCL at most 20 MHz writing (SSD1680 12.1 Table 12-1, SSD1680.md l.1744); the driver
        /// never reads. Checked only when the bus says what it runs at (a mock bus does not).
        static constexpr Units::Hertz maxWriteClock = Units::hertz(20'000'000);
        static_assert(Units::hertz(static_cast<unsigned long>(KVASIR_CONFIG_OR(Bus,
                                                                               writeHz,
                                                                               0UL)))
                        <= maxWriteClock,
                      "the bus write clock exceeds the SSD1680's 20 MHz");

        static constexpr std::string_view name = "SSD1680";

        static constexpr int width    = Config::gateLines;
        static constexpr int height   = Config::sourceLines;
        static constexpr int pages    = (height + 7) / 8;
        static constexpr int rowBytes = Config::sourceLines / 8;

        static_assert(Config::sourceLines > 0 && Config::sourceLines <= 176
                        && Config::sourceLines % 8 == 0,
                      "the SSD1680 drives up to 176 source lines, in whole bytes");
        static_assert(Config::gateLines > 0 && Config::gateLines <= 296,
                      "the SSD1680 drives up to 296 gate lines");
        static_assert(pages <= 32,
                      "MonoCanvasView tracks dirty pages in 32 bits");
        static_assert(Config::chunkRows > 0,
                      "at least one gate line per transfer");

        using PageBytes = std::array<std::uint8_t, static_cast<std::size_t>(width)>;
        using PageRam   = std::span<PageBytes, static_cast<std::size_t>(pages)>;

        static constexpr std::chrono::milliseconds resetPulse{10};
        static constexpr std::chrono::milliseconds resetToCommand{10};

        /// BUSY is believed only this long after a transfer: a command that has not raised it
        /// yet must not look finished.
        static constexpr std::chrono::milliseconds busySettle{2};

    private:
        using u8 = std::uint8_t;
        using ms = std::chrono::milliseconds;
        using tp = typename Clock::time_point;

        static constexpr int lastGate = Config::gateLines - 1;

        // Dcs::Script steps; the delay byte is unused, BUSY is waited for.
        static constexpr std::array<u8, 18> commonBytes{
          Ssd1680Cmd::swReset,
          0,
          0,
          Ssd1680Cmd::driverOutputControl,
          3,
          ssd1680_detail::lo(lastGate),
          ssd1680_detail::hi(lastGate),
          0x00,
          0,
          Ssd1680Cmd::dataEntryMode,
          1,
          0x03,   // X increment, Y increment, counter moves in X
          0,
          Ssd1680Cmd::displayUpdateControl1,
          2,
          0x00,
          0x80,
          0};

        static constexpr std::array<u8, 8> fullBytes{Ssd1680Cmd::borderWaveform,
                                                     1,
                                                     0x05,
                                                     0,
                                                     Ssd1680Cmd::displayUpdateControl2,
                                                     1,
                                                     0xF7,
                                                     0};

        static constexpr auto partialBytes
          = ssd1680_detail::join(std::array<u8, 2>{Ssd1680Cmd::writeLut, 153},
                                 ssd1680_detail::partialLut,
                                 std::array<u8, 1>{0},
                                 std::array<u8, 13>{Ssd1680Cmd::writeDisplayOption,
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
                                 std::array<u8, 15>{Ssd1680Cmd::borderWaveform,
                                                    1,
                                                    0x80,
                                                    0,
                                                    Ssd1680Cmd::displayUpdateControl2,
                                                    1,
                                                    0xC0,
                                                    0,
                                                    Ssd1680Cmd::masterActivation,
                                                    0,
                                                    0,
                                                    Ssd1680Cmd::displayUpdateControl2,
                                                    1,
                                                    0x0C,
                                                    0});

        static constexpr std::array<u8, 21> windowBytes{Ssd1680Cmd::ramXRange,
                                                        2,
                                                        0x00,
                                                        static_cast<u8>(rowBytes - 1),
                                                        0,
                                                        Ssd1680Cmd::ramYRange,
                                                        4,
                                                        0x00,
                                                        0x00,
                                                        ssd1680_detail::lo(lastGate),
                                                        ssd1680_detail::hi(lastGate),
                                                        0,
                                                        Ssd1680Cmd::ramXCounter,
                                                        1,
                                                        0x00,
                                                        0,
                                                        Ssd1680Cmd::ramYCounter,
                                                        2,
                                                        0x00,
                                                        0x00,
                                                        0};

        static constexpr std::array<u8, 3> activateBytes{Ssd1680Cmd::masterActivation, 0, 0};

        static_assert(Dcs::Script::wellFormed(commonBytes) && Dcs::Script::wellFormed(fullBytes)
                      && Dcs::Script::wellFormed(partialBytes)
                      && Dcs::Script::wellFormed(windowBytes)
                      && Dcs::Script::wellFormed(activateBytes));

        enum class Kind : u8 {
            script,     ///< send a Dcs::Script, each command after BUSY
            ram,        ///< write RAM: the command, then the frame repacked in chunks
            waitIdle,   ///< BUSY low: the refresh the activation started is over
        };

        struct Item {
            Kind        kind{};
            Dcs::Script script{};
        };

        static constexpr Item common{Kind::script, Dcs::Script{commonBytes}};
        static constexpr Item full{Kind::script, Dcs::Script{fullBytes}};
        static constexpr Item partialInit{Kind::script, Dcs::Script{partialBytes}};
        static constexpr Item window{Kind::script, Dcs::Script{windowBytes}};
        static constexpr Item ram{Kind::ram, {}};
        static constexpr Item activate{Kind::script, Dcs::Script{activateBytes}};
        static constexpr Item waitIdle{Kind::waitIdle, {}};

        static constexpr std::array<Item, 4> refresh{window, ram, activate, waitIdle};

        /// Also clean(): a full refresh, then with partial the partial waveform.
        static constexpr std::array<Item, Config::partial ? 12 : 6> bringUp = [] {
            std::array<Item, Config::partial ? 12 : 6> p{};
            std::size_t                                i = 0;
            for(auto const& it : {common, full, window, ram, activate, waitIdle}) { p[i++] = it; }
            if constexpr(Config::partial) {
                for(auto const& it : {common, partialInit, window, ram, activate, waitIdle}) {
                    p[i++] = it;
                }
            }
            return p;
        }();

        enum class State : u8 { resetAssert, resetRelease, running, ready, failed };

        /// What is on the bus: a command (with its parameters) or a RAM chunk.
        enum class Tx : u8 { idle, command, data };

        static constexpr std::size_t chunkBytes
          = static_cast<std::size_t>(Config::chunkRows) * static_cast<std::size_t>(rowBytes);

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
        static inline tp                                settled_{};
        static inline std::uint32_t                     refreshes_{};
        static inline std::array<std::byte, chunkBytes> chunk_{};
        static inline std::array<PageBytes, std::size_t{pages}>     ram_{};
        static inline std::array<std::uint32_t, std::size_t{pages}> hashes_{};
        static inline Kvasir::Retry<
          Clock,
          ssd1680_detail::Failure,
          Kvasir::RetryFromMs<static_cast<std::uint64_t>(retryAfter.count())>>
                                    retry_{};
        static inline std::uint32_t failures_{};   ///< since power-up, not since the last success
        static inline bool          up_{};         ///< a program completed since the last (re)start

        /// Every way into State::failed: counted and logged once, the retry armed.
        static void fail(ssd1680_detail::Failure why,
                         tp                      now) {
            state_ = State::failed;
            tx_    = Tx::idle;
            up_    = false;
            ++failures_;
            retry_.failed(why, now);
            UC_LOG_W(
              "{}: the panel or its bus failed ({}, failure {}), bringing it up again in {} ms",
              name,
              why == ssd1680_detail::Failure::busyTimeout ? std::string_view{"BUSY stuck high"}
                                                          : std::string_view{"a transfer failed"},
              failures_,
              static_cast<long long>(std::chrono::duration_cast<ms>(retry_.next - now).count()));
        }

        /// Stands in for a copy of what was last sent.
        [[nodiscard]] static std::uint32_t hash(PageBytes const& page) {
            std::uint32_t h = 2166136261U;
            for(auto const b : page) {
                h ^= b;
                h *= 16777619U;
            }
            return h;
        }

        [[nodiscard]] static bool differs(std::uint32_t pagesArg) {
            for(std::size_t p = 0; p < std::size_t{pages}; ++p) {
                if(((pagesArg >> p) & 1U) != 0 && hash(ram_[p]) != hashes_[p]) { return true; }
            }
            return false;
        }

        static void start(std::span<Item const> program) {
            for(std::size_t p = 0; p < std::size_t{pages}; ++p) { hashes_[p] = hash(ram_[p]); }
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
                up_    = true;
                ++refreshes_;
                if(retry_.pending) { retry_.succeeded(); }
            }
        }

        /// A refusal is a failed transfer: the bus has no room, which nothing here causes.
        static void command(u8                  cmd,
                            std::span<u8 const> params,
                            tp                  now) {
            if(!Bus::writeCommand(cmd, std::as_bytes(params))) {
                fail(ssd1680_detail::Failure::transfer, now);
                return;
            }
            tx_ = Tx::command;
        }

        [[nodiscard]] static bool controllerIdle(tp now) {
            if(now < settled_) { return false; }
            if(Busy::busy()) {
                if(now > settled_ + busyTimeout) {
                    fail(ssd1680_detail::Failure::busyTimeout, now);
                }
                return false;
            }
            return true;
        }

        /// Gate lines [first, first + rows): source bytes MSB first, 1 for white.
        static void repack(int first,
                           int rows) {
            std::size_t i = 0;
            for(int y = first; y < first + rows; ++y) {
                if constexpr(Config::flip) {
                    // Page and bit order both run backwards: each byte stays as it is.
                    auto const x = static_cast<std::size_t>(y);
                    for(std::size_t k = 0; k < std::size_t{rowBytes}; ++k) {
                        auto const v = ram_[std::size_t{pages} - 1 - k][x];
                        chunk_[i++]  = static_cast<std::byte>(static_cast<u8>(~v));
                    }
                } else {
                    auto const x = static_cast<std::size_t>(width - 1 - y);
                    for(std::size_t k = 0; k < std::size_t{rowBytes}; ++k) {
                        auto const v = ssd1680_detail::reverse[ram_[k][x]];
                        chunk_[i++]  = static_cast<std::byte>(static_cast<u8>(~v));
                    }
                }
            }
        }

        static void run(tp now) {
            if(Bus::failed()) {
                fail(ssd1680_detail::Failure::transfer, now);
                return;
            }
            if(tx_ != Tx::idle) {
                if(!Bus::idle()) { return; }
                settled_ = now + busySettle;
                tx_      = Tx::idle;
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
                    command(step.cmd, step.params, now);
                }
                break;

            case Kind::ram:
                if(!ramStarted_) {
                    if(!controllerIdle(now)) { return; }
                    ramStarted_ = true;
                    command(Ssd1680Cmd::writeRamBw, {}, now);
                    return;
                }
                if(pos_ >= std::size_t{width}) {
                    finishItem();
                    return;
                }
                {
                    auto const rows = std::min(Config::chunkRows, width - static_cast<int>(pos_));
                    repack(static_cast<int>(pos_), rows);
                    pos_ += static_cast<std::size_t>(rows);
                    if(!Bus::writeData(std::span<std::byte const>{chunk_}.first(
                         static_cast<std::size_t>(rows) * std::size_t{rowBytes})))
                    {
                        fail(ssd1680_detail::Failure::transfer, now);
                        return;
                    }
                    tx_ = Tx::data;
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
            up_             = false;
            cleanRequested_ = false;
            drawnWhileBusy_ = false;
            Bus::clearError();
        }

        /// A frame may be drawn.
        [[nodiscard]] static bool ready() { return state_ == State::ready && !cleanRequested_; }

        /// The transfer/contract state: BUSY timed out or the bus failed, and nothing is sent until
        /// the panel is brought up again, retryAfter later (doubling while it keeps failing). Not
        /// the health: that is link().
        [[nodiscard]] static bool failed() { return state_ == State::failed; }

        /// The health (SDK Runtime/README.md "Observability"; Health::LinkCheck watches it):
        /// answering from the first completed bring-up on (a refresh keeps it so), absent while failed, starting
        /// until then. Back to answering by itself: the retry (retryAfter, doubling) brings the panel up again.
        [[nodiscard]] static Link link() {
            if(state_ == State::failed) { return Link::absent; }
            return up_ ? Link::answering : Link::starting;
        }

        [[nodiscard]] static std::uint32_t refreshes() { return refreshes_; }

        /// Failures since power-up; restart() does not clear it.
        [[nodiscard]] static std::uint32_t failures() { return failures_; }

        [[nodiscard]] static PageRam pageRam() { return PageRam{ram_}; }

        /// The pages drawn into: refreshed only if one differs from what was last sent.
        static void pagesChanged(std::uint32_t pagesArg) {
            if(state_ == State::ready) {
                if(differs(pagesArg)) { start(refresh); }
            } else if(pagesArg != 0) {
                drawnWhileBusy_ = true;
            }
        }

        /// A full refresh at the next opportunity: clears partial-refresh ghosting.
        static void clean() { cleanRequested_ = true; }

        /// From the main loop; runs the bus's turn() too.
        static void turn() {
            Bus::turn();
            auto const now = Clock::now();
            switch(state_) {
            case State::resetAssert:
                reset_.begin(now, resetPulse, resetToCommand);
                state_ = State::resetRelease;
                break;

            case State::resetRelease:
                if(reset_.done(now)) {
                    settled_ = now;
                    start(bringUp);
                }
                break;

            case State::running: run(now); break;

            case State::ready:
                if(cleanRequested_) {
                    cleanRequested_ = false;
                    drawnWhileBusy_ = false;
                    start(bringUp);
                } else if(drawnWhileBusy_) {
                    drawnWhileBusy_ = false;
                    if(differs(0xFFFF'FFFFU)) { start(refresh); }
                }
                break;

            case State::failed:
                if(retry_.due(now)) { restart(); }
                break;
            }
        }
    };

}}   // namespace Kvasir::Display
