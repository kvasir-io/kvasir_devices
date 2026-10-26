#pragma once
// A display controller on 4-wire SPI with a D/C line (ST7789, ILI9341, the SSD1680 e-paper, ...) as
// a Display::DataBus, over a queued SPI master. Nothing here waits but a register read:
//
// - A command is one request, queued: its byte with D/C low, the parameters with D/C high
//   (flipped in `between`). Up to `commandSlots` are in the master's queue at once, in order; up to
//   `copiedParams` parameter bytes are copied into the command's slot, a longer block is sent from
//   the caller's memory (a script's constants). A command that has not completed `commandTimeout`
//   after the bus last made progress latches failed().
// - Bare data (writeData) is one request with D/C high under its own chip select, from the
//   caller's memory: the SSD1680's frame, chunk by chunk.
// - Pixels are asynchronous, by DMA from the caller's memory; a stream longer than the master
//   takes goes out in chunks under one chip select (`hold`), each submitted from the previous one's
//   completion callback: the ST7789V must never see CSX rise inside a RAMWR (CSX high initialises
//   its serial interface, ST7789V.md:2147). On the SAM completions are polled, so only a loop pause
//   longer than the master's HoldTimeout cuts a fill. Commands are refused while a stream runs (they
//   would land inside its hold).
// - A register read is synchronous: it waits, up to `commandTimeout`, behind what is queued.
#include "../Duration.hpp"
#include "../Quantities.hpp"
#include "../SPI/Client.hpp"
#include "../SPI/QueueCore.hpp"
#include "Bus.hpp"
#include "Dcs.hpp"
#include "kvasir/Hal/Spi.hpp"
#include "kvasir/Util/Config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Display {

    namespace detail {
        /// Named so that a failed check shows both numbers: the commands the bus queues at once
        /// (`Slots`) against what the SPI master's queue holds (`QueueDepth`).
        template<std::size_t Slots, std::size_t QueueDepth>
        inline constexpr bool masterQueueHoldsCommandSlots = Slots <= QueueDepth;
    }   // namespace detail

    struct SpiDcsBusDefaults {
        static constexpr Hal::ClockMode            mode       = Hal::ClockMode::mode0;
        static constexpr Units::Hertz              writeClock = Units::hertz(15'000'000);
        static constexpr Units::Hertz              readClock  = Units::hertz(6'000'000);
        static constexpr std::chrono::milliseconds commandTimeout{50};
        /// Commands in the master's queue at once: DcsPanel's longest burst is its format block
        /// (COLMOD, MADCTL, WRCTRLD, WRDISBV, INVON) and SLPOUT (DcsPanel's FormatBurst checks it).
        /// At most the master's QueueDepth (checked). The SSD1680 has one command out at a time: 1.
        static constexpr std::size_t commandSlots = 6;
        /// Parameter bytes a queued command keeps a copy of: DcsPanel builds CASET/RASET's four
        /// on its stack. A longer block is sent from the caller's memory.
        static constexpr std::size_t copiedParams = 4;
    };

    template<typename Master,
             typename Clock,
             typename Cs,
             typename Dc,
             typename Config = SpiDcsBusDefaults>
    struct SpiDcsBus {
        static constexpr auto WriteSetup = Master::setup(Config::mode, Config::writeClock);
        static constexpr auto ReadSetup  = Master::setup(Config::mode, Config::readClock);
        /// Checked by the panel drivers against the chip's ceilings.
        static constexpr unsigned long writeHz = WriteSetup.hz;
        static constexpr unsigned long readHz  = ReadSetup.hz;

        /// What the master's queue core (SPI/QueueCore.hpp) carries: every queued master has one.
        static constexpr bool Wide = Master::Core::SupportsWide;
        /// The SAM DMAC counts 16 bits.
        static constexpr std::size_t MaxChunk
          = std::min<std::size_t>(Master::Core::MaxFrames, std::size_t{1} << 24U);

        static constexpr std::size_t CommandSlots
          = KVASIR_CONFIG_OR(Config, commandSlots, SpiDcsBusDefaults::commandSlots);
        static constexpr std::size_t CopiedParams
          = KVASIR_CONFIG_OR(Config, copiedParams, SpiDcsBusDefaults::copiedParams);
        static constexpr std::chrono::milliseconds CommandTimeout = Kvasir::asDuration(
          KVASIR_CONFIG_OR(Config, commandTimeout, SpiDcsBusDefaults::commandTimeout));

        static_assert(
          CommandSlots > 0 && CommandSlots < 128,
          "commandSlots: at least one command in the queue, and a count of them in a byte");
        static_assert(detail::masterQueueHoldsCommandSlots<CommandSlots,
                                                           Master::Core::QueueDepth>,
                      "commandSlots is more than the SPI master's queue holds (its QueueDepth): a "
                      "command past it would be refused - lower commandSlots or deepen the master");

        /// Queued behind what is queued; false = refused (a stream runs, every slot is taken, or
        /// the master's queue is full).
        static bool writeCommand(std::uint8_t               cmd,
                                 std::span<std::byte const> params) {
            if(busy_ || outstanding_() >= CommandSlots) { return false; }
            auto& slot = slots_[head_];
            slot[0]    = std::byte{cmd};
            auto tx    = params;
            if(params.size() <= CopiedParams) {
                std::copy(params.begin(), params.end(), slot.begin() + 1);
                tx = std::span<std::byte const>{slot}.subspan(1, params.size());
            }
            return queue_(
              typename Master::Request{.setup   = WriteSetup,
                                       .lines   = commandLines,
                                       .command = std::span<std::byte const>{slot}.first(1),
                                       .tx      = tx});
        }

        /// D/C high under its own chip select, from `bytes`, which stays alive until idle().
        static bool writeData(std::span<std::byte const> bytes) {
            if(busy_ || bytes.empty() || bytes.size() > MaxChunk || outstanding_() >= CommandSlots)
            {
                return false;
            }
            return queue_(
              typename Master::Request{.setup = WriteSetup, .lines = dataLines, .tx = bytes});
        }

        static void writePixels(std::span<std::byte const> bytes) {
            if(busy_) {
                failed_ = true;
                return;
            }
            pixels_  = bytes;
            fill_    = 0;
            started_ = false;
            done_    = false;
            busy_    = true;
            next_();
        }

        /// 16-bit repeat frames where the master has them, a buffer of the pixel otherwise.
        static void fillPixels(std::uint8_t hi,
                               std::uint8_t lo,
                               std::size_t  count) {
            if(busy_) {
                failed_ = true;
                return;
            }
            pattern_ = static_cast<std::uint16_t>((hi << 8U) | lo);
            for(std::size_t i = 0; i < FillBuffer; ++i) {
                fillBuf_[2 * i]     = std::byte{hi};
                fillBuf_[2 * i + 1] = std::byte{lo};
            }
            pixels_  = {};
            fill_    = count;
            started_ = false;
            done_    = false;
            busy_    = true;
            next_();
        }

        /// Only where the module routes MISO; `instr` is unused on 4-wire SPI.
        static bool readRegister(std::uint8_t,
                                 std::uint8_t         cmd,
                                 std::size_t          dummy,
                                 std::span<std::byte> out) {
            if(busy_ || dummy + out.size() > readBuf_.size()) { return false; }
            readCommand_[0] = std::byte{cmd};
            typename Master::Request r{.setup   = ReadSetup,
                                       .lines   = commandLines,
                                       .command = std::span<std::byte const>{readCommand_},
                                       .rx      = std::span{readBuf_}.first(dummy + out.size())};
            if(!read_(r)) { return false; }
            std::copy_n(readBuf_.begin() + static_cast<std::ptrdiff_t>(dummy),
                        out.size(),
                        out.begin());
            return true;
        }

        [[nodiscard]] static bool idle() { return !busy_ && outstanding_() == 0; }

        /// The transfer state, latched: a command, data block or pixel stream failed or made no
        /// progress for commandTimeout. The bus has no link() of its own: the panel on it turns
        /// this into its failed() and its health (DcsPanel / Ssd1680 link()).
        [[nodiscard]] static bool failed() { return failed_; }

        /// A command queued before this that fails later is no failure of what follows.
        static void clearError() {
            failed_     = false;
            generation_ = static_cast<std::uint8_t>(generation_ + 1U);
        }

        /// Where the loop learns a pixel stream's outcome and a command's timeout.
        static void turn() {
            Master::turn();
            if(busy_ && done_) {
                done_ = false;
                if(!ok_) { failed_ = true; }
                finish_();
            }
            if(outstanding_() != 0) {
                auto const now  = Clock::now();
                auto const done = completed_;
                if(done != seen_) {
                    seen_     = done;
                    progress_ = now;
                } else if(now - progress_ > CommandTimeout) {
                    failed_ = true;
                }
            }
        }

    private:
        /// For a master without 16-bit repeat frames.
        static constexpr std::size_t FillBuffer = 128;

        using Select = SPI::ChipSelect<Cs>;

        static void commandLine_() { apply(clear(Dc{})); }

        static void data_() { apply(set(Dc{})); }

        // One `select` for all: a pixel stream's chunks are one hold.
        static constexpr SPI::Lines commandLines{Select::select,
                                                 Select::deselect,
                                                 &commandLine_,
                                                 &data_};
        static constexpr SPI::Lines dataLines{Select::select, Select::deselect, &data_, nullptr};

        using Slot = std::array<std::byte, 1 + CopiedParams>;

        inline static std::array<Slot, CommandSlots>        slots_{};
        inline static std::array<std::byte, 1>              command_{};   ///< a stream's RAMWR
        inline static std::array<std::byte, 1>              readCommand_{};
        inline static std::array<std::byte, 16>             readBuf_{};
        inline static std::array<std::byte, 2 * FillBuffer> fillBuf_{};
        inline static std::uint16_t                         pattern_{};
        // Written by whichever context runs next_() (loop or completion callback, never both),
        // read by the loop only after done_.
        inline static std::span<std::byte const> pixels_{};
        inline static std::size_t                fill_{};
        inline static bool                       started_{};
        // The loop only: the next slot, the requests queued, the completions last seen and when.
        inline static std::uint8_t               head_{};
        inline static std::uint8_t               issued_{};
        inline static std::uint8_t               seen_{};
        inline static typename Clock::time_point progress_{};
        // Crossing from the callback to the loop.
        inline static std::uint8_t volatile completed_{};
        /// Written by the loop only, read by the callbacks.
        inline static std::uint8_t volatile generation_{};
        inline static bool volatile busy_{};
        inline static bool volatile done_{};
        inline static bool volatile ok_{};
        inline static bool volatile failed_{};

        /// The master completes in submission order, so the oldest queued slot frees first.
        [[nodiscard]] static std::size_t outstanding_() {
            return static_cast<std::uint8_t>(issued_ - completed_);
        }

        /// `issued_` counts the request before the submit: its completion may come before submit()
        /// returns. A refused request gets no callback.
        static bool queue_(typename Master::Request r) {
            r.callback = [gen = generation_](auto res) {
                if(res != decltype(res)::ok && gen == generation_) { failed_ = true; }
                completed_ = static_cast<std::uint8_t>(completed_ + 1U);
            };
            if(outstanding_() == 0) {
                seen_     = completed_;
                progress_ = Clock::now();
            }
            issued_ = static_cast<std::uint8_t>(issued_ + 1U);
            if(!Master::submit(r)) {
                issued_ = static_cast<std::uint8_t>(issued_ - 1U);
                return false;
            }
            head_ = static_cast<std::uint8_t>((head_ + 1U) % CommandSlots);
            return true;
        }

        static void finish_() {
            busy_   = false;
            pixels_ = {};
            fill_   = 0;
        }

        using ReadFrame = SPI::OneFrame<Master>;

        /// The frame is static, not on the stack: a completion after the wait gave up must not
        /// write into a frame that is gone. After a timeout the next read is refused until the
        /// timed-out frame's callback has come (the master gives every accepted request exactly one),
        /// so a late completion can never pass for the next read's.
        static bool read_(typename Master::Request r) {
            if(!readFrame_.start(r)) { return false; }
            auto const deadline = Clock::now() + CommandTimeout;
            auto       poll     = readFrame_.take();
            while(poll == ReadFrame::Poll::running) {
                Master::turn();
                if(Clock::now() > deadline) {
                    failed_ = true;
                    return false;
                }
                poll = readFrame_.take();
            }
            return poll == ReadFrame::Poll::ok;
        }

        inline static ReadFrame readFrame_{};

        /// Interrupt context on the RP: nothing here logs or waits.
        static void onChunk_(bool succeeded) {
            if(!succeeded) {
                ok_   = false;
                done_ = true;
                return;
            }
            if(!pixels_.empty() || fill_ != 0) {
                next_();
            } else {
                ok_   = true;
                done_ = true;
            }
        }

        static void next_() {
            typename Master::Request r{.setup = WriteSetup};
            r.callback = [](auto res) { onChunk_(res == decltype(res)::ok); };
            if(!started_) {
                command_[0] = std::byte{Dcs::Cmd::Ramwr};
                r.lines     = commandLines;
                r.command   = std::span<std::byte const>{command_};
            } else {
                r.lines = dataLines;
            }
            if(!pixels_.empty()) {
                auto const n = std::min(pixels_.size(), MaxChunk);
                r.tx         = pixels_.first(n);
                pixels_      = pixels_.subspan(n);
                r.hold       = !pixels_.empty();
            } else if(fill_ != 0) {
                if constexpr(Wide) {
                    auto const n = std::min(fill_, MaxChunk);
                    r.tx         = std::as_bytes(std::span{&pattern_, 1});
                    r.wide       = true;
                    r.repeat     = static_cast<std::uint32_t>(n);
                    fill_ -= n;
                } else {
                    auto const n = std::min(fill_, FillBuffer);
                    r.tx         = std::span<std::byte const>{fillBuf_}.first(2 * n);
                    fill_ -= n;
                }
                r.hold = fill_ != 0;
            } else {
                ok_   = true;
                done_ = true;
                return;
            }
            bool const continuation = started_;
            started_                = true;
            if(!Master::submit(r)) {
                // A continuation is refused only as malformed; let go of the held CS.
                if(continuation) { Master::releaseHold(commandLines); }
                ok_   = false;
                done_ = true;
            }
        }
    };

}}   // namespace Kvasir::Display
