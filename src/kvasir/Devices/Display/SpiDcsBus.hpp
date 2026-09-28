#pragma once
// A DCS panel on 4-wire SPI with a D/C line (ST7789, ILI9341, ...) as a Display::Bus, over a queued
// SPI master. A command is one request: its byte with D/C low, the parameters with D/C high (flipped
// in `between`). Pixels are asynchronous, by DMA from the caller's memory; a stream longer than the
// master takes goes out in chunks under one chip select (`hold`), each submitted from the previous
// one's completion callback: the ST7789V must never see CSX rise inside a RAMWR (CSX high
// initialises its serial interface, ST7789V.md:2147). On the SAM completions are polled, so only a
// loop pause longer than the master's HoldTimeout cuts a fill.
#include "../Quantities.hpp"
#include "../SPI/QueueCore.hpp"
#include "Dcs.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Display {

    struct SpiDcsBusDefaults {
        static constexpr SPI::ClockMode            Mode       = SPI::ClockMode::_0;
        static constexpr Units::Hertz              WriteClock = Units::hertz(15'000'000);
        static constexpr Units::Hertz              ReadClock  = Units::hertz(6'000'000);
        static constexpr std::chrono::milliseconds CommandTimeout{50};
    };

    template<typename Master,
             typename Clock,
             typename Cs,
             typename Dc,
             typename Config = SpiDcsBusDefaults>
    struct SpiDcsBus {
        static constexpr auto WriteSetup = Master::setup(Config::Mode, Config::WriteClock);
        static constexpr auto ReadSetup  = Master::setup(Config::Mode, Config::ReadClock);
        /// Checked by DcsPanel against the chip's ceilings.
        static constexpr unsigned long WriteHz = WriteSetup.hz;
        static constexpr unsigned long ReadHz  = ReadSetup.hz;

        static constexpr bool Wide = [] {
            if constexpr(requires { Master::Core::SupportsWide; }) {
                return Master::Core::SupportsWide;
            } else {
                return true;
            }
        }();
        /// The SAM DMAC counts 16 bits.
        static constexpr std::size_t MaxChunk = [] {
            if constexpr(requires { Master::Core::MaxFrames; }) {
                return std::min<std::size_t>(Master::Core::MaxFrames, std::size_t{1} << 24U);
            } else {
                return std::size_t{1} << 24U;
            }
        }();

        static bool writeCommand(std::uint8_t               cmd,
                                 std::span<std::byte const> params) {
            if(busy_) { return false; }
            command_[0] = std::byte{cmd};
            typename Master::Request r{.setup   = WriteSetup,
                                       .lines   = lines_,
                                       .command = std::span<std::byte const>{command_},
                                       .tx      = params};
            return run_(r);
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
            command_[0] = std::byte{cmd};
            typename Master::Request r{.setup   = ReadSetup,
                                       .lines   = lines_,
                                       .command = std::span<std::byte const>{command_},
                                       .rx      = std::span{readBuf_}.first(dummy + out.size())};
            if(!run_(r)) { return false; }
            std::copy_n(readBuf_.begin() + static_cast<std::ptrdiff_t>(dummy),
                        out.size(),
                        out.begin());
            return true;
        }

        [[nodiscard]] static bool idle() { return !busy_; }

        [[nodiscard]] static bool failed() { return failed_; }

        static void clearError() { failed_ = false; }

        /// Where the loop learns a pixel stream's outcome.
        static void handler() {
            Master::handler();
            if(busy_ && done_) {
                done_ = false;
                if(!ok_) { failed_ = true; }
                finish_();
            }
        }

    private:
        /// For a master without 16-bit repeat frames.
        static constexpr std::size_t FillBuffer = 128;

        static void select_() { apply(clear(Cs{})); }

        static void deselect_() { apply(set(Cs{})); }

        static void command_Line_() { apply(clear(Dc{})); }

        static void data_() { apply(set(Dc{})); }

        static constexpr SPI::Lines lines_{&select_, &deselect_, &command_Line_, &data_};
        static constexpr SPI::Lines dataLines_{&select_, &deselect_, &data_, nullptr};

        inline static std::array<std::byte, 1>              command_{};
        inline static std::array<std::byte, 16>             readBuf_{};
        inline static std::array<std::byte, 2 * FillBuffer> fillBuf_{};
        inline static std::uint16_t                         pattern_{};
        // Written by whichever context runs next_() (loop or completion callback, never both),
        // read by the loop only after done_.
        inline static std::span<std::byte const> pixels_{};
        inline static std::size_t                fill_{};
        inline static bool                       started_{};
        // Crossing from the callback to the loop.
        inline static bool volatile busy_{};
        inline static bool volatile done_{};
        inline static bool volatile ok_{};
        inline static bool volatile failed_{};

        static void finish_() {
            busy_   = false;
            pixels_ = {};
            fill_   = 0;
        }

        /// The flags are static, not on the stack: a completion after the wait gave up must not
        /// write into a frame that is gone.
        static bool run_(typename Master::Request r) {
            cmdDone_   = false;
            cmdOk_     = false;
            r.callback = [](SPI::TransferResult res) {
                cmdOk_   = res == SPI::TransferResult::succeeded;
                cmdDone_ = true;
            };
            if(!Master::submit(r)) { return false; }
            auto const deadline = Clock::now() + Config::CommandTimeout;
            while(!cmdDone_) {
                Master::handler();
                if(Clock::now() > deadline) {
                    failed_ = true;
                    return false;
                }
            }
            return cmdOk_;
        }

        inline static bool volatile cmdDone_{};
        inline static bool volatile cmdOk_{};

        /// Interrupt context on the RP: nothing here logs or waits.
        static void onChunk_(SPI::TransferResult res) {
            if(res != SPI::TransferResult::succeeded) {
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
            r.callback = &onChunk_;
            if(!started_) {
                command_[0] = std::byte{Dcs::Cmd::Ramwr};
                r.lines     = lines_;
                r.command   = std::span<std::byte const>{command_};
            } else {
                r.lines = dataLines_;
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
                if(continuation) { Master::releaseHold(lines_); }
                ok_   = false;
                done_ = true;
            }
        }
    };

}}   // namespace Kvasir::Display
