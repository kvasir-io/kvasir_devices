#pragma once
// An SD card (SPI/SdCard.hpp) as the synchronous block device the fs library's FAT code takes (met by
// shape, nothing here includes fs).
//
// Every call BLOCKS the main loop, running only the master's and the card's handlers, until its block
// is done or `Config::Timeout` is up. A firmware that must keep a display, USB or a watchdog alive
// through a format (a minute) needs the file system on another core.
//
// A read with a CRC error is tried again: the CRC16 covers the wire, not the flash. Writes are not.
#include "SdCard.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace SPI {

    struct SdBlockDeviceDefaults {
        /// Per block, the wait for the card to be ready included.
        static constexpr std::chrono::milliseconds Timeout{2000};
        static constexpr std::uint8_t              ReadRetries = 2;
    };

    template<typename Card,
             typename Master,
             typename Clock,
             typename Config = SdBlockDeviceDefaults>
    class SdBlockDevice {
    public:
        explicit SdBlockDevice(Card& card) : card_{card} {}

        bool waitUp(std::chrono::milliseconds timeout) {
            auto const end = Clock::now() + timeout;
            while(!card_.up()) {
                pump_();
                if(Clock::now() > end) { return false; }
            }
            return true;
        }

        [[nodiscard]] std::uint32_t blockCount() const { return card_.blocks(); }

        bool readBlock(std::uint32_t               lba,
                       std::span<std::byte,
                                 Card::BlockBytes> out) {
            for(std::uint8_t attempt = 0;; ++attempt) {
                auto const r = run_([&] { return card_.read(lba, out); });
                if(r == SdResult::ok) {
                    ++reads_;
                    return true;
                }
                if(r != SdResult::crcError || attempt >= Config::ReadRetries) {
                    ++failures_;
                    return false;
                }
                ++retries_;
            }
        }

        bool writeBlock(std::uint32_t               lba,
                        std::span<std::byte const,
                                  Card::BlockBytes> in) {
            if(run_([&] { return card_.write(lba, in); }) == SdResult::ok) {
                ++writes_;
                return true;
            }
            ++failures_;
            return false;
        }

        [[nodiscard]] std::uint64_t reads() const { return reads_; }

        [[nodiscard]] std::uint64_t writes() const { return writes_; }

        [[nodiscard]] std::uint32_t retries() const { return retries_; }

        [[nodiscard]] std::uint32_t failures() const { return failures_; }

    private:
        Card&         card_;
        std::uint64_t reads_{};
        std::uint64_t writes_{};
        std::uint32_t retries_{};
        std::uint32_t failures_{};

        void pump_() {
            Master::handler();
            card_.handler();
        }

        template<typename Start>
        SdResult run_(Start start) {
            auto const end = Clock::now() + Config::Timeout;
            while(!card_.ready()) {
                pump_();
                if(Clock::now() > end) { return SdResult::timeout; }
            }
            if(auto const r = start(); r != SdResult::ok) { return r; }
            for(;;) {
                pump_();
                if(auto const d = card_.takeDone()) { return *d; }
                if(Clock::now() > end) { return SdResult::timeout; }
            }
        }
    };

}}   // namespace Kvasir::SPI
