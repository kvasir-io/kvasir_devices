#pragma once
/// An SD card in SPI mode for sd_card_test and sd_block_device_test.
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <span>
#include <vector>

namespace Kvasir::Test {

/// A card in SPI mode, byte by byte (SD spec 7).
template<typename Card>
struct SdModel {
    bool          spiMode{};
    bool          appCmd{};
    int           opCondPolls{2};   // ACMD41s before it leaves idle
    std::uint32_t cSize{61'047};    // a "32 GB" card
    std::map<std::uint32_t, std::array<std::uint8_t, 512>> blocks{};
    std::deque<std::uint8_t>                               out{};
    std::array<std::uint8_t, 6>                            cmd{};
    std::size_t                                            cmdAt{};
    // what a write is waiting for
    enum class Rx : std::uint8_t { command, writeToken, writeData } rx{Rx::command};
    std::uint32_t             writeLba{};
    std::vector<std::uint8_t> writeBuf{};
    // faults to play
    bool corruptNextRead{};
    bool noTokenNextRead{};
    bool rejectNextWrite{};
    /// Every read answered with an error token: a bus with no card and MISO floating low.
    bool rejectAllReads{};
    /// 0xFF bytes before a read's data token (the card's access time).
    int tokenDelayBytes{0};
    int busyBytes{3};

    /// CRC16-CCITT, table-driven: independent of the driver's bitwise one.
    static constexpr std::array<std::uint16_t, 256> Crc16Table = [] {
        std::array<std::uint16_t, 256> t{};
        for(std::uint32_t n = 0; n < 256; ++n) {
            std::uint32_t c = n << 8U;
            for(int i = 0; i < 8; ++i) {
                c = (c & 0x8000U) != 0 ? ((c << 1U) ^ 0x1021U) : (c << 1U);
            }
            t[n] = static_cast<std::uint16_t>(c & 0xFFFFU);
        }
        return t;
    }();

    static constexpr std::uint16_t crc16(std::span<std::uint8_t const> d) {
        std::uint16_t crc = 0;
        for(auto b : d) {
            crc = static_cast<std::uint16_t>((crc << 8U) ^ Crc16Table[((crc >> 8U) ^ b) & 0xFFU]);
        }
        return crc;
    }

    void block(std::span<std::uint8_t const> d) {
        out.push_back(0xFF);
        out.push_back(0xFE);
        for(auto b : d) { out.push_back(b); }
        auto crc = crc16(d);
        out.push_back(static_cast<std::uint8_t>(crc >> 8U));
        out.push_back(static_cast<std::uint8_t>(crc));
    }

    void r1(std::uint8_t v) {
        out.push_back(0xFF);   // one byte of NCR
        out.push_back(v);
    }

    void command() {
        auto const index = cmd[0] & 0x3FU;
        auto const arg   = (std::uint32_t{cmd[1]} << 24U) | (std::uint32_t{cmd[2]} << 16U)
                         | (std::uint32_t{cmd[3]} << 8U) | cmd[4];
        bool const idle  = opCondPolls > 0;
        if(!spiMode) {
            // SD mode: only CMD0 with CS low and its CRC 0x95 gets through (7.2.1, 7.2.2)
            if(index == 0 && cmd[5] == 0x95) {
                spiMode = true;
                r1(0x01);
            }
            return;
        }
        auto const was = appCmd;
        appCmd         = false;
        switch(index) {
        case 0: r1(0x01); break;
        case 8:
            r1(idle ? 0x01 : 0x00);
            for(auto b : {0x00, 0x00, 0x01, static_cast<int>(arg & 0xFFU)}) {
                out.push_back(static_cast<std::uint8_t>(b));
            }
            break;
        case 55:
            appCmd = true;
            r1(idle ? 0x01 : 0x00);
            break;
        case 41:
            if(!was) {
                r1(0x05);   // illegal without CMD55
                break;
            }
            if(opCondPolls > 0) { --opCondPolls; }
            r1(opCondPolls > 0 ? 0x01 : 0x00);
            break;
        case 58:
            r1(0x00);
            for(auto b : {0xC0, 0xFF, 0x80, 0x00}) { out.push_back(static_cast<std::uint8_t>(b)); }
            break;
        case 9:
            {
                r1(0x00);
                std::array<std::uint8_t, 16> csd{};
                csd[0] = 0x40;   // CSD_STRUCTURE 1 (version 2.0)
                // C_SIZE [69:48]: bytes 7 (bits 69:64 in its low 6 bits), 8, 9
                csd[7] = static_cast<std::uint8_t>((cSize >> 16U) & 0x3FU);
                csd[8] = static_cast<std::uint8_t>(cSize >> 8U);
                csd[9] = static_cast<std::uint8_t>(cSize);
                block(csd);
                break;
            }
        case 10:
            {
                r1(0x00);
                std::array<std::uint8_t, 16> cid{0x03,
                                                 'S',
                                                 'D',
                                                 'S',
                                                 'C',
                                                 '3',
                                                 '2',
                                                 'G',
                                                 0x80,
                                                 0x12,
                                                 0x34,
                                                 0x56,
                                                 0x78,
                                                 0x01,
                                                 0x6A,
                                                 0x01};
                block(cid);
                break;
            }
        case 17:
            {
                r1(0x00);
                if(noTokenNextRead) {
                    noTokenNextRead = false;
                    break;   // no token ever: the host's 100 ms run out
                }
                if(rejectAllReads) {
                    out.push_back(0x00);   // an error token (7.3.3.3), MISO-low style
                    break;
                }
                for(int i = 0; i < tokenDelayBytes; ++i) { out.push_back(0xFF); }
                auto&                         b    = blocks[arg];
                std::array<std::uint8_t, 512> copy = b;
                block(copy);
                if(corruptNextRead) {
                    corruptNextRead = false;
                    out.back() ^= 0x01U;
                }
                break;
            }
        case 24:
            r1(0x00);
            writeLba = arg;
            rx       = Rx::writeToken;
            writeBuf.clear();
            break;
        default: r1(0x04); break;
        }
    }

    /// The answer to a byte starts at the next one.
    std::uint8_t exchange(std::uint8_t mosi) {
        std::uint8_t v = 0xFF;
        if(!out.empty()) {
            v = out.front();
            out.pop_front();
        }
        switch(rx) {
        case Rx::command:
            if(cmdAt == 0 && (mosi & 0xC0U) != 0x40U) { break; }
            cmd[cmdAt++] = mosi;
            if(cmdAt == 6) {
                cmdAt = 0;
                command();
            }
            break;
        case Rx::writeToken:
            if(mosi == 0xFE) { rx = Rx::writeData; }
            break;
        case Rx::writeData:
            writeBuf.push_back(mosi);
            if(writeBuf.size() == 514) {
                rx = Rx::command;
                if(rejectNextWrite) {
                    rejectNextWrite = false;
                    out.push_back(0x0D);   // 110: write error (7.3.3.1)
                    break;
                }
                std::copy_n(writeBuf.begin(), 512, blocks[writeLba].begin());
                out.push_back(0xE5);   // xxx0 010 1: accepted
                for(int i = 0; i < busyBytes; ++i) { out.push_back(0x00); }
            }
            break;
        }
        return v;
    }

    /// CS going high ends whatever the card was saying (7.2.4: it lets MISO go).
    void deselected() {
        out.clear();
        cmdAt = 0;
    }
};

}   // namespace Kvasir::Test
