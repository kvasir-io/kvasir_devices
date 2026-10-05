#pragma once
// An SD memory card in SPI mode: bring-up, single-block read and write, on a queued SPI master.
// Non-blocking: one request in flight, handler() once a loop turn; the callbacks only set flags.
//
// Facts from the SD Physical Layer Simplified Specification 9.10 ("SD spec", cited as
// section:line). It leaves out the SPI timing values (7.5), so NCR is our own `ResponseBytes`.
//
// A command and its data are one CS-low sequence of held requests. A main-loop pause longer than
// the master's HoldTimeout between two of them cuts it: the card is parked for RetryAfter and
// brought up again.
//
// A pulled card is only seen reliably with a pull-up on MISO (RP2350 pads reset pulled DOWN):
// with MISO low every command reads R1 0x00 and every data phase an error token, hence
// RejectedLost.
#include "../Duration.hpp"
#include "../Link.hpp"
#include "../Log.hpp"
#include "../Quantities.hpp"
#include "QueueCore.hpp"
#include "kvasir/Util/Crc.hpp"
#include "kvasir/Util/Periodic.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace Kvasir { namespace SPI {

    struct SdCardDefaults {
        /// SD spec 4.2.1:2790, 4.2:2782, 4.3:3019.
        static constexpr Units::Hertz IdentificationClock = Units::hertz(400'000);
        /// Default Speed mode (SD spec 2:1404).
        static constexpr Units::Hertz DataClock = Units::hertz(25'000'000);
        /// ACMD41 until the card leaves idle (4.2.3:2829, :2845).
        static constexpr std::chrono::milliseconds InitTimeout{1000};
        /// SDHC/SDXC read (4.6.2.1:4639).
        static constexpr std::chrono::milliseconds ReadTimeout{100};
        /// Busy after a block: 250 ms for SDHC (4.6.2.2:4645), up to 500 ms on SDXC (:4647-4649).
        static constexpr std::chrono::milliseconds WriteTimeout{500};
        static constexpr std::chrono::milliseconds RetryAfter{1000};
        /// Rejections in a row after which the card counts as lost; one is a bad block.
        static constexpr std::uint8_t RejectedLost = 8;
    };

    enum class SdResult : std::uint8_t {
        ok,
        busy,
        notReady,
        timeout,
        crcError,
        rejected,   ///< R1 with an error bit, a data error token, or a write not accepted
        busFault,
    };

    template<typename Spi, typename Clock, typename Cs, typename Config = SdCardDefaults>
    class SdCard {
    public:
        static constexpr std::size_t BlockBytes    = 512;
        using Block                                = std::span<std::byte, BlockBytes>;
        using ConstBlock                           = std::span<std::byte const, BlockBytes>;
        static constexpr std::uint8_t RejectedLost = [] {
            if constexpr(requires { Config::RejectedLost; }) {
                return static_cast<std::uint8_t>(Config::RejectedLost);
            } else {
                return SdCardDefaults::RejectedLost;
            }
        }();

        SdCard() { apply(set(Cs{})); }

        SdCard(SdCard const&)            = delete;   // the transfers' callbacks point at it
        SdCard& operator=(SdCard const&) = delete;

        /// SdResult::ok when started; the outcome comes from takeDone().
        SdResult read(std::uint32_t lba,
                      Block         out) {
            if(auto const r = mayStart_(); r != SdResult::ok) { return r; }
            data_   = out.data();
            op_     = Op::read;
            opLba_  = lba;
            opDone_ = {};
            command_(17, address_(lba), 0, After::readToken);
            return SdResult::ok;
        }

        /// `in` must stay put until takeDone().
        SdResult write(std::uint32_t lba,
                       ConstBlock    in) {
            if(auto const r = mayStart_(); r != SdResult::ok) { return r; }
            cdata_  = in.data();
            op_     = Op::write;
            opLba_  = lba;
            opDone_ = {};
            command_(24, address_(lba), 0, After::writeBlock);
            return SdResult::ok;
        }

        std::optional<SdResult> takeDone() {
            auto const d = opDone_;
            opDone_      = {};
            return d;
        }

        [[nodiscard]] bool ready() const { return phase_ == Phase::ready && op_ == Op::none; }

        [[nodiscard]] bool up() const { return phase_ == Phase::ready; }

        [[nodiscard]] bool highCapacity() const { return ccs_; }

        [[nodiscard]] std::uint32_t blocks() const { return blocks_; }

        [[nodiscard]] std::uint32_t ocr() const { return ocr_; }

        [[nodiscard]] std::array<std::uint8_t,
                                 16> const&
        cid() const {
            return cid_;
        }

        [[nodiscard]] Link link() const { return link_; }

        [[nodiscard]] std::uint32_t bringUps() const { return bringUps_; }

        [[nodiscard]] std::uint32_t crcErrors() const { return crcErrors_; }

        [[nodiscard]] std::uint32_t errors() const { return errors_; }

        [[nodiscard]] std::uint8_t rejectedInRow() const { return rejectedInRow_; }

        void handler() {
            auto const now = Clock::now();
            if(inFlight_) {
                if(!done_) { return; }
                inFlight_ = false;
                if(!ok_) {
                    fault_(SdResult::busFault);
                    return;
                }
                advance_(now);
                return;
            }
            if(phase_ == Phase::waiting && !retry_.armed(now)) { start_(); }
        }

    private:
        enum class Phase : std::uint8_t { waiting, clocks, bringUp, ready };
        enum class Op : std::uint8_t { none, read, write };
        enum class After : std::uint8_t {
            goIdle,       // CMD0
            ifCond,       // CMD8
            appCmd,       // CMD55 before ACMD41
            opCond,       // ACMD41
            readOcr,      // CMD58
            csd,          // CMD9, then its data block
            cidRead,      // CMD10, then its data block
            readToken,    // CMD17, then its data block
            writeBlock,   // CMD24, then the block goes out
        };
        enum class Step : std::uint8_t {
            command,
            r1,
            extra,
            token,
            dataIn,
            dataOut,
            dataResponse,
            busy,
            gap,
        };

        static constexpr std::size_t ResponseBytes = 16;   // NCR bound, see the file comment

        static constexpr typename Spi::Setup identification_
          = Spi::setup(ClockMode::_0, Config::IdentificationClock);
        static constexpr typename Spi::Setup data_clock_
          = Spi::setup(ClockMode::_0, Config::DataClock);

        static void selectCs_() { apply(clear(Cs{})); }

        static void deselectCs_() { apply(set(Cs{})); }

        static constexpr Lines lines_{&selectCs_, &deselectCs_, nullptr};

        Phase                      phase_{Phase::waiting};
        Op                         op_{Op::none};
        Step                       step_{};
        After                      after_{};
        std::optional<SdResult>    opDone_{};
        Kvasir::Deadline<Clock>    retry_{};   ///< stopped: the first bring-up at once
        typename Clock::time_point deadline_{};
        typename Clock::time_point initDeadline_{};
        bool                       inFlight_{};
        bool volatile done_{};   ///< written by the completion
        bool volatile ok_{};
        bool                           ccs_{};
        bool                           v2_{};
        bool                           ocrBusy_{};
        std::uint8_t                   rejectedInRow_{};
        std::uint8_t                   cmd_{};
        std::uint8_t                   r1_{};
        std::uint8_t                   polls_{};
        std::uint8_t                   extraBytes_{};
        std::uint32_t                  ocr_{};
        std::uint32_t                  r7_{};
        std::uint32_t                  blocks_{};
        std::uint32_t                  opLba_{};
        std::uint32_t                  bringUps_{};
        std::uint32_t                  crcErrors_{};
        std::uint32_t                  errors_{};
        Link                           link_{Link::starting};
        std::byte*                     data_{};
        std::byte const*               cdata_{};
        std::size_t                    dataBytes_{};
        std::array<std::byte, 18>      small_{};   // CSD / CID block + CRC
        std::array<std::byte, 7>       frame_{};
        std::array<std::byte, 4>       extra_{};
        std::array<std::byte, 1>       one_{};
        std::array<std::byte, 2>       crc_{};
        std::array<std::byte, 10>      clocks_{};
        std::array<std::uint8_t, 16>   cid_{};
        std::array<std::byte, 1> const startToken_{std::byte{0xFE}};

        SdResult mayStart_() const {
            if(phase_ != Phase::ready) { return SdResult::notReady; }
            if(op_ != Op::none || inFlight_) { return SdResult::busy; }
            return SdResult::ok;
        }

        std::uint32_t address_(std::uint32_t lba) const { return ccs_ ? lba : lba * 512U; }

        // -- the wire ------------------------------------------------------------------------

        bool submit_(typename Spi::Request r) {
            done_ = false;
            ok_   = false;
            r.callback
              = [this](auto res) {   // the master's result type: TransferResult or ...Tracked
                    ok_   = res == decltype(res)::succeeded;
                    done_ = true;
                };
            inFlight_ = Spi::submit(r);
            if(!inFlight_) {
                // A full queue: the sequence is failed and the card tried again.
                fault_(SdResult::busFault);
            }
            return inFlight_;
        }

        void send_(std::span<std::byte const> tx,
                   bool                       hold) {
            submit_(
              typename Spi::Request{.setup = setup_(), .lines = lines_, .tx = tx, .hold = hold});
        }

        void receive_(std::span<std::byte> rx) {
            submit_(
              typename Spi::Request{.setup = setup_(), .lines = lines_, .rx = rx, .hold = true});
        }

        typename Spi::Setup setup_() const {
            return phase_ == Phase::ready ? data_clock_ : identification_;
        }

        /// CRC7, G(x) = x^7 + x^3 + 1 (SD spec 4.5; CRC-7/MMC), as CRC << 1 | end bit.
        static constexpr std::uint8_t crc7_(std::span<std::byte const> bytes) {
            return static_cast<std::uint8_t>(
              (static_cast<unsigned>(Crc::Crc7Mmc<>::compute(bytes)) << 1U) | 1U);
        }

        static_assert(crc7_(std::array{std::byte{0x40},
                                       std::byte{0},
                                       std::byte{0},
                                       std::byte{0},
                                       std::byte{0}})
                        == 0x95,
                      "CMD0's CRC byte is 0x95 (SD spec 7.2.2)");
        static_assert(crc7_(std::array{std::byte{0x48},
                                       std::byte{0},
                                       std::byte{0},
                                       std::byte{0x01},
                                       std::byte{0xAA}})
                        == 0x87,
                      "CMD8(0x1AA)'s CRC byte is 0x87");

    public:
        /// CRC16-CCITT, G(x) = x^16 + x^12 + x^5 + 1 (SD spec 4.5; CRC-16/XMODEM), over a data
        /// block. A byte table (512 bytes of flash): it runs over every 512-byte block read.
        static constexpr std::uint16_t crc16(std::span<std::byte const> bytes) {
            return Crc::Crc16Xmodem<256>::compute(bytes);
        }

    private:
        /// Independent vectors: a wrong CRC16 would pass a model that borrowed this function.
        static constexpr std::uint16_t crc16OfBlockFilledWith_(std::uint8_t v) {
            std::array<std::byte, BlockBytes> b{};
            for(auto& x : b) { x = std::byte{v}; }
            return crc16(b);
        }

        static_assert(crc16OfBlockFilledWith_(0xFF) == 0x7FA1,
                      "CRC16 of 512 x 0xFF is 0x7FA1");
        static_assert(crc16OfBlockFilledWith_(0x00) == 0x0000,
                      "CRC16 of 512 x 0x00 is 0");

        /// A command token (SD spec 7.3.1.1) after one 0xFF of clocks.
        void command_(std::uint8_t  index,
                      std::uint32_t arg,
                      std::uint8_t  extra,
                      After         after) {
            cmd_        = index;
            extraBytes_ = extra;
            after_      = after;
            frame_[0]   = std::byte{0xFF};
            frame_[1]   = std::byte{static_cast<std::uint8_t>(0x40U | index)};
            frame_[2]   = std::byte{static_cast<std::uint8_t>(arg >> 24U)};
            frame_[3]   = std::byte{static_cast<std::uint8_t>(arg >> 16U)};
            frame_[4]   = std::byte{static_cast<std::uint8_t>(arg >> 8U)};
            frame_[5]   = std::byte{static_cast<std::uint8_t>(arg)};
            frame_[6]   = std::byte{crc7_(std::span{frame_}.subspan(1, 5))};
            step_       = Step::command;
            polls_      = 0;
            send_(frame_, true);
        }

        /// CS up, then eight clocks so the card lets go of MISO.
        void endSequence_() {
            Spi::releaseHold(lines_);
            step_ = Step::gap;
            clocks_.fill(std::byte{0xFF});
            submit_(typename Spi::Request{.setup = setup_(),
                                          .lines = Lines{},
                                          .tx    = std::span{clocks_}.first(1)});
        }

        void start_() {
            // At least 74 clocks with CS high (SD spec 6.4.1:11514).
            phase_ = Phase::clocks;
            link_  = Link::starting;
            ccs_   = false;
            v2_    = false;
            clocks_.fill(std::byte{0xFF});
            step_ = Step::gap;
            submit_(
              typename Spi::Request{.setup = identification_, .lines = Lines{}, .tx = clocks_});
        }

        void fault_(SdResult r) {
            ++errors_;
            Spi::releaseHold(lines_);
            if(op_ != Op::none) {
                opDone_ = r;
                op_     = Op::none;
                if(r == SdResult::crcError) {
                    return;   // the card answered: a disturbed transfer, not a lost card
                }
                if(r == SdResult::rejected) {
                    if(rejectedInRow_ != 0xFF) { ++rejectedInRow_; }
                    if(rejectedInRow_ < RejectedLost) { return; }
                    UC_LOG_W("sd card: {} operations rejected in a row -- lost?", rejectedInRow_);
                }
            }
            if(phase_ == Phase::ready) {
                UC_LOG_W("sd card lost ({}) -- bringing it up again", static_cast<std::uint8_t>(r));
            }
            phase_ = Phase::waiting;
            link_  = Link::absent;
            retry_.restart(Kvasir::asDuration(Config::RetryAfter), Clock::now());
        }

        /// Reported once the sequence and its clocks are over, so the next one can start at once.
        void finishOp_(SdResult r) {
            if(r == SdResult::ok) { rejectedInRow_ = 0; }
            result_ = r;
            endSequence_();
        }

        SdResult result_{};

        void advance_(typename Clock::time_point now) {
            switch(step_) {
            case Step::gap:
                if(phase_ == Phase::clocks) {
                    phase_        = Phase::bringUp;
                    initDeadline_ = now + Kvasir::asDuration(Config::InitTimeout);
                    command_(0, 0, 0, After::goIdle);
                } else if(phase_ == Phase::bringUp && after_ == After::cidRead) {
                    up_();
                } else if(phase_ == Phase::bringUp) {
                    nextBringUpCommand_(now);
                } else if(op_ != Op::none) {
                    opDone_ = result_;
                    op_     = Op::none;
                }
                return;
            case Step::command:
                step_ = Step::r1;
                receive_(one_);
                return;
            case Step::r1:
                {
                    auto const b = std::to_integer<std::uint8_t>(one_[0]);
                    if((b & 0x80U) != 0) {   // not a response yet: the MSB of R1 is 0 (7.3.2.1)
                        if(++polls_ >= ResponseBytes) {
                            endAndFault_(SdResult::timeout);
                            return;
                        }
                        receive_(one_);
                        return;
                    }
                    r1_ = b;
                    if(extraBytes_ != 0) {
                        step_ = Step::extra;
                        receive_(std::span{extra_}.first(extraBytes_));
                        return;
                    }
                    afterResponse_(now);
                    return;
                }
            case Step::extra: afterResponse_(now); return;
            case Step::token:
                {
                    auto const b = std::to_integer<std::uint8_t>(one_[0]);
                    if(b == 0xFF) {
                        if(now > deadline_) {
                            endAndFault_(SdResult::timeout);
                            return;
                        }
                        receive_(one_);
                        return;
                    }
                    if(b != 0xFE) {   // a data error token (7.3.3.3)
                        endAndFault_(SdResult::rejected);
                        return;
                    }
                    step_ = Step::dataIn;
                    if(after_ == After::readToken) {
                        receive_(std::span{data_, BlockBytes});
                    } else {
                        receive_(std::span{small_}.first(16));
                    }
                    return;
                }
            case Step::dataIn:
                step_ = Step::dataResponse;   // the CRC16 after the data
                receive_(crc_);
                return;
            case Step::dataOut:
                if(sendParts_ < 3) {
                    sendNextPart_();
                    return;
                }
                step_ = Step::dataResponse;
                receive_(one_);
                return;
            case Step::dataResponse:
                if(after_ == After::writeBlock) {
                    // xxx0sss1: 010 accepted, 101 CRC error, 110 write error (7.3.3.1)
                    auto const b = std::to_integer<std::uint8_t>(one_[0]) & 0x1FU;
                    if(b != 0x05U) {
                        endAndFault_(SdResult::rejected);
                        return;
                    }
                    step_     = Step::busy;
                    deadline_ = now + Kvasir::asDuration(Config::WriteTimeout);
                    receive_(one_);
                    return;
                }
                blockIn_(now);
                return;
            case Step::busy:
                // The card holds MISO low while it programs (7.2.4, R1b 7.3.2.2)
                if(std::to_integer<std::uint8_t>(one_[0]) == 0x00) {
                    if(now > deadline_) {
                        endAndFault_(SdResult::timeout);
                        return;
                    }
                    receive_(one_);
                    return;
                }
                finishOp_(SdResult::ok);
                return;
            }
        }

        void blockIn_(typename Clock::time_point) {
            auto const got
              = static_cast<std::uint16_t>((std::to_integer<std::uint16_t>(crc_[0]) << 8U)
                                           | std::to_integer<std::uint16_t>(crc_[1]));
            if(after_ == After::readToken) {
                if(crc16(std::span<std::byte const>{data_, BlockBytes}) != got) {
                    ++crcErrors_;
                    finishOp_(SdResult::crcError);
                } else {
                    finishOp_(SdResult::ok);
                }
                return;
            }
            if(crc16(std::span{small_}.first(16)) != got) {
                ++crcErrors_;
                endAndFault_(SdResult::crcError);
                return;
            }
            if(after_ == After::csd) {
                csd_();
            } else {
                for(std::size_t i = 0; i < 16; ++i) {
                    cid_[i] = std::to_integer<std::uint8_t>(small_[i]);
                }
            }
            endSequence_();
        }

        void csd_() {
            auto const bit = [this](unsigned hi, unsigned lo) {
                // 128 bits, MSB first
                std::uint32_t v = 0;
                for(unsigned b = hi + 1; b-- > lo;) {
                    auto const byte = std::to_integer<std::uint8_t>(small_[15U - b / 8U]);
                    v               = (v << 1U) | ((byte >> (b % 8U)) & 1U);
                }
                return v;
            };
            if(bit(127, 126) == 1) {
                // CSD version 2.0: C_SIZE [69:48], capacity (C_SIZE + 1) x 512 KByte (5.3.3)
                blocks_ = (bit(69, 48) + 1U) * 1024U;
            } else {
                // version 1.0: (C_SIZE + 1) x 2^(C_SIZE_MULT + 2) x 2^READ_BL_LEN bytes (5.3.2)
                auto const bytes = (bit(73, 62) + 1U) << (bit(49, 47) + 2U + bit(83, 80));
                blocks_          = bytes / 512U;
            }
        }

        void endAndFault_(SdResult r) {
            Spi::releaseHold(lines_);
            fault_(r);
        }

        void afterResponse_(typename Clock::time_point now) {
            auto const extra = [this] {
                return (std::to_integer<std::uint32_t>(extra_[0]) << 24U)
                     | (std::to_integer<std::uint32_t>(extra_[1]) << 16U)
                     | (std::to_integer<std::uint32_t>(extra_[2]) << 8U)
                     | std::to_integer<std::uint32_t>(extra_[3]);
            };
            switch(after_) {
            case After::goIdle:
                // R1 0x01: in idle state, in SPI mode (7.2.1)
                if(r1_ != 0x01) {
                    endAndFault_(SdResult::notReady);
                    return;
                }
                break;
            case After::ifCond:
                if((r1_ & 0x04U) != 0) {
                    v2_ = false;   // illegal command: a version 1.x card (figure 7-2)
                } else if((extra() & 0xFFFU) == 0x1AAU) {
                    v2_ = true;   // VHS 2.7-3.6 V accepted, check pattern echoed (7.3.2.6)
                    r7_ = extra();
                } else {
                    endAndFault_(SdResult::rejected);   // "unusable card"
                    return;
                }
                break;
            case After::appCmd: break;
            case After::opCond:
                if((r1_ & 0x7EU) != 0) {
                    endAndFault_(SdResult::rejected);
                    return;
                }
                break;
            case After::readOcr:
                ocr_ = extra();
                // Bit 31 is the card power-up status: "CCS is valid when the card returns
                // ready (the busy bit is set to 1)" (4.2.3:2833, 5.1:9403, Figure 4-4:2896).
                // ACMD41 said it left idle, so it should be; if not, ACMD41 is polled on.
                ocrBusy_ = (ocr_ & (1U << 31U)) == 0;
                ccs_     = v2_ && !ocrBusy_ && (ocr_ & (1U << 30U)) != 0;   // CCS, bit 30
                break;
            case After::csd:
            case After::cidRead:
            case After::readToken:
                if(r1_ != 0) {
                    endAndFault_(SdResult::rejected);
                    return;
                }
                step_     = Step::token;
                deadline_ = now + Kvasir::asDuration(Config::ReadTimeout);
                receive_(one_);
                return;
            case After::writeBlock:
                if(r1_ != 0) {
                    endAndFault_(SdResult::rejected);
                    return;
                }
                // start token, block, CRC16: one held request each
                step_      = Step::dataOut;
                writeCrc_  = crc16(std::span<std::byte const>{cdata_, BlockBytes});
                crc_[0]    = std::byte{static_cast<std::uint8_t>(writeCrc_ >> 8U)};
                crc_[1]    = std::byte{static_cast<std::uint8_t>(writeCrc_)};
                sendParts_ = 0;
                sendNextPart_();
                return;
            }
            endSequence_();
        }

        std::uint16_t writeCrc_{};
        std::uint8_t  sendParts_{};

        void sendNextPart_() {
            switch(sendParts_++) {
            case 0:  send_(startToken_, true); break;
            case 1:  send_(std::span<std::byte const>{cdata_, BlockBytes}, true); break;
            default: send_(crc_, true); break;
            }
        }

        void nextBringUpCommand_(typename Clock::time_point now) {
            switch(after_) {
            case After::goIdle: command_(8, 0x1AAU, 4, After::ifCond); return;
            case After::ifCond: command_(55, 0, 0, After::appCmd); return;
            case After::appCmd: command_(41, v2_ ? (1U << 30U) : 0U, 0, After::opCond); return;
            case After::opCond:
                if((r1_ & 0x01U) != 0) {   // still idle: ACMD41 again (7.2.1)
                    if(now > initDeadline_) {
                        fault_(SdResult::timeout);
                        return;
                    }
                    command_(55, 0, 0, After::appCmd);
                    return;
                }
                command_(58, 0, 4, After::readOcr);
                return;
            case After::readOcr:
                if(ocrBusy_) {   // not powered up yet by its own OCR: ACMD41 again
                    if(now > initDeadline_) {
                        fault_(SdResult::timeout);
                        return;
                    }
                    command_(55, 0, 0, After::appCmd);
                    return;
                }
                command_(9, 0, 0, After::csd);
                return;
            case After::csd: command_(10, 0, 0, After::cidRead); return;
            default:         return;
            }
        }

        void up_() {
            phase_         = Phase::ready;
            link_          = Link::answering;
            rejectedInRow_ = 0;
            ++bringUps_;
            // CID (5.2): MID [127:120], OID [119:104], PNM [103:64], PSN [55:24]
            UC_LOG_I(
              "sd card up: {}, OCR {:#010x}, {} blocks, CID MID {:#04x} OID {:#06x} PSN "
              "{:#010x}",
              std::string_view{ccs_ ? "SDHC/SDXC" : "SDSC"},
              ocr_,
              blocks_,
              cid_[0],
              static_cast<std::uint16_t>((cid_[1] << 8U) | cid_[2]),
              (std::uint32_t{cid_[9]} << 24U) | (std::uint32_t{cid_[10]} << 16U)
                | (std::uint32_t{cid_[11]} << 8U) | std::uint32_t{cid_[12]});
        }
    };

}}   // namespace Kvasir::SPI
