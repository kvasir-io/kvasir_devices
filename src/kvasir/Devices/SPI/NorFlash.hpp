#pragma once

#include "../Duration.hpp"
#include "../SPIDeviceBase.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::SPI {

/// The knobs of the NOR flash driver, with their defaults; derive and redeclare what you
/// change. The timeouts are how long the part may stay busy after the command, about twice
/// the datasheet maxima of the Winbond W25Q / Macronix MX25 / Micron N25Q families (page
/// program 3 ms, sector erase 400 ms), since they are counted from when the driver saw the
/// command's frame complete; a chip erase takes from 2 s (a 2 MB part) to 200 s (W25Q128) or
/// more, so ChipEraseTimeout is set per part and defaults to twice the W25Q128's. The
/// SPIDeviceDefaults members apply too.
struct NorFlashDefaults : SPIDeviceDefaults {
    static constexpr auto PageProgramTimeout = std::chrono::milliseconds{10};
    static constexpr auto SectorEraseTimeout = std::chrono::milliseconds{1000};
    static constexpr auto ChipEraseTimeout   = std::chrono::seconds{400};

    /// The status register is read at this interval while the part is busy.
    static constexpr auto StatusPollInterval = std::chrono::milliseconds{1};
};

/// A generic SPI NOR flash (Winbond W25Q, Macronix MX25, Micron N25Q ...: the JEDEC
/// SFDP-era command set every one of them shares): JEDEC id 0x9F, read 0x03 with a 24-bit
/// address, write enable 0x06, page program 0x02 (256 bytes, inside one page), sector
/// erase 0x20 (4 KB), chip erase 0xC7, read status 0x05 (bit 0 busy, bit 1 write enable
/// latch), release from deep power-down 0xAB, reset 0x66 then 0x99. Asynchronous: start an
/// operation, poll takeDone() (once per completion) or finished() (level), then result(); the
/// driver runs it as a sequence of frames, waiting on the status register while the chip
/// erases or programs, up to the operation's timeout.
///
///   flash.readJedec();   ... flash.takeDone() -> flash.jedec()
///   flash.read(addr, span)            up to PageSize bytes
///   flash.eraseSector(addr)           addr on a 4 KB boundary
///   flash.eraseChip()
///   flash.program(addr, span)         up to PageSize bytes, not crossing a page
///   flash.unlockAll()                 Global Block/Sector Unlock (0x98)
///
/// readJedec() is the bring-up: it first releases the part from deep power-down (earlier
/// firmware may have left it there, and then it ignores everything else, W25Q128JV 8.2.22)
/// and resets it (8.2.37: any operation still running is abandoned, 30 us), then reads the
/// id. A busy part ignores every instruction but Read Status (W25Q128JV 7.1.1), so an
/// operation after one that failed or timed out, and the first one after readJedec(), waits
/// for BUSY to clear before it starts, up to ChipEraseTimeout; one that finds it still busy
/// then ends timedOut. Program and erase instructions are ignored for tPUW (5 ms) after
/// power-up: readJedec() is not to be called sooner than that.
///
/// A W25Q with WPS = 1 (Status Register-3) powers up with every block lock set, and then
/// erases and programs are ignored without an error; unlockAll() clears them. It is a
/// Winbond (and Macronix) instruction, so it is left to the application.
///
/// Addresses are 24 bits: an operation at or past 16 MB is refused (no 4-byte addressing).
template<typename Spi, typename Clock, typename Cs, typename Config = NorFlashDefaults>
struct NorFlash : SPIDeviceBase<Spi, Clock, Cs, NorFlash<Spi, Clock, Cs, Config>, Config, 4 + 256> {
    static constexpr std::size_t PageSize   = 256;
    static constexpr std::size_t SectorSize = 4096;

    using Base    = SPIDeviceBase<Spi, Clock, Cs, NorFlash, Config, 4 + PageSize>;
    using Outcome = typename Base::Outcome;

    static constexpr std::string_view Name = "SPI NOR flash";

    static constexpr std::uint8_t CmdPageProgram    = 0x02;
    static constexpr std::uint8_t CmdRead           = 0x03;
    static constexpr std::uint8_t CmdReadStatus     = 0x05;
    static constexpr std::uint8_t CmdWriteEnable    = 0x06;
    static constexpr std::uint8_t CmdSectorErase    = 0x20;
    static constexpr std::uint8_t CmdEnableReset    = 0x66;
    static constexpr std::uint8_t CmdGlobalUnlock   = 0x98;
    static constexpr std::uint8_t CmdReset          = 0x99;
    static constexpr std::uint8_t CmdJedecId        = 0x9F;
    static constexpr std::uint8_t CmdReleasePowerDn = 0xAB;
    static constexpr std::uint8_t CmdChipErase      = 0xC7;

    /// The largest address plus one: 24 bits on the wire.
    static constexpr std::uint32_t AddressSpace = std::uint32_t{1} << 24;

    static constexpr std::chrono::milliseconds PageProgramTimeout = [] {
        if constexpr(requires { Config::PageProgramTimeout; }) {
            return Kvasir::asDuration(Config::PageProgramTimeout);
        } else {
            return std::chrono::milliseconds{NorFlashDefaults::PageProgramTimeout};
        }
    }();

    static constexpr std::chrono::milliseconds SectorEraseTimeout = [] {
        if constexpr(requires { Config::SectorEraseTimeout; }) {
            return Kvasir::asDuration(Config::SectorEraseTimeout);
        } else {
            return std::chrono::milliseconds{NorFlashDefaults::SectorEraseTimeout};
        }
    }();

    static constexpr std::chrono::milliseconds ChipEraseTimeout = [] {
        if constexpr(requires { Config::ChipEraseTimeout; }) {
            return Kvasir::asDuration(Config::ChipEraseTimeout);
        } else {
            return std::chrono::milliseconds{NorFlashDefaults::ChipEraseTimeout};
        }
    }();

    static constexpr std::chrono::milliseconds StatusPollInterval = [] {
        if constexpr(requires { Config::StatusPollInterval; }) {
            return Kvasir::asDuration(Config::StatusPollInterval);
        } else {
            return std::chrono::milliseconds{NorFlashDefaults::StatusPollInterval};
        }
    }();

    struct Jedec {
        std::uint8_t manufacturer{};
        std::uint8_t type{};
        std::uint8_t capacity{};   ///< log2 of the size in bytes up to 0x19 (0x16 = 4 MB)

        /// The size the capacity byte encodes, 0 when it is not a size. Up to 0x19 (32 MB)
        /// every vendor uses log2; from 512 Mbit the codes diverge: Winbond, Micron and
        /// GigaDevice use 0x20 for 512 Mbit (64 MB), 0x21 for 1 Gbit and 0x22 for 2 Gbit,
        /// Macronix keeps log2 (0x1A for 512 Mbit). The Winbond/Micron reading is taken for
        /// 0x20..0x22; read the SFDP table for the truth.
        [[nodiscard]] constexpr std::uint64_t bytes() const {
            if(capacity >= 0x10 && capacity <= 0x1F) { return std::uint64_t{1} << capacity; }
            if(capacity >= 0x20 && capacity <= 0x22) {
                return (std::uint64_t{64} << 20) << (capacity - 0x20);
            }
            return 0;
        }
    };

    /// What became of the last operation.
    enum class Result : std::uint8_t {
        none,   ///< nothing finished yet
        ok,
        failed,     ///< a frame failed (the bus, the in-flight watchdog) or the driver reset
        timedOut,   ///< the part stayed busy past the operation's timeout
    };

    /// An operation in flight fails rather than leaving its waiter with no completion.
    void resetLogic() {
        if(op_ != Op::none) { finish_(Result::failed); }
        op_    = Op::none;
        state_ = State::idle;
        this->markStarting();
    }

    void idleLogic() {
        auto const now = Clock::now();
        switch(state_) {
        case State::idle: break;
        case State::settle:
            // A part left busy by an operation that failed or timed out: nothing but Read
            // Status is taken until it is done.
            deadline_ = now + ChipEraseTimeout;
            pollAt_   = now;
            state_    = State::settlePoll;
            break;
        case State::settlePoll:
            if(now >= pollAt_ && statusFrame_()) { state_ = State::settleWait; }
            break;
        case State::settleWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    finish_(Result::failed);
                    break;
                }
                status_ = static_cast<std::uint8_t>(this->rx_[1]);
                if((status_ & StatusBusy) == 0) {
                    settled_ = true;
                    state_   = State::start;
                } else if(now >= deadline_) {
                    ++timeouts_;
                    finish_(Result::timedOut);
                } else {
                    pollAt_ = now + StatusPollInterval;
                    state_  = State::settlePoll;
                }
            }
            break;
        case State::wake:
            if(singleFrame_(WakeSequence[wakeStep_])) { state_ = State::wakeWait; }
            break;
        case State::wakeWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    finish_(Result::failed);
                    break;
                }
                // tRES1 after 0xAB, tRST after 0x99: well under the poll interval
                pollAt_ = now + StatusPollInterval;
                state_  = State::wakeGap;
            }
            break;
        case State::wakeGap:
            if(now >= pollAt_) {
                if(++wakeStep_ < WakeSequence.size()) {
                    state_ = State::wake;
                } else {
                    settled_ = false;   // reset: the first operation after it checks BUSY
                    state_   = State::start;
                }
            }
            break;
        case State::start:
            if(startOp_()) {
                state_ = op_ == Op::jedec || op_ == Op::read ? State::frameWait : State::enableWait;
            }
            break;
        case State::enableWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    finish_(Result::failed);
                    break;
                }
                state_ = State::command;
            }
            break;
        case State::command:
            if(commandFrame_()) { state_ = State::commandWait; }
            break;
        case State::commandWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    finish_(Result::failed);
                    break;
                }
                deadline_ = now + timeoutFor_(op_);
                pollAt_   = now + StatusPollInterval;
                state_    = State::poll;
            }
            break;
        case State::poll:
            if(now >= pollAt_ && statusFrame_()) { state_ = State::pollWait; }
            break;
        case State::pollWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    finish_(Result::failed);
                    break;
                }
                status_ = static_cast<std::uint8_t>(this->rx_[1]);
                if((status_ & StatusBusy) == 0) {
                    finish_(Result::ok);
                } else if(now >= deadline_) {
                    ++timeouts_;
                    finish_(Result::timedOut);
                } else {
                    pollAt_ = now + StatusPollInterval;
                    state_  = State::poll;
                }
            }
            break;
        case State::frameWait:
            if(auto const o = this->take(); o != Outcome::running) {
                if(o != Outcome::ok) {
                    finish_(Result::failed);
                    break;
                }
                if(op_ == Op::jedec) {
                    jedec_ = {static_cast<std::uint8_t>(this->rx_[1]),
                              static_cast<std::uint8_t>(this->rx_[2]),
                              static_cast<std::uint8_t>(this->rx_[3])};
                    if(jedec_.manufacturer != 0 && jedec_.manufacturer != 0xFF) {
                        this->markAnswering();
                    } else {
                        this->reportFailure();
                    }
                } else if(op_ == Op::read) {
                    for(std::size_t i = 0; i < length_; ++i) { target_[i] = this->rx_[4 + i]; }
                }
                finish_(Result::ok);
            }
            break;
        }
    }

    // -- the operations ------------------------------------------------------------------

    [[nodiscard]] bool busy() const { return op_ != Op::none; }

    /// The last operation completed and nobody has taken the completion yet. Non-consuming:
    /// stays true until the next operation starts or takeDone() clears it, so it is safe to
    /// read from more than one place in a condition.
    [[nodiscard]] bool finished() const { return finished_; }

    /// The last operation completed; true once per completion (consumes the flag). Two
    /// reads in one condition see it only once: read finished() for the second.
    [[nodiscard]] bool takeDone() {
        if(!finished_) { return false; }
        finished_ = false;
        return true;
    }

    [[nodiscard]] Result result() const { return result_; }

    [[nodiscard]] bool lastOk() const { return result_ == Result::ok; }

    /// Operations whose part stayed busy past their timeout, over the device's life.
    [[nodiscard]] std::uint32_t timeouts() const { return timeouts_; }

    [[nodiscard]] Jedec const& jedec() const { return jedec_; }

    [[nodiscard]] std::uint8_t status() const { return status_; }

    /// Read the JEDEC id; the part is answering() once a manufacturer byte that is neither
    /// 0x00 nor 0xFF has come back. False while another operation runs.
    [[nodiscard]] bool readJedec() { return begin_(Op::jedec, 0, {}, 0); }

    /// Read `into.size()` bytes (up to PageSize) from `address`. The driver keeps `into` and
    /// writes the bytes through it when the frame completes, so it must stay alive and
    /// unread until takeDone() (or finished()) reports the operation done; on any result
    /// other than ok its contents are unchanged. False while another operation runs, for
    /// more than a page, or past the 24-bit address space.
    [[nodiscard]] bool read(std::uint32_t        address,
                            std::span<std::byte> into) {
        return !into.empty() && begin_(Op::read, address, into, into.size());
    }

    /// Erase the 4 KB sector at `address` (on a sector boundary).
    [[nodiscard]] bool eraseSector(std::uint32_t address) {
        return address % SectorSize == 0 && begin_(Op::eraseSector, address, {}, 0);
    }

    /// Erase the whole part; the wait is ChipEraseTimeout at most.
    [[nodiscard]] bool eraseChip() { return begin_(Op::eraseChip, 0, {}, 0); }

    /// Global Block/Sector Unlock (0x98, after a write enable): every individual block lock
    /// cleared, for a W25Q whose WPS bit selects them (W25Q128JV 7.1.5, 8.2.36).
    [[nodiscard]] bool unlockAll() { return begin_(Op::unlock, 0, {}, 0); }

    /// Program up to a page, inside one page. `data` is copied into the frame now and may
    /// go away once this returns true.
    [[nodiscard]] bool program(std::uint32_t              address,
                               std::span<std::byte const> data) {
        if(data.empty() || data.size() > PageSize || (address % PageSize) + data.size() > PageSize)
        {
            return false;
        }
        if(op_ != Op::none || !inRange_(address, data.size())) { return false; }
        // The data sits behind the command and address bytes; the write-enable frame
        // before it uses tx_[0] only.
        for(std::size_t i = 0; i < data.size(); ++i) { this->tx_[4 + i] = data[i]; }
        return begin_(Op::program, address, {}, data.size());
    }

private:
    enum class Op : std::uint8_t { none, jedec, read, eraseSector, eraseChip, program, unlock };

    enum class State : std::uint8_t {
        idle,
        settle,
        settlePoll,
        settleWait,
        wake,
        wakeWait,
        wakeGap,
        start,
        enableWait,
        command,
        commandWait,
        poll,
        pollWait,
        frameWait,
    };

    static constexpr std::uint8_t StatusBusy = 0x01;

    /// readJedec()'s frames before the id: out of deep power-down, then the reset pair.
    static constexpr std::array<std::uint8_t, 3> WakeSequence{CmdReleasePowerDn,
                                                              CmdEnableReset,
                                                              CmdReset};

    [[nodiscard]] static constexpr bool inRange_(std::uint32_t address,
                                                 std::size_t   length) {
        return address < AddressSpace && length <= AddressSpace - address;
    }

    [[nodiscard]] static constexpr std::chrono::milliseconds timeoutFor_(Op op) {
        switch(op) {
        case Op::program:
        case Op::unlock:      return PageProgramTimeout;
        case Op::eraseSector: return SectorEraseTimeout;
        case Op::eraseChip:   return ChipEraseTimeout;
        default:              return std::chrono::milliseconds{0};
        }
    }

    bool begin_(Op                   op,
                std::uint32_t        address,
                std::span<std::byte> target,
                std::size_t          length) {
        if(op_ != Op::none) { return false; }
        if(length > PageSize || !inRange_(address, length)) { return false; }
        op_       = op;
        address_  = address;
        length_   = length;
        target_   = target;
        finished_ = false;
        result_   = Result::none;
        wakeStep_ = 0;
        state_    = op == Op::jedec ? State::wake : settled_ ? State::start : State::settle;
        return true;
    }

    bool singleFrame_(std::uint8_t command) {
        this->tx_[0] = std::byte{command};
        return this->submit(std::span<std::byte const>{this->tx_}.first(1), {});
    }

    void putAddress_(std::size_t at) {
        this->tx_[at]     = static_cast<std::byte>(address_ >> 16);
        this->tx_[at + 1] = static_cast<std::byte>(address_ >> 8);
        this->tx_[at + 2] = static_cast<std::byte>(address_);
    }

    /// The first frame: the whole operation for a read or the id, write-enable otherwise.
    bool startOp_() {
        switch(op_) {
        case Op::jedec:
            this->tx_[0] = std::byte{CmdJedecId};
            this->tx_[1] = this->tx_[2] = this->tx_[3] = std::byte{0};
            return this->submit(std::span<std::byte const>{this->tx_}.first(4),
                                std::span<std::byte>{this->rx_}.first(4));
        case Op::read:
            this->tx_[0] = std::byte{CmdRead};
            putAddress_(1);
            for(std::size_t i = 0; i < length_; ++i) { this->tx_[4 + i] = std::byte{0}; }
            return this->submit(std::span<std::byte const>{this->tx_}.first(4 + length_),
                                std::span<std::byte>{this->rx_}.first(4 + length_));
        case Op::eraseSector:
        case Op::eraseChip:
        case Op::program:
        case Op::unlock:
            this->tx_[0] = std::byte{CmdWriteEnable};
            return this->submit(std::span<std::byte const>{this->tx_}.first(1), {});
        case Op::none: break;
        }
        return false;
    }

    bool commandFrame_() {
        switch(op_) {
        case Op::eraseSector:
            this->tx_[0] = std::byte{CmdSectorErase};
            putAddress_(1);
            return this->submit(std::span<std::byte const>{this->tx_}.first(4), {});
        case Op::eraseChip: return singleFrame_(CmdChipErase);
        case Op::unlock:    return singleFrame_(CmdGlobalUnlock);
        case Op::program:
            this->tx_[0] = std::byte{CmdPageProgram};
            putAddress_(1);   // the data is in tx_[4..] since program()
            return this->submit(std::span<std::byte const>{this->tx_}.first(4 + length_), {});
        default: return false;
        }
    }

    bool statusFrame_() {
        this->tx_[0] = std::byte{CmdReadStatus};
        this->tx_[1] = std::byte{0};
        return this->submit(std::span<std::byte const>{this->tx_}.first(2),
                            std::span<std::byte>{this->rx_}.first(2));
    }

    void finish_(Result r) {
        if(r != Result::ok) { settled_ = false; }
        result_   = r;
        finished_ = true;
        op_       = Op::none;
        state_    = State::idle;
    }

    Op                         op_{Op::none};
    State                      state_{State::idle};
    Result                     result_{Result::none};
    std::uint32_t              address_{};
    std::size_t                length_{};
    std::span<std::byte>       target_{};
    typename Clock::time_point pollAt_{};
    typename Clock::time_point deadline_{};
    Jedec                      jedec_{};
    std::uint8_t               status_{};
    bool                       finished_{false};
    bool                       settled_{false};   ///< the part is known not to be busy
    std::size_t                wakeStep_{};
    std::uint32_t              timeouts_{};
};

}   // namespace Kvasir::SPI
