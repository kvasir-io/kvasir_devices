#pragma once

#include "../Duration.hpp"
#include "../Log.hpp"
#include "Catalogue.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

/// One I2C bus scan, on the queued bus (Kvasir::I2C::I2CBehaviorQueued), polled from the
/// loop. `Scanner<I2c, Clock, Hints, Probe>` scans the whole range, a sub-range or an
/// explicit list, with either probe policy, and reports a bus fault apart from a NAK.
///
/// It goes round the devices: its probes are submitted past any Bus and any switch. A Bus whose
/// parts are talking wants BusScan.hpp, which takes part in it.
namespace Kvasir::I2C {

/// What one probe transaction looks like. `AddressProbe` is what `i2cdetect` sends by
/// default: an address-only read of a single byte. A device that is there ACKs its address
/// and returns whatever its pointer is at -- a status byte, the last reading -- and none of
/// it changes the device. An empty slot NAKs and the controller aborts.
struct AddressProbe {
    static constexpr std::string_view Name = "read";

    template<typename Request>
    static void fill(Request&             req,
                     std::span<std::byte> tx,
                     std::span<std::byte> rx) {
        static_cast<void>(tx);
        req.receiveData = rx.first(1);
    }
};

/// Write a register address, then read a byte back. Some parts NAK a bare read but answer a
/// pointer set, and on a bus of known chips this is the more reliable probe. It is *not*
/// side-effect free: it moves the device's register pointer.
template<std::uint8_t Register = 0x00>
struct RegisterProbe {
    static constexpr std::string_view Name = "register read";

    template<typename Request>
    static void fill(Request&             req,
                     std::span<std::byte> tx,
                     std::span<std::byte> rx) {
        tx[0]           = std::byte{Register};
        req.sendData    = std::span<std::byte const>{tx.first(1)};
        req.receiveData = rx.first(1);
    }
};

/// No catalogue: the scan reports addresses and nothing about what may live there.
struct NoHints {
    [[nodiscard]] static constexpr std::string_view hint(std::uint8_t) { return {}; }
};

/// The timing a scan runs with. A Scanner's or BusScan's Config may redeclare any of them,
/// and probeTimeout(ms) changes the first at run time.
struct ScanDefaults {
    /// How long one probe may be outstanding. The bus answers a probe within its own
    /// transfer timeout, tens of milliseconds even behind a full queue; this is for a
    /// request the bus lost -- which it must not, but a scan that waited on one for ever
    /// is a scan that never finishes.
    static constexpr auto ProbeTimeout = std::chrono::seconds{1};
    /// BusScan: how long the switches may take to be claimed and set for a segment before one
    /// that is not set is taken to be not there, and its channels are left out.
    static constexpr auto SelectTimeout = std::chrono::milliseconds{500};
};

namespace detail {
    template<typename Cfg>
    constexpr std::chrono::milliseconds probeTimeoutOf() {
        if constexpr(requires { Cfg::ProbeTimeout; }) {
            return Kvasir::asDuration(Cfg::ProbeTimeout);
        } else {
            return Kvasir::asDuration(ScanDefaults::ProbeTimeout);
        }
    }

    template<typename Cfg>
    constexpr std::chrono::milliseconds selectTimeoutOf() {
        if constexpr(requires { Cfg::SelectTimeout; }) {
            return Kvasir::asDuration(Cfg::SelectTimeout);
        } else {
            return Kvasir::asDuration(ScanDefaults::SelectTimeout);
        }
    }
}   // namespace detail

/// One probe on the queued bus and its answer: what a scan is made of (Scanner, BusScan).
template<typename I2c,
         typename Clock,
         typename Probe  = AddressProbe,
         typename Config = ScanDefaults>
class ProbeSlot {
public:
    /// How long one probe may be outstanding (ScanDefaults::ProbeTimeout), until
    /// timeout(ms) says otherwise.
    static constexpr auto Timeout = detail::probeTimeoutOf<Config>();

    /// From now on, give up on a probe after `t`.
    void timeout(std::chrono::milliseconds t) { timeout_ = t; }

    [[nodiscard]] std::chrono::milliseconds timeout() const { return timeout_; }

    enum class Outcome : std::uint8_t {
        waiting,   ///< on the wire
        ack,       ///< something answers there
        nak,       ///< nothing does
        fault,     ///< a bus fault, not a NAK: it says nothing about the address
        lost,      ///< no answer within Timeout, given up on
    };

    /// A probe of `address` on the bus. False when the queue is full: try again next turn.
    bool submit(std::uint8_t address) {
        ready_.store(false, std::memory_order_relaxed);
        typename I2c::Request req{};
        req.address = address;
        Probe::template fill<typename I2c::Request>(req, tx_, rx_);
        // The probe carries its generation: an answer to a probe that was given up on
        // (Timeout) arrives with an old one and is not taken for the current probe's.
        auto const gen = generation_.load(std::memory_order_relaxed);
        req.callback   = [this, gen](typename I2c::Result r) {
            if(gen != generation_.load(std::memory_order_relaxed)) { return; }
            result_ = r;
            ready_.store(true, std::memory_order_release);
        };
        if(!I2c::submit(req)) { return false; }
        sentAt_ = Clock::now();
        return true;
    }

    /// What became of the probe; `waiting` while it is on the wire.
    [[nodiscard]] Outcome poll() {
        if(ready_.load(std::memory_order_acquire)) {
            auto const r = result_.load(std::memory_order_relaxed);
            if(r == I2c::Result::succeeded) { return Outcome::ack; }
            return r == I2c::Result::notAcknowledged ? Outcome::nak : Outcome::fault;
        }
        if(Clock::now() - sentAt_ > timeout_) {
            generation_.fetch_add(1, std::memory_order_relaxed);
            return Outcome::lost;
        }
        return Outcome::waiting;
    }

private:
    std::array<std::byte, 1>          tx_{};
    std::array<std::byte, 1>          rx_{};
    std::chrono::milliseconds         timeout_{Timeout};
    typename Clock::time_point        sentAt_{};
    std::atomic<std::uint32_t>        generation_{};
    std::atomic<bool>                 ready_{false};
    std::atomic<typename I2c::Result> result_{};
};

/// `Hints` is a Kvasir::I2C::Catalogue<Chips...> (or NoHints); it says what *may* answer at
/// an address, and the drivers then identify what does.
///
///     using Scan = Kvasir::I2C::Scanner<I2c1, Clock, Catalogue>;
///     Scan scan{};
///     scan.start();                      // 0x08..0x77
///     scan.start(0x40, 0x4F);            // one range
///     scan.start(std::array<std::uint8_t, 2>{0x3C, 0x3D});   // just these
///     ...
///     scan.handler();                    // once per loop turn
///     if(scan.done() && scan.found(0x48)) { ... }
template<typename I2c,
         typename Clock,
         typename Hints  = NoHints,
         typename Probe  = AddressProbe,
         typename Config = ScanDefaults>
struct Scanner {
    /// 0x00..0x07 and 0x78..0x7F are reserved by the I2C specification and are never
    /// probed, whatever range is asked for.
    static constexpr std::uint8_t First = 0x08;
    static constexpr std::uint8_t Last  = 0x77;

    static constexpr std::size_t Addresses = 0x80;

    using Slot = ProbeSlot<I2c, Clock, Probe, Config>;

    /// How long one probe may be outstanding (ProbeSlot); one given up on is counted as a bus
    /// fault at that address.
    static constexpr auto ProbeTimeout = Slot::Timeout;

    /// Change it at run time, for the scans from now on.
    void probeTimeout(std::chrono::milliseconds t) { slot_.timeout(t); }

    /// Addresses no scan probes from now on: a part whose read has a side effect, or one that
    /// is known to be there and busy. A later start() leaves them out whatever it asks for.
    void exclude(std::span<std::uint8_t const> addresses) {
        for(auto const a : addresses) {
            if(a < Addresses) { set_(excluded_, a); }
        }
    }

    /// The whole non-reserved range.
    void start() { start(First, Last); }

    /// One inclusive range, clamped to the non-reserved addresses.
    void start(std::uint8_t from,
               std::uint8_t to) {
        if(state_ != State::idle) { return; }
        clear_();
        for(unsigned a = from < First ? First : from; a <= (to > Last ? Last : to); ++a) {
            if(!get_(excluded_, static_cast<std::uint8_t>(a))) {
                set_(pending_, static_cast<std::uint8_t>(a));
            }
        }
        begin_();
    }

    /// Only these addresses, in ascending order whatever order they are given in.
    void start(std::span<std::uint8_t const> addresses) {
        if(state_ != State::idle) { return; }
        clear_();
        for(auto const a : addresses) {
            if(a >= First && a <= Last && !get_(excluded_, a)) { set_(pending_, a); }
        }
        begin_();
    }

    [[nodiscard]] bool done() const { return state_ == State::idle; }

    /// At least one scan has finished since boot.
    [[nodiscard]] bool scanned() const { return scans_ != 0; }

    [[nodiscard]] std::uint32_t scans() const { return scans_; }

    [[nodiscard]] bool found(std::uint8_t address) const {
        return address < Addresses && get_(found_, address);
    }

    /// A bus fault, rather than a clean NAK, was seen at this address: a timeout, lost
    /// arbitration or a stuck line. It says nothing about whether a device is there.
    [[nodiscard]] bool faulted(std::uint8_t address) const {
        return address < Addresses && get_(faults_, address);
    }

    [[nodiscard]] std::size_t count() const { return count_; }

    [[nodiscard]] std::size_t faults() const { return faultCount_; }

    /// What the catalogue says may answer at `address`.
    [[nodiscard]] static constexpr std::string_view hint(std::uint8_t address) {
        return Hints::hint(address);
    }

    /// Call f(address, hint) for every address that answered.
    template<typename F>
    void forEachFound(F&& f) const {
        for(unsigned a = First; a <= Last; ++a) {
            auto const address = static_cast<std::uint8_t>(a);
            if(found(address)) { f(address, hint(address)); }
        }
    }

    void handler() {
        switch(state_) {
        case State::idle: break;

        case State::probe:
            // A full queue is not an error: the same address is tried again next turn.
            if(slot_.submit(addr_)) { state_ = State::wait; }
            break;

        case State::wait:
            switch(slot_.poll()) {
            case Slot::Outcome::waiting: return;
            case Slot::Outcome::ack:
                set_(found_, addr_);
                ++count_;
                UC_LOG_I("i2c scan: {:#04x} ACK  {}", addr_, hint(addr_));
                break;
            case Slot::Outcome::nak: break;
            case Slot::Outcome::fault:
                // Not a NAK: a bus fault. Worth a line, because a chain with one dead part
                // looks exactly like this.
                fault_("bus fault (not a NAK)");
                break;
            case Slot::Outcome::lost: fault_("no answer from the bus -- counted as a fault"); break;
            }
            advance_();
            break;
        }
    }

private:
    enum class State : std::uint8_t { idle, probe, wait };

    using Word                         = std::uint32_t;
    static constexpr std::size_t Words = Addresses / 32;

    static constexpr void set_(std::array<Word,
                                          Words>& bitmap,
                               std::uint8_t       address) {
        bitmap[address / 32] |= Word{1} << (address % 32);
    }

    static constexpr bool get_(std::array<Word,
                                          Words> const& bitmap,
                               std::uint8_t             address) {
        return (bitmap[address / 32] & (Word{1} << (address % 32))) != 0;
    }

    void clear_() {
        pending_.fill(0);
        found_.fill(0);
        faults_.fill(0);
        count_      = 0;
        faultCount_ = 0;
    }

    void begin_() {
        addr_ = First;
        if(!seek_()) { return; }   // nothing asked for
        state_ = State::probe;
    }

    /// Move addr_ to the next pending address; false when there are none left.
    bool seek_() {
        for(unsigned a = addr_; a <= Last; ++a) {
            if(get_(pending_, static_cast<std::uint8_t>(a))) {
                addr_ = static_cast<std::uint8_t>(a);
                return true;
            }
        }
        return false;
    }

    void fault_([[maybe_unused]] std::string_view why) {
        set_(faults_, addr_);
        ++faultCount_;
        UC_LOG_W("i2c scan: {:#04x} {}", addr_, why);
    }

    void advance_() {
        if(addr_ == Last) {
            finish_();
            return;
        }
        ++addr_;
        if(seek_()) {
            state_ = State::probe;
        } else {
            finish_();
        }
    }

    void finish_() {
        ++scans_;
        UC_LOG_I("i2c scan #{}: {} device(s), {} bus fault(s)", scans_, count_, faultCount_);
        state_ = State::idle;
    }

    State                   state_{State::idle};
    std::uint8_t            addr_{First};
    Slot                    slot_{};
    std::array<Word, Words> excluded_{};
    std::array<Word, Words> pending_{};
    std::array<Word, Words> found_{};
    std::array<Word, Words> faults_{};
    std::size_t             count_{};
    std::size_t             faultCount_{};
    std::uint32_t           scans_{};
};

}   // namespace Kvasir::I2C
