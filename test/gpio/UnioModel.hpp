#pragma once

/// A UNI/O line for the host tests of the parts bit-banged on one pin, and a Microchip
/// 11AA02E48 on it. Data sheet: Microchip DS20002122D, "11AA02E48/11AA02E64".
///
/// The line (`Line`, pin `Scio`) is the FakeSpi.hpp idiom with the one thing UNI/O is about: the
/// master's pin is either released (`makeInput`) or driven low (`makeOutput` + `clear`), the
/// level is the wired-AND of the master, the model and the pull-up, and the model sees every
/// operation with the simulated time.
///
/// The model decodes the master's bits from the edge times, learns the bit period T_E from the
/// start header's 0x55 as the part does (DS 3.2, 3.8), answers a READ with SAKs and the bytes of
/// its array, Manchester coded at that T_E, and stops driving after the last SAK. What the master
/// does wrong it writes into `errors` with the data sheet rule it broke.
#include <array>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string>
#include <support/FakeClock.hpp>
#include <vector>

namespace Kvasir::Test::Gpio {

/// Simulated time only: a delay winds the clock (rule: no test reads the real clock).
struct UnioClock : Kvasir::Test::FakeClock {
    template<typename Rep,
             typename Period>
    static void delay(std::chrono::duration<Rep,
                                            Period> d) {
        advance(d);
    }
};

using Us = std::chrono::microseconds;

class UnioModel {
public:
    // -- knobs ---------------------------------------------------------------------------------

    /// false: no part on the line. The model still watches the master (headers counts), but it
    /// never drives.
    bool present{true};
    /// The six bytes at 0xFA-0xFF (DS 7.2). The default is the data sheet's own example, with
    /// one of Microchip's OUIs (DS 7.2, Figure 7-2: 00-04-A3-12-34-56).
    std::array<std::uint8_t, 6> eui{0x00, 0x04, 0xA3, 0x12, 0x34, 0x56};

    /// A data bit the part sends without its middle edge (both halves low): byte index of the
    /// read, bit 7 (first) .. 0. byte < 0: none.
    struct Corrupt {
        int byte{-1};
        int bit{};
    } corrupt{};

    /// The data byte (index of the read) after which the part gives no SAK, and goes on as if
    /// it had: only the master's SAK check can tell. < 0: none.
    int withholdSak{-1};
    /// The first n start headers the part loses sync after the device address: NoSAK, then
    /// Idle until the next standby pulse (DS 3.4 "if the slave becomes out of sync", 3.7).
    int failFirstReads{};

    // -- what it saw ---------------------------------------------------------------------------

    int                      headers{};      // start headers after a valid standby pulse
    int                      commands{};     // READs ended by NoMAK + SAK (DS 3.6)
    int                      contention{};   // master low in a slot the part owns
    std::uint32_t            teUs{};         // T_E learned from the last start header
    std::uint16_t            address{};      // word address of the last READ
    std::vector<bool>        maks;           // per data byte of the last READ: MAK (true) / NoMAK
    std::vector<std::string> errors;         // rules the master broke
    std::vector<int>         warningsAtOp;   // Log::warnings at every pin operation

    // -- the line's calls ------------------------------------------------------------------------

    /// The master's side of the line changed: released (high through the pull-up) or low.
    void masterChanged(Us   now,
                       bool released) {
        advance(now);
        history_.push_back({now, released});
        if(released) {
            lastRise_ = now;
            haveRise_ = true;
            if(state_ == State::thdr) {
                // DS Table 1-2 param 11: THDR >= 5 us.
                if(now - headerFall_ < Us{5}) {
                    error("THDR shorter than 5 us (Table 1-2 param 11)");
                }
                headerStart_ = now;
                edges_.clear();
                state_ = State::header;
            } else if(state_ == State::header) {
                headerEdge(now, true);
            }
            return;
        }
        // A fall after the master held the line high for TSTBY is the first low pulse of a
        // start header (DS 3.1: the standby pulse, Table 1-2 param 9: TSTBY >= 600 us). The
        // rising edge before it is the low-to-high transition the POR note asks for.
        if(haveRise_ && now - lastRise_ >= Us{600}) {
            ++headers;
            beginCommand();
            headerFall_ = now;
            state_      = State::thdr;
            return;
        }
        if(state_ == State::header) { headerEdge(now, false); }
    }

    /// Every pin operation: the place a test looks at what the driver did between two of them.
    void op(Us now) {
        advance(now);
        warningsAtOp.push_back(Kvasir::Test::Log::warnings);
    }

    /// Whether the part pulls SCIO low at `now`.
    [[nodiscard]] bool pullsLow(Us now) {
        advance(now);
        if(!present || state_ != State::slots) { return false; }
        Slot const s = slotAt(slot_);
        if(s.owner != Owner::part) { return false; }
        auto const into = now - slotStart(slot_);
        if(into < Us{0}) { return false; }
        bool const firstHalf = into * 2 < te_;
        switch(s.drive) {
        case Drive::none: return false;
        case Drive::one:  return firstHalf;    // low, then high: a '1' (SAK, data 1)
        case Drive::zero: return !firstHalf;   // high, then low: a '0'
        case Drive::flat: return true;         // no middle edge
        }
        return false;
    }

    /// Run the decoder up to `now` (after the last operation of a read).
    void settle(Us now) { advance(now); }

private:
    enum class State : std::uint8_t { idle, thdr, header, slots };
    enum class Owner : std::uint8_t { master, part };
    enum class Drive : std::uint8_t { none, one, zero, flat };
    enum class Kind : std::uint8_t {
        headerMak,     // master: must be MAK (DS 3.3 note)
        headerNoSak,   // part: silent (DS 3.3 note "a NoSAK will occur after the start header")
        masterBit,     // master: device address, command, word address
        masterAck,     // master: MAK / NoMAK after a byte
        sak,           // part: SAK or NoSAK
        partBit,       // part: a data bit
    };

    struct Slot {
        Kind  kind{};
        Owner owner{};
        Drive drive{Drive::none};
    };

    struct Edge {
        Us   at{};
        bool released{};
    };

    std::vector<Edge> history_{
      {Us{0}, true}
    };   // the pin starts as an input
    std::vector<Us> edges_;
    State           state_{State::idle};
    Us              lastRise_{};
    bool            haveRise_{};
    Us              headerFall_{};
    Us              headerStart_{};
    Us              base_{};   // start of the header's first bit
    Us              te_{};
    std::uint32_t   slot_{};   // slots after the header's 8 bits

    // the command being decoded
    int           masterByte_{};   // 0 address, 1 command, 2 address MSB, 3 address LSB
    std::uint8_t  shift_{};
    int           bit_{};            // bits of the current byte so far
    bool          sakNext_{};        // the SAK slot answers with a SAK
    bool          endAfterSak_{};    // NoMAK + SAK: the command ends there
    bool          startSending_{};   // the word address is in: data from the next slot on
    bool          sending_{};        // the part sends data bytes now
    std::uint8_t  out_{};            // the data byte on the wire
    int           outIndex_{};       // data bytes of this READ so far
    std::uint16_t pointer_{};

    void beginCommand() {
        maks.clear();
        masterByte_   = 0;
        shift_        = 0;
        bit_          = 0;
        sakNext_      = false;
        endAfterSak_  = false;
        startSending_ = false;
        sending_      = false;
        outIndex_     = 0;
        slot_         = 0;
    }

    void error(std::string what) {
        errors.push_back(std::move(what));
        state_ = State::idle;   // DS 3.7: ignore everything until a standby pulse
    }

    void headerEdge(Us   now,
                    bool released) {
        // 0x55 after THDR: a fall in the middle of each '0', a rise in the middle of each '1'
        // (DS 3.2, Figure 3-2: the header starts low and its first bit is '0'; DS 8.1: every
        // bit is a transition in the middle of its period), and none between two bits.
        bool const expect = (edges_.size() % 2) == 1;
        if(released != expect) {
            error("start header is not 0x55");
            return;
        }
        edges_.push_back(now);
        if(edges_.size() < 8) { return; }
        te_  = (edges_[7] - edges_[0]) / 7;
        teUs = static_cast<std::uint32_t>(te_.count());
        // DS Table 1-2 param 2: T_E 10 .. 100 us.
        if(te_ < Us{10} || te_ > Us{100}) {
            error("T_E outside 10 .. 100 us (Table 1-2 param 2)");
            return;
        }
        // Param 3, TIJIT +-0.06 UI: every edge where the learned period puts it.
        auto const tol = te_ * 6 / 100;
        for(std::size_t i = 1; i < 8; ++i) {
            auto const d = edges_[i] - edges_[i - 1] - te_;
            if(d > tol || -d > tol) {
                error("start header edge jitter over 0.06 UI (Table 1-2 param 3)");
                return;
            }
        }
        base_          = edges_[0] - te_ / 2;
        auto const off = headerStart_ - base_;
        if(off > tol || -off > tol) {
            error("start header's first bit does not start at the end of THDR");
            return;
        }
        state_ = State::slots;
        slot_  = 0;
    }

    [[nodiscard]] Us slotStart(std::uint32_t n) const {
        return base_ + te_ * static_cast<Us::rep>(8 + n);
    }

    [[nodiscard]] Slot slotAt(std::uint32_t n) const {
        if(n == 0) { return {Kind::headerMak, Owner::master}; }
        if(n == 1) { return {Kind::headerNoSak, Owner::part}; }
        // after the header: bytes of 10 slots each, 8 bits + the master's ack + the part's SAK
        auto const k = (n - 2) % 10;
        if(k == 8) { return {Kind::masterAck, Owner::master}; }
        if(k == 9) {
            bool const give = sakNext_ && (!sending_ || withholdSak != outIndex_);
            return {Kind::sak, Owner::part, give ? Drive::one : Drive::none};
        }
        if(!sending_) { return {Kind::masterBit, Owner::master}; }
        int const bitNo = 7 - static_cast<int>(k);
        if(corrupt.byte == outIndex_ && corrupt.bit == bitNo) {
            return {Kind::partBit, Owner::part, Drive::flat};
        }
        bool const one = ((out_ >> bitNo) & 1U) != 0;
        return {Kind::partBit, Owner::part, one ? Drive::one : Drive::zero};
    }

    [[nodiscard]] bool masterReleasedAt(Us t) const {
        bool level = true;
        for(auto const& e : history_) {
            if(e.at > t) { break; }
            level = e.released;
        }
        return level;
    }

    [[nodiscard]] bool masterReleasedThrough(Us from,
                                             Us to) const {
        if(!masterReleasedAt(from)) { return false; }
        for(auto const& e : history_) {
            if(e.at > from && e.at < to && !e.released) { return false; }
        }
        return true;
    }

    /// The master's bit in slot n: -1 without a middle edge.
    [[nodiscard]] int masterBitIn(std::uint32_t n) const {
        Us const   s     = slotStart(n);
        bool const first = masterReleasedAt(s + te_ / 4);
        bool const last  = masterReleasedAt(s + te_ * 3 / 4);
        if(first == last) { return -1; }
        return last ? 1 : 0;
    }

    void advance(Us now) {
        while(state_ == State::slots && slotStart(slot_ + 1) <= now) {
            finishSlot(slot_);
            if(state_ == State::slots) { ++slot_; }
        }
    }

    void finishSlot(std::uint32_t n) {
        Slot const s = slotAt(n);
        if(s.owner == Owner::part) {
            if(!masterReleasedThrough(slotStart(n), slotStart(n + 1))) {
                ++contention;
                error("master drove SCIO low in a slot the part owns (DS 3.3, 3.5)");
                return;
            }
        }
        switch(s.kind) {
        case Kind::headerMak:
            if(masterBitIn(n) != 1) { error("no MAK after the start header (DS 3.3 note)"); }
            return;
        case Kind::headerNoSak:
            if(!present) { state_ = State::idle; }   // nobody there to take the command
            return;
        case Kind::masterBit:
            {
                int const b = masterBitIn(n);
                if(b < 0) {
                    error("a master bit without its middle edge (DS 8.1)");
                    return;
                }
                shift_
                  = static_cast<std::uint8_t>((unsigned{shift_} << 1U) | static_cast<unsigned>(b));
                ++bit_;
                return;
            }
        case Kind::partBit: return;
        case Kind::masterAck:
            {
                int const ack = masterBitIn(n);
                if(ack < 0) {
                    error("an acknowledge bit without its middle edge (DS 8.1)");
                    return;
                }
                if(sending_) {
                    maks.push_back(ack == 1);
                    sakNext_     = true;
                    endAfterSak_ = ack == 0;   // NoMAK ends the READ (DS 4.1)
                    return;
                }
                masterAcked(ack == 1);
                return;
            }
        case Kind::sak:
            if(!sakNext_) {
                state_ = State::idle;
                return;
            }
            if(endAfterSak_) {
                ++commands;
                state_ = State::idle;   // DS 3.6: NoMAK + SAK, back to standby
                return;
            }
            if(startSending_) {
                startSending_ = false;
                sending_      = true;
                out_          = byteAt(pointer_);
            } else if(sending_) {
                nextOut();
            }
            return;
        }
    }

    /// A master byte and its acknowledge are in: what the part answers in the SAK slot.
    void masterAcked(bool mak) {
        std::uint8_t const byte = shift_;
        shift_                  = 0;
        bit_                    = 0;
        sakNext_                = false;
        if(!mak) {
            // DS 3.4: a NoMAK that ends a command early gets a NoSAK.
            error("NoMAK before the READ's data");
            return;
        }
        switch(masterByte_++) {
        case 0:
            if(byte != 0xA0) { return; }   // DS 3.4: family 1010, device code 0000; else NoSAK
            if(headers <= failFirstReads) { return; }
            sakNext_ = true;
            return;
        case 1:
            if(byte != 0x03) {
                error("only READ (0x03) is modelled (DS Table 4-1)");
                return;
            }
            sakNext_ = true;
            return;
        case 2:
            pointer_ = static_cast<std::uint16_t>(byte << 8U);
            sakNext_ = true;
            return;
        case 3:
            pointer_      = static_cast<std::uint16_t>(pointer_ | byte);
            address       = pointer_;
            sakNext_      = true;
            startSending_ = true;
            return;
        default: return;
        }
    }

    void nextOut() {
        ++outIndex_;
        pointer_
          = static_cast<std::uint16_t>((pointer_ + 1U) & 0xFFU);   // DS 4.1: rolls over to 0x00
        out_ = byteAt(pointer_);
    }

    /// The array as the part ships it, as far as the model knows it: the EUI-48 in 0xFA-0xFF
    /// (DS 7.2), 0xFF elsewhere.
    [[nodiscard]] std::uint8_t byteAt(std::uint16_t a) const {
        auto const i = a & 0xFFU;
        return i >= 0xFA ? eui[i - 0xFA] : std::uint8_t{0xFF};
    }
};

/// The one UNI/O pin of the tests.
struct Scio {};

struct PinOp {
    enum class Kind : std::uint8_t { input, output, set, clear, read };
    Kind kind{};
};

constexpr PinOp makeInput(Scio) { return {PinOp::Kind::input}; }

constexpr PinOp makeOutput(Scio) { return {PinOp::Kind::output}; }

constexpr PinOp set(Scio) { return {PinOp::Kind::set}; }

constexpr PinOp clear(Scio) { return {PinOp::Kind::clear}; }

constexpr PinOp read(Scio) { return {PinOp::Kind::read}; }

/// The master's pin and the model on the line.
struct Line {
    static inline UnioModel* model{};
    static inline bool       output{};   // the pin drives (else it is an input: released)
    static inline bool       latch{true};
    static inline int        ops{};

    static void reset() {
        output = false;
        latch  = true;
        ops    = 0;
    }

    [[nodiscard]] static bool released() { return !output || latch; }

    [[nodiscard]] static Us now() { return UnioClock::now().time_since_epoch(); }
};

/// ADL target of the driver's `apply(makeInput(Pin{}))`, `apply(makeOutput(Pin{}), clear(Pin{}))`
/// and `apply(read(Pin{}))`: the operations in order, the model told of every change of the
/// master's side, and the level read back as the wired-AND of master, part and pull-up.
template<std::same_as<PinOp>... Ops>
bool apply(PinOp first,
           Ops... rest) {
    bool       level  = false;
    bool const before = Line::released();
    for(PinOp const o : {first, rest...}) {
        switch(o.kind) {
        case PinOp::Kind::input:  Line::output = false; break;
        case PinOp::Kind::output: Line::output = true; break;
        case PinOp::Kind::set:    Line::latch = true; break;
        case PinOp::Kind::clear:  Line::latch = false; break;
        case PinOp::Kind::read:
            level
              = Line::released() && (Line::model == nullptr || !Line::model->pullsLow(Line::now()));
            break;
        }
    }
    ++Line::ops;
    if(Line::model != nullptr) {
        if(Line::released() != before) {
            Line::model->masterChanged(Line::now(), Line::released());
        }
        Line::model->op(Line::now());
    }
    return level;
}

}   // namespace Kvasir::Test::Gpio
