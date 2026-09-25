#pragma once

#include "../Duration.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>

/// One scripted bus transaction: "write this register", "send this command", "read n bytes
/// from this register into my buffer", "wait so long". A chip description is a list of these
/// and one engine (Device.hpp) walks it, so a chip is data and not a state machine.
namespace Kvasir::I2C {

/// The argument structs of the Step factories take their fields by name only: each starts with
/// a member a positional value cannot initialize, so `Step::read({0x07, 3, 3})` does not
/// compile and `Step::read({.reg = 0x07, .count = 3, .offset = 3})` says which number is which.
/// A field without a default is Required: leaving it out is a compile error, not a zero.
namespace StepArgs {
    struct NamedOnly {
        explicit constexpr NamedOnly() = default;
    };

    template<typename T>
    struct Required {
        T value;

        constexpr Required(T v)   // NOLINT(google-explicit-constructor): `.count = 3`
          : value{v} {}

        constexpr operator T() const {   // NOLINT(google-explicit-constructor)
            return value;
        }
    };
}   // namespace StepArgs

struct Step {
    enum class Kind : std::uint8_t {
        write,        ///< [register] + payload, payload inline or from the group's buffer
        read,         ///< [register], then `count` bytes into the buffer at `offset` (or as many
                      ///< as an earlier read said, at most `count`: readCounted)
        wait,         ///< no transaction: `delay`
        check,        ///< no transaction: the group's ready(Bytes) decides whether to go on or to
                      ///< run the sequence again after `delay` (a data-ready flag)
        stopUnless,   ///< no transaction: the group's ready(Bytes) decides whether the rest
                      ///< of the script runs; false decodes what the steps before it read
        identify,     ///< Init only, no transaction: the chip's setup() over what was read so
                      ///< far; false ends the bring-up there, as not this chip
        oracle,       ///< Init only, no transaction, and never written by hand: the engine puts
                      ///< it behind the identity reads it makes of `Chip::Identity` (Device.hpp);
                      ///< a register that is not what the data sheet says ends the bring-up
    };

    static constexpr std::size_t InlineBytes = 8;

    Kind          kind{};
    bool          hasRegister{};     ///< the register prefix is sent (Chip::RegisterBytes wide)
    bool          fromBuffer{};      ///< write: the payload is buffer[offset, offset + count)
    bool          regFromBuffer{};   ///< the register bytes are buffer[reg, reg + RegisterBytes)
    bool          counted{};         ///< read: its length is in the buffer (readCounted)
    std::uint16_t reg{};
    std::uint8_t  count{};    ///< payload bytes (write), bytes read, or a counted read's most
    std::uint8_t  offset{};   ///< in the group's buffer: where a read lands / a payload is taken
    std::uint8_t  countOffset{};   ///< counted read: where in the buffer its count is
    std::uint8_t  countBytes{};    ///< counted read: 1 or 2 (big-endian)
    bool          mayNak{};   ///< write: a NAK is what the part does here, and the script goes on
    std::chrono::milliseconds delay{};   ///< after the transaction completes

    std::array<std::uint8_t, InlineBytes> bytes{};

    /// Register write with an inline payload of up to eight bytes (may be empty: the register
    /// address alone, which some chips take as a pointer set).
    ///     Step::write({.reg = 0x81, .payload = {0x10}, .delay = 5ms})
    ///
    /// `.mayNak = true` is a write the part is documented not to acknowledge -- a soft reset
    /// that takes effect before the ACK slot, a wake-up command of a sleeping part: a NAK of it
    /// is neither an error nor a sign of absence, and the script goes on after `delay` as it
    /// would after an ACK. So does it after a bus fault on that write: a part that resets inside
    /// the byte lets go of SDA wherever it is, which a controller may report as a lost
    /// arbitration rather than a NAK. It cannot be the first transaction of Init, which is the presence
    /// probe.
    struct WriteArgs {
        StepArgs::NamedOnly                 named_{};
        StepArgs::Required<std::uint16_t>   reg;
        std::initializer_list<std::uint8_t> payload{};
        std::chrono::milliseconds           delay{};
        bool                                mayNak{};
    };

    static constexpr Step write(WriteArgs a) {
        Step step{.kind        = Kind::write,
                  .hasRegister = true,
                  .reg         = a.reg,
                  .mayNak      = a.mayNak,
                  .delay       = a.delay};
        for(auto const byte : a.payload) {
            if(step.count < InlineBytes) { step.bytes[step.count] = byte; }
            ++step.count;   // past InlineBytes: wellFormed() rejects the script
        }
        return step;
    }

    /// Raw bytes, no register prefix: the command-style chips (Sensirion, AHT20, BH1750).
    ///     Step::command({.payload = {0x24, 0x00}, .delay = 20ms})
    struct CommandArgs {
        StepArgs::NamedOnly                 named_{};
        std::initializer_list<std::uint8_t> payload{};
        std::chrono::milliseconds           delay{};
        bool                                mayNak{};   ///< as Step::write
    };

    static constexpr Step command(CommandArgs a) {
        Step step = write({.reg = 0, .payload = a.payload, .delay = a.delay, .mayNak = a.mayNak});
        step.hasRegister = false;
        return step;
    }

    /// Register write with the payload taken from the group's buffer (a write group's
    /// encoded value, or bytes a previous read put there).
    ///     Step::writeBuffer({.reg = 0x02, .offset = 0, .count = 2})
    struct WriteBufferArgs {
        StepArgs::NamedOnly               named_{};
        StepArgs::Required<std::uint16_t> reg;
        StepArgs::Required<std::uint8_t>  offset;
        StepArgs::Required<std::uint8_t>  count;
        std::chrono::milliseconds         delay{};
    };

    static constexpr Step writeBuffer(WriteBufferArgs a) {
        return {.kind        = Kind::write,
                .hasRegister = true,
                .fromBuffer  = true,
                .reg         = a.reg,
                .count       = a.count,
                .offset      = a.offset,
                .delay       = a.delay};
    }

    /// Raw bytes from the buffer, no register prefix.
    ///     Step::commandBuffer({.offset = 0, .count = 4})
    struct CommandBufferArgs {
        StepArgs::NamedOnly              named_{};
        StepArgs::Required<std::uint8_t> offset;
        StepArgs::Required<std::uint8_t> count;
        std::chrono::milliseconds        delay{};
    };

    static constexpr Step commandBuffer(CommandBufferArgs a) {
        Step step = writeBuffer({.reg = 0, .offset = a.offset, .count = a.count, .delay = a.delay});
        step.hasRegister = false;
        return step;
    }

    /// Register read: the register, then (repeated START) `count` bytes into buffer[offset].
    ///     Step::read({.reg = 0x00, .count = 2, .offset = 4})
    struct ReadArgs {
        StepArgs::NamedOnly               named_{};
        StepArgs::Required<std::uint16_t> reg;
        StepArgs::Required<std::uint8_t>  count;
        std::uint8_t                      offset = 0;
        std::chrono::milliseconds         delay{};
    };

    static constexpr Step read(ReadArgs a) {
        return {.kind        = Kind::read,
                .hasRegister = true,
                .reg         = a.reg,
                .count       = a.count,
                .offset      = a.offset,
                .delay       = a.delay};
    }

    /// Bare read of `count` bytes into buffer[offset]: the answer to a command.
    ///     Step::receive({.count = 6})
    struct ReceiveArgs {
        StepArgs::NamedOnly              named_{};
        StepArgs::Required<std::uint8_t> count;
        std::uint8_t                     offset = 0;
        std::chrono::milliseconds        delay{};
    };

    static constexpr Step receive(ReceiveArgs a) {
        Step step        = read({.reg = 0, .count = a.count, .offset = a.offset, .delay = a.delay});
        step.hasRegister = false;
        return step;
    }

    /// Register read whose register address is in the buffer at `regOffset` (big-endian,
    /// Chip::RegisterBytes wide), put there by a Request's prepare(): a memory read.
    ///     Step::readIndirect({.regOffset = 0, .count = 16, .offset = 2})
    struct ReadIndirectArgs {
        StepArgs::NamedOnly              named_{};
        StepArgs::Required<std::uint8_t> regOffset;
        StepArgs::Required<std::uint8_t> count;
        StepArgs::Required<std::uint8_t> offset;
        std::chrono::milliseconds        delay{};
    };

    static constexpr Step readIndirect(ReadIndirectArgs a) {
        Step step = read(
          {.reg = a.regOffset.value, .count = a.count, .offset = a.offset, .delay = a.delay});
        step.regFromBuffer = true;
        return step;
    }

    /// Register read whose length an earlier step of the same run put in the buffer: the count
    /// is buffer[countOffset, countOffset + countBytes), big-endian, one or two bytes, clamped
    /// to `maxCount`. That many bytes are read into buffer[offset]; a count of 0 sends nothing
    /// and the script goes on with its next step (the delay is skipped with the transaction).
    /// In a read group, decode then sees the buffer up to offset + count. The script checks
    /// (scriptFault) want the count inside an earlier read of the script, and a step's buffer
    /// size, the bus load and the staging buffer are all reckoned at `maxCount`. A GNSS
    /// receiver's "bytes available" register, then its stream.
    ///     Step::readCounted({.reg = 0xFF, .countOffset = 0, .countBytes = 2, .maxCount = 64,
    ///                        .offset = 2})
    struct ReadCountedArgs {
        StepArgs::NamedOnly               named_{};
        StepArgs::Required<std::uint16_t> reg;
        StepArgs::Required<std::uint8_t>  countOffset;
        StepArgs::Required<std::uint8_t>  countBytes;
        StepArgs::Required<std::uint8_t>  maxCount;
        StepArgs::Required<std::uint8_t>  offset;
        std::chrono::milliseconds         delay{};
    };

    static constexpr Step readCounted(ReadCountedArgs a) {
        Step step = read({.reg = a.reg, .count = a.maxCount, .offset = a.offset, .delay = a.delay});
        step.counted     = true;
        step.countOffset = a.countOffset;
        step.countBytes  = a.countBytes;
        return step;
    }

    /// The same without a register: a bare read of the counted length.
    struct ReceiveCountedArgs {
        StepArgs::NamedOnly              named_{};
        StepArgs::Required<std::uint8_t> countOffset;
        StepArgs::Required<std::uint8_t> countBytes;
        StepArgs::Required<std::uint8_t> maxCount;
        StepArgs::Required<std::uint8_t> offset;
        std::chrono::milliseconds        delay{};
    };

    static constexpr Step receiveCounted(ReceiveCountedArgs a) {
        Step step        = readCounted({.reg         = 0,
                                        .countOffset = a.countOffset,
                                        .countBytes  = a.countBytes,
                                        .maxCount    = a.maxCount,
                                        .offset      = a.offset,
                                        .delay       = a.delay});
        step.hasRegister = false;
        return step;
    }

    /// What a counted read reads this run: the count in `buffer`, clamped to the step's most.
    [[nodiscard]] constexpr std::uint8_t countIn(std::span<std::byte const> buffer) const {
        std::uint32_t n = static_cast<std::uint8_t>(buffer[countOffset]);
        if(countBytes == 2) { n = (n << 8U) | static_cast<std::uint8_t>(buffer[countOffset + 1U]); }
        return static_cast<std::uint8_t>(n > count ? count : n);
    }

    static constexpr Step wait(std::chrono::milliseconds delay) {
        return {.kind = Kind::wait, .delay = delay};
    }

    /// The group's `static constexpr bool ready(Bytes)` looks at what the steps before
    /// read; false runs the sequence again from its first step after `retry`, up to
    /// the engine's retry limit, after which the run is rejected.
    static constexpr Step check(std::chrono::milliseconds retry) {
        return {.kind = Kind::check, .delay = retry};
    }

    /// The group's `static constexpr bool ready(Bytes)` looks at what the steps before read;
    /// false skips the rest of the script and the group decodes what it has, true goes on. For
    /// a step that only belongs after a frame that had something in it -- the acknowledge of
    /// a touch controller's "new data" bit, which written after a frame without that bit
    /// could clear one that became ready in between.
    static constexpr Step stopUnless() { return {.kind = Kind::stopUnless}; }

    /// Init only. The chip's setup() looks at what the steps before it read; false ends the
    /// bring-up right there, as "not the chip this description is for", so nothing after it
    /// is written to a part that only shares the address. setup() runs again at the end of a
    /// bring-up that goes on, over everything read by then; until then the bytes of later
    /// reads are zero, so a setup() used with this step decides on the earlier ones alone.
    static constexpr Step identify() { return {.kind = Kind::identify}; }

    /// The engine's own, behind the reads of `Chip::Identity`: see Kind::oracle.
    static constexpr Step oracle() { return {.kind = Kind::oracle}; }

    [[nodiscard]] constexpr bool isTransaction() const {
        return kind == Kind::write || kind == Kind::read;
    }
};

/// How many buffer bytes a script touches: the end of its furthest read or buffered write (a
/// counted read at its most).
[[nodiscard]] constexpr std::size_t bufferBytes(std::span<Step const> steps) {
    std::size_t n = 0;
    for(auto const& s : steps) {
        if(s.kind == Step::Kind::read || (s.kind == Step::Kind::write && s.fromBuffer)) {
            auto const end = static_cast<std::size_t>(s.offset) + s.count;
            n              = end > n ? end : n;
        }
        if(s.regFromBuffer) {
            auto const end = static_cast<std::size_t>(s.reg) + 2;
            n              = end > n ? end : n;
        }
    }
    return n;
}

/// The longest payload any write in the script sends, for the engine's staging buffer. A
/// counted read stages nothing: what it reads lands in the group's buffer.
[[nodiscard]] constexpr std::size_t maxPayload(std::span<Step const> steps) {
    std::size_t n = 0;
    for(auto const& s : steps) {
        if(s.kind == Step::Kind::write && s.count > n) { n = s.count; }
    }
    return n;
}

/// A script is well formed when every inline write fits, every register fits the chip's
/// register width, and a chip without registers never addresses one. Checked at compile
/// time by the engine; the message names what is wrong.
enum class ScriptFault : std::uint8_t {
    none,
    inlinePayloadTooLong,
    registerTooWide,
    registerOnRegisterlessChip,
    emptyRead,
    checkInInit,
    countWidth,            ///< a counted read's count is not one or two bytes
    countNotReadBefore,    ///< ... or is not inside what an earlier read step of the script read
    countedPastBuffer,     ///< ... or offset + maxCount is past the 255 bytes a step addresses
    mayNakProbe,           ///< Init's first transaction may NAK, so it cannot probe for the part
    identifyOutsideInit,   ///< an identify step in a read group, which has no setup() to run
};

[[nodiscard]] constexpr ScriptFault scriptFault(std::span<Step const> steps,
                                                std::size_t           registerBytes,
                                                bool                  allowCheck = true) {
    bool first = true;
    for(std::size_t i = 0; i < steps.size(); ++i) {
        auto const& s = steps[i];
        if((s.kind == Step::Kind::check || s.kind == Step::Kind::stopUnless) && !allowCheck) {
            return ScriptFault::checkInInit;
        }
        if((s.kind == Step::Kind::identify || s.kind == Step::Kind::oracle) && allowCheck) {
            return ScriptFault::identifyOutsideInit;
        }
        if(!s.isTransaction()) { continue; }
        if(first && s.mayNak && !allowCheck) { return ScriptFault::mayNakProbe; }
        first = false;
        if(s.hasRegister && registerBytes == 0) { return ScriptFault::registerOnRegisterlessChip; }
        if(s.hasRegister && !s.regFromBuffer && registerBytes == 1 && s.reg > 0xFF) {
            return ScriptFault::registerTooWide;
        }
        if(s.kind == Step::Kind::write && !s.fromBuffer && s.count > Step::InlineBytes) {
            return ScriptFault::inlinePayloadTooLong;
        }
        if(s.kind == Step::Kind::read && s.count == 0) { return ScriptFault::emptyRead; }
        if(s.kind == Step::Kind::read && s.counted) {
            if(s.countBytes != 1 && s.countBytes != 2) { return ScriptFault::countWidth; }
            if(static_cast<std::size_t>(s.offset) + s.count > 255) {
                return ScriptFault::countedPastBuffer;
            }
            auto const from = static_cast<std::size_t>(s.countOffset);
            auto const to   = from + s.countBytes;
            bool       read = false;
            for(std::size_t k = 0; k < i; ++k) {
                auto const& e = steps[k];
                // an uncounted read: one that is sure to have filled the bytes it names
                read = read
                    || (e.kind == Step::Kind::read && !e.counted && e.offset <= from
                        && to <= static_cast<std::size_t>(e.offset) + e.count);
            }
            if(!read) { return ScriptFault::countNotReadBefore; }
        }
    }
    return ScriptFault::none;
}

[[nodiscard]] constexpr bool wellFormed(std::span<Step const> steps,
                                        std::size_t           registerBytes,
                                        bool                  allowCheck = true) {
    return scriptFault(steps, registerBytes, allowCheck) == ScriptFault::none;
}

}   // namespace Kvasir::I2C
