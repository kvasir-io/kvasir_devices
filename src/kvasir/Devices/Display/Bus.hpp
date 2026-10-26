#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Display {

    /// The transport of a display controller (DcsPanel, Ssd1680): what goes out with D/C low and
    /// what with it high, and the transfer state. One concept for every panel; SpiDcsBus is the
    /// 4-wire SPI one, chip_rp_common's PioQspi the QSPI one.
    ///
    ///   writeCommand(cmd, params)      queued in order, never waits: false = refused (no room, or
    ///                                  the bus is in a pixel stream), which the panel counts as a
    ///                                  failure. A bus that sends at once is one too. Parameters a bus
    ///                                  cannot copy (SpiDcsBus: more than `copiedParams` bytes) must
    ///                                  stay valid until idle() - a script's constants do.
    ///   writePixels(bytes)             asynchronous RAMWR + the bytes; `bytes` stays alive until idle()
    ///   fillPixels(hi, lo, count)      asynchronous RAMWR + one two-byte pixel `count` times
    ///   readRegister(instr, cmd, dummy, out)  synchronous, after what is queued; `dummy` bytes
    ///                                  discarded first
    ///   idle()                         nothing queued, nothing on the wire
    ///   failed(), clearError()         a transfer that failed or ran past the bus's timeout, latched
    ///                                  until the panel's retry clears it
    ///   turn()                         once a loop turn: completions, timeouts
    template<typename B>
    concept Bus = requires(std::uint8_t               u8,
                           std::span<std::byte const> in,
                           std::span<std::byte>       out,
                           std::size_t                n) {
        { B::writeCommand(u8, in) } -> std::same_as<bool>;
        { B::writePixels(in) };
        { B::fillPixels(u8, u8, n) };
        { B::readRegister(u8, u8, n, out) } -> std::same_as<bool>;
        { B::idle() } -> std::same_as<bool>;
        { B::failed() } -> std::same_as<bool>;
        { B::turn() };
        { B::clearError() };
    };

    /// A Bus that also sends bare data with D/C high under its own chip select, after what is
    /// queued: a controller whose memory write is not DCS's RAMWR (the SSD1680's WRITE RAM 24h, its
    /// frame repacked chunk by chunk). `bytes` stays alive until idle(); false = refused.
    /// A 4-wire bus has it (SpiDcsBus); a QSPI one does not (every QSPI write starts with a command).
    template<typename B>
    concept DataBus = Bus<B> && requires(std::span<std::byte const> in) {
        { B::writeData(in) } -> std::same_as<bool>;
    };

}}   // namespace Kvasir::Display
