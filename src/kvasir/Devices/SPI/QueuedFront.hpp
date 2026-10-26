#pragma once
// The public face of a queued SPI master, once for every chip: chip_rp_common SPIQueued and chip_atsam_common
// Sercom_SPIQueued are their hardware half (registers, DMA, the QueueCore over it) and this front, whose members are
// one line each into the core:
//
//     template<...>
//     struct SPIQueued
//       : Detail::SPIQueuedHalf<...>                                   // Regs, Setup, setup(), Hw, Core, Startup
//       , Kvasir::SPI::QueuedFront<typename Detail::SPIQueuedHalf<...>::Core> {};
//
// Why two bases and not the front over the whole master: the base clause needs the core, so a complete Hw, while a
// policy nested in the class it is a base of is incomplete there (the I2C queue_core_test names its core where the
// policy is complete for the same reason). The front holds no state and declares no `Core`: `Master::Core` is the
// half's, `Master::Request`/`Result` the front's, so a name has one meaning.
//
// Also here, for the chips' consteval `setup(mode, maxClock)`: a device's clock ceiling on the bus, refused at compile
// time when the block cannot clock that slowly, and the mode's CPOL/CPHA bits.
#include "../Bus/QueuedFront.hpp"
#include "../BusTypes.hpp"
#include "QueueCore.hpp"
#include "kvasir/Hal/Spi.hpp"
#include "kvasir/Util/Prescaler.hpp"

#include <algorithm>
#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace SPI {

    /// The API of a queued master over its `Core` (a QueueCore): the members both buses share are Bus::QueuedFront's
    /// (Bus/QueuedFront.hpp), the SPI-only ones here.
    template<typename Core>
    struct QueuedFront : ::Kvasir::Bus::QueuedFront<Core, typename Core::RequestT> {
        using Request = typename Core::RequestT;

        /// Ends the hold on `l` (Lines::hold): CS goes up, and the next device may start.
        static void releaseHold(Lines const& l) { Core::releaseHold(l); }

        /// Receive overruns the hardware reported (the transfer failed).
        static std::uint32_t overruns()
            requires(Core::Features.counters)
        {
            return Core::overruns();
        }

        /// Holds no request continued within holdTimeout: released by turn().
        static std::uint32_t holdTimeouts()
            requires(Core::Features.counters)
        {
            return Core::holdTimeouts();
        }

        /// Completions of a transfer the core had already ended (a timeout or a reset raced the interrupt).
        static std::uint32_t staleCompletions()
            requires(Core::Features.counters)
        {
            return Core::staleCompletions();
        }

        /// Requests submit() refused as malformed (QueueCore::wellFormed).
        static std::uint32_t refused()
            requires(Core::Features.counters)
        {
            return Core::refused();
        }

        static_assert(
          std::is_same_v<typename QueuedFront::Extends,
                         typename ::Kvasir::Bus::QueuedFront<Core,
                                                             Request>::Extends>,
          "the MainLoop hook is the shared front's (an ambiguous Extends reads as absent)");
    };

    /// A clock mode's polarity and phase bits (mode = CPOL * 2 + CPHA): PL022 SPO/SPH, SERCOM CTRLA.CPOL/CPHA.
    struct ModeBits {
        std::uint8_t cpol{};
        std::uint8_t cpha{};
    };

    [[nodiscard]] constexpr ModeBits modeBits(Hal::ClockMode mode) {
        auto const m = static_cast<std::uint8_t>(mode);
        return ModeBits{.cpol = static_cast<std::uint8_t>((m >> 1U) & 1U),
                        .cpha = static_cast<std::uint8_t>(m & 1U)};
    }

    /// The fastest SCK a device may get on a bus: its own ceiling `deviceMaxHz`, never above the bus's `baudRate`. The
    /// block makes `clk / divider` for dividers `fastestDivider` .. `slowestDivider`; a ceiling below the slowest
    /// clock is a compile error in the consteval caller, with clang's note "in call to 'rateOutOfReach(wanted Hz,
    /// slowest mHz, fastest mHz)'".
    consteval std::uint32_t ceilingHz(std::uint32_t deviceMaxHz,
                                      std::uint32_t baudRate,
                                      std::uint32_t clk,
                                      std::uint32_t slowestDivider,
                                      std::uint32_t fastestDivider) {
        auto const max = std::min(deviceMaxHz, baudRate);
        if(std::uint64_t{max} * slowestDivider < clk) {
            Prescaler::rateOutOfReach(max,
                                      std::uint64_t{clk} * 1000U / slowestDivider,
                                      std::uint64_t{clk} * 1000U / fastestDivider);
        }
        return max;
    }
}}   // namespace Kvasir::SPI
