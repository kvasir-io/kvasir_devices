#pragma once

#include "Dcs.hpp"
#include "DcsPanel.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir { namespace Co5300 {

    namespace Dcs = Display::Dcs;

    /// The Chipone CO5300 AMOLED controller (Co5300_Datasheet.md; confirmed on a 466 x 466
    /// module, whose panel sits at column offset +6):
    /// - command write 02h + {00h, CMD, 00h}, pixels 32h + {00h, 2Ch, 00h} on four lanes,
    ///   register read 03h + the same address with no dummy byte (5.2.1 - 5.2.3)
    /// - 50 MHz write ceiling (TSCYC >= 20 ns, 6.4.1), 10 MHz read ceiling
    /// - CASET/RASET need an even start and an even count in both axes (5689/5740)
    /// - address fields are 10 bit
    /// - RESET low >= 10 us (tRESW, 6.6), 10 ms to the first command (T3, 5.6.2),
    ///   120 ms after SLPOUT (line 5333)
    /// - RDDID -> 33h, 10h/11h, 00h; only ID1 is worth checking (line 4797)
    struct Chip {
        static constexpr std::string_view name = "CO5300";

        static constexpr std::uint8_t readInstruction = 0x03;
        static constexpr std::size_t  readDummyBytes  = 0;
        static constexpr int          windowAlign     = 2;
        static constexpr int          maxAddress      = 1024;

        static constexpr Units::Hertz maxWriteClock = Units::hertz(50'000'000);
        static constexpr Units::Hertz maxReadClock  = Units::hertz(10'000'000);

        // 3Ah: 55h sets both IFPF and VIPF to 16 bit (line 6156).
        static constexpr std::uint8_t colmod565 = 0x55;

        /// tRESW is 10 us; generous on purpose
        static constexpr std::chrono::milliseconds resetPulse{20};
        /// T3 >= 10 ms, and note 5 wants 5 ms
        static constexpr std::chrono::milliseconds resetToCommand{10};
        /// covers T4 (60 ms SLPOUT to DISPON) too
        static constexpr std::chrono::milliseconds sleepOut{120};

        // 53h: DBV[7:0] does nothing at all unless BCTRL (D5) is set (line 6481).
        static constexpr bool         brightness  = true;
        static constexpr std::uint8_t ctrlDisplay = 0x20;

        static constexpr std::uint8_t expectedId1 = 0x33;

        /// C4h D7 SPI_WRAM gates SPI writes into panel SRAM and resets *disabled* (line
        /// 7529): without it every pixel write is dropped inside the panel. 34h: TE output off.
        static constexpr std::array<std::uint8_t, 7> initBytes{
          0xC4,
          1,
          0x80,
          0,   // SPI mode: SPI_WRAM on
          0x34,
          0,
          0,   // TEOFF
        };
        static_assert(Dcs::Script::wellFormed(initBytes));
        static constexpr Dcs::Script init{initBytes};
    };

    static_assert(Display::Chip<Chip>);

    template<typename Clock, typename Bus, typename Rst, typename Config>
    using Panel = Display::DcsPanel<Clock, Bus, Rst, Chip, Config>;

}}   // namespace Kvasir::Co5300
