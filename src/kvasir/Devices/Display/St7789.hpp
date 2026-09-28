#pragma once
// The Sitronix ST7789V TFT controller as a DcsPanel chip policy, for SpiDcsBus.
// Facts from ~/datasheeds/drivers/ST7789V/ST7789V.md (v1.3, 2014/03):
// - serial clock cycle >= 66 ns writing, >= 150 ns reading (TSCYCW / TSCYCR, :1618, :1621)
// - RDID1 / RDDID's first byte 85h (:7333)
// - SWRESET: wait 5 ms before the next command, 120 ms before SLPOUT if it was sent in sleep in (:5588)
// - SLPOUT: 5 ms before the next command, 120 ms before SLPIN (:6006)
// - COLMOD 55h: 65K colours, 16 bits a pixel
// - frame memory 240 x 320, 9-bit column and row fields
#include "Dcs.hpp"
#include "DcsPanel.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir { namespace St7789 {

    namespace Dcs = Display::Dcs;

    struct Chip {
        static constexpr std::string_view Name = "ST7789V";

        /// Unused on a 4-wire bus.
        static constexpr std::uint8_t ReadInstruction = 0x00;
        static constexpr std::size_t  ReadDummyBytes  = 0;
        static constexpr int          WindowAlign     = 1;
        static constexpr int          MaxAddress      = 320;

        static constexpr unsigned long MaxWriteHz = 15'000'000;   // 1 / 66 ns
        static constexpr unsigned long MaxReadHz  = 6'600'000;    // 1 / 150 ns

        static constexpr std::uint8_t Colmod565 = 0x55;

        /// TRW 10 us, TRT 5 ms (ST7789V.md:1763-1764), for a module with its own RESX.
        static constexpr unsigned ResetPulseMs     = 1;     // tRW >= 10 us
        static constexpr unsigned ResetToCommandMs = 120;   // after RESX in sleep out
        static constexpr unsigned SleepOutMs       = 120;

        /// WRDISBV drives the CABC pin, not the module's backlight.
        static constexpr bool         Brightness  = false;
        static constexpr std::uint8_t CtrlDisplay = 0x00;

        static constexpr std::uint8_t ExpectedId1 = 0x85;

        /// SWRESET first: with RESX on RUN the panel keeps its state across an MCU-only reset.
        /// 150 ms covers the 120 ms from sleep in.
        static constexpr std::array<std::uint8_t, 3> InitBytes{Dcs::Cmd::Swreset, 0, 150};
        static_assert(Dcs::Script::wellFormed(InitBytes));
        static constexpr Dcs::Script Init{InitBytes};
    };

    static_assert(Display::Chip<Chip>);

    template<typename Clock, typename Bus, typename Rst, typename Config>
    using Panel = Display::DcsPanel<Clock, Bus, Rst, Chip, Config>;

}}   // namespace Kvasir::St7789
