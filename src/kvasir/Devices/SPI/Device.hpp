#pragma once
// An SPI part on the chip-description engine (Engine/Device.hpp), through SPI/Transport.hpp.
//
//     using Rtd = Kvasir::SPI::Device<RtdSpi, Clock, Kvasir::SPI::Chips::Max31865<...>, Pin::rtd_cs>;
//     Rtd rtd{};                 // CS released here; the pin itself is configured by the firmware
//     RtdSpi::turn(); rtd.turn(); rtd.latest().temperature ...
#include "../Engine/Device.hpp"
#include "Transport.hpp"

namespace Kvasir { namespace SPI {

    template<typename Master,
             typename Clock,
             SpiChip Chip,
             typename Cs,
             typename Config = Engine::DefaultConfig,
             typename Reset  = Kvasir::NoReset>
    struct Device : Engine::Device<Transport<Master, Cs, Chip>, Clock, Chip, Config, Reset> {
        using MasterT = Master;
        using CsT     = Cs;

        /// CS released before anything else: a part that sees CS low early answers garbage.
        Device() { ChipSelect<Cs>::release(); }
    };

}}   // namespace Kvasir::SPI
