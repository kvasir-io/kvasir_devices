#pragma once
// An SPI part on the chip-description engine (I2C/Device.hpp), through SPI/Transport.hpp.
//
//     using Rtd = Kvasir::SPI::Device<RtdSpi, Clock, Kvasir::SPI::Chips::Max31865<...>, Pin::rtd_cs>;
//     Rtd rtd{};                 // CS released here; the pin itself is configured by the firmware
//     RtdSpi::handler(); rtd.handler(); rtd.latest().temperature ...
#include "../I2C/Device.hpp"
#include "Transport.hpp"

namespace Kvasir { namespace SPI {

    using I2C::Bytes;
    using I2C::DefaultConfig;
    using I2C::List;
    using I2C::NoReset;
    using I2C::Outcome;
    using I2C::RegisterCheck;
    using I2C::Step;

    template<typename Master,
             typename Clock,
             SpiChip Chip,
             typename Cs,
             typename Config = DefaultConfig,
             typename Reset  = NoReset>
    struct Device : I2C::Device<Transport<Master, Cs, Chip>, Clock, Chip, Config, Reset> {
        using TransportT = Transport<Master, Cs, Chip>;
        using MasterT    = Master;
        using CsT        = Cs;

        /// CS released before anything else: a part that sees CS low early answers garbage.
        Device() { apply(set(Cs{})); }
    };

}}   // namespace Kvasir::SPI
