#pragma once

#include "../Quantities.hpp"

/// The quantities live in Kvasir::Units (../Quantities.hpp), shared with the SPI drivers. The
/// I2C spellings are aliases of them: `Kvasir::I2C::Units::celsius`,
/// `I2C::CentiDegC` unqualified inside a description, `I2C::TemperatureSource`.
namespace Kvasir::I2C {

namespace Units = ::Kvasir::Units;

// The aliases are visible unqualified inside a description, which is where they are written.
using Units::CentiDegC;
using Units::DeciDegC;
using Units::DegC;
using Units::MilliDegC;
using Units::Temperature;

using Units::CentiPercent;
using Units::Humidity;
using Units::MilliPercent;

using Units::MicroAmp;
using Units::MicroVolt;
using Units::MicroWatt;
using Units::MilliAmp;
using Units::MilliVolt;
using Units::MilliWatt;
using Units::NanoAmp;
using Units::NanoVolt;
using Units::Pascal;

using Units::MicroDegPerSec;
using Units::MicroG;
using Units::MicroLux;
using Units::MicroTesla;
using Units::MilliDegPerSec;
using Units::MilliG;
using Units::MilliLux;
using Units::NanoTesla;

using Units::CentiDegree;
using Units::Hertz;
using Units::MicroDegree;
using Units::MicroOhm;
using Units::MilliMetre;
using Units::MilliMetrePerSecond;
using Units::MilliOhm;
using Units::MilliUvi;
using Units::Ohm;
using Units::Percent;
using Units::PicoFarad;
using Units::Ppm;
using Units::Rpm;

using ::Kvasir::TemperatureSource;

}   // namespace Kvasir::I2C
