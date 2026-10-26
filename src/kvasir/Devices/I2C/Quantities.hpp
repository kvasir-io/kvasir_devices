#pragma once

#include "../Quantities.hpp"

/// The quantities live in Kvasir::Units (../Quantities.hpp), shared with the SPI drivers. The
/// quantity types are declared into Kvasir::I2C as well, so a description writes `CentiDegC`
/// unqualified; everything else (the units, `si`, `mpu`, `in()`) is spelled `Kvasir::Units::`.
namespace Kvasir::I2C {

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

}   // namespace Kvasir::I2C
