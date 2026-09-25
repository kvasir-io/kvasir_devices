#pragma once

#include "../Gnss/Nmea.hpp"

/// The NMEA framer is transport-independent and lives in Gnss/Nmea.hpp; this makes it available
/// as `Kvasir::I2C::Nmea` too.
namespace Kvasir::I2C { using Gnss::Nmea; }   // namespace Kvasir::I2C
