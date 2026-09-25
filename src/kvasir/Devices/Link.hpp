#pragma once

#include <cstdint>

/// Where a driver is with the part on the wire, on any bus. The I2C engine (I2C/Device.hpp)
/// and the SPI base (SPIDeviceBase.hpp) both report it, so a status page can treat a sensor
/// the same whichever bus it is on.
namespace Kvasir {

enum class Link : std::uint8_t {
    starting,    ///< nothing acknowledged since the driver (re)started: in bring-up, or waiting
                 ///< for the first answer of a part that has no bring-up script
    answering,   ///< through its bring-up, and the part has answered since
    absent,      ///< parked after failures in a row, and probed now and then
    offline,     ///< behind a bridge that is not active (I2C/Bridge.hpp): not talked to on
                 ///< purpose, which is neither a failure nor an absence
};

}   // namespace Kvasir
