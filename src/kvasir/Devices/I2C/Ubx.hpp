#pragma once

#include "../Gnss/Ubx.hpp"

/// The UBX framer and frame builder are transport-independent and live in Gnss/Ubx.hpp; this
/// makes them available under `Kvasir::I2C` too.
namespace Kvasir::I2C {
using Gnss::Ubx;
using Gnss::ubxChecksum;
using Gnss::ubxFrame;
using Gnss::UbxMessage;
}   // namespace Kvasir::I2C
