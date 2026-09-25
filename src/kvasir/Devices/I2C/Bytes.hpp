#pragma once

#include "../Bytes.hpp"

/// The byte helpers live in Kvasir (../Bytes.hpp), shared with the SPI drivers. The I2C
/// spellings are the same names, visible unqualified inside a description.
namespace Kvasir::I2C {

using Kvasir::Bytes;
using Kvasir::crc8;
using Kvasir::crc8Reflected;
using Kvasir::crc8Smbus;
using Kvasir::putBe16;
using Kvasir::putLe16;
using Kvasir::toBcd;

namespace Sensirion = Kvasir::Sensirion;
namespace Dallas    = Kvasir::Dallas;

}   // namespace Kvasir::I2C
