#pragma once

#include "../../Engine/Description.hpp"
#include "Ad7124.hpp"
#include "Bme280.hpp"
#include "Max31865.hpp"
#include "Max7219.hpp"
#include "Mpu9250.hpp"

/// Every SPI chip description on the engine, with default parameters, for the tests that walk
/// them all (test/spi/oracle_test.cpp). A new description gets a line here. NorFlash, ADS8675,
/// ADS131M0x, the LTC2401 reader and the LTC2641 are not descriptions; test/spi/no_part_test.cpp
/// runs every driver.
namespace Kvasir::SPI::Chips {

using Every
  = Engine::List<Bme280,
                 Bmp280,
                 Mpu9250,
                 Max7219<1,
                         0xFF,
                         Max7219Defaults::scanLimit,
                         Max7219Defaults::intensity,
                         Max7219Defaults::refreshPeriod.count()>,
                 Max31865<Max31865Defaults::configuration,
                          std::chrono::milliseconds{Max31865Defaults::registerCheck}.count()>,
                 Ad7124<Ad7124Defaults>>;

}   // namespace Kvasir::SPI::Chips
