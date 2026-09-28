#pragma once
// The Bosch BME280 / BMP280 on SPI: the I2C description plus its wire. The register MSB is the
// R/W bit, 1 = read (BME280.md:1481, 6.3) - the transport's default.
#include "../../I2C/chips/Bme280.hpp"
#include "../Device.hpp"

namespace Kvasir { namespace SPI { namespace Chips {

    template<Bme280Compensation::Model Model = Bme280Compensation::Model::bme280>
    struct Bmx280 : I2C::Chips::Bmx280<Model> {
        /// Mode '00' or '11', chosen by SCK's level at CSB's falling edge (6.3, BME280.md:1456).
        static constexpr ClockMode Mode = ClockMode::_0;
        /// F_spi up to 10 MHz (Table 34, BME280.md:1616).
        static constexpr Units::Hertz MaxClock = Units::hertz(10'000'000);
    };

    using Bme280 = Bmx280<Bme280Compensation::Model::bme280>;
    using Bmp280 = Bmx280<Bme280Compensation::Model::bmp280>;

}}}   // namespace Kvasir::SPI::Chips
