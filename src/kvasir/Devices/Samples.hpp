#pragma once

#include "Quantities.hpp"

#include <concepts>
#include <cstddef>
#include <type_traits>

/// The shapes a Sample takes across chips, as concepts: what a page, a log line or a
/// controller can be written against once instead of once per part. A Sample matches when
/// it has the members named, each a Kvasir::Units quantity that converts losslessly to the
/// canonical one -- so a chip reporting CentiDegC is a Thermometer just as one reporting
/// MilliDegC is, and a reader takes whichever it gets and converts once.
///
///     template<Kvasir::Samples::Climate S>
///     void show(S const& s) { ... in(s.temperature, si::degree_Celsius) ... }
///
/// The device-level `TemperatureSource` (Quantities.hpp) is the same idea one step up:
/// `d.latest().temperature` on any driver, on any bus.
namespace Kvasir::Samples {

namespace detail {
    template<typename Q, typename To>
    concept As = std::convertible_to<std::remove_cvref_t<Q>, To>;
}   // namespace detail

/// A temperature: TMP117, MCP9808, ADT7420, and every part below that carries one.
template<typename S>
concept Thermometer
  = requires(S const& s) { requires detail::As<decltype(s.temperature), Units::Temperature>; };

/// A relative humidity beside the temperature: SHT3x, SHT4x, SHTC3, AHT20, HTU21D, HDC1080,
/// BME280.
template<typename S>
concept Climate = Thermometer<S> && requires(S const& s) {
    requires detail::As<decltype(s.humidity), Units::Humidity>;
};

/// A pressure: BMP180, BMP280, BME280, DPS310, MPL3115A2.
template<typename S>
concept Barometer
  = requires(S const& s) { requires detail::As<decltype(s.pressure), Units::Pascal>; };

/// Bus voltage, current and power: the INA family.
template<typename S>
concept PowerMonitor = requires(S const& s) {
    requires detail::As<decltype(s.busVoltage), Units::MicroVolt>;
    requires detail::As<decltype(s.current), Units::MicroAmp>;
    requires detail::As<decltype(s.power), Units::MicroWatt>;
};

/// Three accelerations as x, y, z: ADXL345, IIS2DULPX, LIS3DH, LSM303AGR, MMA8451, BMA456.
template<typename S>
concept Accelerometer = requires(S const& s) {
    requires detail::As<decltype(s.x), Units::MicroG>;
    requires detail::As<decltype(s.y), Units::MicroG>;
    requires detail::As<decltype(s.z), Units::MicroG>;
};

/// Three accelerations and three rates as arrays: LSM6DS3, LSM9DS1, MPU-6050, QMI8658.
template<typename S>
concept Imu = requires(S const& s) {
    requires detail::As<decltype(s.accel[0]), Units::MicroG>;
    requires detail::As<decltype(s.gyro[0]), Units::MilliDegPerSec>;
};

/// A magnetic field as x, y, z: HMC5883L, LSM303AGR, LSM9DS1, TLV493D.
template<typename S>
concept Magnetometer = requires(S const& s) {
    requires detail::As<decltype(s.x), Units::NanoTesla>;
    requires detail::As<decltype(s.y), Units::NanoTesla>;
    requires detail::As<decltype(s.z), Units::NanoTesla>;
};

/// An illuminance through lux(): BH1750, VEML6030, TSL2561, TSL2591, LTR390, OPT4048.
template<typename S>
concept LightMeter
  = requires(S const& s) { requires detail::As<decltype(s.lux()), Units::MicroLux>; };

/// Channel voltages through voltage(channel) over a `code` array: ADS1015, ADS1115, ADS1219,
/// AD7291.
template<typename S>
concept Adc = requires(S const& s) {
    requires detail::As<decltype(s.voltage(std::size_t{})), Units::MicroVolt>;
    s.code[0];
};

/// A potentiometer: a tap and the resistance it stands for (MCP4018, DS3502).
template<typename S>
concept Wiper = requires(S const& s) {
    s.tap;
    requires detail::As<decltype(s.resistance()), Units::MicroOhm>;
};

/// A distance: VL53L1X.
template<typename S>
concept RangeFinder
  = requires(S const& s) { requires detail::As<decltype(s.distance), Units::MilliMetre>; };

}   // namespace Kvasir::Samples
