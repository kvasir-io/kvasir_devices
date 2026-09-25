#pragma once

#include "../../Bme280Compensation.hpp"
#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Bme280Detail {
    /// The trimming parameters and the datasheet's integer compensation, shared with the SPI
    /// driver (../../Bme280Compensation.hpp).
    using Trim = Kvasir::Bme280Compensation::Trim;
}   // namespace Bme280Detail

/// Bosch BME280 (BST-BME280-DS001-23) and BMP280 (BST-BMP280-DS001; `Model::bmp280`: no
/// ctrl_hum, no 0xE1 trimming, six data bytes). 8-bit registers, one-byte address,
/// auto-increment. Bring-up: soft reset 0xE0 = 0xB6 (2 ms), id 0xD0 (0x60 BME280, 0x58
/// BMP280), trimming 0x88..0xA1 and 0xE1..0xE7, ctrl_hum 0xF2 (x1), config 0xF5 (standby
/// 1 s, filter off) while the part is still in sleep mode -- 5.4.6: in normal mode a write
/// to config may be ignored -- and last ctrl_meas 0xF4 (temperature x1, pressure x1, normal
/// mode), which also latches ctrl_hum. Then one burst read from 0xF7 a second (press 20 bit,
/// temp 20 bit, hum 16 bit, big-endian), compensated with the datasheet's integer code. A
/// frame holding a skipped value -- 0x80000 for pressure or temperature, 0x8000 for
/// humidity, what the registers read before the first conversion -- is rejected. 0x76 with
/// SDO low, 0x77 high.
template<Bme280Compensation::Model Model = Bme280Compensation::Model::bme280>
struct Bmx280 {
    static constexpr bool Humidity = Model == Bme280Compensation::Model::bme280;

    static constexpr std::string_view Name = Humidity ? "BME280" : "BMP280";
    /// The id register 0xD0 reads 0x60 on the BME280 (BME280.md:1106) and 0x58 on the BMP280
    /// (BMP280.md:989).
    static constexpr std::array Identity{
      RegisterCheck{"chip-id", 0xD0, 1, true, 0xFF, Humidity ? 0x60U : 0x58U},
    };
    static constexpr Address7                Address = 0x76;
    static constexpr std::array<Address7, 2> Addresses{0x76, 0x77};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr auto Init = [] {
        if constexpr(Humidity) {
            return std::array{
              Step::write({.reg = 0xE0, .payload = {0xB6}, .delay = std::chrono::milliseconds{3}}),
              Step::read({.reg = 0x88, .count = 26, .offset = 0}),
              Step::read({.reg = 0xE1, .count = 7, .offset = 26}),
              Step::write({.reg = 0xF2, .payload = {0x01}}),
              Step::write({.reg = 0xF5, .payload = {0xA0}}),
              Step::write({.reg = 0xF4, .payload = {0x27}, .delay = std::chrono::milliseconds{10}}),
            };
        } else {
            return std::array{
              Step::write({.reg = 0xE0, .payload = {0xB6}, .delay = std::chrono::milliseconds{3}}),
              Step::read({.reg = 0x88, .count = 26, .offset = 0}),
              Step::write({.reg = 0xF5, .payload = {0xA0}}),
              Step::write({.reg = 0xF4, .payload = {0x27}, .delay = std::chrono::milliseconds{10}}),
            };
        }
    }();

    using State = Bme280Detail::Trim;

    /// `state().deviceId` is the id register as the engine read it for the Identity above -- before
    /// the soft reset Init opens with, which used to go to whatever answered at the address.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint8_t>(ids[0]);
    }

    /// The trimming parameters, read after the reset.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.loadTemperaturePressure(data, 0);
        if constexpr(Humidity) { state.loadHumidity(data, 26); }
        return true;
    }

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{Step::read({.reg = 0xF7, .count = Humidity ? 8 : 6})};

        struct Sample {
            CentiDegC    temperature{};   ///< 0.01 degC
            Pascal       pressure{};
            MilliPercent humidity{};   ///< 0.001 %RH (0 on a BMP280)
        };

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& t) {
            std::int32_t const adcP = static_cast<std::int32_t>(data.be24(0) >> 4);
            std::int32_t const adcT = static_cast<std::int32_t>(data.be24(3) >> 4);
            if(adcP == Bme280Compensation::Skipped20 || adcT == Bme280Compensation::Skipped20) {
                return Outcome<Sample>::reject();
            }
            if constexpr(Humidity) {
                if(data.be16(6) == Bme280Compensation::Skipped16) {
                    return Outcome<Sample>::reject();
                }
            }
            auto const tFine = t.tFine(adcT);
            Sample     sample{};
            sample.temperature = Units::centiDegC(State::centi(tFine));
            sample.pressure    = Units::pascal(t.pressurePa(adcP, tFine));
            if constexpr(Humidity) {
                sample.humidity = Units::milliPercent(
                  t.humidityMilli(static_cast<std::int32_t>(data.be16(6)), tFine));
            }
            return Outcome<Sample>::ok(sample);
        }
    };

    using Reads = List<Measurement>;
};

using Bme280 = Bmx280<Bme280Compensation::Model::bme280>;
using Bmp280 = Bmx280<Bme280Compensation::Model::bmp280>;

}   // namespace Kvasir::I2C::Chips
