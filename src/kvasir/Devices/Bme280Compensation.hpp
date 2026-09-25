#pragma once

#include "Bytes.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

/// The BME280 / BMP280 trimming parameters and the datasheet's integer compensation
/// (BST-BME280-DS001 4.2.3 rev 1.1, BMP280 Table 17: the same formulas), bus-neutral: the
/// I2C description (I2C/chips/Bme280.hpp) and the SPI driver (SPI/Bme280.hpp) both decode
/// through it, so a compensation fix lands in both at once.
namespace Kvasir::Bme280Compensation {

/// Which of the two parts: the BME280 adds humidity (ctrl_hum, 0xE1 trimming, two more data
/// bytes) to the BMP280's pressure and temperature.
enum class Model : std::uint8_t { bmp280, bme280 };

/// What a data register holds for a measurement that was skipped (oversampling 000) or has
/// not run yet since the reset: 0x80000 in the 20-bit pressure and temperature fields,
/// 0x8000 in the humidity word (5.4.7 to 5.4.9, the reset values of 0xF7..0xFE).
inline constexpr std::int32_t  Skipped20 = 0x80000;
inline constexpr std::uint16_t Skipped16 = 0x8000;

struct Trim {
    std::uint16_t               t1{};
    std::int16_t                t2{}, t3{};
    std::array<std::int32_t, 9> p{};
    std::uint8_t                h1{}, h3{};
    std::int16_t                h2{}, h4{}, h5{};
    std::int8_t                 h6{};
    std::uint8_t                deviceId{};

    /// Registers 0x88..0xA1 as 26 bytes at `at`, little-endian words.
    constexpr void loadTemperaturePressure(Bytes       data,
                                           std::size_t at) {
        t1 = data.le16(at);
        t2 = data.s16le(at + 2);
        t3 = data.s16le(at + 4);
        for(std::size_t i = 0; i < 9; ++i) {
            p[i] = i == 0 ? static_cast<std::int32_t>(data.le16(at + 6 + 2 * i))
                          : static_cast<std::int32_t>(data.s16le(at + 6 + 2 * i));
        }
        h1 = data.u8(at + 25);   // 0xA1
    }

    /// Registers 0xE1..0xE7 as 7 bytes at `at`. dig_H4 and dig_H5 are signed 12-bit values
    /// split over 0xE4 / 0xE5 / 0xE6 (Table 16): 0xE4 is the signed high byte of dig_H4
    /// and 0xE5[3:0] its low nibble; 0xE6 the signed high byte of dig_H5 and 0xE5[7:4] its
    /// low nibble. The high byte is read as int8 so the sign reaches bit 11.
    constexpr void loadHumidity(Bytes       data,
                                std::size_t at) {
        h2            = data.s16le(at);
        h3            = data.u8(at + 2);
        auto const e4 = data.s8(at + 3);
        auto const e5 = data.u8(at + 4);
        auto const e6 = data.s8(at + 5);
        h4            = static_cast<std::int16_t>(static_cast<std::int16_t>(e4) * 16 | (e5 & 0x0F));
        h5            = static_cast<std::int16_t>(static_cast<std::int16_t>(e6) * 16 | (e5 >> 4));
        h6            = data.s8(at + 6);
    }

    /// t_fine, and the temperature in 0.01 degC through `centi`.
    [[nodiscard]] constexpr std::int32_t tFine(std::int32_t adcT) const {
        std::int32_t const var1
          = ((((adcT >> 3) - (static_cast<std::int32_t>(t1) << 1))) * t2) >> 11;
        std::int32_t const var2 = (((((adcT >> 4) - static_cast<std::int32_t>(t1))
                                     * ((adcT >> 4) - static_cast<std::int32_t>(t1)))
                                    >> 12)
                                   * t3)
                               >> 14;
        return var1 + var2;
    }

    [[nodiscard]] static constexpr std::int32_t centi(std::int32_t tFine) {
        return (tFine * 5 + 128) >> 8;
    }

    /// Pressure in Pa (the 64-bit variant, Q24.8 rounded down to Pa).
    [[nodiscard]] constexpr std::uint32_t pressurePa(std::int32_t adcP,
                                                     std::int32_t tFine) const {
        std::int64_t v1 = static_cast<std::int64_t>(tFine) - 128000;
        std::int64_t v2 = v1 * v1 * p[5];
        v2              = v2 + ((v1 * p[4]) << 17);
        v2              = v2 + (static_cast<std::int64_t>(p[3]) << 35);
        v1              = ((v1 * v1 * p[2]) >> 8) + ((v1 * p[1]) << 12);
        v1              = (((static_cast<std::int64_t>(1) << 47) + v1)) * p[0] >> 33;
        if(v1 == 0) { return 0; }
        std::int64_t pr = 1048576 - adcP;
        pr              = (((pr << 31) - v2) * 3125) / v1;
        v1              = (static_cast<std::int64_t>(p[8]) * (pr >> 13) * (pr >> 13)) >> 25;
        v2              = (static_cast<std::int64_t>(p[7]) * pr) >> 19;
        pr              = ((pr + v1 + v2) >> 8) + (static_cast<std::int64_t>(p[6]) << 4);
        return static_cast<std::uint32_t>(pr >> 8);
    }

    /// Humidity in 0.001 %RH (from the Q22.10 result). The first term is formed in 64 bits:
    /// `adc_H << 14` and `dig_H4 << 20` each come close to 2^31 and their difference does
    /// not fit an int32 for every trim the part can carry.
    [[nodiscard]] constexpr std::uint32_t humidityMilli(std::int32_t adcH,
                                                        std::int32_t tFine) const {
        std::int32_t const x = tFine - 76800;
        std::int64_t const first
          = ((static_cast<std::int64_t>(adcH) << 14) - (static_cast<std::int64_t>(h4) << 20)
             - static_cast<std::int64_t>(h5) * x + 16384)
         >> 15;
        std::int64_t const second = (((((static_cast<std::int64_t>(x) * h6) >> 10)
                                       * (((static_cast<std::int64_t>(x) * h3) >> 11) + 32768))
                                      >> 10)
                                     + 2097152)
                                    * h2
                                  + 8192;
        std::int64_t       v      = first * (second >> 14);
        v = v - (((((v >> 15) * (v >> 15)) >> 7) * static_cast<std::int64_t>(h1)) >> 4);
        v = v < 0 ? 0 : v;
        v = v > 419430400 ? 419430400 : v;
        return static_cast<std::uint32_t>(((v >> 12) * 1000) >> 10);
    }
};

// A real part's trimming with the decode of one of its frames (0xF7.. = 65 5A C0 7E ED 00
// 80 00): 25.08 degC and 100653 Pa, the numbers the I2C description's test checks too.
namespace detail {
    constexpr Trim exampleTrim() {
        Trim t{};
        t.t1 = 27504;
        t.t2 = 26435;
        t.t3 = -1000;
        t.p  = {36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000};
        t.h1 = 75;
        t.h2 = 369;
        t.h3 = 0;
        t.h4 = 311;
        t.h5 = 50;
        t.h6 = 30;
        return t;
    }

    static_assert(Trim::centi(exampleTrim().tFine(519888)) == 2508);
    static_assert(exampleTrim().pressurePa(415148,
                                           exampleTrim().tFine(519888))
                    >= 100651
                  && exampleTrim().pressurePa(415148,
                                              exampleTrim().tFine(519888))
                       <= 100655);
    static_assert(exampleTrim().humidityMilli(32768,
                                              exampleTrim().tFine(519888))
                  <= 100000);

    /// The H4 / H5 split: 0xE4 = 0xFF, 0xE5 = 0x00, 0xE6 = 0xFF is dig_H4 = -16, dig_H5 = -16
    /// (a positive 0x7F pair would be 2032); an unsigned decode would read them as 4080.
    constexpr Trim humidityTrim() {
        Trim t{};
        t.loadHumidity(
          Bytes{
            std::array{std::byte{0x00},
                       std::byte{0x00},
                       std::byte{0x00},
                       std::byte{0xFF},
                       std::byte{0x00},
                       std::byte{0xFF},
                       std::byte{0x00}}
        },
          0);
        return t;
    }

    static_assert(humidityTrim().h4 == -16 && humidityTrim().h5 == -16);
}   // namespace detail

}   // namespace Kvasir::Bme280Compensation
