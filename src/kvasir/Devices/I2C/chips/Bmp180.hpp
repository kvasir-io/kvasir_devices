#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Bmp180Detail {
    /// oversampling_setting (Table 8): the number of pressure samples averaged, and with it
    /// the conversion time (4.5, 7.5, 13.5, 25.5 ms) and the result width (16 + oss bits).
    enum class Oversampling : std::uint8_t { x1 = 0, x2 = 1, x4 = 2, x8 = 3 };
}   // namespace Bmp180Detail

/// Bosch BMP180 (BST-BMP180-DS000-09). Fixed address 0x77, one-byte registers. Bring-up:
/// id 0xD0 = 0x55, the eleven 16-bit big-endian calibration words from 0xAA (Table 5).
/// A measurement is two conversions (Figure 3): control 0xF4 = 0x2E (temperature, 4.5 ms),
/// read 0xF6..0xF7; control 0xF4 = 0x34 | oss << 6 (pressure; 4.5, 7.5, 13.5, 25.5 ms for
/// oss 0..3, Table 8), read 0xF6..0xF8 (16 + oss bits). Compensation is the datasheet's
/// algorithm (Figure 4), whose worked example (AC1 408 ... UT 27898, UP 23843) gives
/// 15.0 degC and 69964 Pa. `Bmp180<>` is oss 1 (x2).
template<Bmp180Detail::Oversampling Oss = Bmp180Detail::Oversampling::x2>
struct Bmp180 {
    static constexpr std::string_view Name = "BMP180";
    /// Bosch BME280, BMP280, BMP180: the id register 0xD0 reads 0x60 (BME280.md:1106), 0x58
    /// (BMP280.md:989) and 0x55 (BMP180.md:421).
    static constexpr std::array Identity{
      RegisterCheck{"chip-id", 0xD0, 1, true, 0xFF, 0x55},
    };
    static constexpr Address7                Address = 0x77;
    static constexpr std::array<Address7, 1> Addresses{0x77};
    static constexpr std::size_t             RegisterBytes = 1;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{10};

    using Oversampling = Bmp180Detail::Oversampling;

    static constexpr unsigned OssBits = static_cast<unsigned>(Oss);

    static constexpr std::array Init{
      Step::read({.reg = 0xAA, .count = 22, .offset = 0}),
    };

    struct State {
        std::uint8_t  deviceId{};
        std::int16_t  ac1{}, ac2{}, ac3{};
        std::uint16_t ac4{}, ac5{}, ac6{};
        std::int16_t  b1{}, b2{}, mb{}, mc{}, md{};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint8_t>(ids[0]);
    }

    /// The identity is the engine's; what is left is whether the calibration block is a real one.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.ac1 = data.s16be(0);
        state.ac2 = data.s16be(2);
        state.ac3 = data.s16be(4);
        state.ac4 = data.be16(6);
        state.ac5 = data.be16(8);
        state.ac6 = data.be16(10);
        state.b1  = data.s16be(12);
        state.b2  = data.s16be(14);
        state.mb  = data.s16be(16);
        state.mc  = data.s16be(18);
        state.md  = data.s16be(20);
        bool ok   = true;
        for(std::size_t i = 0; i < 22; i += 2) {
            auto const w = data.be16(i);
            ok = ok && w != 0 && w != 0xFFFF;   // 3.4: a word of 0 or 0xFFFF is a bad read
        }
        return ok;
    }

    /// Table 8's maximum conversion times, 4.5, 7.5, 13.5 and 25.5 ms, plus a millisecond each.
    static constexpr std::chrono::milliseconds PressureWait{OssBits == 0   ? 6
                                                            : OssBits == 1 ? 9
                                                            : OssBits == 2 ? 15
                                                                           : 27};

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Step::write({.reg = 0xF4, .payload = {0x2E}, .delay = std::chrono::milliseconds{6}}),
          Step::read({.reg = 0xF6, .count = 2, .offset = 0}),
          Step::write({.reg     = 0xF4,
                       .payload = {static_cast<std::uint8_t>(0x34U | (OssBits << 6))},
                       .delay   = PressureWait}),
          Step::read({.reg = 0xF6, .count = 3, .offset = 2}),
        };

        struct Sample {
            DeciDegC temperature{};   ///< 0.1 degC
            Pascal   pressure{};
        };

        /// The datasheet's algorithm. Two of its divisors, `x1 + md` and `b4`, are zero for
        /// the frames a floating or stuck bus produces (all 0x00, all 0xFF) with typical
        /// calibration; those frames are rejected rather than divided by.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& c) {
            std::int32_t const ut = data.be16(0);
            std::int32_t const up = static_cast<std::int32_t>(data.be24(2) >> (8 - OssBits));

            // temperature
            std::int32_t x1
              = ((ut - static_cast<std::int32_t>(c.ac6)) * static_cast<std::int32_t>(c.ac5)) >> 15;
            if(x1 + c.md == 0) { return Outcome<Sample>::reject(); }
            std::int32_t       x2 = (static_cast<std::int32_t>(c.mc) << 11) / (x1 + c.md);
            std::int32_t const b5 = x1 + x2;
            Sample             sample{};
            sample.temperature = Units::deciDegC((b5 + 8) >> 4);

            // pressure
            std::int32_t const b6 = b5 - 4000;
            x1                    = (c.b2 * ((b6 * b6) >> 12)) >> 11;
            x2                    = (c.ac2 * b6) >> 11;
            std::int32_t       x3 = x1 + x2;
            std::int32_t const b3
              = (((static_cast<std::int32_t>(c.ac1) * 4 + x3) << OssBits) + 2) >> 2;
            x1 = (c.ac3 * b6) >> 13;
            x2 = (c.b1 * ((b6 * b6) >> 12)) >> 16;
            x3 = ((x1 + x2) + 2) >> 2;
            std::uint32_t const b4
              = (static_cast<std::uint32_t>(c.ac4) * static_cast<std::uint32_t>(x3 + 32768)) >> 15;
            if(b4 == 0) { return Outcome<Sample>::reject(); }
            std::uint32_t const b7
              = (static_cast<std::uint32_t>(up) - static_cast<std::uint32_t>(b3))
              * (50000U >> OssBits);
            std::int32_t p  = b7 < 0x80000000U ? static_cast<std::int32_t>((b7 * 2) / b4)
                                               : static_cast<std::int32_t>((b7 / b4) * 2);
            x1              = (p >> 8) * (p >> 8);
            x1              = (x1 * 3038) >> 16;
            x2              = (-7357 * p) >> 16;
            p               = p + ((x1 + x2 + 3791) >> 4);
            sample.pressure = Units::pascal(static_cast<std::uint32_t>(p));
            return Outcome<Sample>::ok(sample);
        }
    };

    using Reads = List<Measurement>;
};

}   // namespace Kvasir::I2C::Chips
