#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Htu21dDetail {
    /// A 3-byte result: 14-bit value with two status bits in the low LSBs, then a CRC-8
    /// (polynomial 0x31, initialised to 0x00: HTU21D "CRC with I2C protocol"). Examples
    /// from the datasheet: CRC(0x683A) = 0x7C, CRC(0x4E85) = 0x6B.
    [[nodiscard]] constexpr bool wordOk(Bytes       data,
                                        std::size_t i) {
        return Sensirion::crc8(data.sub(i, 2), 0x00) == data.u8(i + 2);
    }

    static_assert(Sensirion::crc8(
                    Bytes{
                      std::array{std::byte{0x68},
                                 std::byte{0x3A}}
    },
                    0x00)
                  == 0x7C);
    static_assert(Sensirion::crc8(
                    Bytes{
                      std::array{std::byte{0x4E},
                                 std::byte{0x85}}
    },
                    0x00)
                  == 0x6B);

    /// Bit 1 of a result word says which measurement it is: 0 temperature, 1 humidity
    /// ("Status bits"). Bit 0 is reserved.
    [[nodiscard]] constexpr bool isHumidity(std::uint16_t word) { return (word & 0x0002U) != 0; }

    struct Sample {
        CentiDegC    temperature{};
        CentiPercent humidity{};
    };

    /// T = -46.85 + 175.72 S / 2^16, RH = -6 + 125 S / 2^16 ("Conversion of signal
    /// outputs"; 0x7C80 -> 54.8 %RH), status bits masked, humidity clamped to 0..100. A
    /// word whose status bit names the other measurement -- a slipped command, a read that
    /// answered the previous trigger -- is rejected, as is a bad CRC, and so are the two
    /// diagnostic words ("Diagnostic Status"): all zeros, an open circuit, and all ones, a
    /// short, which would otherwise read -46.85 degC or 128.87 degC.
    [[nodiscard]] constexpr Outcome<Sample> decode(Bytes data) {
        if(!wordOk(data, 0) || !wordOk(data, 3)) { return Outcome<Sample>::reject(); }
        auto const wt         = data.be16(0);
        auto const wh         = data.be16(3);
        auto const diagnostic = [](std::uint16_t w) { return w == 0x0000U || w == 0xFFFFU; };
        if(diagnostic(wt) || diagnostic(wh)) { return Outcome<Sample>::reject(); }
        if(isHumidity(wt) || !isHumidity(wh)) { return Outcome<Sample>::reject(); }
        auto const st = static_cast<std::uint16_t>(wt & 0xFFFC);
        auto const sh = static_cast<std::uint16_t>(wh & 0xFFFC);
        Sample     sample{};
        sample.temperature
          = Units::centiDegC(-4685 + static_cast<std::int32_t>((17572LL * st) >> 16));
        auto const rh = -600 + static_cast<std::int32_t>((12500LL * sh) >> 16);
        sample.humidity
          = Units::centiPercent(static_cast<std::uint32_t>(rh < 0 ? 0 : (rh > 10000 ? 10000 : rh)));
        return Outcome<Sample>::ok(sample);
    }
}   // namespace Htu21dDetail

/// TE HTU21D(F) (datasheet 05/2017). Fixed address 0x40, no registers: a one-byte
/// command, then a bare read of the 3-byte result. "No hold master" mode so the bus is
/// free while it measures: trigger temperature 0xF3 (50 ms max at 14 bit), read 3; trigger
/// humidity 0xF5 (16 ms max at 12 bit), read 3. Soft reset 0xFE takes under 15 ms.
struct Htu21d {
    static constexpr std::string_view        Name    = "HTU21D";
    static constexpr Address7                Address = 0x40;
    static constexpr std::array<Address7, 1> Addresses{0x40};
    static constexpr std::size_t             RegisterBytes = 0;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{15};

    static constexpr std::array Init{
      Step::command({.payload = {0xFE}, .delay = std::chrono::milliseconds{15}})};

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Step::command({.payload = {0xF3}, .delay = std::chrono::milliseconds{50}}),
          Step::receive({.count = 3, .offset = 0}),
          Step::command({.payload = {0xF5}, .delay = std::chrono::milliseconds{16}}),
          Step::receive({.count = 3, .offset = 3})};
        using Sample = Htu21dDetail::Sample;

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            return Htu21dDetail::decode(data);
        }
    };

    using Reads = List<Measurement>;
};

/// Silicon Labs Si7021 (Si7021-A20 datasheet): the HTU21D's command set, plus an
/// electronic ID. The second access 0xFC 0xC9 returns SNB_3, SNB_2, CRC, SNB_1, SNB_0, CRC
/// (5.3; the same CRC-8 as the measurements, from 0x00), and SNB_3 is the device id: 0x15
/// Si7021, 0x14 Si7020, 0x0D Si7013. 80 ms power-up.
///
/// The timing differs: a humidity trigger also converts temperature, so it takes tCONV(RH) +
/// tCONV(T), 12 ms + 10.8 ms at the default resolution (Table 2 note 1), and in no-hold mode
/// the part NAKs the read until then. Temperature alone is 10.8 ms.
struct Si7021 : Htu21d {
    static constexpr std::string_view Name         = "Si7021";
    static constexpr auto             StartupDelay = std::chrono::milliseconds{80};

    struct Measurement {
        static constexpr auto       Period = std::chrono::milliseconds{1000};
        static constexpr std::array Steps{
          Step::command({.payload = {0xF3}, .delay = std::chrono::milliseconds{12}}),
          Step::receive({.count = 3, .offset = 0}),
          Step::command({.payload = {0xF5}, .delay = std::chrono::milliseconds{25}}),
          Step::receive({.count = 3, .offset = 3})};
        using Sample = Htu21dDetail::Sample;

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            return Htu21dDetail::decode(data);
        }
    };

    using Reads = List<Measurement>;

    static constexpr std::array Init{
      Step::command({.payload = {0xFE}, .delay = std::chrono::milliseconds{15}}
      ),
      Step::command({.payload = {0xFC, 0xC9}}
      ),
      Step::receive({.count = 6, .offset = 0}
      ),
    };

    struct State {
        std::uint8_t  deviceId{};   ///< SNB_3
        std::uint32_t serialB{};    ///< SNB_3..SNB_0, the second half of the serial number
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.deviceId = data.u8(0);
        state.serialB  = (static_cast<std::uint32_t>(data.be16(0)) << 16) | data.be16(3);
        if(!Htu21dDetail::wordOk(data, 0) || !Htu21dDetail::wordOk(data, 3)) { return false; }
        return state.deviceId == 0x15 || state.deviceId == 0x14 || state.deviceId == 0x0D;
    }
};

}   // namespace Kvasir::I2C::Chips
