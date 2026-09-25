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

namespace Qmi8658Detail {

    /// CTRL2 bits 6..4, and the counts per g each range gives (the datasheet's aLSB/g).
    enum class AccelRange : std::uint8_t { g2 = 0x00, g4 = 0x10, g8 = 0x20, g16 = 0x30 };

    [[nodiscard]] constexpr std::int32_t countsPerG(AccelRange r) {
        switch(r) {
        case AccelRange::g2:  return 1 << 14;
        case AccelRange::g4:  return 1 << 13;
        case AccelRange::g8:  return 1 << 12;
        case AccelRange::g16: return 1 << 11;
        }
        return 1 << 12;
    }

    /// CTRL3 bits 6..4 (gFS), and the LSB per degree per second from Table 8.
    ///
    /// Both the QMI8658A Rev A and QMI8658C Rev 0.6 datasheets make 0b000 +-16 dps, and so does
    /// lewisxhe's SensorLib (SensorQMI8658.hpp: 0 is 16 dps at 16 / 32768). Some QST Arduino
    /// examples call 0b000 "32 dps" and list a "4096 dps" the part does not have; a driver
    /// built from those programs one range and scales for the next, every gyro reading out by
    /// a factor of two. This follows the datasheets.
    enum class GyroRange : std::uint8_t {
        dps16   = 0x00,
        dps32   = 0x10,
        dps64   = 0x20,
        dps128  = 0x30,
        dps256  = 0x40,
        dps512  = 0x50,
        dps1024 = 0x60,
        dps2048 = 0x70
    };

    [[nodiscard]] constexpr std::int32_t countsPerDps(GyroRange r) {
        switch(r) {
        case GyroRange::dps16:   return 2048;
        case GyroRange::dps32:   return 1024;
        case GyroRange::dps64:   return 512;
        case GyroRange::dps128:  return 256;
        case GyroRange::dps256:  return 128;
        case GyroRange::dps512:  return 64;
        case GyroRange::dps1024: return 32;
        case GyroRange::dps2048: return 16;
        }
        return 32;
    }

    /// The low nibble of CTRL2 (accelerometer) and CTRL3 (gyroscope). The two share the
    /// encoding for the high-resolution rates, which are the only ones used here. The names
    /// are the accelerometer-alone rates; with the gyroscope running too, as here, the part
    /// runs at the 6DOF rate of the same code, set by the gyroscope: 896.8, 448.4, 224.2,
    /// 112.1, 56.05 and 28.025 Hz (CTRL2 table).
    enum class Odr : std::uint8_t {
        hz1000 = 0x03,
        hz500  = 0x04,
        hz250  = 0x05,
        hz125  = 0x06,
        hz62   = 0x07,
        hz31   = 0x08
    };

    /// The chip State: WHO_AM_I, and the two range fields as CTRL2 / CTRL3 hold them now
    /// (their bits 6:4), which decode() scales by and a write changes (applied()).
    struct State {
        std::uint8_t deviceId{};      ///< WHO_AM_I (0x00)
        std::uint8_t resetResult{};   ///< 0x4D after the soft reset: 0x80 on a QMI8658A that reset
        std::uint8_t accelRange{};    ///< CTRL2 bits 6:4, an AccelRange value
        std::uint8_t gyroRange{};     ///< CTRL3 bits 6:4, a GyroRange value
    };

    [[nodiscard]] constexpr MicroG toAccel(std::int16_t raw,
                                           std::uint8_t range) {
        auto const counts = countsPerG(static_cast<AccelRange>(range & 0x30U));
        return Units::microG(
          static_cast<std::int32_t>(static_cast<std::int64_t>(raw) * 1'000'000 / counts));
    }

    [[nodiscard]] constexpr MilliDegPerSec toRate(std::int16_t raw,
                                                  std::uint8_t range) {
        auto const counts = countsPerDps(static_cast<GyroRange>(range & 0x70U));
        return Units::milliDegPerSec(
          static_cast<std::int32_t>(static_cast<std::int64_t>(raw) * 1000 / counts));
    }

}   // namespace Qmi8658Detail

/// QST QMI8658 six-axis IMU, from the QMI8658A datasheet Rev A (document 13-52-25) checked
/// against QMI8658C Rev 0.6, and lewisxhe's SensorLib for the sequence (see GyroRange above
/// for the encoding some vendor examples get wrong).
///
/// One-byte pointer that auto-increments once CTRL1 bit 6 is set, which is what lets the twelve
/// data bytes come out of one read. The SA0 polarity differs between the two parts: the QMI8658C
/// has an internal 200k pull-down, 0x6A with SA0 floating or low and 0x6B with it high; the
/// QMI8658A has an internal 200k pull-up, 0x6A with SA0 floating or high and 0x6B with it low.
/// WHO_AM_I 0x00 reads 0x05. STATUS0 0x2E has aDA (bit 0) and gDA (bit 1), which the read of the
/// data starts with: a frame with neither set is Outcome::unchanged(). RESET 0x60 = 0xB0 is the
/// soft reset the bring-up opens with; the settle after it is `SoftResetSettle`.
///
/// Little-endian 16-bit two's complement, accelerometer 0x35..0x3A then gyroscope
/// 0x3B..0x40. The magnetometer registers exist in the map but the part has no magnetometer:
/// they are for an external one on its host interface.
///
/// The template ranges are the Initial of the AccelConfig / GyroConfig write groups, so a
/// range can be changed at run time and decode() follows from the write's completion on.
template<Qmi8658Detail::AccelRange Acc  = Qmi8658Detail::AccelRange::g8,
         Qmi8658Detail::GyroRange  Gyro = Qmi8658Detail::GyroRange::dps1024,
         Qmi8658Detail::Odr        Rate = Qmi8658Detail::Odr::hz125>
struct Qmi8658 {
    static constexpr std::string_view Name = "QMI8658";
    /// QST QMI8658. qmi8658a.md:882, :1033: WHO_AM_I (0x00) is 0x05.
    static constexpr std::array Identity{
      RegisterCheck{"who-am-i", 0x00, 1, true, 0xFF, 0x05},
    };
    static constexpr Address7                Address = 0x6A;
    static constexpr std::array<Address7, 2> Addresses{0x6A, 0x6B};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::uint8_t Ctrl2
      = static_cast<std::uint8_t>(static_cast<std::uint8_t>(Acc) | static_cast<std::uint8_t>(Rate));
    static constexpr std::uint8_t Ctrl3 = static_cast<std::uint8_t>(
      static_cast<std::uint8_t>(Gyro) | static_cast<std::uint8_t>(Rate));

    /// After the soft reset. The QMI8658A finishes its reset in 15 ms at most (7.4, System Turn
    /// On Time in Tables 7 and 8); the QMI8658C lists a System Turn On Time of 1.75 s from
    /// software reset (Tables 7 and 8).
    static constexpr std::chrono::milliseconds SoftResetSettle{20};

    /// CTRL1 = 0x40: ADDR_AI (bit 6) so the twelve data bytes come out of one read, and BE
    /// (bit 5) *cleared* for little-endian, which is what the AX_L/AX_H register naming and
    /// the s16le decode below assume. The part's reset value is 0x20 -- big-endian -- and
    /// SensorLib keeps BE set (0x60) while it still assembles the data bytes low-first, which
    /// says BE does not reach the data registers in practice; the bit is written clear anyway,
    /// so what is written matches the decode.
    ///
    /// Then the two ranges with their rate, the low-pass filters off (CTRL5 = 0x00), and
    /// CTRL7 = 0x03 to enable accelerometer and gyroscope together (aEN | gEN), followed by
    /// the gyroscope's turn-on time, 150 ms + 3/ODR (Table 8), before the first read.
    ///
    /// WHO_AM_I is read first and checked (Step::identify) before anything is written: 0x6A
    /// and 0x6B are also the LSM6DS3's addresses. Register 0x4D is read right after the reset,
    /// before CTRL7 overwrites it; the QMI8658A sets it to 0x80 after a successful reset
    /// (RESET register, 7.4), and State keeps it -- the QMI8658C documents no such value there
    /// (dQY_L), so it is not a condition of the bring-up.
    [[nodiscard]] static constexpr std::chrono::milliseconds sixDofPeriod(Qmi8658Detail::Odr odr) {
        switch(odr) {
        case Qmi8658Detail::Odr::hz1000: return std::chrono::milliseconds{2};
        case Qmi8658Detail::Odr::hz500:  return std::chrono::milliseconds{3};
        case Qmi8658Detail::Odr::hz250:  return std::chrono::milliseconds{5};
        case Qmi8658Detail::Odr::hz125:  return std::chrono::milliseconds{9};
        case Qmi8658Detail::Odr::hz62:   return std::chrono::milliseconds{18};
        case Qmi8658Detail::Odr::hz31:   return std::chrono::milliseconds{36};
        }
        return std::chrono::milliseconds{36};
    }

    static constexpr std::array Init{
      Step::write(
        {.reg = 0x60, .payload = {0xB0}, .delay = SoftResetSettle}),   // RESET, then the settle
      Step::read({.reg = 0x4D, .count = 1, .offset = 0}),              // the reset result
      Step::write({.reg = 0x02, .payload = {0x40}}),
      Step::write({.reg = 0x03, .payload = {Ctrl2}}),
      Step::write({.reg = 0x04, .payload = {Ctrl3}}),
      Step::write({.reg = 0x06, .payload = {0x00}}),
      Step::write({.reg     = 0x08,
                   .payload = {0x03},
                   .delay   = std::chrono::milliseconds{150} + 3 * sixDofPeriod(Rate)
                            + std::chrono::milliseconds{10}}),
    };

    using State = Qmi8658Detail::State;

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId   = static_cast<std::uint8_t>(ids[0]);
        state.accelRange = static_cast<std::uint8_t>(Acc);
        state.gyroRange  = static_cast<std::uint8_t>(Gyro);
    }

    /// What the reset left in the reset-result register, read after it.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.resetResult = data.u8(0);
        return true;
    }

    struct Motion {
        static constexpr auto Period = std::chrono::milliseconds{50};
        /// So the frames that said nothing new are counted (unchanged<Motion>()).
        static constexpr bool Timestamped = true;

        /// STATUS0, then twelve bytes from 0x35 in one go, which the auto-increment set at
        /// bring-up allows.
        static constexpr std::array Steps{Step::read({.reg = 0x2E, .count = 1, .offset = 0}),
                                          Step::read({.reg = 0x35, .count = 12, .offset = 1})};

        struct Sample {
            std::array<MicroG, 3>         accel{};
            std::array<MilliDegPerSec, 3> gyro{};
        };

        static constexpr std::int32_t CountsPerG   = Qmi8658Detail::countsPerG(Acc);
        static constexpr std::int32_t CountsPerDps = Qmi8658Detail::countsPerDps(Gyro);

        /// At the template ranges.
        [[nodiscard]] static constexpr MicroG toAccel(std::int16_t raw) {
            return Qmi8658Detail::toAccel(raw, static_cast<std::uint8_t>(Acc));
        }

        [[nodiscard]] static constexpr MilliDegPerSec toRate(std::int16_t raw) {
            return Qmi8658Detail::toRate(raw, static_cast<std::uint8_t>(Gyro));
        }

        /// At the ranges the part holds now.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            if((data.u8(0) & 0x03U) == 0) { return Outcome<Sample>::unchanged(); }   // aDA, gDA
            auto const a = [&](std::size_t i) {
                return Qmi8658Detail::toAccel(data.s16le(1 + 2 * i), state.accelRange);
            };
            auto const g = [&](std::size_t i) {
                return Qmi8658Detail::toRate(data.s16le(7 + 2 * i), state.gyroRange);
            };
            return Outcome<Sample>::ok({
              {a(0), a(1), a(2)},
              {g(0), g(1), g(2)}
            });
        }
    };

    /// CTRL2: the accelerometer's range (6:4) and ODR (3:0), changeable at run time.
    struct AccelConfig {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Ctrl2;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x03, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.accelRange = static_cast<std::uint8_t>(value & 0x30U);
        }
    };

    /// CTRL3: the gyroscope's range (6:4) and ODR (3:0).
    struct GyroConfig {
        using Value                          = std::uint8_t;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Ctrl3;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x04, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.gyroRange = static_cast<std::uint8_t>(value & 0x70U);
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<AccelConfig, GyroConfig>;
};

}   // namespace Kvasir::I2C::Chips
