#pragma once

#include "Aht20.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Aosong DHT20 (product manual V1.0). The AHT20's silicon in a housing with four pins, and
/// its measurement: 0xAC 0x33 0x00, 80 ms, seven bytes with the CRC -- Aht20::Measurement as
/// it stands. What differs is the bring-up (manual 7.4): no 0xBE initialisation, and no soft
/// reset (Linux aht10.c sends 0x71 0x28 0x00 as a DHT20 "init" command -- 0x71 is the
/// manual's 8-bit read address, not a command); 100 ms after power the status byte is read, and if bits 3 and 4 (0x18) are not
/// both set, the calibration registers 0x1B, 0x1C and 0x1E are to be restored first.
///
/// That restore is the vendor's routine, never explained (Aosong's sample code, as RobTillaart's
/// DHT20 library and Zephyr carry it): per register, write `reg 0x00 0x00`, 5 ms, read three
/// bytes, 10 ms, write `0xB0 | reg` and the second and third of them back, 5 ms -- 10 ms after
/// the last of the three. It is only to be run on a part that asks for it, and a Step script cannot branch, so
/// it is the on-demand group `Restore` rather than part of Init: the application looks at
/// `calibrated()` once the part is up and requests it if not. Restore reads the status
/// again at its end, which is its Sample. Fixed at 0x38.
///
/// RegisterBytes is 1 only so the write-back can take its two bytes from the buffer behind a
/// register byte (0xBB, 0xBC, 0xBE); every other step is a plain command or read.
struct Dht20 {
    static constexpr std::string_view        Name    = "DHT20";
    static constexpr Address7                Address = 0x38;
    static constexpr std::array<Address7, 1> Addresses{0x38};
    static constexpr std::size_t             RegisterBytes = 1;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{100};

    static constexpr std::uint8_t CalibratedBits = 0x18;

    static constexpr std::array Init{Step::receive({.count = 1, .offset = 0})};

    struct State {
        std::uint8_t status{};

        [[nodiscard]] constexpr bool calibrated() const {
            return (status & CalibratedBits) == CalibratedBits;
        }
    };

    /// Always accepted: a part that lacks the calibration bits is still a DHT20, it only
    /// needs Restore before its readings mean anything.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.status = data.u8(0);
        return true;
    }

    /// The AHT20's measurement, every two seconds: the DHT20 datasheet asks for at least 2 s
    /// between measurements so the sensor does not heat itself (Linux aht10.c enforces the
    /// same minimum).
    struct Measurement : Aht20::Measurement {
        static constexpr auto Period = std::chrono::milliseconds{2000};
    };

    struct Restore {
        static constexpr std::array Steps{
          Step::command({.payload = {0x1B, 0x00, 0x00}, .delay = std::chrono::milliseconds{5}}
          ),
          Step::receive({.count = 3, .offset = 0, .delay = std::chrono::milliseconds{10}}
          ),
          Step::writeBuffer(
            {.reg = 0xBB, .offset = 1, .count = 2, .delay = std::chrono::milliseconds{5}}
          ),
          Step::command({.payload = {0x1C, 0x00, 0x00}, .delay = std::chrono::milliseconds{5}}
          ),
          Step::receive({.count = 3, .offset = 3, .delay = std::chrono::milliseconds{10}}
          ),
          Step::writeBuffer(
            {.reg = 0xBC, .offset = 4, .count = 2, .delay = std::chrono::milliseconds{5}}
          ),
          Step::command({.payload = {0x1E, 0x00, 0x00}, .delay = std::chrono::milliseconds{5}}
          ),
          Step::receive({.count = 3, .offset = 6, .delay = std::chrono::milliseconds{10}}
          ),
          Step::writeBuffer(
            {.reg = 0xBE, .offset = 7, .count = 2, .delay = std::chrono::milliseconds{10}}
          ),
          Step::receive({.count = 1, .offset = 9}
          ),
        };

        struct Sample {
            std::uint8_t status{};   ///< read after the restore

            [[nodiscard]] constexpr bool calibrated() const {
                return (status & CalibratedBits) == CalibratedBits;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return {data.u8(9)}; }
    };

    using Reads = List<Measurement, Restore>;
};

}   // namespace Kvasir::I2C::Chips
