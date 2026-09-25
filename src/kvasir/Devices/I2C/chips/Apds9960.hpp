#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace Apds9960Detail {
    /// CONTROL AGAIN (1:0): 1x, 4x, 16x, 64x.
    enum class Gain : std::uint8_t { x1 = 0, x4 = 1, x16 = 2, x64 = 3 };

    inline constexpr std::array<std::uint8_t, 4> GainValue{1, 4, 16, 64};

    /// One ATIME integration cycle.
    inline constexpr std::chrono::microseconds CycleTime{2780};

    /// The nearest whole number of cycles to `t`; 0 or above 256 is out of range.
    [[nodiscard]] constexpr std::int64_t cyclesFor(std::chrono::microseconds t) {
        return (t + CycleTime / 2) / CycleTime;
    }
}   // namespace Apds9960Detail

/// Broadcom APDS-9960 colour, ambient light and proximity sensor. A single fixed address
/// 0x39; one-byte pointer with auto-increment. ENABLE 0x80 (PON 0, AEN 1, PEN 2, WEN 3,
/// AIEN 4, PIEN 5, GEN 6), ATIME 0x81 (integration time, 2.78 ms per step counted down
/// from 256), WTIME 0x83, CONTROL 0x8F (AGAIN 1:0, PGAIN 3:2, LDRIVE 7:6), ID 0x92 (0xAB),
/// STATUS 0x93 (AVALID 0, PVALID 1, AINT 4, PINT 5, PGSAT 6, CPSAT 7), the four 16-bit
/// little-endian colour channels from CDATAL 0x94 in clear, red, green, blue order, and
/// the 8-bit proximity result PDATA 0x9C.
///
/// The gesture engine is deliberately not driven here. It is a 32-level FIFO fed by four
/// directional photodiodes that has to be drained on the interrupt pin or polled fast
/// enough not to overflow, and turning that into a gesture is a decoder with state -- both
/// of which sit outside what a chip description is. Proximity and colour, which are what
/// the part is usually bought for, are cyclic and belong here. `request<Colour>()` and the
/// AVALID flag are enough to sequence a one-shot read if that is wanted.
///
/// `Gain` is AGAIN, 1x, 4x, 16x or 64x. `Timing::IntegrationTime` (27.8 ms when left out) is
/// set as the nearest whole number of 2.78 ms ATIME cycles, 1..256; `IntegrationTime` is what
/// that comes to. The Sample carries the status with its validity bits, and no lux, so a gain
/// written at run time (Gains) needs no scale here.
///     struct Slow { static constexpr auto IntegrationTime = std::chrono::milliseconds{100}; };
///     using Light = Chips::Apds9960<Apds9960Detail::Gain::x4, Slow>;   // 36 cycles, 100.08 ms
template<Apds9960Detail::Gain Gain = Apds9960Detail::Gain::x4, typename Timing = DefaultTiming>
struct Apds9960 {
    static constexpr std::string_view Name = "APDS-9960";
    /// Broadcom APDS-9960. APDS9960.md:1034..1041, ID register 0x92, "0xAB = APDS-9960".
    /// Configuration: ENABLE (0x80) PON, bit 0, and AEN, bit 1 (APDS9960.md:455..).
    static constexpr std::array Identity{
      RegisterCheck{"id", 0x92, 1, true, 0xFF, 0xAB},
    };
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"power-and-als", 0x80, 1, true, 0x03, 0x03},
    };
    static constexpr Address7                Address = 0x39;
    static constexpr std::array<Address7, 1> Addresses{0x39};
    static constexpr std::size_t             RegisterBytes = 1;

    static constexpr std::int64_t Cycles = Apds9960Detail::cyclesFor([] {
        if constexpr(requires { Timing::IntegrationTime; }) {
            return Kvasir::asDuration<std::chrono::microseconds>(Timing::IntegrationTime);
        } else {
            return 10 * Apds9960Detail::CycleTime;
        }
    }());

    static_assert(Cycles >= 1,
                  "Timing::IntegrationTime is below half an ATIME cycle (1.39 ms)");
    static_assert(Cycles <= 256,
                  "Timing::IntegrationTime is above 256 ATIME cycles (711.68 ms)");

    /// ATIME is 256 - cycles, so 0xFF is one cycle and 0x00 is the full 256.
    static constexpr std::uint8_t Atime = static_cast<std::uint8_t>(256 - Cycles);
    static constexpr std::uint8_t Control
      = static_cast<std::uint8_t>(Gain);   // PGAIN 1x, LDRIVE 100 mA
    /// PON | AEN | PEN: powered, colour and proximity running.
    static constexpr std::uint8_t Enable = 0x07;

    static constexpr std::array<std::uint8_t, 4> GainValue = Apds9960Detail::GainValue;

    /// The integration time the part runs: Cycles x 2.78 ms.
    static constexpr std::chrono::microseconds IntegrationTime = Cycles * Apds9960Detail::CycleTime;

    /// The same in whole milliseconds, rounded up, for the waits.
    static constexpr auto IntegrationWait
      = std::chrono::ceil<std::chrono::milliseconds>(IntegrationTime);

    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    static constexpr std::array Init{
      Step::write({.reg = 0x81, .payload = {Atime}}),
      Step::write({.reg = 0x8F, .payload = {Control}}),
      Step::write({.reg     = 0x80,
                   .payload = {Enable},
                   .delay   = IntegrationWait + std::chrono::milliseconds{10}}),
    };

    struct State {
        std::uint8_t deviceId{};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once. The rest is what Init writes.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId = static_cast<std::uint8_t>(ids[0]);
    }

    struct Colour {
        static constexpr auto Period = IntegrationWait + std::chrono::milliseconds{50};

        static constexpr std::array Steps{
          Step::read({.reg = 0x93, .count = 1, .offset = 0}),   // STATUS
          Step::read({.reg = 0x94, .count = 8, .offset = 1}),   // clear, red, green, blue
          Step::read({.reg = 0x9C, .count = 1, .offset = 9}),   // proximity
          // CICLEAR, addressed with nothing written: CPSAT latches until then (STATUS bit 7:
          // "de-asserted by ... 0xE6 CICLEAR"), so every read reports the saturation since
          // the one before it, not since the first time it happened.
          Step::write({.reg = 0xE6}),
        };

        struct Sample {
            std::uint16_t clear{};
            std::uint16_t red{};
            std::uint16_t green{};
            std::uint16_t blue{};
            std::uint8_t  proximity{};
            std::uint8_t  status{};

            [[nodiscard]] constexpr bool colourValid() const { return (status & 0x01U) != 0; }

            [[nodiscard]] constexpr bool proximityValid() const { return (status & 0x02U) != 0; }

            /// The clear channel saturated since the previous read, so the colour ratios below
            /// are not meaningful.
            [[nodiscard]] constexpr bool saturated() const { return (status & 0x80U) != 0; }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            sample.status    = data.u8(0);
            sample.clear     = data.le16(1);
            sample.red       = data.le16(3);
            sample.green     = data.le16(5);
            sample.blue      = data.le16(7);
            sample.proximity = data.u8(9);
            return sample;
        }
    };

    /// ENABLE, so the application can stop the engines or turn the gesture one on itself.
    using Power = Groups::InitialByte<0x80, Enable>;

    /// AGAIN, PGAIN and the LED drive strength.
    using Gains = Groups::InitialByte<0x8F, Control>;

    using Reads  = List<Colour>;
    using Writes = List<Gains, Power>;
};

}   // namespace Kvasir::I2C::Chips
