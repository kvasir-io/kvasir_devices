#pragma once

#include "../Device.hpp"
#include "../Groups.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {
/// Bosch BMA456 triaxial accelerometer. One-byte pointer: CHIP_ID 0x00 (0x16), acceleration
/// 0x12 (six bytes, little endian X Y Z), temperature 0x22 (1 degC/LSB two's complement,
/// 0x00 is 23 degC, 0x80 is "invalid" -- Bosch's bma4_get_temperature takes the byte unsigned,
/// which the datasheet's table does not), ACC_CONF 0x40, ACC_RANGE 0x41, INIT_CTRL 0x59, PWR_CONF 0x7C,
/// PWR_CTRL 0x7D.
///
/// Bring-up is a fixed sequence and reads as one here: CHIP_ID; a softreset (CMD 0x7E = 0xB6,
/// then StartupDelay again for the boot phase), because writing INIT_CTRL = 0x01 "must not be
/// performed more than once after POR or softreset" (4.2) and a bring-up after a lost part
/// comes without a power cycle; disable advanced power save,
/// wait (450 us, so one millisecond -- a Step delay is whole milliseconds), open the
/// feature-engine config load and close it again, wait the 150 ms the engine takes, then the
/// measurement configuration and 50 ms for the first ODR cycle. The proprietary FEATURES_IN
/// blob is deliberately not uploaded: the feature engine reports init_err and the step
/// counter and gesture features do not work, but raw acceleration is unaffected.
///
/// The temperature byte reads 0x80 ("invalid") until the part's first temperature update,
/// which comes every 1.28 s: a Motion sample then carries the acceleration with
/// `temperatureValid` false, rather than being dropped.
///
/// The full-scale range is the Initial of the Range write group: `Bma456<>` is +-2 g, and a
/// range set at run time is what decode() divides by from the write's completion on.
namespace Bma456Detail {
    enum class Range : std::uint8_t { g2 = 0x00, g4 = 0x01, g8 = 0x02, g16 = 0x03 };

    /// 16384 LSB/g at +-2 g, halving with each range.
    [[nodiscard]] constexpr std::uint16_t countsPerG(std::uint8_t range) {
        return static_cast<std::uint16_t>(16384U >> (range & 0x03U));
    }

    /// The temperature byte the part reports when it has no valid temperature.
    inline constexpr std::uint8_t InvalidTemperature = 0x80;
}   // namespace Bma456Detail

template<Bma456Detail::Range Range = Bma456Detail::Range::g2>
struct Bma456 {
    static constexpr std::string_view Name = "BMA456";
    /// Bosch BMA456. BMA456.md:455: CHIP_ID (0x00) is 0x16.
    static constexpr std::array Identity{
      RegisterCheck{"chip-id", 0x00, 1, true, 0xFF, 0x16},
    };
    static constexpr Address7    Address       = 0x18;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 2> Addresses{0x18, 0x19};

    static constexpr auto StartupDelay = std::chrono::milliseconds{5};

    static constexpr std::uint8_t RangeCode = static_cast<std::uint8_t>(Range);

    /// 16384 LSB/g at +-2 g, halving with each range.
    static constexpr std::uint16_t CountsPerG = Bma456Detail::countsPerG(RangeCode);

    /// perf_mode = 1, bwp = OSR4, odr = 50 Hz.
    static constexpr std::uint8_t AccConf = 0x87;

    static constexpr std::array Init{
      Step::write({.reg = 0x7E, .payload = {0xB6}, .delay = StartupDelay}),   // CMD: softreset
      Step::write({.reg     = 0x7C,
                   .payload = {0x00},
                   .delay   = std::chrono::milliseconds{1}}),   // PWR_CONF: no power save
      Step::write({.reg = 0x59, .payload = {0x00}}),            // INIT_CTRL: begin config load
      Step::write(
        {.reg     = 0x59,
         .payload = {0x01},
         .delay   = std::chrono::milliseconds{150}}),       // and end it; the engine takes 150 ms
      Step::write({.reg = 0x40, .payload = {AccConf}}),     // ACC_CONF
      Step::write({.reg = 0x41, .payload = {RangeCode}}),   // ACC_RANGE
      Step::write({.reg     = 0x7D,
                   .payload = {0x04},
                   .delay   = std::chrono::milliseconds{50}}),   // PWR_CTRL: accelerometer on
    };

    struct State : Groups::DeviceId {
        std::uint8_t range{RangeCode};   ///< ACC_RANGE as written now
    };

    /// What the engine read of the Identity above, once it matched and before Init runs:
    /// `state().deviceId` is the part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.range    = RangeCode;
        state.deviceId = static_cast<std::uint16_t>(ids[0]);
    }

    struct Motion {
        static constexpr auto Period = std::chrono::milliseconds{20};   ///< the 50 Hz ODR

        static constexpr std::array Steps{
          Step::read({.reg = 0x12, .count = 6, .offset = 0}),   // acceleration, little endian
          Step::read({.reg = 0x22, .count = 1, .offset = 6}),   // temperature
        };

        struct Sample {
            MicroG x{};
            MicroG y{};
            MicroG z{};
            DegC   temperature{};        ///< whole degrees, 1 degC/LSB
            bool   temperatureValid{};   ///< false while the part reports 0x80
        };

        [[nodiscard]] static constexpr MicroG toAccel(std::int16_t raw,
                                                      std::uint8_t range) {
            return Units::microG(static_cast<std::int32_t>(
              static_cast<std::int64_t>(raw) * 1'000'000 / Bma456Detail::countsPerG(range)));
        }

        /// At the range the part holds now; the temperature only when its byte is not the
        /// part's "invalid" marker.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes        data,
                                                              State const& state) {
            bool const temperatureValid = data.u8(6) != Bma456Detail::InvalidTemperature;
            return Outcome<Sample>::ok({toAccel(data.s16le(0), state.range),
                                        toAccel(data.s16le(2), state.range),
                                        toAccel(data.s16le(4), state.range),
                                        temperatureValid
                                          ? Units::degC(static_cast<std::int32_t>(data.s8(6)) + 23)
                                          : Units::degC(0),
                                        temperatureValid});
        }
    };

    /// Runtime changes to the measurement registers; no Initial on Config and PowerControl,
    /// because Init has already put the configured values there. Anything set here is written
    /// again after a reset.
    template<std::uint16_t Reg>
    using Byte = Groups::Byte<Reg>;

    using Config       = Byte<0x40>;   ///< ACC_CONF: performance mode, bandwidth, ODR
    using PowerControl = Byte<0x7D>;

    /// ACC_RANGE (1:0), changeable at run time.
    struct RangeSetting {
        using Value                          = Bma456Detail::Range;
        static constexpr std::size_t Bytes   = 1;
        static constexpr Value       Initial = Range;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::writeBuffer({.reg = 0x41, .offset = 0, .count = 1});
        }

        static constexpr void applied(Value const& value,
                                      State&       state) {
            state.range = static_cast<std::uint8_t>(value);
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<Config, PowerControl, RangeSetting>;
};

}   // namespace Kvasir::I2C::Chips
