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

namespace Hmc5883lDetail {
    /// Configuration register A, DO (4:2): the continuous-measurement output rate.
    enum class Rate : std::uint8_t {
        hz0_75 = 0,
        hz1_5  = 1,
        hz3    = 2,
        hz7_5  = 3,
        hz15   = 4,
        hz30   = 5,
        hz75   = 6
    };

    /// Configuration register B, GN (7:5): the gain, as counts per gauss.
    enum class Gain : std::uint8_t {
        g1370 = 0,   ///< +-0.88 Ga
        g1090 = 1,   ///< +-1.3 Ga, the reset value
        g820  = 2,   ///< +-1.9 Ga
        g660  = 3,   ///< +-2.5 Ga
        g440  = 4,   ///< +-4.0 Ga
        g390  = 5,   ///< +-4.7 Ga
        g330  = 6,   ///< +-5.6 Ga
        g230  = 7,   ///< +-8.1 Ga
    };

    /// One period of each rate, rounded up to whole milliseconds.
    inline constexpr std::array<std::chrono::milliseconds, 7> RatePeriod{
      std::chrono::milliseconds{1334},
      std::chrono::milliseconds{667},
      std::chrono::milliseconds{334},
      std::chrono::milliseconds{134},
      std::chrono::milliseconds{67},
      std::chrono::milliseconds{34},
      std::chrono::milliseconds{14}};

    inline constexpr std::array<std::uint16_t, 8>
      CountsPerGauss{1370, 1090, 820, 660, 440, 390, 330, 230};
}   // namespace Hmc5883lDetail

/// Honeywell HMC5883L. Fixed address 0x1E, one-byte pointer, auto-increment. Bring-up:
/// identification 0x0A..0x0C = "H43"; configuration A 0x00 = 0x70 (8 samples averaged,
/// 15 Hz, normal bias); B 0x01 = 0x20 (gain 1090 LSb/gauss); mode 0x02 = 0x00
/// (continuous). Data: six bytes from 0x03 in the order X, Z, Y, int16 big-endian, then
/// status 0x09 (RDY bit 0, LOCK bit 1) in the same burst. -4096 is an overflow (Data Output
/// Register Operation) and the frame is rejected; a frame without RDY is Outcome::unchanged().
/// The period follows the output rate, so every conversion is read once.
template<Hmc5883lDetail::Rate Rate = Hmc5883lDetail::Rate::hz15,
         Hmc5883lDetail::Gain Gain = Hmc5883lDetail::Gain::g1090>
struct Hmc5883lX {
    static constexpr std::string_view Name = "HMC5883L";
    /// Honeywell HMC5883L. HMC5883L.md:594..626: identification registers A, B, C (10..12) read the
    /// ASCII characters H, 4, 3.
    static constexpr std::array Identity{
      RegisterCheck{"id-a", 0x0A, 1, true, 0xFF, 'H'},
      RegisterCheck{"id-b", 0x0B, 1, true, 0xFF, '4'},
      RegisterCheck{"id-c", 0x0C, 1, true, 0xFF, '3'},
    };
    static constexpr Address7                Address = 0x1E;
    static constexpr std::array<Address7, 1> Addresses{0x1E};
    static constexpr std::size_t             RegisterBytes = 1;

    /// 8 samples averaged (MA 6:5 = 11), the rate, normal bias.
    static constexpr std::uint8_t ConfigA
      = static_cast<std::uint8_t>(0x60U | (static_cast<unsigned>(Rate) << 2U));
    static constexpr std::uint8_t ConfigB
      = static_cast<std::uint8_t>(static_cast<unsigned>(Gain) << 5U);

    static constexpr std::uint16_t CountsPerGauss
      = Hmc5883lDetail::CountsPerGauss[static_cast<std::size_t>(Gain)];
    static constexpr std::chrono::milliseconds OutputPeriod
      = Hmc5883lDetail::RatePeriod[static_cast<std::size_t>(Rate)];

    static constexpr std::array Init{
      Step::write({.reg = 0x00, .payload = {ConfigA}}),
      Step::write({.reg = 0x01, .payload = {ConfigB}}),
      Step::write({.reg = 0x02, .payload = {0x00}, .delay = std::chrono::milliseconds{7}}),
      // The measurement the part took at power-up, at the default gain, holds RDY until it is
      // read, and the first measurement after a gain change still uses the previous gain
      // (Configuration Register B): both are read out here and dropped, and the first sample
      // kept is two output periods on (Mode Register, "first set of data after 2/fDO").
      Step::read({.reg = 0x03, .count = 6, .offset = 0, .delay = 2 * OutputPeriod}),
    };

    struct State {
        std::array<char, 3> deviceId{};
    };

    /// What the engine read of the Identity above, once it matched and before Init runs: the
    /// part's own word, read once.
    static constexpr void identified(std::span<std::uint32_t const> ids,
                                     State&                         state) {
        state.deviceId
          = {static_cast<char>(ids[0]), static_cast<char>(ids[1]), static_cast<char>(ids[2])};
    }

    struct Field {
        static constexpr auto       Period = OutputPeriod;
        static constexpr std::array Steps{
          Step::read({.reg = 0x03, .count = 7})};   // X, Z, Y, status

        struct Sample {
            NanoTesla x{}, y{}, z{};
        };

        /// `CountsPerGauss` counts per gauss, and a gauss is 100'000 nT.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data,
                                                              Sample const&) {
            if((data.u8(6) & 0x01U) == 0) { return Outcome<Sample>::unchanged(); }   // RDY
            auto const rx = data.s16be(0);
            auto const rz = data.s16be(2);
            auto const ry = data.s16be(4);
            if(rx == -4096 || ry == -4096 || rz == -4096) { return Outcome<Sample>::reject(); }
            auto const nT = [](std::int16_t raw) {
                return Units::nanoTesla(static_cast<std::int64_t>(raw) * 100'000 / CountsPerGauss);
            };
            return Outcome<Sample>::ok({nT(rx), nT(ry), nT(rz)});
        }
    };

    using Reads = List<Field>;
};

/// The part at its reset configuration: 15 Hz and 1090 counts per gauss.
using Hmc5883l = Hmc5883lX<>;

}   // namespace Kvasir::I2C::Chips
