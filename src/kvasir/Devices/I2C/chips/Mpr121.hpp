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

/// NXP MPR121 12-key capacitive touch controller. One-byte pointer: touch status 0x00/0x01 (13 bits
/// little endian, plus OVCF in bit 15), filtered data 0x04..0x1D (13 x 10-bit little endian),
/// baseline 0x1E..0x2A, the rising/falling filter constants 0x2B..0x32 (the touched constants
/// NHDT/NCLT/FDLT 0x33..0x35 stay at reset), per-electrode touch and release thresholds from 0x41
/// in pairs, debounce 0x5B (left at its reset 0), CDC configuration 0x5C (reset 0x10), CDT configuration 0x5D (reset
/// 0x24: CDT in bits 7:5, SFI in 4:3, ESI in 2:0 with 000 = 1 ms .. 111 = 128 ms), electrode
/// configuration 0x5E and soft reset 0x80.
///
/// The part has no id register, and its default address 0x5A is the MLX90614's too. After a
/// soft reset every register reads 0 except CDC 0x5C (0x10) and CDT 0x5D (0x24) (5.1, "After
/// power on reset (POR) or soft reset"), so those two read back as 10h 24h are the check, made
/// before anything else is written (Step::identify).
///
/// Registers other than the ECR and the GPIO block may only be written in stop mode
/// (ECR = 0), so the bring-up script is: soft reset, the check, stop, the filter constants, the
/// thresholds, the two configuration bytes, and the ECR last -- which is exactly the order
/// AN3944 gives. Writing the ECR is what starts it running. The CDT byte written is 0x20
/// (CDT 0.5 us, SFI 4 samples, ESI 1 ms: a new filtered value every 4 ms), where AN3944
/// uses the reset value 0x24 (ESI 16 ms); the faster one answers a touch sooner.
///
/// An over-current on REXT (OVCF, touch status bit 15) stops the part: it clears the ECR and
/// ignores writes to it until OVCF is cleared by writing 1 to it. `ClearOverCurrent` does
/// that; `rewrite<Config>()` after it starts the electrodes again.
///
/// 0x5A..0x5D by the ADDR pin (VSS, VDD, SDA, SCL).
template<unsigned Electrodes = 12, unsigned TouchThreshold = 12, unsigned ReleaseThreshold = 6>
struct Mpr121 {
    static constexpr std::string_view Name = "MPR121";
    /// NXP MPR121. ECR (0x5E) ELE_EN 3:0: 11xxb is run mode with ELE0..ELE11 enabled, a smaller count
    /// is that many electrodes (MPR121.md:497, :798); the reset value is 0x00, stop mode, so this also says the bring-up got to its last write.
    /// What a finished bring-up leaves in the part: read by the hardware test, not the engine.
    static constexpr std::array AfterBringUp{
      RegisterCheck{"running",
                    0x5E, 1,
                    true, Electrodes == 12 ? 0x0CU : 0x0FU,
                    Electrodes == 12 ? 0x0CU : Electrodes},
    };
    static constexpr Address7    Address       = 0x5A;
    static constexpr std::size_t RegisterBytes = 1;

    static constexpr std::array<Address7, 4> Addresses{0x5A, 0x5B, 0x5C, 0x5D};

    static_assert(Electrodes >= 1 && Electrodes <= 12,
                  "the MPR121 has twelve electrodes");
    static_assert(TouchThreshold < 256 && ReleaseThreshold < 256,
                  "a threshold is one byte, and release should sit below touch");
    static_assert(ReleaseThreshold < TouchThreshold,
                  "the release threshold must be below the touch threshold or the input will "
                  "chatter");

    static constexpr auto StartupDelay = std::chrono::milliseconds{10};

    /// CL = 10 (track the baseline from the initial value), then the electrode count.
    static constexpr std::uint8_t Ecr = static_cast<std::uint8_t>(0x80U | Electrodes);

    /// Soft reset, stop, filters, thresholds, configuration, then the ECR that starts it.
    /// The part always has twelve electrode threshold pairs, whatever `Electrodes` enables.
    static constexpr unsigned ElectrodePairs = 12;

    static constexpr auto Init = [] {
        std::array<Step, 14 + 2 * ElectrodePairs> s{};
        std::size_t                               i = 0;
        s[i++]                                      = Step::write(
          {.reg = 0x80, .payload = {0x63}, .delay = std::chrono::milliseconds{1}});   // soft reset
        s[i++] = Step::read({.reg = 0x5C, .count = 2, .offset = 0});   // CDC, CDT at reset
        s[i++] = Step::identify();
        s[i++] = Step::write(
          {.reg = 0x5E, .payload = {0x00}});   // stop mode: everything below is writable
        s[i++] = Step::write({.reg = 0x2B, .payload = {0x01}});   // MHD rising
        s[i++] = Step::write({.reg = 0x2C, .payload = {0x01}});   // NHD rising
        s[i++] = Step::write({.reg = 0x2D, .payload = {0x00}});   // NCL rising
        s[i++] = Step::write({.reg = 0x2E, .payload = {0x00}});   // FDL rising
        s[i++] = Step::write({.reg = 0x2F, .payload = {0x01}});   // MHD falling
        s[i++] = Step::write({.reg = 0x30, .payload = {0x01}});   // NHD falling
        s[i++] = Step::write({.reg = 0x31, .payload = {0xFF}});   // NCL falling
        s[i++] = Step::write({.reg = 0x32, .payload = {0x02}});   // FDL falling
        for(unsigned e = 0; e < ElectrodePairs; ++e) {
            s[i++] = Step::write({.reg     = static_cast<std::uint16_t>(0x41 + 2 * e),
                                  .payload = {static_cast<std::uint8_t>(TouchThreshold)}});
            s[i++] = Step::write({.reg     = static_cast<std::uint16_t>(0x42 + 2 * e),
                                  .payload = {static_cast<std::uint8_t>(ReleaseThreshold)}});
        }
        s[i++] = Step::write({.reg = 0x5C, .payload = {0x10}});   // CDC 16 uA
        s[i++]
          = Step::write({.reg = 0x5D, .payload = {0x20}});   // CDT 0.5 us, SFI 4 samples, ESI 1 ms
        return s;
    }();

    struct Touch {
        static constexpr auto       Period = std::chrono::milliseconds{50};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2, .offset = 0})};

        struct Sample {
            std::uint16_t touched{};   ///< bit N: electrode N is touched
            bool          overCurrent{};

            [[nodiscard]] constexpr bool electrode(std::size_t n) const {
                return (touched & (1U << n)) != 0;
            }

            [[nodiscard]] constexpr bool any() const { return touched != 0; }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            auto const raw = data.le16(0);
            return {static_cast<std::uint16_t>(raw & 0x1FFFU), (raw & 0x8000U) != 0};
        }
    };

    /// The 10-bit filtered capacitance of each electrode, on demand: request<Filtered>().
    struct Filtered {
        static constexpr std::array Steps{Step::read({.reg = 0x04, .count = 26, .offset = 0})};

        struct Sample {
            std::array<std::uint16_t, 13> counts{};   ///< 12 electrodes plus the proximity one
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            for(std::size_t i = 0; i < 13; ++i) {
                sample.counts[i] = static_cast<std::uint16_t>(data.le16(2 * i) & 0x03FFU);
            }
            return sample;
        }
    };

    /// The electrode configuration register, which is also the run/stop control: writing 0
    /// stops the part, and it is the one register writable while it runs.
    using Config = Groups::InitialByte<0x5E, Ecr>;

    struct State {
        std::uint8_t cdc{};   ///< 0x5C as the reset left it
        std::uint8_t cdt{};   ///< 0x5D as the reset left it
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.cdc = data.u8(0);
        state.cdt = data.u8(1);
        return state.cdc == 0x10 && state.cdt == 0x24;
    }

    /// OVCF cleared, written 1 (touch status bit 15 is register 0x01 bit 7). Transient: a
    /// bring-up resets the part anyway.
    struct ClearOverCurrent {
        struct Value {};

        static constexpr std::size_t Bytes     = 1;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr Step encode(Value const&,
                                                   std::span<std::byte>) {
            return Step::write({.reg = 0x01, .payload = {0x80}});
        }
    };

    using Reads  = List<Touch, Filtered>;
    using Writes = List<Config, ClearOverCurrent>;
};

}   // namespace Kvasir::I2C::Chips
