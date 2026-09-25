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

/// Texas Instruments LMK1D1208I clock buffer (SNAS828A). The command byte is bit 7 (a byte read or
/// write, Table 9-9 in 9.5) plus a 7-bit register address, so the registers are addressed as 0x80 +
/// n: R0 0x80 output enable, one bit per output (reset 0x00); R1 0x81 amplitude select, one bit per
/// output (reset 0x00); R2 0x82 bank control (reset 0xF1, Table 9-14) with bits 1:0 the input
/// enables, 3:2 the bank mutes, 5:4 the bank input selects and bits 7:6 reserved, which "can be
/// written to 1" and read back what was written; R5 0x85 the read-only identification (Table 9-15,
/// reset 0x20): REV_ID in bits 7:4, DEV_ID in 3:0.
///
/// Writes are verified by read-back: the part is configuration only, nothing polls it, and
/// a clock buffer that silently loses its output-enable byte takes the board with it. Every
/// write is read back 100 ms later and again every second. `BankControl` keeps the reserved
/// bits at 1 whatever it is given and leaves them out of the comparison.
///
/// The I2C address is set by strapping, so it is a template parameter with no default:
/// `Lmk1d1208i<0x68>`. `Timing::StartupDelay` is the wait after power before the first transaction:
/// the datasheet gives no figure and 500 ms is a conservative default.
template<Address7 Addr, typename Timing = DefaultTiming>
struct Lmk1d1208i {
    static constexpr std::string_view Name          = "LMK1D1208I";
    static constexpr Address7         Address       = Addr;
    static constexpr std::size_t      RegisterBytes = 1;

    static constexpr std::array<Address7, 1> Addresses{Addr};

    static constexpr std::chrono::milliseconds StartupDelay = [] {
        if constexpr(requires { Timing::StartupDelay; }) {
            return Kvasir::asDuration(Timing::StartupDelay);
        } else {
            return std::chrono::milliseconds{500};
        }
    }();

    /// R5 DEV_ID (bits 3:0): 0 on this part (Table 9-15).
    static constexpr std::uint8_t DeviceId = 0x0;

    /// The identification read is the presence probe.
    static constexpr std::array Init{Step::read({.reg = 0x85, .count = 1, .offset = 0})};

    struct State {
        std::uint8_t deviceId{};   ///< R5 bits 3:0
        std::uint8_t revision{};   ///< R5 bits 7:4, recorded and not checked
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.deviceId = static_cast<std::uint8_t>(data.u8(0) & 0x0FU);
        state.revision = static_cast<std::uint8_t>(data.u8(0) >> 4);
        return state.deviceId == DeviceId;
    }

    /// R5, so the id is also visible after bring-up and the part is polled at all.
    struct Identity {
        static constexpr auto       Period = std::chrono::milliseconds{5000};
        static constexpr std::array Steps{Step::read({.reg = 0x85, .count = 1, .offset = 0})};

        struct Sample {
            std::uint8_t deviceId{};   ///< R5 bits 3:0
            std::uint8_t revision{};   ///< R5 bits 7:4
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {static_cast<std::uint8_t>(data.u8(0) & 0x0FU),
                    static_cast<std::uint8_t>(data.u8(0) >> 4)};
        }
    };

    /// Read back 100 ms after a write, then every second.
    struct ByteTiming {
        static constexpr std::chrono::milliseconds VerifyDelay{100};
        static constexpr std::chrono::milliseconds VerifyInterval{1000};
    };

    /// One configuration register, written back and checked.
    template<std::uint16_t Reg, std::uint8_t Init>
    using Byte = Groups::VerifiedByte<Reg, Init, ByteTiming>;

    using OutputEnable = Byte<0x80, 0x00>;   ///< one bit per output
    using Amplitude    = Byte<0x81, 0x00>;   ///< one bit per output, 1 = boosted

    /// R2. Bits 7:6 are reserved and must stay 1 ("writing a different value than 1 will
    /// affect device functionality"), so they are forced on the way out and masked on the
    /// way back: a value with them clear is written and verified as if they were set.
    struct BankControl : Byte<0x82, 0xF1> {
        static constexpr std::uint8_t ReservedBits = 0xC0;

        [[nodiscard]] static constexpr Step encode(std::uint8_t const&  value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value | ReservedBits);
            return Step::writeBuffer({.reg = 0x82, .offset = 0, .count = 1});
        }

        [[nodiscard]] static constexpr bool verify(Kvasir::I2C::Bytes written,
                                                   Kvasir::I2C::Bytes readBack) {
            return (written.u8(0) & ~ReservedBits) == (readBack.u8(0) & ~ReservedBits);
        }
    };

    /// BankControl bit positions.
    static constexpr std::uint8_t InputEnableShift = 0;   ///< IN0, IN1
    static constexpr std::uint8_t BankMuteShift    = 2;   ///< bank 0, bank 1
    static constexpr std::uint8_t BankInSelShift   = 4;   ///< set = IN0, clear = IN1

    using Reads  = List<Identity>;
    using Writes = List<OutputEnable, Amplitude, BankControl>;
};

}   // namespace Kvasir::I2C::Chips
