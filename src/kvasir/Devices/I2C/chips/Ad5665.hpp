#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Analog Devices AD5665R quad 16-bit nanoDAC. No register pointer: every transaction is
/// three bytes, a command byte (C2:C1:C0 in bits 5:3, the DAC address in bits 2:0) and the
/// 16-bit code, big endian. Command 011 is write-and-update, so a channel is set with
/// {0x18 | channel, code >> 8, code & 0xFF}; command 111 with a payload of 0x0000 / 0x0001
/// turns the internal reference off or on. The package sets the address. The 10-lead and
/// 12-ball parts are 0b00011_A1A0 by their one ADDR pin (Table 9): VDD 0x0C, floating 0x0E,
/// GND 0x0F. The 14-lead TSSOP is 0b001_A3A2A1A0 by ADDR2 and ADDR1 (Table 10): 0x10, 0x12,
/// 0x13, 0x18, 0x1A, 0x1B, 0x1C, 0x1E, 0x1F.
struct Ad5665 {
    static constexpr std::string_view Name          = "AD5665R";
    static constexpr Address7         Address       = 0x0F;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array<Address7, 12>
      Addresses{0x0C, 0x0E, 0x0F, 0x10, 0x12, 0x13, 0x18, 0x1A, 0x1B, 0x1C, 0x1E, 0x1F};

    static constexpr auto StartupDelay = std::chrono::milliseconds{50};

    /// A read returns only the last command written, nothing that identifies the part, so
    /// the bring-up is the software reset alone -- command 101 with DB0 = 1, which also resets
    /// the LDAC, power-down and internal-reference settings (Table 16) that a warm part would
    /// otherwise keep -- and it is what an absent device is probed with. Level's Initial and
    /// a set InternalReference follow it.
    static constexpr std::array Init{
      Step::command({.payload = {0x28, 0x00, 0x01}}
      )
    };

    /// One item per channel: set<Level>(Ad5665::B, 0x8000).
    struct Level {
        using Value                          = std::uint16_t;
        static constexpr std::size_t Items   = 4;
        static constexpr std::size_t Bytes   = 3;
        static constexpr Value       Initial = 0;

        [[nodiscard]] static constexpr Step encode(Value const&         code,
                                                   std::size_t          item,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(0x18U | item);   // write and update channel
            buffer[1] = static_cast<std::byte>(code >> 8);
            buffer[2] = static_cast<std::byte>(code & 0xFF);
            return Step::commandBuffer({.offset = 0, .count = 3});
        }
    };

    /// The internal reference, 1.25 V or 2.5 V by variant. No Initial: the part powers up with it off and the
    /// application says so explicitly, but once set it is written again after every reset.
    enum class Reference : std::uint8_t { off, on };

    struct InternalReference {
        using Value                        = Reference;
        static constexpr std::size_t Bytes = 3;

        [[nodiscard]] static constexpr Step encode(Value const&         reference,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{0x38};   // command 111, DAC address don't care
            buffer[1] = std::byte{0x00};
            buffer[2] = reference == Reference::on ? std::byte{0x01} : std::byte{0x00};
            return Step::commandBuffer({.offset = 0, .count = 3});
        }
    };

    static constexpr std::size_t A = 0;
    static constexpr std::size_t B = 1;
    static constexpr std::size_t C = 2;
    static constexpr std::size_t D = 3;

    using Writes = List<Level, InternalReference>;
};

}   // namespace Kvasir::I2C::Chips
