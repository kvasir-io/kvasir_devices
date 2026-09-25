#pragma once

#include "../Device.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// The I2C general call (address 0, UM10204 3.1.13), as a description: one byte that every
/// part on the segment listening to it receives. The specification gives 0x06 (reset, and
/// sample the address pins) and 0x04 (sample them) a meaning; a part may define more. The
/// TLV493D-A1B6 resets on a general call, and the SDA level of the byte's bits picks the
/// address it comes back at: 0xFF (SDA high) for 0x5E, 0x00 for 0x1F (Infineon's
/// Tlv493d::resetSensor).
///
/// Behind a MuxGate only the parts on that one channel hear it, which is what makes it safe
/// to use on a bus with many parts. A part that does not listen to the general call NAKs,
/// as the engine counts any NAK; three in a row park the description, which costs nothing for
/// something that is only ever written on demand.
///
///     using ResetCh6 = OnChannel<Chips::GeneralCall, 6>;
///     bus.get<ResetCh6>().set<Chips::GeneralCall::Command>(0xFF);
struct GeneralCall {
    static constexpr std::string_view Name          = "general call";
    static constexpr Address7         Address       = Address7::reserved(0x00);
    static constexpr std::size_t      RegisterBytes = 0;
    /// Not a part (Concepts.hpp): nothing goes out until it is asked for, so until then it is
    /// `starting`, and a Bus that counted it would never have all its parts answering.
    static constexpr bool Broadcast = true;

    /// The one byte. Transient: a command, not a state, so it is not sent again after a
    /// reset of the description -- and no Initial, so nothing goes out until it is asked for.
    struct Command {
        using Value                            = std::uint8_t;
        static constexpr std::size_t Bytes     = 1;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = static_cast<std::byte>(value);
            return Step::commandBuffer({.offset = 0, .count = 1});
        }
    };

    using Writes = List<Command>;
};

}   // namespace Kvasir::I2C::Chips
