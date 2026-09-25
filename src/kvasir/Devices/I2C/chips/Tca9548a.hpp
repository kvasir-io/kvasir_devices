#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Texas Instruments TCA9548A / PCA9548A 8-channel I2C switch (SCPS207H). One control
/// byte, no pointer: bit n enables channel n, several may be on at once; a read returns
/// the control byte (7.5.4). All channels start closed. 0x70..0x77 by A2..A0. The devices
/// behind it are ordinary Devices on the same bus, gated by a MuxGate (../Mux.hpp).
struct Tca9548a {
    static constexpr std::string_view Name          = "TCA9548A";
    static constexpr Address7         Address       = 0x70;
    static constexpr std::size_t      RegisterBytes = 0;

    static constexpr std::array<Address7, 8>
      Addresses{0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77};

    /// The control byte. Read back every `VerifyInterval` (the part returns it on a bare
    /// read, 7.5.4) and written again when it differs: a switch that browned out comes up
    /// with every channel shut while the engine still believes the last value, and without
    /// this every part behind it would be parked as absent and never come back.
    struct Channels {
        using Value                                 = std::uint8_t;
        static constexpr std::size_t Bytes          = 1;
        static constexpr Value       Initial        = 0;
        static constexpr auto        VerifyDelay    = std::chrono::milliseconds{1};
        static constexpr auto        VerifyInterval = std::chrono::milliseconds{500};

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{value};
            return Step::commandBuffer({.offset = 0, .count = 1});
        }
    };

    struct Selected {
        static constexpr std::array Steps{Step::receive({.count = 1, .offset = 0})};
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes data) { return data.u8(0); }
    };

    using Reads  = List<Selected>;
    using Writes = List<Channels>;
};

}   // namespace Kvasir::I2C::Chips
