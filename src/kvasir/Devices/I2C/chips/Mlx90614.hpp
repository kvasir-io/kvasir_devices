#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Melexis MLX90614 infrared thermometer (datasheet rev 021), SMBus read word: the
/// command byte (RAM 0x06 ambient TA, 0x07 object TOBJ1), a repeated START, LSB, MSB, PEC.
/// The PEC is CRC-8 (polynomial 0x07) over the whole frame including both address bytes
/// (4.1.4.3.7: address 0x5A, command 0x07, 0xD2 0x3A -> PEC 0x30; Linux mlx90614.c skips the
/// PEC as "not valid", but the datasheet's own example checks out), which is why the
/// address is a template parameter here rather than an `At<>` config: the PEC check depends
/// on it, so a part re-programmed to another address is `Mlx90614<0x5B>`, not `At<0x5B>`.
/// Temperatures are in 0.02 K; bit 15 set is an error flag. The RAM holds no valid result
/// until 250 ms after power-on (Tvalid), and a frame read sooner can pass its PEC with a zero
/// word -- -273.15 degC -- so the first read waits for it, and an object word below 0x27AD
/// (-70.01 degC, the bottom of TO1's sweep, 4.1.4.3.6) or an ambient word below 0x2D89
/// (-40 degC, the part's operating range) is rejected. The default address is 0x5A;
/// the SMBus address EEPROM cell (0x0E) takes any 7-bit address, so the scan hint lists the
/// factory one only.
template<Address7 Addr = 0x5A>
struct Mlx90614 {
    static constexpr std::string_view        Name    = "MLX90614";
    static constexpr Address7                Address = Addr;
    static constexpr std::array<Address7, 1> Addresses{Addr};
    static constexpr std::size_t             RegisterBytes = 1;
    static constexpr auto                    StartupDelay  = std::chrono::milliseconds{250};

    static constexpr std::uint16_t MinObject  = 0x27AD;
    static constexpr std::uint16_t MinAmbient = 0x2D89;

    [[nodiscard]] static constexpr bool pecOk(Bytes        data,
                                              std::size_t  i,
                                              std::uint8_t command) {
        std::array<std::byte, 5> frame{static_cast<std::byte>(Addr << 1),
                                       std::byte{command},
                                       static_cast<std::byte>((Addr << 1) | 1),
                                       data.at(i),
                                       data.at(i + 1)};
        return crc8Smbus(Bytes{frame}) == data.u8(i + 2);
    }

    struct Temperature {
        static constexpr auto       Period = std::chrono::milliseconds{200};
        static constexpr std::array Steps{Step::read({.reg = 0x06, .count = 3, .offset = 0}),
                                          Step::read({.reg = 0x07, .count = 3, .offset = 3})};

        struct Sample {
            CentiDegC ambient{};
            CentiDegC object{};
        };

        [[nodiscard]] static constexpr CentiDegC centi(std::uint16_t raw) {
            return Units::centiDegC(static_cast<std::int32_t>(raw) * 2 - 27315);
        }

        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes data) {
            if(!pecOk(data, 0, 0x06) || !pecOk(data, 3, 0x07)) { return Outcome<Sample>::reject(); }
            auto const ta = data.le16(0);
            auto const to = data.le16(3);
            if((ta & 0x8000U) != 0 || (to & 0x8000U) != 0) { return Outcome<Sample>::reject(); }
            if(ta < MinAmbient || to < MinObject) { return Outcome<Sample>::reject(); }
            return Outcome<Sample>::ok({centi(ta), centi(to)});
        }
    };

    using Reads = List<Temperature>;
};

}   // namespace Kvasir::I2C::Chips
