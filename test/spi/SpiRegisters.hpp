#pragma once

/// A register part on SPI: a control byte (`decode`: read?, register), then auto-incrementing data;
/// writes as (control, data) pairs (`Writes::pairs`, the BME280) or consecutive (`Writes::increment`).
#include "FakeSpi.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace Kvasir::Test::Spi {

struct SpiRegisters {
    enum class Writes : std::uint8_t { pairs, increment };

    std::array<std::uint8_t, 256> reg{};
    std::size_t                   csId{Cs::id};
    Writes                        writes{Writes::pairs};
    /// control byte -> (is a read, register); bit 7 = R/W by default
    std::function<std::pair<bool, std::uint8_t>(std::uint8_t)> decode{[](std::uint8_t c) {
        return std::pair{(c & 0x80U) != 0, static_cast<std::uint8_t>(c & 0x7FU)};
    }};
    /// A register written, as it arrives.
    std::function<void(std::uint8_t, std::uint8_t)>    onWrite{};
    std::vector<std::pair<std::uint8_t, std::uint8_t>> written{};

    std::uint32_t seenRises{};
    bool          haveControl{};
    bool          reading{};
    std::uint8_t  at{};

    std::uint8_t exchange(std::uint8_t mosi) {
        if(Pins::rises[csId] != seenRises) {   // a new frame
            seenRises   = Pins::rises[csId];
            haveControl = false;
        }
        if(!haveControl) {
            auto const [isRead, r] = decode(mosi);
            haveControl            = true;
            reading                = isRead;
            at                     = r;
            return 0xFF;
        }
        if(reading) { return reg[at++]; }
        reg[at] = mosi;
        written.emplace_back(at, mosi);
        if(onWrite) { onWrite(at, mosi); }
        if(writes == Writes::pairs) {
            haveControl = false;   // the next byte is a control byte again
        } else {
            ++at;
        }
        return 0xFF;
    }
};

}   // namespace Kvasir::Test::Spi
