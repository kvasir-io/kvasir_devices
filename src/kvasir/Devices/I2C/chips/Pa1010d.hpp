#pragma once

#include "../Device.hpp"
#include "../Nmea.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// CDTop PA1010D GNSS module (PA1010D data sheet; the MediaTek MT3333 PMTK command set).
/// Not a register chip: it streams NMEA text. A read of n bytes returns the next n bytes
/// of the stream, or 0x0A (a bare line feed) where there is nothing yet, so the reader
/// drops those. Commands are PMTK sentences written as text. Fixed address 0x10.
///
/// The read group takes 32 bytes every 50 ms, 640 bytes a second (the module emits well over
/// 300 bytes a second at the default 1 Hz with RMC + GGA + GSA + GSV + VTG, three or four GSV
/// lines alone, until the Initial PMTK314 cuts it to RMC + GGA); `Stream::Sample` is the chunk, and
/// the application assembles sentences from it (`Kvasir::I2C::Nmea`, I2C/Nmea.hpp, does the
/// framing and the fields; `Chips::Nmea` is its default instance). `set<Command>(Pmtk::of(
/// "PMTK314,..."))` sends one PMTK sentence with its checksum; the Initial one restricts the
/// output to RMC and GGA.
///
/// `Timing::StartupDelay` is the wait after power before the first transaction: the data sheet gives
/// no I2C-ready figure, and 500 ms is a conservative default.
namespace Pa1010dDetail {

    /// A PMTK sentence: "$" body "*" checksum "\r\n", at most 80 characters. The builder is
    /// NMEA's, not the module's (Gnss/Nmea.hpp), and serves a receiver on a UART as well.
    using Pmtk = Kvasir::Gnss::NmeaSentence<80>;

}   // namespace Pa1010dDetail

template<typename Timing = DefaultTiming>
struct Pa1010d {
    static constexpr std::string_view          Name    = "PA1010D";
    static constexpr Address7                  Address = 0x10;
    static constexpr std::array<Address7, 1>   Addresses{0x10};
    static constexpr std::size_t               RegisterBytes = 0;
    static constexpr std::chrono::milliseconds StartupDelay  = [] {
        if constexpr(requires { Timing::StartupDelay; }) {
            return Kvasir::asDuration(Timing::StartupDelay);
        } else {
            return std::chrono::milliseconds{500};
        }
    }();

    static constexpr std::size_t ChunkBytes = 32;

    struct Stream {
        static constexpr auto       Period = std::chrono::milliseconds{50};
        static constexpr std::array Steps{Step::receive({.count = ChunkBytes})};

        struct Sample {
            std::array<char, ChunkBytes> chars{};
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            Sample sample{};
            for(std::size_t i = 0; i < ChunkBytes; ++i) {
                sample.chars[i] = static_cast<char>(data.u8(i));
            }
            return sample;
        }
    };

    using Pmtk = Pa1010dDetail::Pmtk;

    /// A PMTK sentence is a command, not a state: the same one again is asking again.
    struct Command {
        using Value                              = Pmtk;
        static constexpr std::size_t Bytes       = 80;
        static constexpr bool        AlwaysWrite = true;
        static constexpr Value Initial = Pmtk::of("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

        [[nodiscard]] static constexpr Step encode(Value const&         value,
                                                   std::span<std::byte> buffer) {
            for(std::size_t i = 0; i < value.length; ++i) {
                buffer[i] = static_cast<std::byte>(value.text[i]);
            }
            return Step::commandBuffer({.offset = 0, .count = value.length});
        }
    };

    using Reads  = List<Stream>;
    using Writes = List<Command>;
};

/// `Chips::Nmea`: the sentence framer over a Pa1010d's chunks (I2C/Nmea.hpp) at its default
/// line length.
using Nmea = Kvasir::I2C::Nmea<>;

}   // namespace Kvasir::I2C::Chips
