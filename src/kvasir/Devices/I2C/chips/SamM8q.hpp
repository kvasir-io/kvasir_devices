#pragma once

#include "../../Engine/Description.hpp"
#include "../../Gnss/Ubx.hpp"
#include "../Address.hpp"
#include "../Quantities.hpp"
#include "kvasir/Util/Config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// u-blox M8 GNSS receivers on their DDC (I2C) port: the SAM-M8Q antenna module, and any other
/// M8 part with DDC. Sources: u-blox 8 / u-blox M8 Receiver Description Including Protocol
/// Specification (UBX-13003221 R28), SAM-M8Q Data Sheet and Hardware Integration Manual.
///
/// The port (protocol spec 11.5): the receiver is a slave at 0x42 by default (the address is
/// `mode.slaveAddr` of UBX-CFG-PRT for DDC, 0x07 < slaveAddr < 0x78, 32.10.25.5). It is not a
/// register chip but a stream behind three registers (11.5.1): 0xFD and 0xFE hold the number
/// of bytes waiting, 0xFF delivers them, one byte per read, and 0xFF when nothing waits.
/// Registers 0x00..0xFC are reserved. A random-access read names the register (11.5.1.1), so
/// the Stream group asks 0xFD for the two-byte count, then 0xFF for that many bytes, at most
/// `MaxChunk` -- a counted read (Step::readCounted): nothing is read, and nothing goes on the
/// wire after the count, while nothing waits. The count is big-endian, 0xFD the high byte, as
/// SparkFun's u-blox library reads it (sfe_bus.cpp). That library also clears bit 15 of the
/// count, which it has seen read wrongly set on a logic analyser; `Sample::available` has it
/// cleared too, and the counted read clamps to `MaxChunk` either way. A write is a
/// UBX or NMEA message of at least two bytes (11.5.2); a UBX frame is at least eight.
///
/// `Stream::Sample` is the chunk. The application assembles UBX frames from it with
/// `Kvasir::Gnss::Ubx<>` (Gnss/Ubx.hpp) and decodes the
/// navigation solution with `Kvasir::Gnss::Ublox::decodeNavPvt`:
///
///     using Gnss = Kvasir::I2C::Device<I2c1, Clock, Chips::SamM8q<>>;
///     Gnss        gnss{};
///     Kvasir::Gnss::Ubx<> ubx{};
///     ...
///     gnss.turn();
///     if(gnss.fresh()) {   // every loop turn: a chunk not taken is overwritten by the next
///         ubx.feed(gnss.latest().data(), [&](Kvasir::Gnss::UbxMessage const& m) {
///             if(m.is(Kvasir::Gnss::Ublox::NavPvtClass, Kvasir::Gnss::Ublox::NavPvtId)) {
///                 auto const pvt = Kvasir::Gnss::Ublox::decodeNavPvt(m.payload);
///                 if(pvt.fixType == Kvasir::Gnss::Ublox::FixType::fix3d) { ... pvt.lat ... }
///             }
///         });
///     }
///
/// A run completes at most one chunk per loop turn, so checking fresh() after each turn()
/// sees every one. At 50 ms and 128 bytes the group drains 2.5 kB a second; UBX-NAV-PVT at 1 Hz
/// is 100 bytes a second, and the receiver's default NMEA output before the port is
/// configured is a few hundred.
///
/// The bring-up is three write groups with `Initial`, so they are written again after every
/// reset of the device (the part keeps its configuration in RAM unless it is saved, 3.1):
///   Port            UBX-CFG-PRT for DDC (0x06 0x00, 32.10.25.5): portID 0, slaveAddr 0x42
///                   kept, input protocols UBX, output UBX only -- NMEA output stops, which the
///                   default leaves on (C.15.4). inProtoMask / outProtoMask bit 0 is UBX, bit 1
///                   NMEA (SparkFun's COM_TYPE_UBX and COM_TYPE_NMEA). `mode` puts slaveAddr
///                   in bits 7..1 ("bit 0 must be 0"), so 0x42 is written as 0x84 (SparkFun
///                   shifts the address left by one the same way). txReady and flags are
///                   written 0: a TX-ready pin configured on this port is turned off.
///   NavPvtRate      UBX-CFG-MSG "set message rate" for the current port (0x06 0x01, 3-byte
///                   form, 32.10.18.3): UBX-NAV-PVT (0x01 0x07) every navigation solution.
///                   The current port is the one the message arrives on, DDC, so the rates
///                   of the UART and USB are left alone.
///   MeasurementRate UBX-CFG-RATE (0x06 0x08, 32.10.27.1): measRate from `Timing::measRate`, navRate 1,
///                   timeRef 1 (GPS time, the default of C.16). measRate must be 50 ms or more
///                   below protocol version 24 (25 ms from 24 on); the SAM-M8Q's firmware is
///                   older, so 50 is the floor here. The module itself updates at most at 10 Hz
///                   with its default GPS + GLONASS and 18 Hz with one constellation (SAM-M8Q
///                   data sheet, Table 1), so below 100 ms only a single-GNSS setup keeps up.
/// Every CFG message is answered on the stream with UBX-ACK-ACK or -NAK (32.5.1), which the
/// application sees through the framer.
///
/// `Command` sends any prebuilt UBX frame (`Frame::of(Gnss::ubxFrame(0x0A, 0x04))`, a MON-VER
/// poll). It is Transient -- a command, not a state -- so it is not sent again after a reset: a
/// replayed UBX-CFG-RST would restart the receiver after every bring-up. A frame of up to
/// `CommandBytes` (255 at most, a step's count is one byte) is one transaction, so no command
/// needs more than one Step.
///
/// The DDC port times out after about 2 s without the host talking to it (11.2), which a
/// polling Stream never lets happen. TX_READY (11.1) is not used.
/// The messages themselves -- CFG-PRT, CFG-MSG, CFG-RATE, NAV-PVT and its decoder -- are the
/// same on every port and live in Gnss/Ubx.hpp (`Kvasir::Gnss::Ublox`), where a receiver on a
/// UART uses them too.

/// `MaxChunk`: the most bytes one poll takes (2 + MaxChunk of the group's buffer).
/// `CommandBytes`: the longest frame Command sends. `Timing`: `Period`, how often the stream is
/// polled (50 ms), and `MeasRate`, the measurement rate the bring-up sets and with it how often
/// UBX-NAV-PVT comes (1 s); either may be left out.
template<std::uint8_t MaxChunk     = 128,
         std::size_t  CommandBytes = 100,
         typename Timing           = Engine::DefaultTiming>
struct UbloxM8 {
    static constexpr std::chrono::milliseconds PollPeriod
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Timing, period, std::chrono::milliseconds{50}));
    static constexpr std::chrono::milliseconds MeasRate
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Timing, measRate, std::chrono::milliseconds{1000}));

    static_assert(PollPeriod > std::chrono::milliseconds::zero(),
                  "the stream is polled; a period of 0 never reads it");
    static_assert(MaxChunk >= 1 && MaxChunk <= 253,
                  "a chunk lands behind the two count bytes, and a step addresses 255 bytes");
    static_assert(MeasRate >= std::chrono::milliseconds{50}
                    && MeasRate <= std::chrono::milliseconds{65535},
                  "UBX-CFG-RATE measRate is a U2, 50 ms or more below protocol version 24");
    static_assert(CommandBytes >= 8 && CommandBytes <= 255,
                  "a UBX frame is 8 bytes and more, a step's count is one byte");

    static constexpr std::string_view Name    = "SAM-M8Q";
    static constexpr Address7         Address = 0x42;
    /// The default. slaveAddr can be moved within 0x08..0x77 by UBX-CFG-PRT (32.10.25.5), but
    /// the Port group writes 0x42 back, so the description stays at the one address.
    static constexpr std::array<Address7, 1> Addresses{0x42};
    static constexpr std::size_t             RegisterBytes = 1;

    /// 11.5.1: the count of waiting bytes, and the stream.
    static constexpr std::uint8_t BytesAvailable = 0xFD;
    static constexpr std::uint8_t StreamData     = 0xFF;

    using Frame   = Kvasir::Gnss::Ublox::Frame<CommandBytes>;
    using FixType = Kvasir::Gnss::Ublox::FixType;
    using NavPvt  = Kvasir::Gnss::Ublox::NavPvt;

    struct Stream {
        static constexpr auto       Period = PollPeriod;
        static constexpr std::array Steps{
          Engine::Step::read({.reg = BytesAvailable, .count = 2, .offset = 0}),
          Engine::Step::readCounted({.reg         = StreamData,
                                     .countOffset = 0,
                                     .countBytes  = 2,
                                     .maxCount    = MaxChunk,
                                     .offset      = 2}),
        };

        struct Sample {
            std::array<std::byte, MaxChunk> bytes{};
            std::uint8_t                    length{};
            std::uint16_t available{};   ///< what 0xFD..0xFE said, this chunk included

            /// The chunk, for `Ubx::feed`.
            [[nodiscard]] constexpr std::span<std::byte const> data() const {
                return std::span<std::byte const>{bytes}.first(length);
            }
        };

        /// Nothing waiting is not a sample: fresh() steps once per chunk that holds bytes.
        [[nodiscard]] static constexpr Engine::Outcome<Sample> decode(Bytes data) {
            if(data.size() <= 2) { return Engine::Outcome<Sample>::unchanged(); }
            Sample s{};
            s.available = static_cast<std::uint16_t>(data.be16(0) & 0x7FFFU);
            s.length    = static_cast<std::uint8_t>(data.size() - 2);
            for(std::size_t i = 0; i < s.length; ++i) { s.bytes[i] = data.at(2 + i); }
            return Engine::Outcome<Sample>::ok(s);
        }
    };

    /// The DDC port's protocols (UBX-CFG-PRT): UBX in, UBX out.
    struct Protocols {
        std::uint16_t in{};
        std::uint16_t out{};

        constexpr bool operator==(Protocols const&) const = default;
    };

    struct Port {
        using Value                        = Protocols;
        static constexpr std::size_t Bytes = 28;
        static constexpr Value       Initial
          = {.in = Kvasir::Gnss::Ublox::ProtoUbx, .out = Kvasir::Gnss::Ublox::ProtoUbx};

        [[nodiscard]] static constexpr Engine::Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
            auto const f = Kvasir::Gnss::Ublox::cfgPrtDdc(Address, value.in, value.out);
            for(std::size_t i = 0; i < f.size(); ++i) { buffer[i] = std::byte{f[i]}; }
            return Engine::Step::commandBuffer(
              {.offset = 0, .count = static_cast<std::uint8_t>(f.size())});
        }
    };

    /// UBX-NAV-PVT on this port: every `value`-th navigation solution, 0 off.
    ///
    /// `Timing::messageRatePerPort = true` sends the six-port form of UBX-CFG-MSG (32.10.18.2:
    /// the value for DDC, 0 for the other ports) instead of the three-byte one. Why anybody
    /// would: the short frame for NAV-PVT every solution ends in the checksum 0x13 0x51, and
    /// 0x51 is "0x28, read". Measured on a SAM-M8Q sharing a TCA9548A channel at
    /// 400 kHz: a TSL2591 (0x29, and 0x28 beside it), an APDS-9960 and an LTR-507ALS each took
    /// a byte that is their address with the read bit, anywhere in a WRITE to the receiver, for
    /// their own address -- and held SDA low behind the frame until a bus recovery clocked them
    /// out. The same bytes written to another part of the channel did nothing, and nor did they
    /// in what is READ from the receiver: it is the receiver's acknowledge (SDA low 0.8 us after
    /// SCL fell, where the other parts and the controller are done after 0.3) that those three
    /// take for a START. A frame to the receiver must not hold the address byte of such a
    /// neighbour; `frames()` is there so that an application can check that at compile time.
    struct NavPvtRate {
        using Value = std::uint8_t;
        static constexpr bool PerPort
          = static_cast<bool>(KVASIR_CONFIG_OR(Timing, messageRatePerPort, false));
        static constexpr std::size_t Bytes   = PerPort ? 16 : 11;
        static constexpr Value       Initial = 1;

        [[nodiscard]] static constexpr auto frame(Value value) {
            if constexpr(PerPort) {
                return Kvasir::Gnss::Ublox::cfgMsgPorts(Kvasir::Gnss::Ublox::NavPvtClass,
                                                        Kvasir::Gnss::Ublox::NavPvtId,
                                                        {value, 0, 0, 0, 0, 0});
            } else {
                return Kvasir::Gnss::Ublox::cfgMsg(Kvasir::Gnss::Ublox::NavPvtClass,
                                                   Kvasir::Gnss::Ublox::NavPvtId,
                                                   value);
            }
        }

        [[nodiscard]] static constexpr Engine::Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
            auto const f = frame(value);
            for(std::size_t i = 0; i < f.size(); ++i) { buffer[i] = std::byte{f[i]}; }
            return Engine::Step::commandBuffer(
              {.offset = 0, .count = static_cast<std::uint8_t>(f.size())});
        }
    };

    /// The measurement rate (UBX-CFG-RATE), 50 ms .. 65535 ms.
    struct MeasurementRate {
        using Value                          = std::chrono::milliseconds;
        static constexpr std::size_t Bytes   = 14;
        static constexpr Value       Initial = MeasRate;

        [[nodiscard]] static constexpr Engine::Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
            auto const ms
              = std::clamp(value, std::chrono::milliseconds{50}, std::chrono::milliseconds{65535});
            auto const f = Kvasir::Gnss::Ublox::cfgRate(ms);
            for(std::size_t i = 0; i < f.size(); ++i) { buffer[i] = std::byte{f[i]}; }
            return Engine::Step::commandBuffer(
              {.offset = 0, .count = static_cast<std::uint8_t>(f.size())});
        }
    };

    /// Whether one of the frames the bring-up writes (Port, NavPvtRate and MeasurementRate at
    /// their Initial) holds `byte` -- for the static_assert of an application whose receiver has
    /// neighbours that take its acknowledge for a START (see NavPvtRate):
    ///     static_assert(!Gnss::Chip::bringUpWrites(0x29 << 1 | 1));
    [[nodiscard]] static constexpr bool bringUpWrites(std::uint8_t byte) {
        auto const in = [&](auto const& frame) {
            for(auto const b : frame) {
                if(b == byte) { return true; }
            }
            return false;
        };
        return in(Kvasir::Gnss::Ublox::cfgPrtDdc(Address, Port::Initial.in, Port::Initial.out))
            || in(NavPvtRate::frame(NavPvtRate::Initial))
            || in(Kvasir::Gnss::Ublox::cfgRate(MeasurementRate::Initial));
    }

    /// Any prebuilt UBX frame, once.
    struct Command {
        using Value                            = Frame;
        static constexpr std::size_t Bytes     = CommandBytes;
        static constexpr bool        Transient = true;

        [[nodiscard]] static constexpr Engine::Step encode(Value const&         value,
                                                           std::span<std::byte> buffer) {
            for(std::size_t i = 0; i < value.length; ++i) { buffer[i] = std::byte{value.bytes[i]}; }
            return Engine::Step::commandBuffer({.offset = 0, .count = value.length});
        }
    };

    using Reads  = Engine::List<Stream>;
    using Writes = Engine::List<Port, NavPvtRate, MeasurementRate, Command>;
};

/// The SAM-M8Q is the M8 this was written for.
template<std::uint8_t MaxChunk     = 128,
         std::size_t  CommandBytes = 100,
         typename Timing           = Engine::DefaultTiming>
using SamM8q = UbloxM8<MaxChunk, CommandBytes, Timing>;

}   // namespace Kvasir::I2C::Chips
