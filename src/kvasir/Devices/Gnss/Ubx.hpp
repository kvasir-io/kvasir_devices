#pragma once

#include "../Quantities.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ranges>
#include <span>
#include <type_traits>

/// u-blox UBX over any byte stream: the chunks the DDC (I2C) port's Stream group reads
/// (I2C/chips/SamM8q.hpp), the bytes a UART receive buffer hands out one at a time, SPI, USB.
/// Nothing here knows the transport.
///
///     Kvasir::Gnss::Ubx<> ubx{};
///     auto const onFrame = [&](Kvasir::Gnss::UbxMessage const& m) {
///         if(m.is(Kvasir::Gnss::Ublox::NavPvtClass, Kvasir::Gnss::Ublox::NavPvtId)) {
///             auto const pvt = Kvasir::Gnss::Ublox::decodeNavPvt(m.payload);
///         }
///     };
///
///     ubx.feed(gnss.latest().data(), onFrame);               // an I2C chunk
///
///     std::optional<std::byte> b;                            // a UART, byte by byte
///     while(Uart::receive(b)) {
///         if(b) { ubx.push(*b, onFrame); }
///         else  { ubx.abandon(); }                           // framing / overrun error
///     }
///
/// The frame (u-blox 8 / M8 Receiver Description and Protocol Specification, UBX-13003221
/// R28, 32.2): the preamble 0xB5 0x62, a class byte, an id byte, the payload length as a
/// little-endian U2 (the payload alone), the payload, and CK_A CK_B -- the 8-bit Fletcher
/// checksum (32.4) over class, id, length and payload.
///
/// Between frames the stream may carry anything else: the 0xFF a DDC (I2C) or SPI port returns
/// when it has nothing to send (11.5, 11.6.2), NMEA sentences on a port that outputs both. The
/// framer hunts for the preamble and drops the rest. A stream carrying both protocols can be fed
/// to a Ubx and a Nmea (Gnss/Nmea.hpp) side by side: each skips what the other takes.
namespace Kvasir::Gnss {

/// One good frame, as the callback sees it. The payload is a view into the framer: copy what
/// you keep before the next feed.
struct UbxMessage {
    std::uint8_t               cls{};
    std::uint8_t               id{};
    std::span<std::byte const> payload{};

    [[nodiscard]] constexpr bool is(std::uint8_t c,
                                    std::uint8_t i) const {
        return cls == c && id == i;
    }
};

/// The 8-bit Fletcher checksum of 32.4 over `data` (class, id, length and payload): CK_A in
/// the low byte, CK_B in the high byte.
[[nodiscard]] constexpr std::uint16_t ubxChecksum(std::span<std::uint8_t const> data) {
    std::uint8_t a = 0;
    std::uint8_t b = 0;
    for(auto const x : data) {
        a = static_cast<std::uint8_t>(a + x);
        b = static_cast<std::uint8_t>(b + a);
    }
    return static_cast<std::uint16_t>(a | (b << 8U));
}

/// A frame to send, built at compile time: `ubxFrame(0x06, 0x08, std::array<std::uint8_t,
/// 6>{0xE8, 0x03, 0x01, 0x00, 0x01, 0x00})` is UBX-CFG-RATE at 1 Hz, fourteen bytes.
template<std::size_t N>
[[nodiscard]] constexpr std::array<std::uint8_t,
                                   N + 8>
ubxFrame(std::uint8_t         cls,
         std::uint8_t         id,
         std::array<std::uint8_t,
                    N> const& payload) {
    std::array<std::uint8_t, N + 8> f{};
    f[0] = 0xB5;
    f[1] = 0x62;
    f[2] = cls;
    f[3] = id;
    f[4] = static_cast<std::uint8_t>(N & 0xFFU);
    f[5] = static_cast<std::uint8_t>(N >> 8U);
    for(std::size_t i = 0; i < N; ++i) { f[6 + i] = payload[i]; }
    auto const ck = ubxChecksum(std::span<std::uint8_t const>{f}.subspan(2, N + 4));
    f[N + 6]      = static_cast<std::uint8_t>(ck & 0xFFU);
    f[N + 7]      = static_cast<std::uint8_t>(ck >> 8U);
    return f;
}

/// A frame without a payload: a poll (32.5.2).
[[nodiscard]] constexpr std::array<std::uint8_t,
                                   8>
ubxFrame(std::uint8_t cls,
         std::uint8_t id) {
    return ubxFrame(cls, id, std::array<std::uint8_t, 0>{});
}

// UBX-MON-VER poll (32.16.13.1): class 0x0A, id 0x04, no payload. The converted spec prints no
// example frame, so the checksum here is 32.4 worked by hand: CK_A = 0x0A + 0x04 = 0x0E,
// CK_B = 0x0A + 0x0E + 0x0E + 0x0E = 0x34.
static_assert(ubxFrame(0x0A,
                       0x04)
              == std::array<std::uint8_t,
                            8>{0xB5,
                               0x62,
                               0x0A,
                               0x04,
                               0x00,
                               0x00,
                               0x0E,
                               0x34});
// UBX-CFG-RATE (32.10.27.1) at the default of C.16: measRate 1000 ms, navRate 1, timeRef 1.
static_assert(ubxFrame(0x06,
                       0x08,
                       std::array<std::uint8_t,
                                  6>{0xE8,
                                     0x03,
                                     0x01,
                                     0x00,
                                     0x01,
                                     0x00})
              == std::array<std::uint8_t,
                            14>{0xB5,
                                0x62,
                                0x06,
                                0x08,
                                0x06,
                                0x00,
                                0xE8,
                                0x03,
                                0x01,
                                0x00,
                                0x01,
                                0x00,
                                0x01,
                                0x39});

/// `MaxPayload` is the longest payload kept (UBX-NAV-PVT is 92 bytes; a UBX-MON-VER reply is
/// 40 + 30 bytes per extension string, usually well over 100). A frame whose length says more
/// is dropped and counted in `oversize`, never handed on cut short, and the framer hunts for the
/// next preamble from the byte after the length.
template<std::size_t MaxPayload = 256>
struct Ubx {
    std::array<std::byte, MaxPayload> payload{};
    std::uint32_t                     frames{};   ///< handed to the callback
    std::uint32_t badChecksum{};                  ///< complete frames whose CK_A CK_B did not match
    std::uint32_t oversize{};                     ///< frames longer than MaxPayload, dropped
    std::uint32_t abandoned{};                    ///< partial frames dropped by abandon()

    /// Push a chunk -- any range of std::byte, std::uint8_t or char; `onFrame(UbxMessage const&)`
    /// for every frame that completes inside it.
    template<std::ranges::input_range R,
             typename F>
    constexpr void feed(R const& chunk,
                        F&&      onFrame) {
        for(auto const b : chunk) { push(b, onFrame); }
    }

    /// Drop the frame in progress: the transport lost bytes (a UART framing or overrun error),
    /// so the framer hunts for the next preamble.
    constexpr void abandon() {
        if(state_ != State::sync1) { ++abandoned; }
        state_ = State::sync1;
    }

    /// One byte.
    template<typename Byte,
             typename F>
        requires(std::is_same_v<Byte,
                                std::byte>
                 || std::is_same_v<Byte,
                                   std::uint8_t>
                 || std::is_same_v<Byte,
                                   char>)
    constexpr void push(Byte b,
                        F&&  onFrame) {
        auto const x = static_cast<std::uint8_t>(b);
        switch(state_) {
        case State::sync1:
            // 0xFF filler, NMEA text, the tail of a frame that was dropped: all skipped
            if(x == 0xB5) { state_ = State::sync2; }
            return;
        case State::sync2:
            state_ = x == 0x62 ? State::cls : x == 0xB5 ? State::sync2 : State::sync1;
            return;
        case State::cls:
            cls_   = x;
            state_ = State::id;
            return;
        case State::id:
            id_    = x;
            state_ = State::len1;
            return;
        case State::len1:
            length_ = x;
            state_  = State::len2;
            return;
        case State::len2:
            length_ = static_cast<std::uint16_t>(length_ | (x << 8U));
            if(length_ > MaxPayload) {
                ++oversize;
                state_ = State::sync1;
                return;
            }
            at_ = 0;
            {
                std::array<std::uint8_t, 4> const head{cls_,
                                                       id_,
                                                       static_cast<std::uint8_t>(length_ & 0xFFU),
                                                       static_cast<std::uint8_t>(length_ >> 8U)};
                sum_ = ubxChecksum(std::span<std::uint8_t const>{head});
            }
            state_ = length_ == 0 ? State::ckA : State::body;
            return;
        case State::body:
            payload[at_++] = std::byte{x};
            add_(x);
            if(at_ == length_) { state_ = State::ckA; }
            return;
        case State::ckA:
            ckA_   = x;
            state_ = State::ckB;
            return;
        case State::ckB:
            state_ = State::sync1;
            if(ckA_ == (sum_ & 0xFFU) && x == (sum_ >> 8U)) {
                ++frames;
                onFrame(UbxMessage{.cls     = cls_,
                                   .id      = id_,
                                   .payload = std::span<std::byte const>{payload}.first(length_)});
            } else {
                ++badChecksum;
            }
            return;
        }
    }

    /// Frames that were not handed on, for whatever reason.
    [[nodiscard]] constexpr std::uint32_t bad() const { return badChecksum + oversize + abandoned; }

private:
    enum class State : std::uint8_t { sync1, sync2, cls, id, len1, len2, body, ckA, ckB };

    constexpr void add_(std::uint8_t x) {
        auto a = static_cast<std::uint8_t>((sum_ & 0xFFU) + x);
        auto b = static_cast<std::uint8_t>((sum_ >> 8U) + a);
        sum_   = static_cast<std::uint16_t>(a | (b << 8U));
    }

    State         state_{State::sync1};
    std::uint8_t  cls_{};
    std::uint8_t  id_{};
    std::uint8_t  ckA_{};
    std::uint16_t length_{};
    std::size_t   at_{};
    std::uint16_t sum_{};   ///< CK_A low, CK_B high, so far
};

// A UBX-ACK-ACK for CFG-RATE (32.8.1.1) behind 0xFF filler and a line of NMEA text: one frame.
static_assert([] {
    constexpr std::array<std::uint8_t, 18> stream{0xFF,
                                                  0xFF,
                                                  '$',
                                                  'G',
                                                  'N',
                                                  '\r',
                                                  '\n',
                                                  0xB5,
                                                  0x62,
                                                  0x05,
                                                  0x01,
                                                  0x02,
                                                  0x00,
                                                  0x06,
                                                  0x08,
                                                  0x16,
                                                  0x3F,
                                                  0xFF};
    Ubx<8>                                 ubx{};
    bool                                   acked = false;
    for(auto const b : stream) {
        ubx.push(b, [&](UbxMessage const& m) {
            acked = m.is(0x05, 0x01) && m.payload.size() == 2 && m.payload[0] == std::byte{0x06}
                 && m.payload[1] == std::byte{0x08};
        });
    }
    return acked && ubx.frames == 1 && ubx.bad() == 0;
}());

/// The UBX messages a u-blox M8 receiver is configured and read with (UBX-13003221 R28), the
/// same on every port: only UBX-CFG-PRT names the port it configures.
namespace Ublox {

    using Units::CentiDegree;
    using Units::MicroDegree;
    using Units::MilliMetre;
    using Units::MilliMetrePerSecond;

    /// UBX-ACK-ACK and -NAK (32.8): every CFG message is answered with one.
    inline constexpr std::uint8_t AckClass = 0x05;
    inline constexpr std::uint8_t AckNak   = 0x00;
    inline constexpr std::uint8_t AckAck   = 0x01;

    /// The CFG class (32.10) and the ids used here: an ACK names the message it answers.
    inline constexpr std::uint8_t CfgClass  = 0x06;
    inline constexpr std::uint8_t CfgPrtId  = 0x00;
    inline constexpr std::uint8_t CfgMsgId  = 0x01;
    inline constexpr std::uint8_t CfgRateId = 0x08;

    inline constexpr std::uint8_t NavPvtClass = 0x01;
    inline constexpr std::uint8_t NavPvtId    = 0x07;

    /// UBX-CFG-PRT in/outProtoMask bits (32.10.25.5).
    inline constexpr std::uint16_t ProtoUbx  = 0x0001;
    inline constexpr std::uint16_t ProtoNmea = 0x0002;

    /// UBX-CFG-PRT for the DDC port (32.10.25.5): port 0, no TX-ready, `address` as
    /// mode.slaveAddr, the protocols, no flags. 28 bytes.
    [[nodiscard]] constexpr std::array<std::uint8_t,
                                       28>
    cfgPrtDdc(std::uint8_t  address,
              std::uint16_t in,
              std::uint16_t out) {
        auto const mode = static_cast<std::uint8_t>(address << 1U);
        return ubxFrame(0x06,
                        0x00,
                        std::array<std::uint8_t, 20>{0x00,
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     mode,
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     static_cast<std::uint8_t>(in & 0xFFU),
                                                     static_cast<std::uint8_t>(in >> 8U),
                                                     static_cast<std::uint8_t>(out & 0xFFU),
                                                     static_cast<std::uint8_t>(out >> 8U),
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     0x00});
    }

    /// UBX-CFG-PRT for UART port `port` (1 is UART1, 2 UART2; 32.10.25.2): 8 data bits, no
    /// parity, 1 stop bit at `baud`, the protocols, no flags. `mode` is 0x08D0 -- charLen 8
    /// (bits 7..6 = 11), parity none (bits 11..9 = 100), one stop bit, and bit 4 set as u-center
    /// writes it. Sent over the UART itself the receiver switches baud rate before it answers,
    /// so the ACK arrives at the new rate. 28 bytes.
    [[nodiscard]] constexpr std::array<std::uint8_t,
                                       28>
    cfgPrtUart(std::uint32_t baud,
               std::uint16_t in,
               std::uint16_t out,
               std::uint8_t  port = 1) {
        auto const b
          = [&](unsigned shift) { return static_cast<std::uint8_t>((baud >> shift) & 0xFFU); };
        return ubxFrame(0x06,
                        0x00,
                        std::array<std::uint8_t, 20>{port,
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     0xD0,
                                                     0x08,
                                                     0x00,
                                                     0x00,
                                                     b(0),
                                                     b(8),
                                                     b(16),
                                                     b(24),
                                                     static_cast<std::uint8_t>(in & 0xFFU),
                                                     static_cast<std::uint8_t>(in >> 8U),
                                                     static_cast<std::uint8_t>(out & 0xFFU),
                                                     static_cast<std::uint8_t>(out >> 8U),
                                                     0x00,
                                                     0x00,
                                                     0x00,
                                                     0x00});
    }

    // UART1 at 9600 baud, UBX in, UBX out: the frame u-center sends for that.
    static_assert(cfgPrtUart(9600,
                             ProtoUbx,
                             ProtoUbx)
                  == std::array<std::uint8_t,
                                28>{0xB5,
                                    0x62,
                                    0x06,
                                    0x00,
                                    0x14,
                                    0x00,
                                    0x01,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0xD0,
                                    0x08,
                                    0x00,
                                    0x00,
                                    0x80,
                                    0x25,
                                    0x00,
                                    0x00,
                                    0x01,
                                    0x00,
                                    0x01,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x9A,
                                    0x79});

    /// UBX-CFG-MSG, the rate of one message on the port the command arrives on (32.10.18.3):
    /// every `rate`-th navigation solution, 0 off. 11 bytes.
    [[nodiscard]] constexpr std::array<std::uint8_t,
                                       11>
    cfgMsg(std::uint8_t cls,
           std::uint8_t id,
           std::uint8_t rate) {
        return ubxFrame(0x06, 0x01, std::array<std::uint8_t, 3>{cls, id, rate});
    }

    /// UBX-CFG-MSG, the rates of one message on all six I/O ports at once (32.10.18.2; port 0 is
    /// DDC, 1 and 2 the UARTs, 3 USB, 4 SPI: "Port Number assignment"). 16 bytes. The same
    /// setting as cfgMsg() for the port named, and another frame on the wire -- which is the
    /// point where a byte of the short one is a neighbour's address (chips/SamM8q.hpp,
    /// `MessageRatePerPort`).
    [[nodiscard]] constexpr std::array<std::uint8_t,
                                       16>
    cfgMsgPorts(std::uint8_t  cls,
                std::uint8_t  id,
                std::array<std::uint8_t,
                           6> rates) {
        return ubxFrame(
          0x06,
          0x01,
          std::array<std::uint8_t,
                     8>{cls, id, rates[0], rates[1], rates[2], rates[3], rates[4], rates[5]});
    }

    /// UBX-CFG-RATE (32.10.27.1): a measurement every `measRate`, a solution every one of them,
    /// aligned to GPS time. 14 bytes. The field is 16 bits and "should be greater than or equal
    /// to 25 ms" (50 ms below protocol version 24): a rate outside 25 ms .. 65535 ms is clamped
    /// to that range rather than wrapped.
    [[nodiscard]] constexpr std::array<std::uint8_t,
                                       14>
    cfgRate(std::chrono::milliseconds measRate) {
        auto const ms = static_cast<std::uint16_t>(
          std::clamp(measRate, std::chrono::milliseconds{25}, std::chrono::milliseconds{65535})
            .count());
        return ubxFrame(0x06,
                        0x08,
                        std::array<std::uint8_t, 6>{static_cast<std::uint8_t>(ms & 0xFFU),
                                                    static_cast<std::uint8_t>(ms >> 8U),
                                                    0x01,
                                                    0x00,
                                                    0x01,
                                                    0x00});
    }

    // Two of the SAM-M8Q's bring-up frames at their defaults, worked by hand against 32.4.
    static_assert(cfgPrtDdc(0x42,
                            ProtoUbx,
                            ProtoUbx)
                  == std::array<std::uint8_t,
                                28>{0xB5,
                                    0x62,
                                    0x06,
                                    0x00,
                                    0x14,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x84,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x01,
                                    0x00,
                                    0x01,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0x00,
                                    0xA0,
                                    0x96});
    static_assert(cfgMsg(NavPvtClass,
                         NavPvtId,
                         1)
                  == std::array<std::uint8_t,
                                11>{0xB5,
                                    0x62,
                                    0x06,
                                    0x01,
                                    0x03,
                                    0x00,
                                    0x01,
                                    0x07,
                                    0x01,
                                    0x13,
                                    0x51});

    /// A prebuilt UBX frame of up to `Capacity` bytes, the Value of a command write group.
    template<std::size_t Capacity>
    struct Frame {
        std::array<std::uint8_t, Capacity> bytes{};
        std::uint8_t                       length{};

        template<std::size_t N>
        [[nodiscard]] static constexpr Frame of(std::array<std::uint8_t,
                                                           N> const& frame) {
            static_assert(N <= Capacity, "the frame is longer than the command group holds");
            Frame f{};
            for(std::size_t i = 0; i < N; ++i) { f.bytes[i] = frame[i]; }
            f.length = static_cast<std::uint8_t>(N);
            return f;
        }

        [[nodiscard]] constexpr std::span<std::uint8_t const> data() const {
            return std::span<std::uint8_t const>{bytes}.first(length);
        }

        constexpr bool operator==(Frame const&) const = default;
    };

    /// UBX-NAV-PVT fixType (32.17.17.1).
    enum class FixType : std::uint8_t {
        noFix             = 0,
        deadReckoning     = 1,
        fix2d             = 2,
        fix3d             = 3,
        gnssDeadReckoning = 4,
        timeOnly          = 5,
    };

    /// What UBX-NAV-PVT (0x01 0x07, 92 bytes, 32.17.17.1) says, in the library's units.
    struct NavPvt {
        std::chrono::milliseconds iTow{};   ///< GPS time of week of the navigation epoch
        /// UTC (valid only with validDate / validTime): the calendar date, the time of day,
        /// and `nano`, the fraction of the second, which the receiver reports from -1e9 to
        /// 1e9 ns -- it may be negative, and is added to hour:minute:second as it is.
        std::chrono::year_month_day date{};
        std::chrono::hours          hour{};
        std::chrono::minutes        minute{};
        std::chrono::seconds        second{};   ///< 0..60: a leap second is 60
        std::chrono::nanoseconds    nano{};
        std::chrono::nanoseconds    tAcc{};   ///< time accuracy estimate
        bool                        validDate{};
        bool                        validTime{};
        bool                        fullyResolved{};
        FixType                     fixType{FixType::noFix};
        bool                        fixOk{};   ///< gnssFixOK: within the DOP and accuracy masks
        bool                        invalidLlh{};   ///< flags3: lon, lat, height, hMSL invalid
        std::uint8_t                numSv{};        ///< satellites used in the solution
        /// The message gives 1e-7 degree; these are that divided by 10 (truncated towards 0).
        MicroDegree         lon{};
        MicroDegree         lat{};
        MilliMetre          height{};   ///< above the ellipsoid
        MilliMetre          hMsl{};     ///< above mean sea level
        MilliMetre          hAcc{};     ///< horizontal accuracy estimate (U4, clamped to int32)
        MilliMetre          vAcc{};     ///< vertical accuracy estimate (U4, clamped to int32)
        MilliMetrePerSecond gSpeed{};   ///< ground speed, 2-D
        /// Heading of motion, 2-D: the message gives 1e-5 degree, divided by 1000 here.
        CentiDegree headMot{};
        /// Position DOP, 0.01 per count: a ratio, not a physical value.
        std::uint16_t pDop{};
    };

    namespace NavPvtImpl {
        [[nodiscard]] constexpr std::uint8_t u1(std::span<std::byte const> p,
                                                std::size_t                i) {
            return static_cast<std::uint8_t>(p[i]);
        }

        [[nodiscard]] constexpr std::uint16_t u2(std::span<std::byte const> p,
                                                 std::size_t                i) {
            return static_cast<std::uint16_t>(u1(p, i) | (u1(p, i + 1) << 8U));
        }

        [[nodiscard]] constexpr std::uint32_t u4(std::span<std::byte const> p,
                                                 std::size_t                i) {
            return u2(p, i) | (static_cast<std::uint32_t>(u2(p, i + 2)) << 16U);
        }

        [[nodiscard]] constexpr std::int32_t i4(std::span<std::byte const> p,
                                                std::size_t                i) {
            return static_cast<std::int32_t>(u4(p, i));
        }

        [[nodiscard]] constexpr std::int32_t clamped(std::uint32_t v) {
            auto const max = static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max());
            return static_cast<std::int32_t>(v > max ? max : v);
        }
    }   // namespace NavPvtImpl

    inline constexpr std::size_t NavPvtBytes = 92;

    /// A UBX-NAV-PVT payload (the framer's `payload`, without header and checksum). Offsets
    /// and scalings are 32.17.17.1's. The bit positions of `valid` (validDate 0, validTime 1,
    /// fullyResolved 2), `flags` (gnssFixOK 0) and `flags3` (invalidLlh 0, a 16-bit field at
    /// offset 78) are SparkFun's u-blox_structs.h, which the bitfield graphics missing from the
    /// converted spec would give. A payload shorter than 92 bytes gives a default NavPvt: no
    /// fix, nothing valid.
    [[nodiscard]] constexpr NavPvt decodeNavPvt(std::span<std::byte const> p) {
        using namespace NavPvtImpl;
        NavPvt n{};
        if(p.size() < NavPvtBytes) { return n; }
        n.iTow   = std::chrono::milliseconds{u4(p, 0)};
        n.date   = std::chrono::year_month_day{std::chrono::year{static_cast<int>(u2(p, 4))},
                                               std::chrono::month{u1(p, 6)},
                                               std::chrono::day{u1(p, 7)}};
        n.hour   = std::chrono::hours{u1(p, 8)};
        n.minute = std::chrono::minutes{u1(p, 9)};
        n.second = std::chrono::seconds{u1(p, 10)};
        auto const valid = u1(p, 11);
        n.validDate      = (valid & 0x01U) != 0;
        n.validTime      = (valid & 0x02U) != 0;
        n.fullyResolved  = (valid & 0x04U) != 0;
        n.tAcc           = std::chrono::nanoseconds{u4(p, 12)};
        n.nano           = std::chrono::nanoseconds{i4(p, 16)};
        n.fixType        = static_cast<FixType>(u1(p, 20));
        n.fixOk          = (u1(p, 21) & 0x01U) != 0;
        n.numSv          = u1(p, 23);
        n.lon            = Units::microDegree(i4(p, 24) / 10);
        n.lat            = Units::microDegree(i4(p, 28) / 10);
        n.height         = Units::milliMetre(i4(p, 32));
        n.hMsl           = Units::milliMetre(i4(p, 36));
        n.hAcc           = Units::milliMetre(clamped(u4(p, 40)));
        n.vAcc           = Units::milliMetre(clamped(u4(p, 44)));
        n.gSpeed         = Units::milliMetrePerSecond(i4(p, 60));
        n.headMot        = Units::centiDegree(i4(p, 64) / 1000);
        n.pDop           = u2(p, 76);
        n.invalidLlh     = (u2(p, 78) & 0x0001U) != 0;
        return n;
    }

}   // namespace Ublox

}   // namespace Kvasir::Gnss
