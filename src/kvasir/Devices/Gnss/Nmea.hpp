#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <string_view>
#include <type_traits>
#include <utility>

/// NMEA 0183 over any byte stream: the chunks an I2C module's Stream group reads
/// (I2C/chips/Pa1010d.hpp, I2C/chips/SamM8q.hpp), the bytes a UART receive buffer hands out one
/// at a time, a USB CDC read, a test string. Nothing here knows the transport.
///
///     Kvasir::Gnss::Nmea<> nmea{};
///     auto const onSentence = [&](Kvasir::Gnss::Nmea<> const& n) {
///         if(n.is("GGA")) { ... n.field(Kvasir::Gnss::Gga::satellites) ... }
///     };
///
///     nmea.feed(gps.latest().chars, onSentence);            // an I2C chunk
///
///     std::optional<std::byte> b;                            // a UART, byte by byte
///     while(Uart::receive(b)) {
///         if(b) { nmea.push(*b, onSentence); }
///         else  { nmea.abandon(); }                          // framing / overrun error
///     }
///
/// The callback runs once per complete "$...\r\n" line with a good checksum; inside it, is()
/// and field() look at that sentence. The views they return point into the framer's buffer and
/// are valid inside the callback only.
namespace Kvasir::Gnss {

/// `MaxLine` is the longest sentence kept. NMEA allows 82 characters including "$" and "\r\n";
/// a line that outgrows the buffer is dropped when its "\n" arrives and counted in `truncated`,
/// never handed on cut short. A sentence starts at "$"; whatever arrives between the end of one
/// and the next "$" is skipped. A sentence ends at "\r\n": a "\n" without the "\r" before it
/// is dropped wherever it is, because an I2C module with nothing more to send pads its answer
/// with 0x0A bytes, which can land inside a sentence that is still being written into its
/// buffer (Adafruit_GPS drops the same bytes).
template<std::size_t MaxLine = 96>
struct Nmea {
    static_assert(MaxLine >= 16,
                  "a sentence has at least a talker, a type and a checksum");

    std::array<char, MaxLine> line{};
    std::size_t               length{};
    std::uint32_t             sentences{};     ///< handed to the callback
    std::uint32_t             badChecksum{};   ///< complete lines whose checksum did not match
    std::uint32_t             truncated{};     ///< lines longer than MaxLine, dropped
    std::uint32_t             abandoned{};     ///< partial lines dropped by abandon()

    /// Push a chunk -- any range of char, std::byte or std::uint8_t; `onSentence(nmea)` for
    /// every sentence that completes inside it.
    template<std::ranges::input_range R,
             typename F>
    constexpr void feed(R const& chunk,
                        F&&      onSentence) {
        for(auto const c : chunk) { push(c, onSentence); }
    }

    /// One byte of the stream.
    template<typename Byte,
             typename F>
        requires(std::is_same_v<Byte,
                                char>
                 || std::is_same_v<Byte,
                                   std::byte>
                 || std::is_same_v<Byte,
                                   std::uint8_t>)
    constexpr void push(Byte b,
                        F&&  onSentence) {
        auto const c = static_cast<char>(b);
        if(c == '$') {
            length    = 0;
            overflow_ = false;
        } else if(length == 0 && !overflow_) {
            // Not inside a sentence: an I2C module's "nothing yet" newlines, or the binary
            // frames of another protocol on the same port (Gnss/Ubx.hpp). Skipped, not counted.
            return;
        }
        if(c == '\n' && !overflow_ && line[length - 1] != '\r') { return; }   // padding
        if(length < MaxLine) {
            line[length++] = c;
        } else {
            overflow_ = true;
        }
        if(c != '\n') { return; }
        if(overflow_) {
            ++truncated;
        } else if(checksumOk()) {
            ++sentences;
            onSentence(std::as_const(*this));
        } else {
            ++badChecksum;
        }
        length    = 0;
        overflow_ = false;
    }

    /// Drop the line in progress: the transport lost bytes (a UART framing or overrun error), so
    /// whatever follows until the next "$" does not belong to it.
    constexpr void abandon() {
        if(length != 0 || overflow_) { ++abandoned; }
        length    = 0;
        overflow_ = false;
    }

    /// Lines that were not handed on, for whatever reason.
    [[nodiscard]] constexpr std::uint32_t bad() const {
        return badChecksum + truncated + abandoned;
    }

    [[nodiscard]] constexpr std::string_view sentence() const { return {line.data(), length}; }

    /// Field i of the current sentence (0 is the talker + type, e.g. "GNGGA").
    [[nodiscard]] constexpr std::string_view field(std::size_t i) const {
        std::string_view s = sentence();
        if(s.empty() || s[0] != '$') { return {}; }
        s = s.substr(1);
        for(std::size_t n = 0; n < i; ++n) {
            auto const comma = s.find(',');
            if(comma == std::string_view::npos) { return {}; }
            s = s.substr(comma + 1);
        }
        auto const end = s.find_first_of(",*\r");
        return end == std::string_view::npos ? s : s.substr(0, end);
    }

    /// A field by the enumerator of a sentence's layout: `field(Gga::satellites)`.
    template<typename E>
        requires std::is_enum_v<E>
    [[nodiscard]] constexpr std::string_view field(E f) const {
        return field(static_cast<std::size_t>(std::to_underlying(f)));
    }

    /// The sentence type without the talker: is("GGA") matches $GPGGA and $GNGGA.
    [[nodiscard]] constexpr bool is(std::string_view type) const {
        auto const f = field(0);
        return f.size() == 5 && f.substr(2) == type;
    }

private:
    bool overflow_{};   ///< the line in progress outgrew the buffer

    [[nodiscard]] constexpr bool checksumOk() const {
        auto const s    = sentence();
        auto const star = s.find('*');
        if(s.size() < 4 || s[0] != '$' || star == std::string_view::npos || star + 3 > s.size()) {
            return false;
        }
        std::uint8_t sum = 0;
        for(std::size_t i = 1; i < star; ++i) { sum ^= static_cast<std::uint8_t>(s[i]); }
        auto const hex = [](char c) -> int {
            if(c >= '0' && c <= '9') { return c - '0'; }
            if(c >= 'A' && c <= 'F') { return c - 'A' + 10; }
            if(c >= 'a' && c <= 'f') { return c - 'a' + 10; }
            return -1;
        };
        auto const hi = hex(s[star + 1]);
        auto const lo = hex(s[star + 2]);
        return hi >= 0 && lo >= 0 && sum == static_cast<std::uint8_t>(hi * 16 + lo);
    }
};

/// The fields of GGA by position after the type (field 0):
/// $xxGGA,time,lat,N,lon,E,quality,satellites,hdop,altitude,M,separation,M,age,station*CS
enum class Gga : std::size_t {
    time = 1,
    latitude,
    latitudeHemisphere,
    longitude,
    longitudeHemisphere,
    quality,
    satellites,
    hdop,
    altitude,
    altitudeUnit,
    separation,
    separationUnit,
    age,
    station
};

/// The fields of RMC: $xxRMC,time,status,lat,N,lon,E,speed,course,date,variation,E,mode*CS
enum class Rmc : std::size_t {
    time = 1,
    status,
    latitude,
    latitudeHemisphere,
    longitude,
    longitudeHemisphere,
    speed,
    course,
    date,
    variation,
    variationDirection,
    mode
};

/// A sentence to send -- "$" body "*" checksum "\r\n" -- built at compile time, for any receiver
/// that takes NMEA-framed commands (MediaTek PMTK, u-blox PUBX, ...), over any transport.
template<std::size_t Capacity = 82>
struct NmeaSentence {
    static_assert(Capacity >= 6,
                  "\"$\", \"*\", two checksum digits and \"\\r\\n\"");

    std::array<char, Capacity> text{};
    std::uint8_t               length{};

    /// From the body between "$" and "*", e.g. "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0".
    /// A body longer than Capacity - 6 is cut.
    [[nodiscard]] static constexpr NmeaSentence of(std::string_view body) {
        NmeaSentence p{};
        std::uint8_t sum   = 0;
        p.text[p.length++] = '$';
        for(auto const c : body) {
            if(p.length >= Capacity - 5) { break; }
            p.text[p.length++] = c;
            sum ^= static_cast<std::uint8_t>(c);
        }
        constexpr std::string_view hex = "0123456789ABCDEF";
        p.text[p.length++]             = '*';
        p.text[p.length++]             = hex[sum >> 4U];
        p.text[p.length++]             = hex[sum & 0x0FU];
        p.text[p.length++]             = '\r';
        p.text[p.length++]             = '\n';
        return p;
    }

    [[nodiscard]] constexpr std::string_view view() const { return {text.data(), length}; }

    constexpr bool operator==(NmeaSentence const&) const = default;
};

static_assert(NmeaSentence<>::of("PMTK000").view() == "$PMTK000*32\r\n");

// The same sentence through three kinds of input, a byte at a time and in chunks, with line
// noise and a broken line in front of it.
static_assert([] {
    constexpr std::string_view gga
      = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
    Nmea<>   n{};
    unsigned seen = 0;
    auto     on   = [&](Nmea<> const& s) {
        if(s.is("GGA") && s.field(Gga::satellites) == "08" && s.field(Gga::altitude) == "545.4") {
            ++seen;
        }
    };
    n.feed(std::string_view{"\n\n$GNGGA,12"}, on);
    n.abandon();
    n.feed(gga, on);
    for(auto const c : gga) { n.push(static_cast<std::byte>(c), on); }
    for(auto const c : gga) { n.push(static_cast<std::uint8_t>(c), on); }
    return seen == 3 && n.abandoned == 1 && n.badChecksum == 0;
}());

// A port that outputs UBX as well: a frame whose payload holds "\n" bytes, between two
// sentences, is skipped and counts as nothing bad.
static_assert([] {
    constexpr std::string_view gga
      = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
    constexpr std::array<std::uint8_t, 12>
             ubx{0xB5, 0x62, 0x01, 0x07, 0x04, 0x00, 0x0A, 0x0A, 0xFF, 0x0A, 0x2E, 0x0A};
    Nmea<>   n{};
    unsigned seen = 0;
    auto     on   = [&](Nmea<> const&) { ++seen; };
    n.feed(gga, on);
    n.feed(ubx, on);
    n.feed(gga, on);
    return seen == 2 && n.bad() == 0;
}());

}   // namespace Kvasir::Gnss
