#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// Infineon TLV493D-A1B6 3D magnetic sensor (data sheet rev 1.1, 2019-04-09; the register
/// bitmap the data sheet defers to its user manual for is the one Infineon's own drivers
/// use: TLV493D-A1B6 Arduino library, Tlv493d_conf.h, and its XENSIV successor,
/// TLx493D_A1B6.c). There is no register pointer: a read always starts at 0x00 and a write
/// always at 0x10, so to this engine it is a register-less chip -- reads are bare receives
/// and the one write is the four configuration registers as a raw payload.
///
/// Read registers:
///   0x00 Bx[11:4]            0x01 By[11:4]            0x02 Bz[11:4]
///   0x03 Temp[11:8] FRM[3:2] CH[1:0]
///   0x04 Bx[3:0] By[3:0]     0x05 - T FF PD Bz[3:0]   0x06 Temp[7:0]
///   0x07..0x09 factory settings: 0x07 bits 4:3, all of 0x08, 0x09 bits 4:0
/// Write registers:
///   0x10 reserved, 0
///   0x11 MOD1: P(7) IICAddr(6:5) factory(4:3) INT(2) FAST(1) LOW(0)
///   0x12 factory (the byte read at 0x08)
///   0x13 MOD2: T(7, 1 = temperature off) LP(6) PT(5, parity test) factory(4:0)
///
/// Each factory field must be written back as it was read, so the configuration cannot be
/// a constant: it is rebuilt from 0x07..0x09 before every write. P makes the 32 bits of
/// 0x10..0x13 odd (Infineon's calcParity), and PT = 1 has the part check it.
///
/// FAST/LOW select the mode: 0/0 power down (the state after power-up, data sheet 2.1.1),
/// 1/0 fast (3.3 kHz, faster than a 400 kHz bus can read it), 0/1 low power (MOD2 LP: a
/// 100 ms period when clear, 12 ms when set), 1/1 master controlled -- one conversion per request, then back to power down.
/// This description uses master controlled with INT off: a part converting on its own
/// clock marks each finished conversion with a 1.5 us low pulse on SCL (Table 9), which on
/// a bus shared with other parts, or behind an I2C switch, lands in their transfers.
///
/// Values are 12-bit two's complement (Table 10): 0.098 mT per count, which is exactly
/// 98 uT and is reported in nanotesla like every other magnetometer here (Linux tlv493d.c
/// reports 98/1000 gauss per count, ten times too little: 0.098 mT is 0.98 G); temperature is
/// 340 counts at 25 degC and 1.1 degC per count (Table 8). CH is
/// the channel being converted: anything but 0 means the frame is mid-conversion and its
/// channels do not belong together (Infineon's updateData). PD (0x05 bit 4) is documented as
/// "must be 1 at readout" (user manual 7.2.1.1, Bz2 register) and is not used: see Field::decode
/// for what the part does instead. FRM steps
/// with every conversion, modulo 4. T is the test-mode flag, FF the fuse-parity flag.
///
/// Addresses: the level of SDA/ADDR when the part comes out of reset picks 0x5E (high,
/// the Adafruit 4366 pulls it up) or 0x1F (low); IICAddr then moves it among four
/// addresses from there. Written as 00 here, so the part stays where it powered up.
///
/// No reset or recovery frames. Infineon's library opens with a general call (address
/// 0x00) carrying 0xFF or 0x00, whose SDA level re-selects the address, and documents a
/// frame to address 0xFF that frees an interrupted transfer. A general call is a broadcast
/// every other part on the segment -- on the selected channel, behind a switch -- also
/// receives; the address it re-selects follows whatever is on SDA at that moment, so bus
/// traffic can move the part to 0x1F where nothing looks for it; and a Device only ever
/// talks to its own address. The data sheet asks for that reset after power-up because
/// the part has no classic reset (3.2); what stands in for it here is that the
/// configuration is written again twice a second (Configure), so a part that has fallen
/// back to its power-down default is measuring again within half a second. A bus stuck
/// mid-transfer is the bus driver's to recover.
struct Tlv493d {
    static constexpr std::string_view        Name    = "TLV493D";
    static constexpr Address7                Address = 0x5E;
    static constexpr std::array<Address7, 2> Addresses{0x5E, 0x1F};
    static constexpr std::size_t             RegisterBytes = 0;
    /// TLV493D_STARTUPDELAY in Infineon's library.
    static constexpr auto StartupDelay = std::chrono::milliseconds{40};

    /// 0.098 mT per count.
    static constexpr NanoTesla FieldPerCount = Units::nanoTesla(98'000);

    /// How long the part is left alone after each transaction. In master-controlled mode a readout,
    /// and a write of the mode, starts a conversion of about 270 us (data sheet Table 7,
    /// "Electrical Setup"), and the data sheet has nothing read during one (chapter 4, "Managing
    /// correct sensor read outs": no shadow buffers); the user manual asks for the time between two
    /// readouts to be longer than a frame plus a conversion (5.5). Without a gap, the Configure
    /// run's write would follow its readout inside the conversion that readout started, and the
    /// next group's readout could land inside the one the write started. A step's delay keeps the
    /// run -- and so every other group of this part -- off the wire for that long; other parts use
    /// the gap.
    ///
    /// The A1B6's ADC can also hang (user manual 5.6: in master-controlled or fast mode "the
    /// ADC conversion may hang up", the frame counter stops, and only a general reset brings
    /// it back). Infineon gives no cause, so recovery is left to the caller (StallWatch).
    static constexpr std::chrono::milliseconds ConversionGap{1};

    /// MOD1 without P and the factory bits: IICAddr 00, INT off, FAST and LOW (master
    /// controlled). MOD2 without the factory bits: temperature on, LP set (Infineon's value
    /// for this mode; LP only paces low-power mode), parity test on.
    static constexpr std::uint8_t Mod1 = 0x03;
    static constexpr std::uint8_t Mod2 = 0x60;

    /// 0x10..0x13 over the factory bytes read at 0x07..0x09, P set so the 32 bits are odd.
    [[nodiscard]] static constexpr std::array<std::uint8_t,
                                              4>
    configuration(std::uint8_t factory1,
                  std::uint8_t factory2,
                  std::uint8_t factory3) {
        std::array<std::uint8_t, 4> c{0x00,
                                      static_cast<std::uint8_t>(Mod1 | (factory1 & 0x18U)),
                                      factory2,
                                      static_cast<std::uint8_t>(Mod2 | (factory3 & 0x1FU))};
        int                         ones = 0;
        for(auto const b : c) { ones += std::popcount(b); }
        if((ones & 1) == 0) { c[1] = static_cast<std::uint8_t>(c[1] | 0x80U); }
        return c;
    }

    /// Ten bytes: the presence probe, and the flags that say the part came up.
    static constexpr std::array Init{Step::receive({.count = 10, .offset = 0})};

    struct State {
        bool testMode{};     ///< T, 0x05 bit 6
        bool fuseParity{};   ///< FF, 0x05 bit 5: the fuses loaded with good parity
    };

    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state.testMode   = (data.u8(5) & 0x40U) != 0;
        state.fuseParity = (data.u8(5) & 0x20U) != 0;
        return state.fuseParity && !state.testMode;
    }

    /// The configuration, twice a second: the ten registers read, then 0x10..0x13 written.
    ///
    /// The configuration is staged behind the ten bytes by prepare(), from the factory bits
    /// the previous run read, and the check lets the write go only when it matches the
    /// factory bits just read. So a corrupted read of 0x07..0x09 never reaches the part --
    /// the run is repeated instead. The first run after the Device is constructed has nothing
    /// read yet to build from: its staged configuration is built over zero factory bytes,
    /// and because a check step repeats the sequence without calling prepare() again, the
    /// staged bytes never catch up within that run -- it spends the engine's retry budget
    /// (MaxRetries polls of a millisecond each) and ends in one rejection, writing nothing.
    /// The group's buffer keeps the ten bytes it read, so the next run configures the part,
    /// and a later bring-up of the same part starts from real factory bytes. (Neither Init
    /// nor a write group can compute a payload from bytes read at bring-up, and prepare()
    /// sees only the group's buffer, not the chip State, so the first run cannot be seeded
    /// from the Init read; that is why the write is a read group of its own.)
    ///
    /// A group apart from the field, not a step before every readout. In
    /// master-controlled mode a conversion starts once a result has been
    /// read out (Infineon's Tlv493d.h, MASTERCONTROLLEDMODE), and Infineon writes the mode
    /// once. Writing the mode in front of every readout restarts the conversion each time,
    /// so the readout can return a stale frame, and it adds transactions the bus does not
    /// need.
    struct Configure {
        static constexpr auto Period = std::chrono::milliseconds{500};

        static constexpr std::uint8_t ConfigOffset = 10;

        struct Request {};

        static constexpr void prepare(Request const&,
                                      std::span<std::byte> buffer) {
            auto const c = configuration(static_cast<std::uint8_t>(buffer[7]),
                                         static_cast<std::uint8_t>(buffer[8]),
                                         static_cast<std::uint8_t>(buffer[9]));
            for(std::size_t i = 0; i < c.size(); ++i) {
                buffer[ConfigOffset + i] = static_cast<std::byte>(c[i]);
            }
        }

        static constexpr std::array Steps{
          Step::receive({.count = 10, .offset = 0, .delay = ConversionGap}),
          Step::check(std::chrono::milliseconds{1}),
          Step::commandBuffer({.offset = ConfigOffset, .count = 4, .delay = ConversionGap}),
        };

        [[nodiscard]] static constexpr bool ready(Bytes data) {
            auto const c = configuration(data.u8(7), data.u8(8), data.u8(9));
            for(std::size_t i = 0; i < c.size(); ++i) {
                if(data.u8(ConfigOffset + i) != c[i]) { return false; }
            }
            return true;
        }
    };

    /// The field: 0x00..0x06. In master-controlled mode reading them out is what starts the
    /// next conversion (about 0.3 ms, Table 7), so each readout returns the frame the one
    /// before it asked for -- one period old, and fresh. The frame's CH says if it was not
    /// done. The gap after it keeps the Configure run off the conversion it started.
    struct Field {
        static constexpr auto Period = std::chrono::milliseconds{50};

        static constexpr std::array Steps{
          Step::receive({.count = 7, .offset = 0, .delay = ConversionGap})};

        struct Sample {
            NanoTesla    x{};   ///< positive: a south pole facing the Hall element (Figure 4)
            NanoTesla    y{};
            NanoTesla    z{};
            MilliDegC    temperature{};
            std::uint8_t frame{};   ///< FRM, modulo 4

            friend constexpr bool operator==(Sample const&,
                                             Sample const&) = default;
        };

        /// Twelve bits, the top eight in `high` and the bottom four in `low`, as a field.
        [[nodiscard]] static constexpr NanoTesla field(std::uint8_t high,
                                                       std::uint8_t low) {
            auto const counts
              = Bytes::signExtend((static_cast<std::uint32_t>(high) << 4U) | (low & 0x0FU), 12);
            return Units::nanoTesla(counts * Units::value(FieldPerCount));
        }

        /// A mid-conversion frame -- CH not 00 -- is read again after a millisecond. A part still
        /// at its power-down reset values has never converted: every count of its frame is 0,
        /// temperature included, which no conversion gives (0 counts would be -349 degC), so that
        /// frame is rejected rather than reported. T set is a part in test mode, whose data the
        /// manual calls tampered: rejected. A frame equal to the last one -- the same field,
        /// temperature and FRM -- is a part that did not convert (unchanged); a default Sample can
        /// never equal a real frame, because no count decodes to exactly 0 m degC.
        ///
        /// PD is **not** asked for, against the user manual's "must be 1 at readout" (7.2.1.1).
        /// On the bench's A1B6 (i2c_testing doc/hwtest-findings.md, 2026-09-18) PD read 0 in every
        /// readout in master-controlled mode -- with CH 00, FF set and a sane field and
        /// temperature -- and 1 only after a general reset, where the manual gives its reset value
        /// as 0: in this mode a readout starts the next conversion, and PD at byte 5 already
        /// reports that one. Requiring it rejected every frame. Linux tlv493d.c checks none of
        /// PD, CH and FRM.
        [[nodiscard]] static constexpr Outcome<Sample> decode(Bytes         data,
                                                              Sample const& previous) {
            auto const b3 = data.u8(3);
            auto const b5 = data.u8(5);
            if((b3 & 0x03U) != 0) { return Outcome<Sample>::retry(std::chrono::milliseconds{1}); }
            bool const neverConverted = data.u8(0) == 0 && data.u8(1) == 0 && data.u8(2) == 0
                                     && (b3 & 0xF0U) == 0 && data.u8(4) == 0 && (b5 & 0x0FU) == 0
                                     && data.u8(6) == 0;
            if(neverConverted) { return Outcome<Sample>::reject(); }
            if((b5 & 0x40U) != 0) { return Outcome<Sample>::reject(); }
            auto const temperatureCounts
              = Bytes::signExtend(((static_cast<std::uint32_t>(b3) >> 4U) << 8U) | data.u8(6), 12);
            Sample sample{};
            sample.x           = field(data.u8(0), static_cast<std::uint8_t>(data.u8(4) >> 4U));
            sample.y           = field(data.u8(1), data.u8(4));
            sample.z           = field(data.u8(2), data.u8(5));
            sample.temperature = Units::milliDegC((temperatureCounts - 340) * 1100 + 25000);
            sample.frame       = static_cast<std::uint8_t>((b3 >> 2U) & 0x03U);
            if(sample == previous) { return Outcome<Sample>::unchanged(); }
            return Outcome<Sample>::ok(sample);
        }

        static_assert(25000 % 1100 != 0,
                      "no temperature count decodes to 0 m degC, which is what keeps a default "
                      "Sample from matching the first frame");
    };

    using Reads = List<Field, Configure>;
};

static_assert(Tlv493d::configuration(0x00,
                                     0x00,
                                     0x00)
                == std::array<std::uint8_t,
                              4>{0x00,
                                 0x83,
                                 0x00,
                                 0x60},
              "four mode bits set: P makes it odd");
static_assert(Tlv493d::configuration(0xFF,
                                     0xA5,
                                     0xFF)
                == std::array<std::uint8_t,
                              4>{0x00,
                                 0x1B,
                                 0xA5,
                                 0x7F},
              "fifteen bits set: P stays clear, and only the factory fields are taken");
static_assert(Tlv493d::Field::field(0xF0,
                                    0x0F)
                == Units::nanoTesla(-241 * 98'000),
              "Table 10: 1111 0000 1111 is -241 counts, -23.6 mT");
static_assert(Tlv493d::Field::field(0x7F,
                                    0x0F)
                  == Units::nanoTesla(2047 * 98'000)
                && Tlv493d::Field::field(0x80,
                                         0x00)
                     == Units::nanoTesla(-2048 * 98'000),
              "the ends of the twelve-bit range");

}   // namespace Kvasir::I2C::Chips
