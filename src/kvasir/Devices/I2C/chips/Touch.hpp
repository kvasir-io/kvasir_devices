#pragma once

#include "../Device.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

namespace TouchDetail {

    /// What a data frame amounted to, before the engine turns it into an Outcome.
    /// `stale`: the controller had nothing new (the GT911's buffer-status bit).
    /// `noFinger` and `point` are readings. `malformed`: a frame that cannot be right (a
    /// count past the chip's maximum, a pattern the chip never produces).
    enum class Frame : std::uint8_t { stale, noFinger, point, malformed };

    /// One frame in the controller's own coordinates, before any mount transform.
    struct Raw {
        Frame         kind{};
        std::uint8_t  count{};
        std::uint16_t x{};
        std::uint16_t y{};
    };

    /// What a controller's `identify()` made of the bytes its Init script read. `ok` is the
    /// policy's verdict (this is the chip the policy is for); `chip`, `firmware` and `extra`
    /// are logged as hex and mean whatever the policy says; `width`/`height` is the
    /// controller's own resolution when it can be read (0: unknown).
    struct ChipInfo {
        bool          ok{};
        std::uint32_t chip{};
        std::uint32_t firmware{};
        std::uint32_t extra{};
        std::uint16_t width{};
        std::uint16_t height{};
    };

    /// Which INT edge the controller produces per report. Advisory: the engine never looks
    /// at it, and the application's GPIO interrupt uses it to pick its edge. The values
    /// match Kvasir::Io::Edge.
    enum class IntEdge : std::uint8_t { falling = 1, rising = 2, both = 3 };

    /// How often to read, when the application does not say. A read every HeldPoll while a
    /// finger is down, so a release is never missed if the controller does not signal one;
    /// every IdlePoll otherwise, so a finger is still found if INT never fires -- which also
    /// makes a dead INT line visible, since the application can compare readings with edge
    /// counts. Either may be zero: 0 for IdlePoll means "on INT and a held poll only", 0 for
    /// both means "on INT only", and then the group runs when request<Data>() asks.
    struct PollDefaults {
        static constexpr auto HeldPoll = std::chrono::milliseconds{50};
        static constexpr auto IdlePoll = std::chrono::milliseconds{100};
    };

    /// What differs between controllers.
    template<typename C>
    concept Controller = requires(Bytes data) {
        { C::Name } -> std::convertible_to<std::string_view>;
        { C::Address } -> std::same_as<Address7 const&>;
        { std::span<Address7 const>{C::Addresses} };                // every one it can have
        { C::RegisterBytes } -> std::convertible_to<std::size_t>;   // 1 or 2, big-endian
        { C::ResetLow } -> std::convertible_to<std::chrono::milliseconds>;
        { C::ResetSettle } -> std::convertible_to<std::chrono::milliseconds>;
        { std::span<Step const>{C::Init} };
        { C::identify(data) } -> std::same_as<ChipInfo>;
        { C::DataRegister } -> std::convertible_to<std::uint16_t>;
        { C::DataLength } -> std::convertible_to<std::size_t>;
        { C::decode(data) } -> std::same_as<Raw>;
        { C::AckRegister } -> std::convertible_to<std::uint16_t>;   // 0: no write after a read
        { C::AckValue } -> std::convertible_to<std::uint8_t>;
        { C::Int } -> std::convertible_to<IntEdge>;
    };

    /// Hynitron CST816S / CST816T / CST816D, the single-touch controller on the round 1.28"
    /// modules. 0x15, 8-bit registers. From the CST816S register map (FA IrqCtl, FE
    /// DisAutoSleep).
    ///
    /// - Reset: RST low 10 ms, then 200 ms (esp-bsp's figure). The reset line is not optional
    ///   in practice: out of reset the chip enters its low-power mode after AutoSleepTime (2 s,
    ///   register F9) and then does not answer I2C, so without a reset pulse the Init script's
    ///   first write finds it asleep on every bring-up after the first two seconds of power --
    ///   Linux hynitron-cst816x.c always resets it. Give the Controller a Reset type.
    /// - Init: FE = 01 keeps the chip from going to sleep on its own (asleep, it does not
    ///   answer and the held poll would find nothing), FA = 60 (EnTouch | EnChange) makes it
    ///   pulse INT for every report while a finger is down, not once per gesture.
    /// - Identification: A7 chip id (B4 CST816S, B5 CST816T, B6 CST816D), A9 firmware. These
    ///   ids and the event bits below are esp-bsp's; the register map documents A7 only as
    ///   the chip id and XposH bits 3:0.
    /// - Report: 5 bytes from 02: count (low nibble), XH (event in bits 7..6, x[11:8] in
    ///   3..0), XL, YH, YL. Event 1 is "up". Register 01 holds a gesture id the driver does
    ///   not read; the event layer classifies gestures itself.
    /// - INT: a low pulse per report with FA = 60.
    struct Cst816s {
        static constexpr std::string_view        Name    = "CST816S";
        static constexpr Address7                Address = 0x15;
        static constexpr std::array<Address7, 1> Addresses{0x15};
        static constexpr std::size_t             RegisterBytes = 1;
        static constexpr auto                    ResetLow      = std::chrono::milliseconds{10};
        static constexpr auto                    ResetSettle   = std::chrono::milliseconds{200};

        static constexpr std::array<Step, 4> Init{
          Step::write({.reg = 0xFE, .payload = {0x01}}),        // DisAutoSleep
          Step::write({.reg = 0xFA, .payload = {0x60}}),        // IrqCtl: EnTouch | EnChange
          Step::read({.reg = 0xA7, .count = 1, .offset = 0}),   // chip id
          Step::read({.reg = 0xA9, .count = 1, .offset = 1}),   // firmware version
        };

        [[nodiscard]] static constexpr ChipInfo identify(Bytes data) {
            ChipInfo r{};
            r.chip     = data.u8(0);
            r.firmware = data.u8(1);
            r.ok       = r.chip == 0xB4 || r.chip == 0xB5 || r.chip == 0xB6;
            return r;
        }

        static constexpr std::uint16_t DataRegister = 0x02;
        static constexpr std::size_t   DataLength   = 5;

        [[nodiscard]] static constexpr Raw decode(Bytes data) {
            auto const count = data.u8(0) & 0x0FU;
            if(count == 0) { return {Frame::noFinger, 0, 0, 0}; }
            // a single-touch chip; 0xFF is a sleeping one
            if(count > 2) { return {Frame::malformed, 0, 0, 0}; }
            if((data.u8(1) >> 6) == 1) { return {Frame::noFinger, 0, 0, 0}; }   // event: up
            return {
              Frame::point,
              static_cast<std::uint8_t>(count),
              static_cast<std::uint16_t>(((unsigned{data.u8(1)} & 0x0FU) << 8U) | data.u8(2)),
              static_cast<std::uint16_t>(((unsigned{data.u8(3)} & 0x0FU) << 8U) | data.u8(4))};
        }

        static constexpr std::uint16_t AckRegister = 0;
        static constexpr std::uint8_t  AckValue    = 0;
        static constexpr IntEdge       Int         = IntEdge::falling;
    };

    /// Hynitron CST9217 at 0x5A, with its own INT and RESET pins. There is no datasheet; the
    /// protocol below is what the vendor reference driver does:
    ///
    /// - 16-bit big-endian register addresses; a write of the address, then a
    ///   repeated-START read.
    /// - Reset: RST low 10 ms, high, 80 ms before the first transaction (vendor: 50 + 30).
    /// - Init: an address-only write of D1 01. The vendor means to send two payload bytes
    ///   after it but its helper passes the wrong length, so the chip has only ever seen the
    ///   bare address; that is reproduced, not fixed. Then 10 ms.
    /// - Identification: 4 bytes each from D1FC (checkcode), D1F8 (resolution, two
    ///   little-endian u16) and D204 (project id, chip type; 0x9217).
    /// - Report: 15 bytes from D000. Finger 0 is bytes 0..4, byte 5 the count, byte 6 an ack
    ///   (0xAB), finger 1 bytes 7..11. Status is the low nibble of byte 0, 0x06 while
    ///   touching; x = b1 << 4 | b3 >> 4, y = b2 << 4 | b3 & 0xF, 12 bits each.
    /// - No finger: the frame has no ack byte, count 0, the finger slot all ones, and its
    ///   tail carries the chip's own resolution and checkcode. It is the resting state, not
    ///   a corrupt frame, and decodes as "no finger". The chip answers this way until the
    ///   first touch after reset; afterwards it keeps the ack byte and says count 0.
    /// - The lift: an acked frame with the finger still in its slot, current coordinates, and
    ///   a status nibble of 0 instead of 6, with an INT pulse like any report. The register
    ///   keeps answering it until the next touch, so there is no count-0 frame after a lift
    ///   and this frame is the release. Near the edge of the panel the chip can lose a finger
    ///   and find it again, each time with this frame and an INT pulse: a real lift and a
    ///   momentary loss are indistinguishable on the wire, and debouncing belongs to the
    ///   layer above. So the noFinger case below is a *reading* whenever a finger was
    ///   down, and only "nothing new" when none was -- treating it as "nothing new"
    ///   unconditionally would leave the finger down for good.
    /// - INT: one pulse per report, falling and rising edges in equal numbers. The pad has a
    ///   pull-up, so the pulse is low-going.
    ///
    /// Only finger 0 is decoded; a second finger contributes to the count and nothing else.
    struct Cst9217 {
        static constexpr std::string_view        Name    = "CST9217";
        static constexpr Address7                Address = 0x5A;
        static constexpr std::array<Address7, 1> Addresses{0x5A};
        static constexpr std::size_t             RegisterBytes = 2;
        static constexpr auto                    ResetLow      = std::chrono::milliseconds{10};
        static constexpr auto                    ResetSettle   = std::chrono::milliseconds{80};

        static constexpr std::uint16_t ChipType  = 0x9217;
        static constexpr std::uint8_t  Ack       = 0xAB;
        static constexpr std::uint8_t  Touching  = 0x06;
        static constexpr std::size_t   MaxPoints = 2;

        static constexpr std::array<Step, 4> Init{
          Step::write({.reg = 0xD101, .payload = {}, .delay = std::chrono::milliseconds{10}}),
          Step::read({.reg = 0xD1FC, .count = 4, .offset = 0}),
          Step::read({.reg = 0xD1F8, .count = 4, .offset = 4}),
          Step::read({.reg = 0xD204, .count = 4, .offset = 8}),
        };

        [[nodiscard]] static constexpr ChipInfo identify(Bytes data) {
            ChipInfo r{};
            r.extra    = data.le32(0);   // checkcode
            r.width    = data.le16(4);
            r.height   = data.le16(6);
            r.firmware = data.le16(8);   // project id
            r.chip     = data.le16(10);
            r.ok       = r.chip == ChipType;
            return r;
        }

        static constexpr std::uint16_t DataRegister = 0xD000;
        static constexpr std::size_t   DataLength   = MaxPoints * 5 + 5;

        [[nodiscard]] static constexpr Raw decode(Bytes data) {
            if(data.u8(6) != Ack) { return {Frame::noFinger, 0, 0, 0}; }
            auto const count = static_cast<std::size_t>(data.u8(5) & 0x7FU);
            if(count == 0) { return {Frame::noFinger, 0, 0, 0}; }
            // the count field is four bits; the vendor clamps what it reports to 2, as MaxPoints
            // does below
            if(count > 0x0F) { return {Frame::malformed, 0, 0, 0}; }
            if((data.u8(0) & 0x0FU) != Touching) { return {Frame::noFinger, 0, 0, 0}; }
            return {
              Frame::point,
              static_cast<std::uint8_t>(count < MaxPoints ? count : MaxPoints),
              static_cast<std::uint16_t>((unsigned{data.u8(1)} << 4U) | (data.u8(3) >> 4U)),
              static_cast<std::uint16_t>((unsigned{data.u8(2)} << 4U) | (data.u8(3) & 0x0FU))};
        }

        static constexpr std::uint16_t AckRegister = 0;
        static constexpr std::uint8_t  AckValue    = 0;
        static constexpr IntEdge       Int         = IntEdge::falling;
    };

    /// FocalTech FT6236 / FT6336 / FT6336U and the FT5x06 family, the controllers on most
    /// 1.3" to 2.8" capacitive modules. 0x38, 8-bit registers. From the FT6x36 datasheet's
    /// register table.
    ///
    /// - Reset: RST low 10 ms, then 300 ms (the datasheet's Trsi, "time of starting to report
    ///   point after resetting"; some vendor examples use 10 ms).
    /// - Init: none. The nine threshold and period registers a vendor driver typically
    ///   writes hold their reset defaults; a design that needs others adds Step::write
    ///   entries here.
    /// - Identification: six bytes from A3: A3 chip id (0x36 FT6236, 0x64 FT6336U, 0x06
    ///   FT6206), A4 mode, A5 power mode, A6 firmware, A7 (FT5x06: state), A8 vendor id,
    ///   0x11 for FocalTech. Either is the verdict: a vendor id of 0x11, or one of the three
    ///   FT6x36 chip ids. On FT5x06 panels A8 is the panel maker's model code (Linux
    ///   edt-ft5x06.c lists 0x35, 0x43, 0x59, 0x5A and more, and accepts any), so one of those
    ///   comes up through its chip id or not at all.
    /// - Report: 7 bytes from 02: count (low nibble), then point 1: XH (event in bits 7..6:
    ///   0 down, 1 up, 2 contact, 3 none; x[11:8] in 3..0), XL, YH (id in 7..4, y[11:8] in
    ///   3..0), YL, weight, area. Events "up" and "none" are no finger.
    /// - INT: in the default polling mode a low pulse per report; register A4 = 1 selects
    ///   trigger mode (level low while touched), which the driver does not need.
    struct Ft6x36 {
        static constexpr std::string_view        Name    = "FT6x36";
        static constexpr Address7                Address = 0x38;
        static constexpr std::array<Address7, 1> Addresses{0x38};
        static constexpr std::size_t             RegisterBytes = 1;
        static constexpr auto                    ResetLow      = std::chrono::milliseconds{10};
        static constexpr auto                    ResetSettle   = std::chrono::milliseconds{300};

        static constexpr std::uint8_t VendorId = 0x11;

        static constexpr std::array<Step, 1> Init{
          Step::read({.reg = 0xA3, .count = 6, .offset = 0}),
        };

        [[nodiscard]] static constexpr ChipInfo identify(Bytes data) {
            ChipInfo r{};
            r.chip     = data.u8(0);
            r.firmware = data.u8(3);
            r.extra    = data.u8(5);   // vendor id
            r.ok       = r.extra == VendorId || r.chip == 0x06 || r.chip == 0x36 || r.chip == 0x64;
            return r;
        }

        static constexpr std::uint16_t DataRegister = 0x02;
        static constexpr std::size_t   DataLength   = 7;

        [[nodiscard]] static constexpr Raw decode(Bytes data) {
            auto const count = data.u8(0) & 0x0FU;
            if(count == 0) { return {Frame::noFinger, 0, 0, 0}; }
            // 0xFF: the chip is not answering
            if(count > 5) { return {Frame::malformed, 0, 0, 0}; }
            auto const event = data.u8(1) >> 6;
            if(event == 1 || event == 3) { return {Frame::noFinger, 0, 0, 0}; }
            return {
              Frame::point,
              static_cast<std::uint8_t>(count),
              static_cast<std::uint16_t>(((unsigned{data.u8(1)} & 0x0FU) << 8U) | data.u8(2)),
              static_cast<std::uint16_t>(((unsigned{data.u8(3)} & 0x0FU) << 8U) | data.u8(4))};
        }

        static constexpr std::uint16_t AckRegister = 0;
        static constexpr std::uint8_t  AckValue    = 0;
        static constexpr IntEdge       Int         = IntEdge::falling;
    };

    /// Goodix GT911, the five-point controller on the 4.3" to 10" panels. 16-bit registers.
    /// From the GT911 programming guide's register map.
    ///
    /// - Address: the chip samples INT while RST rises: low gives 0x5D, high gives 0x14. The
    ///   driver cannot drive INT (the pin is the application's GPIO interrupt), so the pad's
    ///   pull decides: a pull-down in the pin configuration gives 0x5D (the default here), a
    ///   pull-up gives 0x14, and the config's `Address` has to say which. The level must be
    ///   stable before RST rises, which a pull is.
    /// - Reset: RST low 10 ms, then 50 ms (esp-bsp: 10 + 50 + 10).
    /// - Identification: 8140 product id, four ASCII bytes ("911"), 8144 firmware version
    ///   (little-endian u16); 8048 / 804A the configured x and y resolution.
    /// - Report: 814E buffer status (bit 7: new data; low nibble: count), then 8 bytes per
    ///   point from 814F: track id, x (LE16), y (LE16), size (LE16), reserved. Point 1 is
    ///   read in the same transaction. After every read of a frame 814E must be written 00,
    ///   or the chip repeats the frame and never raises INT again -- that is the last Step of
    ///   the read group below. A frame without bit 7 is stale: nothing changes, and 814E is
    ///   not written, because bit 7 comes up a little after INT (the programming guide asks
    ///   the host to read again; Linux goodix.c measured about 10 ms) and a 00 written after
    ///   the stale read would clear a frame that became ready in between, unread. The frame
    ///   is then taken by the next poll, so a GT911 run from its INT line alone keeps an
    ///   IdlePoll.
    /// - INT: a pulse per report; its edge is bits 1..0 of config byte 804D (INT trigger:
    ///   00 rising, 01 falling, the usual stock value). The advisory edge here is falling.
    struct Gt911 {
        static constexpr std::string_view        Name           = "GT911";
        static constexpr Address7                Address        = 0x5D;
        static constexpr Address7                AddressIntHigh = 0x14;
        static constexpr std::array<Address7, 2> Addresses{0x5D, 0x14};
        static constexpr std::size_t             RegisterBytes = 2;
        static constexpr auto                    ResetLow      = std::chrono::milliseconds{10};
        static constexpr auto                    ResetSettle   = std::chrono::milliseconds{50};

        static constexpr std::array<Step, 2> Init{
          Step::read({.reg = 0x8140, .count = 6, .offset = 0}),   // product id (4) + firmware (2)
          Step::read({.reg = 0x8048, .count = 4, .offset = 6}),   // x resolution, y resolution
        };

        [[nodiscard]] static constexpr ChipInfo identify(Bytes data) {
            ChipInfo r{};
            r.chip     = (static_cast<std::uint32_t>(data.u8(0)) << 16U)
                       | (static_cast<std::uint32_t>(data.u8(1)) << 8U) | data.u8(2);   // "911"
            r.firmware = data.le16(4);
            r.width    = data.le16(6);
            r.height   = data.le16(8);
            r.ok       = data.u8(0) == '9' && data.u8(1) == '1' && data.u8(2) == '1';
            return r;
        }

        static constexpr std::uint16_t DataRegister = 0x814E;
        static constexpr std::size_t   DataLength   = 1 + 8;

        [[nodiscard]] static constexpr Raw decode(Bytes data) {
            auto const status = data.u8(0);
            if((status & 0x80U) == 0) { return {Frame::stale, 0, 0, 0}; }
            auto const count = status & 0x0FU;
            if(count == 0) { return {Frame::noFinger, 0, 0, 0}; }
            if(count > 5) { return {Frame::malformed, 0, 0, 0}; }
            return {Frame::point, static_cast<std::uint8_t>(count), data.le16(2), data.le16(4)};
        }

        static constexpr std::uint16_t AckRegister = 0x814E;
        static constexpr std::uint8_t  AckValue    = 0;
        static constexpr IntEdge       Int         = IntEdge::falling;
    };

}   // namespace TouchDetail

/// A capacitive touch controller as a description: the reset pulse, the init script, one
/// identification, and a data read per INT pulse or poll. What differs between controllers
/// is a policy in TouchDetail (`Cst816s`, `Cst9217`, `Ft6x36`, `Gt911`); what is here is the
/// shape they share.
///
/// The reading is in the controller's own coordinates. Turning it into panel pixels, giving
/// it a sequence number and a timestamp, and counting what the glass did is
/// `Kvasir::I2C::Touch::Controller` (../TouchController.hpp), which owns everything about a
/// *panel*; this owns everything about a *chip*.
///
/// Three things make the frames of a touch controller different from a sensor's, and all
/// three are why `decode` here takes the previous Sample:
///   * a repeated "no finger" is not news. The first one after a finger is a release and
///     steps `seq`; the ones after it are `Outcome::unchanged()`, so an application that
///     watches `seq` is not woken ten times a second by untouched glass.
///   * a release keeps the last coordinates, because that is where the finger left.
///   * the GT911's buffer-status bit says "nothing new", which is not a fault and must not
///     be counted as one -- also `unchanged()`.
///
/// `Cfg` may set `HeldPoll` and `IdlePoll` (TouchDetail::PollDefaults), and -- through the
/// engine's Config, which is the same type -- `Address`, for the GT911's INT strap.
template<TouchDetail::Controller Ctrl, typename Cfg = TouchDetail::PollDefaults>
struct TouchPanel {
    using Controller = Ctrl;
    using ChipInfo   = TouchDetail::ChipInfo;
    using Frame      = TouchDetail::Frame;
    using IntEdge    = TouchDetail::IntEdge;

    static constexpr std::string_view Name          = Ctrl::Name;
    static constexpr Address7         Address       = Ctrl::Address;
    static constexpr auto             Addresses     = Ctrl::Addresses;
    static constexpr std::size_t      RegisterBytes = Ctrl::RegisterBytes;
    static constexpr auto             ResetLow      = Ctrl::ResetLow;
    static constexpr auto             ResetSettle   = Ctrl::ResetSettle;

    /// The engine ignores this; the application reads it to pick its GPIO edge.
    static constexpr IntEdge Int = Ctrl::Int;

    static constexpr auto HeldPoll = [] {
        if constexpr(requires { Cfg::HeldPoll; }) {
            return Kvasir::asDuration(Cfg::HeldPoll);
        } else {
            return TouchDetail::PollDefaults::HeldPoll;
        }
    }();

    static constexpr auto IdlePoll = [] {
        if constexpr(requires { Cfg::IdlePoll; }) {
            return Kvasir::asDuration(Cfg::IdlePoll);
        } else {
            return TouchDetail::PollDefaults::IdlePoll;
        }
    }();

    static constexpr auto Init = Ctrl::Init;

    /// The identification, kept for the application: state().
    using State = ChipInfo;

    /// The engine logs "not the chip this description is for" and carries on; identified()
    /// is the verdict.
    [[nodiscard]] static constexpr bool setup(Bytes  data,
                                              State& state) {
        state = Ctrl::identify(data);
        return state.ok;
    }

    /// One reading, in the controller's own coordinates. `points` is the finger count; 0 is
    /// a release, and then x and y are where the finger left.
    struct Reading {
        std::uint8_t  points{};
        std::uint16_t x{};
        std::uint16_t y{};

        constexpr bool operator==(Reading const&) const = default;
    };

    /// What the engine calls the reading.
    using Sample = Reading;

    struct Data {
        using Sample = Reading;

        /// Only the compile-time "this group is cyclic", and the first deadline; what is
        /// actually waited is period() below.
        static constexpr auto Period = IdlePoll >= HeldPoll ? IdlePoll : HeldPoll;

        /// Fast while a finger is down, slow when the glass is idle. Zero parks the group
        /// until the INT line's interrupt calls request<Data>().
        [[nodiscard]] static constexpr std::chrono::milliseconds period(Reading const& s) {
            return s.points != 0 ? HeldPoll : IdlePoll;
        }

        /// The data read, and -- for a controller that wants one -- the write that lets it
        /// produce the next frame, once the frame read was not stale. The engine runs the
        /// script, then decodes, so the acknowledge goes out after the read, which is where
        /// the chip wants it.
        static constexpr auto Steps = [] {
            if constexpr(Ctrl::AckRegister != 0) {
                return std::array<Step, 3>{
                  Step::read({.reg    = Ctrl::DataRegister,
                              .count  = static_cast<std::uint8_t>(Ctrl::DataLength),
                              .offset = 0}),
                  Step::stopUnless(),
                  Step::write({.reg = Ctrl::AckRegister, .payload = {Ctrl::AckValue}})};
            } else {
                return std::array<Step, 1>{
                  Step::read({.reg    = Ctrl::DataRegister,
                              .count  = static_cast<std::uint8_t>(Ctrl::DataLength),
                              .offset = 0})};
            }
        }();

        /// The stopUnless step's question: was the frame read a frame, not a stale buffer.
        [[nodiscard]] static constexpr bool ready(Bytes data) {
            return Ctrl::decode(data).kind != Frame::stale;
        }

        [[nodiscard]] static constexpr Outcome<Reading> decode(Bytes          data,
                                                               Reading const& previous) {
            auto const raw = Ctrl::decode(data);
            switch(raw.kind) {
            case Frame::stale: return Outcome<Reading>::unchanged();

            case Frame::malformed: return Outcome<Reading>::reject();

            case Frame::noFinger:
                // The first one after a finger is the release, and it keeps the last
                // coordinates; the ones after it say nothing.
                if(previous.points == 0) { return Outcome<Reading>::unchanged(); }
                return Outcome<Reading>::ok(Reading{0, previous.x, previous.y});

            case Frame::point: return Outcome<Reading>::ok(Reading{raw.count, raw.x, raw.y});
            }
            return Outcome<Reading>::reject();
        }
    };

    using Reads = List<Data>;

    static_assert(RegisterBytes == 1 || RegisterBytes == 2,
                  "a register address is one or two bytes");
    static_assert(Ctrl::DataLength > 0 && Ctrl::DataLength <= 255,
                  "a data frame is between one and 255 bytes");
};

template<typename Cfg = TouchDetail::PollDefaults>
using Cst816s = TouchPanel<TouchDetail::Cst816s, Cfg>;

template<typename Cfg = TouchDetail::PollDefaults>
using Cst9217 = TouchPanel<TouchDetail::Cst9217, Cfg>;

template<typename Cfg = TouchDetail::PollDefaults>
using Ft6x36 = TouchPanel<TouchDetail::Ft6x36, Cfg>;

template<typename Cfg = TouchDetail::PollDefaults>
using Gt911 = TouchPanel<TouchDetail::Gt911, Cfg>;

}   // namespace Kvasir::I2C::Chips
