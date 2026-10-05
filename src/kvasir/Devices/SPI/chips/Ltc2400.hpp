#pragma once
// Linear/ADI LTC2401 (LTC2402 too) 24-bit no-latency delta-sigma ADC, read over SPI with an external
// serial clock (LTC2401_LTC2402.md; the data sheet covers both). Checked against that data sheet
// only: the LTC2400 and LTC2411 are said to send the same 32-bit word, but their data sheets were
// not checked - use this for them only after comparing their "Output Data Format" table with
// Table 2 below.
//
// Not a description on the chip-description engine: the part has no command byte and no
// registers, and its readiness is the SDO pin itself - with CS low and before any SCK, SDO is EOC,
// high while it converts (133 or 160 ms) and low once a result waits (md:422, :588). The engine
// can only ask with a whole frame, and clocking SCK while it converts is what the data sheet asks
// to avoid (digital transitions during the conversion state, md:774). So:
//
// - `Ltc2400::decode(word)` is the pure part: a 32-bit frame to a signed reading in sub-LSB units,
//   the range it lies in, or "not ready" / "not a frame this part sends". Host tests and
//   static_asserts check every SIG/EXR case of Tables 1 and 2.
// - `Ltc2400::Reader<Master, Clock, Cs, Sdo, Config>` is a driver on a queued SPI master
//   (QueueCore.hpp) that polls SDO as a GPIO with CS held low and reads 4 bytes when it is low.
//   The LTC2401 must be ALONE on its bus (CS stays low between frames, so its SDO drives MISO).
// - The constants below are what a firmware needs to do the same by hand.
#include "../../Link.hpp"
#include "../../Log.hpp"
#include "../../Quantities.hpp"
#include "../QueueCore.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir { namespace SPI { namespace Ltc2400 {

    inline constexpr std::string_view Name = "LTC2401";

    // The wire (LTC2401_LTC2402.md, Timing Characteristics, page 4; external SCK = Note 9)

    /// SCK idles LOW and the result is latched on the rising edge, shifted out on the falling one
    /// (md:457, :590): mode 0. SCK must also be LOW at power-up and at every CS falling edge, or
    /// the part enters INTERNAL SCK mode and drives SCK itself (md:542, :640).
    inline constexpr ClockMode Mode = ClockMode::_0;
    /// fESCK at most 2000 kHz (md:188), tLESCK / tHESCK at least 250 ns (md:189-190).
    inline constexpr Units::Hertz  MaxClock     = Units::hertz(2'000'000);
    inline constexpr std::uint32_t SckLowMinNs  = 250;
    inline constexpr std::uint32_t SckHighMinNs = 250;
    /// t1: CS low to SDO driven (EOC readable) at most 150 ns (md:193) - wait that long after CS
    /// falls before reading SDO as the ready bit.
    inline constexpr std::uint32_t CsToSdoValidMaxNs = 150;
    /// t4: CS low to the first SCK rising edge at least 50 ns (md:196).
    inline constexpr std::uint32_t CsToSckRiseMinNs = 50;
    /// t5: SCK low at least 50 ns before CS falls (md:199): what selects the external SCK mode.
    inline constexpr std::uint32_t SckSetupBeforeCsMinNs = 50;
    /// tKQMAX: SCK falling to SDO valid at most 200 ns (md:197).
    inline constexpr std::uint32_t SckToSdoValidMaxNs = 200;
    /// One frame is 32 bits, MSB (EOC) first (md:420, Figure 3).
    inline constexpr std::size_t FrameBytes = 4;

    // Conversion (Timing Characteristics tCONV, md:185; FO pin md:329; Table 3 md:554)

    /// FO tied to VCC: internal oscillator, first notch at 50 Hz; FO at GND: 60 Hz (md:329, and
    /// Notes 7/8 md:216/:230). An external clock on FO is not described here.
    enum class Rejection : std::uint8_t { hz50, hz60 };

    struct ConversionTime {
        std::chrono::microseconds typical;
        std::chrono::microseconds max;
    };

    /// tCONV, internal oscillator: FO = VCC 157.03 / 160.23 / 163.44 ms, FO = 0 V 130.86 / 133.53
    /// / 136.20 ms (min / typ / max, md:185).
    [[nodiscard]] constexpr ConversionTime conversionTime(Rejection r) {
        return r == Rejection::hz50 ? ConversionTime{std::chrono::microseconds{160'230},
                                                     std::chrono::microseconds{163'440}}
                                    : ConversionTime{std::chrono::microseconds{133'530},
                                                     std::chrono::microseconds{136'200}};
    }

    /// The POR after VCC rises lasts "approximately 0.5ms", then the first conversion starts
    /// (md:395).
    inline constexpr std::chrono::microseconds PowerOnReset{500};

    // The 32-bit word (Output Data Format, md:418-501; Tables 1 and 2)

    /// Bit 31 EOC (1 while converting), bit 30 DMY on the LTC2401 ("always low", md:424) and the
    /// channel on the LTC2402 (0 = CH0), bit 29 SIG, bit 28 EXR, bits 27..4 the 24-bit result MSB
    /// first, bits 3..0 sub-LSBs (md:420-451).
    inline constexpr std::uint32_t EocBit  = 1U << 31U;
    inline constexpr std::uint32_t Bit30   = 1U << 30U;
    inline constexpr std::uint32_t SigBit  = 1U << 29U;
    inline constexpr std::uint32_t ExrBit  = 1U << 28U;
    inline constexpr std::uint32_t RawMask = 0x0FFF'FFFFU;   ///< result + sub-LSBs, 28 bits

    /// Counts are in sub-LSB units: 1/16 of a 24-bit LSB, so VREF (FSSET - ZSSET) is 2^28 counts
    /// (0x0FFFFFFx is VREF - 1 LSB, "VREF + 1LSB" is 2^24 LSBs, Table 2).
    inline constexpr std::int32_t FullScaleCounts = std::int32_t{1} << 28;
    /// The extended range clamps at 9/8 VREF and -1/8 VREF (md:459; Table 2's first and last rows:
    /// 24-bit result 0x1FFFFF with SIG/EXR 1/1, 0xE00000 with 0/1).
    inline constexpr std::uint32_t AboveClamp24 = 0x1F'FFFFU;
    inline constexpr std::uint32_t BelowClamp24 = 0xE0'0000U;

    enum class Part : std::uint8_t { ltc2401, ltc2402 };

    /// Table 1: EXR clear is the normal range 0 <= VIN <= VREF; EXR set with SIG set is above
    /// VREF, with SIG clear below zero.
    enum class Range : std::uint8_t { normal, aboveVref, belowZero };

    struct Sample {
        std::int32_t counts{};   ///< 1/16 LSB24; 0 at ZSSET, 2^28 at FSSET; -2^25 .. 2^28 + 2^25
        Range        range{};
        bool         clamped{};   ///< at 9/8 VREF or -1/8 VREF: the input may be further out
        std::uint8_t channel{};   ///< LTC2402: the channel converted; LTC2401: 0

        /// The 24-bit result, sub-LSBs dropped (rounded towards minus infinity).
        [[nodiscard]] constexpr std::int32_t code24() const { return counts >> 4; }

        /// The input relative to ZSSET, given VREF = FSSET - ZSSET.
        [[nodiscard]] constexpr Units::MicroVolt voltage(Units::MicroVolt vref) const {
            auto const uv = vref.numerical_value_in(Units::si::micro<Units::si::volt>);
            return Units::microVolt(
              static_cast<std::int32_t>(std::int64_t{counts} * uv / FullScaleCounts));
        }
    };

    struct Decoded {
        enum class Kind : std::uint8_t {
            ok,
            notReady,   ///< EOC set: the part was converting, nothing was shifted out
            invalid,    ///< a status/result combination Tables 1 and 2 do not have
        };

        Kind   kind{};
        Sample sample{};
    };

    /// The four SIG/EXR cases (Table 1, Table 2):
    ///
    ///   SIG EXR  input                24-bit result R     counts
    ///    1   0   0 < VIN <= VREF       0 .. 0xFFFFFF       raw28
    ///    1   1   VREF < VIN (9/8 max)  0 .. 0x1FFFFF       2^28 + raw28
    ///    0   1   VIN < 0 (-1/8 min)    0xE00000 .. FFFFFF  raw28 - 2^28
    ///    0   0   VIN = 0- only         0                   0
    ///
    /// The word as a whole is offset binary: bits 29..0 minus 2^29, except the 0- code, which the
    /// data sheet makes 0 (SIG flips "during the zero code", md:432, Table 2 note **); its
    /// sub-LSBs are given as X and ignored. Anything outside those columns - a result past a clamp,
    /// a non-zero result with SIG/EXR 0/0, bit 30 set on an LTC2401 - is no frame of this part.
    [[nodiscard]] constexpr Decoded decode(std::uint32_t word,
                                           Part          part = Part::ltc2401) {
        if((word & EocBit) != 0) { return {.kind = Decoded::Kind::notReady}; }
        bool const bit30 = (word & Bit30) != 0;
        if(bit30 && part == Part::ltc2401) { return {.kind = Decoded::Kind::invalid}; }
        bool const sig = (word & SigBit) != 0;
        bool const exr = (word & ExrBit) != 0;
        auto const raw = word & RawMask;
        auto const r24 = raw >> 4U;
        Sample     s{.channel = static_cast<std::uint8_t>(bit30 ? 1 : 0)};
        if(sig && !exr) {
            s.counts = static_cast<std::int32_t>(raw);
            s.range  = Range::normal;
        } else if(sig && exr) {
            if(r24 > AboveClamp24) { return {.kind = Decoded::Kind::invalid}; }
            s.counts  = FullScaleCounts + static_cast<std::int32_t>(raw);
            s.range   = Range::aboveVref;
            s.clamped = r24 == AboveClamp24;
        } else if(exr) {
            if(r24 < BelowClamp24) { return {.kind = Decoded::Kind::invalid}; }
            s.counts  = static_cast<std::int32_t>(raw) - FullScaleCounts;
            s.range   = Range::belowZero;
            s.clamped = r24 == BelowClamp24;
        } else {
            if(r24 != 0) { return {.kind = Decoded::Kind::invalid}; }
            s.counts = 0;
            s.range  = Range::normal;
        }
        return {.kind = Decoded::Kind::ok, .sample = s};
    }

    /// The four bytes as they came off the wire, MSB first.
    [[nodiscard]] constexpr std::uint32_t wordOf(std::span<std::byte const,
                                                           FrameBytes> b) {
        return (std::to_integer<std::uint32_t>(b[0]) << 24U)
             | (std::to_integer<std::uint32_t>(b[1]) << 16U)
             | (std::to_integer<std::uint32_t>(b[2]) << 8U) | std::to_integer<std::uint32_t>(b[3]);
    }

    // A driver on a queued SPI master

    /// The Reader's Config: derive from this and override what differs (`rejection` must match
    /// how FO is wired; `part` ltc2402 takes bit 30 as the channel).
    struct Defaults {
        static constexpr Rejection    rejection = Rejection::hz50;
        static constexpr Part         part      = Part::ltc2401;
        static constexpr Units::Hertz MaxClock  = Ltc2400::MaxClock;
        /// After the driver starts: the part's POR and a conversion that may already be running.
        static constexpr auto StartupDelay = std::chrono::milliseconds{1};
        /// Failures in a row (a timeout, an impossible frame, SDO low after a frame) before link()
        /// says absent; it goes on trying regardless.
        static constexpr std::uint8_t AbsentAfterFailures = 3;
        /// CS stays high this long after a failure before the next try (a MISO stuck low would
        /// otherwise be a frame every loop turn).
        static constexpr auto RetryDelay = std::chrono::milliseconds{10};
    };

    /// Polls an LTC2401/LTC2402 that is alone on its SPI bus (external SCK, CS held low between
    /// frames - "CS may remain LOW and EOC monitored", md:592). Call handler() once a loop turn.
    ///
    /// - CS goes low in one handler() call and SDO is first read in a later one, which is far more
    ///   than t1 (150 ns) apart on any main loop.
    /// - SDO low: one 4-byte read (MOSI clocks 0xFF, unconnected). On its completion SDO must be
    ///   HIGH again - the 32nd SCK falling edge starts the next conversion and SDO shows EOC = 1
    ///   (md:457, :590). A MISO that stays low (no part, floating low) is caught there; one that
    ///   floats high never shows a result and times out (2 x the longest conversion).
    /// - Any failure takes CS high for RetryDelay: during the data output that aborts it and starts
    ///   a new conversion (md:317, :574), during a conversion it does nothing; then CS goes
    ///   low again with SCK idle low, which also re-selects the external SCK mode (md:542).
    ///
    /// `Sdo` is a pin `read()` can sample while it is the SPI block's MISO (on an STM32 the input
    /// data register samples the pin in alternate-function mode too, RM0430 7.3.11).
    template<typename Master, typename Clock, typename Cs, typename Sdo, typename Config = Defaults>
    class Reader {
    public:
        using TimePoint = typename Clock::time_point;

        static constexpr Rejection    RejectionSetting    = Config::rejection;
        static constexpr Part         PartSetting         = Config::part;
        static constexpr auto         Conversion          = conversionTime(Config::rejection);
        static constexpr auto         Timeout             = 2 * Conversion.max;
        static constexpr std::uint8_t AbsentAfterFailures = Config::AbsentAfterFailures;
        static constexpr auto         Setup               = Master::setup(Mode, Config::MaxClock);

        static_assert(Config::MaxClock <= MaxClock,
                      "fESCK is at most 2 MHz (LTC2401_LTC2402.md:188)");

        Reader() : next_{Clock::now() + Config::StartupDelay} { apply(set(Cs{})); }

        Reader(Reader const&)            = delete;
        Reader& operator=(Reader const&) = delete;

        /// The last good reading; meaningful once valid().
        [[nodiscard]] Sample const& latest() const { return latest_; }

        [[nodiscard]] bool valid() const { return samples_ != 0; }

        [[nodiscard]] std::uint32_t samples() const { return samples_; }

        /// Frames this part cannot send (Decoded::Kind::invalid / notReady after SDO was low).
        [[nodiscard]] std::uint32_t invalid() const { return invalid_; }

        /// No result within Timeout of CS going low or the last frame.
        [[nodiscard]] std::uint32_t timeouts() const { return timeouts_; }

        /// SDO still low after a frame: MISO stuck or floating low.
        [[nodiscard]] std::uint32_t stuckLow() const { return stuckLow_; }

        /// Frames the master failed or refused.
        [[nodiscard]] std::uint32_t errors() const { return errors_; }

        [[nodiscard]] Link link() const {
            if(failuresInRow_ == 0 && samples_ != 0) { return Link::answering; }
            return failuresInRow_ >= AbsentAfterFailures ? Link::absent : Link::starting;
        }

        [[nodiscard]] bool inFlight() const { return running_.load(std::memory_order_acquire); }

        void handler() {
            auto const now = Clock::now();
            if(running_.load(std::memory_order_acquire)) { return; }
            if(failed_.exchange(false, std::memory_order_acq_rel)) {
                ++errors_;
                fail_(now);
                return;
            }
            if(done_.exchange(false, std::memory_order_acq_rel)) {
                finish_(now);
                return;
            }
            switch(state_) {
            case State::startup:
            case State::released:
                if(now >= next_) {
                    apply(clear(Cs{}));   // SCK idles low (mode 0): external SCK mode
                    next_  = now + Timeout;
                    state_ = State::waiting;
                }
                break;
            case State::waiting:
                if(!apply(read(Sdo{}))) {
                    start_();
                } else if(now >= next_) {
                    ++timeouts_;
                    if(failuresInRow_ == 0) {
                        UC_LOG_W("ltc2401: no result in {} ms", Timeout.count() / 1000);
                    }
                    fail_(now);
                }
                break;
            }
        }

    private:
        enum class State : std::uint8_t { startup, released, waiting };

        static void select_() { apply(clear(Cs{})); }

        /// No deselect: CS stays low after the frame, SDO then shows EOC.
        static constexpr Lines lines_{&select_, nullptr, nullptr, nullptr};

        void start_() {
            frame_ = {};
            running_.store(true, std::memory_order_release);
            if(!Master::submit(
                 typename Master::Request{.setup = Setup,
                                          .lines = lines_,
                                          .rx    = std::span{frame_},
                                          .callback =
                                            [this](auto r) {
                                                if(r == decltype(r)::succeeded) {
                                                    // after the 32nd falling SCK edge SDO is EOC of the next conversion
                                                    sdoHighAfter_ = apply(read(Sdo{}));
                                                    done_.store(true, std::memory_order_relaxed);
                                                } else {
                                                    failed_.store(true, std::memory_order_relaxed);
                                                }
                                                running_.store(false, std::memory_order_release);
                                            }}))
            {
                failed_.store(true, std::memory_order_relaxed);
                running_.store(false, std::memory_order_release);
            }
        }

        void finish_(TimePoint now) {
            auto const got
              = decode(wordOf(std::span<std::byte const, FrameBytes>{frame_}), PartSetting);
            if(!sdoHighAfter_) {
                ++stuckLow_;
                if(failuresInRow_ == 0) {
                    UC_LOG_W("ltc2401: SDO still low after a frame - no part, or MISO stuck low");
                }
                fail_(now);
                return;
            }
            if(got.kind != Decoded::Kind::ok) {
                ++invalid_;
                if(failuresInRow_ == 0) {
                    UC_LOG_W("ltc2401: frame {:#010x} is not one this part sends",
                             wordOf(std::span<std::byte const, FrameBytes>{frame_}));
                }
                fail_(now);
                return;
            }
            latest_ = got.sample;
            ++samples_;
            if(failuresInRow_ != 0) { UC_LOG_I("ltc2401: answering"); }
            failuresInRow_ = 0;
            next_          = now + Timeout;
            state_         = State::waiting;
        }

        void fail_(TimePoint now) {
            if(failuresInRow_ != 0xFF) { ++failuresInRow_; }
            if(failuresInRow_ == AbsentAfterFailures) {
                UC_LOG_W("ltc2401: absent after {} failures in a row", failuresInRow_);
            }
            apply(set(Cs{}));   // aborts a data output in progress, starts a new conversion
            next_  = now + Config::RetryDelay;
            state_ = State::released;
        }

        TimePoint                         next_{};
        std::array<std::byte, FrameBytes> frame_{};
        std::atomic<bool>                 running_{false};
        std::atomic<bool>                 done_{false};
        std::atomic<bool>                 failed_{false};
        bool                              sdoHighAfter_{};
        State                             state_{State::startup};
        std::uint8_t                      failuresInRow_{};
        Sample                            latest_{};
        std::uint32_t                     samples_{};
        std::uint32_t                     invalid_{};
        std::uint32_t                     timeouts_{};
        std::uint32_t                     stuckLow_{};
        std::uint32_t                     errors_{};
    };

}}}   // namespace Kvasir::SPI::Ltc2400
