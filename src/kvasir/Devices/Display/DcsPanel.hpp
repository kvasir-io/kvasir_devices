#pragma once
#include "../Bytes.hpp"
#include "../Duration.hpp"
#include "../Log.hpp"
#include "../Quantities.hpp"
#include "../ResetLine.hpp"
#include "Bus.hpp"
#include "Dcs.hpp"
#include "kvasir/Port/Extends.hpp"
#include "kvasir/Util/Backoff.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Link.hpp"
#include "kvasir/Util/Periodic.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace Display {

    /// A pixel in 16-bit mode (COLMOD 55h): two bytes, high first. Any type with `hi` and `lo`
    /// bytes is one (gfx::Rgb565 is): gfx::Display matches structurally.
    struct Rgb565 {
        std::byte hi{};
        std::byte lo{};
    };

    struct Rect {
        int x{};
        int y{};
        int w{};
        int h{};
    };

    template<typename C>
    concept Pixel = sizeof(C) == 2 && std::is_trivially_copyable_v<C> && requires(C c) {
        { c.hi } -> std::convertible_to<std::byte>;
        { c.lo } -> std::convertible_to<std::byte>;
    };

    template<typename R>
    concept Area = requires(R r) {
        { r.x } -> std::convertible_to<int>;
        { r.y } -> std::convertible_to<int>;
        { r.w } -> std::convertible_to<int>;
        { r.h } -> std::convertible_to<int>;
    };

    namespace detail {
        /// The Pixel concept sees the members, not their order: a {lo, hi} layout would blit swapped.
        template<Pixel C>
        consteval bool highByteFirst() {
            static_assert(std::is_standard_layout_v<C>,
                          "a Pixel is standard-layout: two bytes, hi then lo");
            return offsetof(C, hi) == 0 && offsetof(C, lo) == 1;
        }

        /// Named so that a failed check shows both numbers: the commands DcsPanel queues in one
        /// turn (`Burst`) against what the bus holds at once (`Slots`, its CommandSlots).
        template<std::size_t Burst, std::size_t Slots>
        inline constexpr bool busHoldsFormatBurst = Burst <= Slots;
    }   // namespace detail

    /// Defaults for a panel config; a config derives from this and redeclares what it sets.
    struct DcsDefaults {
        static constexpr int          columnOffset = 0;
        static constexpr int          rowOffset    = 0;
        static constexpr std::uint8_t brightness   = 0xA0;
        static constexpr bool         invert       = false;
        static constexpr std::uint8_t madctl       = 0;
        /// After a failure the bring-up runs again this much later, the pause doubling while it keeps failing (up
        /// to ten minutes, or retryAfter if that is longer); a bring-up that completes starts it over. Above 0.
        static constexpr std::chrono::milliseconds retryAfter{1000};
    };

    /// A DCS controller's own facts, as constants:
    ///   Name              for the log
    ///   ReadInstruction   the bus instruction that starts a register read (03h, 0Bh)
    ///   ReadDummyBytes    bytes clocked out before the answer (0 or 1)
    ///   WindowAlign       CASET/RASET start and count must be multiples of this
    ///   MaxAddress        one past the largest column/row the address fields can hold
    ///   MaxWriteClock     the bus clock ceiling for writes (Units::Hertz); the transport is checked
    ///   MaxReadClock      ... and for reads
    ///   Colmod565         the COLMOD parameter for 16 bit pixels
    ///   ResetPulse        RESX low time (std::chrono::milliseconds, as the three below)
    ///   ResetToCommand    RESX high to the first command
    ///   SleepOut          SLPOUT (its completion) to the next command
    ///   Init              the chip's own script (Dcs::Script), sent before COLMOD
    ///   Brightness        whether WRCTRLD/WRDISBV do anything on this chip
    ///   CtrlDisplay       the WRCTRLD parameter that enables brightness control
    ///   ExpectedId1       RDDID's first byte, or 0 if unknown
    template<typename C>
    concept Chip = requires {
        { C::Name } -> std::convertible_to<std::string_view>;
        { C::ReadInstruction } -> std::convertible_to<std::uint8_t>;
        { C::ReadDummyBytes } -> std::convertible_to<std::size_t>;
        { C::WindowAlign } -> std::convertible_to<int>;
        { C::MaxAddress } -> std::convertible_to<int>;
        { C::MaxWriteClock } -> std::convertible_to<Units::Hertz>;
        { C::MaxReadClock } -> std::convertible_to<Units::Hertz>;
        { C::Colmod565 } -> std::convertible_to<std::uint8_t>;
        { C::ResetPulse } -> std::convertible_to<std::chrono::milliseconds>;
        { C::ResetToCommand } -> std::convertible_to<std::chrono::milliseconds>;
        { C::SleepOut } -> std::convertible_to<std::chrono::milliseconds>;
        { C::Init } -> std::convertible_to<Dcs::Script>;
        { C::Brightness } -> std::convertible_to<bool>;
        { C::CtrlDisplay } -> std::convertible_to<std::uint8_t>;
        { C::ExpectedId1 } -> std::convertible_to<std::uint8_t>;
    };

    /// Driver for any DCS controller. The module's facts come from `UserConfig`:
    ///   Width, Height         active area in pixels                       (required)
    ///   columnOffset (0), rowOffset (0)   where the glass sits in controller RAM
    ///   brightness   (0xA0)   initial DBV[7:0], where the chip supports it
    ///   invert       (false)  send INVON during bring-up
    ///   madctl       (0)      MADCTL bits (Dcs::Madctl), for orientation and BGR
    ///
    /// Bring-up: RESX pulse (none with Kvasir::NoReset), the chip's init script, COLMOD,
    /// MADCTL, brightness, INVON, SLPOUT and its wait, a black fill, DISPON. The fill comes
    /// before DISPON because RAM is undefined after reset. Every wait is a state polled by
    /// turn(), not a delay.
    template<typename Clock, Bus BusT, ResetLine Reset, Chip ChipT, typename UserConfig>
    struct DcsPanel {
        using Chip = ChipT;
        using Bus  = BusT;

        static_assert(std::derived_from<UserConfig,
                                        DcsDefaults>,
                      "derive the panel config from Kvasir::Display::DcsDefaults");
        using Config = UserConfig;

        static constexpr std::chrono::milliseconds RetryAfter
          = Kvasir::asDuration(Config::retryAfter);
        static_assert(RetryAfter > std::chrono::milliseconds::zero(),
                      "retryAfter is the pause before a failed panel is brought up again");

        static constexpr int Width  = static_cast<int>(Config::Width);
        static constexpr int Height = static_cast<int>(Config::Height);

        /// Window start and size must be multiples of this (gfx::Display contract).
        static constexpr int WindowAlign = Chip::WindowAlign;

        static_assert(Width > 0 && Height > 0);
        static_assert(Config::columnOffset % Chip::WindowAlign == 0
                        && Width % Chip::WindowAlign == 0,
                      "this controller needs the column start and count aligned (WindowAlign)");
        static_assert(Config::rowOffset % Chip::WindowAlign == 0 && Height % Chip::WindowAlign == 0,
                      "this controller needs the row start and count aligned (WindowAlign)");
        static_assert(Config::columnOffset + Width <= Chip::MaxAddress
                        && Config::rowOffset + Height <= Chip::MaxAddress,
                      "the window does not fit the controller's address fields");

        // Checked only when the bus says what it runs at (a mock bus does not).
        static constexpr unsigned long BusWriteHz
          = static_cast<unsigned long>(KVASIR_CONFIG_OR(Bus, writeHz, 0UL));

        static constexpr unsigned long BusReadHz
          = static_cast<unsigned long>(KVASIR_CONFIG_OR(Bus, readHz, 0UL));

        static_assert(Units::hertz(BusWriteHz) <= Units::Hertz{Chip::MaxWriteClock},
                      "the bus write clock exceeds what this controller allows");
        static_assert(Units::hertz(BusReadHz) <= Units::Hertz{Chip::MaxReadClock},
                      "the bus read clock exceeds what this controller allows");

        /// The commands one turn() hands the bus without waiting for it: format() (COLMOD,
        /// MADCTL, WRCTRLD + WRDISBV with brightness control, INVON when inverted) and SLPOUT.
        /// Every other step waits until the bus is idle.
        static constexpr std::size_t FormatBurst
          = 3U + (Chip::Brightness ? 2U : 0U) + (Config::invert ? 1U : 0U);

        // Checked only when the bus queues its commands (SpiDcsBus: CommandSlots, itself checked
        // against the SPI master's QueueDepth); a bus that sends each command at once holds any burst.
        static_assert(detail::busHoldsFormatBurst<FormatBurst,
                                                  KVASIR_CONFIG_OR(Bus,
                                                                   CommandSlots,
                                                                   FormatBurst)>,
                      "the bus queues fewer commands than format() and SLPOUT hand it in one turn: "
                      "the last of them would be refused and every bring-up fail - raise the bus's "
                      "commandSlots (and the SPI master's QueueDepth with it)");

        using Color = Display::Rgb565;
        using Rect  = Display::Rect;

        static constexpr Rect FullScreen{0, 0, Width, Height};

        struct DeviceId {
            std::uint8_t id1{};
            std::uint8_t id2{};
            std::uint8_t id3{};
        };

        enum class State : std::uint8_t {
            resetAssert,
            resetRelease,
            script,
            format,
            sleepOut,
            clearScreen,
            clearWait,
            displayOn,
            ready,
            failed,
        };

    private:
        static inline State                            state_{State::resetAssert};
        static inline typename Clock::time_point       waitUntil_{};
        static inline Kvasir::ResetPulse<Reset, Clock> reset_{};
        static inline std::size_t                      scriptPos_{};
        /// A wait that starts once the bus is idle: the command it follows is still queued. A
        /// duration (Millis32), not a point in time: waited() turns it into waitUntil_.
        static inline Millis32 delay_{};

        static void fail() { state_ = State::failed; }

        /// The retry schedule: the first pause retryAfter, doubling up to ten minutes (or retryAfter if that is
        /// longer).
        static inline Kvasir::
          Backoff<Clock, Kvasir::RetryFromMs<static_cast<std::uint64_t>(RetryAfter.count())>>
                                    retry_{};   ///< failed when a failure is seen
        static inline std::uint32_t failures_{};
        static inline bool          failureSeen_{};

        static constexpr bool HasResetLine = !requires { requires Reset::NoLine; };

        static bool send(std::uint8_t               cmd,
                         std::span<std::byte const> params = {}) {
            if(!Bus::writeCommand(cmd, params)) {
                fail();
                return false;
            }
            return true;
        }

        /// `d` from now, or from when the bus has sent what is queued.
        static void waitAfter(typename Clock::time_point now,
                              std::chrono::milliseconds  d) {
            if(Bus::idle()) {
                waitUntil_ = now + d;
                delay_     = {};
            } else {
                delay_ = d;
            }
        }

        /// The bus idle and the wait since then over.
        [[nodiscard]] static bool waited(typename Clock::time_point now) {
            if(!Bus::idle()) { return false; }
            if(delay_ != Millis32{}) {
                waitUntil_ = now + delay_;
                delay_     = {};
            }
            return now > waitUntil_;
        }

        static bool send1(std::uint8_t cmd,
                          std::uint8_t p) {
            std::array<std::byte, 1> const v{std::byte{p}};
            return send(cmd, v);
        }

        static void window(int x0,
                           int y0,
                           int x1,
                           int y1) {
            // start and end, 16 bits each, high byte first (MIPI DCS set_column/page_address)
            auto const span = [](int first, int last) {
                std::array<std::byte, 4> a{};
                putBe16(a, 0, static_cast<std::uint16_t>(first));
                putBe16(a, 2, static_cast<std::uint16_t>(last));
                return a;
            };
            auto const caset = span(x0, x1);
            auto const raset = span(y0, y1);
            if(send(Dcs::Cmd::Caset, caset)) { send(Dcs::Cmd::Raset, raset); }
        }

        /// All before SLPOUT, where every datasheet's power-on figure puts them.
        static bool format() {
            if(!send1(Dcs::Cmd::Colmod, Chip::Colmod565)) { return false; }
            if(!send1(Dcs::Cmd::Madctl, Config::madctl)) { return false; }
            if constexpr(Chip::Brightness) {
                if(!send1(Dcs::Cmd::Wrctrld, Chip::CtrlDisplay)) { return false; }
                if(!send1(Dcs::Cmd::Wrdisbv, Config::brightness)) { return false; }
            }
            if constexpr(Config::invert) {
                if(!send(Dcs::Cmd::Invon)) { return false; }
            }
            return true;
        }

    public:
        static constexpr std::string_view Name = Chip::Name;

        /// Back to the start of bring-up. The bus is reset separately (Bus::primaryPrepare).
        static void restart() {
            failureSeen_ = false;
            state_       = State::resetAssert;
            scriptPos_   = 0;
            waitUntil_   = {};
            delay_       = {};
        }

        static bool initialised() { return state_ == State::ready; }

        /// A new draw call may start.
        static bool ready() { return initialised() && Bus::idle(); }

        /// The transfer/contract state: a command was refused or a transfer failed, and nothing is
        /// sent until the retry brings the panel up again. Not the health: that is link().
        static bool failed() { return state_ == State::failed; }

        /// The health (SDK Runtime/README.md "Observability"; Health::LinkCheck watches it):
        /// answering once brought up, absent while failed, starting in bring-up. Back to answering by itself: the
        /// retry (retryAfter, doubling) brings the panel up again.
        [[nodiscard]] static Link link() {
            if(failed()) { return Link::absent; }
            return initialised() ? Link::answering : Link::starting;
        }

        static std::uint32_t failures() { return failures_; }

        static State state() { return state_; }

        /// One command of the caller's, queued on the bus (Display::Bus): `params` must stay valid until the bus is
        /// idle() where the bus does not copy them (SpiDcsBus: more than `copiedParams` bytes). False = refused.
        static bool sendCommand(std::uint8_t               cmd,
                                std::span<std::byte const> params = {}) {
            return Bus::writeCommand(cmd, params);
        }

        /// In screen coordinates; the module's RAM offset is added here.
        template<Area R>
        static void setWindow(R const& r) {
            assert(r.w > 0 && r.h > 0 && r.x >= 0 && r.y >= 0);
            assert(r.x + r.w <= Width && r.y + r.h <= Height);
            // A misaligned window silently renders something else.
            assert(r.x % Chip::WindowAlign == 0 && r.w % Chip::WindowAlign == 0);
            assert(r.y % Chip::WindowAlign == 0 && r.h % Chip::WindowAlign == 0);

            auto const x0 = r.x + Config::columnOffset;
            auto const y0 = r.y + Config::rowOffset;
            window(x0, y0, x0 + r.w - 1, y0 + r.h - 1);
        }

        /// In controller RAM coordinates, no offset, no clamp: for finding where the glass
        /// sits in RAM (the CO5300 has 480 x 480 behind a 466 x 466 module). Not for drawing.
        static void setWindowRaw(int x0,
                                 int y0,
                                 int x1,
                                 int y1) {
            assert(x0 >= 0 && y0 >= 0 && x0 <= x1 && y0 <= y1);
            assert(x1 < Chip::MaxAddress && y1 < Chip::MaxAddress);
            assert(x0 % Chip::WindowAlign == 0 && (x1 - x0 + 1) % Chip::WindowAlign == 0);
            window(x0, y0, x1, y1);
        }

        template<Pixel C>
        static void blitRaw(int                x0,
                            int                y0,
                            int                x1,
                            int                y1,
                            std::span<C const> pixels) {
            static_assert(detail::highByteFirst<C>(), "a Pixel is {hi, lo}, high byte first");
            assert(pixels.size()
                   == static_cast<std::size_t>(x1 - x0 + 1)
                        * static_cast<std::size_t>(y1 - y0 + 1));
            setWindowRaw(x0, y0, x1, y1);
            if(failed()) { return; }
            Bus::writePixels(std::as_bytes(pixels));
        }

        /// Asynchronous; the bus repeats one pixel, no buffer.
        template<Area  R,
                 Pixel C>
        static void fill(R const& r,
                         C        c) {
            static_assert(detail::highByteFirst<C>(), "a Pixel is {hi, lo}, high byte first");
            setWindow(r);
            if(failed()) { return; }
            Bus::fillPixels(static_cast<std::uint8_t>(c.hi),
                            static_cast<std::uint8_t>(c.lo),
                            static_cast<std::size_t>(r.w) * static_cast<std::size_t>(r.h));
        }

        template<Pixel C>
        static void fillScreen(C c) {
            fill(FullScreen, c);
        }

        /// Asynchronous, zero-copy: `pixels` must stay unmodified until ready().
        template<Area  R,
                 Pixel C>
        static void blit(R const&           r,
                         std::span<C const> pixels) {
            static_assert(detail::highByteFirst<C>(), "a Pixel is {hi, lo}, high byte first");
            assert(pixels.size() == static_cast<std::size_t>(r.w) * static_cast<std::size_t>(r.h));
            setWindow(r);
            if(failed()) { return; }
            Bus::writePixels(std::as_bytes(pixels));
        }

        static void setBrightness(std::uint8_t dbv) {
            if constexpr(Chip::Brightness) { send1(Dcs::Cmd::Wrdisbv, dbv); }
        }

        static void displayOn(bool on) { send(on ? Dcs::Cmd::Dispon : Dcs::Cmd::Dispoff); }

        static void sleep(bool on) { send(on ? Dcs::Cmd::Slpin : Dcs::Cmd::Slpout); }

        static void invert(bool on) { send(on ? Dcs::Cmd::Invon : Dcs::Cmd::Invoff); }

        /// Tells a broken bus from a misconfigured panel: both look like a black screen.
        static std::optional<DeviceId> readId() {
            std::array<std::byte, 3> b{};
            if(!Bus::readRegister(Chip::ReadInstruction, Dcs::Cmd::Rddid, Chip::ReadDummyBytes, b))
            {
                return std::nullopt;
            }
            return DeviceId{static_cast<std::uint8_t>(b[0]),
                            static_cast<std::uint8_t>(b[1]),
                            static_cast<std::uint8_t>(b[2])};
        }

        /// From the main loop.
        static void turn() {
            Bus::turn();

            // A payload transfer fails inside Bus::turn(), where no send() sees it.
            if(Bus::failed() && state_ != State::failed) { state_ = State::failed; }

            auto const now = Clock::now();

            if(state_ == State::failed && !failureSeen_) {
                failureSeen_ = true;
                retry_.failed(now);
                ++failures_;
                UC_LOG_W(
                  "{}: the panel or its bus failed (failure {}), bringing it up again in {} ms",
                  Chip::Name,
                  failures_,
                  static_cast<long long>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(retry_.next - now)
                      .count()));
            }

            switch(state_) {
            case State::resetAssert:
                if constexpr(HasResetLine) {
                    reset_.begin(now, Chip::ResetPulse, Chip::ResetToCommand);
                    state_ = State::resetRelease;
                } else {
                    // The script's SWRESET does the resetting (ST7789V.md:5588).
                    waitUntil_ = now;
                    scriptPos_ = 0;
                    state_     = State::script;
                }
                break;

            case State::resetRelease:
                if(reset_.done(now)) {
                    waitUntil_ = now;
                    scriptPos_ = 0;
                    state_     = State::script;
                }
                break;

            case State::script:
                if(waited(now)) {
                    Dcs::Step step{};
                    if(Chip::Init.next(scriptPos_, step)) {
                        if(!send(step.cmd, std::as_bytes(step.params))) { break; }
                        waitAfter(now, std::chrono::milliseconds{step.delayMs});
                    } else {
                        state_ = State::format;
                    }
                }
                break;

            case State::format:
                if(!format()) { break; }
                if(!send(Dcs::Cmd::Slpout)) { break; }
                waitAfter(now, Chip::SleepOut);
                state_ = State::sleepOut;
                break;

            case State::sleepOut:
                if(waited(now)) { state_ = State::clearScreen; }
                break;

            case State::clearScreen:
                fill(FullScreen, Rgb565{});   // black
                if(!failed()) { state_ = State::clearWait; }
                break;

            case State::clearWait:
                if(Bus::idle()) { state_ = State::displayOn; }
                break;

            case State::displayOn:
                if(send(Dcs::Cmd::Dispon)) {
                    state_ = State::ready;
                    retry_.succeeded();
                }
                break;

            case State::ready: break;
            case State::failed:
                if(retry_.due(now)) {
                    Bus::clearError();
                    restart();
                }
                break;
            }
        }

        // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() / SecondaryCore::run calls
        // it (Port/Extends.hpp); a firmware that runs the hook must not also call it by hand
        using Extends = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &DcsPanel::turn>;
    };

}}   // namespace Kvasir::Display
