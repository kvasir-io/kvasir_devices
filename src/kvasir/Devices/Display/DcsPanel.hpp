#pragma once
#include "../Log.hpp"
#include "../ResetLine.hpp"
#include "Dcs.hpp"
#include "kvasir/StartUp/Hooks.hpp"
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

    /// What DcsPanel needs from a bus:
    ///   writeCommand(cmd, params)          synchronous; false on timeout
    ///   writePixels(bytes)                 asynchronous; `bytes` stays alive until idle()
    ///   fillPixels(hi, lo, count)          asynchronous; repeat one two-byte pixel
    ///   readRegister(instr, cmd, dummy, out)  synchronous; `dummy` bytes discarded first
    ///   idle(), failed(), handler()        transfer state, polled
    ///   clearError()                       un-latches failed() for the panel's retry
    template<typename B>
    concept Bus = requires(std::uint8_t               u8,
                           std::span<std::byte const> in,
                           std::span<std::byte>       out,
                           std::size_t                n) {
        { B::writeCommand(u8, in) } -> std::same_as<bool>;
        { B::writePixels(in) };
        { B::fillPixels(u8, u8, n) };
        { B::readRegister(u8, u8, n, out) } -> std::same_as<bool>;
        { B::idle() } -> std::same_as<bool>;
        { B::failed() } -> std::same_as<bool>;
        { B::handler() };
        { B::clearError() };
    };

    using Kvasir::ResetLine;

    namespace detail {
        /// The Pixel concept sees the members, not their order: a {lo, hi} layout would blit swapped.
        template<Pixel C>
        consteval bool highByteFirst() {
            static_assert(std::is_standard_layout_v<C>,
                          "a Pixel is standard-layout: two bytes, hi then lo");
            return offsetof(C, hi) == 0 && offsetof(C, lo) == 1;
        }
    }   // namespace detail

    /// Defaults for a panel config; a config derives from this and redeclares what it sets.
    struct DcsDefaults {
        static constexpr int          ColumnOffset = 0;
        static constexpr int          RowOffset    = 0;
        static constexpr std::uint8_t Brightness   = 0xA0;
        static constexpr bool         Invert       = false;
        static constexpr std::uint8_t Madctl       = 0;
        /// After a failure the bring-up runs again this much later. 0: stay failed.
        static constexpr unsigned RetryAfterMs = 1000;
    };

    /// A DCS controller's own facts, as constants:
    ///   Name              for the log
    ///   ReadInstruction   the bus instruction that starts a register read (03h, 0Bh)
    ///   ReadDummyBytes    bytes clocked out before the answer (0 or 1)
    ///   WindowAlign       CASET/RASET start and count must be multiples of this
    ///   MaxAddress        one past the largest column/row the address fields can hold
    ///   MaxWriteHz        the bus clock ceiling for writes; the transport is checked
    ///   MaxReadHz         ... and for reads
    ///   Colmod565         the COLMOD parameter for 16 bit pixels
    ///   ResetPulseMs      RESX low time
    ///   ResetToCommandMs  RESX high to the first command
    ///   SleepOutMs        SLPOUT to the next command
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
        { C::MaxWriteHz } -> std::convertible_to<unsigned long>;
        { C::MaxReadHz } -> std::convertible_to<unsigned long>;
        { C::Colmod565 } -> std::convertible_to<std::uint8_t>;
        { C::ResetPulseMs } -> std::convertible_to<unsigned>;
        { C::ResetToCommandMs } -> std::convertible_to<unsigned>;
        { C::SleepOutMs } -> std::convertible_to<unsigned>;
        { C::Init } -> std::convertible_to<Dcs::Script>;
        { C::Brightness } -> std::convertible_to<bool>;
        { C::CtrlDisplay } -> std::convertible_to<std::uint8_t>;
        { C::ExpectedId1 } -> std::convertible_to<std::uint8_t>;
    };

    /// Driver for any DCS controller. The module's facts come from `Config_`:
    ///   Width, Height         active area in pixels                       (required)
    ///   ColumnOffset (0), RowOffset (0)   where the glass sits in controller RAM
    ///   Brightness   (0xA0)   initial DBV[7:0], where the chip supports it
    ///   Invert       (false)  send INVON during bring-up
    ///   Madctl       (0)      MADCTL bits (Dcs::Madctl), for orientation and BGR
    ///
    /// Bring-up: RESX pulse (none with Kvasir::NoReset), the chip's init script, COLMOD,
    /// MADCTL, brightness, INVON, SLPOUT and its wait, a black fill, DISPON. The fill comes
    /// before DISPON because RAM is undefined after reset. Every wait is a state polled by
    /// handler(), not a delay.
    template<typename Clock, Bus BusT, ResetLine Reset, Chip ChipT, typename Config_>
    struct DcsPanel {
        using Chip = ChipT;
        using Bus  = BusT;

        static_assert(std::derived_from<Config_,
                                        DcsDefaults>,
                      "derive the panel config from Kvasir::Display::DcsDefaults");
        using Config = Config_;

        static constexpr int Width  = static_cast<int>(Config::Width);
        static constexpr int Height = static_cast<int>(Config::Height);

        /// Window start and size must be multiples of this (gfx::Display contract).
        static constexpr int WindowAlign = Chip::WindowAlign;

        static_assert(Width > 0 && Height > 0);
        static_assert(Config::ColumnOffset % Chip::WindowAlign == 0
                        && Width % Chip::WindowAlign == 0,
                      "this controller needs the column start and count aligned (WindowAlign)");
        static_assert(Config::RowOffset % Chip::WindowAlign == 0 && Height % Chip::WindowAlign == 0,
                      "this controller needs the row start and count aligned (WindowAlign)");
        static_assert(Config::ColumnOffset + Width <= Chip::MaxAddress
                        && Config::RowOffset + Height <= Chip::MaxAddress,
                      "the window does not fit the controller's address fields");

        // Checked only when the bus says what it runs at (a mock bus does not).
        static constexpr unsigned long BusWriteHz = [] {
            if constexpr(requires { Bus::WriteHz; }) {
                return static_cast<unsigned long>(Bus::WriteHz);
            } else {
                return 0UL;
            }
        }();

        static constexpr unsigned long BusReadHz = [] {
            if constexpr(requires { Bus::ReadHz; }) {
                return static_cast<unsigned long>(Bus::ReadHz);
            } else {
                return 0UL;
            }
        }();

        static_assert(BusWriteHz <= Chip::MaxWriteHz,
                      "the bus write clock exceeds what this controller allows");
        static_assert(BusReadHz <= Chip::MaxReadHz,
                      "the bus read clock exceeds what this controller allows");

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

        static constexpr std::byte hiByte(int v) {
            return static_cast<std::byte>((static_cast<unsigned>(v) >> 8U) & 0xFFU);
        }

        static constexpr std::byte loByte(int v) {
            return static_cast<std::byte>(static_cast<unsigned>(v) & 0xFFU);
        }

        static void fail() { state_ = State::failed; }

        static inline Kvasir::Deadline<Clock> retry_{};   ///< armed when a failure is seen
        static inline std::uint32_t           failures_{};
        static inline bool                    failureSeen_{};

        static constexpr unsigned RetryAfterMs = static_cast<unsigned>(Config::RetryAfterMs);

        static constexpr bool HasResetLine = !requires { requires Reset::NoLine; };

        static bool send(std::uint8_t               cmd,
                         std::span<std::byte const> params = {}) {
            if(!Bus::writeCommand(cmd, params)) {
                fail();
                return false;
            }
            return true;
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
            std::array<std::byte, 4> const caset{hiByte(x0), loByte(x0), hiByte(x1), loByte(x1)};
            std::array<std::byte, 4> const raset{hiByte(y0), loByte(y0), hiByte(y1), loByte(y1)};
            if(send(Dcs::Cmd::Caset, caset)) { send(Dcs::Cmd::Raset, raset); }
        }

        /// All before SLPOUT, where every datasheet's power-on figure puts them.
        static bool format() {
            if(!send1(Dcs::Cmd::Colmod, Chip::Colmod565)) { return false; }
            if(!send1(Dcs::Cmd::Madctl, Config::Madctl)) { return false; }
            if constexpr(Chip::Brightness) {
                if(!send1(Dcs::Cmd::Wrctrld, Chip::CtrlDisplay)) { return false; }
                if(!send1(Dcs::Cmd::Wrdisbv, Config::Brightness)) { return false; }
            }
            if constexpr(Config::Invert) {
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
        }

        static bool initialised() { return state_ == State::ready; }

        /// A new draw call may start.
        static bool ready() { return initialised() && Bus::idle(); }

        static bool failed() { return state_ == State::failed; }

        static std::uint32_t failures() { return failures_; }

        static State state() { return state_; }

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

            auto const x0 = r.x + Config::ColumnOffset;
            auto const y0 = r.y + Config::RowOffset;
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

        static std::optional<std::uint8_t> readRegister8(std::uint8_t cmd) {
            std::array<std::byte, 1> b{};
            if(!Bus::readRegister(Chip::ReadInstruction, cmd, Chip::ReadDummyBytes, b)) {
                return std::nullopt;
            }
            return static_cast<std::uint8_t>(b[0]);
        }

        /// From the main loop.
        static void handler() {
            Bus::handler();

            // A payload transfer fails inside Bus::handler(), where no send() sees it.
            if(Bus::failed() && state_ != State::failed) { state_ = State::failed; }

            auto const now = Clock::now();

            if(state_ == State::failed && !failureSeen_) {
                failureSeen_ = true;
                retry_.restart(std::chrono::milliseconds{RetryAfterMs}, now);
                ++failures_;
                UC_LOG_W(
                  "{}: the panel or its bus failed (failure {}), bringing it up again in {} ms",
                  Chip::Name,
                  failures_,
                  RetryAfterMs);
            }

            switch(state_) {
            case State::resetAssert:
                if constexpr(HasResetLine) {
                    reset_.begin(now,
                                 std::chrono::milliseconds{Chip::ResetPulseMs},
                                 std::chrono::milliseconds{Chip::ResetToCommandMs});
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
                if(now > waitUntil_ && Bus::idle()) {
                    Dcs::Step step{};
                    if(Chip::Init.next(scriptPos_, step)) {
                        if(!send(step.cmd, std::as_bytes(step.params))) { break; }
                        waitUntil_ = now + std::chrono::milliseconds{step.delayMs};
                    } else {
                        state_ = State::format;
                    }
                }
                break;

            case State::format:
                if(!format()) { break; }
                if(!send(Dcs::Cmd::Slpout)) { break; }
                waitUntil_ = now + std::chrono::milliseconds{Chip::SleepOutMs};
                state_     = State::sleepOut;
                break;

            case State::sleepOut:
                if(now > waitUntil_) { state_ = State::clearScreen; }
                break;

            case State::clearScreen:
                fill(FullScreen, Rgb565{});   // black
                if(!failed()) { state_ = State::clearWait; }
                break;

            case State::clearWait:
                if(Bus::idle()) { state_ = State::displayOn; }
                break;

            case State::displayOn:
                if(send(Dcs::Cmd::Dispon)) { state_ = State::ready; }
                break;

            case State::ready: break;
            case State::failed:
                if(RetryAfterMs != 0 && retry_.expired(now)) {
                    Bus::clearError();
                    restart();
                }
                break;
            }
        }

        // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() / SecondaryCore::run calls
        // it (StartUp/Hooks.hpp); a firmware that runs the hook must not also call it by hand
        using Extends = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &DcsPanel::handler>;
    };

}}   // namespace Kvasir::Display
