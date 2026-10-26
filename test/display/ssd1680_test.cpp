/// The SSD1680 e-paper driver against a mock Display::DataBus and BUSY: what goes on the wire.
/// Not whether the waveform suits a given module.
///
///   ssd1680_test [-v]     -v prints every command
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <kd/Case.hpp>
#include <kvasir/Devices/Display/Ssd1680.hpp>
#include <span>
#include <string>
#include <string_view>
#include <support/FakeClock.hpp>
#include <vector>

namespace {

/// What gfx::MonoFrame does for a MonoPanel, cut down to what the test draws: clear the page RAM,
/// set pixels, hand every page over (the panel hashes them and sends only what changed).
template<typename P>
struct MiniFrame {
    struct Canvas {
        void clear() {
            for(auto& page : P::pageRam()) { page.fill(0); }
        }

        void pixel(int x,
                   int y) {
            P::pageRam()[static_cast<std::size_t>(y / 8)][static_cast<std::size_t>(x)]
              |= static_cast<std::uint8_t>(1U << static_cast<unsigned>(y % 8));
        }
    };

    template<typename F>
    bool frame(F&& draw) {
        if(!P::ready()) { return false; }
        Canvas c{};
        draw(c);
        P::pagesChanged(P::Pages >= 32 ? 0xFFFF'FFFFU
                                       : (1U << static_cast<unsigned>(P::Pages)) - 1U);
        return true;
    }
};

bool verbose = false;

using FakeClock = Kvasir::Test::FakeClock;
using Cmd       = Kvasir::Display::Ssd1680Cmd;

struct Transfer {
    bool                      data{};
    std::vector<std::uint8_t> bytes;
    FakeClock::time_point     at;
};

/// A command with its parameters, and for WRITE RAM the whole frame that followed it.
struct Command {
    std::uint8_t              cmd{};
    std::vector<std::uint8_t> params;
};

/// High for a few milliseconds after a SW reset and for a refresh's length after an
/// activation, as the controller is; `stuck` holds it high for ever.
struct MockBusy {
    static inline FakeClock::time_point until{};
    static inline bool                  stuck{};

    static bool busy() { return stuck || FakeClock::now() < until; }

    static void onCommand(std::uint8_t cmd) {
        if(cmd == Cmd::SwReset) { until = FakeClock::now() + std::chrono::milliseconds{3}; }
        if(cmd == Cmd::MasterActivation) {
            until = FakeClock::now() + std::chrono::milliseconds{400};
        }
    }
};

struct MockReset {
    static inline FakeClock::time_point holdAt{};
    static inline FakeClock::time_point releaseAt{};

    static void hold() { holdAt = FakeClock::now(); }

    static void release() { releaseAt = FakeClock::now(); }
};

/// Records every transfer with its D/C level: a command as its byte (D/C low) and its parameters
/// (D/C high), a RAM chunk as data. Busy for two turns after each; refuses while busy (the driver
/// waits for idle()).
struct MockBus {
    static inline std::vector<Transfer> log{};
    static inline int                   polls{};
    static inline bool                  commandWhileBusy{};
    static inline bool                  refused{};
    static inline bool                  failNext{};   ///< the transfer in flight fails
    static inline bool                  failed_{};

    static bool writeCommand(std::uint8_t               cmd,
                             std::span<std::byte const> params) {
        if(polls > 0) {
            refused = true;
            return false;
        }
        if(MockBusy::busy()) { commandWhileBusy = true; }
        MockBusy::onCommand(cmd);
        log.push_back({false, {cmd}, FakeClock::now()});
        if(!params.empty()) {
            Transfer t{true, {}, FakeClock::now()};
            for(auto const b : params) { t.bytes.push_back(static_cast<std::uint8_t>(b)); }
            log.push_back(std::move(t));
        }
        polls = 2;
        return true;
    }

    static bool writeData(std::span<std::byte const> bytes) {
        if(polls > 0) {
            refused = true;
            return false;
        }
        Transfer t{true, {}, FakeClock::now()};
        for(auto const b : bytes) { t.bytes.push_back(static_cast<std::uint8_t>(b)); }
        log.push_back(std::move(t));
        polls = 2;
        return true;
    }

    [[maybe_unused]] static void writePixels(std::span<std::byte const>) {}

    [[maybe_unused]] static void fillPixels(std::uint8_t,
                                            std::uint8_t,
                                            std::size_t) {}

    [[maybe_unused]] static bool readRegister(std::uint8_t,
                                              std::uint8_t,
                                              std::size_t,
                                              std::span<std::byte>) {
        return false;
    }

    static bool idle() { return polls == 0; }

    static bool failed() { return failed_; }

    static void clearError() { failed_ = false; }

    static void turn() {
        if(polls == 0) { return; }
        if(failNext) {
            failNext = false;
            polls    = 0;
            failed_  = true;
            return;
        }
        --polls;
    }
};

static_assert(Kvasir::Display::DataBus<MockBus>);
static_assert(Kvasir::BusyLine<MockBusy>);
static_assert(Kvasir::ResetLine<MockReset>);

void resetMocks() {
    MockBus::log.clear();
    MockBus::polls            = 0;
    MockBus::commandWhileBusy = false;
    MockBus::refused          = false;
    MockBus::failNext         = false;
    MockBus::failed_          = false;
    MockBusy::until           = {};
    MockBusy::stuck           = false;
    FakeClock::reset();
}

/// The log as commands, data transfers folded into the command before them.
std::vector<Command> commands(std::size_t from = 0) {
    std::vector<Command> out{};
    for(std::size_t i = from; i < MockBus::log.size(); ++i) {
        auto const& t = MockBus::log[i];
        if(!t.data) {
            for(auto const b : t.bytes) { out.push_back({b, {}}); }
        } else if(!out.empty()) {
            out.back().params.insert(out.back().params.end(), t.bytes.begin(), t.bytes.end());
        }
    }
    return out;
}

std::vector<Command> without(std::vector<Command> cs,
                             std::uint8_t         cmd) {
    std::erase_if(cs, [cmd](Command const& c) { return c.cmd == cmd; });
    return cs;
}

std::string describe(Command const& c) {
    char buf[16];
    std::snprintf(buf, sizeof buf, "%02Xh", c.cmd);
    std::string s = buf;
    if(c.cmd == Cmd::WriteRamBw || c.cmd == Cmd::WriteLut) {
        return s + " x" + std::to_string(c.params.size());
    }
    for(auto const p : c.params) {
        std::snprintf(buf, sizeof buf, " %02X", p);
        s += buf;
    }
    return s;
}

using Kvasir::Test::check;

/// Polls a millisecond per turn until `done` or `limitMs`.
template<typename Done>
void pump(Done&& done,
          int    limitMs) {
    for(int i = 0; i < limitMs && !done(); ++i) {
        FakeClock::advance(std::chrono::milliseconds{1});
        for(int t = 0; t < 4; ++t) { done.panelHandler(); }
    }
}

template<typename Panel>
struct UntilReady {
    static void panelHandler() { Panel::turn(); }

    bool operator()() const { return Panel::ready() || Panel::failed(); }
};

template<typename Panel>
struct Forever {
    static void panelHandler() { Panel::turn(); }

    bool operator()() const { return Panel::failed(); }
};

/// Up again: failed() does not count as done.
template<typename Panel>
struct UntilUp {
    static void panelHandler() { Panel::turn(); }

    bool operator()() const { return Panel::ready(); }
};

/// Every turn of the pump, nothing ends it early.
template<typename Panel>
struct Never {
    static void panelHandler() { Panel::turn(); }

    bool operator()() const { return false; }
};

/// Until the panel has seen one more failure than `before`.
template<typename Panel>
struct UntilFailure {
    std::uint32_t before{};

    static void panelHandler() { Panel::turn(); }

    bool operator()() const { return Panel::failures() > before; }
};

/// Ink at gate line `y`, source `x` of a WRITE RAM frame (0 bits are black).
bool inkAt(Command const& ram,
           int            rowBytes,
           int            y,
           int            x) {
    auto const at   = (static_cast<std::size_t>(y) * static_cast<std::size_t>(rowBytes))
                    + static_cast<std::size_t>(x / 8);
    auto const byte = ram.params[at];
    return ((byte >> (7 - (x % 8))) & 1) == 0;
}

std::size_t inkCount(Command const& ram) {
    std::size_t n = 0;
    for(auto const b : ram.params) {
        n += static_cast<std::size_t>(8 - __builtin_popcount(static_cast<unsigned>(b)));
    }
    return n;
}

Command const* lastRam(std::vector<Command> const& cs) {
    Command const* r = nullptr;
    for(auto const& c : cs) {
        if(c.cmd == Cmd::WriteRamBw) { r = &c; }
    }
    return r;
}

template<typename Panel>
void exercise(char const* name) {
    using Config          = typename Panel::Config;
    int const  width      = Panel::Width;
    int const  height     = Panel::Height;
    auto const rowBytes   = Panel::RowBytes;
    auto const frameBytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(rowBytes);
    Kvasir::Test::Case const thisCase{name};

    std::printf("  %d x %d%s%s\n",
                width,
                height,
                Config::flip ? ", flipped" : "",
                Config::partial ? ", partial" : "");

    resetMocks();
    Panel::restart();
    pump(UntilReady<Panel>{}, 5000);
    check(Panel::ready(), "bring-up completes");
    check(!Panel::failed(), "bring-up does not fail");
    check(!MockBus::commandWhileBusy, "no command is sent while BUSY is high");
    check(!MockBus::refused, "nothing is handed to the bus before the last transfer is off it");

    auto const bringUp = commands();
    if(verbose) {
        for(auto const& c : bringUp) { std::printf("    %s\n", describe(c).c_str()); }
    }

    check(!MockBus::log.empty() && MockReset::releaseAt - MockReset::holdAt >= Panel::ResetPulse,
          "RES# is held low for the reset pulse");
    check(!MockBus::log.empty()
            && MockBus::log.front().at - MockReset::releaseAt >= Panel::ResetToCommand,
          "nothing is sent until the reset-to-command wait is over");

    // The full-waveform bring-up, command by command.
    std::uint8_t const         lastLo = static_cast<std::uint8_t>((Config::gateLines - 1) & 0xFF);
    std::uint8_t const         lastHi = static_cast<std::uint8_t>((Config::gateLines - 1) >> 8);
    std::vector<Command> const fullHead{
      {              Cmd::SwReset,                                              {}},
      {  Cmd::DriverOutputControl,                             {lastLo, lastHi, 0}},
      {        Cmd::DataEntryMode,                                          {0x03}},
      {Cmd::DisplayUpdateControl1,                                    {0x00, 0x80}},
      {       Cmd::BorderWaveform,                                          {0x05}},
      {Cmd::DisplayUpdateControl2,                                          {0xF7}},
      {            Cmd::RamXRange, {0x00, static_cast<std::uint8_t>(rowBytes - 1)}},
      {            Cmd::RamYRange,                          {0, 0, lastLo, lastHi}},
      {          Cmd::RamXCounter,                                             {0}},
      {          Cmd::RamYCounter,                                          {0, 0}},
    };
    bool headOk = bringUp.size() > fullHead.size() + 1;
    for(std::size_t i = 0; headOk && i < fullHead.size(); ++i) {
        headOk = bringUp[i].cmd == fullHead[i].cmd && bringUp[i].params == fullHead[i].params;
    }
    check(headOk, "bring-up: reset, output control, entry mode, full waveform, RAM window");
    check(headOk && bringUp[fullHead.size()].cmd == Cmd::WriteRamBw
            && bringUp[fullHead.size()].params == std::vector<std::uint8_t>(frameBytes, 0xFF),
          "bring-up writes the whole frame, white");
    check(headOk && bringUp[fullHead.size() + 1].cmd == Cmd::MasterActivation,
          "and activates the refresh");

    std::size_t rams = 0;
    for(auto const& c : bringUp) { rams += c.cmd == Cmd::WriteRamBw ? 1U : 0U; }
    if(Config::partial) {
        std::size_t lut = 0;
        for(auto const& c : bringUp) {
            if(c.cmd == Cmd::WriteLut) { lut = c.params.size(); }
        }
        check(lut == 153, "partial: the 153 byte LUT is loaded");
        check(rams == 2, "partial: the frame is refreshed with both waveforms");
        check(!bringUp.empty() && bringUp.back().cmd == Cmd::MasterActivation,
              "partial: bring-up ends on the partial refresh");
    } else {
        check(rams == 1, "full: one refresh at bring-up");
        check(bringUp.size() == fullHead.size() + 2, "full: nothing else is sent");
    }

    // Three pixels drawn as gfx::MonoFrame does (8-row pages, bit 0 the top row), and where they land.
    MiniFrame<Panel> screen{};
    auto const       draw = [&](auto& c) {
        c.clear();
        c.pixel(0, 0);
        c.pixel(width - 1, height - 1);
        c.pixel(5, 9);
    };
    auto const before = MockBus::log.size();
    check(screen.frame(draw), "a frame is drawn once the panel is ready");
    check(!Panel::ready(), "and the panel is busy with it");
    pump(UntilReady<Panel>{}, 5000);
    check(Panel::ready() && !MockBus::commandWhileBusy, "the refresh completes, BUSY respected");

    auto const frame = commands(before);
    if(verbose) {
        for(auto const& c : frame) { std::printf("    %s\n", describe(c).c_str()); }
    }
    std::vector<std::uint8_t> const order{Cmd::RamXRange,
                                          Cmd::RamYRange,
                                          Cmd::RamXCounter,
                                          Cmd::RamYCounter,
                                          Cmd::WriteRamBw,
                                          Cmd::MasterActivation};
    bool                            orderOk = frame.size() == order.size();
    for(std::size_t i = 0; orderOk && i < order.size(); ++i) { orderOk = frame[i].cmd == order[i]; }
    check(orderOk, "a refresh is window, WRITE RAM, activation and nothing else");

    auto const* ram = lastRam(frame);
    check(ram != nullptr && ram->params.size() == frameBytes, "the refresh sends the whole frame");
    if(ram != nullptr && ram->params.size() == frameBytes) {
        check(inkCount(*ram) == 3, "exactly the three drawn pixels are black");
        if(Config::flip) {
            check(inkAt(*ram, rowBytes, 0, height - 1), "flipped: (0, 0) is gate 0, last source");
            check(inkAt(*ram, rowBytes, width - 1, 0),
                  "flipped: the far corner is last gate, source 0");
            check(inkAt(*ram, rowBytes, 5, height - 1 - 9),
                  "flipped: (5, 9) is gate 5, source H-10");
        } else {
            check(inkAt(*ram, rowBytes, width - 1, 0), "(0, 0) is the last gate line, source 0");
            check(inkAt(*ram, rowBytes, 0, height - 1),
                  "the far corner is gate 0, the last source");
            check(inkAt(*ram, rowBytes, width - 1 - 5, 9), "(5, 9) is gate W-6, source 9");
        }
    }

    // The same picture again: nothing is sent, nothing flickers.
    {
        auto const quiet = MockBus::log.size();
        check(screen.frame(draw), "an identical frame is drawn");
        pump(UntilReady<Panel>{}, 1000);
        check(Panel::ready() && MockBus::log.size() == quiet,
              "an identical frame is not sent or refreshed");
    }

    // clean(): the bring-up waveform again, with the current picture.
    {
        auto const mark = MockBus::log.size();
        Panel::clean();
        check(!Panel::ready(), "clean() makes the panel busy");
        pump(UntilReady<Panel>{}, 10000);
        auto const  cleaned = commands(mark);
        auto const* r       = lastRam(cleaned);
        check(Panel::ready() && !cleaned.empty() && cleaned.front().cmd == Cmd::SwReset,
              "clean() starts over with a SW reset");
        check(r != nullptr && r->params.size() == frameBytes && inkCount(*r) == 3,
              "clean() refreshes with the picture that was showing");
        check(without(cleaned, Cmd::WriteRamBw).size() == without(bringUp, Cmd::WriteRamBw).size(),
              "clean() sends the bring-up sequence");
    }

    // BUSY that never drops fails the panel instead of hanging it.
    {
        resetMocks();
        Panel::restart();
        check(Panel::link() == Kvasir::Link::starting, "link: starting after restart()");
        MockBusy::stuck = true;
        Kvasir::Test::Log::reset();
        auto const failuresBefore = Panel::failures();
        pump(Forever<Panel>{}, static_cast<int>(Config::busyTimeout.count()) + 2000);
        check(Panel::failed(), "BUSY stuck high fails the panel after the timeout");
        check(commands().size() <= 1, "and nothing past the first command is sent");
        check(Panel::failures() == failuresBefore + 1, "the failure is counted once");
        check(Kvasir::Test::Log::warnings == 1, "and logged once");
        check(Panel::link() == Kvasir::Link::absent, "link: absent while failed");

        // (a) BUSY lets go: nothing until retryAfter, then a reset and the whole bring-up with
        // the picture that was showing.
        auto const failedAt = FakeClock::now();
        auto const mark     = MockBus::log.size();
        MockBusy::stuck     = false;
        pump(Never<Panel>{}, static_cast<int>(Config::retryAfter.count()) - 5);
        check(Panel::failed() && MockBus::log.size() == mark,
              "a failed panel sends nothing before retryAfter");
        check(Kvasir::Test::Log::warnings == 1, "and logs the failure only once");
        pump(UntilUp<Panel>{}, 10000);
        check(Panel::ready(), "a failed panel comes back by itself, without restart()");
        check(Panel::link() == Kvasir::Link::answering, "link: answering again, by itself");
        check(MockReset::holdAt - failedAt >= Config::retryAfter
                && MockReset::holdAt - failedAt
                     <= Config::retryAfter + std::chrono::milliseconds{2},
              "the bring-up starts retryAfter after the failure, with a reset pulse");
        auto const  back = commands(mark);
        auto const* r    = lastRam(back);
        check(!back.empty() && back.front().cmd == Cmd::SwReset, "the bring-up starts over");
        check(r != nullptr && r->params.size() == frameBytes && inkCount(*r) == 3,
              "and resends the picture that was showing");
        check(!MockBus::commandWhileBusy, "no command while BUSY, across the failure");
        check(Panel::failures() == failuresBefore + 1, "failures() keeps the count");

        // (b) A failed transfer in the middle of a refresh: failed, then up again with the new
        // picture.
        auto const redraw = [&](auto& c) {
            draw(c);
            c.pixel(width / 2, height / 2);
        };
        check(screen.frame(redraw), "a new frame is drawn after the recovery");
        MockBus::failNext = true;
        auto const mark2  = MockBus::log.size();
        pump(Forever<Panel>{}, 1000);
        check(Panel::failed(), "a failed transfer fails the panel");
        check(Panel::failures() == failuresBefore + 2, "the second failure is counted");
        pump(UntilUp<Panel>{}, static_cast<int>(Config::retryAfter.count()) + 10000);
        auto const  again = commands(mark2);
        auto const* r2    = lastRam(again);
        check(Panel::ready(), "and the panel comes back after a failed transfer");
        check(r2 != nullptr && r2->params.size() == frameBytes && inkCount(*r2) == 4,
              "with the frame drawn before the failure");
        check(Kvasir::Test::Log::warnings == 2, "one warning per failure");

        // (c) A panel that keeps failing is tried less and less often (the pause doubles), and
        // the pause starts over once it was back.
        MockBusy::stuck = true;
        Panel::clean();
        pump(UntilFailure<Panel>{failuresBefore + 2},
             static_cast<int>(Config::busyTimeout.count()) + 2000);
        auto const third = FakeClock::now();
        check(Panel::failures() == failuresBefore + 3, "stuck again: a third failure");
        pump(UntilFailure<Panel>{failuresBefore + 3},
             static_cast<int>((Config::retryAfter + Config::busyTimeout).count()) + 2000);
        auto const fourth = FakeClock::now();
        check(Panel::failures() == failuresBefore + 4
                && MockReset::holdAt - third >= Config::retryAfter
                && MockReset::holdAt - third <= Config::retryAfter + std::chrono::milliseconds{2},
              "the first retry after a recovery comes retryAfter later");
        pump(Never<Panel>{}, static_cast<int>(2 * Config::retryAfter.count()) + 100);
        check(MockReset::holdAt - fourth >= 2 * Config::retryAfter
                && MockReset::holdAt - fourth
                     <= 2 * Config::retryAfter + std::chrono::milliseconds{2},
              "the next one twice as late");
        MockBusy::stuck = false;
        pump(UntilUp<Panel>{}, static_cast<int>(4 * Config::retryAfter.count()) + 10000);
        check(Panel::ready(), "and it still comes back once BUSY lets go");
    }
}

struct FullConfig : Kvasir::Display::Ssd1680Defaults {};

struct FlipConfig : Kvasir::Display::Ssd1680Defaults {
    static constexpr bool flip = true;
};

/// A module with fewer source lines, the partial waveform, and chunks that do not divide
/// the gate lines evenly.
struct PartialConfig : Kvasir::Display::Ssd1680Defaults {
    static constexpr int  sourceLines = 120;
    static constexpr int  gateLines   = 250;
    static constexpr bool partial     = true;
    static constexpr int  chunkRows   = 7;
};

template<typename Config>
using Panel = Kvasir::Display::Ssd1680<FakeClock, MockBus, MockReset, MockBusy, Config>;

// gfx::MonoPanel matches structurally (pageRam(), pagesChanged(), ready(), failed(), turn()).
static_assert(requires {
    Panel<FullConfig>::pageRam();
    Panel<FullConfig>::pagesChanged(0U);
});

}   // namespace

int main(int          argc,
         char const** argv) {
    verbose = argc > 1 && std::strcmp(argv[1], "-v") == 0;

    exercise<Panel<FullConfig>>("SSD1680 2.9\"");
    exercise<Panel<FlipConfig>>("SSD1680 2.9\" upside down");
    exercise<Panel<PartialConfig>>("SSD1680 2.13\"");

    return Kvasir::Test::finish();
}
