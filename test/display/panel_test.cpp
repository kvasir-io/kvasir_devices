/// The DCS panel driver's bring-up for every chip policy against a mock bus: the command stream,
/// in order, with its waits. Not whether the vendor init table suits a given module.
///
///   panel_test [-v]     -v prints every transaction
// clang-format off
#include <support/LogStubs.hpp>
// clang-format on
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <kvasir/Devices/Display/Co5300.hpp>
#include <kvasir/Devices/Display/DcsPanel.hpp>
#include <kvasir/Devices/Display/St7789.hpp>
#include <kvasir/Devices/Display/St77916.hpp>
#include <span>
#include <string>
#include <string_view>
#include <support/FakeClock.hpp>
#include <vector>

namespace {

bool verbose = false;

/// A clock the test winds by hand.
using FakeClock = Kvasir::Test::FakeClock;

struct Transaction {
    enum class Kind { command, pixels, fill, read, resetHold, resetRelease } kind{};
    std::uint8_t              cmd{};
    std::vector<std::uint8_t> params{};
    std::size_t               count{};   // pixels / fill / dummy bytes
    std::uint8_t              instruction{};
    FakeClock::time_point     at{};

    auto time_since_epoch_at() const { return at.time_since_epoch(); }
};

/// The bus, recording; a pixel transfer stays busy for a few handler() calls (clearWait).
struct MockBus {
    static inline std::vector<Transaction> log{};
    static inline int                      busy{};
    static inline bool                     fail{};
    static inline std::array<std::byte, 3> idBytes{std::byte{0x33},
                                                   std::byte{0x10},
                                                   std::byte{0x00}};

    static void reset() {
        log.clear();
        busy = 0;
        fail = false;
    }

    static bool writeCommand(std::uint8_t               cmd,
                             std::span<std::byte const> params) {
        Transaction t{};
        t.kind = Transaction::Kind::command;
        t.cmd  = cmd;
        t.at   = FakeClock::now();
        for(auto const b : params) { t.params.push_back(static_cast<std::uint8_t>(b)); }
        log.push_back(t);
        return !fail;
    }

    static void writePixels(std::span<std::byte const> data,
                            bool = false) {
        log.push_back({Transaction::Kind::pixels, 0, {}, data.size() / 2, 0, FakeClock::now()});
        busy = 3;
    }

    static void fillPixels(std::uint8_t hi,
                           std::uint8_t lo,
                           std::size_t  count,
                           bool = false) {
        log.push_back({
          Transaction::Kind::fill,
          0,
          {hi, lo},
          count,
          0,
          FakeClock::now()
        });
        busy = 3;
    }

    static bool readRegister(std::uint8_t         instruction,
                             std::uint8_t         cmd,
                             std::size_t          dummyBytes,
                             std::span<std::byte> out) {
        log.push_back(
          {Transaction::Kind::read, cmd, {}, dummyBytes, instruction, FakeClock::now()});
        for(std::size_t i = 0; i < out.size() && i < idBytes.size(); ++i) { out[i] = idBytes[i]; }
        return true;
    }

    static bool idle() { return busy == 0; }

    static bool failed() { return false; }

    static void clearError() {}

    static void handler() {
        if(busy > 0) { --busy; }
    }
};

struct MockReset {
    static void hold() {
        MockBus::log.push_back({Transaction::Kind::resetHold, 0, {}, 0, 0, FakeClock::now()});
    }

    static void release() {
        MockBus::log.push_back({Transaction::Kind::resetRelease, 0, {}, 0, 0, FakeClock::now()});
    }
};

static_assert(Kvasir::Display::Bus<MockBus>);
static_assert(Kvasir::Display::ResetLine<MockReset>);

std::string describe(Transaction const& t) {
    char       buf[64];
    auto const ms
      = std::chrono::duration_cast<std::chrono::milliseconds>(t.at.time_since_epoch()).count();
    std::string s = "t=" + std::to_string(ms) + "ms ";
    switch(t.kind) {
    case Transaction::Kind::resetHold:    return s + "RESX low";
    case Transaction::Kind::resetRelease: return s + "RESX high";
    case Transaction::Kind::pixels:       return s + "pixels x" + std::to_string(t.count);
    case Transaction::Kind::fill:
        std::snprintf(buf, sizeof buf, "fill %02X%02X x%zu", t.params[0], t.params[1], t.count);
        return s + buf;
    case Transaction::Kind::read:
        std::snprintf(buf,
                      sizeof buf,
                      "read %02Xh cmd %02Xh dummy %zu",
                      t.instruction,
                      t.cmd,
                      t.count);
        return s + buf;
    case Transaction::Kind::command:
        std::snprintf(buf, sizeof buf, "cmd %02Xh", t.cmd);
        s += buf;
        for(auto const p : t.params) {
            std::snprintf(buf, sizeof buf, " %02X", p);
            s += buf;
        }
        return s;
    }
    return s;
}

int failures = 0;

void check(bool             ok,
           std::string_view what) {
    if(!ok) {
        std::printf("  FAIL: %.*s\n", static_cast<int>(what.size()), what.data());
        ++failures;
    } else if(verbose) {
        std::printf("  ok:   %.*s\n", static_cast<int>(what.size()), what.data());
    }
}

/// Index of the first command `cmd` in the log, or -1.
int findCommand(std::uint8_t cmd,
                std::size_t  from = 0) {
    for(std::size_t i = from; i < MockBus::log.size(); ++i) {
        auto const& t = MockBus::log[i];
        if(t.kind == Transaction::Kind::command && t.cmd == cmd) { return static_cast<int>(i); }
    }
    return -1;
}

int findKind(Transaction::Kind kind,
             std::size_t       from = 0) {
    for(std::size_t i = from; i < MockBus::log.size(); ++i) {
        if(MockBus::log[i].kind == kind) { return static_cast<int>(i); }
    }
    return -1;
}

long msBetween(int a,
               int b) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
             MockBus::log[static_cast<std::size_t>(b)].at
             - MockBus::log[static_cast<std::size_t>(a)].at)
      .count();
}

template<typename Panel>
void exercise(std::string_view name,
              int              width,
              int              height,
              int              colOff,
              int              rowOff) {
    using Dcs  = Kvasir::Display::Dcs::Cmd;
    using Chip = typename Panel::Chip;

    std::printf("%.*s (%d x %d at +%d,+%d)\n",
                static_cast<int>(name.size()),
                name.data(),
                width,
                height,
                colOff,
                rowOff);
    MockBus::reset();
    FakeClock::current = {};
    Panel::restart();

    int turns = 0;
    while(!Panel::initialised() && !Panel::failed() && turns < 5000) {
        Panel::handler();
        FakeClock::advance(std::chrono::milliseconds{1});
        ++turns;
    }
    check(Panel::initialised(), "bring-up completes");
    check(!Panel::failed(), "bring-up does not fail");

    if(verbose) {
        for(auto const& t : MockBus::log) { std::printf("    %s\n", describe(t).c_str()); }
    }

    // Reset pulse and the wait before the first command.
    auto const hold    = findKind(Transaction::Kind::resetHold);
    auto const release = findKind(Transaction::Kind::resetRelease);
    check(hold == 0, "RESX is driven low first");
    check(release > hold, "RESX is released after the pulse");
    check(msBetween(hold, release) >= static_cast<long>(Chip::ResetPulseMs),
          "reset pulse is long enough");
    auto const firstCmd = findKind(Transaction::Kind::command);
    check(firstCmd > release, "no command before RESX is released");
    check(msBetween(release, firstCmd) >= static_cast<long>(Chip::ResetToCommandMs),
          "reset-to-command wait honoured");

    // The chip's script went out first, verbatim.
    std::size_t                pos = 0;
    Kvasir::Display::Dcs::Step step{};
    auto                       idx      = static_cast<std::size_t>(firstCmd);
    bool                       scriptOk = true;
    std::size_t                steps    = 0;
    while(Chip::Init.next(pos, step)) {
        auto const& t = MockBus::log[idx];
        if(t.kind != Transaction::Kind::command || t.cmd != step.cmd
           || t.params.size() != step.params.size()
           || !std::equal(t.params.begin(), t.params.end(), step.params.begin()))
        {
            scriptOk = false;
            break;
        }
        if(step.delayMs != 0 && idx + 1 < MockBus::log.size()) {
            if(msBetween(static_cast<int>(idx), static_cast<int>(idx) + 1) < step.delayMs) {
                scriptOk = false;
            }
        }
        ++idx;
        ++steps;
    }
    check(scriptOk,
          "init script sent verbatim with its delays (" + std::to_string(steps) + " steps)");

    // Then the format block, sleep out, its wait, the clear, display on.
    auto const colmod = findCommand(Dcs::Colmod, idx);
    auto const madctl = findCommand(Dcs::Madctl, idx);
    auto const slpout = findCommand(Dcs::Slpout, idx);
    auto const fill   = findKind(Transaction::Kind::fill);
    auto const dispon = findCommand(Dcs::Dispon);
    check(colmod > 0
            && MockBus::log[static_cast<std::size_t>(colmod)].params
                 == std::vector<std::uint8_t>{Chip::Colmod565},
          "COLMOD carries the chip's 16 bit code");
    check(madctl > colmod, "MADCTL after COLMOD");
    check(slpout > madctl, "SLPOUT after the format block");
    check(fill > slpout, "clear after SLPOUT");
    check(msBetween(slpout, fill) >= static_cast<long>(Chip::SleepOutMs), "SLPOUT wait honoured");
    check(dispon > fill, "DISPON after the clear");
    check(findCommand(Dcs::Slpout) == slpout, "exactly one SLPOUT, from the driver");
    if(Chip::Brightness) {
        check(findCommand(Dcs::Wrctrld, idx) > 0 && findCommand(Dcs::Wrdisbv, idx) > 0,
              "brightness enabled and set");
    } else {
        check(findCommand(Dcs::Wrdisbv) < 0, "no brightness command on a chip without it");
    }

    // The clear covers the whole active area, at the module's offset.
    {
        auto const  caset = findCommand(Dcs::Caset, static_cast<std::size_t>(slpout));
        auto const  raset = findCommand(Dcs::Raset, static_cast<std::size_t>(slpout));
        auto const& c     = MockBus::log[static_cast<std::size_t>(caset)].params;
        auto const& r     = MockBus::log[static_cast<std::size_t>(raset)].params;
        auto const  x0    = (c[0] << 8) | c[1];
        auto const  x1    = (c[2] << 8) | c[3];
        auto const  y0    = (r[0] << 8) | r[1];
        auto const  y1    = (r[2] << 8) | r[3];
        check(x0 == colOff && x1 == colOff + width - 1,
              "clear CASET spans the active columns at the offset");
        check(y0 == rowOff && y1 == rowOff + height - 1,
              "clear RASET spans the active rows at the offset");
        check(MockBus::log[static_cast<std::size_t>(fill)].count
                == static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
              "clear fills width x height pixels");
        check(MockBus::log[static_cast<std::size_t>(fill)].params
                == std::vector<std::uint8_t>{0, 0},
              "clear is black");
    }

    // A strip blit lands where a StripRenderer would put it.
    {
        auto const                           before = MockBus::log.size();
        std::vector<Kvasir::Display::Rgb565> px(static_cast<std::size_t>(width) * 48);
        while(!Panel::ready()) { Panel::handler(); }
        Panel::blit(Kvasir::Display::Rect{0, 48, width, 48},
                    std::span<Kvasir::Display::Rgb565 const>{px});
        auto const  caset = findCommand(Dcs::Caset, before);
        auto const  raset = findCommand(Dcs::Raset, before);
        auto const& r     = MockBus::log[static_cast<std::size_t>(raset)].params;
        check(caset >= 0 && raset > caset, "blit sets the window");
        check(((r[0] << 8) | r[1]) == rowOff + 48 && ((r[2] << 8) | r[3]) == rowOff + 95,
              "blit RASET is rows 48..95 at the offset");
        auto const pixels = findKind(Transaction::Kind::pixels, before);
        check(pixels > raset && MockBus::log[static_cast<std::size_t>(pixels)].count == px.size(),
              "blit streams exactly the strip's pixels after the window");
        check(!Panel::ready(), "the bus is busy after the blit");
        for(int i = 0; i < 5; ++i) { Panel::handler(); }
        check(Panel::ready(), "and idle again once the transfer completes");
    }

    // A register read uses the chip's instruction and dummy count.
    {
        auto const before = MockBus::log.size();
        auto const id     = Panel::readId();
        auto const rd     = findKind(Transaction::Kind::read, before);
        check(rd >= 0
                && MockBus::log[static_cast<std::size_t>(rd)].instruction == Chip::ReadInstruction
                && MockBus::log[static_cast<std::size_t>(rd)].count == Chip::ReadDummyBytes,
              "readId uses the chip's read instruction and dummy byte count");
        check(id.has_value() && id->id1 == 0x33, "readId returns the bus's bytes");
    }

    // A failing command fails the panel rather than being ignored.
    {
        MockBus::fail = true;
        Panel::displayOn(false);
        check(Panel::failed(), "a bus timeout fails the panel");
        MockBus::fail = false;
        // ... and brought up again RetryAfterMs later
        auto const failuresBefore = Panel::failures();
        int        retryTurns     = 0;
        while(!Panel::initialised() && retryTurns < 5000) {
            FakeClock::advance(std::chrono::milliseconds{1});
            Panel::handler();
            ++retryTurns;
        }
        check(Panel::initialised(), "the panel is up again after the failure");
        check(retryTurns >= 1000, "after RetryAfterMs");
        check(Panel::failures() == failuresBefore + 1, "one failure counted");
        check(Kvasir::Test::Log::warnings >= 1, "and logged");
    }
}

struct Co5300Config : Kvasir::Display::DcsDefaults {
    static constexpr int          Width        = 466;
    static constexpr int          Height       = 466;
    static constexpr int          ColumnOffset = 6;
    static constexpr int          RowOffset    = 0;
    static constexpr std::uint8_t Brightness   = 0xA0;
};

struct St77916Config : Kvasir::Display::DcsDefaults {
    static constexpr int Width  = 360;
    static constexpr int Height = 360;
};

struct St77922Config : Kvasir::Display::DcsDefaults {
    static constexpr int Width  = 532;
    static constexpr int Height = 300;
};

struct St7789Config : Kvasir::Display::DcsDefaults {
    static constexpr int Width  = 240;
    static constexpr int Height = 135;
};

/// RESX on RUN (Kvasir::NoReset): no pulse, no wait; SWRESET first, at once (ST7789V.md:5588).
void noResetLine() {
    using Panel = Kvasir::St7789::Panel<FakeClock, MockBus, Kvasir::NoReset, St7789Config>;
    using Chip  = Panel::Chip;
    std::printf("ST7789V with no reset line\n");
    MockBus::reset();
    FakeClock::current = {};
    Panel::restart();
    int turns = 0;
    while(!Panel::initialised() && !Panel::failed() && turns < 5000) {
        Panel::handler();
        FakeClock::advance(std::chrono::milliseconds{1});
        ++turns;
    }
    check(Panel::initialised(), "bring-up completes without a reset line");
    check(findKind(Transaction::Kind::resetHold) < 0
            && findKind(Transaction::Kind::resetRelease) < 0,
          "no pulse on a line that does not exist");
    auto const first = findKind(Transaction::Kind::command);
    check(first == 0, "the first transaction is a command");
    check(first == 0 && MockBus::log[0].cmd == Kvasir::Display::Dcs::Cmd::Swreset,
          "and it is the script's SWRESET");
    auto const atMs
      = std::chrono::duration_cast<std::chrono::milliseconds>(MockBus::log[0].time_since_epoch_at())
          .count();
    check(atMs < static_cast<long>(Chip::ResetToCommandMs),
          "sent without the reset-to-command wait");
}

}   // namespace

int main(int          argc,
         char const** argv) {
    verbose = argc > 1 && std::strcmp(argv[1], "-v") == 0;

    exercise<Kvasir::Co5300::Panel<FakeClock, MockBus, MockReset, Co5300Config>>("CO5300",
                                                                                 466,
                                                                                 466,
                                                                                 6,
                                                                                 0);
    exercise<Kvasir::St77916::Panel<FakeClock, MockBus, MockReset, St77916Config>>("ST77916",
                                                                                   360,
                                                                                   360,
                                                                                   0,
                                                                                   0);
    exercise<Kvasir::St77922::Panel<FakeClock, MockBus, MockReset, St77922Config>>("ST77922",
                                                                                   532,
                                                                                   300,
                                                                                   0,
                                                                                   0);
    noResetLine();

    std::printf("%s\n", failures == 0 ? "panel test passed" : "PANEL TEST FAILED");
    return failures == 0 ? 0 : 1;
}
