/// What must not compile, built by ctest and judged by its error (test/display/CMakeLists.txt): a queued display bus
/// that cannot hold what DcsPanel hands it in one turn.
///   BURST_FAIL_SLOTS_SHORT  an ST7789 panel (INVON: COLMOD, MADCTL, INVON, SLPOUT = 4 commands) on a SpiDcsBus with
///                           commandSlots 2 (DcsPanel's busHoldsFormatBurst)
///   BURST_FAIL_QUEUE_SHORT  a SpiDcsBus with the default 6 command slots over a master whose queue holds 4
///                           (SpiDcsBus's masterQueueHoldsCommandSlots)
#include <kvasir/Devices/Display/SpiDcsBus.hpp>
#include <kvasir/Devices/Display/St7789.hpp>
#include <spi/Harness.hpp>
#include <support/FakeClock.hpp>

namespace {

namespace D     = Kvasir::Display;
namespace Spi   = Kvasir::Test::Spi;
using FakeClock = Kvasir::Test::FakeClock;
using Dc        = Spi::Pin<4>;

struct Tag {};

using Bus = Kvasir::Test::QueuedSpi::Bus<Tag>;

/// The same fake with a queue of 4 requests.
struct ShortTag {};

using ShortBase = Kvasir::Test::QueuedSpi::Bus<ShortTag>;

struct ShortBus {
    using Hw      = ShortBase::Hw;
    using Core    = Kvasir::SPI::QueueCore<Hw, FakeClock, 4, 16>;
    using Request = Core::RequestT;
    using Setup   = Kvasir::Test::QueuedSpi::Setup;

    static constexpr Setup setup(Kvasir::Hal::ClockMode mode,
                                 Kvasir::Units::Hertz   maxClock) {
        return ShortBase::setup(mode, maxClock);
    }

    static bool submit(Request const& r) { return Core::submit(r); }

    static void releaseHold(Kvasir::SPI::Lines const& l) { Core::releaseHold(l); }

    static void turn() { Core::turn(); }
};

struct PanelConfig : D::DcsDefaults {
    static constexpr int  Width  = 240;
    static constexpr int  Height = 135;
    static constexpr bool invert = true;
};

template<typename Lcd>
using Panel = Kvasir::St7789::Panel<FakeClock, Lcd, Kvasir::NoReset, PanelConfig>;

struct TwoSlots : D::SpiDcsBusDefaults {
    static constexpr std::size_t commandSlots = 2;
};

struct FourSlots : D::SpiDcsBusDefaults {
    static constexpr std::size_t commandSlots = 4;
};

// The controls: the defaults over the full queue, and a bus as deep as the short queue (no panel on it) compile.
using Lcd = D::SpiDcsBus<Bus, FakeClock, Spi::Cs, Dc>;
static_assert(sizeof(Panel<Lcd>) > 0 && Panel<Lcd>::FormatBurst == 4);
static_assert(sizeof(D::SpiDcsBus<ShortBus,
                                  FakeClock,
                                  Spi::Cs,
                                  Dc,
                                  FourSlots>)
              > 0);

#if defined(BURST_FAIL_SLOTS_SHORT)
static_assert(sizeof(Panel<D::SpiDcsBus<Bus,
                                        FakeClock,
                                        Spi::Cs,
                                        Dc,
                                        TwoSlots>>)
              > 0);
#endif

#if defined(BURST_FAIL_QUEUE_SHORT)
static_assert(sizeof(D::SpiDcsBus<ShortBus,
                                  FakeClock,
                                  Spi::Cs,
                                  Dc>)
              > 0);
#endif

}   // namespace
