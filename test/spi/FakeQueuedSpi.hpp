#pragma once

/// A queued SPI master for host tests: the real QueueCore over a fake policy that hands every byte
/// to the attached part model while its CS is low. A transfer completes at the next tick() unless
/// `stall` holds it back. Every frame is recorded in `Bus::frames`.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <kvasir/Devices/Quantities.hpp>
#include <kvasir/Devices/SPI/QueueCore.hpp>
#include <support/FakeClock.hpp>
#include <vector>

namespace Kvasir::Test::QueuedSpi {

struct Setup {
    std::uint8_t  mode{};
    std::uint32_t hz{1'000'000};
    std::uint32_t usPerBitQ10{Kvasir::SPI::usPerBitQ10(1'000'000)};

    constexpr bool operator==(Setup const&) const = default;
};

struct Frame {
    Setup                     setup{};
    bool                      selected{};
    bool                      wide{};
    std::vector<std::uint8_t> mosi{};
    std::vector<std::uint8_t> miso{};
};

template<typename Tag>
struct Bus {
    struct Hw {
        struct Snapshot {};

        using Setup = QueuedSpi::Setup;

        static constexpr unsigned Instance = 0;

        static void mask() {}

        static void unmask() {}

        static void configure(Setup const& s) { setup = s; }

        static void start(Kvasir::SPI::Transfer const& t,
                          std::uint32_t                g,
                          void (*)(std::uint32_t,
                                   bool)) {
            Frame f{.setup = setup, .selected = partSelected(), .wide = t.wide};
            if(onFrame) { onFrame(f.selected); }
            auto const unit  = t.wide ? 2U : 1U;
            auto const bytes = t.frames * unit;
            // A 16-bit frame goes out MSB first from a native halfword, as the PL022 shifts it: on a
            // little-endian host the bytes in memory are the other way round.
            auto const wireIndex
              = [&](std::uint32_t i) { return t.wide ? (i & ~1U) | (1U - (i & 1U)) : i; };
            for(std::uint32_t i = 0; i < bytes; ++i) {
                auto const   at   = t.txIncrement ? wireIndex(i) : wireIndex(i) % unit;
                auto const   mosi = std::to_integer<std::uint8_t>(t.tx[at]);
                std::uint8_t miso = floating;
                if(f.selected && exchange) { miso = exchange(mosi); }
                f.mosi.push_back(mosi);
                f.miso.push_back(miso);
                if(t.rxIncrement) {
                    t.rx[wireIndex(i)] = std::byte{miso};
                } else {
                    t.rx[wireIndex(i) % unit] = std::byte{miso};
                }
            }
            frames.push_back(std::move(f));
            pendingGen = g;
            pending    = true;
        }

        static void abort() {
            pending = false;
            ++aborts;
        }

        static void reinit() { ++reinits; }

        static Snapshot snapshot() { return {}; }

        static void log(Snapshot const&) {}

        static inline Setup setup{};
    };

    using Core = Kvasir::SPI::QueueCore<Hw, FakeClock, 8, 16>;

    static inline std::vector<Frame>                        frames{};
    static inline std::function<std::uint8_t(std::uint8_t)> exchange{};
    static inline std::function<bool()>                     partSelected{[] { return false; }};
    /// Called as each frame starts, with whether the part is selected.
    static inline std::function<void(bool)> onFrame{};
    static inline std::uint8_t              floating{0xFF};
    static inline bool                      pending{};
    static inline std::uint32_t             pendingGen{};
    static inline bool                      stall{};
    /// handler() completes the transfer on the wire, for drivers that spin on it (SpiDcsBus).
    static inline bool autoComplete{};
    static inline bool overrunNext{};
    static inline int  aborts{};
    static inline int  reinits{};

    /// The transfer that is out completes now, as the RX DMA would.
    static void tick() {
        if(pending && !stall) {
            pending        = false;
            bool const ovr = overrunNext;
            overrunNext    = false;
            Core::complete(pendingGen, ovr);
        }
    }

    static void reset() {
        Core::reset();
        frames.clear();
        exchange     = {};
        partSelected = [] { return false; };
        onFrame      = {};
        floating     = 0xFF;
        pending      = false;
        stall        = false;
        autoComplete = false;
        overrunNext  = false;
        aborts = reinits = 0;
    }

    using Request = typename Core::RequestT;
    using Setup   = QueuedSpi::Setup;

    static constexpr Setup setup(Kvasir::SPI::ClockMode mode,
                                 Kvasir::Units::Hertz   maxClock) {
        auto const hz = maxClock.numerical_value_in(Kvasir::Units::si::hertz);
        return Setup{static_cast<std::uint8_t>(mode), hz, Kvasir::SPI::usPerBitQ10(hz)};
    }

    static bool submit(Request const& r) { return Core::submit(r); }

    static void releaseHold(Kvasir::SPI::Lines const& l) { Core::releaseHold(l); }

    static void handler() {
        if(autoComplete) { tick(); }
        Core::handler();
    }
};

}   // namespace Kvasir::Test::QueuedSpi
