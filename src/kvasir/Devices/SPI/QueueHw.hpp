#pragma once
// The contract between the queued SPI engine (QueueCore.hpp) and a chip's hardware half (chip_rp_common
// SPIQueued.hpp, chip_atsam_common Sercom_SPIQueued.hpp, the test fakes). A policy that misses a member fails
// here, with this concept's name, instead of deep inside the engine.
//
//   using Setup;   mode, dividers, frame size, `hz`, `usPerBitQ10`; equality comparable
//   static constexpr unsigned Instance;
//   static void mask(); static void unmask();    masks every interrupt that may submit(); called for the outermost
//                  section only (Bus/MaskDepth.hpp), so a policy never enables an interrupt inside a masked section
//   static void configure(Setup const&);         only with nothing on the wire
//   static void start(Transfer const&, std::uint32_t gen, void (*done)(std::uint32_t, bool));
//                  both DMA directions; `done(gen, overrun)` from the RX completion
//   static void abort();                         stop both channels, wait (bounded) for the
//                  shifter to go idle, empty the RX FIFO (PL022: no flush but a reset, TRM 2.3.1)
//   static void reinit();
//   using Snapshot; static Snapshot snapshot(); static void log(Snapshot const&);
// and optionally (QueueCore asks for them by `requires`, so they are not part of the concept):
//   static void poll();                          no completion interrupt: called masked
//   static constexpr bool SupportsWide;          false: no 16-bit frames
//   static constexpr std::uint32_t MaxFrames;    the DMA's count limit
#include <concepts>
#include <cstddef>
#include <cstdint>

namespace Kvasir { namespace SPI {
    struct Transfer;

    template<typename Hw>
    concept QueueHw = requires(typename Hw::Setup const&    s,
                               typename Hw::Snapshot const& n,
                               Transfer const&              t,
                               std::uint32_t                gen,
                               void (*done)(std::uint32_t, bool)) {
        { Hw::instance } -> std::convertible_to<unsigned>;
        requires std::equality_comparable<typename Hw::Setup>;
        { s.hz } -> std::convertible_to<std::uint32_t>;
        { s.usPerBitQ10 } -> std::convertible_to<std::uint32_t>;
        Hw::mask();
        Hw::unmask();
        Hw::configure(s);
        Hw::start(t, gen, done);
        Hw::abort();
        Hw::reinit();
        { Hw::snapshot() } -> std::same_as<typename Hw::Snapshot>;
        Hw::log(n);
    };
}}   // namespace Kvasir::SPI
