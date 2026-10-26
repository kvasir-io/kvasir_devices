#pragma once
// The contract between the queued I2C engine (QueueCore.hpp) and a chip's hardware half (chip_rp_common
// I2CQueued.hpp, chip_atsam_common Sercom_I2CQueued.hpp, the host test's fake). A policy that misses a member fails
// here, with this concept's name, instead of deep inside the engine. Every hook is static; the hardware half reads
// and moves the core's request state (`Core::currentRequest_`, `sendIndex_`, `receivedCount_`, `state_`) and ends
// a request with `Core::complete(result)` - member function bodies are a complete-class context, so a policy nested
// in the chip's hardware half (Detail::I2CQueuedHalf) names the half's Core there.
//
//   static constexpr unsigned Instance;
//   using Recovery;          LineRecovery's static API (tick, begin, checkBusStuck, deferSettle, sdaIsHigh, ...)
//   using Request;           QueuedRequestFor<...> (Request.hpp) for the bus's features
//   static constexpr bool Cancellable, Deadlines, PerDeviceClock;
//   static constexpr Bus::QueueFeatures BusFeatures;   the config's queueFeatures (the shape's)
//   using Snapshot;          the block's registers at a timeout (the core's TimeoutSnapshot holds it as `hw`)
//   static void mask(); static void unmask();      the bus interrupt line(s); the core counts the depth and calls them
//                            for the outermost section only (Bus/MaskDepth.hpp) - so a policy never enables its
//                            line inside a masked section (reinit() included); only the core's outermost unlock does
//   static void reinit();    the block back to its initial set-up (reset(); masked)
//   static void start(Request const&);             the request on the wire; sets Core::state_
//   static void finish();    after every completion, before the callback (RP: disable, interrupts off)
//   static void requestStop();                     stop the request on the wire (cancel, deadline)
//   static void timeoutAbort();                    the block let go of a request that timed out
//   static bool busActive(); the master still owns the bus (a failure's STOP on its way out)
//   static auto transferTimeout(std::size_t bytes, Request const&);   a duration
//   static Snapshot snapshot(); static void logTimeout(time_point now);
//   static void logLastTimeout(TimeoutSnapshot<Snapshot> const&);   the one line, registers and progress
//   static void onIsr();     the interrupt, after the core's latency stamp
//   static std::uint32_t waitsExhausted();          the block's handshake waits that ran out (disableWaitsExhausted)
//   static std::uint32_t clockSwitches();           its TimingCache's switches; called on a PerDeviceClock bus only
#include "../../BusTypes.hpp"

#include <concepts>
#include <cstddef>
#include <cstdint>

namespace Kvasir { namespace I2C {

    /// Where the request on the wire is. idle also while no request runs, and from the moment a request completes
    /// (before its callback): an interrupt entry left over from it finds nothing to finish.
    enum class QueueState { idle, sending, receiving };

    /// What a request that timed out looked like, read before the abort: the core's half (the request's progress,
    /// the lines, the interrupt timing) once for every chip, and `hw`, the chip's registers (Hw::Snapshot).
    template<typename HwSnapshot>
    struct TimeoutSnapshot {
        HwSnapshot    hw{};
        std::uint16_t sent{};   ///< bytes sent of toSend
        std::uint16_t toSend{};
        std::uint16_t received{};   ///< bytes received of toReceive
        std::uint16_t toReceive{};
        std::uint8_t  address{};
        std::uint8_t  state{};   ///< 0 idle, 1 sending, 2 receiving (QueueState)
        /// The lines themselves: SCL low with the master stalled is a part holding the clock (or the master
        /// holding it for its interrupt or a command that never came); both high is the block waiting on nothing
        /// the wire shows.
        bool          sdaHigh{};
        bool          sclHigh{};
        std::uint32_t isrEntries{};   ///< interrupt entries since this request started
        std::uint32_t usSinceIsr{};   ///< since the last interrupt entry, of any request
        std::uint32_t usAge{};        ///< since the request started
    };

    /// `Clock` is the core's: logTimeout() takes its time_point.
    template<typename Hw, typename Clock>
    concept QueueHw = requires(typename Hw::Request const&                   r,
                               TimeoutSnapshot<typename Hw::Snapshot> const& s,
                               std::size_t                                   n,
                               typename Clock::time_point                    now) {
        { Hw::instance } -> std::convertible_to<unsigned>;
        typename Hw::Recovery;
        { Hw::cancellable } -> std::convertible_to<bool>;
        { Hw::deadlines } -> std::convertible_to<bool>;
        { Hw::perDeviceClock } -> std::convertible_to<bool>;
        { Hw::busFeatures } -> std::convertible_to<::Kvasir::Bus::QueueFeatures>;
        Hw::mask();
        Hw::unmask();
        Hw::reinit();
        Hw::start(r);
        Hw::finish();
        Hw::requestStop();
        Hw::timeoutAbort();
        { Hw::busActive() } -> std::same_as<bool>;
        Hw::transferTimeout(n, r);
        { Hw::snapshot() } -> std::same_as<typename Hw::Snapshot>;
        Hw::logTimeout(now);
        Hw::logLastTimeout(s);
        Hw::onIsr();
        { Hw::waitsExhausted() } -> std::same_as<std::uint32_t>;
        { Hw::clockSwitches() } -> std::same_as<std::uint32_t>;
    };
}}   // namespace Kvasir::I2C
