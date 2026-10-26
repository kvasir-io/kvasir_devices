#pragma once
// What a Device's Config and the application name, beside the description: the config helpers (At, DefaultConfig,
// AtClock), the engine's policy defaults (EngineDefaults, BackoffReset), the gate vocabulary (NoGate, WhileOff,
// detail::BridgedGate - the bridge's, Bridge.hpp), Ticket and SampleGaps. No engine.
#include "../Quantities.hpp"

#include <chrono>
#include <cstdint>

namespace Kvasir::Engine {

/// A config that only sets the address: `Device<I2c, Clock, Chips::Bme280, At<0x77>>`.
template<std::uint8_t A>
struct At {
    static constexpr std::uint8_t address = A;
};

/// No config: the chip's default address.
struct DefaultConfig {};

/// Another config with this device's bus clock added, for a bus that runs each device at its
/// own clock (I2CConfig::perDeviceClock): `Device<I2c, Clock, Chips::Tlv493d, AtClock<100'000>>`.
template<std::uint32_t Hz, typename Base = DefaultConfig>
struct AtClock : Base {
    static constexpr Units::Hertz busClock = Units::hertz(Hz);
};

/// What starts the fault pause over at EngineDefaults::faultBackoff (EngineDefaults::faultBackoffResetOn).
enum class BackoffReset : std::uint8_t {
    scriptCompleted,   ///< a script (bring-up, write, read-back, read) that goes through
    anyAck,            ///< any acknowledged transaction, also one inside a script that fails later
};

/// The engine's retry and failure policy. A Device's Config may redeclare any of them:
///     struct Config { static constexpr std::uint8_t maxRetries = 3; };
struct EngineDefaults {
    /// How often a decode or a check step may ask for a retry before the sample is rejected,
    /// and how often a read-back mismatch rewrites a register before it is left alone.
    static constexpr std::uint8_t maxRetries = 8;
    /// How long a write, a read-back or a bring-up waits after a failed transaction before it
    /// is tried again. What was owed stays owed; this only keeps a broken bus from being
    /// hammered every loop turn.
    static constexpr auto faultBackoff = std::chrono::milliseconds{5};
    /// While the faults go on, each pause is twice the last, up to this; what starts it over at faultBackoff is
    /// faultBackoffResetOn.
    static constexpr auto faultBackoffLongest = std::chrono::seconds{1};
    /// What starts the doubling over: the first script that completes (a part that ACKs the first step of its
    /// bring-up and NAKs a later one keeps doubling), or any ACK (such a part is retried every faultBackoff).
    static constexpr BackoffReset faultBackoffResetOn = BackoffReset::scriptCompleted;
    /// Failed transactions in a row, past the bring-up, after which the chip is configured
    /// again from the start: a part that keeps faulting may have lost its registers.
    static constexpr std::uint8_t faultsBeforeReinit = 5;
    /// A request whose callback never came. The bus guarantees exactly one callback per
    /// submitted request, so this never fires on a correct bus; it is the net under that
    /// guarantee, and when it does fire the request is treated as failed.
    static constexpr auto inFlightTimeout = std::chrono::seconds{2};
    /// setup() said "not the chip this description is for": how long to wait before the
    /// bring-up is tried again. Nothing else is sent to the part meanwhile -- a config write
    /// meant for one chip is not what an unknown part at the same address should get. 0 runs
    /// the description anyway, with identified() false, for a board that must talk to a part
    /// whose id it does not know.
    static constexpr auto unidentifiedRetry = std::chrono::seconds{1};
};

/// No gate: the device may talk whenever the engine wants it to, which is every device that
/// is not behind an I2C switch. See Mux.hpp for the one that is.
///
/// A gate is an object the Device holds, not a set of static functions, so a gate that needs
/// to know *which* switch it is on can keep a pointer to it and the switch need not be a
/// global. This one is empty and `[[no_unique_address]]`, so an ungated device pays nothing.
struct NoGate {
    static constexpr bool Gated = false;
    static constexpr bool Front = true;   // not behind a switch (Mux.hpp, MuxFrontGate)

    bool claim() { return true; }

    void release() {}
};

/// What the parts behind a bridge go through while it is not active (Bridge.hpp), which is what
/// the engine owes them when it is active again.
enum class WhileOff : std::uint8_t {
    unpowered,      ///< they lose their supply: brought up from the start, as after power-on
    disconnected,   ///< they stay powered and keep their registers: the engine carries on
};

namespace detail {
    /// A gate that can say "not now, and for a reason": the part is behind a bridge
    /// (Bridge.hpp, BridgeGate). Besides claim() and release() it has `offline()`,
    /// `generation()` -- which steps every time the bridge becomes active -- and names the
    /// bridge, whose `WhileOff` the engine reads.
    template<typename G>
    concept BridgedGate = requires { requires G::Bridged; };
}   // namespace detail

/// What request<G>() hands back, for answer<G>() to be asked about. A value-initialised one
/// is `none()`: false, and answer<G>() says `pending` for it -- so a caller can keep one
/// Ticket and ask "did I request anything" without a flag beside it.
struct Ticket {
    std::uint32_t id{};

    [[nodiscard]] static constexpr Ticket none() { return {}; }

    [[nodiscard]] constexpr explicit operator bool() const { return id != 0; }

    [[nodiscard]] constexpr bool operator==(Ticket const&) const = default;
};

/// The gaps between one read group's samples over an interval (Device::takeGaps<G>()): the
/// longest, and how many were two periods or more -- a tick the part did not get.
template<typename Duration>
struct SampleGaps {
    Duration      longest{};
    std::uint32_t late{};
};

}   // namespace Kvasir::Engine
