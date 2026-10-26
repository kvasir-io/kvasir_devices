#pragma once

#include "../ResetLine.hpp"
#include "DeviceConfig.hpp"
#include "DeviceCore.hpp"
#include "EngineFeatures.hpp"
#include "Presence.hpp"
#include "kvasir/Util/Config.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

/// One driver for any I2C device that is a description: what it is (address, register
/// width), how it is brought up (a Step script, an optional setup() over what the script
/// read), what is read from it cyclically (ReadGroups: a Step script at a period, decoded
/// into a Sample) and what is written to it when the application changes it (WriteGroups:
/// a Value encoded into one transaction -- or a fixed sequence of them -- per item). The
/// engine below owns the state machine: the wait after each step, the pending handshake, the
/// register prefix, the period, and the retry and failure policy -- including whether the
/// device is there at all (Presence.hpp: parked after three NAKs, probed at 1 s .. 30 s)
/// and the one safety net under it, a request the bus never answered. One place decides
/// what a failed transaction means, and nothing above or below it decides differently.
///
///     using Sensor = Kvasir::I2C::Device<I2c1, Clock, Chips::Sht3x>;
///     ...
///     Sensor sht{};                                       // a local in main(), or a member
///     sht.turn();                                         // once per loop turn
///     if(sht.valid()) { auto const& s = sht.latest(); }   // the one read group's Sample
///
/// A device keeps all of its state in members and the bus callback captures `this` when a
/// transfer is submitted, so nothing has to find it by name: it belongs wherever the
/// application keeps its objects. Static storage is what an interrupt that must reach the
/// device or a second core asks for -- not the engine.
///
/// What a write group may add beyond Value/Bytes/encode:
///   Items         -- one dirty bit and one Value each: set<W>(i, v)
///   Initial       -- written after every bring-up unless the application set one first
///   Period        -- rewritten cyclically: the items set so far (all of them with Initial);
///                    one never set is left at the chip's own value
///   Transient     -- a one-shot command, not a state: NOT re-sent after a reset. Everything
///                    else the application has written is, because the chip came back at its
///                    defaults (an EEPROM page write and setting a clock are the exceptions).
///   encode() may return std::array<Step, N> instead of a Step: the item is N transactions
///                    run back to back, each honouring its own delay, and only when the
///                    last one is done does the item count as written. A failure anywhere
///                    re-dirties the item and the sequence starts again from its first step.
///                    Not compatible with the read-back verify below. (An SSD1306 page: the
///                    window command, then the RAM bytes behind it.)
///   VerifyDelay   -- read the register back this long after writing it
///   VerifyInterval-- and look again this often
///   verify(Kvasir::Bytes written, Kvasir::Bytes readBack) -- what counts as a
///                    match, when bytewise equality does not (a self-clearing bit, a status
///                    bit sharing the byte). Spell the type out: the group's own Bytes member
///                    shadows it. A mismatch rewrites the item, at most maxRetries times.
///
/// What a cyclic read group may add beyond Period/Steps/Sample/decode:
///   prepare()     -- may return a std::size_t instead of void: how many bytes this run
///                    reads, so one description serves a four-byte and a sixty-four-byte
///                    read. Such a group has exactly one read step.
///   Step::readCounted / receiveCounted -- a read whose length an earlier read of the same
///                    run put in the buffer (one or two bytes, big-endian), clamped to the
///                    step's most: a GNSS receiver's "bytes available", then that many of
///                    its stream. A count of 0 skips the transaction; decode sees the buffer
///                    up to the counted read's offset + count.
///   periodNow(Sample const&) -- the wait after this answer, instead of the constant Period
///                    (which stays as the compile-time "this group is cyclic" and the first
///                    deadline). A touch controller polled every 50 ms while a finger is
///                    down and every 100 ms when the glass is idle. 0 parks the group until
///                    request<G>() asks for it.
///   decode(Bytes, Sample const& previous) -- for a chip whose frames only mean something
///                    next to the last one. With Outcome::unchanged(), which keeps the
///                    sample, does not step seq and is counted by unchanged<G>(), a
///                    controller's "no new data" bit and a repeated "no finger" stop waking
///                    a reader that watches seq. A group takes the chip State or the
///                    previous Sample, not both.
///   Timestamped   -- keep stamp<G>(), the Clock::time_point the chip's answer arrived on the
///                    bus, from the interrupt that completed it rather than the loop turn
///                    that noticed. A group with the decode above keeps it anyway, together
///                    with unchanged<G>(); every other read group carries neither and pays
///                    nothing for them.
///
///   ConfirmChange -- `static constexpr std::uint8_t ConfirmChange = N;` (or the Config's, for
///                    the groups in its `using Confirm = List<...>;`): a changed reading is
///                    decoded only once a second read agrees, or after N that did not. For
///                    level registers (an expander's inputs), where a bad read is a burst of
///                    false edges. The re-reads are the run's retries: N <= maxRetries.
///
/// What a description may add for a part that NAKs while it wakes:
///   WakeRetries   -- `static constexpr std::uint8_t WakeRetries = N;`: a NAKed transaction is
///                    put on the wire again up to N times, WakeRetryDelay apart (default 1 ms,
///                    the gate let go meanwhile), before it counts as a NAK -- towards
///                    errors(), presence and the failure policy. The retries count nowhere
///                    but wakeRetryCount(). A CY8CMBR3xxx NAKs until it has woken from its
///                    sleep; a description without WakeRetries pays nothing for it.
///
/// The chip descriptions are in chips/.
namespace Kvasir::Engine {

/// The driver. `Chip` is the description (chips/); `Config` may set `address`, the presence
/// knobs (Presence.hpp PresenceDefaults: absentAfterNaks, probeInterval, probeIntervalMax) and the
/// engine's own (DeviceConfig.hpp EngineDefaults: maxRetries, faultBackoff, faultBackoffLongest,
/// faultBackoffResetOn, faultsBeforeReinit, inFlightTimeout, unidentifiedRetry) - lowerCamel: a
/// member spelled otherwise is not read, and the default applies without a word; `Reset` is a line
/// with hold()/release() (any GPIO reset callable) for a chip that declares ResetLow/ResetSettle.
///
/// What a run-time caller may change: `period<G>(ms)` sets a cyclic read group's period from
/// then on (0 parks it until request<G>() asks; the description's Period stays the nominal
/// rate the bus load is computed from).
///
/// The chip's code is DeviceCore's; this adds what only this device has, as DeviceOps (Hooks).
template<typename Transport,
         typename Clock,
         typename ChipT,
         typename Config = DefaultConfig,
         typename Reset  = NoReset,
         typename Gate   = NoGate>
    requires Transport::template
accepts<ChipT> struct Device
  : DeviceCore<detail::PortOf<Transport>, Clock, ChipT, detail::confirmsOf<ChipT, Config>()>
  , detail::ForwardClaims<Reset> {   // the reset line's pin claim, if any, becomes the device's
    using Core
      = DeviceCore<detail::PortOf<Transport>, Clock, ChipT, detail::confirmsOf<ChipT, Config>()>;
    /// The transport this device is on (I2C/Device.hpp, SPI/Transport.hpp): it states the log
    /// id, the part's own clock and whether requests carry it; the device asks nothing else.
    using TransportT   = Transport;
    using Chip         = ChipT;
    using GateT        = Gate;
    using ConfigT      = Config;
    using ResetT       = Reset;
    using EngineStateT = typename Core::EngineStateT;
    using Reads        = typename Core::Reads;

    /// Behind a bridge that can be switched off (Bridge.hpp): link() can say `offline`.
    static constexpr bool Bridged = detail::BridgedGate<Gate>;

    /// Config's `static bool enabled();` takes the part offline for a reason the bus cannot see
    /// (a module not fitted), as an inactive bridge would, and combines with one. It comes back
    /// as `WhileDisabled` says: `unpowered` (default, brought up again) or `disconnected`.
    ///     struct Config {
    ///         static constexpr std::uint8_t address = 0x49;
    ///         static bool enabled() { return slots.fitted(2); }
    ///     };
    static constexpr bool Enableable = requires {
        { Config::enabled() } -> std::convertible_to<bool>;
    };

    /// Or one function shared by many devices, with a value that says which one it is asked
    /// for (an index, a key packed into a word):
    ///     struct Config {
    ///         static constexpr bool (*enabledBy)(std::uint32_t) = &slotFitted;
    ///         static constexpr std::uint32_t enabledArg = 2;   // the slot
    ///         static bool enabled() { return slotFitted(2); }   // for the application
    ///     };
    static constexpr bool SharedEnable = requires {
        { Config::enabledBy(Config::enabledArg) } -> std::convertible_to<bool>;
    };
    // Either may add `static constexpr std::atomic<std::uint32_t> const* enabledGeneration`: a
    // counter the application steps whenever the answer may change, after the change (an interrupt
    // on this core may step it; one writer: `g.store(g.load(relaxed) + 1, release)`, no RMW call
    // on an M0+). Until it moves, the engine reuses the last answer instead of calling
    // the function on every loop turn (DeviceOps). A change nobody announces is not seen.

    static constexpr bool Switchable = Bridged || Enableable || SharedEnable;
    static_assert(!Switchable || detail::PortOf<Transport>::Features.switchable,
                  "this device is behind a bridge or has Config::enabled(), and its bus leaves the "
                  "engine's switchable feature out: give the bus one, "
                  "Kvasir::Engine::WithFeatures<Bus, {.switchable = true}> (EngineFeatures.hpp)");

    static constexpr WhileOff WhileDisabled
      = KVASIR_CONFIG_OR(Config, whileDisabled, WhileOff::unpowered);

    /// The part's I2C address; on an SPI transport, its chip-select GPIO (for the log).
    static constexpr std::uint8_t Address = Transport::template logId<Config, Chip>();

    static constexpr bool HasResetLine = !requires { Reset::NoLine; };

    // The policy knobs (EngineDefaults), each overridable by the Config.
    static constexpr std::uint8_t MaxRetries
      = static_cast<std::uint8_t>(KVASIR_CONFIG_OR(Config, maxRetries, EngineDefaults::maxRetries));

    // ConfirmChange<G> is the core's (from detail::confirmsOf<ChipT, Config>(), the same Config).

    static_assert(detail::checkConfirmConfig<Config,
                                             Reads>()
                    != detail::ConfirmConfig::unnamed,
                  "Config::confirmChange names no read group: add `using Confirm = List<...>;` "
                  "with the groups whose changes are confirmed");
    static_assert(detail::checkConfirmConfig<Config,
                                             Reads>()
                    != detail::ConfirmConfig::uncounted,
                  "Config::Confirm without Config::confirmChange confirms nothing");
    static_assert(detail::checkConfirmConfig<Config,
                                             Reads>()
                    != detail::ConfirmConfig::foreign,
                  "Config::Confirm names a group that is not one of this part's reads");
    static_assert(detail::checkConfirmConfig<Config,
                                             Reads>()
                    != detail::ConfirmConfig::counted,
                  "a read group with a counted read (a stream, a FIFO) cannot confirm changes: the "
                  "confirming read is a second read at once");

    static_assert(
      [] {
          bool ok = true;
          detail::forEach<Reads>([&](auto i) {
              using G = typename detail::Nth<Reads>::template type<decltype(i)::value>;
              ok      = ok && Core::template ConfirmChange<G> <= MaxRetries;
          });
          return ok;
      }(),
      "a read group's ConfirmChange is over MaxRetries: its confirming re-reads are the run's "
      "retries, and a run out of retries rejects the reading");

    static constexpr std::chrono::milliseconds FaultBackoff
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Config, faultBackoff, EngineDefaults::faultBackoff));

    static constexpr std::chrono::milliseconds FaultBackoffLongest
      = Kvasir::asDuration(KVASIR_CONFIG_OR(Config,
                                            faultBackoffLongest,
                                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                              EngineDefaults::faultBackoffLongest)));
    static_assert(FaultBackoffLongest >= FaultBackoff,
                  "faultBackoffLongest is the ceiling faultBackoff doubles up to: not below it");

    static constexpr BackoffReset FaultBackoffResetOn
      = KVASIR_CONFIG_OR(Config, faultBackoffResetOn, EngineDefaults::faultBackoffResetOn);

    static constexpr std::uint8_t FaultsBeforeReinit = static_cast<std::uint8_t>(
      KVASIR_CONFIG_OR(Config, faultsBeforeReinit, EngineDefaults::faultsBeforeReinit));

    static constexpr std::chrono::milliseconds InFlightTimeout = Kvasir::asDuration(
      KVASIR_CONFIG_OR(Config, inFlightTimeout, EngineDefaults::inFlightTimeout));

    static constexpr std::chrono::milliseconds UnidentifiedRetry = Kvasir::asDuration(
      KVASIR_CONFIG_OR(Config, unidentifiedRetry, EngineDefaults::unidentifiedRetry));

    /// The clock this part asks for, in Hz, or 0 when it asks for none (the transport says:
    /// I2C/Device.hpp `Config::busClock`, else `Chip::I2cMaxClock`; SPI none). On a transport
    /// with perDeviceClock the device is run at it (capped at the bus's baudRate), and every
    /// request it submits carries the bus's timing for it; on a bus without, a clock below the
    /// bus's rate is a compile error (the driver's timing()).
    static constexpr std::uint32_t OwnClockHz = Transport::template ownClockHz<Config, Chip>();

    /// What the part runs at: its own clock capped at the bus's rate, the bus's rate when it
    /// asks for none, 0 on a bus that says no rate. Bus::busLoad() weighs its bits with it.
    static constexpr std::uint32_t BusHz = [] {
        if constexpr(requires { Transport::BaudRate; }) {
            auto const bus = static_cast<std::uint32_t>(Transport::BaudRate);
            return OwnClockHz != 0 ? std::min(OwnClockHz, bus) : bus;
        } else {
            return OwnClockHz;
        }
    }();

    /// The bus runs each device at its own clock: its request has a `timing` member.
    static constexpr bool PerDeviceClock = Transport::perDeviceClock;

    /// The timing every request of this device carries on a PerDeviceClock bus, for its own
    /// clock or else the bus's rate. On any other bus the driver's timing() only checks a
    /// clock the part asks for (a part slower than the bus fails the build there).
    static constexpr auto BusTiming = [] {
        if constexpr(PerDeviceClock) {
            return Transport::timing(
              OwnClockHz != 0 ? OwnClockHz : static_cast<std::uint32_t>(Transport::BaudRate));
        } else {
            if constexpr(OwnClockHz != 0 && requires { Transport::timing(OwnClockHz); }) {
                [[maybe_unused]] constexpr auto checked = Transport::timing(OwnClockHz);
            }
            return Empty{};
        }
    }();

    static_assert(
      [] {
          if constexpr(requires { Transport::CallbackSize; }) {
              return static_cast<std::size_t>(Transport::CallbackSize) >= Core::CallbackBytes;
          }
          return true;
      }(),
      "the bus behavior's CallbackSize is smaller than the engine's completion lambda "
      "(Device::CallbackBytes): raise the I2CBehaviorQueued CallbackSize template argument");
    static_assert(presenceKnobsOk<Config>,
                  "a probe interval, and a maximum it grows towards");

    constexpr Device() : Core(&Hooks) {}

    /// The gate, for an application that has to point it at something: a device behind an
    /// I2C switch is told which switch and which arbiter at start-up (Mux.hpp). An ungated
    /// device has an empty one and nothing to say to it.
    [[nodiscard]] Gate& gate() { return gate_; }

private:
    [[nodiscard]] static Device& self_(EngineStateT& e) { return static_cast<Device&>(e); }

    [[nodiscard]] static Device const& self_(EngineStateT const& e) {
        return static_cast<Device const&>(e);
    }

    using PortT      = detail::PortOf<Transport>;
    using DeviceOpsT = detail::DeviceOps<PortT, Clock>;
    using KnobsT     = detail::DeviceKnobs<PortT, Clock>;

    /// The per-device hooks of the gate, the bridge and the enable: static functions of exactly the types of their
    /// DeviceKnobs / DeviceOps fields. Each is named (so instantiated) only where the device needs it; the tables
    /// below pick it or a null pointer.
    struct DeviceHooks {
        static bool claimGate(EngineStateT& e) { return self_(e).gate_.claim(); }

        static void releaseGate(EngineStateT& e) { self_(e).gate_.release(); }

        static std::uint32_t gateGeneration(EngineStateT const& e) {
            return static_cast<std::uint32_t>(self_(e).gate_.generation());
        }

        static bool gateOffline(EngineStateT const& e) { return self_(e).gate_.offline(); }

        static bool enabled(std::uint32_t) { return static_cast<bool>(Config::enabled()); }
    };

    /// What this device shares with others that have the same: its knobs, one object per
    /// distinct set (detail::deviceKnobs); a hook it does not need is null.
    static constexpr KnobsT Knobs{
      .claimGate =
        [] {
            using Fn = bool (*)(EngineStateT&);
            if constexpr(Gate::Gated) {
                return Fn{&DeviceHooks::claimGate};
            } else {
                return Fn{};
            }
        }(),
      .releaseGate =
        [] {
            using Fn = void (*)(EngineStateT&);
            if constexpr(Gate::Gated) {
                return Fn{&DeviceHooks::releaseGate};
            } else {
                return Fn{};
            }
        }(),
      .gateGeneration =
        [] {
            using Fn = std::uint32_t (*)(EngineStateT const&);
            if constexpr(Bridged) {
                return Fn{&DeviceHooks::gateGeneration};
            } else {
                return Fn{};
            }
        }(),
      .resetHold =
        [] {
            using Fn = void (*)();
            if constexpr(HasResetLine) {
                return Fn{[] { Reset::hold(); }};
            } else {
                return Fn{};
            }
        }(),
      .resetRelease =
        [] {
            using Fn = void (*)();
            if constexpr(HasResetLine) {
                return Fn{[] { Reset::release(); }};
            } else {
                return Fn{};
            }
        }(),
      .presence              = presenceKnobs<Config>(),
      .faultBackoffMs        = static_cast<std::uint32_t>(FaultBackoff.count()),
      .faultBackoffLongestMs = static_cast<std::uint32_t>(FaultBackoffLongest.count()),
      .inFlightTimeoutMs     = static_cast<std::uint32_t>(InFlightTimeout.count()),
      .unidentifiedRetryMs   = static_cast<std::uint32_t>(UnidentifiedRetry.count()),
      .faultsBeforeReinit    = FaultsBeforeReinit,
      .maxRetries            = MaxRetries,
      .bridgeUnpowered =
        [] {
            if constexpr(Bridged) {
                return Gate::Bridge::whileOff == WhileOff::unpowered;
            } else {
                return false;
            }
        }(),
      .disabledUnpowered = WhileDisabled == WhileOff::unpowered,
      .faultBackoffOnAck = FaultBackoffResetOn == BackoffReset::anyAck};

    /// What only this device has, for the engine.
    static constexpr DeviceOpsT Hooks{
      .submit =
        [] {
            using Fn = bool (*)(typename Transport::Request const&);
            if constexpr(PerDeviceClock) {
                // The engine's code is shared by every device on the port: the timing is this
                // device's own, so it goes in here, on the way out.
                return Fn{[](typename Transport::Request const& r) -> bool {
                    auto timed   = r;
                    timed.timing = BusTiming;
                    return Transport::submit(timed);
                }};
            } else {
                return Fn{[](typename Transport::Request const& r) -> bool {
                    return Transport::submit(r);
                }};
            }
        }(),
      .knobs = detail::deviceKnobs<PortT, Clock, Knobs>,
      .enabled =
        [] {
            using Fn = bool (*)(std::uint32_t);
            if constexpr(SharedEnable) {
                static_assert(
                  std::is_convertible_v<decltype(Config::enabledBy), Fn>,
                  "Config::enabledBy is a bool(*)(std::uint32_t): the value it is given is "
                  "Config::enabledArg, a std::uint32_t");
                return Fn{Config::enabledBy};
            } else if constexpr(Enableable) {
                return Fn{&DeviceHooks::enabled};
            } else {
                return Fn{};
            }
        }(),
      .enabledArg =
        [] {
            if constexpr(SharedEnable) {
                // braces: a pointer or a wider value does not narrow into the word silently
                return std::uint32_t{Config::enabledArg};
            } else {
                return std::uint32_t{};
            }
        }(),
      .enabledGeneration =
        [] {
            if constexpr(requires { Config::enabledGeneration; }) {
                return static_cast<std::atomic<std::uint32_t> const*>(Config::enabledGeneration);
            } else {
                return static_cast<std::atomic<std::uint32_t> const*>(nullptr);
            }
        }(),
      .gateOffline =
        [] {
            using Fn = bool (*)(EngineStateT const&);
            if constexpr(Bridged) {
                return Fn{&DeviceHooks::gateOffline};
            } else {
                return Fn{};
            }
        }(),
      .address = Address};

    [[no_unique_address]] Gate gate_{};
};

}   // namespace Kvasir::Engine
