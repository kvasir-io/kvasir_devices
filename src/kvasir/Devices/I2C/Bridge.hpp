#pragma once

#include "../Log.hpp"
#include "Device.hpp"

#include <chrono>
#include <concepts>
#include <cstdint>
#include <tuple>
#include <type_traits>

/// Parts behind a bridge that is not always active: a buffer or an isolator with an enable pin,
/// an analog switch, a port something can be plugged into. While the bridge is not active the
/// parts behind it are not on the wire, and a driver that talks all the same collects NAKs,
/// counts errors, gets parked as absent and probes for ever -- for a part that is fine.
///
/// Which bridge a part is behind is said once, in its type, and the engine does the rest:
///
///     struct ExtPort : Kvasir::I2C::GpioBridge<HW::Pin::extEnable> {       // BridgeGpio.hpp
///         static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
///         static constexpr auto Settle   = std::chrono::milliseconds{20};
///     };
///     using Sht = Kvasir::I2C::BehindBridge<ExtPort, Kvasir::I2C::Device<I2c, Clock, Chips::Sht3x>>;
///     ...
///     bus.bridge<ExtPort>().on();        // the application's side; off(), state()
///     bus.get<Sht>().link();             // Link::offline while the bridge is not `on`
///
/// While its bridge is not `on` a device sends nothing, counts nothing, is not probed and says
/// `Link::offline`; latest() keeps the last sample, valid() is false, what the application
/// sets stays owed and a request<G>() stays pending. When the bridge is `on` again the device
/// goes on by itself, the way the bridge's `WhileOff` says: brought up from the start
/// (`unpowered`: StartupDelay, the identity, Init, every owned write group sent again,
/// bringUps() steps) or carrying on where it was (`disconnected`).
///
/// A bridge description is a type with static members:
///
///   - `WhileOff` (Device.hpp), required: it is the one thing nobody can guess;
///   - `Settle`: from switching on to the first transaction (default 1 ms), which is also the
///     debounce of a sensed bridge -- it starts again whenever the input goes away meanwhile;
///   - `Start`: `BridgeStart::off` (default) or `on` at the first handler() turn;
///   - `Claims`, optional: the pin, for the Startup list;
///   - `Mode`: `BridgeMode::manual` (default) or `switched` -- the bridge used as if it were a
///     channel of a switch: the engine switches it on for the transactions of the parts behind
///     it, leaves it on while nobody else wants the wire, and switches it off for another
///     bridge of its ExclusiveGroup or for a part in front with a BridgeFrontGate. The parts are
///     not offline meanwhile, they wait. on() and off() enable and disable it (disabled: the
///     parts are offline); it starts enabled. Needs WhileOff::disconnected;
///   - `Addresses`: `BridgeAddresses::shared` (default) or `separate` -- the application
///     promises that this bridge and another one are never active together, and the Bus's
///     address check then lets a part behind this one share an address with a part behind the
///     other. It never does with a part behind no bridge: that one is always on the wire;
///   - `ExclusiveGroup`, optional: a tag type. Of the bridges that name the same one at most
///     one is ever active -- a Bus sees to it, break before make -- which is what makes the
///     same address behind two of them legal;
///   - *who switches it*, one of
///       `static void drive(bool active)`   the firmware does, through on() and off()
///                                          (GpioBridge, BridgeGpio.hpp);
///       `using Part`, drive(part, active), driven(part)
///                                          the firmware does, through an output of another
///                                          part of the same Bus (PartBridge below);
///       `static bool active()`             something else does, and this says what it did
///                                          (SensedBridge below).
///
/// The run-time side of a bridge is its BridgeLine: `off -> settling -> on -> closing -> off`.
/// `closing` is what keeps the firmware from cutting its own transaction: off() stops the
/// parts behind the bridge from starting anything at once, and the pin moves when the last
/// transaction of theirs is off the wire. A sensed bridge has no `closing` -- it is gone when
/// it is gone -- and the transaction it took with it is dropped by the engine, not counted.
///
/// A Bus owns the lines of the bridges its devices name and handles them (Bus.hpp). A device
/// outside a Bus is bound by hand: `line.handler()` each turn before the device's, and once
/// `device.gate().bind(line)`.
namespace Kvasir::I2C {

/// What a bridge is at the first handler() turn.
enum class BridgeStart : std::uint8_t { off, on };

/// Which level of its enable makes a bridge active.
enum class BridgePolarity : std::uint8_t { activeHigh, activeLow };

/// Who decides when a bridge is active.
enum class BridgeMode : std::uint8_t {
    manual,     ///< the application (on(), off()) or the sensed input: off means offline
    switched,   ///< the engine, as it does a channel of a switch (Mux.hpp): the bridge is
                ///< switched on for the transactions of the parts behind it and off when
                ///< somebody else needs the wire. on() and off() enable and disable it.
};

/// What a bridge does for the Bus's address check (Bus.hpp, addressesDistinct).
enum class BridgeAddresses : std::uint8_t {
    shared,     ///< none of it: an address behind the bridge is taken everywhere else as well
    separate,   ///< the application sees to it that this bridge and any other one are never
                ///< active together: an address behind it may be used behind another bridge
                ///< too -- never in front, where a part is on the wire all the time
};

enum class BridgeState : std::uint8_t {
    off,        ///< not active: the parts behind it are offline
    settling,   ///< switched on, or sensed active: `Settle` runs, the parts are still offline
    on,         ///< active: the parts talk
    closing,    ///< asked to go off: nothing new starts, the pin moves when the wire is free
};

namespace detail {
    template<typename B>
    concept PinBridge = requires { B::drive(true); };

    /// Switched through a write group of another device: the line is bound to that device.
    template<typename B>
    concept PartDrivenBridge = requires { typename B::Part; };

    template<typename B>
    concept DrivenBridge = PinBridge<B> || PartDrivenBridge<B>;

    template<typename B>
    concept SensedBridge = requires {
        { B::active() } -> std::convertible_to<bool>;
    };

    template<typename B>
    [[nodiscard]] constexpr std::chrono::milliseconds bridgeSettle() {
        if constexpr(requires { B::Settle; }) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(B::Settle);
        } else {
            return std::chrono::milliseconds{1};
        }
    }

    template<typename B>
    inline constexpr bool bridgeSwitched = [] {
        if constexpr(requires { B::Mode; }) {
            return B::Mode == BridgeMode::switched;
        } else {
            return false;
        }
    }();

    template<typename B>
    inline constexpr bool bridgeSeparates = [] {
        if constexpr(requires { B::Addresses; }) {
            return B::Addresses == BridgeAddresses::separate;
        } else {
            return false;
        }
    }();

    /// A switched bridge is there to be used, so it starts enabled unless it says otherwise.
    template<typename B>
    [[nodiscard]] constexpr BridgeStart bridgeStart() {
        if constexpr(requires { B::Start; }) {
            return B::Start;
        } else {
            return bridgeSwitched<B> ? BridgeStart::on : BridgeStart::off;
        }
    }

    /// The device a PartBridge is switched through, and the bring-ups of it the line has seen.
    /// An empty member for every other bridge.
    template<typename B>
    struct BridgePart {
        using PartT = void*;
    };

    template<PartDrivenBridge B>
    struct BridgePart<B> {
        using PartT = typename B::Part;

        PartT*        part{nullptr};
        std::uint16_t bringUps{};
    };

    /// The bridge's pin claim, if it has one, becomes the line's (as PowerRail's switch).
    template<typename B>
    struct BridgeClaims {};

    template<typename B>
        requires requires { typename B::Claims; }
    struct BridgeClaims<B> {
        using Claims = typename B::Claims;
    };
}   // namespace detail

/// What the gates of the parts behind a bridge see of it: no clock in it, so a gate needs to
/// know the bridge and nothing else.
template<typename B>
class BridgeLink {
public:
    static constexpr bool Switched = detail::bridgeSwitched<B>;

    [[nodiscard]] BridgeState state() const { return state_; }

    [[nodiscard]] bool active() const { return state_ == BridgeState::on; }

    /// Whether the parts behind it are offline: not active -- or, for a switched bridge, not
    /// enabled; one that is only switched away is a wait, as a channel of a switch is.
    [[nodiscard]] bool partsOffline() const {
        if constexpr(Switched) {
            return !enabled_;
        } else {
            return state_ != BridgeState::on;
        }
    }

    /// Steps every time the parts behind it are back from being offline.
    [[nodiscard]] std::uint32_t generation() const { return generation_; }

    /// Parts behind the bridge with a transaction on the wire, or about to have one.
    [[nodiscard]] std::uint8_t holders() const { return holders_; }

    void hold() { ++holders_; }

    void letGo() {
        if(holders_ != 0) { --holders_; }
    }

    /// A switched bridge: a part behind it has something to send. Said every turn it has.
    void request() { requested_ = true; }

    /// A switched bridge that is enabled, off, and asked for: it waits for the wire.
    [[nodiscard]] bool waiting() const {
        return Switched && enabled_ && requested_ && state_ == BridgeState::off;
    }

    /// A part in front wants the bridge off for a run of its own (BridgeFrontGate): asked every
    /// turn until it is, then held for as long as the run is on the wire.
    void requestClosed() { closeRequested_ = true; }

    void holdClosed() { ++closedHolders_; }

    void letGoClosed() {
        if(closedHolders_ != 0) { --closedHolders_; }
    }

protected:
    BridgeState   state_{BridgeState::off};
    std::uint32_t generation_{};
    std::uint8_t  holders_{};
    std::uint8_t  closedHolders_{};
    bool          requested_{};
    bool          closeRequested_{};
    /// What the application or the sensed input wants of it: manual, active; switched, usable.
    bool enabled_{detail::bridgeStart<B>() == BridgeStart::on};
};

/// One bridge at run time.
template<typename Clock, typename B>
class BridgeLine
  : public BridgeLink<B>
  , public detail::BridgeClaims<B> {
    static_assert(
      requires { B::WhileOff; },
      "a bridge description says what its parts go through while it is off: "
      "static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered; (or "
      "::disconnected)");
    static_assert(detail::DrivenBridge<B> != detail::SensedBridge<B>,
                  "a bridge description has either drive(bool), for a bridge the firmware "
                  "switches, or active(), for one it only senses (Bridge.hpp)");

    static_assert(!detail::bridgeSwitched<B> || detail::DrivenBridge<B>,
                  "BridgeMode::switched is for a bridge the firmware switches: drive(bool) or a "
                  "PartBridge");
    static_assert(!detail::bridgeSwitched<B> || B::WhileOff == WhileOff::disconnected,
                  "BridgeMode::switched switches the bridge off between the transactions of its "
                  "parts: they have to keep their registers meanwhile (WhileOff::disconnected)");

public:
    using Bridge    = B;
    using TimePoint = typename Clock::time_point;

    static constexpr bool Switched   = detail::bridgeSwitched<B>;
    static constexpr bool Driven     = detail::DrivenBridge<B>;
    static constexpr bool PartDriven = detail::PartDrivenBridge<B>;
    static constexpr auto Settle     = detail::bridgeSettle<B>();

    /// Switch it on: the parts talk after `Settle`. Taken back by off() at any time. A
    /// switched bridge is enabled by it: from now on the engine switches it as its parts ask.
    void on()
        requires Driven
    {
        if constexpr(Switched) {
            if(!this->enabled_) { ++this->generation_; }   // the parts are back from offline
        }
        this->enabled_ = true;
    }

    /// Switch it off: the parts are offline from now on, the pin follows when the wire is free.
    void off()
        requires Driven
    {
        this->enabled_ = false;
        if(this->state_ == BridgeState::on) { this->state_ = BridgeState::closing; }
    }

    [[nodiscard]] bool wanted() const { return this->enabled_; }

    /// Times the parts came back from offline (manual: times it became `on`), for a status page.
    [[nodiscard]] std::uint32_t activations() const { return this->generation_; }

    /// Times a switched bridge was switched on.
    [[nodiscard]] std::uint32_t switchOns() const { return switchOns_; }

    /// When it last became `on`: what a Bus takes turns by among switched bridges of a group.
    [[nodiscard]] TimePoint servedAt() const { return servedAt_; }

    /// The device the bridge is switched through (a PartBridge). Called once, before the first
    /// handler() turn; a Bus does it.
    void bindPart(typename detail::BridgePart<B>::PartT& part)
        requires PartDriven
    {
        part_.part = &part;
    }

    /// Once per loop turn, before the devices behind it. `mayStart` false keeps a bridge that
    /// is off from leaving `off`: how a Bus holds the members of an ExclusiveGroup apart.
    /// `contested` tells a switched bridge that another one of its group waits for the wire:
    /// it lets no new run start and goes off when the last one is over.
    void handler(bool mayStart  = true,
                 bool contested = false) {
        auto const now = Clock::now();
        if(!started_) {
            if constexpr(PartDriven) {
                // Not before the part's first bring-up: only then does its write group hold
                // the description's Initial, and a modify() before that would start from a
                // zero Value -- every other pin of the expander driven low.
                if(part_.part == nullptr || part_.part->bringUps() == 0) {
                    partTurn_();
                    return;
                }
            }
            started_ = true;
            if constexpr(Driven) {
                drive_(false);   // a defined level, and an output, before anything else
            }
        }
        if constexpr(!Driven) { this->enabled_ = B::active(); }
        if constexpr(PartDriven) { partTurn_(); }

        // What the state machine follows. Manual: what is wanted of it. Switched: on for the
        // parts that ask, left on while nobody else wants the wire (switching costs a Settle),
        // off for another bridge of the group or a part in front that does.
        bool wanted = this->enabled_;
        if constexpr(Switched) {
            bool const demand     = this->requested_ || this->holders_ != 0;
            bool const closeAsked = this->closeRequested_ || this->closedHolders_ != 0;
            this->requested_      = false;
            this->closeRequested_ = false;
            switch(this->state_) {
            case BridgeState::off:      wanted = this->enabled_ && !closeAsked && demand; break;
            case BridgeState::settling: wanted = this->enabled_ && !closeAsked; break;
            case BridgeState::on:       wanted = this->enabled_ && !closeAsked && !contested; break;
            case BridgeState::closing:  wanted = false; break;   // off first, then a clean start
            }
        } else {
            mayStart              = mayStart && this->closedHolders_ == 0;
            this->closeRequested_ = false;
        }

        switch(this->state_) {
        case BridgeState::off:
            if(wanted && mayStart) {
                drive_(true);
                until_       = now + Settle;
                this->state_ = BridgeState::settling;
            }
            break;
        case BridgeState::settling:
            if(!wanted) {
                drive_(false);
                this->state_ = BridgeState::off;
            } else if(!driven_()) {
                until_ = now + Settle;   // Settle runs from the enable reaching the bridge
            } else if(now > until_) {
                this->state_ = BridgeState::on;
                servedAt_    = now;
                ++switchOns_;
                if constexpr(!Switched) {
                    ++this->generation_;
                    UC_LOG_I("bridge: on");
                }
            }
            break;
        case BridgeState::on:
            if(wanted) { break; }
            if constexpr(Driven) {
                this->state_ = BridgeState::closing;
            } else {
                this->state_ = BridgeState::off;
                UC_LOG_I("bridge: off");
            }
            break;
        case BridgeState::closing:
            if(wanted) {
                // Nothing was cut. The parts of a manual bridge went offline when the closing
                // began, so they come back as from a real one: what it costs is a bring-up.
                this->state_ = BridgeState::on;
                if constexpr(!Switched) { ++this->generation_; }
            } else if(this->holders_ == 0) {
                drive_(false);
                this->state_ = BridgeState::off;
                if constexpr(!Switched) { UC_LOG_I("bridge: off"); }
            }
            break;
        }
    }

private:
    void drive_([[maybe_unused]] bool active) {
        if constexpr(PartDriven) {
            if(part_.part != nullptr) { B::drive(*part_.part, active); }
        } else if constexpr(Driven) {
            B::drive(active);
        }
    }

    /// The enable is where it was driven to: at once for a pin, for a part once the write is
    /// through and the part answers.
    [[nodiscard]] bool driven_() const {
        if constexpr(PartDriven) {
            return part_.part != nullptr && B::driven(*part_.part);
        } else {
            return true;
        }
    }

    /// The part the enable is an output of was brought up again: its outputs were at their
    /// reset levels for a while, so the bridge was off whatever this line thought. The
    /// bring-up has sent the write group again, the enable with it; the parts behind the
    /// bridge go through a return.
    void partTurn_() {
        if(part_.part == nullptr) {
            if(!warned_) {
                warned_ = true;
                UC_LOG_W(
                  "a bridge switched through another part was never bound to it: "
                  "bindPart() it from the object that owns the line");
            }
            return;
        }
        if(!part_.part->broughtUp(part_.bringUps)) { return; }
        if(this->state_ == BridgeState::on || this->state_ == BridgeState::closing) {
            UC_LOG_W("bridge: the part it is switched through started over");
            if(this->enabled_) {
                until_       = Clock::now() + Settle;
                this->state_ = BridgeState::settling;
            } else {
                this->state_ = BridgeState::off;
            }
        }
    }

    bool                                        started_{false};
    bool                                        warned_{false};
    TimePoint                                   until_{};
    TimePoint                                   servedAt_{TimePoint::min()};
    std::uint32_t                               switchOns_{};
    [[no_unique_address]] detail::BridgePart<B> part_{};
};

namespace detail {
    /// What a gate on a switch says about it, for a BridgeGate around it to say as well.
    template<typename Inner>
    struct InnerSwitch {};

    template<typename Inner>
        requires requires { typename Inner::Mux; }
    struct InnerSwitch<Inner> {
        using Mux      = typename Inner::Mux;
        using Channels = typename Inner::Channels;
        using Arbiter  = typename Inner::Arbiter;

        static constexpr std::uint8_t Mask = Inner::Mask;
    };
}   // namespace detail

/// Behind bridge `B`, as a gate. `Inner` is the gate the part would have without the bridge:
/// NoGate, or the channel of a switch (Mux.hpp) when both are between the part and the
/// controller. The bridge is asked first, so a part that is offline never takes a switch.
template<typename B, typename Inner = NoGate>
struct BridgeGate : detail::InnerSwitch<Inner> {
    static constexpr bool Gated   = true;
    static constexpr bool Bridged = true;
    /// In front of its switch when the inner gate says so (MuxFrontGate).
    static constexpr bool Front = requires { requires static_cast<bool>(Inner::Front); };

    using Bridge = B;
    using InnerT = Inner;

    /// Say which line this is behind. Called once, before the device is handled.
    void bind(BridgeLink<B>& line) { line_ = &line; }

    /// The inner gate's binding (a MuxGate's switch and arbiter).
    template<typename... Ts>
    void bind(Ts&... ts)
        requires requires(Inner& i) { i.bind(ts...); }
    {
        inner_.bind(ts...);
    }

    [[nodiscard]] bool bound() const {
        if constexpr(requires(Inner const& i) { i.bound(); }) {
            return line_ != nullptr && inner_.bound();
        } else {
            return line_ != nullptr;
        }
    }

    [[nodiscard]] Inner& inner() { return inner_; }

    [[nodiscard]] bool offline() const { return line_ == nullptr || line_->partsOffline(); }

    [[nodiscard]] std::uint32_t generation() const {
        return line_ == nullptr ? 0 : line_->generation();
    }

    bool claim() {
        if(line_ == nullptr) {
            // Never bound, so for ever offline: say it once instead of looking switched off.
            if(!warned_) {
                warned_ = true;
                UC_LOG_W(
                  "a device behind a bridge was never bound to its line: bind() it from "
                  "the object that owns the line");
            }
            return false;
        }
        // A switched bridge that is closing is still on, for the run that holds it: that run
        // goes on to its end (or its next wait), and only then does the bridge go.
        bool const finishing
          = BridgeLink<B>::Switched && holding_ && line_->state() == BridgeState::closing;
        if(!line_->active() && !finishing) {
            line_->request();   // a switched bridge comes for it; any other takes no notice
            return false;
        }
        if(!inner_.claim()) { return false; }
        if(!holding_) {
            holding_ = true;
            line_->hold();
        }
        return true;
    }

    void release() {
        inner_.release();
        if(holding_) {
            holding_ = false;
            line_->letGo();
        }
    }

private:
    BridgeLink<B>*              line_{nullptr};
    [[no_unique_address]] Inner inner_{};
    bool                        holding_{};
    bool                        warned_{};
};

/// A part *in front of* bridges whose traffic must not reach what is behind them -- a part
/// there misreads it, as the TLV493D does the SAM-M8Q's (Mux.hpp, MuxFrontGate): a gate that
/// claims the bridges `Bs...` off, as a MuxFrontGate claims its switch closed. A switched bridge
/// goes off for it (after the run on the wire) and stays off while this part's run lasts; a
/// manual or a sensed one is only waited for. Its address still has to be unique: the gate
/// decides when the part talks, and it listens all the time.
///     using Rtc = Device<I2c, Clock, Chips::Rv8803, DefaultConfig, NoReset,
///                        BridgeFrontGate<ExtPort>>;
template<typename... Bs>
struct BridgeFrontGate {
    static constexpr bool Gated = true;
    static constexpr bool Front = true;

    /// Whether `B` is one of the bridges this gate keeps off while its part talks.
    template<typename B>
    static constexpr bool Closes = (std::is_same_v<B, Bs> || ...);

    template<typename B>
        requires Closes<B>
    void bind(BridgeLink<B>& line) {
        std::get<BridgeLink<B>*>(lines_) = &line;
    }

    [[nodiscard]] bool bound() const {
        return ((std::get<BridgeLink<Bs>*>(lines_) != nullptr) && ...);
    }

    bool claim() {
        if(!bound()) { return false; }
        if(holding_) { return true; }
        bool allOff = true;
        ((std::get<BridgeLink<Bs>*>(lines_)->requestClosed(),
          allOff = allOff && std::get<BridgeLink<Bs>*>(lines_)->state() == BridgeState::off),
         ...);
        if(!allOff) { return false; }
        holding_ = true;
        (std::get<BridgeLink<Bs>*>(lines_)->holdClosed(), ...);
        return true;
    }

    void release() {
        if(!holding_) { return; }
        holding_ = false;
        (std::get<BridgeLink<Bs>*>(lines_)->letGoClosed(), ...);
    }

private:
    std::tuple<BridgeLink<Bs>*...> lines_{};
    bool                           holding_{};
};

/// Device `D` behind bridge `B`: the same device, its gate wrapped.
///     using Light = BehindBridge<ExtPort, OnChannel<Chips::Veml7700, 2>>;
template<typename B, typename D>
using BehindBridge = Device<typename D::I2cBus,
                            typename D::ClockT,
                            typename D::Chip,
                            typename D::ConfigT,
                            typename D::ResetT,
                            BridgeGate<B, typename D::GateT>>;

/// A bridge the firmware only senses, from a `Source` with `static bool active()`: a
/// plug-detect pin, a power-good line, a flag another task keeps.
///     struct Dock : Kvasir::I2C::SensedBridge<DockDetect> {
///         static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
///         static constexpr auto Settle   = std::chrono::milliseconds{200};   // and debounce
///     };
template<typename Source>
struct SensedBridge {
    [[nodiscard]] static bool active() { return Source::active(); }
};

/// A bridge whose enable is an output of another part of the same Bus -- a pin of a port
/// expander, a bit of a load switch -- as a base for its description: `Mask` of write group `W`
/// of device `PartT`, whose Value is an unsigned integer.
///     struct ExtPort : Kvasir::I2C::PartBridge<Expander, Chips::Pcf8574::Port, 0x10> {
///         static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
///     };
/// The write goes through the engine like any other, so the parts behind the bridge wait for
/// it; and when the expander is brought up again the bridge counts as having been off.
/// Nothing is driven before the expander's first bring-up, so until then the enable is what
/// the expander's reset and the write group's `Initial` make it: choose them to mean "off".
template<typename PartT, typename W, auto Mask, BridgePolarity P = BridgePolarity::activeHigh>
struct PartBridge {
    using Part = PartT;

    static void drive(Part& part,
                      bool  active) {
        part.template modify<W>([&](auto& v) {
            using V = std::remove_cvref_t<decltype(v)>;
            if(active == (P == BridgePolarity::activeHigh)) {
                v = static_cast<V>(v | static_cast<V>(Mask));
            } else {
                v = static_cast<V>(v & static_cast<V>(~static_cast<V>(Mask)));
            }
        });
    }

    [[nodiscard]] static bool driven(Part const& part) {
        return part.answering() && !part.template pending<W>();
    }
};

}   // namespace Kvasir::I2C
