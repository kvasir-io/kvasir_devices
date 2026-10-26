#pragma once
// An I2C part on the chip-description engine (Engine/Device.hpp): the I2C transport's traits and
// the binding firmwares write.
//
//     using Sensor = Kvasir::I2C::Device<I2c1, Clock, Chips::Sht3x>;
//     Sensor sht{};
//     sht.turn();
//     if(sht.valid()) { auto const& s = sht.latest(); }
#include "../Engine/Device.hpp"
#include "../Quantities.hpp"
#include "Concepts.hpp"

#include <cstdint>

namespace Kvasir::I2C {

/// An I2C master (a queued chip driver, a WithFeatures over one, the host FakeBus) as the
/// engine's transport: the master's Request, submit() and features unchanged - so the engine's
/// port (Engine::detail::PortOf) is the master's own -, plus what the engine asks a transport.
template<typename Master>
struct Transport : Master {
    /// What the log names a device by: its I2C address (`Config::address`, else the chip's).
    template<typename Config,
             typename ChipT>
    static constexpr std::uint8_t logId() {
        if constexpr(requires { Config::address; }) {
            return static_cast<std::uint8_t>(Config::address);
        } else {
            return static_cast<std::uint8_t>(ChipT::Address);
        }
    }

    /// The clock the part asks for, in Hz, or 0: `Config::busClock` (this device), else
    /// `Chip::I2cMaxClock` (what the part's data sheet allows; not `MaxClock`, which an SPI
    /// description of the same part uses for its SPI clock). Both are Units::Hertz.
    template<typename Config,
             typename ChipT>
    static constexpr std::uint32_t ownClockHz() {
        if constexpr(requires { Config::busClock; }) {
            if constexpr(requires { ChipT::I2cMaxClock; }) {
                static_assert(Config::busClock.numerical_value_in(Units::si::hertz)
                                <= ChipT::I2cMaxClock.numerical_value_in(Units::si::hertz),
                              "Config::busClock is above the chip's Chip::I2cMaxClock");
            }
            return Config::busClock.numerical_value_in(Units::si::hertz);
        } else if constexpr(requires { ChipT::I2cMaxClock; }) {
            return ChipT::I2cMaxClock.numerical_value_in(Units::si::hertz);
        } else {
            return std::uint32_t{0};
        }
    }

    /// The bus runs each device at its own clock: its request has a `timing` member
    /// (I2CConfig::perDeviceClock).
    static constexpr bool perDeviceClock = requires(typename Master::Request r) { r.timing; };

    template<typename ChipT>
    static constexpr bool accepts = Chip<ChipT>;
};

/// A chip description on an I2C bus: Engine::Device over the master's Transport. `I2cBus` is
/// the master itself, so a Bus<> can check that the devices it was handed are all on the bus it
/// names - a mixed set would give a queue depth and a bus load for traffic that is not on one wire.
template<typename I2c,
         typename Clock,
         typename ChipT,
         typename Config = Engine::DefaultConfig,
         typename Reset  = Kvasir::NoReset,
         typename Gate   = Engine::NoGate>
    requires Chip<ChipT>
struct Device : Engine::Device<Transport<I2c>, Clock, ChipT, Config, Reset, Gate> {
    using I2cBus = I2c;

    /// User-provided on purpose: defaulted below the class, not on this declaration, and not
    /// implicit. Otherwise `Device()` (std::tuple's value-initialization of each device in Bus<>)
    /// zero-fills the whole object before the engine's constructor runs - one memset per device,
    /// and a different inlining of the caller that builds the bus.
    constexpr Device();

    static_assert(
      [] {
          if constexpr(detail::HasAddresses<ChipT>) {
              for(auto const a : ChipT::Addresses) {
                  if(a == Device::Address) { return true; }
              }
              return false;
          }
          return true;
      }(),
      "Config::address is not one of the addresses this chip can have (Chip::Addresses)");
};

template<typename I2c,
         typename Clock,
         typename ChipT,
         typename Config,
         typename Reset,
         typename Gate>
    requires Chip<ChipT>
constexpr Device<I2c,
                 Clock,
                 ChipT,
                 Config,
                 Reset,
                 Gate>::Device() = default;

}   // namespace Kvasir::I2C
