/// What must not compile, built by ctest and judged by its error (test/i2c/CMakeLists.txt): one switch gate claimed
/// twice for the same address (Mux.hpp SwitchGate, Bus.hpp clash()).
///   GATE_FAIL_SAME_CHANNEL  two parts at 0x44 on channel 0 of one switch (two MuxGate<Mux, 0>)
///   GATE_FAIL_FRONT_TWICE   two parts at 0x44 in front of one switch (two MuxFrontGate<Mux>)
#include "Harness.hpp"

#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Mux.hpp>
#include <kvasir/Devices/I2C/chips/Sht3x.hpp>
#include <kvasir/Devices/I2C/chips/Tca9548a.hpp>

namespace {

using MuxDev = Kvasir::Test::Dev<Kvasir::I2C::Chips::Tca9548a>;

/// The same part twice behind one gate type, as two device types (the second names its address).
template<typename Gate>
using First = Kvasir::I2C::Device<Kvasir::Test::FakeBus,
                                  Kvasir::Test::FakeClock,
                                  Kvasir::I2C::Chips::Sht3x,
                                  Kvasir::Engine::DefaultConfig,
                                  Kvasir::NoReset,
                                  Gate>;
template<typename Gate>
using Second = Kvasir::I2C::Device<Kvasir::Test::FakeBus,
                                   Kvasir::Test::FakeClock,
                                   Kvasir::I2C::Chips::Sht3x,
                                   Kvasir::Engine::At<0x44>,
                                   Kvasir::NoReset,
                                   Gate>;

template<typename Gate>
using Twice = Kvasir::I2C::
  Bus<Kvasir::Test::FakeBus, Kvasir::Test::FakeClock, MuxDev, First<Gate>, Second<Gate>>;

// The control: the same two on channels 0 and 3 are legal.
static_assert(sizeof(Kvasir::I2C::Bus<Kvasir::Test::FakeBus,
                                      Kvasir::Test::FakeClock,
                                      MuxDev,
                                      First<Kvasir::I2C::MuxGate<MuxDev,
                                                                 0>>,
                                      Second<Kvasir::I2C::MuxGate<MuxDev,
                                                                  3>>>)
              > 0);

}   // namespace

int main() {
#if defined(GATE_FAIL_SAME_CHANNEL)
    static_assert(sizeof(Twice<Kvasir::I2C::MuxGate<MuxDev, 0>>) > 0);
#endif
#if defined(GATE_FAIL_FRONT_TWICE)
    static_assert(sizeof(Twice<Kvasir::I2C::MuxFrontGate<MuxDev>>) > 0);
#endif
    return 0;
}
