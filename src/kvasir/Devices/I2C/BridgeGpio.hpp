#pragma once

#include "Bridge.hpp"

#include <cstdint>
#include <kvasir/Io/Io.hpp>

namespace Kvasir::I2C {

/// A bridge on an enable pin of the controller, as a base for its description (Bridge.hpp).
/// The pin is an input out of reset, so the first drive() also makes it an output.
///     struct ExtPort : Kvasir::I2C::GpioBridge<HW::Pin::extEnable> {
///         static constexpr auto WhileOff = Kvasir::I2C::WhileOff::unpowered;
///     };
template<typename Pin, BridgePolarity P = BridgePolarity::activeHigh>
struct GpioBridge {
    using Claims = Kvasir::Io::PinClaims<Pin>;

    static void drive(bool active) {
        if(active == (P == BridgePolarity::activeHigh)) {
            apply(set(Pin{}));
        } else {
            apply(clear(Pin{}));
        }
        apply(makeOutput(Pin{}));
    }
};

}   // namespace Kvasir::I2C
