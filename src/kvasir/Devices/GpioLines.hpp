#pragma once

#include "Lines.hpp"
#include "kvasir/Io/Types.hpp"

namespace Kvasir {

/// Declare the pin with makeOutput().
template<typename Pin>
struct GpioDataCommand {
    using Claims = Kvasir::Io::PinClaims<Pin>;

    static void command() { apply(clear(Pin{})); }

    static void data() { apply(set(Pin{})); }
};

/// Declare the pin with makeInput(). The SSD16xx drive it high while busy.
template<typename Pin, bool ActiveHigh = true>
struct GpioBusy {
    using Claims = Kvasir::Io::PinClaims<Pin>;

    static bool busy() { return (apply(read(Pin{})) != 0) == ActiveHigh; }
};

}   // namespace Kvasir
