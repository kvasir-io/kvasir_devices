#pragma once

#include "ResetLine.hpp"
#include "kvasir/Io/Types.hpp"

namespace Kvasir {

/// A ResetLine on a Kvasir GPIO. Declare the pin with makeOutput(): low, so the device is
/// held in reset from the first instruction.
template<typename Pin>
struct GpioReset {
    // A pin nobody configures is a build error (kvasir/StartUp/Resources.hpp).
    using Claims = Kvasir::Io::PinClaims<Pin>;

    static void hold() { apply(clear(Pin{})); }

    static void release() { apply(set(Pin{})); }
};

static_assert(ResetLine<NoReset>);

}   // namespace Kvasir
