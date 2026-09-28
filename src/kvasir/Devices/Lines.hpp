#pragma once

#include <concepts>

/// An SPI panel's D/C and BUSY lines as policies. SDK-free.
namespace Kvasir {

template<typename L>
concept DataCommandLine = requires {
    L::command();
    L::data();
};

/// `busy()`: true while the controller must not be sent anything.
template<typename L>
concept BusyLine = requires {
    { L::busy() } -> std::convertible_to<bool>;
};

}   // namespace Kvasir
