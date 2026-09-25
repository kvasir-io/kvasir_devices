#pragma once

#include "Address.hpp"

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

/// What a chip description has to say about itself, apart from the engine that runs it: enough
/// for a catalogue of the chips a build knows (Catalogue.hpp) to name what may answer at an
/// address without pulling in Device.hpp.
namespace Kvasir::I2C {

template<typename C>
concept Chip = requires {
    { C::Name } -> std::convertible_to<std::string_view>;
    { C::Address } -> std::same_as<Address7 const&>;
    { C::RegisterBytes } -> std::convertible_to<std::size_t>;
};

/// A description of something that is not a part: an address every part on the segment listens
/// to (the general call, chips/GeneralCall.hpp), written on demand and never expected to answer
/// on its own. A Bus keeps it out of its parts: Bus::Parts, answeringCount(), counts(), and a
/// scan's "not answering" lines.
template<typename C>
concept Broadcast = requires { requires C::Broadcast; };

namespace detail {
    template<typename C>
    concept HasAddresses = requires { std::span<Address7 const>{C::Addresses}; };
}   // namespace detail

}   // namespace Kvasir::I2C
