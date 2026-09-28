#pragma once

/// The pins of the SPI host tests, as levels by id, found by ADL from the drivers' unqualified
/// `apply(set(Cs{}))` / `read(Drdy{})`. The master is FakeQueuedSpi.hpp.
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <utility>

namespace Kvasir::Test::Spi {

/// Every pin idles high: CS released, DRDY and RVS not asserted.
struct Pins {
    static inline std::array<bool, 8> level{};
    /// Rising edges per pin: a model sees CS go up between two back-to-back frames.
    static inline std::array<std::uint32_t, 8> rises{};

    static void reset() {
        level.fill(true);
        rises.fill(0);
    }
};

template<std::size_t Id>
struct Pin {
    static constexpr std::size_t id = Id;
};

using Cs   = Pin<0>;
using Drdy = Pin<1>;
using Rvs  = Pin<2>;
using Rst  = Pin<3>;

struct PinOp {
    enum class Kind : std::uint8_t { none, set, clear, read };
    Kind        kind{Kind::none};
    std::size_t id{};
};

template<std::size_t Id>
constexpr PinOp makeInput(Pin<Id>) {
    return {};
}

template<std::size_t Id>
constexpr PinOp makeOutput(Pin<Id>) {
    return {};
}

template<std::size_t Id>
constexpr PinOp set(Pin<Id>) {
    return {PinOp::Kind::set, Id};
}

template<std::size_t Id>
constexpr PinOp clear(Pin<Id>) {
    return {PinOp::Kind::clear, Id};
}

template<std::size_t Id>
constexpr PinOp read(Pin<Id>) {
    return {PinOp::Kind::read, Id};
}

template<std::same_as<PinOp>... Ops>
bool apply(PinOp first,
           Ops... rest) {
    bool level = false;
    for(PinOp const op : {first, rest...}) {
        switch(op.kind) {
        case PinOp::Kind::none: break;
        case PinOp::Kind::set:
            if(!Pins::level[op.id]) { ++Pins::rises[op.id]; }
            Pins::level[op.id] = true;
            break;
        case PinOp::Kind::clear: Pins::level[op.id] = false; break;
        case PinOp::Kind::read:  level = Pins::level[op.id]; break;
        }
    }
    return level;
}

}   // namespace Kvasir::Test::Spi
