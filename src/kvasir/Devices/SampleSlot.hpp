#pragma once

#include <cstdint>

/// The last reading of a driver that is no engine device, and how many there were: what a reader of
/// Max31820, Max31865, ADS131M0x and the LTC2400 reader asks for, written once. `latest()` is the last
/// published reading (a default one before the first), `seq()` steps with every publication (and wraps),
/// `fresh(seen)` is true once per new reading for the reader that keeps `seen`. Whether a reading is
/// `valid()` stays each driver's (the part answering, a fault since, a reading at all).
///
///     Kvasir::SampleSlot<Sample> slot_{};
///     slot_.publish({temperatureFor(code), code});   // where a reading is complete
///     [[nodiscard]] Sample const& latest() const { return slot_.latest(); }
///
/// Std only, as the engine's description headers.
namespace Kvasir {

template<typename Sample>
class SampleSlot {
public:
    /// User-provided on purpose (defaulted below the class, as I2C::Device's): a driver's `SampleSlot<S> slot_{}` is
    /// then member-initialized only. With the implicit constructor that braced member is value-initialized - a zero
    /// fill (an `__aeabi_memclr` call) before the member initializers, which cost a firmware with a MAX31865 144 B of
    /// `main` and moved its inlining.
    constexpr SampleSlot();

    [[nodiscard]] constexpr Sample const& latest() const { return latest_; }

    /// Steps with every published reading.
    [[nodiscard]] constexpr std::uint32_t seq() const { return samples_; }

    /// Readings published over the driver's life (the same count as seq()).
    [[nodiscard]] constexpr std::uint32_t samples() const { return samples_; }

    /// True once per new reading for the reader that keeps `seen`.
    [[nodiscard]] constexpr bool fresh(std::uint32_t& seen) const {
        if(seen == samples_) { return false; }
        seen = samples_;
        return true;
    }

    /// A complete reading: the one latest() returns from now on.
    constexpr void publish(Sample const& sample) {
        latest_ = sample;
        ++samples_;
    }

private:
    Sample        latest_{};
    std::uint32_t samples_{};
};

template<typename Sample>
constexpr SampleSlot<Sample>::SampleSlot() = default;

}   // namespace Kvasir
