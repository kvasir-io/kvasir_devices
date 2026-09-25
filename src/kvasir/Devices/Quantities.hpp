#pragma once

#include <concepts>
#include <cstdint>
#include <mp-units/systems/si.h>

/// What a chip description reports, with the scale in the type rather than in the member
/// name. One conversion then serves every chip, and generic code over "a temperature sensor"
/// is possible.
///
/// These are `std::chrono::duration`'s design applied to physics, which is what mp-units is:
/// a scale and a representation in the type, implicit conversion where it is exact and
/// widening, `value_cast` where it is not.
///
/// The representation stays integral. A description decodes into `std::int32_t` at the chip's
/// own scale, so `sizeof(Sample)` is the int it holds, the decode arithmetic is integer, and
/// no soft-float reaches a Cortex-M0+ target. Floating point belongs to the application,
/// which converts once at the edge.
///
/// A description that measures nothing scaled (a GPIO expander, a touch controller, an
/// EEPROM) does not include this header and does not pay for it.
///
/// Bus-agnostic: the I2C descriptions and the SPI drivers report through the same types.
/// I2C/Quantities.hpp re-exports everything here into Kvasir::I2C, where the descriptions
/// name the aliases unqualified.
namespace Kvasir {

namespace Units {
    namespace mpu = mp_units;
    namespace si  = mp_units::si;

    // -- temperature ---------------------------------------------------------------------
    //
    // Celsius is an offset unit in mp-units, so a value is built with delta<> (a difference
    // from the ice point, which is what every one of these chips reports) rather than by
    // multiplication. The four scales are the ones the chips report in.
    using DegC      = mpu::quantity<si::degree_Celsius, std::int32_t>;
    using DeciDegC  = mpu::quantity<si::deci<si::degree_Celsius>, std::int32_t>;
    using CentiDegC = mpu::quantity<si::centi<si::degree_Celsius>, std::int32_t>;
    using MilliDegC = mpu::quantity<si::milli<si::degree_Celsius>, std::int32_t>;

    /// The canonical one: everything converts to it without loss, so this is what a generic
    /// reader and the TemperatureSource concept below name.
    using Temperature = MilliDegC;

    template<std::integral T>
    [[nodiscard]] constexpr DegC degC(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return mpu::delta<si::degree_Celsius>(x);
    }

    template<std::integral T>
    [[nodiscard]] constexpr DeciDegC deciDegC(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return mpu::delta<si::deci<si::degree_Celsius>>(x);
    }

    template<std::integral T>
    [[nodiscard]] constexpr CentiDegC centiDegC(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return mpu::delta<si::centi<si::degree_Celsius>>(x);
    }

    template<std::integral T>
    [[nodiscard]] constexpr MilliDegC milliDegC(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return mpu::delta<si::milli<si::degree_Celsius>>(x);
    }

    /// The one place a temperature leaves the type system, for an application that wants a
    /// float. Generic over the scale, which is the point: one bridge rather than one per
    /// scale.
    template<auto R,
             typename Rep>
    [[nodiscard]] constexpr float celsius(mpu::quantity<R,
                                                        Rep> t) {
        return mpu::value_cast<float>(t).numerical_value_in(si::degree_Celsius);
    }

    /// The same quantity with a float representation, in the same unit: the one place an
    /// application converts before it divides, filters or prints, instead of spelling out
    /// `value_cast<float>` at every use. `toFloat(sample.pressure).numerical_value_in(si::pascal)`.
    template<auto R,
             typename Rep>
    [[nodiscard]] constexpr mpu::quantity<R,
                                          float>
    toFloat(mpu::quantity<R,
                          Rep> q) {
        return mpu::value_cast<float>(q);
    }

    /// The integer a quantity holds, in its own unit: what a description writes to a register
    /// once the scale has been fixed by the quantity's type.
    template<auto R,
             typename Rep>
    [[nodiscard]] constexpr Rep value(mpu::quantity<R,
                                                    Rep> q) {
        return q.numerical_value_in(q.unit);
    }

    /// The same bridge for any quantity and any unit it converts to: `in(sample.pressure,
    /// si::hecto<si::pascal>)`, `in(sample.humidity, mpu::percent)`.
    template<auto R,
             typename Rep,
             mpu::Unit U>
    [[nodiscard]] constexpr float in(mpu::quantity<R,
                                                   Rep> q,
                                     U                  unit) {
        return mpu::value_cast<float>(q).numerical_value_in(unit);
    }

    // -- relative humidity ---------------------------------------------------------------
    using CentiPercent = mpu::quantity<si::centi<mpu::percent>, std::uint32_t>;
    using MilliPercent = mpu::quantity<si::milli<mpu::percent>, std::uint32_t>;
    using Humidity     = MilliPercent;

    template<std::integral T>
    [[nodiscard]] constexpr CentiPercent centiPercent(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::centi<mpu::percent>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MilliPercent milliPercent(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::milli<mpu::percent>;
    }

    // -- pressure ------------------------------------------------------------------------
    using Pascal = mpu::quantity<si::pascal, std::int32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr Pascal pascal(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::pascal;
    }

    // -- voltage, current, power ---------------------------------------------------------
    using NanoVolt  = mpu::quantity<si::nano<si::volt>, std::int32_t>;
    using MicroVolt = mpu::quantity<si::micro<si::volt>, std::int32_t>;
    using MilliVolt = mpu::quantity<si::milli<si::volt>, std::int32_t>;
    using NanoAmp   = mpu::quantity<si::nano<si::ampere>, std::int32_t>;
    using MicroAmp  = mpu::quantity<si::micro<si::ampere>, std::int32_t>;
    using MilliAmp  = mpu::quantity<si::milli<si::ampere>, std::int32_t>;
    using MicroWatt = mpu::quantity<si::micro<si::watt>, std::uint32_t>;
    using MilliWatt = mpu::quantity<si::milli<si::watt>, std::uint32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr NanoVolt nanoVolt(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::nano<si::volt>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MicroVolt microVolt(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::micro<si::volt>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MilliVolt milliVolt(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::milli<si::volt>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr NanoAmp nanoAmp(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::nano<si::ampere>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MilliAmp milliAmp(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::milli<si::ampere>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MicroAmp microAmp(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::micro<si::ampere>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MicroWatt microWatt(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::micro<si::watt>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MilliWatt milliWatt(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::milli<si::watt>;
    }

    // -- illuminance ---------------------------------------------------------------------
    using MicroLux = mpu::quantity<si::micro<si::lux>, std::uint32_t>;
    using MilliLux = mpu::quantity<si::milli<si::lux>, std::uint32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr MicroLux microLux(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::micro<si::lux>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MilliLux milliLux(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::milli<si::lux>;
    }

    // -- acceleration, as the accelerometers report it: a fraction of standard gravity ----
    using MilliG = mpu::quantity<si::milli<si::standard_gravity>, std::int32_t>;
    using MicroG = mpu::quantity<si::micro<si::standard_gravity>, std::int32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr MilliG milliG(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::milli<si::standard_gravity>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MicroG microG(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::micro<si::standard_gravity>;
    }

    // -- magnetic flux density -----------------------------------------------------------
    using NanoTesla  = mpu::quantity<si::nano<si::tesla>, std::int32_t>;
    using MicroTesla = mpu::quantity<si::micro<si::tesla>, std::int32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr NanoTesla nanoTesla(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::nano<si::tesla>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MicroTesla microTesla(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::micro<si::tesla>;
    }

    // -- angular rate --------------------------------------------------------------------
    using MicroDegPerSec = mpu::quantity<si::micro<si::degree> / si::second, std::int32_t>;
    using MilliDegPerSec = mpu::quantity<si::milli<si::degree> / si::second, std::int32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr MicroDegPerSec microDegPerSec(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::micro<si::degree> / si::second;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MilliDegPerSec milliDegPerSec(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::milli<si::degree> / si::second;
    }

    // -- resistance ----------------------------------------------------------------------
    using MicroOhm = mpu::quantity<si::micro<si::ohm>, std::uint32_t>;
    using MilliOhm = mpu::quantity<si::milli<si::ohm>, std::uint32_t>;
    using Ohm      = mpu::quantity<si::ohm, std::uint32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr MicroOhm microOhm(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::micro<si::ohm>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MilliOhm milliOhm(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::milli<si::ohm>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr Ohm ohm(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::ohm;
    }

    // -- length --------------------------------------------------------------------------
    using MilliMetre = mpu::quantity<si::milli<si::metre>, std::int32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr MilliMetre milliMetre(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::milli<si::metre>;
    }

    // -- speed ---------------------------------------------------------------------------
    using MilliMetrePerSecond = mpu::quantity<si::milli<si::metre> / si::second, std::int32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr MilliMetrePerSecond milliMetrePerSecond(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::milli<si::metre> / si::second;
    }

    // -- plane angle ---------------------------------------------------------------------
    using CentiDegree = mpu::quantity<si::centi<si::degree>, std::int32_t>;
    /// A latitude or longitude: +-180e6 fits the int32 with room to spare.
    using MicroDegree = mpu::quantity<si::micro<si::degree>, std::int32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr CentiDegree centiDegree(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::centi<si::degree>;
    }

    template<std::integral T>
    [[nodiscard]] constexpr MicroDegree microDegree(T value) {
        auto const x = static_cast<std::int32_t>(value);
        return x * si::micro<si::degree>;
    }

    // -- capacitance ---------------------------------------------------------------------
    using PicoFarad = mpu::quantity<si::pico<si::farad>, std::uint32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr PicoFarad picoFarad(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::pico<si::farad>;
    }

    // -- frequency -----------------------------------------------------------------------
    using Hertz = mpu::quantity<si::hertz, std::uint32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr Hertz hertz(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::hertz;
    }

    // -- rotational speed, as a fan tachometer reports it --------------------------------
    using Rpm = mpu::quantity<mpu::one / mpu::non_si::minute, std::uint32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr Rpm rpm(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * (mpu::one / mpu::non_si::minute);
    }

    // -- ratios: whole percent, parts per million ----------------------------------------
    using Percent = mpu::quantity<mpu::percent, std::uint32_t>;
    using Ppm     = mpu::quantity<mpu::parts_per_million, std::uint32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr Percent percent(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * mpu::percent;
    }

    template<std::integral T>
    [[nodiscard]] constexpr Ppm ppm(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * mpu::parts_per_million;
    }

    // -- UV index ------------------------------------------------------------------------
    //
    // Dimensionless, but not a ratio of two like quantities, so it is its own unit rather
    // than a scale on `one`: a UV index cannot be added to a percentage by accident.
    inline constexpr struct uv_index final : mpu::named_unit<"UVI", mpu::one> {
    } uv_index;

    using MilliUvi = mpu::quantity<si::milli<uv_index>, std::uint32_t>;

    template<std::integral T>
    [[nodiscard]] constexpr MilliUvi milliUvi(T value) {
        auto const x = static_cast<std::uint32_t>(value);
        return x * si::milli<uv_index>;
    }

    static_assert(MilliDegC{centiDegC(2350)} == milliDegC(23500),
                  "centi widens to milli exactly");
    static_assert(MilliDegC{deciDegC(235)} == milliDegC(23500),
                  "and so does deci");
    static_assert(sizeof(MilliDegC) == sizeof(std::int32_t),
                  "a quantity is the integer it holds");
    static_assert(MicroOhm{milliOhm(2)} == microOhm(2000),
                  "a quantity converts to a finer scale implicitly");
    static_assert(toFloat(milliDegC(23500)).numerical_value_in(si::milli<si::degree_Celsius>)
                    == 23500.0f,
                  "toFloat keeps the unit and changes the representation");

}   // namespace Units

/// A device whose one read group reports a temperature: what a display or a log formatter
/// can be written against once, instead of once per chip.
template<typename D>
concept TemperatureSource = requires(D const& d) {
    { d.latest().temperature } -> std::convertible_to<Units::Temperature>;
};

}   // namespace Kvasir
