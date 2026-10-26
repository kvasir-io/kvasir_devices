#pragma once

#include <concepts>
#include <cstdint>
// constants.h: the SI units, prefixes and standard_gravity - what the aliases below name (si.h adds unit_symbols)
#include <mp-units/systems/si/constants.h>

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
namespace Kvasir { namespace Units {
    namespace mpu = mp_units;
    namespace si  = mp_units::si;

    namespace detail {
        /// The one factory body: `Units::milliDegC(x)` is x in the unit of the quantity type, x converted to its
        /// representation (int32 or uint32). An object rather than a function template per quantity, so every
        /// factory below is one line; a call compiles as it did. Built with delta<>, which an offset unit (Celsius)
        /// needs and every other unit takes too.
        template<typename Q>
        struct Make {
            template<std::integral T>
            [[nodiscard]] constexpr Q operator()(T value) const {
                return mpu::delta<Q::reference>(static_cast<typename Q::rep>(value));
            }
        };
    }   // namespace detail

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
    /// reader names.
    using Temperature = MilliDegC;

    inline constexpr detail::Make<DegC> degC{};

    inline constexpr detail::Make<DeciDegC> deciDegC{};

    inline constexpr detail::Make<CentiDegC> centiDegC{};

    inline constexpr detail::Make<MilliDegC> milliDegC{};

    /// The one place a temperature leaves the type system, for an application that wants a
    /// float. Generic over the scale, which is the point: one bridge rather than one per
    /// scale.
    template<auto R,
             typename Rep>
    [[nodiscard]] constexpr float celsius(mpu::quantity<R,
                                                        Rep> t) {
        return mpu::value_cast<float>(t).numerical_value_in(si::degree_Celsius);
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

    inline constexpr detail::Make<CentiPercent> centiPercent{};

    inline constexpr detail::Make<MilliPercent> milliPercent{};

    // -- pressure ------------------------------------------------------------------------
    using Pascal = mpu::quantity<si::pascal, std::int32_t>;

    inline constexpr detail::Make<Pascal> pascal{};

    // -- voltage, current, power ---------------------------------------------------------
    using NanoVolt  = mpu::quantity<si::nano<si::volt>, std::int32_t>;
    using MicroVolt = mpu::quantity<si::micro<si::volt>, std::int32_t>;
    using MilliVolt = mpu::quantity<si::milli<si::volt>, std::int32_t>;
    using NanoAmp   = mpu::quantity<si::nano<si::ampere>, std::int32_t>;
    using MicroAmp  = mpu::quantity<si::micro<si::ampere>, std::int32_t>;
    using MilliAmp  = mpu::quantity<si::milli<si::ampere>, std::int32_t>;
    using MicroWatt = mpu::quantity<si::micro<si::watt>, std::uint32_t>;
    using MilliWatt = mpu::quantity<si::milli<si::watt>, std::uint32_t>;

    inline constexpr detail::Make<NanoVolt> nanoVolt{};

    inline constexpr detail::Make<MicroVolt> microVolt{};

    inline constexpr detail::Make<MilliVolt> milliVolt{};

    inline constexpr detail::Make<NanoAmp> nanoAmp{};

    inline constexpr detail::Make<MilliAmp> milliAmp{};

    inline constexpr detail::Make<MicroAmp> microAmp{};

    inline constexpr detail::Make<MicroWatt> microWatt{};

    inline constexpr detail::Make<MilliWatt> milliWatt{};

    // -- illuminance ---------------------------------------------------------------------
    using MicroLux = mpu::quantity<si::micro<si::lux>, std::uint32_t>;
    using MilliLux = mpu::quantity<si::milli<si::lux>, std::uint32_t>;

    inline constexpr detail::Make<MicroLux> microLux{};

    inline constexpr detail::Make<MilliLux> milliLux{};

    // -- acceleration, as the accelerometers report it: a fraction of standard gravity ----
    using MilliG = mpu::quantity<si::milli<si::standard_gravity>, std::int32_t>;
    using MicroG = mpu::quantity<si::micro<si::standard_gravity>, std::int32_t>;

    inline constexpr detail::Make<MilliG> milliG{};

    inline constexpr detail::Make<MicroG> microG{};

    // -- magnetic flux density -----------------------------------------------------------
    using NanoTesla  = mpu::quantity<si::nano<si::tesla>, std::int32_t>;
    using MicroTesla = mpu::quantity<si::micro<si::tesla>, std::int32_t>;

    inline constexpr detail::Make<NanoTesla> nanoTesla{};

    inline constexpr detail::Make<MicroTesla> microTesla{};

    // -- angular rate --------------------------------------------------------------------
    using MicroDegPerSec = mpu::quantity<si::micro<si::degree> / si::second, std::int32_t>;
    using MilliDegPerSec = mpu::quantity<si::milli<si::degree> / si::second, std::int32_t>;

    inline constexpr detail::Make<MicroDegPerSec> microDegPerSec{};

    inline constexpr detail::Make<MilliDegPerSec> milliDegPerSec{};

    // -- resistance ----------------------------------------------------------------------
    using MicroOhm = mpu::quantity<si::micro<si::ohm>, std::uint32_t>;
    using MilliOhm = mpu::quantity<si::milli<si::ohm>, std::uint32_t>;
    using Ohm      = mpu::quantity<si::ohm, std::uint32_t>;

    inline constexpr detail::Make<MicroOhm> microOhm{};

    inline constexpr detail::Make<MilliOhm> milliOhm{};

    inline constexpr detail::Make<Ohm> ohm{};

    // -- length --------------------------------------------------------------------------
    using MilliMetre = mpu::quantity<si::milli<si::metre>, std::int32_t>;

    inline constexpr detail::Make<MilliMetre> milliMetre{};

    // -- speed ---------------------------------------------------------------------------
    using MilliMetrePerSecond = mpu::quantity<si::milli<si::metre> / si::second, std::int32_t>;

    inline constexpr detail::Make<MilliMetrePerSecond> milliMetrePerSecond{};

    // -- plane angle ---------------------------------------------------------------------
    using CentiDegree = mpu::quantity<si::centi<si::degree>, std::int32_t>;
    /// A latitude or longitude: +-180e6 fits the int32 with room to spare.
    using MicroDegree = mpu::quantity<si::micro<si::degree>, std::int32_t>;

    inline constexpr detail::Make<CentiDegree> centiDegree{};

    inline constexpr detail::Make<MicroDegree> microDegree{};

    // -- capacitance ---------------------------------------------------------------------
    using PicoFarad = mpu::quantity<si::pico<si::farad>, std::uint32_t>;

    inline constexpr detail::Make<PicoFarad> picoFarad{};

    // -- frequency -----------------------------------------------------------------------
    using Hertz = mpu::quantity<si::hertz, std::uint32_t>;

    inline constexpr detail::Make<Hertz> hertz{};

    // -- rotational speed, as a fan tachometer reports it --------------------------------
    using Rpm = mpu::quantity<mpu::one / mpu::non_si::minute, std::uint32_t>;

    inline constexpr detail::Make<Rpm> rpm{};

    // -- ratios: whole percent, parts per million ----------------------------------------
    using Percent = mpu::quantity<mpu::percent, std::uint32_t>;
    using Ppm     = mpu::quantity<mpu::parts_per_million, std::uint32_t>;

    inline constexpr detail::Make<Percent> percent{};

    inline constexpr detail::Make<Ppm> ppm{};

    // -- UV index ------------------------------------------------------------------------
    //
    // Dimensionless, but not a ratio of two like quantities, so it is its own unit rather
    // than a scale on `one`: a UV index cannot be added to a percentage by accident.
    inline constexpr struct UvIndex final : mpu::named_unit<"UVI", mpu::one> {
    } uvIndex;

    using MilliUvi = mpu::quantity<si::milli<uvIndex>, std::uint32_t>;

    inline constexpr detail::Make<MilliUvi> milliUvi{};

    static_assert(MilliDegC{centiDegC(2350)} == milliDegC(23500),
                  "centi widens to milli exactly");
    static_assert(MilliDegC{deciDegC(235)} == milliDegC(23500),
                  "and so does deci");
    static_assert(sizeof(MilliDegC) == sizeof(std::int32_t),
                  "a quantity is the integer it holds");
    static_assert(MicroOhm{milliOhm(2)} == microOhm(2000),
                  "a quantity converts to a finer scale implicitly");

}}   // namespace Kvasir::Units
