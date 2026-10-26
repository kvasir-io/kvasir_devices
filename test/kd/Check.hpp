#pragma once

#include <concepts>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <source_location>
#include <string>
#include <string_view>
#include <test_harness.hpp>
#include <type_traits>
#include <utility>

/// kvasir_devices' checks, layered on the SDK's one harness (Kvasir_SDK tests/test_harness.hpp, through
/// kvasir::test_support): its `failures` count, its case name (`currentTest`) and its one FAIL line,
/// `FAIL [case] text (file:line)` (failf). What this adds: a case that prints its name, checks that compare
/// quantities and integers of either signedness and print both sides, and `finish()`.
namespace Kvasir::Test {

/// Starts a case: names the FAIL lines that follow (the harness's test()) and prints the name.
inline void testCase(std::string_view name) {
    test(name);
    std::printf("%.*s\n", static_cast<int>(name.size()), name.data());
}

namespace CheckImpl {
    /// An mp_units quantity, by its shape: no include of the library here, so a test of
    /// something that has no quantities does not pay for it.
    template<typename T>
    concept QuantityLike = requires(T const& t) { t.numerical_value_in(T::unit); };

    /// An integer std::cmp_equal accepts: not bool, not a character type.
    template<typename T>
    concept PlainInteger = std::integral<T> && !std::same_as<T, bool> && !std::same_as<T, char>
                        && !std::same_as<T, wchar_t> && !std::same_as<T, char8_t>
                        && !std::same_as<T, char16_t> && !std::same_as<T, char32_t>;

    /// A quantity as the number in its own unit -- the integer the chip decoded, which is what
    /// the checks are written against; anything else as it is.
    template<typename T>
    constexpr auto plain(T const& v) {
        if constexpr(QuantityLike<T>) {
            return v.numerical_value_in(T::unit);
        } else {
            return v;
        }
    }

    /// Equality that does not trip over signedness: a chip's unsigned count against an int
    /// literal.
    template<typename X,
             typename Y>
    constexpr bool same(X const& x,
                        Y const& y) {
        if constexpr(PlainInteger<X> && PlainInteger<Y>) {
            return std::cmp_equal(x, y);
        } else {
            return x == y;
        }
    }

    /// Two quantities a common quantity type holds both of: one dimension at two scales
    /// (CentiDegC and MilliDegC), or the same type twice. The common type is the finer scale,
    /// so the conversion to it is exact.
    template<typename A, typename B>
    concept CommonQuantities = QuantityLike<A> && QuantityLike<B> && requires {
        typename std::common_type_t<A, B>;
        requires QuantityLike<std::common_type_t<A, B>>;
    };

    /// A quantity as the number in the unit of the quantity type `C` (a common type of it).
    template<typename C,
             typename Q>
    constexpr auto numberIn(Q const& q) {
        return C{q}.numerical_value_in(C::unit);
    }

    /// A value as checkEq shows it in its FAIL line.
    template<typename T>
    std::string show(T const& v) {
        if constexpr(std::same_as<T, bool>) {
            return v ? "true" : "false";
        } else if constexpr(std::is_enum_v<T>) {
            return std::format("{}", static_cast<long long>(std::to_underlying(v)));
        } else if constexpr(std::signed_integral<T>) {
            return std::format("{}", static_cast<long long>(v));
        } else if constexpr(std::unsigned_integral<T>) {
            return std::format("{}", static_cast<unsigned long long>(v));
        } else if constexpr(std::floating_point<T>) {
            return std::format("{:g}", static_cast<double>(v));
        } else if constexpr(QuantityLike<T>) {
            return show(v.numerical_value_in(T::unit));
        } else {
            return "?";
        }
    }
}   // namespace CheckImpl

inline void check(bool                 ok,
                  std::string_view     what,
                  std::source_location at = std::source_location::current()) {
    if(!ok) { failf(at, "{}", what); }
}

/// What checkEq compares, usable in a static_assert: two quantities in their common unit
/// (so 25.08 degC as CentiDegC equals 25080 as MilliDegC), a quantity against a number as the
/// number in the quantity's own unit (the integer the chip decoded), integers regardless of
/// signedness, anything else with ==.
template<typename A,
         typename B>
[[nodiscard]] constexpr bool equal(A const& a,
                                   B const& b) {
    if constexpr(CheckImpl::CommonQuantities<A, B>) {
        return CheckImpl::same(CheckImpl::numberIn<std::common_type_t<A, B>>(a),
                               CheckImpl::numberIn<std::common_type_t<A, B>>(b));
    } else {
        return CheckImpl::same(CheckImpl::plain(a), CheckImpl::plain(b));
    }
}

/// `a == b` as equal() has it, both sides printed on failure; two quantities of different
/// scales are both printed in their common unit.
template<typename A,
         typename B>
void checkEq(A const&             a,
             B const&             b,
             std::string_view     what,
             std::source_location at = std::source_location::current()) {
    if(!equal(a, b)) {
        if constexpr(CheckImpl::CommonQuantities<A, B>) {
            using C = std::common_type_t<A, B>;
            failf(at,
                  "{}: {} != {}",
                  what,
                  CheckImpl::show(CheckImpl::numberIn<C>(a)),
                  CheckImpl::show(CheckImpl::numberIn<C>(b)));
        } else {
            failf(at, "{}: {} != {}", what, CheckImpl::show(a), CheckImpl::show(b));
        }
    }
}

template<typename A>
void checkNear(A const&             a,
               double               b,
               double               tol,
               char const*          what,
               std::source_location at = std::source_location::current()) {
    auto const x = static_cast<double>(CheckImpl::plain(a));
    auto const d = x > b ? x - b : b - x;
    if(d > tol) { failf(at, "{}: {:g} vs {:g} (tol {:g})", what, x, b, tol); }
}

/// The harness's report(), with a line when everything passed.
inline int finish() {
    if(failures.load() == 0) { std::printf("all checks passed\n"); }
    std::fflush(stdout);
    return report();
}

}   // namespace Kvasir::Test
