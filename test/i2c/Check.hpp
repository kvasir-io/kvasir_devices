#pragma once

#include <concepts>
#include <cstdio>
#include <cstdlib>
#include <source_location>
#include <type_traits>
#include <utility>

/// The smallest test harness: a named case, checks that print on failure with the file and line
/// they came from, a count.
namespace Kvasir::Test {

inline int         failures{};
inline char const* currentCase{"?"};

inline void testCase(char const* name) {
    currentCase = name;
    std::printf("%s\n", name);
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

    template<typename T>
    void print(T const& v) {
        if constexpr(std::same_as<T, bool>) {
            std::fputs(v ? "true" : "false", stdout);
        } else if constexpr(std::is_enum_v<T>) {
            std::printf("%lld", static_cast<long long>(std::to_underlying(v)));
        } else if constexpr(std::signed_integral<T>) {
            std::printf("%lld", static_cast<long long>(v));
        } else if constexpr(std::unsigned_integral<T>) {
            std::printf("%llu", static_cast<unsigned long long>(v));
        } else if constexpr(std::floating_point<T>) {
            std::printf("%g", static_cast<double>(v));
        } else if constexpr(QuantityLike<T>) {
            print(v.numerical_value_in(T::unit));
        } else {
            std::fputs("?", stdout);
        }
    }

    inline void where(std::source_location const& at) {
        std::printf("  FAIL [%s] %s:%u: ", currentCase, at.file_name(), at.line());
    }
}   // namespace CheckImpl

inline void check(bool                 ok,
                  char const*          what,
                  std::source_location at = std::source_location::current()) {
    if(!ok) {
        CheckImpl::where(at);
        std::printf("%s\n", what);
        ++failures;
    }
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
             char const*          what,
             std::source_location at = std::source_location::current()) {
    if(!equal(a, b)) {
        CheckImpl::where(at);
        std::printf("%s: ", what);
        if constexpr(CheckImpl::CommonQuantities<A, B>) {
            CheckImpl::print(CheckImpl::numberIn<std::common_type_t<A, B>>(a));
            std::fputs(" != ", stdout);
            CheckImpl::print(CheckImpl::numberIn<std::common_type_t<A, B>>(b));
        } else {
            CheckImpl::print(a);
            std::fputs(" != ", stdout);
            CheckImpl::print(b);
        }
        std::fputs("\n", stdout);
        ++failures;
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
    if(d > tol) {
        CheckImpl::where(at);
        std::printf("%s: %g vs %g (tol %g)\n", what, x, b, tol);
        ++failures;
    }
}

inline int finish() {
    if(failures == 0) {
        std::printf("all checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) FAILED\n", failures);
    return EXIT_FAILURE;
}

}   // namespace Kvasir::Test
