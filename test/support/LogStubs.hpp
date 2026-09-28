#pragma once

/// The uc_log macros the device drivers and their base use without including uc_log
/// (the target TU has it). A host test counts them instead. Include before the driver.
///
/// With remote_fmt on the include path every line is also checked as on the target: the format
/// string against its arguments, and a formatter for every argument (a `char const*` has none).
#if __has_include(<remote_fmt/remote_fmt.hpp>)
    #include <remote_fmt/remote_fmt.hpp>
    #include <type_traits>
    #define KVASIR_TEST_LOG_CHECKED 1
#else
    #define KVASIR_TEST_LOG_CHECKED 0
#endif

namespace Kvasir::Test::Log {
inline int warnings{};
inline int infos{};
inline int debugs{};
inline int errors{};

inline void reset() { warnings = infos = debugs = errors = 0; }

#if KVASIR_TEST_LOG_CHECKED
template<typename T>
concept Formattable = requires { sizeof(remote_fmt::formatter<std::remove_cvref_t<T>>); };

template<typename Fmt,
         typename... Args>
int count(int& counter,
          Fmt,
          Args const&...) {
    static_assert((Formattable<Args> && ...),
                  "a log argument remote_fmt has no formatter for: it does not compile on the "
                  "target (char const*: use std::string_view)");
    remote_fmt::checkFormatString<Args const&...>(Fmt{});
    return ++counter;
}
#endif
}   // namespace Kvasir::Test::Log

#if KVASIR_TEST_LOG_CHECKED
    #define KVASIR_TEST_LOG(counter, fmt, ...)                              \
        ::Kvasir::Test::Log::count(::Kvasir::Test::Log::counter,            \
                                   SC_LIFT(fmt) __VA_OPT__(, ) __VA_ARGS__)
#else
    #define KVASIR_TEST_LOG(counter, fmt, ...) (++::Kvasir::Test::Log::counter)
#endif

#define UC_LOG_W(...) KVASIR_TEST_LOG(warnings, __VA_ARGS__)
#define UC_LOG_I(...) KVASIR_TEST_LOG(infos, __VA_ARGS__)
#define UC_LOG_D(...) KVASIR_TEST_LOG(debugs, __VA_ARGS__)
#define UC_LOG_E(...) KVASIR_TEST_LOG(errors, __VA_ARGS__)
#define KVASIR_LOG_LIMITED(decision, LOG, ...)               \
    do {                                                     \
        if(auto const d_ = (decision)) { LOG(__VA_ARGS__); } \
    } while(false)
