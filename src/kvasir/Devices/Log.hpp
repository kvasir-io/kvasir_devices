#pragma once

/// The log lines the drivers write. On a target the uc_log macros come from uc_log itself,
/// and KVASIR_LOG_LIMITED from the SDK's LogLimited.hpp (with its no-op fallback); a host test defines its own before
/// including a driver (kvasir_devices test/kd/LogStubs.hpp). A host build that defines
/// neither gets silent no-ops rather than a parse error deep inside a driver.
#if __has_include("uc_log/uc_log.hpp")
    #include "uc_log/uc_log.hpp"
#endif

#include "kvasir/Log/LogLimited.hpp"
#include "kvasir/Log/RateLimiter.hpp"

namespace Kvasir::detail {
/// The no-op lines' arguments: named only inside `sizeof`, never called or defined. They stay
/// type-checked, and two lines that differ only in their text stay two different statements.
template<typename... Args>
int unloggedLine(Args const&...);
}   // namespace Kvasir::detail

#ifndef UC_LOG_W
    #define UC_LOG_W(...) static_cast<void>(sizeof(::Kvasir::detail::unloggedLine(__VA_ARGS__)))
#endif
#ifndef UC_LOG_I
    #define UC_LOG_I(...) static_cast<void>(sizeof(::Kvasir::detail::unloggedLine(__VA_ARGS__)))
#endif
#ifndef UC_LOG_D
    #define UC_LOG_D(...) static_cast<void>(sizeof(::Kvasir::detail::unloggedLine(__VA_ARGS__)))
#endif
#ifndef UC_LOG_E
    #define UC_LOG_E(...) static_cast<void>(sizeof(::Kvasir::detail::unloggedLine(__VA_ARGS__)))
#endif
// uc_log's scoped settings (uc_log/LogEnv.hpp); nothing where uc_log is not there. The drivers'
// log modules need no declaration: uc_log derives each line's from the scope of its function
// (Kvasir::I2C::Bus<...>::run -> "i2c.bus", Kvasir::Engine::detail::logUp -> "engine").
#ifndef UC_LOG_SCOPE_MODULE
    #define UC_LOG_SCOPE_MODULE(name) static_assert(true)
#endif

/// A log line several call sites share, so the line's code exists once and a site is its
/// arguments and a call (Engine/EngineLog.hpp: `Device<>` is instantiated per device, and the
/// "up" line alone was 76 bytes x 55 of them in one firmware).
///
/// `noinline` only while logging is compiled in: without USE_UC_LOG the bodies are empty and
/// the attribute would leave a call to an empty function at every site instead of nothing.
#ifdef USE_UC_LOG
    #define KVASIR_LOG_SHARED [[gnu::noinline]] inline
#else
    #define KVASIR_LOG_SHARED inline
#endif
