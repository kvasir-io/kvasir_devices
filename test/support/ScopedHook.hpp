#pragma once

#include <utility>

namespace Kvasir::Test {

/// Puts a callback into a static hook (FakeBus::respond, a fake pin's onOff, ...) for as long as this object lives,
/// and empties the hook when it goes. A callback that captures the test's locals by reference then never outlives
/// them: before, it stayed installed until the next reset(), and anything that completed a request in between called
/// into a dead stack frame - clang 23's -Wlifetime-safety-dangling-global said so at every such assignment.
///
///     ScopedHook const answering{FakeBus::respond, [&](std::uint8_t a, auto sent, auto recv) { ... }};
///
/// Declare it after the locals the callback captures, so it is destroyed before them.
template<typename Hook>
class ScopedHook {
public:
    template<typename F>
    ScopedHook(Hook& hook,
               F&&   f)
      : hook_{hook} {
        hook_ = std::forward<F>(f);
    }

    ~ScopedHook() { hook_ = Hook{}; }

    ScopedHook(ScopedHook const&)            = delete;
    ScopedHook& operator=(ScopedHook const&) = delete;

private:
    Hook& hook_;
};

template<typename Hook,
         typename F>
ScopedHook(Hook&,
           F&&) -> ScopedHook<Hook>;

}   // namespace Kvasir::Test
