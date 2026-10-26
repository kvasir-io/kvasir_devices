#pragma once

#include <kd/Check.hpp>

namespace Kvasir::Test {

/// A named case: prints its name (testCase), counts the checks that fail while it lives, and on leaving calls `onFail`
/// once if any did - not when an EARLIER case failed, which is what `if(failures != 0) { dump(); }` at the end of
/// every case did with the process-wide counter.
class Case {
public:
    explicit Case(char const* name,
                  void (*onFail)() = nullptr)
      : before_{failures}
      , onFail_{onFail} {
        testCase(name);
    }

    ~Case() {
        if(failed() && onFail_ != nullptr) { onFail_(); }
    }

    /// Whether a check failed since this case (or its last next()) began.
    [[nodiscard]] bool failed() const { return failures != before_; }

    /// A sub-step that continues this case's state (no clean-up, no new failure scope): what a second testCase()
    /// inside one function did when it built on the one before.
    void step(char const* name) const { testCase(name); }

    Case(Case const&)            = delete;
    Case& operator=(Case const&) = delete;

protected:
    /// Starts a new failure scope, for a case that ends one part and begins the next in the same function.
    void restart(char const* name) {
        testCase(name);
        before_ = failures;
    }

private:
    int before_;
    void (*onFail_)();
};

}   // namespace Kvasir::Test
