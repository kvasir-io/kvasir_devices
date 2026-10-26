#pragma once

// The one callback rule of the queued buses (I2C/Controller/QueueCore.hpp, SPI/QueueCore.hpp): every callback runs
// through Invoker::invoke(), which marks it running, and a core starts nothing on the wire while inCallback() - what a
// callback submits starts after it has returned. Without the mark a callback run inside the core (a queued cancel,
// the deadline sweep, the expired-at-start loop, a completion) that submits could start a request the running loop
// has still to reach, or start a second one over the first.
//
// invoke() calls the callback where it is: a caller that walks the queue copies the callback first (a callback that
// resets the bus and submits again can push into the slot being walked), the request on the wire and a request
// already popped are called in place. An object, owned by its bus core (an `inline static` member of that core).
namespace Kvasir::Bus {
class Invoker {
public:
    [[nodiscard]] bool inCallback() const { return inCallback_; }

    template<typename Callback,
             typename Result>
    void invoke(Callback const& cb,
                Result          result) {
        if(!cb) { return; }
        auto const outer = inCallback_;
        inCallback_      = true;
        cb(result);
        inCallback_ = outer;
    }

private:
    bool inCallback_{};
};
}   // namespace Kvasir::Bus
