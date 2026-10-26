#pragma once
// What a driver on the queued SPI master (QueueCore.hpp, the chips' SPIQueued / Sercom_SPIQueued) uses: its chip
// select as Lines, and one frame in flight.
//
// Contexts: ChipSelect's functions run where the master calls them (the completion interrupt on the RP, turn()
// on the SAM). OneFrame::complete() runs in the frame's callback, any context; start(), begin(), refuse(), take()
// and running() are the loop's (one reader: take() is called by the loop only).
#include "QueueCore.hpp"

#include <atomic>
#include <cstdint>

namespace Kvasir::SPI {

namespace detail {
    /// The pin's two functions, apart from ChipSelect's `Deselect`: `select` is one function per pin.
    template<typename Cs>
    struct CsPin {
        static void select() { apply(clear(Cs{})); }

        static void deselect() { apply(set(Cs{})); }
    };
}   // namespace detail

/// A GPIO chip select, active low. `select` is also the device's identity for a hold (QueueCore): one pin,
/// one holder. Deselect = false: CS stays low after the frame (LTC2400 shows EOC on SDO, ADS8675's CS
/// rise starts a conversion), `lines.deselect` is null.
template<typename Cs, bool Deselect = true>
struct ChipSelect {
    static constexpr void (*select)()   = &detail::CsPin<Cs>::select;
    static constexpr void (*deselect)() = &detail::CsPin<Cs>::deselect;

    /// CS high before anything else: a part that sees CS low early answers garbage. For constructors.
    static void release() { apply(set(Cs{})); }

    static constexpr Lines lines{select, Deselect ? deselect : nullptr, nullptr, nullptr};
};

/// One frame in flight between the loop and the completion (an interrupt on the RP, turn() on the SAM).
/// Loads and stores only, no read-modify-write: safe on the Cortex-M0+ without the atomic shims.
///
/// - start(request): false = a frame is still running, or the master refused it (then no callback comes); the
///   state is left as it was in both cases. A driver that counts a refusal as a failed frame calls refuse().
/// - take(): `ok` / `failed` once, then `idle`; `running` while the frame is on the wire.
/// - `running` is stored before the submit: the completion may run before submit() returns.
template<typename Master>
class OneFrame {
public:
    enum class Poll : std::uint8_t { idle, running, ok, failed };

    template<typename Request>
    bool start(Request r) {
        if(!begin()) { return false; }
        r.callback = [this](auto res) { complete(res == decltype(res)::ok); };
        if(!Master::submit(r)) {
            state_.store(Poll::idle, std::memory_order_relaxed);
            return false;
        }
        return true;
    }

    /// For a callback with a driver add-on (Ltc2400 reads SDO first): begin(), then the driver's own submit with
    /// a callback that ends in complete(); on a refused submit the driver calls refuse() or abandon().
    bool begin() {
        if(state_.load(std::memory_order_acquire) == Poll::running) { return false; }
        state_.store(Poll::running, std::memory_order_relaxed);
        return true;
    }

    /// The frame's end, from its callback (any context). cancelled / timedOut count as failed.
    void complete(bool succeeded) {
        state_.store(succeeded ? Poll::ok : Poll::failed, std::memory_order_release);
    }

    /// A refused submit as a failed frame: the next take() says `failed`.
    void refuse() { state_.store(Poll::failed, std::memory_order_release); }

    /// A refused submit after begin(), counted as nothing: back to idle.
    void abandon() { state_.store(Poll::idle, std::memory_order_relaxed); }

    Poll take() {
        auto const s = state_.load(std::memory_order_acquire);
        if(s == Poll::ok || s == Poll::failed) {
            state_.store(Poll::idle, std::memory_order_relaxed);
        }
        return s;
    }

    [[nodiscard]] bool running() const {
        return state_.load(std::memory_order_acquire) == Poll::running;
    }

    /// Nothing out and no verdict waiting for take().
    [[nodiscard]] bool idle() const { return state_.load(std::memory_order_acquire) == Poll::idle; }

private:
    std::atomic<Poll> state_{Poll::idle};
};
}   // namespace Kvasir::SPI
