#pragma once
// The depth-counted interrupt mask of a queued bus (I2C/Controller/QueueCore.hpp, SPI/QueueCore.hpp): thread code
// that shares state with the bus's interrupt runs between lock() and unlock(), and a callback run inside such a
// section may open another (a submit() from a cancel's callback). Only the outermost lock() calls Hw::mask() and only
// the outermost unlock() calls Hw::unmask(): a nested mask must not run, because a policy may save the state it
// replaces into ONE place (SPI: `irqs_ = IrqState::saveAndMask()`, PRIMASK), and a second save would overwrite the
// first and leave interrupts off after the outermost unmask. So a Hw policy never enables its line inside a masked
// section; only the outermost unlock() does (I2C/Controller/QueueHw.hpp, SPI/QueueHw.hpp).
//
// The depth goes up before the mask and down before the unmask, with a compiler fence between: an interrupt taken
// before the outermost mask has taken effect sees a section open and leaves the line alone, and one between the read
// and the write of the depth runs a balanced pair of its own. An object, owned by its bus core (an `inline static`
// member of that core): two buses of one shape are two cores, so two depths.
#include <atomic>
#include <cstdint>

namespace Kvasir::Bus {
template<typename Hw>
class MaskDepth {
public:
    void lock() {
        if(depth_++ == 0) {
            std::atomic_signal_fence(std::memory_order_seq_cst);
            Hw::mask();
        }
    }

    void unlock() {
        std::atomic_signal_fence(std::memory_order_seq_cst);
        if(--depth_ == 0) { Hw::unmask(); }
    }

    /// Sections open now (0: unmasked).
    [[nodiscard]] std::uint8_t depth() const { return depth_; }

    /// lock() for a scope.
    class [[nodiscard]] Guard {
    public:
        explicit Guard(MaskDepth& m) : m_{m} { m_.lock(); }

        ~Guard() { m_.unlock(); }

        Guard(Guard const&)            = delete;
        Guard& operator=(Guard const&) = delete;

    private:
        MaskDepth& m_;
    };

private:
    std::uint8_t depth_{};
};
}   // namespace Kvasir::Bus
