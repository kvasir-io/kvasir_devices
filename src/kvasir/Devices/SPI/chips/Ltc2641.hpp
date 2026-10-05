#pragma once
// Linear/ADI LTC2641 (and LTC2642) 16-/14-/12-bit voltage-output DAC on SPI (LTC2641_LTC2642.md,
// "fd" revision). Write-only: CS, SCLK, DIN; no register, no command, no read-back. A frame is 16
// bits MSB first, taken on the SCLK rising edges while CS is low; CS going high latches the word
// and updates the output (Serial Interface, md:517-529, Figure 1a md:542). Fewer than 16 rising
// edges corrupt the word, more keep the last 16 (md:529).
//
// - `Ltc2641::frame<Bits>(code)` is the 16-bit word for a code, left-justified for the -14/-12
//   parts (md:529, Tables 1b/1c md:579-601); a firmware that writes the SPI data register itself
//   (from a timer interrupt) sends exactly that halfword, then waits until the block is not busy
//   before CS goes high.
// - The timing below is what such a firmware has to meet; all minimums are a few ns.
// - `Ltc2641::Dac<Master, Cs, Bits>` writes it through a queued SPI master (QueueCore.hpp), one
//   16-bit frame a write, the newest code winning while one is on the wire.
#include "../../Quantities.hpp"
#include "../QueueCore.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir { namespace SPI { namespace Ltc2641 {

    inline constexpr std::string_view Name = "LTC2641";

    /// DIN is taken on the SCLK rising edge (pin functions md:247, md:529), and SCLK's idle level
    /// is not constrained beyond t7/t8: mode 0 (mode 3 would do as well).
    inline constexpr ClockMode Mode = ClockMode::_0;

    // Timing Characteristics (md:222-238, page 5), over temperature, VDD 3 V or 5 V.
    /// fSCLK at most 50 MHz, 50 % duty cycle.
    inline constexpr Units::Hertz MaxClock = Units::hertz(50'000'000);
    /// t1: DIN valid to SCLK rising, setup >= 10 ns.
    inline constexpr std::uint32_t DinSetupMinNs = 10;
    /// t2: DIN hold after SCLK rising >= 0 ns.
    inline constexpr std::uint32_t DinHoldMinNs = 0;
    /// t3 / t4: SCLK high / low >= 9 ns each.
    inline constexpr std::uint32_t SclkHighMinNs = 9;
    inline constexpr std::uint32_t SclkLowMinNs  = 9;
    /// t5: CS high between two frames >= 10 ns.
    inline constexpr std::uint32_t CsHighMinNs = 10;
    /// t6: the 16th (last) SCLK rising edge to CS rising >= 8 ns.
    inline constexpr std::uint32_t LastSclkToCsHighMinNs = 8;
    /// t7: CS falling to the first SCLK rising >= 8 ns.
    inline constexpr std::uint32_t CsLowToSclkMinNs = 8;
    /// t8: CS rising to the next SCLK rising >= 8 ns.
    inline constexpr std::uint32_t CsHighToSclkMinNs = 8;
    /// t9: CLR low pulse >= 15 ns (CLR clears to code 0 on the LTC2641, midscale on the LTC2642).
    inline constexpr std::uint32_t ClrLowMinNs = 15;
    /// VDD high to CS low, power-up delay, 30 us typical (no minimum given).
    inline constexpr std::chrono::microseconds PowerUpDelay{30};
    /// Unbuffered VOUT settling to +-0.5 LSB of full scale, 1 us typical with CL = 10 pF (md:185);
    /// it is a single pole of ROUT (6.2 kOhm) x (COUT + CL) (md:687-690).
    inline constexpr std::chrono::microseconds SettlingTypical{1};

    /// One frame: 16 bits.
    inline constexpr std::size_t FrameBits = 16;

    /// LTC2641 powers up (and CLR clears) to code 0, the LTC2642 to midscale (md:533, md:14).
    inline constexpr std::uint16_t PowerOnCodeLtc2641 = 0;

    /// The 16-bit word for `code` on a `Bits`-bit part: the code left-justified, the don't-care
    /// LSBs zero (md:529; Tables 1a-1c md:565-601). A code above the part's range saturates at
    /// full scale rather than wrapping to a low voltage.
    template<unsigned Bits = 16>
    [[nodiscard]] constexpr std::uint16_t frame(std::uint32_t code) {
        static_assert(Bits == 16 || Bits == 14 || Bits == 12, "LTC2641-16, -14 or -12");
        constexpr std::uint32_t Max = (1U << Bits) - 1U;
        auto const              c   = code > Max ? Max : code;
        return static_cast<std::uint16_t>(c << (16U - Bits));
    }

    /// The frame as the two bytes on the wire, MSB first (for an 8-bit SPI frame size).
    template<unsigned Bits = 16>
    [[nodiscard]] constexpr std::array<std::byte,
                                       2>
    frameBytes(std::uint32_t code) {
        auto const w = frame<Bits>(code);
        return {std::byte{static_cast<std::uint8_t>(w >> 8U)},
                std::byte{static_cast<std::uint8_t>(w & 0xFFU)}};
    }

    /// VOUT for a code on the unipolar LTC2641: VREF x code / 2^Bits (Table 1a, md:565-575).
    template<unsigned Bits = 16>
    [[nodiscard]] constexpr Units::MicroVolt voltage(std::uint32_t    code,
                                                     Units::MicroVolt vref) {
        constexpr std::uint32_t Max = (1U << Bits) - 1U;
        auto const              c   = code > Max ? Max : code;
        auto const              uv  = vref.numerical_value_in(Units::si::micro<Units::si::volt>);
        return Units::microVolt(static_cast<std::int32_t>(std::int64_t{uv} * std::int64_t{c}
                                                          / (std::int64_t{1} << Bits)));
    }

    /// The LTC2641 on a queued SPI master, for occasional writes: write(code) sends one 16-bit
    /// frame (CS low, 16 bits, CS high = output updated). A write while one is on the wire is
    /// kept and sent from that one's completion; of several such, only the newest goes out. A
    /// frame the master refuses (queue full) or fails is counted and not repeated: write again.
    template<typename Master,
             typename Cs,
             unsigned          Bits    = 16,
             Units::Hertz::rep ClockHz = 25'000'000>
    class Dac {
    public:
        static_assert(ClockHz <= 50'000'000,
                      "fSCLK is at most 50 MHz (LTC2641_LTC2642.md:237)");
        static constexpr auto Setup = Master::setup(Mode, Units::hertz(ClockHz));

        Dac() { apply(set(Cs{})); }

        Dac(Dac const&)            = delete;
        Dac& operator=(Dac const&) = delete;

        /// Sends `code` now, or once the frame on the wire is done. Callable from the main loop
        /// and from an interrupt (the completion runs from the master's).
        void write(std::uint32_t code) {
            next_.store(frame<Bits>(code), std::memory_order_relaxed);
            pending_.store(true, std::memory_order_release);
            kick_();
        }

        /// Frames the master completed / failed or refused.
        [[nodiscard]] std::uint32_t writes() const {
            return writes_.load(std::memory_order_relaxed);
        }

        [[nodiscard]] std::uint32_t failures() const {
            return failures_.load(std::memory_order_relaxed);
        }

        /// The last word the master completed (what the output shows, if the part is there).
        [[nodiscard]] std::uint16_t lastWritten() const {
            return last_.load(std::memory_order_relaxed);
        }

        [[nodiscard]] bool busy() const { return running_.load(std::memory_order_acquire); }

    private:
        static void select_() { apply(clear(Cs{})); }

        static void deselect_() { apply(set(Cs{})); }

        static constexpr Lines lines_{&select_, &deselect_, nullptr, nullptr};

        /// Whoever turns `running_` from false to true owns the wire and sends what is pending; a
        /// completion gives it back and kicks again, so a write that lands while a frame is out is
        /// sent from that frame's completion, and none is left behind.
        void kick_() {
            for(;;) {
                if(running_.exchange(true, std::memory_order_acq_rel)) { return; }
                if(pending_.exchange(false, std::memory_order_acq_rel)) {
                    start_(next_.load(std::memory_order_relaxed));
                    return;
                }
                running_.store(false, std::memory_order_release);
                if(!pending_.load(std::memory_order_acquire)) { return; }
            }
        }

        void start_(std::uint16_t w) {
            word_         = w;   // a native halfword: `wide` sends it MSB first
            bool const ok = Master::submit(
              typename Master::Request{.setup    = Setup,
                                       .lines    = lines_,
                                       .tx       = std::as_bytes(std::span{&word_, 1}),
                                       .rx       = std::as_writable_bytes(std::span{&sink_, 1}),
                                       .wide     = true,
                                       .callback = [this](auto r) {
                                           if(r == decltype(r)::succeeded) {
                                               last_.store(word_, std::memory_order_relaxed);
                                               writes_.fetch_add(1, std::memory_order_relaxed);
                                           } else {
                                               failures_.fetch_add(1, std::memory_order_relaxed);
                                           }
                                           running_.store(false, std::memory_order_release);
                                           kick_();
                                       }});
            if(!ok) {
                failures_.fetch_add(1, std::memory_order_relaxed);
                running_.store(false, std::memory_order_release);
            }
        }

        std::uint16_t              word_{};
        std::uint16_t              sink_{};
        std::atomic<std::uint16_t> next_{};
        std::atomic<std::uint16_t> last_{};
        std::atomic<bool>          pending_{false};
        std::atomic<bool>          running_{false};
        std::atomic<std::uint32_t> writes_{0};
        std::atomic<std::uint32_t> failures_{0};
    };

}}}   // namespace Kvasir::SPI::Ltc2641
