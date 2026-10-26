#pragma once
#include "Log.hpp"
#include "kvasir/Util/literals.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace Kvasir { namespace Eeprom11AA02E48 {
    using namespace Kvasir::literals;

    /// The UNI/O timing. `halfBit` is half a UNI/O bit period: every bit is Manchester coded
    /// as two halves of `halfBit` each, so the bit period T_E is 2 x halfBit. The UNI/O bus
    /// specification allows T_E from 10 us to 100 us (the 11AA02E48 data sheet's "Bit
    /// Period"), and the device learns it from the header's 0x55. 25 us puts T_E at 50 us,
    /// the middle of the range; a halfBit of 50 us would run at T_E(max), where a slow edge
    /// or a delay a little long overruns the specification. Another timing is a
    /// type with its own `static constexpr std::chrono::microseconds halfBit`.
    struct DefaultTiming {
        static constexpr std::chrono::microseconds halfBit{25};
    };

    /// Read the six-byte EUI-48 of a Microchip 11AA02E48 (UNI/O, one wire) blocking, bit
    /// banged on `Pin` with Clock::delay. The whole read is timed by delays alone, so the
    /// caller masks interrupts around it: a stretched half bit is a Manchester violation the
    /// part answers by going idle.
    template<typename Clock,
             typename Pin,
             typename Timing = DefaultTiming>
    std::optional<std::array<std::byte,
                             6>>
    readMacBlocking() {
        using std::chrono::microseconds;
        static constexpr microseconds HalfBit = Timing::halfBit;
        static_assert(HalfBit >= microseconds{5} && HalfBit <= microseconds{50},
                      "UNI/O bit period T_E = 2 x HalfBit is 10 .. 100 us");
        bool error = false;
        // Counted here, logged once the line is released: a log call between two half bits
        // stretches the next one, and the part answers that by going idle.
        std::uint32_t violations{};

        auto setPin = []() { apply(makeInput(Pin{})); };

        auto clearPin = []() { apply(makeOutput(Pin{}), clear(Pin{})); };

        // Each half bit is sampled a quarter bit into it, and the bit's remainder is waited out
        // from that sample on, so the sample point does not drift when HalfBit is odd.
        static constexpr microseconds Quarter = HalfBit / 2;
        static constexpr microseconds Rest    = HalfBit - Quarter;

        // The SAK is the part's to drive: the line is released first, since a NoMAK before it
        // leaves the master driving low.
        auto checkSak = [&, first = true]() mutable {
            setPin();
            Clock::delay(Quarter);
            bool const startBit = apply(read(Pin{}));
            Clock::delay(HalfBit);
            bool const bit = apply(read(Pin{}));
            Clock::delay(Rest);
            if(first) {
                first = false;
                return;
            }
            if(startBit || !bit) { error = true; }
        };

        auto outBit = [&](bool v) {
            if(v) {
                clearPin();
                Clock::delay(HalfBit);
                setPin();
            } else {
                setPin();
                Clock::delay(HalfBit);
                clearPin();
            }
            Clock::delay(HalfBit);
        };

        auto const outByte = [&](std::byte v, bool mak) {
            for(std::size_t i{}; i < 8; ++i) { outBit((v & (1_b << (7 - i))) != 0x0_b); }
            outBit(mak);
            checkSak();
        };

        auto const inByte = [&](bool mak) {
            std::byte v{};
            for(std::size_t i{}; i < 8; ++i) {
                Clock::delay(Quarter);
                bool const startBit = apply(read(Pin{}));
                Clock::delay(HalfBit);
                bool const bit = apply(read(Pin{}));
                v |= (bit ? 0x1_b : 0x0_b) << (7 - i);
                Clock::delay(Rest);
                if(startBit == bit) { ++violations; }
            }
            outBit(mak);
            checkSak();
            return v;
        };

        clearPin();
        Clock::delay(microseconds{1000});
        setPin();
        Clock::delay(microseconds{650});   // standby pulse: TSTBY is 600 us minimum
        clearPin();
        Clock::delay(microseconds{10});
        outByte(0x55_b, true);
        outByte(0xA0_b, true);
        outByte(0x03_b, true);

        static constexpr std::size_t N = 6;
        outByte(0x00_b, true);
        outByte(std::byte{0xFF - (N - 1)}, true);

        std::array<std::byte, N> mac{};
        // MAK after each byte but the last, which gets NoMAK: that ends the READ (4.1) and
        // the part answers it with its SAK and returns to standby, leaving the line to the master.
        for(std::size_t i{}; i < N; ++i) { mac[i] = inByte(i + 1 < N); }

        setPin();
        if(violations != 0) {
            UC_LOG_W("11AA02E48: {} Manchester violation(s) (start bit equal to data bit)",
                     violations);
        }
        if(error || violations != 0) { return std::nullopt; }
        return mac;
    }

    /// readMacBlocking(), up to `attempts` reads until one checks out.
    template<typename Clock,
             typename Pin,
             typename Timing = DefaultTiming>
    std::optional<std::array<std::byte,
                             6>>
    readMacBlockingRetry(std::size_t attempts) {
        std::optional<std::array<std::byte, 6>> mac;
        while(!mac && attempts != 0) {
            mac = readMacBlocking<Clock, Pin, Timing>();
            Clock::delay(std::chrono::microseconds{10});
            --attempts;
        }

        return mac;
    }

}}   // namespace Kvasir::Eeprom11AA02E48
