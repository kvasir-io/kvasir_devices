#pragma once
#include <array>
#include <cstddef>
#include <optional>

namespace Kvasir { namespace _11AA02E48 {

    template<typename Clock, typename Pin>
    std::optional<std::array<std::byte, 6>> readMacBlocking() {
        static constexpr std::size_t CLKPeriod = 50;
        bool                         error     = false;

        auto set_pin = []() { apply(makeInput(Pin{})); };

        auto clear_pin = []() { apply(makeOutput(Pin{}), clear(Pin{})); };

        auto check_sak = [&, first = true]() mutable {
            Clock::template delay<std::chrono::microseconds, CLKPeriod / 2>();
            bool const start_bit = apply(read(Pin{}));
            Clock::template delay<std::chrono::microseconds, CLKPeriod>();
            bool const bit = apply(read(Pin{}));
            Clock::template delay<std::chrono::microseconds, CLKPeriod / 2>();
            if(first) {
                first = false;
                return;
            }
            if(start_bit != false || bit != true) {
                error = true;
            }
        };

        auto out_bit = [&](bool v) {
            if(v) {
                clear_pin();
                Clock::template delay<std::chrono::microseconds, CLKPeriod>();
                set_pin();
            } else {
                set_pin();
                Clock::template delay<std::chrono::microseconds, CLKPeriod>();
                clear_pin();
            }
            Clock::template delay<std::chrono::microseconds, CLKPeriod>();
        };

        auto out_byte = [&](std::byte v, bool mak) {
            for(std::size_t i{}; i < 8; ++i) {
                out_bit((v & (1_b << (7 - i))) != 0x0_b);
            }
            out_bit(mak);
            check_sak();
        };

        auto in_byte = [&](bool mak) {
            std::byte v{};
            for(std::size_t i{}; i < 8; ++i) {
                Clock::template delay<std::chrono::microseconds, CLKPeriod / 2>();
                bool const start_bit = apply(read(Pin{}));
                Clock::template delay<std::chrono::microseconds, CLKPeriod>();
                bool const bit = apply(read(Pin{}));
                v |= (bit ? 0x1_b : 0x0_b) << (7 - i);
                Clock::template delay<std::chrono::microseconds, CLKPeriod / 2>();
                if(start_bit == bit) {
                    UC_LOG_C("bad read");
                    error = true;
                }
            }
            out_bit(mak);
            check_sak();
            return v;
        };

        clear_pin();
        Clock::template delay<std::chrono::microseconds, 1000>();
        set_pin();
        Clock::template delay<std::chrono::microseconds, 600>();
        clear_pin();
        Clock::template delay<std::chrono::microseconds, 10>();
        out_byte(0x55_b, true);
        out_byte(0xA0_b, true);
        out_byte(0x03_b, true);

        static constexpr std::size_t N = 6;
        out_byte(0x00_b, true);
        out_byte(std::byte{0xFF - (N - 1)}, true);

        std::array<std::byte, N> mac{};
        for(std::size_t i{}; i < N; ++i) {
            mac[i] = in_byte(true);
        }

        set_pin();
        if(error) {
            return std::nullopt;
        }
        return mac;
    }

    template<typename Clock, typename Pin>
    std::optional<std::array<std::byte, 6>> readMacBlockingRetry(std::size_t retrys) {
        std::optional<std::array<std::byte, 6>> mac;
        while(!mac && retrys != 0) {
            mac = readMacBlocking<Clock, Pin>();
            Clock::template delay<std::chrono::microseconds, 10>();
            --retrys;
        }

        return mac;
    }

}}   // namespace Kvasir::_11AA02E48
