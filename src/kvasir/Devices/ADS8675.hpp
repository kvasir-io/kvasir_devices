#pragma once
#pragma once

#include "kvasir/Util/StaticFunction.hpp"
#include "kvasir/Util/using_literals.hpp"

#include <array>
#include <cstddef>
#include <optional>
namespace Kvasir {
template<typename Clock, typename SPI, typename Cs, typename Rvs, typename Rst>
struct ADS8675 {
    using tp = typename Clock::time_point;
    Kvasir::StaticFunction<void(std::int16_t), 128> dataF;
    tp                                              next;
    std::atomic<bool>                               dma_running{false};
    std::atomic<bool>                               ready{false};

    enum class State { reset, wait_reset, wait_init, wait_rdy, wait_init_data, idle };

    State s{State::reset};

    std::array<std::byte, 4> data{};

    template<typename F>
    ADS8675(F&& f) : dataF{std::forward<F>(f)} {}

    void sampleCallback() {
        if(ready) {
            if(dma_running) {
                UC_LOG_W("bad");
            }
            apply(set(Cs{}));
        }
    }

    void pinInterrupt() {
        if(ready) {
            apply(clear(Cs{}));
            dma_running = true;
            data        = std::array<std::byte, 4>{};
            SPI::send_receive_nocopy(std::span{data}, std::span{data}, [this]() {
                std::uint16_t v{};
                std::memcpy(&v, data.data(), 2);
                std::uint16_t v2 = std::byteswap(v);
                v2               = v2 >> 2;
                std::int32_t v3  = v2;
                v3               = v3 - 8192;
                dataF(static_cast<std::int16_t>(v3));
                dma_running = false;
            });
        }
    }

    void handler() {
        auto const now = Clock::now();
        switch(s) {
        case State::reset:
            {
                apply(clear(Rst{}));
                apply(clear(Cs{}));
                next = now + 500ms;
                s    = State::wait_reset;
            }
            break;
        case State::wait_reset:
            {
                if(now > next) {
                    apply(set(Rst{}));
                    next = now + 100ms;
                    s    = State::wait_init;
                }
            }
            break;
        case State::wait_init:
            {
                if(now > next) {
                    apply(set(Cs{}));
                    s = State::wait_rdy;
                }
            }
            break;
        case State::wait_rdy:
            {
                bool const rdy = apply(read(Rvs{}));
                if(rdy) {
                    apply(clear(Cs{}));
                    s           = State::wait_init_data;
                    dma_running = true;
                    data        = std::array<std::byte, 4>{};
                    data[0]     = 0xD0_b;
                    data[1]     = 0x14_b;
                    data[2]     = 0x00_b;
                    data[3]     = 0x04_b;
                    SPI::send_receive_nocopy(std::span{data}, std::span{data}, [this]() {
                        dma_running = false;
                    });
                }
            }
            break;
        case State::wait_init_data:
            {
                if(!dma_running) {
                    s = State::idle;
                }
            }
            break;
        case State::idle:
            {
                ready = true;
            }
            break;
        }
    }
};
}   // namespace Kvasir
