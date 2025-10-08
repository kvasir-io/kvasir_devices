#pragma once

#include "kvasir/Util/using_literals.hpp"

#include <array>
#include <cstddef>
#include <optional>

namespace Kvasir {
template<typename Clock, typename SPI, typename Cs, typename Drdy, typename Config>
struct Ads131m0x {
    static constexpr auto MaxChannels = 8;
    // needed config
    // numChannels
    static_assert(MaxChannels >= Config::numChannels && Config::numChannels != 0,
                  "wrong channels");

    using tp = typename Clock::time_point;

    tp next = Clock::now() + 1s;

    std::array<std::byte, 3 + 3 * Config::numChannels>           data;
    std::atomic<bool>                                            running{false};
    std::optional<std::array<std::int16_t, Config::numChannels>> v{};

    void handler() {
        auto const now = Clock::now();
        auto       rdy = apply(read(Drdy{}));

        if(now > next && !running && !rdy) {
            apply(clear(Cs{}));
            next    = now + 5ms;
            running = true;
            std::fill(data.begin(), data.end(), 0_b);
            SPI::send_receive_nocopy(std::span{data}, std::span{data}, [this]() {
                apply(set(Cs{}));
                //       UC_LOG_D("adc {::3x}", data);
                running = false;
                v.emplace();
                for(std::size_t channel{0}; auto& vv : *v) {
                    std::uint16_t vvv;
                    std::memcpy(&vvv, data.data() + 3 + 3 * channel, 2);
                    vv = static_cast<std::int16_t>(std::byteswap(vvv));
                    ++channel;
                }
                //    UC_LOG_D("data {::10}", v);
                // UC_LOG_D("");
            });
        }
    }
};
}   // namespace Kvasir
