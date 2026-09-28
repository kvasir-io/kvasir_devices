#pragma once
// The Ssd1680's SpiBus over a queued SPI master: each send is one request under its own chip
// select, straight from the caller's memory. The driver drives D/C itself between sends.
#include "../Quantities.hpp"
#include "../SPI/QueueCore.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Display {

    struct SpiStreamDefaults {
        /// SSD1680: mode 0 (6.1.2 Table 6-2), fSCL <= 20 MHz writing (12.1 Table 12-1).
        static constexpr SPI::ClockMode Mode  = SPI::ClockMode::_0;
        static constexpr Units::Hertz   Clock = Units::hertz(20'000'000);
    };

    template<typename Master, typename Cs, typename Config = SpiStreamDefaults>
    struct SpiStream {
        enum class OperationState { idle, ongoing, succeeded, failed };

        static constexpr auto Setup = Master::setup(Config::Mode, Config::Clock);

        static constexpr std::size_t MaxChunk = [] {
            if constexpr(requires { Master::Core::MaxFrames; }) {
                return std::size_t{Master::Core::MaxFrames};
            } else {
                return std::size_t{1} << 24U;
            }
        }();

        static void send_nocopy(std::span<std::byte const> const& bytes) {
            if(bytes.size() > MaxChunk) {
                state_ = OperationState::failed;
                return;
            }
            state_ = OperationState::ongoing;
            // A send restart() abandoned may still complete; it must not judge the next one.
            auto const               gen = ++generation_;
            typename Master::Request r{.setup = Setup, .lines = lines_, .tx = bytes};
            r.callback = [gen](SPI::TransferResult res) {
                if(gen != generation_) { return; }
                state_ = res == SPI::TransferResult::succeeded ? OperationState::succeeded
                                                               : OperationState::failed;
            };
            if(!Master::submit(r)) { state_ = OperationState::failed; }
        }

        static OperationState operationState() {
            Master::handler();
            return state_;
        }

        static void restart() {
            ++generation_;
            state_ = OperationState::idle;
        }

    private:
        static void select_() { apply(clear(Cs{})); }

        static void deselect_() { apply(set(Cs{})); }

        static constexpr SPI::Lines lines_{&select_, &deselect_, nullptr, nullptr};

        inline static OperationState volatile state_{OperationState::idle};
        /// Written by the loop only, read by the callback as one aligned word.
        inline static std::uint32_t generation_{};
    };

}}   // namespace Kvasir::Display
