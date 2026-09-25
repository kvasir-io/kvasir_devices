#pragma once

#include "../Log.hpp"
#include "Descriptors.hpp"

#include <array>
#include <cstdint>
#include <utility>

namespace Kvasir::USB::detail {

/// Where endpoint 0's control transfer stands (USB 2.0 chapter 8.5.3), shared by its IN and OUT
/// side. A transition the protocol does not allow is logged and taken anyway: the host decides
/// what happens next, the device only notices that it lost track.
struct EP0ControlState {
private:
    ControlStage current_stage{ControlStage::Idle};
    // A protocol bug repeats per transfer; no clock here, so count based.
    Kvasir::CountLimiter<> badTransitionLog_{};

    // Transition table: for each destination state, bitmask of valid source states
    // valid_from[to_state] = bitmask where bit N = 1 if transition from state N is valid
    // Bit 0=Idle, Bit 1=Setup, Bit 2=Data, Bit 3=Status, Bit 4=Stall
    static constexpr std::array<std::uint8_t, 5> valid_from = {
      0b01000,   // [0] to Idle:   valid from Status (bit 3)
      0b11111,   // [1] to Setup:  valid from any state (bits 0-4)
      0b00010,   // [2] to Data:   valid from Setup (bit 1)
      0b00110,   // [3] to Status: valid from Setup or Data (bits 1-2)
      0b00110,   // [4] to Stall:  valid from Setup or Data (bits 1-2)
    };

public:
    constexpr void transition(ControlStage new_stage) {
        auto const valid_mask  = valid_from[std::to_underlying(new_stage)];
        auto const current_bit = 1U << std::to_underlying(current_stage);

        if(!(valid_mask & current_bit)) {
            KVASIR_LOG_LIMITED(badTransitionLog_.allow(),
                               UC_LOG_E,
                               "Invalid EP0 stage transition {} -> {}",
                               current_stage,
                               new_stage);
        }

        current_stage = new_stage;
    }

    constexpr ControlStage stage() const { return current_stage; }

    constexpr void reset() { current_stage = ControlStage::Idle; }
};
}   // namespace Kvasir::USB::detail
