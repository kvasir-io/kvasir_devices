#pragma once

#include "kvasir/Devices/SharedBusDevice.hpp"
#include "kvasir/Devices/utils.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
#include <variant>

namespace Kvasir {

template<typename I2C, typename Clock>
struct I2CScanner : SharedBusDevice<I2C> {
    using tp = typename Clock::time_point;
    using dt = typename Clock::duration;
    using OS = typename I2C::OperationState;

    using SharedBusDevice<I2C>::acquire;
    using SharedBusDevice<I2C>::release;
    using SharedBusDevice<I2C>::isOwner;

    static constexpr std::uint8_t MinValidAddr = 0x08;
    static constexpr std::uint8_t MaxValidAddr = 0x77;
    static constexpr std::size_t  MaxDevices   = MaxValidAddr - MinValidAddr + 1;

    static constexpr bool isReservedAddr(std::uint8_t addr) {
        return std::clamp(addr, MinValidAddr, MaxValidAddr) != addr;
    }

    static constexpr std::uint8_t nextNonReservedAddr(std::uint8_t addr) {
        ++addr;
        while(addr <= MaxValidAddr && isReservedAddr(addr)) { ++addr; }
        return addr;
    }

    struct Idle {};

    struct Scanning {
        std::uint8_t endAddr;
        std::uint8_t currentAddr;
    };

    struct ReceiveWait {
        std::uint8_t endAddr;
        std::uint8_t currentAddr;
    };

    struct Done {};

    using State = std::variant<Idle, Scanning, ReceiveWait, Done>;

    State                                state{Idle{}};
    std::array<std::uint8_t, MaxDevices> foundAddresses{};
    std::size_t                          foundCount{0};

    constexpr I2CScanner() = default;

    void startScan() { startScan(MinValidAddr, MaxValidAddr); }

    void startScan(std::uint8_t startAddr,
                   std::uint8_t endAddr) {
        reset();

        if(startAddr <= endAddr) {
            state = Scanning{endAddr, startAddr};
        } else {
            state = Done{};
        }
    }

    bool isDone() const { return std::holds_alternative<Done>(state); }

    std::span<std::uint8_t const> getFoundAddresses() const {
        return std::span<std::uint8_t const>{foundAddresses.data(), foundCount};
    }

    void reset() {
        if(isOwner()) { release(); }
        state      = Idle{};
        foundCount = 0;
    }

private:
    void addFoundAddress(std::uint8_t addr) {
        if(foundCount < MaxDevices) { foundAddresses[foundCount++] = addr; }
    }

public:
    void handler() {
        auto const currentTime = Clock::now();

        auto scanNext = [](auto const& st) -> State {
            std::uint8_t nextAddr = nextNonReservedAddr(st.currentAddr);
            if(nextAddr <= st.endAddr) {
                return Scanning{st.endAddr, nextAddr};
            } else {
                return Done{};
            }
        };

        state = Kvasir::SM::match(
          state,
          [&](Idle const& st) -> State { return st; },
          [&](Scanning const& st) -> State {
              if(acquire()) {
                  I2C::receive(currentTime, st.currentAddr, 1);
                  return ReceiveWait{st.endAddr, st.currentAddr};
              }
              return st;
          },
          [&](ReceiveWait const& st) -> State {
              assert(isOwner());

              switch(I2C::operationState(currentTime)) {
              case OS::ongoing:
                  {
                      return st;
                  }

              case OS::succeeded:
                  {
                      release();
                      UC_LOG_D("Found device on {:#x}", st.currentAddr);

                      addFoundAddress(st.currentAddr);

                      return scanNext(st);
                  }

              case OS::failed:
                  {
                      release();
                      UC_LOG_D("Found no device on {:#x}", st.currentAddr);

                      return scanNext(st);
                  }
              }
              return st;
          },
          [&](Done const& st) -> State { return st; });
    }
};

}   // namespace Kvasir
