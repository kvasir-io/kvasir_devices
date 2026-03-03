#pragma once

#include "kvasir/Devices/SharedBusDevice.hpp"
#include "kvasir/Devices/utils.hpp"
#include "kvasir/Util/StaticMap.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <variant>

namespace Kvasir {

namespace detail {
    template<typename Derived, typename I2C, typename Clock>
    struct I2CScannerBase : SharedBusDevice<I2C> {
    private:
        using OS = typename I2C::OperationState;

        using SharedBusDevice<I2C>::acquire;
        using SharedBusDevice<I2C>::release;
        using SharedBusDevice<I2C>::isOwner;

        static constexpr std::uint8_t MinValidAddr = 0x08;
        static constexpr std::uint8_t MaxValidAddr = 0x77;
        static constexpr std::size_t  MaxDevices   = MaxValidAddr - MinValidAddr + 1u;

        struct Probing {
            std::uint8_t currentAddr;
        };

        struct ReceiveWait {
            std::uint8_t currentAddr;
        };

        struct Done {};

        using State = std::variant<Done, Probing, ReceiveWait>;

        State state{Done{}};

    public:
        bool isDone() const { return std::holds_alternative<Done>(state); }

        void handler() {
            auto const currentTime = Clock::now();
            auto&      self        = static_cast<Derived&>(*this);

            state = Kvasir::SM::match(
              state,
              [](Done const& st) -> State { return st; },
              [&](Probing const& st) -> State {
                  if(acquire()) {
                      I2C::receive(currentTime, st.currentAddr, 1);
                      return ReceiveWait{st.currentAddr};
                  }
                  return st;
              },
              [&](ReceiveWait const& st) -> State {
                  assert(isOwner());

                  auto step = [&](bool found) -> State {
                      auto const next = self.scanResult(st.currentAddr, found);
                      if(next) {
                          return State{Probing{*next}};
                      } else {
                          release();
                          return State{Done{}};
                      }
                  };

                  switch(I2C::operationState(currentTime)) {
                  case OS::ongoing:   return st;
                  case OS::succeeded: return step(true);
                  case OS::failed:    return step(false);
                  }
                  return st;
              });
        }

    protected:
        constexpr I2CScannerBase() = default;

        void startScanBase(std::uint8_t firstAddr) { state = Probing{firstAddr}; }
    };
}   // namespace detail

template<typename I2C, typename Clock>
struct I2CScannerRange : detail::I2CScannerBase<I2CScannerRange<I2C, Clock>, I2C, Clock> {
    using Base = detail::I2CScannerBase<I2CScannerRange<I2C, Clock>, I2C, Clock>;

    friend Base;

    void startScan() { startScan(Base::MinValidAddr, Base::MaxValidAddr); }

    void startScan(std::uint8_t startAddr,
                   std::uint8_t endAddr) {
        endAddr_ = endAddr;
        result_.clear();
        std::uint8_t a = startAddr;
        while(a <= endAddr && isReservedAddr(a)) { ++a; }
        if(a <= endAddr) { Base::startScanBase(a); }
    }

    std::span<std::pair<std::uint8_t,
                        bool> const>
    getResult() const {
        return {result_.begin(), result_.size()};
    }

private:
    std::optional<std::uint8_t> scanResult(std::uint8_t addr,
                                           bool         found) {
        result_.insert({addr, found});
        std::uint8_t next = nextNonReservedAddr(addr);
        return (next <= endAddr_) ? std::optional(next) : std::nullopt;
    }

    std::uint8_t                                            endAddr_{Base::MaxValidAddr};
    Kvasir::StaticMap<std::uint8_t, bool, Base::MaxDevices> result_{};

    static constexpr bool isReservedAddr(std::uint8_t addr) {
        return std::clamp(addr, Base::MinValidAddr, Base::MaxValidAddr) != addr;
    }

    static constexpr std::uint8_t nextNonReservedAddr(std::uint8_t addr) {
        ++addr;
        while(addr <= Base::MaxValidAddr && isReservedAddr(addr)) { ++addr; }
        return addr;
    }
};

template<typename I2C, typename Clock, std::size_t Size>
struct I2CScannerList : detail::I2CScannerBase<I2CScannerList<I2C, Clock, Size>, I2C, Clock> {
    using Base = detail::I2CScannerBase<I2CScannerList<I2C, Clock, Size>, I2C, Clock>;

    friend Base;

    void startScan(std::span<std::uint8_t const> addrs) {
        result_.clear();
        for(auto a : addrs) { result_.insert({a, false}); }

        if(!result_.empty()) { Base::startScanBase(result_.begin()->first); }
    }

    std::span<std::pair<std::uint8_t,
                        bool> const>
    getResult() const {
        return {result_.begin(), result_.size()};
    }

private:
    std::optional<std::uint8_t> scanResult(std::uint8_t addr,
                                           bool         found) {
        auto it = result_.find(addr);
        assert(it != result_.end());
        it->second = found;
        auto next  = std::next(it);
        return (next != result_.end()) ? std::optional(next->first) : std::nullopt;
    }

    Kvasir::StaticMap<std::uint8_t, bool, Size> result_{};
};

}   // namespace Kvasir
