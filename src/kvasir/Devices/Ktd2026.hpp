#pragma once

#include "kvasir/Devices/SharedBusDevice.hpp"
#include "kvasir/Devices/utils.hpp"
#include "kvasir/Util/using_literals.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <variant>

namespace Kvasir {
template<typename I2C, typename Clock>
struct Ktd2026 : SharedBusDevice<I2C> {
    using tp = typename Clock::time_point;
    using dt = typename Clock::duration;
    using SharedBusDevice<I2C>::acquire;
    using SharedBusDevice<I2C>::release;
    using SharedBusDevice<I2C>::isOwner;
    using SharedBusDevice<I2C>::incementErrorCount;
    using SharedBusDevice<I2C>::resetErrorCount;
    using SharedBusDevice<I2C>::resetHandler;
    using OS = typename I2C::OperationState;

    struct Init {
        explicit Init(tp t) : timeout{t} {}

        tp timeout{};
    };

    struct Idle {};

    using nsv = std::variant<Init, Idle>;

    struct SendWait {
        nsv nextState;
        nsv failState;
    };

    struct Registers {
        static constexpr std::byte EnRst{0x00};
        static constexpr std::byte ChannelControl{0x04};
    };

    static constexpr std::size_t                     initSize{5};
    static constexpr std::size_t                     colorSize{6};
    static constexpr std::array<std::byte, initSize> initOut{Registers::EnRst,
                                                             0x1C_b,
                                                             0_b,
                                                             0_b,
                                                             0_b};

    static constexpr dt startup_time{std::chrono::milliseconds(500)};
    static constexpr dt reset_time{std::chrono::microseconds(800)};
    static constexpr dt fail_retry_time{std::chrono::milliseconds(200)};

    using sv = std::variant<Init, Idle, SendWait>;

    static constexpr sv toSv(nsv const& ns) {
        if(std::holds_alternative<Init>(ns)) {
            return std::get<Init>(ns);
        }
        if(std::holds_alternative<Idle>(ns)) {
            return std::get<Idle>(ns);
        }
        assert(false);
    }

    sv st_;

    std::uint8_t const               i2caddress;
    std::array<std::byte, colorSize> colorOut_;
    bool                             colorChanged_;

    static_assert(I2C::BufferSize >= std::max(initSize,
                                              colorSize),
                  "I2C buffer to small");

    constexpr explicit Ktd2026(std::uint8_t address)
      : st_{Init{tp{} + startup_time}}
      , i2caddress{address}
      , colorOut_{Registers::ChannelControl,
                  0x00_b,
                  0x00_b,
                  0x00_b,
                  0x00_b,
                  0x00_b}
      , colorChanged_{true} {}

    constexpr Ktd2026() : Ktd2026(0x30) {}

    template<typename RT,
             typename GT,
             typename BT>
    void set(RT R,
             GT G,
             BT B) {
        auto      cc      = [](auto c) { return std::byte(c != 0 ? c - 1 : c); };
        std::byte onState = (R != 0 ? 1_b : 0_b) | (B != 0 ? 4_b : 0_b) | (G != 0 ? 0x10_b : 0_b);
        if(cc(R) != colorOut_[3] || cc(B) != colorOut_[4] || cc(G) != colorOut_[5]
           || colorOut_[1] != onState)
        {
            colorChanged_ = true;
            colorOut_[1]  = onState;
            colorOut_[3]  = cc(R);
            colorOut_[4]  = cc(B);
            colorOut_[5]  = cc(G);
        }
    }

    template<typename C>
    void set(C const& c) {
        set(c.r, c.g, c.b);
    }

    struct Resetter {
        explicit Resetter(bool& a_) : a{a_} {}

        void reset() { a = true; }

        bool& a;
    };

    void handler() {
        auto const currentTime = Clock::now();
        if(auto r = Resetter{colorChanged_}; resetHandler(r)) {
            st_.template emplace<Init>(currentTime + startup_time);
        }

        st_ = Kvasir::SM::match(
          st_,
          [&](Init const& state) -> sv {
              if(currentTime > state.timeout && acquire()) {
                  I2C::send(currentTime, i2caddress, initOut);
                  return SendWait{Idle{}, Init{currentTime + fail_retry_time}};
              }
              return state;
          },
          [&](Idle const& state) -> sv {
              if(colorChanged_ && acquire()) {
                  I2C::send(currentTime, i2caddress, colorOut_);
                  colorChanged_ = false;
                  return SendWait{Idle{}, Idle{}};
              }
              return state;
          },
          [&](SendWait const& state) -> sv {
              assert(isOwner());
              switch(I2C::operationState(currentTime)) {
              case OS::ongoing:
                  {
                      return state;
                  }
              case OS::succeeded:
                  {
                      release();
                      resetErrorCount();
                      return toSv(state.nextState);
                  }
              case OS::failed:
                  {
                      release();
                      incementErrorCount();
                      colorChanged_ = true;
                      return toSv(state.failState);
                  }
              }
              return state;
          });
    }
};
}   // namespace Kvasir
