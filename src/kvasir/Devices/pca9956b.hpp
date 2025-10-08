#pragma once

#include "kvasir/Devices/SharedBusDevice.hpp"
#include "kvasir/Util/using_literals.hpp"

#include <array>
#include <bitset>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <optional>
#include <variant>

namespace Kvasir {
template<typename I2C, typename Clock>
struct Pca9956b : SharedBusDevice<I2C> {
    using tp = typename Clock::time_point;
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

    struct Idle {
        tp timeout{};
    };

    struct SendWait {
        bool* reseter;
    };

    struct ReadWait {};

    struct Registers {
        static constexpr std::byte Mode2{0x81};
        static constexpr std::byte LedOut{0x82};
        static constexpr std::byte PwmAll{0xBF};
        static constexpr std::byte IRefAll{0xC0};
        static constexpr std::byte Pwm{0x8A};
        static constexpr std::byte IRef{0xA2};
    };

    static constexpr std::size_t ledOutSize{7};
    static constexpr std::size_t irefSize{25};
    static constexpr std::size_t pwmSize{25};

    using sv = std::variant<Init, Idle, SendWait, ReadWait>;

    sv st_;

    std::uint8_t const                i2caddress;
    std::uint16_t const               rExt_;
    std::byte                         irefAll_;
    std::byte                         pwmAll_;
    std::array<std::byte, ledOutSize> ledOut_;
    std::array<std::byte, irefSize>   iref_;
    std::array<std::byte, pwmSize>    pwm_;
    bool                              irefAllChanged_;
    bool                              pwmAllChanged_;
    bool                              ledOutChanged_;
    bool                              irefChanged_;
    bool                              pwmChanged_;

    static constexpr auto startup_time{500ms};
    static constexpr auto fail_retry_time{200ms};
    static constexpr auto error_read_interval{1000ms};
    static constexpr auto error_read_interval_after_set{10ms};

    static_assert(I2C::BufferSize >= std::max({ledOutSize,
                                               irefSize,
                                               pwmSize}),
                  "I2C buffer to small");

    constexpr Pca9956b(std::uint8_t  address,
                       std::uint16_t rExt)
      : st_{Init{tp{} + startup_time}}
      , i2caddress{address}
      , rExt_{rExt}
      , irefAll_{}
      , pwmAll_{}
      , ledOut_{Registers::LedOut}
      , iref_{Registers::IRef}
      , pwm_{Registers::Pwm}
      , irefAllChanged_{false}
      , pwmAllChanged_{false}
      , ledOutChanged_{false}
      , irefChanged_{false}
      , pwmChanged_{false} {}

    template<typename PWM>
    void setPwmAll(PWM pwm) {
        pwmAllChanged_ = true;
        pwmAll_        = std::byte(pwm);
        std::generate(std::next(begin(pwm_)), end(pwm_), [&] { return pwmAll_; });
    }

    constexpr std::byte calcIref(std::uint16_t rExt,
                                 std::uint16_t current) {
        return std::byte((current * rExt * 4U) / 9000U);
    }

    void setCurrentAll(std::uint16_t current) {
        irefAllChanged_ = true;
        irefAll_        = calcIref(rExt_, current);
        std::generate(std::next(begin(iref_)), end(iref_), [&] { return irefAll_; });
    }

    void setLedOut(std::array<std::byte,
                              6> const& ledOut) {
        ledOutChanged_ = true;
        std::copy(begin(ledOut), end(ledOut), std::next(begin(ledOut_)));
    }

    template<typename PWM>
    void setPwm(std::size_t index,
                PWM         pwm) {
        assert(pwm_.size() > index + 1);
        if(pwm_[index + 1] == std::byte(pwm)) {
            return;
        }
        pwmChanged_     = true;
        pwm_[index + 1] = std::byte(pwm);
    }

    void setCurrent(std::size_t   index,
                    std::uint16_t current) {
        assert(iref_.size() > index + 1);
        auto ir = calcIref(rExt_, current);
        if(iref_[index + 1] == ir) {
            return;
        }
        irefChanged_     = true;
        iref_[index + 1] = ir;
    }

    struct Resetter {
        Resetter(bool& a_,
                 bool& b_,
                 bool& c_,
                 bool& d_,
                 bool& e_)
          : a{a_}
          , b{b_}
          , c{c_}
          , d{d_}
          , e{e_} {}

        void reset() {
            a = true;
            b = true;
            c = true;
            d = false;
            e = false;
        }

        bool& a;
        bool& b;
        bool& c;
        bool& d;
        bool& e;
    };

    template<class... Ts>
    struct overloaded : Ts... {
        using Ts::operator()...;
    };
    template<class... Ts>
    overloaded(Ts...) -> overloaded<Ts...>;

    template<typename Variant,
             typename... Matchers>
    auto match(Variant&& variant,
               Matchers&&... matchers) {
        return std::visit(overloaded{std::forward<Matchers>(matchers)...},
                          std::forward<Variant>(variant));
    }

    void handler() {
        auto const currentTime = Clock::now();
        if(auto r
           = Resetter{ledOutChanged_, pwmChanged_, irefChanged_, irefAllChanged_, pwmAllChanged_};
           resetHandler(r))
        {
            st_.template emplace<Init>(currentTime + startup_time);
        }
        st_ = match(
          st_,
          [&](Init const& state) -> sv {
              if(currentTime > state.timeout) {
                  return Idle{currentTime};
              }
              return state;
          },
          [&](Idle const& state) -> sv {
              if(irefChanged_ && acquire()) {
                  I2C::send(currentTime, i2caddress, iref_);
                  return SendWait{&irefChanged_};
              }
              if(pwmChanged_ && acquire()) {
                  I2C::send(currentTime, i2caddress, pwm_);
                  return SendWait{&pwmChanged_};
              }
              if(irefAllChanged_ && acquire()) {
                  I2C::send(currentTime, i2caddress, std::array{Registers::IRefAll, irefAll_});
                  return SendWait{&irefAllChanged_};
              }
              if(ledOutChanged_ && acquire()) {
                  I2C::send(currentTime, i2caddress, ledOut_);
                  return SendWait{&ledOutChanged_};
              }
              if(pwmAllChanged_ && acquire()) {
                  I2C::send(currentTime, i2caddress, std::array{Registers::PwmAll, pwmAll_});
                  return SendWait{&pwmAllChanged_};
              }
              if(currentTime > state.timeout && acquire()) {
                  I2C::send_receive(currentTime, i2caddress, Registers::Mode2, 1);
                  return ReadWait{};
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
                      *(state.reseter) = false;
                      release();
                      resetErrorCount();
                      return Idle{currentTime + error_read_interval_after_set};
                  }
              case OS::failed:
                  {
                      release();
                      incementErrorCount();
                      return Idle{currentTime + fail_retry_time};
                  }
              }
              return state;
          },
          [&](ReadWait const& state) -> sv {
              assert(isOwner());
              switch(I2C::operationState(currentTime)) {
              case OS::ongoing:
                  {
                      return state;
                  }
              case OS::succeeded:
                  {
                      release();
                      std::array<std::byte, 1> buffer{};
                      I2C::getReceivedBytes(buffer);
                      // TODO check for error
                      resetErrorCount();
                      return Idle{currentTime + error_read_interval};
                  }
              case OS::failed:
                  {
                      release();
                      incementErrorCount();
                      return Idle{currentTime + fail_retry_time};
                  }
              }
              return state;
          });
    }
};
}   // namespace Kvasir
