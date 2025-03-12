#include <array>
#include <optional>
namespace Kvasir {

template<typename Clock, typename SPI, typename CsPin, typename DrdyPin>
struct Max31865 {
    using tp  = typename Clock::time_point;
    using dtT = float;

    static constexpr auto startup_time{std::chrono::milliseconds(500)};
    static constexpr auto readout_time{std::chrono::milliseconds(500)};

    enum class State : std::uint8_t {
        reset,
        init_wait,
        init,
        idle,
        read_wait,
    };

    State st_{State::reset};
    tp    waitTime_;

    Max31865() { apply(makeInput(DrdyPin{}), makeOutput(CsPin{})); }

    static constexpr std::array<std::byte, 2> initCommand{0x80_b, 0b11000001_b};
    std::array<std::byte, 3>                  readoutBuffer{};

    std::optional<dtT> t_{};
    std::optional<dtT> t() const { return t_; }

    void handler() {
        auto const currentTime = Clock::now();
        switch(st_) {
        case State::reset:
            {
                st_       = State::init;
                waitTime_ = currentTime + startup_time;
                apply(set(CsPin{}));
            }
            break;

        case State::init:
            {
                if(currentTime > waitTime_) {
                    apply(clear(CsPin{}));
                    SPI::send_nocopy(initCommand);
                    st_ = State::init_wait;
                }
            }
            break;

        case State::init_wait:
            {
                if(SPI::operationState() == SPI::OperationState::succeeded) {
                    apply(set(CsPin{}));
                    st_       = State::idle;
                    waitTime_ = currentTime + readout_time;
                }
            }
            break;

        case State::idle:
            {
                if(!apply(read(DrdyPin{}))) {
                    st_              = State::read_wait;
                    readoutBuffer[0] = 0x01_b;
                    apply(clear(CsPin{}));
                    SPI::send_receive_nocopy(readoutBuffer);
                } else if(currentTime > waitTime_) {
                    st_ = State::reset;
                    t_.reset();
                }
            }
            break;

        case State::read_wait:
            {
                if(SPI::operationState() == SPI::OperationState::succeeded) {
                    waitTime_ = currentTime + readout_time;
                    apply(set(CsPin{}));
                    st_ = State::idle;

                    float const tt = static_cast<float>(((std::to_integer<std::uint32_t>(readoutBuffer[1]) << 8U
                                         | std::to_integer<std::uint32_t>(readoutBuffer[2]))
                                        >> 1U)
                                       * 1000)
                                    / 32767.0f;

                    static constexpr auto b = 1.95415f;
                    static constexpr auto a = 0.00028875f;

                    auto const c = 500.0f - tt;

                    auto const temp = (-b + std::sqrt(b * b - 4.0f * a * c)) / (2.0f * a);

                    t_ = temp;
                }
            }
            break;
        }
    }
};
}   // namespace Kvasir
