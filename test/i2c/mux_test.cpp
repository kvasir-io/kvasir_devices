/// The switch under a typical load: a representative mixed bus of about fifty synthetic parts
/// at typical rates, several of them behind a TCA9548A -- register reads, commands with a
/// conversion wait, OLED pages as a window command and a page-sized transfer -- on a wire
/// that takes the time 400 kHz takes, one transaction after another, and a main loop of a
/// fixed length.
///
/// The loop is a short turn for the bus and the device handlers, and every 25 ms a long one
/// that also renders a frame.
///
/// It compares the ShareChannel, HoldChannel and DrainChannel policies: whether the 100 Hz
/// part on channel 6 gets its samples on time, and whether every segment reaches its nominal
/// rate. The same bus runs with that part in declaration order, first, and last in the Bus,
/// because the arbiter hands a free switch to whoever asks first and the order the
/// Bus walks its devices in decides who that is.
///
/// The numbers are printed, and the ones a policy is chosen on are checked, at the typical
/// loop of 245 us a turn: the 100 Hz part gets at least 99 % of its samples under ShareChannel,
/// 98.5 % under HoldChannel and 97.5 % under DrainChannel, with at most 2, 6 and 12 gaps of two
/// periods or more; every segment reaches 99 % of its nominal rate; the switch is held by
/// channel 6, the busiest, between 7 and 20 % of the time, by the sleepy channels 2 and 5 under
/// 2.5 % each, and by all channels together under 75 %. On a loop twice as slow the policies
/// rank ShareChannel, HoldChannel, DrainChannel (at least 96, 92 and 85 %), and where the part
/// sits in the Bus makes no more than 1 % of difference to it. The window is 10 s after a
/// second of warm-up: enough for the slowest part (10 s) to go once and for the rates to settle,
/// and short enough for the nine runs to stay under twenty seconds with the sanitizers on.
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <type_traits>

// clang-format off
#include <support/LogStubs.hpp>
#include "FakeBus.hpp"
#include "Check.hpp"
// clang-format on
#include <kvasir/Devices/I2C/Bus.hpp>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/Mux.hpp>
#include <kvasir/Devices/I2C/chips/Tca9548a.hpp>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

// -- timings: a part's period, and for a command or a sweep the conversion it waits -------------

struct Every10 {
    static constexpr std::chrono::milliseconds Period{10};
};

struct Every20 {
    static constexpr std::chrono::milliseconds Period{20};
};

struct Every50 {
    static constexpr std::chrono::milliseconds Period{50};
};

struct Every62 {
    static constexpr std::chrono::milliseconds Period{62};
};

struct Every78 {
    static constexpr std::chrono::milliseconds Period{78};
};

struct Every80 {
    static constexpr std::chrono::milliseconds Period{80};
};

struct Every97 {
    static constexpr std::chrono::milliseconds Period{97};
};

struct Every100 {
    static constexpr std::chrono::milliseconds Period{100};
};

struct Every110 {
    static constexpr std::chrono::milliseconds Period{110};
};

struct Every200 {
    static constexpr std::chrono::milliseconds Period{200};
};

struct Every250 {
    static constexpr std::chrono::milliseconds Period{250};
};

struct Every300 {
    static constexpr std::chrono::milliseconds Period{300};
};

struct Every450 {
    static constexpr std::chrono::milliseconds Period{450};
};

struct Every500 {
    static constexpr std::chrono::milliseconds Period{500};
};

struct Every1000 {
    static constexpr std::chrono::milliseconds Period{1000};
};

struct Every10000 {
    static constexpr std::chrono::milliseconds Period{10000};
};

struct Every100Wait1 : Every100 {
    static constexpr std::chrono::milliseconds Wait{1};
};

struct Every100Wait2 : Every100 {
    static constexpr std::chrono::milliseconds Wait{2};
};

struct Every1000Wait9 : Every1000 {
    static constexpr std::chrono::milliseconds Wait{9};
};

struct Every1000Wait12 : Every1000 {
    static constexpr std::chrono::milliseconds Wait{12};
};

struct Every1000Wait16 : Every1000 {
    static constexpr std::chrono::milliseconds Wait{16};
};

struct Every1000Wait50 : Every1000 {
    static constexpr std::chrono::milliseconds Wait{50};
};

struct Every1000Wait80 : Every1000 {
    static constexpr std::chrono::milliseconds Wait{80};
};

// -- synthetic parts: a period and the shape of the traffic, nothing else --------------------

/// One register read of N bytes: most parts.
template<Address7 A, typename T, std::uint8_t N>
struct Reg {
    static constexpr std::string_view Name          = "REG";
    static constexpr Address7         Address       = A;
    static constexpr std::size_t      RegisterBytes = 1;

    struct Data {
        static constexpr auto       Period = T::Period;
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = N, .offset = 0})};
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes b) { return b.u8(0); }
    };

    using Reads = List<Data>;
};

/// A bare read of N bytes (BH1750, MCP3221, the easyC boards, MCP4018).
template<Address7 A, typename T, std::uint8_t N>
struct Recv {
    static constexpr std::string_view Name          = "RECV";
    static constexpr Address7         Address       = A;
    static constexpr std::size_t      RegisterBytes = 0;

    struct Data {
        static constexpr auto       Period = T::Period;
        static constexpr std::array Steps{Step::receive({.count = N, .offset = 0})};
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes b) { return b.u8(0); }
    };

    using Reads = List<Data>;
};

/// A measurement command, T::Wait of conversion, then N bytes (Sensirion, AHT20, HTU21D).
template<Address7 A, typename T, std::uint8_t N>
struct Cmd {
    static constexpr std::string_view Name          = "CMD";
    static constexpr Address7         Address       = A;
    static constexpr std::size_t      RegisterBytes = 0;

    struct Data {
        static constexpr auto       Period = T::Period;
        static constexpr std::array Steps{
          Step::command({.payload = {0x24, 0x00}, .delay = T::Wait}
          ),
          Step::receive({             .count = N,      .offset = 0}
          )
        };
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes b) { return b.u8(0); }
    };

    using Reads = List<Data>;
};

/// An OLED page: the window command, then a transfer the size of the page's RAM write. It
/// models a panel that sends only the pages whose content changed, at the given rate.
template<Address7 A, typename T, std::uint8_t N>
struct Page {
    static constexpr std::string_view Name          = "PAGE";
    static constexpr Address7         Address       = A;
    static constexpr std::size_t      RegisterBytes = 0;

    struct Data {
        static constexpr auto       Period = T::Period;
        static constexpr std::array Steps{
          Step::command({.payload = {0x00, 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07}}
          ),
          Step::receive({.count = N, .offset = 0}
          )
        };
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes b) { return b.u8(0); }
    };

    using Reads = List<Data>;
};

/// Four single-shot conversions, each a config write, T::Wait, and a 2-byte read (ADS1x15).
template<Address7 A, typename T>
struct Sweep {
    static constexpr std::string_view Name          = "SWEEP";
    static constexpr Address7         Address       = A;
    static constexpr std::size_t      RegisterBytes = 1;

    struct Data {
        static constexpr auto       Period = T::Period;
        static constexpr std::array Steps{
          Step::write({.reg = 0x01, .payload = {0xC3, 0x83}, .delay = T::Wait}
          ),
          Step::read({.reg = 0x00,              .count = 2,      .offset = 0}
          ),
          Step::write({.reg = 0x01, .payload = {0xD3, 0x83}, .delay = T::Wait}
          ),
          Step::read({.reg = 0x00,              .count = 2,      .offset = 2}
          ),
          Step::write({.reg = 0x01, .payload = {0xE3, 0x83}, .delay = T::Wait}
          ),
          Step::read({.reg = 0x00,              .count = 2,      .offset = 4}
          ),
          Step::write({.reg = 0x01, .payload = {0xF3, 0x83}, .delay = T::Wait}
          ),
          Step::read({.reg = 0x00,              .count = 2,      .offset = 6}
          )
        };
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes b) { return b.u8(0); }
    };

    using Reads = List<Data>;
};

/// The VL53L1X's shape: the status, 15 ms, the 17-byte result, the interrupt clear.
template<Address7 A, typename T>
struct Ranging {
    static constexpr std::string_view Name          = "RANGING";
    static constexpr Address7         Address       = A;
    static constexpr std::size_t      RegisterBytes = 2;

    struct Data {
        static constexpr auto       Period = T::Period;
        static constexpr std::array Steps{
          Step::read({.reg = 0x0031, .count = 1, .offset = 0, .delay = 15ms}),
          Step::read({.reg = 0x0089, .count = 17, .offset = 1}),
          Step::write({.reg = 0x0086, .payload = {0x01}})};
        using Sample = std::uint8_t;

        [[nodiscard]] static constexpr Sample decode(Bytes b) { return b.u8(0); }
    };

    using Reads = List<Data>;
};

/// The bus as the Bus sees it: 400 kHz and a queue deeper than the parts on it.
struct Wire : FakeBus {
    static constexpr std::uint32_t BaudRate   = 400'000;
    static constexpr std::size_t   QueueDepth = 64;
};

using MuxDev = Device<Wire, FakeClock, Chips::Tca9548a>;

template<typename C>
using Up = Device<Wire, FakeClock, C>;

template<typename C, std::uint8_t Ch, typename Pol>
using On = Device<Wire, FakeClock, C, DefaultConfig, NoReset, MuxGate<MuxDev, Ch, Pol>>;

// -- the parts, in declaration order ---------------------------------------------------------

/// The IIS2DULPX: six bytes every 10 ms on channel 6.
template<typename Pol>
using TargetOf                                   = On<Reg<0x19, Every10, 6>, 6, Pol>;
constexpr std::chrono::microseconds TargetPeriod = 10ms;

template<typename... Ts>
struct Pack {};

/// Everything before the target in the Bus, behind gates of policy `Pol`: in front of the switch, channels 0..5,
/// and the LSM6DSO. Periods are example rates; an EEPROM (on demand), a DAC,
/// an LCD and an absent IMU put no cyclic traffic on the wire and are left out.
template<typename Pol>
using FrontOf = Pack<
  // in front of the switch: the MCP23017, the BH1750, the 64 x 48 OLED
  Up<Reg<0x20, Every50, 2>>,
  Up<Recv<0x23, Every200, 2>>,
  Up<Page<0x3D, Every97, 64>>,
  // CH0: VEML6030, TSL2591, APDS-9960, LTR-507, LTR390, HTU21D
  On<Reg<0x48, Every200, 2>, 0, Pol>,
  On<Reg<0x29, Every500, 4>, 0, Pol>,
  On<Reg<0x39, Every80, 4>, 0, Pol>,
  On<Reg<0x3A, Every110, 4>, 0, Pol>,
  On<Reg<0x53, Every300, 3>, 0, Pol>,
  On<Cmd<0x40, Every1000Wait50, 3>, 0, Pol>,
  // CH1: AS7341, VL53L1X, OPT4048, TMP119, INA3221, a 128x64 OLED
  On<Reg<0x39, Every200, 12>, 1, Pol>,
  On<Ranging<0x29, Every100>, 1, Pol>,
  On<Reg<0x44, Every450, 8>, 1, Pol>,
  On<Reg<0x48, Every1000, 2>, 1, Pol>,
  On<Reg<0x40, Every200, 12>, 1, Pol>,
  On<Page<0x3C, Every78, 128>, 1, Pol>,
  // CH2: SHT41, SHTC3, AHT20, DPS310, MCP9808, TMP117
  On<Cmd<0x44, Every1000Wait9, 6>, 2, Pol>,
  // Not at 0x70: that is the switch, which is on every channel's wire (Bus: addressesDistinct).
  On<Cmd<0x72, Every1000Wait12, 6>, 2, Pol>,
  On<Cmd<0x38, Every1000Wait80, 7>, 2, Pol>,
  On<Reg<0x77, Every500, 6>, 2, Pol>,
  On<Reg<0x18, Every250, 2>, 2, Pol>,
  On<Reg<0x48, Every1000, 2>, 2, Pol>,
  // CH3: ADS1115, MCP4018, INA219, an ADC, SHT45
  On<Sweep<0x48, Every100Wait2>, 3, Pol>,
  On<Recv<0x2F, Every500, 1>, 3, Pol>,
  On<Reg<0x40, Every100, 4>, 3, Pol>,
  On<Recv<0x30, Every50, 2>, 3, Pol>,
  On<Cmd<0x44, Every1000Wait9, 6>, 3, Pol>,
  // CH4: ADS1015, INA228, an encoder, DS3502, a vibration sensor
  On<Sweep<0x48, Every100Wait1>, 4, Pol>,
  On<Reg<0x40, Every500, 9>, 4, Pol>,
  On<Recv<0x30, Every20, 4>, 4, Pol>,
  On<Reg<0x28, Every500, 1>, 4, Pol>,
  On<Recv<0x4D, Every20, 2>, 4, Pol>,
  // CH5: ADS1219, SHT31, a 128x64 OLED, DHT20
  On<Reg<0x40, Every200, 3>, 5, Pol>,
  On<Cmd<0x44, Every1000Wait16, 6>, 5, Pol>,
  On<Page<0x3C, Every10000, 128>, 5, Pol>,
  On<Cmd<0x38, Every1000Wait80, 7>, 5, Pol>,
  // CH6: the LSM6DSO
  On<Reg<0x6B, Every20, 14>, 6, Pol>>;

/// Everything after it: the rest of channel 6 and channel 7.
template<typename Pol>
using BackOf = Pack<
  // CH6: AS5600, TLV493D, MPR121, INA237
  On<Reg<0x36, Every20, 2>, 6, Pol>,
  On<Recv<0x5E, Every50, 7>, 6, Pol>,
  On<Reg<0x5A, Every50, 2>, 6, Pol>,
  On<Reg<0x40, Every500, 2>, 6, Pol>,
  // CH7: LSM303AGR accel and mag, the 128 x 32 OLED, the PCA9956B
  On<Reg<0x19, Every50, 6>, 7, Pol>,
  On<Reg<0x1E, Every50, 6>, 7, Pol>,
  On<Page<0x3C, Every62, 128>, 7, Pol>,
  On<Reg<0x3F, Every500, 2>, 7, Pol>>;

template<typename A, typename B, typename C>
struct MakeBus;

template<typename... A, typename... B, typename... C>
struct MakeBus<Pack<A...>, Pack<B...>, Pack<C...>> {
    using type = Kvasir::I2C::Bus<Wire, FakeClock, MuxDev, A..., B..., C...>;
};

template<typename Pol>
using DeclOrder = typename MakeBus<FrontOf<Pol>, Pack<TargetOf<Pol>>, BackOf<Pol>>::type;
template<typename Pol>
using TargetFirst = typename MakeBus<Pack<TargetOf<Pol>>, FrontOf<Pol>, BackOf<Pol>>::type;
template<typename Pol>
using TargetLast = typename MakeBus<FrontOf<Pol>, BackOf<Pol>, Pack<TargetOf<Pol>>>::type;

// -- the simulation ------------------------------------------------------------------------

constexpr std::chrono::microseconds WarmUp = 1s;
constexpr std::chrono::microseconds Run    = 10s;

/// What a run has to reach: the 100 Hz part's share of its samples, how many late gaps it may
/// have, and the rate every segment has to reach. The held-time bands are the same for every run.
struct Bands {
    double        targetPercentMin{};
    std::uint32_t lateGapsMax{};
    double        segmentPercentMin{};
};

constexpr std::size_t Segments = 9;   // channels 0..7, and in front of the switch as 8

std::chrono::microseconds now() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
      FakeClock::now().time_since_epoch());
}

/// A transaction's time on a 400 kHz wire: nine bits a byte for the address, the bytes sent
/// and, for a read, the repeated start's address and the bytes received, plus START and STOP,
/// and ~100 us from submit to the driver's first interrupt.
std::chrono::microseconds wireTime(FakeBus::Request const& r) {
    std::size_t bytes = 1 + r.sendData.size();
    if(!r.receiveData.empty()) { bytes += 1 + r.receiveData.size(); }
    auto const bits = static_cast<std::int64_t>(bytes * 9 + 2);
    return std::chrono::microseconds{1s} * bits / static_cast<std::int64_t>(Wire::BaudRate) + 100us;
}

/// A main-loop turn: `turn` for the handlers, plus `render` in the turn that draws a frame,
/// which one in every `renderEvery` does.
struct Loop {
    std::chrono::microseconds turn{};
    std::chrono::microseconds render{};
    std::chrono::microseconds renderEvery{};
};

struct Outcome {
    std::uint32_t                targetSamples{};
    std::uint32_t                lateGaps{};   // gaps of two periods or more
    std::chrono::microseconds    longestGap{};
    std::array<double, Segments> segmentPercent{};
    std::array<double, 8>        heldPercent{};
    std::array<double, 8>        grantsPerSecond{};
};

template<typename Pol,
         template<typename> typename Order>
Outcome simulate(Loop const& loop) {
    using BusT   = Order<Pol>;
    using Target = TargetOf<Pol>;
    FakeBus::reset();
    FakeClock::current = FakeClock::time_point{} + 1s;
    Log::reset();

    // The gates were bound to the switch and to the Bus's own arbiter for it when the Bus was
    // made; the numbers below are read off that arbiter.
    auto  bus     = std::make_unique<BusT>();
    auto& arbiter = bus->template arbiter<0>();
    auto& target  = bus->template get<Target>();

    Outcome                                  out{};
    std::array<std::uint32_t, BusT::Count>   base{};
    auto                                     arbiterBase = arbiter;
    std::deque<std::chrono::microseconds>    doneAt{};   // per pending request, in queue order
    std::chrono::microseconds                busyUntil{};
    std::uint8_t                             control = 0;
    std::optional<std::chrono::microseconds> lastSample{};
    std::uint32_t                            lastSamples = 0;
    bool                                     measuring   = false;
    auto const                               start       = now();
    auto                                     nextRender  = start;

    for(;;) {
        auto const at = now();
        if(at - start >= WarmUp + Run) { break; }
        if(!measuring && at - start >= WarmUp) {
            measuring       = true;
            std::size_t idx = 0;
            bus->forEach([&](auto const& d) { base[idx++] = d.samples(); });
            lastSamples = target.samples();
            arbiterBase = arbiter;
        }

        bus->handler();

        if(measuring && target.samples() != lastSamples) {
            lastSamples = target.samples();
            if(lastSample) {
                auto const gap = at - *lastSample;
                out.longestGap = std::max(out.longestGap, gap);
                if(gap >= 2 * TargetPeriod) { ++out.lateGaps; }
            }
            lastSample = at;
        }

        // What was submitted this turn goes on the wire behind what is already there.
        for(std::size_t i = doneAt.size(); i < FakeBus::pending.size(); ++i) {
            busyUntil = std::max(busyUntil, at) + wireTime(FakeBus::pending[i]);
            doneAt.push_back(busyUntil);
        }
        auto turn = loop.turn;
        if(loop.render > std::chrono::microseconds::zero() && at >= nextRender) {
            nextRender = at + loop.renderEvery;
            turn += loop.render;
        }
        FakeClock::current += turn;
        // and the interrupt finishes whatever the wire got through meanwhile.
        auto const after = now();
        while(!doneAt.empty() && doneAt.front() <= after) {
            doneAt.pop_front();
            // The switch keeps its control byte and answers the engine's read-back with it.
            auto const& r = FakeBus::pending.front();
            if(r.address == MuxDev::Address) {
                if(!r.sendData.empty()) { control = static_cast<std::uint8_t>(r.sendData[0]); }
                for(auto& b : r.receiveData) { b = std::byte{control}; }
            }
            FakeBus::complete(FakeBus::Result::succeeded);
        }
        FakeBus::log.clear();
    }

    std::array<double, Segments> got{};
    std::array<double, Segments> owed{};
    std::size_t                  idx = 0;
    bus->forEach([&](auto const& d) {
        using D      = std::remove_cvref_t<decltype(d)>;
        auto const n = d.samples() - base[idx++];
        if constexpr(requires { D::Chip::Data::Period; }) {
            std::size_t seg = Segments - 1;
            if constexpr(D::GateT::Gated) {
                seg = static_cast<std::size_t>(std::countr_zero(D::GateT::Mask));
            }
            got[seg] += static_cast<double>(n);
            owed[seg] += std::chrono::duration<double>{Run} / D::Chip::Data::Period;
        }
    });
    for(std::size_t s = 0; s < Segments; ++s) {
        out.segmentPercent[s] = owed[s] > 0 ? 100.0 * got[s] / owed[s] : 100.0;
    }
    for(std::size_t c = 0; c < 8; ++c) {
        out.heldPercent[c] = 100.0
                           * std::chrono::duration<double>{arbiter.held[c] - arbiterBase.held[c]}
                           / std::chrono::duration<double>{Run};
        out.grantsPerSecond[c] = static_cast<double>(arbiter.grants[c] - arbiterBase.grants[c])
                               / std::chrono::duration<double>{Run}.count();
    }
    out.targetSamples
      = target.samples() - base[detail::IndexOf<Target, typename BusT::Devices>::value];
    return out;
}

void report(char const*    order,
            Loop const&    loop,
            Outcome const& o,
            Bands const&   bands) {
    auto const owed = static_cast<std::uint32_t>(Run / TargetPeriod);
    std::printf("    %-17s turn %4lld us | target %4u of %4u, %3u late gap(s), longest %5.1f ms\n",
                order,
                static_cast<long long>(loop.turn.count()),
                o.targetSamples,
                owed,
                o.lateGaps,
                std::chrono::duration<double, std::milli>{o.longestGap}.count());
    std::printf("      rate %%   ");
    for(std::size_t s = 0; s < Segments; ++s) {
        std::printf(" %c %5.1f",
                    s == Segments - 1 ? 'U' : static_cast<char>('0' + s),
                    o.segmentPercent[s]);
    }
    std::printf("\n      held %%   ");
    for(std::size_t c = 0; c < 8; ++c) {
        std::printf(" %c %5.1f", static_cast<char>('0' + c), o.heldPercent[c]);
    }
    std::printf("\n      grants/s ");
    for(std::size_t c = 0; c < 8; ++c) {
        std::printf(" %c %5.0f", static_cast<char>('0' + c), o.grantsPerSecond[c]);
    }
    std::printf("\n");

    check(o.targetSamples > 0, "the harness ran the target");
    auto const targetPercent
      = 100.0 * static_cast<double>(o.targetSamples) / static_cast<double>(owed);
    check(targetPercent >= bands.targetPercentMin, "the 100 Hz part gets its share of samples");
    check(o.lateGaps <= bands.lateGapsMax, "and no more late gaps than the policy is allowed");
    for(std::size_t s = 0; s < Segments; ++s) {
        check(o.segmentPercent[s] >= bands.segmentPercentMin, "every segment reaches its rate");
    }
    check(o.segmentPercent[Segments - 1] >= 99.5, "the wire in front of the switch is not slowed");
    check(o.heldPercent[6] >= 7.0 && o.heldPercent[6] <= 20.0,
          "the busiest channel holds the switch between 7 and 20 % of the time");
    check(o.heldPercent[2] <= 2.5 && o.heldPercent[5] <= 2.5,
          "the sleepy channels under 2.5 % each");
    double held = 0.0;
    for(auto const h : o.heldPercent) { held += h; }
    check(held <= 75.0, "and all of them together under 75 %");
}

}   // namespace

int main() {
    constexpr Loop Typical{.turn = 245us, .render = 1200us, .renderEvery = 25ms};
    constexpr Loop Fast{.turn = 120us, .render = 1200us, .renderEvery = 25ms};
    constexpr Loop Slow{.turn = 490us, .render = 1200us, .renderEvery = 25ms};

    constexpr Bands ShareTypical{.targetPercentMin  = 99.0,
                                 .lateGapsMax       = 2,
                                 .segmentPercentMin = 99.0};
    constexpr Bands HoldTypical{.targetPercentMin  = 98.5,
                                .lateGapsMax       = 6,
                                .segmentPercentMin = 99.0};
    constexpr Bands DrainTypical{.targetPercentMin  = 97.5,
                                 .lateGapsMax       = 12,
                                 .segmentPercentMin = 99.0};
    constexpr Bands ShareSlow{.targetPercentMin  = 96.0,
                              .lateGapsMax       = 12,
                              .segmentPercentMin = 98.0};
    constexpr Bands HoldSlow{.targetPercentMin  = 92.0,
                             .lateGapsMax       = 40,
                             .segmentPercentMin = 96.0};
    constexpr Bands DrainSlow{.targetPercentMin  = 85.0,
                              .lateGapsMax       = 80,
                              .segmentPercentMin = 94.0};

    testCase("Mux simulation: ShareChannel, the 100 Hz part at three places in the Bus");
    auto const declared = simulate<ShareChannel, DeclOrder>(Typical);
    auto const first    = simulate<ShareChannel, TargetFirst>(Typical);
    auto const last     = simulate<ShareChannel, TargetLast>(Typical);
    report("declaration order", Typical, declared, ShareTypical);
    report("target first", Typical, first, ShareTypical);
    report("target last", Typical, last, ShareTypical);
    {
        auto const spread
          = std::max({declared.targetSamples, first.targetSamples, last.targetSamples})
          - std::min({declared.targetSamples, first.targetSamples, last.targetSamples});
        check(spread <= Run / TargetPeriod / 100,
              "where the part sits makes at most 1 % of difference");
    }

    testCase("Mux simulation: ShareChannel, a loop twice as fast and twice as slow");
    report("declaration order", Fast, simulate<ShareChannel, DeclOrder>(Fast), ShareTypical);
    auto const shareSlow = simulate<ShareChannel, DeclOrder>(Slow);
    report("declaration order", Slow, shareSlow, ShareSlow);

    testCase("Mux simulation: HoldChannel, a typical loop and one twice as slow");
    report("declaration order", Typical, simulate<HoldChannel, DeclOrder>(Typical), HoldTypical);
    auto const holdSlow = simulate<HoldChannel, DeclOrder>(Slow);
    report("declaration order", Slow, holdSlow, HoldSlow);

    testCase("Mux simulation: DrainChannel, a typical loop and one twice as slow");
    report("declaration order", Typical, simulate<DrainChannel, DeclOrder>(Typical), DrainTypical);
    auto const drainSlow = simulate<DrainChannel, DeclOrder>(Slow);
    report("declaration order", Slow, drainSlow, DrainSlow);

    testCase(
      "Mux simulation: on a slow loop the policies rank Share, Hold, Drain for the fast part");
    check(shareSlow.targetSamples >= holdSlow.targetSamples
            && holdSlow.targetSamples >= drainSlow.targetSamples,
          "ShareChannel changes hands most often and so serves the 100 Hz part best");
    check(shareSlow.lateGaps <= holdSlow.lateGaps && holdSlow.lateGaps <= drainSlow.lateGaps,
          "and has the fewest late gaps");
    return finish();
}
