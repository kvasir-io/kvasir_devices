/// Every SPI driver with MISO floating low and high: none may take that for a part or write a
/// configuration to it. Also the one place every driver is instantiated on the host.
// clang-format off
#include <support/LogStubs.hpp>
#include <support/Check.hpp>
// clang-format on

#include "FakeQueuedSpi.hpp"
#include "FakeSpi.hpp"

#include <chrono>
#include <cstddef>
#include <kvasir/Devices/SPI/SdCard.hpp>
#include <kvasir/Devices/SPI/chips/Ad7124.hpp>
#include <kvasir/Devices/SPI/chips/Ads131m0X.hpp>
#include <kvasir/Devices/SPI/chips/Ads8675.hpp>
#include <kvasir/Devices/SPI/chips/Bme280.hpp>
#include <kvasir/Devices/SPI/chips/Max31865.hpp>
#include <kvasir/Devices/SPI/chips/Max7219.hpp>
#include <kvasir/Devices/SPI/chips/Mpu9250.hpp>
#include <kvasir/Devices/SPI/chips/NorFlash.hpp>
#include <support/FakeClock.hpp>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using Kvasir::Test::Spi::Pins;
namespace A = Kvasir::SPI;

namespace {

struct Tag {};

using Bus = QueuedSpi::Bus<Tag>;

struct Adc131Config {
    static constexpr std::size_t Channels = 2;
};

std::uint8_t level{};

void fresh(std::uint8_t l) {
    level = l;
    Bus::reset();
    Pins::reset();
    FakeClock::set(1s);
    Log::reset();
    Bus::floating     = l;
    Bus::partSelected = [] { return !Pins::level[Spi::Cs::id]; };
}

/// Runs `turn` for `span`; DRDY/RVS float to MISO's level, so either sense is met in one run.
template<typename Turn>
void run(std::chrono::milliseconds span,
         Turn                      turn) {
    auto const until = FakeClock::now() + span;
    while(FakeClock::now() < until) {
        FakeClock::advance(1ms);
        Pins::level[Spi::Drdy::id] = level != 0;
        Pins::level[Spi::Rvs::id]  = level != 0;
        Bus::handler();
        turn();
        Bus::tick();
    }
}

/// Frames longer than a command byte with CS low: a configuration written to nothing.
std::size_t configurationFrames() {
    std::size_t n = 0;
    for(auto const& f : Bus::frames) { n += (f.selected && f.mosi.size() > 2) ? 1U : 0U; }
    return n;
}

void all(std::uint8_t l) {
    testCase(l == 0 ? "no part, MISO low" : "no part, MISO high");

    fresh(l);
    A::Device<Bus, FakeClock, A::Chips::Bme280, Spi::Cs> bme{};
    run(5s, [&] { bme.handler(); });
    check(!bme.answering() && configurationFrames() == 0, "BME280: not answering, nothing written");
    check(Pins::level[Spi::Cs::id], "BME280: CS high");

    fresh(l);
    A::Device<Bus, FakeClock, A::Chips::Mpu9250, Spi::Cs> mpu{};
    run(5s, [&] { mpu.handler(); });
    check(!mpu.answering() && configurationFrames() == 0,
          "MPU-9250: not answering, nothing written");

    fresh(l);
    A::Max31865<Bus, FakeClock, Spi::Cs, Spi::Drdy> rtd{};
    run(5s, [&] { rtd.handler(); });
    check(!rtd.answering() && !rtd.temperature(), "MAX31865: not answering, no reading");
    check(Pins::level[Spi::Cs::id], "MAX31865: CS high");

    fresh(l);
    A::Ad7124<Bus, FakeClock, Spi::Cs> adc{};
    run(5s, [&] { adc.handler(); });
    check(!adc.present() && adc.conversions() == 0, "AD7124: not answering, nothing converted");
    std::size_t configured = 0;
    for(auto const& f : Bus::frames) {
        configured += (f.mosi.size() > 1 && f.mosi[0] == 0x07) ? 1U : 0U;
    }
    checkEq(configured, 0U, "AD7124: no ERROR_EN written to nothing");

    fresh(l);
    A::NorFlash<Bus, FakeClock, Spi::Cs> flash{};
    static_cast<void>(flash.readJedec());
    run(1s, [&] {
        flash.handler();
        if(flash.takeDone() && !flash.present()) { static_cast<void>(flash.readJedec()); }
    });
    check(!flash.present(), "NOR flash: no JEDEC id from nothing");
    check(flash.absent(), "NOR flash: reported absent");

    fresh(l);
    A::Max7219<Bus, FakeClock, Spi::Cs> leds{};
    run(2s, [&] { leds.handler(); });
    check(leds.answering(), "MAX7219: write-only, so the frames going out is all it can say");
    check(Pins::level[Spi::Cs::id], "MAX7219: LOAD high between frames");

    fresh(l);
    std::uint32_t                                           got = 0;
    A::Ads8675<Bus, FakeClock, Spi::Cs, Spi::Rvs, Spi::Rst> ads{
      [&](Kvasir::Units::MicroVolt) { ++got; }};
    run(3s, [&] {
        ads.handler();
        ads.sampleCallback();
        ads.pinInterrupt();
    });
    check(!ads.ready() && got == 0, "ADS8675: RANGE_SEL not read back, no samples");

    fresh(l);
    A::Ads131m0X<Bus, FakeClock, Spi::Cs, Spi::Drdy, Adc131Config> m0x{};
    run(3s, [&] { m0x.handler(); });
    check(!m0x.present() && !m0x.valid(), "ADS131M0x: no reset acknowledge, no reading");
    check(Pins::level[Spi::Cs::id], "ADS131M0x: CS high");

    fresh(l);
    A::SdCard<Bus, FakeClock, Spi::Cs> card{};
    run(3s, [&] { card.handler(); });
    check(!card.up(), "SD card: not up");
}

}   // namespace

int main() {
    all(0x00);
    all(0xFF);
    return finish();
}
