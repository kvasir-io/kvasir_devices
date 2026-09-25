/// Accelerometers, gyroscopes, magnetometers and the angle sensor: each description
/// against a model of the part on the wire.
#include "Harness.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <kvasir/Devices/I2C/Device.hpp>
#include <kvasir/Devices/I2C/chips/All.hpp>
#include <kvasir/Devices/Samples.hpp>
#include <map>
#include <memory>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;
using namespace Kvasir::Test;
using namespace Kvasir::I2C;

namespace {

void mpu6050() {
    testCase("MPU6050");
    fresh();
    RegisterModel<1> m{0x68};
    m.set(0x75, {0x68});
    m.set(0x3A, {0x01});   // INT_STATUS: DATA_RDY_INT
    m.set(0x3B,
          {0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xFF, 0x7D, 0x00, 0x00, 0x00, 0x00});
    for(std::uint32_t r = 0x3A; r <= 0x48; ++r) { m.readOnly.push_back(r); }
    m.readOnly.push_back(0x75);
    FakeBus::respond = std::ref(m);
    Dev<Chips::Mpu6050> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 1s), "first sample");
    check(d.identified(), "WHO_AM_I");
    check(writes()
            == std::vector<std::vector<std::uint8_t>>{{0x6B, 0x80}, {0x6B, 0x01}, {0x19, 0x09}, {0x1A, 0x03}, {0x1B, 0x00}, {0x1C, 0x00}, {0x38, 0x01}, {0x1C, 0x00}, {0x1B, 0x00}},
          "bring-up writes in order, INT_ENABLE last, then the two range groups' Initial");
    check(FakeBus::log[0].isRead() && FakeBus::log[0].sent == std::vector<std::uint8_t>{0x75},
          "WHO_AM_I read before anything is written");
    // [0] the engine's identity read -- the only read of WHO_AM_I --, [1] the reset, [2] the wake
    check(FakeBus::log[1].isWrite() && FakeBus::log[2].at - FakeBus::log[1].at >= 100ms,
          "100 ms after the reset");
    checkEq(d.latest().accel[2], 1000000, "the model's frame reached the decode");
    // DATA_RDY_INT, then accel z 16384 (1 g), temperature raw 0, gyro x -131 (-1 deg/s)
    static_assert([] {
        auto const f   = frame(0x01,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x40,
                               0x00,
                               0x00,
                               0x00,
                               0xFF,
                               0x7D,
                               0x00,
                               0x00,
                               0x00,
                               0x00);
        auto const got = Chips::Mpu6050::Motion::decode(Bytes{f}, Chips::Mpu6050::State{});
        return isOk(got) && equal(got.value.accel[2], 1000000) && equal(got.value.gyro[0], -1000)
            && equal(got.value.temperature, 3653);
    }());
    {
        testCase("MPU6050 at +-16 g and 2000 dps");
        RegisterModel<1> w{0x68};
        w.set(0x75, {0x68});
        w.set(0x3A, {0x01});
        w.set(0x3B,
              {0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xFF, 0x7D, 0x00, 0x00, 0x00, 0x00});
        for(std::uint32_t r = 0x3A; r <= 0x48; ++r) { w.readOnly.push_back(r); }
        w.readOnly.push_back(0x75);
        auto const saved = FakeBus::respond;
        FakeBus::respond = std::ref(w);
        using WideMpu    = Chips::Mpu6050x<Chips::Mpu6050Detail::AccelRange::g16,
                                           Chips::Mpu6050Detail::GyroRange::dps2000>;
        Dev<WideMpu> dw{};
        check(runUntil(dw, [&] { return dw.samples() == 1; }, 1s), "first sample");
        check(w.word(0x1B) == 0x18 && w.word(0x1C) == 0x18, "FS_SEL 3, AFS_SEL 3");
        checkEq(dw.latest().accel[2],
                8000000,
                "16384 counts at +-16 g is 8 g: the State the bring-up set");
        // the same frame at AFS_SEL 3 and FS_SEL 3: 8 g, and -131 counts at 16.4 LSB/(deg/s)
        static_assert([] {
            auto const            f = frame(0x01,
                                            0x00,
                                            0x00,
                                            0x00,
                                            0x00,
                                            0x40,
                                            0x00,
                                            0x00,
                                            0x00,
                                            0xFF,
                                            0x7D,
                                            0x00,
                                            0x00,
                                            0x00,
                                            0x00);
            Chips::Mpu6050::State st{};
            st.accelRange  = 3;
            st.gyroRange   = 3;
            auto const got = Chips::Mpu6050::Motion::decode(Bytes{f}, st);
            return isOk(got) && equal(got.value.accel[2], 8000000)
                && equal(got.value.gyro[0], -7987);
        }());
        FakeBus::respond = saved;
    }
    auto const n = d.samples();
    runFor(d, 1s);
    checkEq(d.samples() - n, 50U, "50 samples a second");
    if(failures != 0) { dump(); }
}

void lsm303agr() {
    testCase("LSM303AGR");
    fresh();
    RegisterModel<1> la{0x19};
    la.set(0x0F, {0x33});
    la.set(0x27,
           {0x08,
            0x00,
            0x40,
            0x00,
            0xC0,
            0x00,
            0x00});               // STATUS_REG_A ZYXDA; x = 16384, y = -16384, z = 0
    la.set(0x0C, {0x34, 0x12});   // temperature raw 0x1234
    la.readOnly = {0x0F, 0x0C, 0x0D, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
    // the accelerometer takes auto-increment as the sub-address MSB
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(a == 0x19 && !sent.empty() && !recv.empty()) {
                std::array<std::byte, 1> r{
                  static_cast<std::byte>(static_cast<std::uint8_t>(sent[0]) & 0x7FU)};
                return la(a, r, recv);
            }
            return la(a, sent, recv);
        };
    Dev<Chips::Lsm303agrAccel<>> l3{};
    check(runUntil(l3, [&] { return l3.valid(); }, 500ms), "first sample");
    check(l3.identified(), "WHO_AM_I_A 0x33");
    check(hasWrite({0x23, 0x88}), "CTRL_REG4_A: BDU and high resolution at +-2 g");
    check(hasWrite({0x1F, 0xC0}), "and the temperature sensor enabled");
    // the burst must address 0xA7, not 0x27, or only one register would be read
    {
        bool inc = false;
        for(auto const& tr : FakeBus::log) {
            if(tr.isRead() && tr.sent == std::vector<std::uint8_t>{0xA7} && tr.recvLen == 7) {
                inc = true;
            }
        }
        check(inc, "the seven-byte burst from STATUS_REG_A sets the auto-increment bit: 0xA7");
    }
    // 12 bits left-aligned, then ST's (lsb/16) x 0.98 mg
    checkEq(l3.latest().x, 1003520, "the model's frame reached the decode");
    checkEq(l3.latest().temperature, 43203, "OUT_TEMP_A's step lands at offset 7");
    // ZYXDA, x 16384 and y -16384 left-aligned 12 bit at 0.98 mg, then OUT_TEMP 0x1234
    static_assert([] {
        auto const f = frame(0x08, 0x00, 0x40, 0x00, 0xC0, 0x00, 0x00, 0x34, 0x12);
        auto const got
          = Chips::Lsm303agrAccel<>::Motion::decode(Bytes{f}, Chips::Lsm303agrAccel<>::State{});
        return isOk(got) && equal(got.value.x, 1003520) && equal(got.value.y, -1003520)
            && equal(got.value.z, 0) && equal(got.value.temperature, 43203);
    }());

    fresh();
    RegisterModel<1> lm3{0x1E};
    lm3.set(0x4F, {0x40});
    lm3.set(0x67, {0x08, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00});   // STATUS_REG_M zyxda; x = 1000
    lm3.readOnly = {0x4F, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D};
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(a == 0x1E && !sent.empty() && !recv.empty()) {
                std::array<std::byte, 1> r{
                  static_cast<std::byte>(static_cast<std::uint8_t>(sent[0]) & 0x7FU)};
                return lm3(a, r, recv);
            }
            return lm3(a, sent, recv);
        };
    Dev<Chips::Lsm303agrMag<>> l3m{};
    check(runUntil(l3m, [&] { return l3m.valid(); }, 500ms), "magnetometer sample");
    check(l3m.identified(), "WHO_AM_I_M 0x40");
    check(hasWrite({0x60, 0x88}), "CFG_REG_A_M out of power-down into continuous, 50 Hz");
    check(hasWrite({0x61, 0x02}), "CFG_REG_B_M: offset cancellation");
    {
        bool plain = false;
        for(auto const& tr : FakeBus::log) {
            plain = plain
                 || (tr.isRead() && tr.sent == std::vector<std::uint8_t>{0x67} && tr.recvLen == 7);
        }
        check(plain, "the magnetometer burst from 0x67, without the MSB");
    }
    checkEq(l3m.latest().x, 150000, "the model's frame reached the decode");
    // Zyxda, x 1000 counts at 1.5 mgauss: 150 uT
    static_assert([] {
        auto const f   = frame(0x08, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00);
        auto const got = Chips::Lsm303agrMag<>::Field::decode(Bytes{f}, {});
        return isOk(got) && equal(got.value.x, 150000) && equal(got.value.y, 0)
            && equal(got.value.z, 0);
    }());
    if(failures != 0) { dump(); }
}

void iis2dulpx() {
    testCase("IIS2DULPX");
    fresh();
    RegisterModel<1> iis{0x18};
    iis.set(0x0F, {0x47});
    // x = 16384 (1 g at +-2 g), y = -16384, z = 0, temperature = +3555 lsb (35 degC)
    iis.set(0x28, {0x00, 0x40, 0x00, 0xC0, 0x00, 0x00, 0xE3, 0x0D});
    iis.set(0x25, {0x01});   // STATUS: DRDY
    iis.readOnly = {0x0F, 0x25, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F};
    // deep power-down: the address is NAKed while the part powers up
    int asleep = 2;
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(asleep > 0) {
                --asleep;
                return FakeBus::Result::notAcknowledged;
            }
            return iis(a, sent, recv);
        };
    Dev<Chips::Iis2dulpx<>> iu{};
    check(runUntil(iu, [&] { return iu.valid(); }, 500ms), "first sample");
    check(iu.identified(), "WHO_AM_I 0x47");
    checkEq(iu.wakeRetries(), 2U, "the NAKs of the power-up were retried");
    check(!hasWrite({0x3E, 0x01}), "no SOFT_PD write: that is the SPI way out");
    check(hasWrite({0x10, 0x20}), "then the software reset");
    check(hasWrite({0x12, 0x04}), "CTRL3: HP_EN for the high-performance ODR");
    check(hasWrite({0x10, 0x10}), "then IF_ADD_INC so the burst read walks the registers");
    check(hasWrite({0x14, 0x80}), "CTRL5: 100 Hz high performance at +-2 g");
    check(hasWrite({0x13, 0x20}), "CTRL4: BDU");
    checkEq(iu.latest().x, 999424, "the model's frame reached the decode");
    // x 16384, y -16384, z 0 at 61 ug/LSB, temperature 3555 lsb (35 degC)
    static_assert([] {
        auto const f   = frame(0x00, 0x40, 0x00, 0xC0, 0x00, 0x00, 0xE3, 0x0D, 0x01);
        auto const out = Chips::Iis2dulpx<>::Motion::decode(Bytes{f}, Chips::Iis2dulpx<>::State{});
        auto const got = out.value;
        auto       old = f;
        old[8]         = std::byte{0x00};   // DRDY clear: the sample already read
        return isOk(out) && equal(got.x, 999424) && equal(got.y, -999424) && equal(got.z, 0)
            && equal(got.temperature, 35000)
            && isUnchanged(
                 Chips::Iis2dulpx<>::Motion::decode(Bytes{old}, Chips::Iis2dulpx<>::State{}));
    }());
    // BW the rate allows: none on the ultra-low-power rates, ODR/16 only at 6 Hz low power,
    // at least ODR/8 at 12.5 Hz; the high-performance rates take what is asked
    static_assert(Chips::Iis2dulpx<Chips::Iis2dulpxDetail::FullScale::g2,
                                   Chips::Iis2dulpxDetail::Odr::ulp25,
                                   Chips::Iis2dulpxDetail::Bandwidth::div8>::Ctrl5
                    == 0x30
                  && Chips::Iis2dulpx<Chips::Iis2dulpxDetail::FullScale::g2,
                                      Chips::Iis2dulpxDetail::Odr::lp6>::Ctrl5
                       == 0x4C
                  && Chips::Iis2dulpx<Chips::Iis2dulpxDetail::FullScale::g2,
                                      Chips::Iis2dulpxDetail::Odr::lp12_5,
                                      Chips::Iis2dulpxDetail::Bandwidth::div4>::Ctrl5
                       == 0x58
                  && Chips::Iis2dulpx<Chips::Iis2dulpxDetail::FullScale::g2,
                                      Chips::Iis2dulpxDetail::Odr::hp6,
                                      Chips::Iis2dulpxDetail::Bandwidth::div2>::Ctrl5
                       == 0x40);
    checkEq(Chips::Iis2dulpx<Chips::Iis2dulpxDetail::FullScale::g16>::AccelPerCount[3],
            488,
            "0.488 mg/LSB at +-16 g");
    check(Chips::Iis2dulpx<>::OutputPeriod == 10ms, "100 Hz is a 10 ms period");
    check(Chips::Iis2dulpx<Chips::Iis2dulpxDetail::FullScale::g2,
                           Chips::Iis2dulpxDetail::Odr::hp400>::OutputPeriod
            == 3ms,
          "400 Hz rounds up to 3 ms");
    iu.set<Chips::Iis2dulpx<>::Config>(0x81);   // +-4 g
    check(runUntil(iu, [&] { return iis.word(0x14) == 0x81; }, 200ms), "CTRL5 at +-4 g written");
    auto const iisSeq = iu.seq();
    check(runUntil(iu, [&] { return iu.seq() > iisSeq + 1; }, 200ms), "and sampled after it");
    checkEq(iu.latest().x,
            1998848,
            "16384 counts at 122 ug/LSB: decode follows the written full scale");
    // the same frame with CTRL5 at +-4 g
    static_assert([] {
        auto const                f = frame(0x00, 0x40, 0x00, 0xC0, 0x00, 0x00, 0xE3, 0x0D, 0x01);
        Chips::Iis2dulpx<>::State st{};
        st.fs          = 1;
        auto const got = Chips::Iis2dulpx<>::Motion::decode(Bytes{f}, st).value;
        return equal(got.x, 1998848) && equal(got.y, -1998848);
    }());
    iu.set<Chips::Iis2dulpx<>::Config>(0x71);   // 50 Hz, +-4 g
    check(runUntil(iu, [&] { return iis.word(0x14) == 0x71; }, 200ms), "CTRL5 at 50 Hz written");
    runFor(iu, 50ms);
    {
        auto const n = iu.samples<Chips::Iis2dulpx<>::Motion>();
        runFor(iu, 1s);
        auto const perSecond = iu.samples<Chips::Iis2dulpx<>::Motion>() - n;
        check(perSecond >= 48 && perSecond <= 51,
              "the Motion group follows CTRL5 to 50 samples a second");
    }
    // acceleration and temperature come out of one transaction
    {
        bool burst = false;
        for(auto const& tr : FakeBus::log) {
            if(tr.isRead() && tr.sent == std::vector<std::uint8_t>{0x28} && tr.recvLen == 8) {
                burst = true;
            }
        }
        check(burst, "one eight-byte burst covers 0x28..0x2F");
    }
    if(failures != 0) { dump(); }
}

void lsm9ds1() {
    testCase("LSM9DS1");
    fresh();
    RegisterModel<1> ag{0x6B};
    ag.set(0x0F, {0x68});
    ag.set(0x15, {0x00, 0x01, 0x03});   // temperature raw 256, STATUS_REG XLDA | GDA
    ag.set(0x18, {0x21, 0x03, 0x00, 0x00, 0x00, 0x00});   // gyro x = 801
    ag.set(0x28, {0x00, 0x00, 0x00, 0x00, 0x00, 0x40});   // accel z = 16384
    ag.readOnly      = {0x0F, 0x15, 0x16, 0x17, 0x18, 0x19, 0x28, 0x2D};
    FakeBus::respond = std::ref(ag);
    Dev<Chips::Lsm9ds1Ag<>> l9{};
    check(runUntil(l9, [&] { return l9.valid(); }, 500ms), "first sample");
    check(l9.identified(), "WHO_AM_I 0x68");
    check(hasWrite({0x10, 0x60}) && hasWrite({0x20, 0x60}), "119 Hz on both, default ranges");
    check(writes().size() >= 3 && writes()[0] == std::vector<std::uint8_t>{0x22, 0x05}
            && writes()[1] == std::vector<std::uint8_t>{0x22, 0x84}
            && writes()[2] == std::vector<std::uint8_t>{0x22, 0x44},
          "SW_RESET, BOOT, then BDU with IF_ADD_INC, before the configuration");
    // FS_G 10 does not exist: written as 11, and scaled as 2000 dps
    static_assert([] {
        std::array<std::byte, 1> b{};
        using G = Chips::Lsm9ds1Ag<>::GyroConfig;
        static_cast<void>(G::encode(0x70, b));
        Chips::Lsm9ds1Ag<>::State st{};
        G::applied(0x70, st);
        return b[0] == std::byte{0x78} && st.gyroFs == 3;
    }());
    checkEq(l9.latest().accel[2], 999424, "the accelerometer's step lands at offset 9");
    checkEq(l9.latest().gyro[0], 7008, "the gyroscope's step lands at offset 3");
    // OUT_TEMP 256 and XLDA | GDA, gyro x 801, accel z 16384
    static_assert([] {
        auto const f   = frame(0x00,
                               0x01,
                               0x03,
                               0x21,
                               0x03,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x00,
                               0x40);
        auto const got = Chips::Lsm9ds1Ag<>::Motion::decode(Bytes{f}, Chips::Lsm9ds1Ag<>::State{});
        return isOk(got) && equal(got.value.accel[2], 999424) && equal(got.value.gyro[0], 7008)
            && equal(got.value.temperature, 41000);
    }());

    fresh();
    RegisterModel<1> mg{0x1E};
    mg.set(0x0F, {0x3D});
    mg.set(0x27, {0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00});   // STATUS_REG_M ZYXDA, then x = 4096
    mg.readOnly = {0x0F, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
    FakeBus::respond
      = [&](std::uint8_t a, std::span<std::byte const> sent, std::span<std::byte> recv) {
            if(a == 0x1E && !sent.empty() && !recv.empty()) {
                // the magnetometer takes auto-increment as the sub-address MSB
                std::array<std::byte, 1> r{
                  static_cast<std::byte>(static_cast<std::uint8_t>(sent[0]) & 0x7FU)};
                return mg(a, r, recv);
            }
            return mg(a, sent, recv);
        };
    Dev<Chips::Lsm9ds1Mag<>> lm{};
    check(runUntil(lm, [&] { return lm.valid(); }, 500ms), "magnetometer sample");
    check(lm.identified(), "WHO_AM_I_M 0x3D");
    check(hasWrite({0x22, 0x00}), "continuous conversion mode, out of power-down");
    check(hasWrite({0x24, 0x40}), "BDU on the magnetometer half too");
    checkEq(lm.latest().x, 59801, "the model's frame reached the decode");
    // ZYXDA, x 4096 counts at 146 ugauss/LSB (1/6842 gauss): 59801.6 nT
    static_assert([] {
        auto const f   = frame(0x08, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00);
        auto const got = Chips::Lsm9ds1Mag<>::Field::decode(Bytes{f}, Chips::Lsm9ds1Mag<>::State{});
        return isOk(got) && equal(got.value.x, 59801) && equal(got.value.y, 0);
    }());
    if(failures != 0) { dump(); }
}

void bma456() {
    testCase("BMA456");
    fresh();
    RegisterModel<1> bm{0x18};
    bm.set(0x00, {0x16});                                 // CHIP_ID
    bm.set(0x12, {0x00, 0x10, 0x00, 0xF0, 0x00, 0x40});   // x=4096, y=-4096, z=16384
    bm.set(0x22, {0x07});                                 // 23 + 7 degC
    bm.readOnly      = {0x00, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x22};
    FakeBus::respond = std::ref(bm);
    Dev<Chips::Bma456<>> bb{};
    check(runUntil(bb, [&] { return bb.valid(); }, 1s), "first sample");
    check(bb.identified(), "CHIP_ID 0x16");
    check(writes(0).size() >= 7, "the seven bring-up writes");
    check(!writes(0).empty() && writes(0)[0] == std::vector<std::uint8_t>{0x7E, 0xB6},
          "a softreset before INIT_CTRL, so a second bring-up does not load it twice");
    check(hasWrite({0x7C, 0x00}) && hasWrite({0x59, 0x00}) && hasWrite({0x59, 0x01})
            && hasWrite({0x40, 0x87}) && hasWrite({0x41, 0x00}) && hasWrite({0x7D, 0x04}),
          "power save off, the config load, then ACC_CONF, ACC_RANGE and PWR_CTRL");
    checkEq(bb.latest().x, 250000, "the model's frame reached the decode");
    checkEq(bb.latest().temperature, 30, "the temperature's step lands at offset 6");
    // x 4096, y -4096, z 16384 little endian at 16384 LSB/g, then temperature 0x07 (23 + 7)
    static_assert([] {
        auto const f   = frame(0x00, 0x10, 0x00, 0xF0, 0x00, 0x40, 0x07);
        auto const got = Chips::Bma456<>::Motion::decode(Bytes{f}, Chips::Bma456<>::State{});
        return isOk(got) && equal(got.value.x, 250000) && equal(got.value.y, -250000)
            && equal(got.value.z, 1000000) && equal(got.value.temperature, 30)
            && got.value.temperatureValid;
    }());
    // temperature 0x80, the part's "no temperature yet": the acceleration still counts
    static_assert([] {
        auto const f   = frame(0x00, 0x10, 0x00, 0xF0, 0x00, 0x40, 0x80);
        auto const got = Chips::Bma456<>::Motion::decode(Bytes{f}, Chips::Bma456<>::State{});
        return isOk(got) && equal(got.value.x, 250000) && !got.value.temperatureValid;
    }());
    checkEq(Chips::Bma456<Chips::Bma456Detail::Range::g16>::CountsPerG,
            std::uint16_t{2048},
            "2048 LSB/g at +-16 g");
    if(failures != 0) { dump(); }
}

void motion() {
    testCase("LIS3DH");
    fresh();
    RegisterModel<1> m{0x18};
    m.set(0x0F, {0x33});
    m.set(0xA8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x40});   // z = 0x4000 -> 1024 mg
    FakeBus::respond = std::ref(m);
    Dev<Chips::Lis3dh> a{};
    check(runUntil(a, [&] { return a.samples() == 1; }, 500ms), "first sample");
    check(a.identified(), "WHO_AM_I 0x33");
    check(m.word(0x20) == 0x57 && m.word(0x23) == 0x88, "CTRL_REG1, CTRL_REG4");
    checkEq(a.latest().z, 1024000, "the model's frame reached the decode");
    // z 0x4000 left-aligned: 1024 counts at 1 mg
    static_assert([] {
        auto const f   = frame(0x00, 0x00, 0x00, 0x00, 0x00, 0x40);
        auto const got = Chips::Lis3dh::Motion::decode(Bytes{f}, Chips::Lis3dh::State{});
        return equal(got.x, 0) && equal(got.y, 0) && equal(got.z, 1024000);
    }());

    testCase("LSM6DS3");
    fresh();
    RegisterModel<1> g{0x6A};
    g.set(0x0F, {0x69});
    g.set(0x20,
          {0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x40});
    FakeBus::respond = std::ref(g);
    Dev<Chips::Lsm6ds3> l{};
    check(runUntil(l, [&] { return l.samples() == 1; }, 500ms), "first sample");
    check(l.identified() && g.word(0x10) == 0x40 && g.word(0x11) == 0x40,
          "WHO_AM_I, CTRL1_XL, CTRL2_G");
    checkEq(l.latest().accel[2], 999973, "the model's frame reached the decode");
    // temperature 0 (25 degC), gyro x 16 counts at 8.75 mdps, accel z 16393 counts at 61 ug
    static_assert([] {
        auto const            f = frame(0x00,
                                        0x00,
                                        0x10,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x09,
                                        0x40);
        Chips::Lsm6ds3::State st{};
        st.deviceId    = 0x69;
        auto const got = Chips::Lsm6ds3::Motion::decode(Bytes{f}, st);
        return equal(got.temperature, 2500) && equal(got.gyro[0], 140)
            && equal(got.accel[2], 999973);
    }());

    testCase("LSM6DS3 at +-16 g and 2000 dps");
    fresh();
    RegisterModel<1> g16{0x6A};
    g16.set(0x0F, {0x69});
    g16.set(0x20,
            {0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x40});
    FakeBus::respond = std::ref(g16);
    using WideLsm    = Chips::Lsm6ds3x<Chips::Lsm6ds3Detail::AccelRange::g16,
                                       Chips::Lsm6ds3Detail::GyroRange::dps2000>;
    Dev<WideLsm> lw{};
    check(runUntil(lw, [&] { return lw.samples() == 1; }, 500ms), "first sample");
    check(g16.word(0x10) == 0x44 && g16.word(0x11) == 0x4C, "CTRL1_XL FS_XL 01, CTRL2_G FS_G 11");
    checkEq(lw.latest().accel[2], 7999784, "16393 counts at 488 ug: the State the bring-up set");
    // the same frame at FS_XL 01 (+-16 g) and FS_G 11 (2000 dps)
    static_assert([] {
        auto const            f = frame(0x00,
                                        0x00,
                                        0x10,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x09,
                                        0x40);
        Chips::Lsm6ds3::State st{};
        st.deviceId    = 0x69;
        st.accelFs     = 1;
        st.gyroFs      = 3;
        auto const got = Chips::Lsm6ds3::Motion::decode(Bytes{f}, st);
        return equal(got.accel[2], 7999784) && equal(got.gyro[0], 1120);
    }());

    testCase("ADXL345");
    fresh();
    RegisterModel<1> x{0x53};
    x.set(0x00, {0xE5});
    x.set(0x30, {0x80});   // INT_SOURCE: DATA_READY
    x.set(0x32, {0x00, 0x00, 0x00, 0x00, 0x00, 0x01});
    FakeBus::respond = std::ref(x);
    Dev<Chips::Adxl345> d{};
    check(runUntil(d, [&] { return d.samples() == 1; }, 500ms), "first sample");
    check(d.identified() && x.word(0x31) == 0x08 && x.word(0x2C) == 0x0A && x.word(0x2D) == 0x08,
          "DEVID, format, rate, measure");
    checkEq(d.latest().z, 1000000, "the model's frame reached the decode");
    // DATA_READY, DATA_FORMAT, then z 256 counts: 1 g
    static_assert([] {
        auto const f   = frame(0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01);
        auto const got = Chips::Adxl345::Motion::decode(Bytes{f}, {});
        return isOk(got) && equal(got.value.x, 0) && equal(got.value.z, 1000000);
    }());

    testCase("HMC5883L");
    fresh();
    RegisterModel<1> h{0x1E};
    h.set(0x0A, {'H', '4', '3'});
    h.set(0x03, {0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x01});   // X, Z, Y, then STATUS RDY
    FakeBus::respond = std::ref(h);
    Dev<Chips::Hmc5883l> c{};
    check(runUntil(c, [&] { return c.samples() == 1; }, 500ms), "first sample");
    check(c.identified() && h.word(0x00) == 0x70 && h.word(0x01) == 0x20 && h.word(0x02) == 0x00,
          "H43, CRA, CRB, mode");
    checkEq(c.latest().x, 23486, "the model's frame reached the decode");
    // X 256, Z 512, Y 768 big-endian at 1090 counts per gauss, then STATUS RDY
    static_assert([] {
        auto const f   = frame(0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x01);
        auto const got = Chips::Hmc5883l::Field::decode(Bytes{f}, {});
        return isOk(got) && equal(got.value.x, 23486) && equal(got.value.z, 46972)
            && equal(got.value.y, 70458);
    }());

    testCase("AS5600");
    fresh();
    RegisterModel<1> e{0x36};
    e.set(0x0B, {0x20, 0x08, 0x00, 0x05, 0x37});
    e.set(0x1A, {0x80, 0x0A, 0xBC});
    FakeBus::respond = std::ref(e);
    Dev<Chips::As5600> r{};
    check(runUntil(
            r,
            [&] {
                return r.samples<Chips::As5600::Angle>() == 1
                    && r.samples<Chips::As5600::Gain>() == 1;
            },
            500ms),
          "both groups");
    checkEq(r.latest<Chips::As5600::Angle>().raw, 2048U, "the model's frames reached the decode");
    checkEq(r.latest<Chips::As5600::Angle>().scaledAngle, 1335U, "ANGLE's step lands at offset 3");
    checkEq(r.latest<Chips::As5600::Gain>().magnitude,
            0xABCU,
            "MAGNITUDE's step lands at offset 1");
    // STATUS MD, RAW ANGLE 2048 (180 degrees), ANGLE 1335
    static_assert([] {
        auto const f   = frame(0x20, 0x08, 0x00, 0x05, 0x37);
        auto const got = Chips::As5600::Angle::decode(Bytes{f});
        return isOk(got) && got.value.magnet && !got.value.tooWeak && !got.value.tooStrong
            && equal(got.value.raw, 2048) && equal(got.value.scaledAngle, 1335)
            && equal(got.value.angle, 18000);
    }());
    // AGC 0x80, MAGNITUDE 0xABC
    static_assert([] {
        auto const f   = frame(0x80, 0x0A, 0xBC);
        auto const got = Chips::As5600::Gain::decode(Bytes{f});
        return isOk(got) && equal(got.value.agc, 0x80) && equal(got.value.magnitude, 0xABC);
    }());
    if(failures != 0) { dump(); }
}

// -- accelerometer, pressure, display and GPS descriptions -------------------------------

void mma8451() {
    testCase("MMA8451");
    fresh();
    RegisterModel<1> m{0x1D};
    m.set(0x0D, {0x1A});
    // +1 g on Z: 4096 counts, left-justified 14 bit = 0x4000; X = -0.5 g (-2048 -> 0xE000)
    m.set(0x00, {0x08, 0xE0, 0x00, 0x00, 0x00, 0x40, 0x00});   // STATUS ZYXDR, then the data
    m.readOnly       = {0x0D, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    FakeBus::respond = std::ref(m);

    Dev<Chips::Mma8451> d{};
    check(runUntil(d, [&] { return d.answering(); }, 500ms), "bring-up completes");
    check(d.identified() && d.state().deviceId == 0x1A, "WHO_AM_I 0x1A");
    check(hasWrite({0x2B, 0x40}) && hasWrite({0x0E, 0x00}) && hasWrite({0x2A, 0x19}),
          "reset, 2 g range, 100 Hz active");
    check(runUntil(d, [&] { return d.samples() == 1; }, 500ms), "first sample");
    checkEq(d.latest().raw[2], 4096, "the model's frame reached the decode");
    // ZYXDR, X 0xE000 (-2048 counts, -0.5 g), Z 0x4000 (4096 counts, 1 g), left-justified 14 bit
    static_assert([] {
        auto const f   = frame(0x08, 0xE0, 0x00, 0x00, 0x00, 0x40, 0x00);
        auto const got = Chips::Mma8451::Motion::decode(Bytes{f}, {});
        return isOk(got) && equal(got.value.raw[2], 4096) && equal(got.value.raw[0], -2048)
            && equal(got.value.z, 1000000) && equal(got.value.x, -500000) && equal(got.value.y, 0);
    }());
    if(failures != 0) { dump(); }
}

// -- TMAG5273 ------------------------------------------------------------------------------------

using Tmag = Chips::Tmag5273<>;

static_assert(Kvasir::Samples::Magnetometer<Tmag::Field::Sample>
              && Kvasir::Samples::Thermometer<Tmag::Field::Sample>);

/// 0x10..0x1B: T 18088 (35 degC), X 16384, Y -8192, Z 256, CONV_STATUS, ANGLE, MAGNITUDE 64.
constexpr std::array<std::byte,
                     12>
tmagFrame(std::uint8_t status,
          std::uint8_t angleMsb = 0x16,
          std::uint8_t angleLsb = 0x28) {
    return frame(0x46, 0xA8, 0x40, 0x00, 0xE0, 0x00, 0x01, 0x00, status, angleMsb, angleLsb, 0x40);
}

constexpr Tmag::State tmagState(std::uint8_t version) {
    Tmag::State st{};
    st.version = version;
    return st;
}

// +-40 mT: B = code x 40 mT / 32768; T = 25 + (code - 17508) / 58; 354.5 degrees; 64 x 40 mT / 128
static_assert([] {
    auto const  got = Tmag::Field::decode(Bytes{tmagFrame(0x01)}, tmagState(1));
    auto const& s   = got.value;
    return isOk(got) && equal(s.x, 20'000'000) && equal(s.y, -10'000'000) && equal(s.z, 312'500)
        && equal(s.temperature, 35'000) && equal(s.angle, 35'450) && s.angleCode == 5672
        && equal(s.magnitude, 20'000'000) && !s.diagnostic && !s.powerOnReset;
}());

// the +-133 / +-266 mT part with X_Y_RANGE high: 16384 x 266 mT / 32768, Z at 133 mT
static_assert([] {
    auto st        = tmagState(2);
    st.ranges.xy   = Tmag::RangeSetting::high;
    auto const got = Tmag::Field::decode(Bytes{tmagFrame(0x01)}, st);
    return isOk(got) && equal(got.value.x, 133'000'000) && equal(got.value.y, -66'500'000)
        && equal(got.value.z, 1'039'062) && equal(got.value.magnitude, 133'000'000);
}());

// 17.25 degrees (the data sheet's second example); -40 degC; a channel not enabled reads 0;
// DIAG_STATUS and POR are reported
static_assert([] {
    auto st        = tmagState(1);
    st.channels    = Tmag::ChannelSet::xy;
    auto f         = tmagFrame(0x13, 0x01, 0x14);
    f[0]           = std::byte{0x35};   // 13738 = 17508 - 65 x 58
    f[1]           = std::byte{0xAA};
    auto const got = Tmag::Field::decode(Bytes{f}, st);
    return isOk(got) && equal(got.value.angle, 1725) && equal(got.value.temperature, -40'000)
        && equal(got.value.z, 0) && equal(got.value.x, 20'000'000) && got.value.diagnostic
        && got.value.powerOnReset;
}());

// RESULT_STATUS clear: nothing new
static_assert(isUnchanged(Tmag::Field::decode(Bytes{tmagFrame(0x10)},
                                              tmagState(1))));

// the period follows 25 us x (averages x (channels + T) + 1), rounded up to milliseconds
static_assert(Tmag::ReadPeriod == 4ms
              && Chips::Tmag5273Detail::conversionTime(Tmag::AveragingCount::x32,
                                                       Tmag::ChannelSet::x)
                   == 1625us
              && Chips::Tmag5273Detail::readPeriod(Tmag::AveragingCount::x1,
                                                   Tmag::ChannelSet::xyz)
                   == 1ms
              && Chips::Tmag5273Detail::readPeriod(Tmag::AveragingCount::x16,
                                                   Tmag::ChannelSet::xyz)
                   == 2ms
              && Chips::Tmag5273Detail::conversionTime(Tmag::AveragingCount::x1,
                                                       Tmag::ChannelSet::x)
                   == 75us);

RegisterModel<1> tmagModel(std::uint8_t address,
                           std::uint8_t deviceId,
                           std::uint8_t mfrLsb) {
    RegisterModel<1> m{address};
    m.set(0x0D, {deviceId, mfrLsb, 0x54});
    m.set(0x10, {0x46, 0xA8, 0x40, 0x00, 0xE0, 0x00, 0x01, 0x00, 0x01, 0x16, 0x28, 0x40});
    for(std::uint32_t r = 0x0D; r <= 0x1B; ++r) { m.readOnly.push_back(r); }
    return m;
}

void tmag5273() {
    testCase("TMAG5273");
    fresh();
    auto m           = tmagModel(0x22, 0x01, 0x49);
    FakeBus::respond = std::ref(m);
    Dev<Tmag> d{};
    check(runUntil(d, [&] { return d.valid(); }, 200ms), "first sample");
    check(d.identified(), "MANUFACTURER_ID 0x5449");
    checkEq(d.state().manufacturer, 0x5449U, "manufacturer");
    checkEq(d.state().version, 1U, "VER 1: the +-40 / +-80 mT part");
    {
        auto const w = writes();
        check(w.size() >= 5
                && std::vector<std::vector<std::uint8_t>>(w.begin(), w.begin() + 5)
                     == std::vector<std::vector<std::uint8_t>>{{0x00, 0x14}, {0x02, 0x70, 0x04}, {0x07, 0x01}, {0x18, 0x10}, {0x01, 0x02}},
              "32x averaging, XYZ with the XY angle at low range, T on, POR cleared, continuous last");
    }
    checkEq(d.latest().x, 20'000'000, "the model's frame reached the decode");
    checkEq(d.latest().temperature, 35'000, "35 degC");
    check(d.period<Tmag::Field>() == 4ms, "32x over X, Y, Z and T is 3.2 ms: a 4 ms period");

    d.set<Tmag::Averaging>(Tmag::AveragingCount::x1);
    check(runUntil(d, [&] { return d.writes<Tmag::Averaging>() == 1; }, 200ms), "CONV_AVG written");
    checkEq(m.word(0x00), 0x00U, "1x");
    check(d.period<Tmag::Field>() == 1ms, "and the period follows it");

    d.set<Tmag::Ranges>({.xy    = Tmag::RangeSetting::high,
                         .z     = Tmag::RangeSetting::low,
                         .angle = Tmag::AngleChannels::xy});
    check(runUntil(
            d,
            [&] { return d.writes<Tmag::Ranges>() == 1; },
            200ms),
          "SENSOR_CONFIG_2 written");
    checkEq(m.word(0x03), 0x06U, "ANGLE_EN XY, X_Y_RANGE high");
    auto const seq = d.seq();
    check(runUntil(d, [&] { return d.seq() > seq + 1; }, 200ms), "sampled after it");
    checkEq(d.latest().x, 40'000'000, "16384 counts at +-80 mT: decode follows the written range");
    checkEq(d.latest().z, 312'500, "Z stays at +-40 mT");

    m.set(0x18, {0x00});   // RESULT_STATUS clear
    auto const before = d.seq();
    auto const idle   = d.unchanged<Tmag::Field>();
    runFor(d, 50ms);
    check(d.unchanged<Tmag::Field>() > idle && d.seq() == before,
          "no conversion complete: unchanged");

    testCase("TMAG5273: identification");
    fresh();
    auto wrong       = tmagModel(0x22, 0x01, 0x48);
    FakeBus::respond = std::ref(wrong);
    Dev<Tmag> r{};
    runFor(r, 50ms);
    check(!r.identified() && r.unidentified() >= 1, "manufacturer 0x5448 refused");

    fresh();
    auto reserved    = tmagModel(0x22, 0x03, 0x49);
    FakeBus::respond = std::ref(reserved);
    Dev<Tmag> v{};
    runFor(v, 50ms);
    check(!v.identified(), "a reserved VER refused");

    fresh();
    auto a           = tmagModel(0x35, 0x02, 0x49);
    FakeBus::respond = std::ref(a);
    Dev<Tmag, At<0x35>> va{};
    check(runUntil(va, [&] { return va.valid(); }, 200ms), "the A version at 0x35");
    checkEq(va.state().version, 2U, "VER 2");
    checkEq(va.latest().x, 66'500'000, "16384 counts at +-133 mT");
    if(failures != 0) { dump(); }
}

}   // namespace

int main() {
    tmag5273();
    mpu6050();
    lsm303agr();
    iis2dulpx();
    lsm9ds1();
    bma456();
    motion();
    mma8451();
    return finish();
}
