# The I2C device engine (src/kvasir/Devices/I2C) and every chip description against a fake bus, a fake clock and a
# device model, on the host. The SDK is used header-only (RateLimiter, its FakeClock); KVASIR_ROOT says where it is and
# defaults to the sibling checkout.
#
# cmake -S test/i2c -B build/i2c && cmake --build build/i2c --parallel && ctest --test-dir build/i2c
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_i2c_test LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/KvasirDevicesTest.cmake)

# One executable per topic: the engine on its own, the Bus, the switch gate, the touch controllers, the panels, then the
# descriptions by what they measure; a description whose model of the part is big enough to want a file of its own
# (VL53L1X, TLV493D) gets one, and the switch simulation and the BusScan are their own too. All built and run the same
# way. Named i2c_<topic> in ctest, as the other suites prefix theirs.
foreach(
    test
    engine_test
    bus_test
    power_rail_test
    bridge_test
    mux_gate_test
    touch_test
    panel_test
    thermal_test
    light_test
    motion_test
    power_io_test
    rtc_test
    adc_test
    capsense_test
    gnss_test
    vl53l1x_test
    tlv493d_test
    mux_test
    bus_scan_test
    segment_test
    identify_test
    oracle_test)
    kvasir_devices_test(i2c_${test} SOURCE ${test}.cpp)
endforeach()
