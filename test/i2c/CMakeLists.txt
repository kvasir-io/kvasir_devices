# The I2C device engine (src/kvasir/Devices/I2C) and every chip description against a fake bus, a fake clock and a
# device model, on the host. The SDK is used header-only (RateLimiter, its FakeClock); cmake/KvasirDevicesTest.cmake
# finds or fetches it.

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
    enabled_test
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
    oracle_test
    clock_test
    line_recovery_test
    queue_core_test
    chip_headers_lean)
    kvasir_devices_test(i2c_${test} SOURCE ${test}.cpp)
endforeach()

# chips/All.hpp and chips/Every.hpp are kept by hand: they agree with the directory (catalogue_lists.cmake).
add_test(NAME i2c_catalogue_lists COMMAND ${CMAKE_COMMAND} -DCHIPS=${KVASIR_DEVICES_ROOT}/src/kvasir/Devices/I2C/chips
                                          -P ${CMAKE_CURRENT_SOURCE_DIR}/catalogue_lists.cmake)

# What must not compile (clock_fail.cpp): the per-device clock checks, judged by the reason in the compiler's error.
kvasir_devices_compile_fail(
    i2c_clock_fail
    SOURCE
    clock_fail.cpp
    DEFINE_PREFIX
    CLOCK_FAIL
    CASES
    "SLOW_PART|i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus"
    "ABOVE_CHIP|Config::busClock is above the chip"
    "BELOW_MIN|i2cDeviceClockBelowMinBaudRateOfTheBus")

# The same for the engine's features (feature_fail.cpp): a device that needs a feature its bus leaves out.
kvasir_devices_compile_fail(
    i2c_feature_fail
    SOURCE
    feature_fail.cpp
    DEFINE_PREFIX
    FEATURE_FAIL
    CASES
    "ENABLED|leaves the engine's switchable feature out")

# A queued bus's accessor of an optional part that is off (queue_core_test.cpp, its lean bus): no such member.
kvasir_devices_compile_fail(
    i2c_queue_gate
    SOURCE
    queue_core_test.cpp
    DEFINE_PREFIX
    I2C_QUEUE_GATE
    CASES
    "COUNTERS|Features\\.counters"
    "LATENCY|Features\\.latency"
    "SNAPSHOT|Features\\.timeoutSnapshot")

# One switch gate claimed twice (gate_fail.cpp): the same address on one channel, or twice in front of the switch.
kvasir_devices_compile_fail(
    i2c_gate_fail
    SOURCE
    gate_fail.cpp
    DEFINE_PREFIX
    GATE_FAIL
    CASES
    "SAME_CHANNEL|have the same address and are on the same gate"
    "FRONT_TWICE|have the same address and are on the same gate")
