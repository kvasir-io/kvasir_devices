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
    oracle_test
    clock_test)
    kvasir_devices_test(i2c_${test} SOURCE ${test}.cpp)
endforeach()

# What must not compile (clock_fail.cpp): each case is an object library outside `all`, which ctest builds and judges by
# the reason in the compiler's error, so a failure for any other reason (a typo, a missing include) does not pass.
foreach(case_reason IN ITEMS "SLOW_PART|i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus"
                             "ABOVE_CHIP|Config::BusClock is above the chip")
    string(REPLACE "|" ";" case_reason "${case_reason}")
    list(GET case_reason 0 case)
    list(GET case_reason 1 reason)
    string(TOLOWER "i2c_clock_fail_${case}" target)
    add_library(${target} OBJECT EXCLUDE_FROM_ALL clock_fail.cpp)
    target_compile_definitions(${target} PRIVATE CLOCK_FAIL_${case})
    target_link_libraries(${target} PRIVATE kvasir::devices kvasir::test_support)
    target_include_directories(${target} PRIVATE ${KVASIR_DEVICES_ROOT}/test ${CMAKE_CURRENT_SOURCE_DIR})
    target_include_directories(${target} SYSTEM PRIVATE ${KVASIR_ROOT}/uc_log/remote_fmt/src
                                                        ${KVASIR_ROOT}/uc_log/remote_fmt/string_constant/src)
    add_test(NAME ${target} COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${target})
    # Each case runs a build of this very tree: two at once (ctest -j) corrupt its .ninja_deps for good.
    set_tests_properties(${target} PROPERTIES PASS_REGULAR_EXPRESSION "${reason}" RESOURCE_LOCK build_tree)
endforeach()

# The same for the engine's features (feature_fail.cpp): a device that needs a feature its bus leaves out.
foreach(case_reason IN ITEMS "ENABLED|leaves the engine's switchable feature out")
    string(REPLACE "|" ";" case_reason "${case_reason}")
    list(GET case_reason 0 case)
    list(GET case_reason 1 reason)
    string(TOLOWER "i2c_feature_fail_${case}" target)
    add_library(${target} OBJECT EXCLUDE_FROM_ALL feature_fail.cpp)
    target_compile_definitions(${target} PRIVATE FEATURE_FAIL_${case})
    target_link_libraries(${target} PRIVATE kvasir::devices kvasir::test_support)
    target_include_directories(${target} PRIVATE ${KVASIR_DEVICES_ROOT}/test ${CMAKE_CURRENT_SOURCE_DIR})
    target_include_directories(${target} SYSTEM PRIVATE ${KVASIR_ROOT}/uc_log/remote_fmt/src
                                                        ${KVASIR_ROOT}/uc_log/remote_fmt/string_constant/src)
    add_test(NAME ${target} COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${target})
    # Builds this very tree, like the clock cases: one at a time.
    set_tests_properties(${target} PROPERTIES PASS_REGULAR_EXPRESSION "${reason}" RESOURCE_LOCK build_tree)
endforeach()
