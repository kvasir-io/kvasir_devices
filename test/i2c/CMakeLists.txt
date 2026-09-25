# The I2C device engine (src/kvasir/Devices/I2C) and every chip description against a fake bus, a fake clock and a
# device model, on the host. The SDK is used header-only (RateLimiter, its FakeClock); KVASIR_ROOT says where it is and
# defaults to the sibling checkout.
#
# cmake -S test/i2c -B build/i2c && cmake --build build/i2c --parallel && ctest --test-dir build/i2c
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_i2c_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(root ${CMAKE_CURRENT_SOURCE_DIR}/../..)

if(NOT KVASIR_ROOT)
    if(DEFINED ENV{KVASIR_ROOT})
        set(KVASIR_ROOT $ENV{KVASIR_ROOT})
    else()
        get_filename_component(KVASIR_ROOT "${root}/../Kvasir_SDK" ABSOLUTE)
    endif()
endif()
if(NOT EXISTS "${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake")
    message(FATAL_ERROR "KVASIR_ROOT does not point at a Kvasir SDK checkout: ${KVASIR_ROOT}")
endif()
# kvasir::sdk (the headers) and kvasir::test_support (<support/FakeClock.hpp>); the library's LogStubs are in
# test/support.
include(${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake)

enable_testing()

# The library's own CMakeLists, so the host test gets the same mp-units wiring a firmware does (fetched, freestanding,
# attached as SYSTEM includes) from one definition rather than a copy.
add_subdirectory(${root} kvasir_devices_build)

# One executable per topic: the engine on its own, the Bus, the switch gate, the touch controllers, the panels, then the
# descriptions by what they measure; a description whose model of the part is big enough to want a file of its own
# (VL53L1X, TLV493D) gets one, and the switch simulation and the BusScan are their own too. All built and run the same
# way.
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
    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} PRIVATE kvasir::devices kvasir::test_support)
    target_include_directories(${test} PRIVATE ${root}/test ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME ${test} COMMAND ${test})

    # The same warning set the target build enforces (Kvasir_SDK/cmake), so a header that is clean here is clean there.
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(
            ${test}
            PRIVATE -Weverything
                    -Wno-switch-default
                    -Wno-c++98-compat
                    -Wno-c++98-compat-pedantic
                    -Wno-c++20-compat
                    -Wno-pre-c2x-compat
                    -pedantic-errors
                    -Wno-padded
                    -Wno-covered-switch-default
                    -Wno-switch-enum
                    -Wno-float-equal
                    -Wno-unknown-warning-option
                    -Wno-reserved-identifier
                    -Wno-zero-as-null-pointer-constant
                    -Wno-documentation-unknown-command
                    -Wno-documentation
                    -Wno-declaration-after-statement
                    -Wno-braced-scalar-init
                    -Wno-unsafe-buffer-usage
                    -Wno-nrvo
                    -Wno-exit-time-destructors
                    -Wno-global-constructors
                    -fsanitize=address,undefined
                    -fno-omit-frame-pointer)
        target_link_options(${test} PRIVATE -fsanitize=address,undefined)
    else()
        target_compile_options(${test} PRIVATE -Wall -Wextra -pedantic -Wconversion -Wsign-conversion)
    endif()
endforeach()
