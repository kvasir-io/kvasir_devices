# The USB device (src/kvasir/Devices/USB) against a fake controller with the host on its other side, on the host. The
# SDK is used header-only (RateLimiter, Atomic/Queue, its FakeClock, string_constant); KVASIR_ROOT says where it is and
# defaults to the sibling checkout.
#
# cmake -S test/usb -B build/usb && cmake --build build/usb --parallel && ctest --test-dir build/usb
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_usb_test LANGUAGES CXX)

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

# The library's own CMakeLists (a no-op when test/CMakeLists.txt already added it for the I2C suite).
add_subdirectory(${root} kvasir_devices_build)

# One executable per topic, each run against both kinds of controller (FakeUsbBackend.hpp).
foreach(test control_test cdc_test bulk_test hid_test fuzz_test)
    add_executable(usb_${test} ${test}.cpp)
    target_link_libraries(usb_${test} PRIVATE kvasir::devices kvasir::test_support)
    # string_constant: the descriptors' strings are lifted into types with it; on a target it arrives with uc_log.
    target_include_directories(usb_${test} PRIVATE ${root}/test ${KVASIR_ROOT}/uc_log/remote_fmt/string_constant/src
                                                   ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME usb_${test} COMMAND usb_${test})

    # The same warning set the target build enforces (Kvasir_SDK/cmake), so a header that is clean here is clean there.
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(
            usb_${test}
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
        target_link_options(usb_${test} PRIVATE -fsanitize=address,undefined)
    else()
        # -Wno-missing-field-initializers: gcc counts the descriptors' base class (which a designated initialiser cannot
        # name, and which has defaults) as a member left out.
        target_compile_options(usb_${test} PRIVATE -Wall -Wextra -pedantic -Wconversion -Wsign-conversion
                                                   -Wno-missing-field-initializers)
    endif()
endforeach()
