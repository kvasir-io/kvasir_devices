# The USB device (src/kvasir/Devices/USB) against a fake controller with the host on its other side, on the host. The
# SDK is used header-only (RateLimiter, Atomic/Queue, its FakeClock, string_constant); KVASIR_ROOT says where it is and
# defaults to the sibling checkout.
#
# cmake -S test/usb -B build/usb && cmake --build build/usb --parallel && ctest --test-dir build/usb
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_usb_test LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/KvasirDevicesTest.cmake)

# One executable per topic, each run against both kinds of controller (FakeUsbBackend.hpp).
foreach(test control_test cdc_test bulk_test hid_test fuzz_test)
    # string_constant: the descriptors' strings are lifted into types with it; on a target it arrives with uc_log.
    # -Wno-missing-field-initializers: gcc counts the descriptors' base class (which a designated initialiser cannot
    # name, and which has defaults) as a member left out.
    kvasir_devices_test(
        usb_${test}
        SOURCE
        ${test}.cpp
        INCLUDES
        ${KVASIR_ROOT}/uc_log/remote_fmt/string_constant/src
        GCC_OPTIONS
        -Wno-missing-field-initializers)
endforeach()
