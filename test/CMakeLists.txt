# Every host test suite in one build tree, one project. What the suites share is cmake/KvasirDevicesTest.cmake, included
# once here; the suites are its subdirectories.
#
# cmake -S test -B build/clang -G Ninja && cmake --build build/clang && ctest --test-dir build/clang
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_test LANGUAGES CXX)

include(cmake/KvasirDevicesTest.cmake)

add_subdirectory(bus)
add_subdirectory(display)
add_subdirectory(gpio)
add_subdirectory(headers)
add_subdirectory(i2c)
add_subdirectory(input)
add_subdirectory(spi)
add_subdirectory(usb)
