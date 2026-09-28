# Every host test suite in one build tree. Each is also a project of its own (cmake -S test/spi -B build/spi); what they
# share is cmake/KvasirDevicesTest.cmake.
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_test LANGUAGES CXX)

enable_testing()

add_subdirectory(display)
add_subdirectory(headers)
add_subdirectory(i2c)
add_subdirectory(spi)
add_subdirectory(usb)
