# The display drivers (src/kvasir/Devices/Display/) against mock buses.
#
# cmake -S test/display -B build/display && cmake --build build/display --parallel && ctest --test-dir build/display
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_display_test LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/KvasirDevicesTest.cmake)

foreach(test panel_test ssd1680_test spi_dcs_bus_test spi_stream_test)
    kvasir_devices_test(display_${test} SOURCE ${test}.cpp)
endforeach()
