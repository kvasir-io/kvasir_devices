# Input devices without a bus: the quadrature decoder of the rotary encoders.
#
# cmake -S test/input -B build/input && cmake --build build/input --parallel && ctest --test-dir build/input
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_input_test LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/KvasirDevicesTest.cmake)

kvasir_devices_test(input_quadrature_test SOURCE quadrature_test.cpp)
