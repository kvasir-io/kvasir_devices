# Every header of the library compiled alone in a translation unit of its own, with the tests' warnings: a header that
# leans on another one being included first, or that no test includes, fails here. No logging (Log.hpp's fallbacks).
#
# cmake -S test/headers -B build/headers && cmake --build build/headers --parallel
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_headers_test LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/KvasirDevicesTest.cmake)

file(
    GLOB_RECURSE headers
    RELATIVE ${KVASIR_DEVICES_ROOT}/src
    CONFIGURE_DEPENDS ${KVASIR_DEVICES_ROOT}/src/*.hpp)

set(sources)
foreach(header ${headers})
    string(MAKE_C_IDENTIFIER ${header} name)
    set(source ${CMAKE_CURRENT_BINARY_DIR}/alone/${name}.cpp)
    file(CONFIGURE OUTPUT ${source} CONTENT "#include <${header}>\n")
    list(APPEND sources ${source})
endforeach()

add_library(kvasir_devices_headers OBJECT ${sources})
target_link_libraries(kvasir_devices_headers PRIVATE kvasir::devices)
# string_constant: the USB descriptors lift their strings into types with it; on a target it arrives with uc_log.
target_include_directories(kvasir_devices_headers PRIVATE ${KVASIR_ROOT}/uc_log/remote_fmt/string_constant/src)
# -Wno-missing-field-initializers: as in the USB suite.
kvasir_devices_warnings(kvasir_devices_headers GCC_OPTIONS -Wno-missing-field-initializers)
