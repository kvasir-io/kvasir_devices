# Every header of the library compiled alone in a translation unit of its own, with the tests' warnings: a header that
# leans on another one being included first, or that no test includes, fails here. No logging (Log.hpp's fallbacks).

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
# string_constant only (SYSTEM): the USB descriptors lift their strings into types with it; on a target it arrives with
# uc_log. Not remote_fmt or the log stubs.
target_link_libraries(kvasir_devices_headers PRIVATE kvasir::devices kvasir_devices_test_string_constant)
# -Wno-missing-field-initializers: as in the USB suite.
kvasir_devices_warnings(kvasir_devices_headers GCC_OPTIONS -Wno-missing-field-initializers)

# The kvasir::bus tier (CMakeLists.txt): the headers the chip drivers build on, each alone again, linked to kvasir::bus
# only - no mp-units include directory, so a bus header that reaches Quantities.hpp (mp-units) fails here. The list is
# the layer table's (Kvasir_SDK tools/layers.toml, the kvasir_devices rows in layer "bus", which layer_check reads), so
# the two cannot drift: a new bus header is one entry there. Quantities.hpp is in layer bus (the chips' SPI drivers
# include it for Units::Hertz) but needs mp-units, i.e. kvasir::devices: the one bus header not checked here.
file(STRINGS ${KVASIR_ROOT}/tools/layers.toml bus_rows REGEX "^\"src/kvasir/Devices/[^\"]*\" = \"bus\"")
set(bus_headers)
foreach(row IN LISTS bus_rows)
    string(REGEX MATCH "^\"src/([^\"]*)\"" path "${row}")
    set(path "${CMAKE_MATCH_1}")
    if(path MATCHES "^([^{]*)\\{([^}]*)\\}(.*)$")
        set(prefix "${CMAKE_MATCH_1}")
        set(suffix "${CMAKE_MATCH_3}")
        string(REPLACE "," ";" names "${CMAKE_MATCH_2}")
        foreach(name IN LISTS names)
            list(APPEND bus_headers "${prefix}${name}${suffix}")
        endforeach()
    else()
        list(APPEND bus_headers "${path}")
    endif()
endforeach()
list(REMOVE_ITEM bus_headers kvasir/Devices/Quantities.hpp)
if(NOT bus_headers)
    message(FATAL_ERROR "no kvasir_devices row in layer \"bus\" found in ${KVASIR_ROOT}/tools/layers.toml")
endif()
foreach(header IN LISTS bus_headers)
    if(NOT EXISTS ${KVASIR_DEVICES_ROOT}/src/${header})
        message(FATAL_ERROR "${KVASIR_ROOT}/tools/layers.toml puts ${header} in layer bus; there is no such header")
    endif()
endforeach()
set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS ${KVASIR_ROOT}/tools/layers.toml)
set(bus_sources)
foreach(header ${bus_headers})
    string(MAKE_C_IDENTIFIER ${header} name)
    set(source ${CMAKE_CURRENT_BINARY_DIR}/bus/${name}.cpp)
    file(CONFIGURE OUTPUT ${source} CONTENT "#include <${header}>\n")
    list(APPEND bus_sources ${source})
endforeach()
add_library(kvasir_bus_headers OBJECT ${bus_sources})
target_link_libraries(kvasir_bus_headers PRIVATE kvasir::bus kvasir_devices_test_string_constant)
kvasir_devices_warnings(kvasir_bus_headers GCC_OPTIONS -Wno-missing-field-initializers)
