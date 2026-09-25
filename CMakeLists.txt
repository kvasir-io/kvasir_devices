cmake_minimum_required(VERSION 3.28)

if(TARGET kvasir_devices)
    return()
endif()

# Small device drivers on the Kvasir SDK, header-only: sensors, converters, memories, displays and touch controllers on
# I2C, SPI and 1-Wire. Link-layer drivers (W5500, CYW43, LAN8650) are not part of this library.
add_library(kvasir_devices INTERFACE)
target_include_directories(kvasir_devices INTERFACE src)
# The headers use C++26 (deducing this, concepts, designated initialisers). Declared here so a consumer that forgets to
# set the standard gets it rather than a wall of parse errors.
target_compile_features(kvasir_devices INTERFACE cxx_std_26)
add_library(kvasir::devices ALIAS kvasir_devices)

# The SDK's headers (RateLimiter, Atomic, Io, ...) and, on a target, uc_log: kvasir::sdk. A firmware has it from
# Kvasir_SDK/cmake/util.cmake; otherwise (host tests, a firmware that fetches this before the SDK is configured) it
# comes from KVASIR_ROOT: variable, environment, a FetchContent'd Kvasir_SDK, else the sibling checkout.
if(NOT TARGET kvasir::sdk)
    if(KVASIR_ROOT)
        set(_kvasir_root "${KVASIR_ROOT}")
    elseif(DEFINED ENV{KVASIR_ROOT})
        set(_kvasir_root "$ENV{KVASIR_ROOT}")
    elseif(kvasir_sdk_SOURCE_DIR)
        set(_kvasir_root "${kvasir_sdk_SOURCE_DIR}")
    else()
        get_filename_component(_kvasir_root "${CMAKE_CURRENT_LIST_DIR}/../Kvasir_SDK" ABSOLUTE)
    endif()
    if(NOT EXISTS "${_kvasir_root}/cmake/kvasir_sdk_targets.cmake")
        message(
            FATAL_ERROR "kvasir_devices needs the Kvasir SDK: '${_kvasir_root}' has no cmake/kvasir_sdk_targets.cmake")
    endif()
    include(${_kvasir_root}/cmake/kvasir_sdk_targets.cmake)
endif()
target_link_libraries(kvasir_devices INTERFACE kvasir::sdk)

# ---- physical quantities ------------------------------------------------------------------------------------------
#
# mp-units (https://github.com/mpusz/mp-units), fetched and pinned at v2.5.0. A chip description says what it measured
# and at what scale -- quantity<si::milli<si::degree_Celsius>, int32_t> rather than a bare int named temperatureMilli --
# so the scale is in the type, one conversion serves every chip, and generic code over "a temperature sensor" is
# possible. The representation stays integral: sizeof is the int it holds and no soft-float reaches a Cortex-M0+ target.
#
# Durations stay std::chrono - the log backend serialises those natively, unit and all. Frequencies are quantities too
# (Units::Hertz), including as non-type template arguments of a chip description.
#
# It is declared here rather than in each firmware so the dependency propagates with the headers that need it.
#
# Freestanding, because the target has no operating system: MP_UNITS_API_FREESTANDING drops everything that needs the
# hosted library, and a freestanding build requires the contract checks to be off. The headers are attached as SYSTEM
# includes below, so -Weverything -pedantic-errors applies to our code and not to the library.
include(FetchContent)
set(MP_UNITS_API_FREESTANDING
    ON
    CACHE BOOL "" FORCE)
set(MP_UNITS_API_CONTRACTS
    "NONE"
    CACHE STRING "" FORCE)
set(MP_UNITS_API_NATURAL_UNITS
    OFF
    CACHE BOOL "" FORCE)
set(MP_UNITS_BUILD_CXX_MODULES
    OFF
    CACHE BOOL "" FORCE)
set(MP_UNITS_BUILD_INSTALL
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
    mp-units
    GIT_REPOSITORY https://github.com/mpusz/mp-units.git
    GIT_TAG v2.5.0
    GIT_SHALLOW ON
    SOURCE_SUBDIR src
    PATCH_COMMAND git apply --reverse --check ${CMAKE_CURRENT_LIST_DIR}/cmake/mp-units.patch || git apply
                  ${CMAKE_CURRENT_LIST_DIR}/cmake/mp-units.patch
    UPDATE_DISCONNECTED ON)
FetchContent_MakeAvailable(mp-units)

# mp-units is not written against -Weverything and its warnings are not ours to fix.
foreach(_t mp-units-core mp-units-systems)
    if(TARGET ${_t})
        get_target_property(_dirs ${_t} INTERFACE_INCLUDE_DIRECTORIES)
        if(_dirs)
            target_include_directories(${_t} SYSTEM INTERFACE ${_dirs})
        endif()
    endif()
endforeach()

target_link_libraries(kvasir_devices INTERFACE mp-units::mp-units)
