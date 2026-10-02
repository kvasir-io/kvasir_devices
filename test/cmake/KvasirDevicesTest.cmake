# What every host test suite shares: language settings, the SDK (header-only), the library, the warning and sanitizer
# set, and kvasir_devices_test(). Included once per suite, deliberately without an include guard: its variables must be
# set in every suite's directory scope.

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

get_filename_component(KVASIR_DEVICES_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

# The SDK: KVASIR_ROOT (variable or environment), else the sibling checkout.
if(NOT KVASIR_ROOT)
    if(DEFINED ENV{KVASIR_ROOT})
        set(KVASIR_ROOT $ENV{KVASIR_ROOT})
    else()
        get_filename_component(KVASIR_ROOT "${KVASIR_DEVICES_ROOT}/../Kvasir_SDK" ABSOLUTE)
    endif()
endif()
if(NOT EXISTS "${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake")
    message(FATAL_ERROR "KVASIR_ROOT does not point at a Kvasir SDK checkout: ${KVASIR_ROOT}")
endif()
# kvasir::sdk and kvasir::test_support (<support/FakeClock.hpp>).
include(${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake)

enable_testing()

# The library's own CMakeLists, for the same mp-units wiring a firmware gets. A no-op once another suite added it.
add_subdirectory(${KVASIR_DEVICES_ROOT} kvasir_devices_build)

# The warning set the target build enforces (Kvasir_SDK/cmake).
function(kvasir_devices_warnings target)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "GCC_OPTIONS")
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(
            ${target}
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
                    # clang 23: only proposals for [[clang::lifetimebound]] marks, off in the firmware build too
                    # (Kvasir_SDK/cmake/arm_clang.cmake); the dangling-reference checks stay on.
                    -Wno-lifetime-safety-intra-tu-suggestions
                    -Wno-lifetime-safety-intra-tu-constructor-suggestions
                    -fsanitize=address,undefined
                    -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -pedantic -Wconversion -Wsign-conversion
                                                 ${arg_GCC_OPTIONS})
    endif()
endfunction()

# kvasir_devices_test(<name> [SOURCE <file>] [INCLUDES <dir>...] [GCC_OPTIONS <option>...])
#
# One ctest executable from <name>.cpp (or SOURCE) in the calling suite's directory.
function(kvasir_devices_test name)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "SOURCE" "INCLUDES;GCC_OPTIONS")
    if(NOT arg_SOURCE)
        set(arg_SOURCE ${name}.cpp)
    endif()
    add_executable(${name} ${arg_SOURCE})
    target_link_libraries(${name} PRIVATE kvasir::devices kvasir::test_support)
    target_include_directories(${name} PRIVATE ${KVASIR_DEVICES_ROOT}/test ${CMAKE_CURRENT_SOURCE_DIR} ${arg_INCLUDES})
    # remote_fmt: LogStubs.hpp checks every log line with it. SYSTEM: not held to this warning set.
    target_include_directories(${name} SYSTEM PRIVATE ${KVASIR_ROOT}/uc_log/remote_fmt/src
                                                      ${KVASIR_ROOT}/uc_log/remote_fmt/string_constant/src)
    kvasir_devices_warnings(${name} GCC_OPTIONS ${arg_GCC_OPTIONS})
    add_test(NAME ${name} COMMAND ${name})
    # Without halt_on_error a UBSan report is printed and the test still passes.
    set_tests_properties(${name} PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
endfunction()
