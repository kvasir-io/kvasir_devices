# What every host test suite shares: language settings, the SDK (header-only), the library, the warning and sanitizer
# set, kvasir_devices_test() and kvasir_devices_compile_fail(). Included once, from test/CMakeLists.txt, before the
# suites' add_subdirectory(): the suites are directories of that one project, so they inherit KVASIR_DEVICES_ROOT, the
# SDK and the language settings from its scope, and the include guard keeps a second include from doing anything.
include_guard(GLOBAL)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# No C++ modules here: without this every TU gets a dependency scan step (.ddi), as the firmware toolchain turns off
# too.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

get_filename_component(KVASIR_DEVICES_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

# The SDK: KVASIR_ROOT (variable or environment), else the sibling checkout, else fetched (the SDK's
# cmake/kvasir_locate.cmake). The fetch takes the SDK without its submodules and then only uc_log -> remote_fmt ->
# string_constant, at the commits the SDK records (kvasir_locate SDK_SUBMODULES: the log stubs check every line with
# remote_fmt; the USB descriptors use string_constant). The SDK and this library change together: the fetch is always
# the SDK's master.
include(FetchContent)
if(NOT KVASIR_ROOT AND DEFINED ENV{KVASIR_ROOT})
    set(KVASIR_ROOT "$ENV{KVASIR_ROOT}")
endif()
if(NOT KVASIR_ROOT AND EXISTS "${KVASIR_DEVICES_ROOT}/../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${KVASIR_DEVICES_ROOT}/../Kvasir_SDK" ABSOLUTE)
elseif(NOT KVASIR_ROOT)
    include(FetchContent)
    FetchContent_Declare(
        kvasir_sdk
        GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_SUBMODULES "" SOURCE_SUBDIR fetch-only)
    FetchContent_MakeAvailable(kvasir_sdk)
    set(KVASIR_ROOT "${kvasir_sdk_SOURCE_DIR}")
endif()
include("${KVASIR_ROOT}/cmake/kvasir_locate.cmake")
kvasir_locate(SDK_SUBMODULES uc_log/remote_fmt)
# kvasir::sdk and kvasir::test_support (<support/FakeClock.hpp>, <test_harness.hpp>): <support/...> is the SDK's, this
# repository's own test helpers are <kd/...> (test/kd/).
include(${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake)

enable_testing()

# The library's own CMakeLists, for the same mp-units wiring a firmware gets.
add_subdirectory(${KVASIR_DEVICES_ROOT} kvasir_devices_build)

# string_constant alone: the USB descriptors lift their strings into types with it (on a target it arrives with uc_log).
# SYSTEM: not held to this warning set.
add_library(kvasir_devices_test_string_constant INTERFACE)
target_include_directories(kvasir_devices_test_string_constant SYSTEM
                           INTERFACE ${KVASIR_ROOT}/uc_log/remote_fmt/string_constant/src)
# remote_fmt as well: LogStubs.hpp checks every log line with it, as the target build does.
add_library(kvasir_devices_test_log INTERFACE)
target_include_directories(kvasir_devices_test_log SYSTEM INTERFACE ${KVASIR_ROOT}/uc_log/remote_fmt/src)
target_link_libraries(kvasir_devices_test_log INTERFACE kvasir_devices_test_string_constant)

# The warning and language set the firmware build enforces, from the SDK's own list
# (Kvasir_SDK/cmake/kvasir_warnings.cmake, which the toolchain files read too), with ASan+UBSan on clang.
include(${KVASIR_ROOT}/cmake/kvasir_warnings.cmake)
function(kvasir_devices_warnings target)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "GCC_OPTIONS")
    kvasir_host_test_flags(${target} GCC_OPTIONS ${arg_GCC_OPTIONS})
endfunction()

# The log stubs come first in every test TU: forced, so no include order (or clang-format) can put a driver before them.
# Not for the headers suite, which compiles the library without logging.
function(kvasir_devices_force_log_stubs target)
    target_compile_options(${target} PRIVATE -include ${KVASIR_DEVICES_ROOT}/test/kd/LogStubs.hpp)
    target_link_libraries(${target} PRIVATE kvasir_devices_test_log)
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
    kvasir_devices_force_log_stubs(${name})
    kvasir_devices_warnings(${name} GCC_OPTIONS ${arg_GCC_OPTIONS})
    add_test(NAME ${name} COMMAND ${name})
    # Without halt_on_error a UBSan report is printed and the test still passes.
    set_tests_properties(${name} PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
endfunction()

# kvasir_devices_compile_fail(<prefix> SOURCE <file> DEFINE_PREFIX <P> CASES "<CASE>|<reason regex>" ...)
#
# What must not compile: each case is an object library outside `all`, built from SOURCE with <P>_<CASE> defined, which
# ctest builds and judges by the reason in the compiler's error, so a failure for any other reason (a typo, a missing
# include) does not pass. The ctest name is <prefix>_<case>, lower case.
function(kvasir_devices_compile_fail prefix)
    cmake_parse_arguments(PARSE_ARGV 1 arg "" "SOURCE;DEFINE_PREFIX" "CASES")
    foreach(case_reason IN LISTS arg_CASES)
        string(REPLACE "|" ";" case_reason "${case_reason}")
        list(GET case_reason 0 case)
        list(GET case_reason 1 reason)
        string(TOLOWER "${prefix}_${case}" target)
        add_library(${target} OBJECT EXCLUDE_FROM_ALL ${arg_SOURCE})
        target_compile_definitions(${target} PRIVATE ${arg_DEFINE_PREFIX}_${case})
        target_link_libraries(${target} PRIVATE kvasir::devices kvasir::test_support)
        target_include_directories(${target} PRIVATE ${KVASIR_DEVICES_ROOT}/test ${CMAKE_CURRENT_SOURCE_DIR})
        kvasir_devices_force_log_stubs(${target})
        add_test(NAME ${target} COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${target})
        # Each case runs a build of this very tree: two at once (ctest -j) corrupt its .ninja_deps for good.
        set_tests_properties(${target} PROPERTIES PASS_REGULAR_EXPRESSION "${reason}" RESOURCE_LOCK build_tree)
    endforeach()
endfunction()
