# Local development helpers for kvasir_devices.
#
# The library itself is header-only and has no build of its own: the top-level
# CMakeLists.txt is an INTERFACE target meant to be add_subdirectory'd by a
# firmware project. What is buildable here are the host test suites under
# test/, one CMake project: i2c - the I2C device engine and every chip
# description against a fake bus -, spi - the SPI drivers against the queued
# master's core -, usb - the USB device against a fake controller with the host
# on its other side -, display, gpio, input, and headers - every header compiled
# on its own. What they share is test/cmake/KvasirDevicesTest.cmake. Builds live
# in build/<config>/.
#
#   just                    # build + run the tests with the default config
#   just test gcc           # same, with gcc
#   just test clang Bme     # only tests matching "Bme" (and only their executables built)
#   just ci                 # the CI workflow through act, then the layer check
#   just test-all           # every config
#
# The Kvasir SDK is found or fetched: test/cmake/KvasirDevicesTest.cmake takes
# KVASIR_ROOT, else the sibling checkout, else clones it from GitHub - see `kvasir_root`.
#
# `just --list` shows everything.

set shell := ["bash", "-euo", "pipefail", "-c"]

root := source_directory()
self := source_file()   # not justfile(): that names the ROOT justfile when loaded as a module
tree := parent_directory(root)
build_root := root / "build"
jobs := num_cpus()

# Override with `just kvasir_root=/path test` or $KVASIR_ROOT. Passed to CMake only
# when it exists; without it the test tree fetches the SDK.
kvasir_root := env('KVASIR_ROOT', tree / 'Kvasir_SDK')

default_config := "clang"

# clang builds the suites with -Weverything and asan+ubsan (test/cmake/KvasirDevicesTest.cmake),
# gcc with the -Wall -Wextra -Wconversion set - they catch different things.
configs := "clang gcc"

default: test

[private]
flags config:
    #!/usr/bin/env bash
    set -euo pipefail
    case "{{ config }}" in
        clang) echo "clang clang++" ;;
        gcc)   echo "gcc g++" ;;
        *)
            echo "unknown config '{{ config }}'; known: {{ configs }}" >&2
            exit 1 ;;
    esac

# Wipe a build tree whose recorded source directory is not this one any more -
# what a moved checkout leaves behind. Without this CMake refuses outright:
# "the current CMakeCache.txt directory ... is different than the directory".
#
# Also wipes a tree configured with another generator: every recipe here drives
# Ninja, and CMake refuses to switch a tree's generator in place.
[private]
guard dir src:
    #!/usr/bin/env bash
    set -euo pipefail
    cache="{{ dir }}/CMakeCache.txt"
    [ -f "$cache" ] || exit 0
    why=""
    home=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache")
    [ "$home" = "{{ src }}" ] || why="source moved ($home)"
    p=$(sed -n 's/^KVASIR_ROOT:[A-Za-z]*=//p' "$cache" | head -1)
    [ -z "$p" ] || [ -d "$p" ] || why="KVASIR_ROOT gone ($p)"
    gen=$(sed -n 's/^CMAKE_GENERATOR:INTERNAL=//p' "$cache")
    [ "$gen" = Ninja ] || why="generator is '$gen', not Ninja"
    if [ -n "$why" ]; then
        echo "==> wiping {{ dir }}: $why"
        rm -rf "{{ dir }}"
    fi

# Configure build/<config> if it does not exist yet (no-op afterwards)
configure config=default_config: (guard (build_root / config) (root / "test"))
    #!/usr/bin/env bash
    set -euo pipefail
    dir="{{ build_root }}/{{ config }}"
    if [ -f "$dir/build.ninja" ]; then exit 0; fi
    sdk=""
    [ -f "{{ kvasir_root }}/cmake/kvasir_sdk_targets.cmake" ] && sdk="-DKVASIR_ROOT={{ kvasir_root }}"
    read -r cc cxx < <(just --justfile "{{ self }}" flags "{{ config }}")
    launcher=""
    command -v ccache >/dev/null && launcher="-DCMAKE_CXX_COMPILER_LAUNCHER=ccache"
    env CC="$cc" CXX="$cxx" cmake -S "{{ root }}/test" -B "$dir" -G Ninja \
        -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        $sdk $launcher

# Wipe build/<config> and configure it again
reconfigure config=default_config: (clean config) (configure config)

# Build the test binaries of a configuration (incremental)
build config=default_config: (configure config)
    cmake --build "{{ build_root }}/{{ config }}" -j{{ jobs }}

# A must-not-compile case is built by its test itself, so a filter builds only the matching test executables.
# Build and run the tests; FILTER is a ctest -R regex (then only the matching tests are built)
test config=default_config filter="": (configure config)
    #!/usr/bin/env bash
    set -euo pipefail
    dir="{{ build_root }}/{{ config }}"
    if [ -z "{{ filter }}" ]; then
        cmake --build "$dir" -j{{ jobs }}
        exec ctest --test-dir "$dir" --output-on-failure -j{{ jobs }}
    fi
    # a test whose command is its own executable is a target of that name (kvasir_devices_test)
    mapfile -t targets < <(ctest --test-dir "$dir" -N -R '{{ filter }}' --show-only=json-v1 | python3 -c '
    import json, os, sys
    for t in json.load(sys.stdin)["tests"]:
        if t.get("command") and os.path.basename(t["command"][0]) == t["name"]:
            print(t["name"])')
    if [ ${#targets[@]} -ne 0 ]; then
        cmake --build "$dir" -j{{ jobs }} --target "${targets[@]}"
    fi
    ctest --test-dir "$dir" --output-on-failure -j{{ jobs }} -R '{{ filter }}' --no-tests=error

# List the tests of a configuration
list config=default_config: (configure config)
    ctest --test-dir "{{ build_root }}/{{ config }}" -N

# Run the tests in every configuration (continues after failures)
test-all filter="":
    #!/usr/bin/env bash
    set -uo pipefail
    failed=()
    for c in {{ configs }}; do
        echo "==> test $c"
        just --justfile "{{ self }}" test "$c" "{{ filter }}" || failed+=("$c")
    done
    if [ ${#failed[@]} -ne 0 ]; then
        echo "FAILED: ${failed[*]}" >&2
        exit 1
    fi
    echo "all configurations passed"

# Build every configuration
build-all:
    #!/usr/bin/env bash
    set -euo pipefail
    for c in {{ configs }}; do
        echo "==> build $c"
        just --justfile "{{ self }}" build "$c"
    done

# The build-and-test job runs through act (gcc and clang, warnings as errors, against the SDK it fetches from GitHub),
# the layer job's check against the sibling Kvasir_SDK's table.
# What .github/workflows/ci.yml runs: build-and-test, then the layer check
ci:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ root }}"
    docker image inspect archlinux:latest >/dev/null 2>&1 || docker pull archlinux:latest
    act push -j build-and-test --reuse --pull=false
    python3 -B "{{ kvasir_root }}/tools/layer_check.py" --repo "kvasir_devices={{ root }}"

# Format every header and test source in place (.clang-format at the repo root)
format:
    clang-format -i $(find src test -name '*.hpp' -o -name '*.cpp')

# Fail if anything is not formatted, without changing it
format-check:
    clang-format --dry-run -Werror $(find src test -name '*.hpp' -o -name '*.cpp')

# Remove build/<config>
clean config=default_config:
    rm -rf "{{ build_root }}/{{ config }}"

# Remove every build directory
clean-all:
    rm -rf "{{ build_root }}"

# --- clang-tidy ----------------------------------------------------------------

# Kvasir_SDK/tools/tidy.py with the SDK's .clang-tidy over build/clang: findings in this repository fail, the
# other libraries' count as foreign. `just tidy --changed`, `just tidy --fix`, `--audit <check>`, `--only '<glob>'`.
# clang-tidy over the clang host tree (exit 0 clean, 1 findings)
[positional-arguments]
tidy *args: (configure "clang")
    python3 -B "{{ kvasir_root }}/tools/tidy.py" --tree "{{ build_root }}/clang" --repo "{{ root }}" "$@"

# The configured clang tree `tidy` reads, if there is one (the workspace's `just tidy` collects them)
tidy-trees:
    @[ ! -f "{{ build_root }}/clang/compile_commands.json" ] || echo "{{ build_root }}/clang"
