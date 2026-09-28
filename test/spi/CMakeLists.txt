# The SPI drivers (src/kvasir/Devices/SPI/) on the host, over the queued master's core with a fake transfer
# (FakeQueuedSpi.hpp) and a model of the part.
#
# cmake -S test/spi -B build/spi && cmake --build build/spi --parallel && ctest --test-dir build/spi
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_spi_test LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/KvasirDevicesTest.cmake)

foreach(
    test
    max31865_test
    no_part_test
    queue_core_test
    sd_card_test
    engine_spi_test
    ad7124_test
    stream_test
    oracle_test)
    kvasir_devices_test(spi_${test} SOURCE ${test}.cpp)
endforeach()

# The SD card as a block device; with the fs library next to this checkout (FS_ROOT, or ../fs) also FAT32 on top of it.
if(NOT FS_ROOT)
    get_filename_component(FS_ROOT "${KVASIR_DEVICES_ROOT}/../fs" ABSOLUTE)
endif()
if(EXISTS "${FS_ROOT}/src/fs/fat/Volume.hpp")
    kvasir_devices_test(spi_sd_block_device_test SOURCE sd_block_device_test.cpp INCLUDES ${FS_ROOT}/src)
else()
    kvasir_devices_test(spi_sd_block_device_test SOURCE sd_block_device_test.cpp)
endif()
