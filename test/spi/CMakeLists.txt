# The SPI drivers (src/kvasir/Devices/SPI/) on the host, over the queued master's core with a fake transfer
# (FakeQueuedSpi.hpp) and a model of the part.

foreach(
    test
    client_test
    max31865_test
    no_part_test
    queue_core_test
    sd_card_test
    engine_spi_test
    ad7124_test
    stream_test
    oracle_test
    ltc2400_test
    ltc2641_test
    nor_flash_test)
    kvasir_devices_test(spi_${test} SOURCE ${test}.cpp)
endforeach()

# QueuedFront.hpp's ceilingHz(): a device ceiling below the slowest clock the block makes does not compile
kvasir_devices_compile_fail(
    spi_front
    SOURCE
    queue_core_test.cpp
    DEFINE_PREFIX
    QUEUE_CORE_FRONT
    CASES
    "BELOW_REACH|rateOutOfReach")

# The SD card as a block device; with the fs library next to this checkout (FS_ROOT, or ../fs) also FAT32 on top of it.
# fs is found by path only (it is not published, so there is nothing to fetch): a lone checkout and CI build the branch
# without it.
if(NOT FS_ROOT)
    get_filename_component(FS_ROOT "${KVASIR_DEVICES_ROOT}/../fs" ABSOLUTE)
endif()
if(EXISTS "${FS_ROOT}/src/fs/fat/Volume.hpp")
    kvasir_devices_test(spi_sd_block_device_test SOURCE sd_block_device_test.cpp INCLUDES ${FS_ROOT}/src)
else()
    kvasir_devices_test(spi_sd_block_device_test SOURCE sd_block_device_test.cpp)
endif()
