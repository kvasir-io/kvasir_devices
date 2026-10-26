# The display drivers (src/kvasir/Devices/Display/) against mock buses.

foreach(test panel_test ssd1680_test spi_dcs_bus_test ssd1680_spi_test)
    kvasir_devices_test(display_${test} SOURCE ${test}.cpp)
endforeach()

# A queued display bus too short for what DcsPanel hands it in one turn (burst_fail.cpp), judged by the named check.
kvasir_devices_compile_fail(
    display_burst_fail
    SOURCE
    burst_fail.cpp
    DEFINE_PREFIX
    BURST_FAIL
    CASES
    "SLOTS_SHORT|busHoldsFormatBurst<4(UL)?, 2(UL)?>"
    "QUEUE_SHORT|masterQueueHoldsCommandSlots<6(UL)?, 4(UL)?>")
