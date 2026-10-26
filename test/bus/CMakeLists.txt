# What the two queued-bus cores (I2C/Controller/QueueCore.hpp, SPI/QueueCore.hpp) share, one scenario list run on both
# through QueueBuses.hpp (the I2C fake of i2c/FakeQueueHw.hpp, the SPI one of spi/FakeQueuedSpi.hpp). Named bus_<topic>
# in ctest.
foreach(test request_queue_contract_test exactly_one_callback_test)
    kvasir_devices_test(bus_${test} SOURCE ${test}.cpp)
endforeach()
