# Devices bit-banged on one pin: the 11AA02E48 (UNI/O) against a model of the part on a simulated line, the 1-Wire
# master (OneWire.hpp) against a model of a slave.

kvasir_devices_test(gpio_eeprom_11aa02e48_test SOURCE eeprom_11aa02e48_test.cpp)
kvasir_devices_test(gpio_max31820_test SOURCE max31820_test.cpp)
kvasir_devices_test(gpio_one_wire_test SOURCE one_wire_test.cpp)
