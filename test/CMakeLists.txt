# Every host test suite of the library in one build tree (what the justfile configures): the I2C device engine and the
# chip descriptions (i2c/), the USB device (usb/). Each of the two is a project of its own as well, for a build of just
# that one: cmake -S test/usb -B build/usb.
cmake_minimum_required(VERSION 3.28)

project(kvasir_devices_test LANGUAGES CXX)

enable_testing()

add_subdirectory(i2c)
add_subdirectory(usb)
