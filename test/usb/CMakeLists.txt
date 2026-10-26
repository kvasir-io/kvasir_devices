# The USB device (src/kvasir/Devices/USB) against a fake controller with the host on its other side, on the host. The
# SDK is used header-only (RateLimiter, Atomic/Queue, its FakeClock, string_constant); cmake/KvasirDevicesTest.cmake
# finds or fetches it.

# One executable per topic, each run against both kinds of controller (FakeUsbBackend.hpp).
foreach(test control_test cdc_test bulk_test hid_test fuzz_test)
    # -Wno-missing-field-initializers: gcc counts the descriptors' base class (which a designated initialiser cannot
    # name, and which has defaults) as a member left out.
    kvasir_devices_test(usb_${test} SOURCE ${test}.cpp GCC_OPTIONS -Wno-missing-field-initializers)
endforeach()
