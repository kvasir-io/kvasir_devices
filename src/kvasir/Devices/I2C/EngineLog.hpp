#pragma once

/// The engine's log lines, once each instead of once per device.
///
/// `Device<>` is instantiated per device, so every `UC_LOG_*` in it used to be printed into
/// the image that often: the "up" line alone was 76 bytes x 55 devices on i2c_testing's bench
/// (2026-09-20). None of these lines says anything that is not a value -- the chip's name and
/// address are arguments like the rest -- so one copy serves every device, and a call site is
/// then the arguments and a call.
///
/// `KVASIR_LOG_SHARED` (Devices/Log.hpp) is what keeps that true: the helpers are `noinline`
/// only while logging is compiled in. In a build without it the bodies are empty and the
/// attribute would leave 55 calls to an empty function, so it goes away with the logging.
#include "../Link.hpp"
#include "../Log.hpp"

#include <chrono>
#include <cstdint>
#include <string_view>

namespace Kvasir::I2C::detail {

/// The bring-up finished: the part is there and, unless it failed its Identity, is the chip
/// the description is for.
KVASIR_LOG_SHARED void logUp([[maybe_unused]] std::string_view name,
                             [[maybe_unused]] std::uint8_t     address,
                             [[maybe_unused]] bool             identified) {
    if(identified) {
        UC_LOG_I("{} at {:#04x}: up", name, address);
    } else {
        UC_LOG_W("{} at {:#04x}: up, but not the chip this description is for", name, address);
    }
}

/// What the device is, where, and how it is doing: Bus::logHealth's line, per device.
KVASIR_LOG_SHARED void logHealth([[maybe_unused]] std::string_view name,
                                 [[maybe_unused]] std::uint8_t     address,
                                 [[maybe_unused]] Link             link,
                                 [[maybe_unused]] std::uint32_t    samples,
                                 [[maybe_unused]] std::uint32_t    rejected,
                                 [[maybe_unused]] std::uint32_t    errors) {
    UC_LOG_I("{} at {:#04x}: {} samples {} rejected {} errors {}",
             name,
             address,
             link,
             samples,
             rejected,
             errors);
}

/// The net under the bus's "exactly one callback per request": it never came.
KVASIR_LOG_SHARED void logNoBusAnswer([[maybe_unused]] std::string_view          name,
                                      [[maybe_unused]] std::uint8_t              address,
                                      [[maybe_unused]] std::chrono::milliseconds timeout) {
    UC_LOG_W("{} at {:#04x}: no answer from the bus in {} -- the request is given up on",
             name,
             address,
             timeout);
}

/// The bridge this device is behind came back, or went away (Bridge.hpp).
KVASIR_LOG_SHARED void logBridge([[maybe_unused]] std::string_view name,
                                 [[maybe_unused]] std::uint8_t     address,
                                 [[maybe_unused]] bool             active) {
    if(active) {
        UC_LOG_I("{} at {:#04x}: the bridge is active again", name, address);
    } else {
        UC_LOG_I("{} at {:#04x}: offline, its bridge is not active", name, address);
    }
}

/// setup() turned the bring-up down: something answers at the address, but it is not this chip.
KVASIR_LOG_SHARED void logUnidentified([[maybe_unused]] std::string_view          name,
                                       [[maybe_unused]] std::uint8_t              address,
                                       [[maybe_unused]] std::chrono::milliseconds retry) {
    UC_LOG_W(
      "{} at {:#04x}: answers, but is not the chip this description is for "
      "-- bring-up again every {}",
      name,
      address,
      retry);
}

/// An Identity register read something the data sheet does not allow for this part.
KVASIR_LOG_SHARED void logIdentityMismatch([[maybe_unused]] std::string_view name,
                                           [[maybe_unused]] std::uint8_t     address,
                                           [[maybe_unused]] std::string_view check,
                                           [[maybe_unused]] std::uint16_t    reg,
                                           [[maybe_unused]] std::uint32_t    got,
                                           [[maybe_unused]] std::uint32_t    expect,
                                           [[maybe_unused]] std::uint32_t    mask) {
    UC_LOG_W("{} at {:#04x}: {} ({:#06x}) reads {:#x}, the data sheet says {:#x} under {:#x}",
             name,
             address,
             check,
             reg,
             got,
             expect,
             mask);
}

/// A register that does not keep what was written to it, MaxRetries times over.
KVASIR_LOG_SHARED void logVerifyStuck([[maybe_unused]] std::string_view name,
                                      [[maybe_unused]] std::uint8_t     address,
                                      [[maybe_unused]] std::uint16_t    reg,
                                      [[maybe_unused]] std::uint32_t    mismatches) {
    UC_LOG_W(
      "{} at {:#04x}: register {:#04x} will not hold its value "
      "({} mismatches so far) -- left alone until the next verify",
      name,
      address,
      reg,
      mismatches);
}

/// A sized prepare() asked for more bytes than its group's buffer holds; the read is clamped.
KVASIR_LOG_SHARED void logOversizedRead([[maybe_unused]] std::string_view name,
                                        [[maybe_unused]] std::uint8_t     address,
                                        [[maybe_unused]] std::size_t      asked,
                                        [[maybe_unused]] std::size_t      holds) {
    UC_LOG_W("{} at {:#04x}: asked for {} bytes, the group holds {}", name, address, asked, holds);
}

}   // namespace Kvasir::I2C::detail
