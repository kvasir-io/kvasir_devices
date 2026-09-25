#pragma once

#include "../Log.hpp"
#include "Descriptors.hpp"
#include "Mixins.hpp"

#include <cstddef>
#include <type_traits>

/// The reset interface picotool speaks: a vendor interface without endpoints whose two requests
/// reboot the chip, into its bootloader (0x01) or into the application again (0x02). How a chip
/// does either is its package's to say:
///
///     struct Actions {
///         static void bootloader();   // does not return
///         static void reboot();       // does not return
///     };
///
/// and the chip package binds it to the five-parameter shape a device's mixin list takes. The two
/// callbacks run first, from inside the USB interrupt - to let a log line drain, say.
namespace Kvasir::USB::VendorReset {

template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber,
         typename Actions,
         typename BeforeBootselCallback = void,
         typename BeforeFlashCallback   = void>
struct Mixin {
private:
    friend Derived;
    friend struct Kvasir::USB::detail::MixinTraits;   // Grants access to helper functions

    static constexpr std::size_t InterfaceCount = 1;
    static constexpr std::size_t EndpointCount  = 0;

    static constexpr auto InterfaceDescriptor
      = Kvasir::USB::Descriptors::Interface{.bInterfaceNumber   = FirstInterfaceNumber,
                                            .bAlternateSetting  = 0,
                                            .bNumEndpoints      = 0,
                                            .bInterfaceClass    = 255,
                                            .bInterfaceSubClass = 0,
                                            .bInterfaceProtocol = 1};

    // Callbacks
    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        if(pkt.type() == SetupPacket::Type::standard || pkt.wIndex != FirstInterfaceNumber
           || pkt.recipient() != SetupPacket::Recipient::interface)
        {
            return false;
        }

        static constexpr SetupPacket::Request REQUEST_BOOTSEL{0x01};
        static constexpr SetupPacket::Request REQUEST_FLASH{0x02};

        if(pkt.bRequest == REQUEST_BOOTSEL) {
            UC_LOG_I("USB: Rebooting to the bootloader");
            if constexpr(!std::is_same_v<BeforeBootselCallback, void>) {
                BeforeBootselCallback{}();
            }
            Actions::bootloader();
            return true;
        } else if(pkt.bRequest == REQUEST_FLASH) {
            UC_LOG_I("USB: Rebooting to flash");
            if constexpr(!std::is_same_v<BeforeFlashCallback, void>) { BeforeFlashCallback{}(); }
            Actions::reboot();
            return true;
        }
        return false;
    }
};
}   // namespace Kvasir::USB::VendorReset
