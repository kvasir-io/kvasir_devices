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

// The two requests (bRequest of a vendor request to the interface).
inline constexpr SetupPacket::Request bootselRequest{0x01};
inline constexpr SetupPacket::Request flashRequest{0x02};

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
    friend struct Kvasir::USB::MixinTraits;   // Grants access to helper functions

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
    static bool setupPacketRequestCallback(SetupPacket const& pkt) {
        if(pkt.type() == SetupPacket::Type::standard || pkt.wIndex != FirstInterfaceNumber
           || pkt.recipient() != SetupPacket::Recipient::interface)
        {
            return false;
        }

        if(pkt.bRequest == bootselRequest) {
            UC_LOG_D("USB: Rebooting to the bootloader");
            if constexpr(!std::is_same_v<BeforeBootselCallback, void>) {
                BeforeBootselCallback{}();
            }
            Actions::bootloader();
            return true;
        }
        if(pkt.bRequest == flashRequest) {
            UC_LOG_D("USB: Rebooting to flash");
            if constexpr(!std::is_same_v<BeforeFlashCallback, void>) { BeforeFlashCallback{}(); }
            Actions::reboot();
            return true;
        }
        return false;
    }
};
}   // namespace Kvasir::USB::VendorReset
