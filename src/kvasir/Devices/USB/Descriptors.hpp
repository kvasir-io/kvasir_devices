#pragma once

#include <array>
#include <cassert>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <kvasir/Util/StaticVector.hpp>
#include <memory>
#include <span>
#include <string_constant/string_constant.hpp>
#include <string_view>
#include <type_traits>
#include <utility>

namespace Kvasir::USB {

enum class DescriptorType : std::uint8_t {
    device                    = 1,
    configuration             = 2,
    string                    = 3,
    interface                 = 4,
    endpoint                  = 5,
    deviceQualifier           = 6,
    otherSpeedConfiguration   = 7,
    interfacePower            = 8,
    otg                       = 9,
    debug                     = 10,
    interfaceAssociation      = 11,
    security                  = 12,
    key                       = 13,
    encryptionType            = 14,
    bos                       = 15,
    deviceCapability          = 16,
    wirelessEndpointCompanion = 17,
    csInterface               = 36
};

enum class DescriptorSubType : std::uint8_t {
    cdcHeader         = 0x00,
    cdcCallManagement = 0x01,
    cdcUnion          = 0x06,
    cdcAcmDescriptor  = 0x02
};

enum class DeviceClass : std::uint8_t {
    useInterfaceClass = 0x00,   // the class is each interface's (a HID device, say)
    communication     = 0x02,
    commonClass       = 0x02,   // bDeviceSubClass of the composite-device triple EF/02/01
    cdcData           = 0x0A,
    miscellaneous     = 0xEF,
};

// Bit 7 is reserved and must always be set.
enum class ConfigurationAttributes : std::uint8_t {
    remoteWakeup = 0x20,
    selfPowered  = 0xC0,
    busPowered   = 0x80
};

enum class EndpointTransferType : std::uint8_t {
    control     = 0x0,
    isochronous = 0x1,
    bulk        = 0x2,
    interrupt   = 0x3,
};

enum class EndpointDirection : std::uint8_t { out = 0x00, in = 0x80 };

// Control transfer stages (USB 2.0 Specification Chapter 5.5.5)
enum class ControlStage : std::uint8_t {
    idle = 0,
    setup,
    data,
    status,
    stall,
};

constexpr std::uint8_t makeEndpointAddress(EndpointDirection direction,
                                           std::uint8_t      endpointID) {
    return static_cast<std::uint8_t>(direction) | endpointID;
}

struct SetupPacket {
    enum class Direction : std::uint8_t { hostToDevice, deviceToHost };

    enum class Type : std::uint8_t { standard, classT, vendor, reserved };

    enum class Recipient : std::uint8_t { device, interface, endpoint, other };

    enum class Request : std::uint8_t {
        getStatus        = 0,
        clearFeature     = 1,
        setFeature       = 3,
        setAddress       = 5,
        getDescriptor    = 6,
        setDescriptor    = 7,
        getConfiguration = 8,
        setConfiguration = 9,
        getInterface     = 10,
        setInterface     = 11,
    };

    [[nodiscard]] Direction direction() const {
        return static_cast<Direction>((bmRequestType & 0x80) >> 7);
    }

    [[nodiscard]] Type type() const { return static_cast<Type>((bmRequestType & 0x60) >> 5); }

    [[nodiscard]] Recipient recipient() const {
        return static_cast<Recipient>(bmRequestType & 0x1f);
    }

    [[nodiscard]] DescriptorType descriptorType() const {
        return static_cast<DescriptorType>(wValue >> 8);
    }

    uint8_t  bmRequestType;
    Request  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

// Copied byte for byte out of the controller's setup buffer.
static_assert(sizeof(SetupPacket) == 8);

/// Full speed: every endpoint's packets are 64 bytes. A backend's MaxPacketSize has to agree.
inline constexpr std::size_t maxPacketSize = 64;

namespace Detail {
    static constexpr std::uint16_t bcdUSB{0x0200};

    // bLength is not defaulted to sizeof(Derived): gcc cannot constant-evaluate bit_cast of a
    // class whose base default-initializes a member with sizeof(Derived) ("sorry, unimplemented").
    // generateArray() stamps it via descriptorLength() instead.
    template<typename Derived, DescriptorType DT>
    struct DescriptorBase {
        std::uint8_t   bLength{0};
        DescriptorType bDescriptorType{DT};

        static constexpr std::uint8_t descriptorLength() { return sizeof(Derived); }
    };

    template<typename Derived, DescriptorSubType DST>
    struct InterfaceDescriptorBase : DescriptorBase<Derived, DescriptorType::csInterface> {
        DescriptorSubType bDescriptorSubType{DST};
    };
}   // namespace Detail

namespace Descriptors {

    struct [[gnu::packed]] Device : Detail::DescriptorBase<Device, DescriptorType::device> {
        std::uint16_t bcdUSB{Detail::bcdUSB};
        DeviceClass   bDeviceClass{};
        DeviceClass   bDeviceSubClass{};
        std::uint8_t  bDeviceProtocol{};
        std::uint8_t  bMaxPacketSize0{};
        std::uint16_t idVendor{};
        std::uint16_t idProduct{};
        std::uint16_t bcdDevice{};
        std::uint8_t  iManufacturer{};
        std::uint8_t  iProduct{};
        std::uint8_t  iSerialNumber{};
        std::uint8_t  bNumConfigurations{};
    };

    struct [[gnu::packed]] Configuration
      : Detail::DescriptorBase<Configuration, DescriptorType::configuration> {
        std::uint16_t           wTotalLength{};
        std::uint8_t            bNumInterfaces{};
        std::uint8_t            bConfigurationValue{};
        std::uint8_t            iConfiguration{};
        ConfigurationAttributes bmAttributes{ConfigurationAttributes::busPowered};
        std::uint8_t            bMaxPower{};
    };

    struct [[gnu::packed]] Interface
      : Detail::DescriptorBase<Interface, DescriptorType::interface> {
        std::uint8_t bInterfaceNumber{};
        std::uint8_t bAlternateSetting{};
        std::uint8_t bNumEndpoints{};
        std::uint8_t bInterfaceClass{};
        std::uint8_t bInterfaceSubClass{};
        std::uint8_t bInterfaceProtocol{};
        std::uint8_t iInterface{};
    };

    struct [[gnu::packed]] InterfaceAssociation
      : Detail::DescriptorBase<InterfaceAssociation, DescriptorType::interfaceAssociation> {
        std::uint8_t bFirstInterface{};
        std::uint8_t bInterfaceCount{};
        std::uint8_t bFunctionClass{};
        std::uint8_t bFunctionSubClass{};
        std::uint8_t bFunctionProtocol{};
        std::uint8_t iFunction{};
    };

    struct [[gnu::packed]] Endpoint : Detail::DescriptorBase<Endpoint, DescriptorType::endpoint> {
        std::uint8_t         bEndpointAddress{};
        EndpointTransferType bmAttributes{};
        std::uint16_t        wMaxPacketSize{};
        std::uint8_t         bInterval{};
    };

    /// An endpoint descriptor: endpoint `number` in direction `dir`, `maxPacket` bytes a packet
    /// (full speed: 64 for bulk), polled every `interval` ms (interrupt endpoints; 0 otherwise).
    constexpr Endpoint endpoint(EndpointDirection    dir,
                                std::size_t          number,
                                EndpointTransferType type,
                                std::size_t          maxPacket = maxPacketSize,
                                std::uint8_t         interval  = 0) {
        return Endpoint{
          .bEndpointAddress{makeEndpointAddress(dir, static_cast<std::uint8_t>(number))},
          .bmAttributes{type},
          .wMaxPacketSize{static_cast<std::uint16_t>(maxPacket)},
          .bInterval{interval}};
    }

    struct [[gnu::packed]] Bos : Detail::DescriptorBase<Bos, DescriptorType::bos> {
        std::uint16_t wTotalLength{};
        std::uint8_t  bNumDeviceCaps{};
    };

    namespace Detail {
        template<typename T>
        constexpr std::size_t getActualSize() {
            // For std::array, use tuple_size to get the actual element count
            if constexpr(requires { typename std::tuple_size<T>::type; }) {
                return std::tuple_size_v<T>;
            } else {
                return sizeof(T);
            }
        }
    }   // namespace Detail

    /// The descriptors laid end to end as bytes (each stamped with its bLength): a configuration block.
    template<typename... Ts>
    constexpr auto generateArray(Ts const&... args) {
        std::array<std::byte, (Detail::getActualSize<Ts>() + ...)> buffer{};
        std::size_t                                                offset = 0;

        auto const copyOne = [&]<typename T>(T const& arg) {
            constexpr std::size_t actualSize = Detail::getActualSize<T>();
            if constexpr(actualSize > 0) {
                auto byteArray = [&] {
                    if constexpr(requires { T::descriptorLength(); }) {
                        T stamped       = arg;
                        stamped.bLength = T::descriptorLength();
                        return std::bit_cast<std::array<std::byte, sizeof(T)>>(stamped);
                    } else {
                        return std::bit_cast<std::array<std::byte, sizeof(T)>>(arg);
                    }
                }();
                for(std::size_t i = 0; i < actualSize; ++i) { buffer[offset + i] = byteArray[i]; }
                offset += actualSize;
            }
        };

        (copyOne(args), ...);
        return buffer;
    }

    namespace Detail {
        // Calls f(descriptor) for each descriptor of a bLength-chained block, e.g. a whole
        // configuration descriptor.
        template<typename F>
        constexpr void forEachDescriptor(std::span<std::byte const> block,
                                         F                          f) {
            while(block.size() >= 2) {
                auto const length = std::to_integer<std::size_t>(block[0]);
                if(length < 2 || length > block.size()) { return; }
                f(block.first(length));
                block = block.subspan(length);
            }
        }

    }   // namespace Detail

    template<std::uint16_t DeviceVersion,
             std::uint16_t VendorID,
             std::uint16_t ProductID,
             std::uint8_t  ManufacturerStringID,
             std::uint8_t  ProductStringID,
             std::uint8_t  SerialNumberStringID,
             DeviceClass   Class,
             DeviceClass   SubClass,
             std::uint16_t BcdUSB = Kvasir::USB::Detail::bcdUSB>
    consteval auto makeDeviceDescriptorArray() {
        constexpr USB::Descriptors::Device deviceDescriptor{
          .bcdUSB{BcdUSB},
          .bDeviceClass{Class},
          .bDeviceSubClass{SubClass},
          // EF/02/01 is the interface association triple; a device that leaves the class to its
          // interfaces has protocol 0 with it.
          .bDeviceProtocol{Class == DeviceClass::miscellaneous ? std::uint8_t{1} : std::uint8_t{0}},
          .bMaxPacketSize0{Kvasir::USB::maxPacketSize},
          .idVendor{VendorID},
          .idProduct{ProductID},
          .bcdDevice{DeviceVersion},
          .iManufacturer{ManufacturerStringID},
          .iProduct{ProductStringID},
          .iSerialNumber{SerialNumberStringID},
          .bNumConfigurations{1}};
        return generateArray(deviceDescriptor);
    }

    template<typename... Interfaces>
    consteval auto makeConfigDescriptorArray(std::uint16_t busPower,
                                             bool          busPowered,
                                             std::uint8_t  numInterfaces,
                                             Interfaces const&... interfaces) {
        assert(500 >= busPower);
        auto const remainingConfig = generateArray(interfaces...);

        USB::Descriptors::Configuration const configDescriptor{
          .wTotalLength{sizeof(remainingConfig) + sizeof(USB::Descriptors::Configuration)},
          .bNumInterfaces{numInterfaces},
          .bConfigurationValue{1},
          .bmAttributes{busPowered ? ConfigurationAttributes::busPowered
                                   : ConfigurationAttributes::selfPowered},
          .bMaxPower{static_cast<std::uint8_t>(busPower / 2)}};

        return generateArray(configDescriptor, remainingConfig);
    }

}   // namespace Descriptors

template<typename Buffer>
struct DescriptorString {
    Buffer buffer{};

    constexpr DescriptorString(std::string_view s) {
        assert(buffer.max_size() >= s.size() * 2);

        if constexpr(requires { buffer.resize(0); }) {
            buffer.resize(s.size() * 2);
        } else {
            assert(buffer.max_size() == s.size() * 2);
        }

        assert(buffer.size() == s.size() * 2);
        auto pos = buffer.begin();
        for(auto const c : s) {
            *pos = static_cast<std::byte>(c);
            ++pos;
            *pos = std::byte{0};
            ++pos;
        }
    }

    [[nodiscard]] constexpr std::size_t size() const { return buffer.size(); }

    [[nodiscard]] constexpr std::byte const* begin() const { return buffer.data(); }

    [[nodiscard]] constexpr std::byte const* end() const { return buffer.data() + buffer.size(); }

    [[nodiscard]] constexpr std::byte const* data() const { return buffer.data(); }

    [[nodiscard]] constexpr DescriptorString const& get() const { return *this; }
};

template<std::size_t ByteCount>
consteval std::size_t checkedDescriptorStringSize() {
    static_assert(ByteCount <= 252, "String too long for USB string descriptor (max 126 chars)");
    return ByteCount;
}

template<std::size_t N>
DescriptorString(char const (&)[N])
  -> DescriptorString<std::array<std::byte,
                                 checkedDescriptorStringSize<(N - 1) * 2>()>>;
DescriptorString(std::string_view s) -> DescriptorString<Kvasir::StaticVector<std::byte,
                                                                              252>>;
template<char... Chars>
DescriptorString(sc::StringConstant<Chars...> sc)
  -> DescriptorString<std::array<std::byte,
                                 checkedDescriptorStringSize<sizeof...(Chars) * 2>()>>;

template<typename F>
struct RuntimeDescriptorString {
    F f{};

    constexpr RuntimeDescriptorString(F f_) : f{f_} {}

    using DT = DescriptorString<Kvasir::StaticVector<std::byte, 252>>;

    struct RuntimeDescriptorStringImpl : DT {
        template<typename T>
        RuntimeDescriptorStringImpl(T const& v)
          : DT{[&]() {
              if constexpr(requires { std::string_view{v}; }) {
                  return DT{std::string_view{v}};
              } else {
                  std::array<char, 126> b;
                  auto const            ret = std::to_chars(b.begin(), b.end(), v);
                  assert(ret.ec == std::errc{});
                  return DT{
                    std::string_view{b.begin(), ret.ptr}
                  };
              }
          }()} {}
    };

    [[nodiscard]] RuntimeDescriptorStringImpl get() const {
        return RuntimeDescriptorStringImpl{f()};
    }
};

template<typename F>
RuntimeDescriptorString(F f) -> RuntimeDescriptorString<F>;

template<typename I,
         typename T>
I insertStringDescriptor(T const& descriptor,
                         I        first,
                         I        last) {
    // Check if buffer has enough space
    if(static_cast<std::size_t>(std::distance(first, last)) < descriptor.size()) {
        return first;   // Return unchanged position on error
    }
    std::memcpy(std::addressof(*first), descriptor.data(), descriptor.size());
    return std::next(first, std::ssize(descriptor));
}

// Returns iterator to end of inserted data, or 'first' unchanged if index is invalid
template<typename IT,
         typename T>
IT insertStringDescriptor(std::size_t index,
                          T const&    descriptors,
                          IT          first,
                          IT          last) {
    static constexpr auto size = std::tuple_size_v<T>;
    if(index >= size) {
        return first;   // index is out of range
    }

    IT   result = first;
    bool found  = false;

    auto action = [&](auto i) {
        static constexpr std::size_t idx = decltype(i)::value;
        if(idx == index) {
            result = insertStringDescriptor(std::get<idx>(descriptors).get(), first, last);
            found  = true;
            return false;
        }
        return true;
    };

    [&]<std::size_t... Ns>(std::index_sequence<Ns...>) {
        (action(std::integral_constant<std::size_t, Ns>{}) && ...);
    }(std::make_index_sequence<size>{});

    return found ? result : first;
}
}   // namespace Kvasir::USB

// Last in the file, and before any header that logs a SetupPacket: an explicit specialization
// has to be seen before its first use.
#if defined(KVASIR_LOG_FORMATTERS) && __has_include("remote_fmt/remote_fmt.hpp")
    #include "remote_fmt/remote_fmt.hpp"

namespace remote_fmt {
template<>
struct formatter<Kvasir::USB::SetupPacket> {
    template<typename Printer>
    constexpr auto format(Kvasir::USB::SetupPacket const& pkt,
                          Printer&                        printer) const {
        // The request by name and the raw fields. Direction, type and recipient are
        // bmRequestType's bits and the descriptor type is wValue's high byte: decoded here
        // each was an enum switch per call site (~650 B for one STALL warning).
        return format_to(printer,
                         SC_LIFT("request:{} bmRequestType:{:#x} bRequest:{:#x} wValue:{:#x} "
                                 "wIndex:{:#x} wLength:{:#x}"),
                         pkt.bRequest,
                         pkt.bmRequestType,
                         std::to_underlying(pkt.bRequest),
                         pkt.wValue,
                         pkt.wIndex,
                         pkt.wLength);
    }
};
}   // namespace remote_fmt
#endif
