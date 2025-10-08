#pragma once
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Util/StaticString.hpp"
#include "kvasir/Util/StaticVector.hpp"
#include "kvasir/Util/concepts.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "uc_log/uc_log.hpp"

#include <bit>
#include <cassert>
#include <charconv>
#include <expected>

struct Header {
    std::uint16_t address{};
    std::uint8_t  control{};

    enum class Size { variable = 0, _1 = 1, _2 = 2, _4 = 3 };

    struct BlockSelect {
        std::uint8_t bs;

        static constexpr BlockSelect CommonRegister() { return BlockSelect{0}; }

        static constexpr BlockSelect SocketRegister(std::size_t n) {
            assert(7 >= n);
            return BlockSelect{static_cast<std::uint8_t>(4 * n + 1)};
        }

        static constexpr BlockSelect SocketTxBuffer(std::size_t n) {
            assert(7 >= n);
            return BlockSelect{static_cast<std::uint8_t>(4 * n + 2)};
        }

        static constexpr BlockSelect SocketRxBuffer(std::size_t n) {
            assert(7 >= n);
            return BlockSelect{static_cast<std::uint8_t>(4 * n + 3)};
        }
    };

    enum class ReadWrite { Read, Write };

    constexpr Header(std::uint16_t      address_,
                     ReadWrite          rw,
                     BlockSelect const& bs,
                     Size               size)
      : address{address_}
      , control{static_cast<std::uint8_t>(bs.bs << 3 | ((rw == ReadWrite::Read ? 0 : 1) << 2)
                                          | static_cast<int>(size))} {}

    constexpr Header(std::byte          address_,
                     ReadWrite          rw,
                     BlockSelect const& bs,
                     Size               size)
      : Header{static_cast<std::uint16_t>(address_),
               rw,
               bs,
               size} {}

    template<typename A>
    constexpr Header(A                  address_,
                     ReadWrite          rw,
                     BlockSelect const& bs,
                     Size               size)
      : Header{static_cast<std::uint16_t>(address_),
               rw,
               bs,
               size} {
        static_assert(sizeof(std::uint16_t) >= sizeof(A));
    }

    template<typename I>
    constexpr I write(I first,
                      I last) const {
        using T = std::remove_cvref_t<decltype(*first)>;
        assert(std::distance(first, last) >= 3);
        *first++ = static_cast<T>((address & 0xff00) >> 8);
        *first++ = static_cast<T>(address & 0x00ff);
        *first++ = static_cast<T>(control);
        return first;
    }
};

struct MacAddress {
    std::array<std::byte, 6> octets;

    static constexpr auto address_string_to_octets(std::string_view address_string) {
        std::array<std::byte, 6> octets;

        auto pos = address_string.begin();
        auto end = address_string.end();
        for(auto& b : octets) {
            std::uint8_t v;
            auto         res = std::from_chars(pos, end, v, 16);
            assert(res.ec == std::errc{});
            b = static_cast<std::byte>(v);
            if(&b != &octets.back()) {
                assert(res.ptr != end);
                assert(*res.ptr == ':');
                pos = res.ptr + 1;
            } else {
                pos = res.ptr;
            }
        }
        assert(pos == end);

        return octets;
    }

    constexpr MacAddress(std::string_view address_string)
      : octets{address_string_to_octets(address_string)} {}

    constexpr MacAddress() = default;

    template<typename I>
    constexpr I write(I first,
                      I last) const {
        assert(static_cast<std::size_t>(std::distance(first, last)) >= octets.size());
        return std::copy(octets.begin(), octets.end(), first);
    }
};

constexpr MacAddress operator""_mac(char const* str,
                                    std::size_t n) {
    return MacAddress{
      std::string_view{str, n}
    };
}

struct IPAddress {
    std::array<std::byte, 4> octets;

    static constexpr auto address_string_to_octets(std::string_view address_string) {
        std::array<std::byte, 4> octets;
        auto                     pos = address_string.begin();
        auto                     end = address_string.end();
        for(auto& b : octets) {
            std::uint8_t v;
            auto         res = std::from_chars(pos, end, v, 10);
            assert(res.ec == std::errc{});
            b = static_cast<std::byte>(v);
            if(&b != &octets.back()) {
                assert(res.ptr != end);
                assert(*res.ptr == '.');
                pos = res.ptr + 1;
            } else {
                pos = res.ptr;
            }
        }
        assert(pos == end);

        return octets;
    }

    template<typename I>
    constexpr IPAddress(I first,
                        I last) {
        assert(static_cast<std::size_t>(std::distance(first, last)) >= octets.size());
        std::memcpy(octets.data(), first, 4);
    }

    constexpr IPAddress(std::string_view address_string)
      : octets{address_string_to_octets(address_string)} {}

    constexpr IPAddress() = default;

    template<typename I>
    constexpr I write(I first,
                      I last) const {
        assert(static_cast<std::size_t>(std::distance(first, last)) >= octets.size());
        return std::copy(octets.begin(), octets.end(), first);
    }

    constexpr bool operator==(IPAddress const&) const = default;

    constexpr MacAddress toMulticastMac() const {
        auto mac = "01:00:5e:00:00:00"_mac;

        std::copy(octets.begin() + 1, octets.end(), mac.octets.begin() + 3);

        mac.octets[3] = mac.octets[3] & 0x7F_b;

        return mac;
    }
};

struct SubnetMask : IPAddress {};

struct Gateway : IPAddress {};

constexpr IPAddress operator""_ip(char const* str,
                                  std::size_t n) {
    return IPAddress{
      std::string_view{str, n}
    };
}

constexpr SubnetMask operator""_subnetMask(char const* str,
                                           std::size_t n) {
    return SubnetMask{
      std::string_view{str, n}
    };
}

constexpr Gateway operator""_gateway(char const* str,
                                     std::size_t n) {
    return Gateway{
      std::string_view{str, n}
    };
}

struct StaticIpConfig {
    IPAddress  ownIp;
    Gateway    gateway;
    SubnetMask subnetMask;
};

struct DhcpIpConfig : StaticIpConfig {
    constexpr DhcpIpConfig()
      : StaticIpConfig{"0.0.0.0"_ip,
                       "0.0.0.0"_gateway,
                       "0.0.0.0"_subnetMask} {}
};

enum class DHCPMessageType : std::uint8_t {
    Discover = 1,
    Offer    = 2,
    Request  = 3,
    Decline  = 4,
    Ack      = 5,
    NAck     = 6,
    Release  = 7,
    Inform   = 8
};
enum class DHCPOptions : std::uint8_t {
    Pad                           = 0,
    SubnetMask                    = 1,
    DNSNameServers                = 6,
    HostName                      = 12,
    DomainName                    = 15,
    MaximumDatagramReassemblySize = 22,
    NTPServers                    = 42,
    RequestedIPAddress            = 50,
    IPAddressLeaseTime            = 51,
    OptionOverload                = 52,
    MessageType                   = 53,
    ServerIdentifier              = 54,
    ParameterRequestList          = 55,
    Message                       = 56,
    MaximumDHCPMessageSize        = 57,
    RenewalTimerValue             = 58,
    RebindTimerValue              = 59,
    VendorClassIdentifier         = 60,
    ClientIntentifier             = 61,
    TFTPServerName                = 66,
    BootFileName                  = 67,
    End                           = 255,
};

namespace OptionFields {
struct Pad {
    static constexpr std::uint8_t ID{0};

    static constexpr auto parse() {}
};

struct End {
    static constexpr std::uint8_t ID{255};

    static constexpr auto parse() {}
};

struct Unknown {
    static constexpr auto parse() {}
};

struct Router {
    static constexpr std::uint8_t ID{3};
    Gateway                       address;
};

struct RequestedIPAddress {
    static constexpr std::uint8_t ID{50};
    IPAddress                     address;
};

struct DomainNameServer {
    static constexpr std::uint8_t ID{6};
    IPAddress                     address;
};

struct SubnetMask {
    static constexpr std::uint8_t ID{1};
    ::SubnetMask                  address;
};

struct ServerIdentifier {
    static constexpr std::uint8_t ID{54};
    IPAddress                     address;
};

struct IPAddressLeaseTime {
    static constexpr std::uint8_t ID{51};
    std::uint32_t                 leaseTime{};
};

struct RenewalTimerValue {
    static constexpr std::uint8_t ID{58};
    std::uint32_t                 renewalTime{};
};

struct RebindTimerValue {
    static constexpr std::uint8_t ID{59};
    std::uint32_t                 rebindTime{};
};

struct MessageType {
    static constexpr std::uint8_t ID{53};
    DHCPMessageType               messageType;
};

struct ParameterRequestList {
    static constexpr std::uint8_t ID{55};
    std::vector<DHCPOptions>      options;
};

}   // namespace OptionFields
enum class Opcode : std::uint8_t { Request = 1, Reply = 2 };
enum class HType : std::uint8_t { Ethernet = 1, IEEE802_Network = 6 };
enum class Flags : std::uint16_t { NotBroadcast = 0x0, Broadcast = 0x0080 };
static constexpr std::uint32_t MagicCookie{0x63538263U};
static constexpr std::uint8_t  HardwareAddressLength{6};

struct dhcp {
    Opcode                        opcode{Opcode::Request};
    HType                         htype{HType::Ethernet};
    std::uint8_t                  hlen{HardwareAddressLength};
    std::uint8_t                  hops{};
    std::uint32_t                 xid{};
    std::uint16_t                 secs{};
    Flags                         flags{Flags::Broadcast};
    std::uint32_t                 ciaddr{};
    std::uint32_t                 yiaddr{};
    std::uint32_t                 siaddr{};
    std::uint32_t                 giaddr{};
    std::array<std::byte, 16>     chaddr{};
    std::array<std::uint8_t, 64>  sname{};
    std::array<std::uint8_t, 128> file{53, 1, 1, 57, 2, 0x02, 0x40, 22, 2,  0x02, 0x40,

                                       55, 8, 1, 3,  6, 42,   51,   54, 58, 59,   255};
    std::uint32_t                 magic_cookie{MagicCookie};
    std::array<std::uint8_t, 4>   dhcp_options{52, 1, 1, 255};

    dhcp() {}

    dhcp(std::uint32_t    xid_,
         std::string_view hostname,
         MacAddress       mac)
      : xid{xid_} {
        std::copy(mac.octets.begin(), mac.octets.end(), chaddr.begin());
        if(hostname.empty() || hostname.size() > (sname.size() - 4)) {
            return;
        }
        dhcp_options[2] = 3;
        sname[0]        = 12;
        sname[1]        = static_cast<std::uint8_t>(hostname.size());
        std::memcpy(sname.data() + 2, hostname.data(), hostname.size());
        sname[2 + hostname.size()] = 255;
    }

    void change_to_request(IPAddress server,
                           IPAddress myIp) {
        file[2] = 3;
        auto it = file.data() + 21;
        *it     = 54;
        ++it;
        *it = 4;
        ++it;
        std::memcpy(&(*it), std::addressof(server), 4);
        it += 4;
        *it = 50;
        ++it;
        *it = 4;
        ++it;
        std::memcpy(&(*it), std::addressof(myIp), 4);
        it += 4;
        *it = 255;
    }
};

template<DHCPMessageType AcceptedMessageType,
         typename Callback>
std::optional<std::span<std::byte const>> parseOptions(std::span<std::byte const> buffer,
                                                       Callback                   cb) {
    bool finished = false;
    while(!finished && !buffer.empty()) {
        std::uint8_t const option = static_cast<std::uint8_t>(buffer[0]);
        buffer                    = buffer.subspan(1);

        switch(option) {
        case OptionFields::End::ID:
            {
                finished = true;
            }
            break;
        case OptionFields::MessageType::ID:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }
                std::uint8_t const size             = static_cast<std::uint8_t>(buffer[0]);
                using T                             = OptionFields::MessageType;
                static constexpr std::uint8_t TSize = sizeof(T);
                buffer                              = buffer.subspan(1);

                if(size != TSize && buffer.size() < TSize) {
                    return std::nullopt;
                }
                T mt;
                std::memcpy(std::addressof(mt), buffer.data(), TSize);
                buffer = buffer.subspan(size);
                cb(mt);
                if(mt.messageType != AcceptedMessageType) {
                    return std::nullopt;
                }
            }
            break;
        case OptionFields::SubnetMask::ID:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }
                std::uint8_t const size             = static_cast<std::uint8_t>(buffer[0]);
                using T                             = OptionFields::SubnetMask;
                static constexpr std::uint8_t TSize = sizeof(T);
                buffer                              = buffer.subspan(1);

                if(size != TSize && buffer.size() < TSize) {
                    return std::nullopt;
                }
                T mt;
                std::memcpy(std::addressof(mt), buffer.data(), TSize);
                buffer = buffer.subspan(size);
                cb(mt);
            }
            break;
        case OptionFields::DomainNameServer::ID:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }
                std::uint8_t const size             = static_cast<std::uint8_t>(buffer[0]);
                using T                             = OptionFields::DomainNameServer;
                static constexpr std::uint8_t TSize = sizeof(T);
                buffer                              = buffer.subspan(1);

                if(size < TSize && buffer.size() < TSize) {
                    return std::nullopt;
                }
                T mt;
                std::memcpy(std::addressof(mt), buffer.data(), TSize);
                buffer = buffer.subspan(size);
                cb(mt);
            }
            break;
        case OptionFields::Router::ID:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }
                std::uint8_t const size             = static_cast<std::uint8_t>(buffer[0]);
                using T                             = OptionFields::Router;
                static constexpr std::uint8_t TSize = sizeof(T);
                buffer                              = buffer.subspan(1);

                if(size < TSize && buffer.size() < TSize) {
                    return std::nullopt;
                }
                T mt;
                std::memcpy(std::addressof(mt), buffer.data(), TSize);
                buffer = buffer.subspan(size);
                cb(mt);
            }
            break;
        case OptionFields::ServerIdentifier::ID:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }
                std::uint8_t const size             = static_cast<std::uint8_t>(buffer[0]);
                using T                             = OptionFields::ServerIdentifier;
                static constexpr std::uint8_t TSize = sizeof(T);
                buffer                              = buffer.subspan(1);

                if(size < TSize && buffer.size() < TSize) {
                    return std::nullopt;
                }
                T mt;
                std::memcpy(std::addressof(mt), buffer.data(), TSize);
                buffer = buffer.subspan(size);
                cb(mt);
            }
            break;
        case OptionFields::Pad::ID:
            {
            }
            break;
        case OptionFields::IPAddressLeaseTime::ID:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }

                std::uint8_t const size             = static_cast<std::uint8_t>(buffer[0]);
                using T                             = OptionFields::IPAddressLeaseTime;
                static constexpr std::uint8_t TSize = sizeof(T);
                buffer                              = buffer.subspan(1);
                UC_LOG_D("buffer: {} {} {::#x}", option, size, buffer.subspan(0, size));

                if(size < TSize && buffer.size() < TSize) {
                    return std::nullopt;
                }
                T mt;
                std::memcpy(std::addressof(mt), buffer.data(), TSize);
                buffer = buffer.subspan(size);
                cb(mt);
            }
            break;
        case OptionFields::RenewalTimerValue::ID:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }
                std::uint8_t const size             = static_cast<std::uint8_t>(buffer[0]);
                using T                             = OptionFields::RenewalTimerValue;
                static constexpr std::uint8_t TSize = sizeof(T);
                buffer                              = buffer.subspan(1);

                if(size < TSize && buffer.size() < TSize) {
                    return std::nullopt;
                }
                T mt;
                std::memcpy(std::addressof(mt), buffer.data(), TSize);
                buffer = buffer.subspan(size);
                cb(mt);
            }
            break;
        case OptionFields::RebindTimerValue::ID:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }
                std::uint8_t const size             = static_cast<std::uint8_t>(buffer[0]);
                using T                             = OptionFields::RebindTimerValue;
                static constexpr std::uint8_t TSize = sizeof(T);
                buffer                              = buffer.subspan(1);

                if(size < TSize && buffer.size() < TSize) {
                    return std::nullopt;
                }
                T mt;
                std::memcpy(std::addressof(mt), buffer.data(), TSize);
                buffer = buffer.subspan(size);
                cb(mt);
            }
            break;

        default:
            {
                if(buffer.empty()) {
                    return std::nullopt;
                }
                std::uint8_t const size = static_cast<std::uint8_t>(buffer[0]);
                buffer                  = buffer.subspan(1);
                UC_LOG_D("Unknown DHCP option {} {} {}", option, size, buffer.subspan(0, size));
                buffer = buffer.subspan(size);
            }
            break;
        }
    }
    return buffer;
}

template<DHCPMessageType AcceptedMessageType,
         typename Callback>
std::optional<std::span<std::byte const>> dhcp_parse(std::span<std::byte const> buffer,
                                                     std::uint32_t              xid,
                                                     Callback                   cb) {
    if(buffer.empty()) {
        UC_LOG_D("empty {}", __LINE__);
        return std::nullopt;
    }
    if(static_cast<Opcode>(buffer[0]) != Opcode::Reply) {
        UC_LOG_D("Opcode wrong {} {}", buffer[0], static_cast<std::byte>(Opcode::Reply));
        return std::nullopt;
    }

    buffer = buffer.subspan(2);
    if(buffer.empty()) {
        UC_LOG_D("empty {}", __LINE__);
        return std::nullopt;
    }
    if(static_cast<std::uint8_t>(buffer[0]) != HardwareAddressLength) {
        UC_LOG_D("hlen wrong {} {}", buffer[0], HardwareAddressLength);
        return std::nullopt;
    }

    buffer = buffer.subspan(2);
    if(4 > buffer.size()) {
        UC_LOG_D("empty {}", __LINE__);
        return std::nullopt;
    }
    std::uint32_t packageXid;
    std::memcpy(std::addressof(packageXid), buffer.data(), sizeof(packageXid));
    if(packageXid != xid) {
        UC_LOG_D("xid wrong {} {}", packageXid, xid);
        return std::nullopt;
    }

    buffer = buffer.subspan(12);
    if(4 > buffer.size()) {
        UC_LOG_D("empty {}", __LINE__);
        return std::nullopt;
    }
    IPAddress const address = [&]() {
        IPAddress a;
        std::memcpy(std::addressof(a), buffer.data(), sizeof(a));
        return a;
    }();

    cb(OptionFields::RequestedIPAddress{address});

    buffer = buffer.subspan(12 + 16);

    if(64 > buffer.size()) {
        UC_LOG_D("empty {}", __LINE__);
        return std::nullopt;
    }

    //    std::span<std::byte const> const sname{buffer.subspan(64)};
    buffer = buffer.subspan(64);

    if(128 > buffer.size()) {
        UC_LOG_D("empty {}", __LINE__);
        return std::nullopt;
    }

    //  std::span<std::byte const> const file{buffer.subspan(128)};
    buffer = buffer.subspan(128);

    if(4 > buffer.size()) {
        UC_LOG_D("empty {}", __LINE__);
        return std::nullopt;
    }
    std::uint32_t cookie;
    std::memcpy(std::addressof(cookie), buffer.data(), sizeof(cookie));
    if(cookie != MagicCookie) {
        UC_LOG_D("cookie wrong {} {}", cookie, MagicCookie);
        return std::nullopt;
    }

    buffer = buffer.subspan(4);

    return parseOptions<AcceptedMessageType>(buffer, cb);
}

template<typename Chip>
struct DHCPHandler {
    static constexpr std::uint16_t          ClientPort = 68;
    static constexpr std::uint16_t          ServerPort = 67;
    static constexpr IPAddress              ServerAddress{"255.255.255.255"_ip};
    Chip&                                   chip;
    bool                                    ready{false};
    std::optional<typename Chip::UDPSocket> socket;

    DHCPHandler(Chip& c) : chip{c} {}

    enum State { init, wait, sendRequest, waitForOffer, waitForAck, fin, checkLease };

    State s{State::init};

    dhcp dhcp_data{};

    Kvasir::StaticVector<std::byte, 1024> recvBuffer;

    using Clock = typename Chip::Clock;
    typename Clock::time_point next{};
    typename Clock::time_point timeout{};
    typename Clock::time_point nextLeaseRenew{};

    std::optional<IPAddress> dns;
    std::optional<IPAddress> ip;
    std::optional<IPAddress> oldIp;

    void handle() {
        switch(s) {
        case State::init:
            {
                next = Clock::now() + 100ms;
                s    = State::wait;
            }
            break;
        case State::wait:
            {
                if(Clock::now() > next) {
                    next = Clock::now() + 100ms;
                    if(!socket) {
                        socket = chip.udpSocket(ClientPort);
                    }
                    if(socket) {
                        UC_LOG_D("dhcp socket created");
                        timeout = Clock::now() + 5s;
                        s       = State::sendRequest;
                    }
                }
            }
            break;
        case State::sendRequest:
            {
                if(Clock::now() > next) {
                    next = Clock::now() + 100ms;
                    if(socket->send_ready()) {
                        auto const xid = next.time_since_epoch().count();
                        dhcp_data      = dhcp{static_cast<std::uint32_t>(xid),
                                         Chip::Config::Hostname(),
                                         Chip::Config::Mac()};
                        if(socket->send_to(ServerAddress,
                                           ServerPort,
                                           std::as_bytes(std::span{std::addressof(dhcp_data), 1})))
                        {
                            UC_LOG_D("dhcp send Request");
                            timeout = Clock::now() + 5s;
                            s       = State::waitForOffer;
                        }
                    }
                }
            }
            break;
        case State::waitForOffer:
            {
                if(!(Clock::now() > timeout)) {
                    recvBuffer.resize(recvBuffer.max_size());
                    auto const packet = socket->recv_from(std::span{recvBuffer});
                    if(packet) {
                        UC_LOG_D("dhcp got offer");
                        UC_LOG_D("recv IP: {}, Port: {}, Data: {::#x}",
                                 std::get<0>(*packet).octets,
                                 std::get<1>(*packet),
                                 std::get<2>(*packet));
                        std::optional<IPAddress> ownIp;
                        std::optional<IPAddress> server;
                        //TODO make checks and timeout and ips...
                        if(dhcp_parse<DHCPMessageType::Offer>(
                             std::get<2>(*packet),
                             dhcp_data.xid,
                             [&]<typename T>(T const& x) {
                                 if constexpr(std::is_same_v<T, OptionFields::ServerIdentifier>) {
                                     server = x.address;
                                 }
                                 if constexpr(std::is_same_v<T, OptionFields::RequestedIPAddress>) {
                                     ownIp = x.address;
                                 }
                             }))
                        {
                            if(oldIp.has_value() && ownIp.has_value()
                               && (ownIp.value() != oldIp.value()))
                            {
                                //reset einfügen
                                UC_LOG_D("Ip changed! Restarting");
                                apply(Kvasir::SystemControl::SystemReset());
                            } else {
                                if(server && ownIp) {
                                    dhcp_data.change_to_request(*server, *ownIp);
                                    socket->send_to(
                                      ServerAddress,
                                      ServerPort,
                                      std::as_bytes(std::span{std::addressof(dhcp_data), 1}));
                                    timeout = Clock::now() + 5s;
                                    s       = State::waitForAck;
                                } else {
                                    UC_LOG_D("foo");
                                }
                            }
                        }
                    }
                } else {
                    UC_LOG_D("Timeout wait for offer");
                    s = State::sendRequest;
                }
            }
            break;

        case State::waitForAck:
            {
                if(!(Clock::now() > timeout)) {
                    recvBuffer.resize(recvBuffer.max_size());
                    auto const packet = socket->recv_from(std::span{recvBuffer});
                    if(packet) {
                        UC_LOG_D("dhcp got ack");
                        UC_LOG_D("recv IP: {}, Port: {}, Data: {::#x}",
                                 std::get<0>(*packet).octets,
                                 std::get<1>(*packet),
                                 std::get<2>(*packet));
                        //TODO make checks and timeout and ips...

                        std::optional<Gateway>       gateway;
                        std::optional<SubnetMask>    subnetMask;
                        std::optional<IPAddress>     ownIp;
                        std::optional<std::uint32_t> ownLeaseTime;
                        std::optional<std::uint32_t> ownRenewalTime;
                        std::optional<std::uint32_t> ownRebindTime;
                        if(dhcp_parse<DHCPMessageType::Ack>(
                             std::get<2>(*packet),
                             dhcp_data.xid,
                             [&]<typename T>(T const& x) {
                                 if constexpr(std::is_same_v<T, OptionFields::Router>) {
                                     gateway = x.address;
                                 }
                                 if constexpr(std::is_same_v<T, OptionFields::SubnetMask>) {
                                     subnetMask = x.address;
                                 }
                                 if constexpr(std::is_same_v<T, OptionFields::RequestedIPAddress>) {
                                     ownIp = x.address;
                                 }
                                 if constexpr(std::is_same_v<T, OptionFields::IPAddressLeaseTime>) {
                                     ownLeaseTime = std::byteswap(x.leaseTime);
                                     if(ownLeaseTime) {
                                         nextLeaseRenew
                                           = Clock::now()
                                           + std::chrono::seconds(ownLeaseTime.value()) / 2;
                                     }
                                     UC_LOG_D("leaseTime: {}", ownLeaseTime);
                                 }
                                 if constexpr(std::is_same_v<T, OptionFields::RebindTimerValue>) {
                                     ownRebindTime = std::byteswap(x.rebindTime);
                                     UC_LOG_D("rebindTime: {}", ownRebindTime);
                                 }
                                 if constexpr(std::is_same_v<T, OptionFields::RenewalTimerValue>) {
                                     ownRenewalTime = std::byteswap(x.renewalTime);
                                     UC_LOG_D("renewTime: {}", ownRenewalTime);
                                 }
                                 if constexpr(std::is_same_v<T, OptionFields::DomainNameServer>) {
                                     UC_LOG_D("DNS: {}", x.address.octets);
                                     dns = x.address;
                                 }
                             }))
                        {
                            if(gateway && subnetMask && ownIp) {
                                ip    = ownIp;
                                oldIp = ip;
                                s     = State::fin;
                                chip.append_ip_config(*ownIp, *gateway, *subnetMask);
                                ready = true;
                            } else {
                                UC_LOG_D("dhcp failed");
                            }
                        }
                    }
                } else {
                    UC_LOG_D("Timeout wait for offer");
                    s = State::sendRequest;
                }
            }
            break;
        case State::fin:
            {
                socket.reset();
                s = State::checkLease;
            }
            break;
        case State::checkLease:
            {
                if(Clock::now() > nextLeaseRenew) {
                    UC_LOG_D("Lease time reached, sending new request");
                    s = State::init;
                }
            }
            break;
        }
    }
};

template<typename Chip>
struct EmptyDHCPHandler {
    EmptyDHCPHandler(Chip&) {}

    void handle() {}
};

template<typename Clock_, typename SPI, typename Cs, typename Rst, typename Config_>
struct W5500 {
    using Config = Config_;
    using Clock  = Clock_;

    struct EphemeralPortRange {
        static constexpr std::uint16_t min = 32768;
        static constexpr std::uint16_t max = 60999;
    };

    enum class CommonRegister : std::uint8_t {
        Mode                      = 0x00,
        GatewayAddress            = 0x01,
        SubnetMaskAddress         = 0x05,
        SourceHardwareAddress     = 0x09,
        SourceIpAddress           = 0x0F,
        InterruptLowLevelTimer    = 0x13,
        Interrupt                 = 0x15,
        InterruptMask             = 0x16,
        SocketMask                = 0x17,
        SocketInterrupt           = 0x18,
        RetryTime                 = 0x19,
        RetryCount                = 0x1B,
        PPP_LCP_RequestTimer      = 0x1C,
        PPP_LCP_MagicNumber       = 0x1D,
        PPP_DestinationMACAddress = 0x1E,
        PPP_SessionIdentification = 0x24,
        PPP_MaximumSegmentSize    = 0x26,
        UnreachableIpAddress      = 0x28,
        UnreachablePort           = 0x2C,
        PHY_Configuration         = 0x2E,
        ChipVersion               = 0x39
    };
    enum class SocketRegister : std::uint8_t {
        Mode                       = 0x00,
        Command                    = 0x01,
        Interrupt                  = 0x02,
        Status                     = 0x03,
        SourcePort                 = 0x04,
        DestinationHardwareAddress = 0x06,
        DestinationIPAddress       = 0x0C,
        DestinationPort            = 0x10,
        MaximumSegmentSize         = 0x12,
        IP_TOS                     = 0x15,
        IP_TTL                     = 0x16,
        ReceiveBufferSize          = 0x1E,
        TransmitBufferSize         = 0x1F,
        TX_FreeSize                = 0x20,
        TX_ReadPointer             = 0x22,
        TX_WritePointer            = 0x24,
        RX_ReceivedSize            = 0x26,
        RX_ReadPointer             = 0x28,
        RX_WritePointer            = 0x2A,
        InterruptMask              = 0x2C,
        FragmentOffsetInIpHeader   = 0x2D,
        KeepAliveTimer             = 0x30
    };

    static constexpr bool hasDHCP
      = std::is_same_v<DhcpIpConfig, std::remove_cvref_t<decltype(Config::IpConfig)>>;

    static constexpr std::size_t MaxNSockets{8};

    static constexpr std::size_t NSockets{6};

    struct SocketNumber {
        std::uint8_t n;

        SocketNumber(std::uint8_t n_) : n{n_} { assert(NSockets > n); }

        bool isInvalid() const { return n >= NSockets; }

        void setInvalid() { n = NSockets; }
    };

    struct SocketState {
        std::array<std::byte, 64>             command_buffer;
        std::array<std::byte, 64>             recv_command_buffer;
        Kvasir::StaticVector<std::byte, 1024> recv_buffer;
        Kvasir::StaticVector<std::byte, 1024> tmp_recv_buffer;
        std::byte*                            last_read_pos{};
        std::byte*                            last_state_pos{};
        std::byte*                            last_read_pos_recv{};
        std::byte                             expectedSockState{};
        std::optional<std::uint16_t>          tx_write{};
        std::optional<std::uint16_t>          rx_read{};

        std::optional<std::uint16_t> last_rx_write{};
        bool                         connected{false};
        bool                         send_in_progress{false};
        bool                         free{true};
        bool                         ready{true};
        bool                         error{false};
        bool                         rdyAccept{false};

        void reset_raw() {
            tx_write.reset();
            rx_read.reset();
            last_rx_write.reset();
            recv_buffer.clear();
            tmp_recv_buffer.clear();
            expectedSockState = 0_b;
            rdyAccept         = false;
            connected         = false;
            send_in_progress  = false;
            error             = false;
            ready             = true;
        }

        void reset(W5500&       w5500,
                   SocketNumber sn) {
            reset_raw();
            error = true;
            ready = false;

            w5500.clear_commands_of(sn);

            auto pos = command_buffer.begin();
            pos      = Header{SocketRegister::Command,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, command_buffer.end());

            *pos++ = 0x1_b;

            w5500.command_q.push(typename Commands::Write{command_buffer.begin(), pos});
            add_read_state(pos, command_buffer.end(), w5500, sn);

            w5500.command_q.push(
              typename Commands::Call_Socket{&SocketState::check_sock_closed, sn});
        }

        template<typename I>
        static I add_read_state(I            first,
                                I            last,
                                W5500&       w5500,
                                SocketNumber sn) {
            assert(std::distance(first, last) >= 3);

            auto& socket = w5500.sockets[sn.n];

            auto pos = Header{SocketRegister::Status,
                              Header::ReadWrite::Read,
                              Header::BlockSelect::SocketRegister(sn.n),
                              Header::Size::_1}
                         .write(first, last);

            w5500.command_q.push(typename Commands::Write{first, pos});

            w5500.command_q.push(typename Commands::Read{first, 1});

            socket.last_state_pos = first;

            return first + 3;
        }

        template<typename I>
        static I add_read_ir(I            first,
                             I            last,
                             W5500&       w5500,
                             SocketNumber sn) {
            assert(std::distance(first, last) >= 3);

            auto& socket = w5500.sockets[sn.n];

            auto pos = Header{SocketRegister::Interrupt,
                              Header::ReadWrite::Read,
                              Header::BlockSelect::SocketRegister(sn.n),
                              Header::Size::_1}
                         .write(first, last);

            w5500.command_q.push(typename Commands::Write{first, pos});

            w5500.command_q.push(typename Commands::Read{first, 1});

            socket.last_read_pos = first;

            return first + 3;
        }

        template<typename I>
        static I add_clear_ir(I            first,
                              I            last,
                              W5500&       w5500,
                              SocketNumber sn) {
            auto pos = Header{SocketRegister::Interrupt,
                              Header::ReadWrite::Write,
                              Header::BlockSelect::SocketRegister(sn.n),
                              Header::Size::_1}
                         .write(first, last);

            *pos++ = 0x1f_b;
            w5500.command_q.push(typename Commands::Write{first, pos});
            return pos;
        }

        template<typename I>
        static I add_read_tx_write(I            first,
                                   I            last,
                                   W5500&       w5500,
                                   SocketNumber sn) {
            auto& socket = w5500.sockets[sn.n];

            auto pos = Header{SocketRegister::TX_WritePointer,
                              Header::ReadWrite::Read,
                              Header::BlockSelect::SocketRegister(sn.n),
                              Header::Size::_2}
                         .write(first, last);
            w5500.command_q.push(typename Commands::Write{first, pos});
            w5500.command_q.push(typename Commands::Read{first, 2});
            socket.last_read_pos = first;

            w5500.command_q.push(typename Commands::Call_Socket{&SocketState::read_tx_write, sn});
            return pos;
        }

        template<typename I>
        static I add_read_rx_write(I            first,
                                   I            last,
                                   W5500&       w5500,
                                   SocketNumber sn) {
            auto& socket = w5500.sockets[sn.n];

            auto pos = Header{SocketRegister::RX_WritePointer,
                              Header::ReadWrite::Read,
                              Header::BlockSelect::SocketRegister(sn.n),
                              Header::Size::_2}
                         .write(first, last);
            w5500.command_q.push(typename Commands::Write{first, pos});
            w5500.command_q.push(typename Commands::Read{first, 2});
            socket.last_read_pos_recv = first;

            w5500.command_q.push(typename Commands::Call_Socket{&SocketState::read_rx_write, sn});
            return pos;
        }

        template<typename I>
        static I add_read_rx_read(I            first,
                                  I            last,
                                  W5500&       w5500,
                                  SocketNumber sn) {
            auto& socket = w5500.sockets[sn.n];

            auto pos = Header{SocketRegister::RX_ReadPointer,
                              Header::ReadWrite::Read,
                              Header::BlockSelect::SocketRegister(sn.n),
                              Header::Size::_2}
                         .write(first, last);
            w5500.command_q.push(typename Commands::Write{first, pos});
            w5500.command_q.push(typename Commands::Read{first, 2});
            socket.last_read_pos_recv = first;

            w5500.command_q.push(typename Commands::Call_Socket{&SocketState::read_rx_read, sn});
            return pos;
        }

        void read_tx_write(W5500&,
                           SocketNumber) {
            if(!ready || free) {
                return;
            }
            auto old_tx_write = tx_write;

            tx_write
              = static_cast<std::uint16_t>((static_cast<std::uint32_t>(last_read_pos[0]) << 8)
                                           | static_cast<std::uint32_t>(last_read_pos[1]));

            if(old_tx_write) {
                UC_LOG_D("read {} assume {}", tx_write, old_tx_write);
            }
        }

        void read_rx_write(W5500&       w5500,
                           SocketNumber sn) {
            if(!ready || free) {
                return;
            }
            auto const rx_write
              = static_cast<std::uint16_t>((static_cast<std::uint32_t>(last_read_pos_recv[0]) << 8)
                                           | static_cast<std::uint32_t>(last_read_pos_recv[1]));

            if(last_rx_write && *last_rx_write == rx_write) {
                last_rx_write.reset();
                std::size_t const bytes_to_read = [&]() {
                    if(rx_write >= *rx_read) {
                        return static_cast<std::size_t>(rx_write - *rx_read);
                    } else {
                        return static_cast<std::size_t>(
                          (std::numeric_limits<std::uint16_t>::max() - *rx_read) + rx_write + 1);
                    }
                }();

                if(bytes_to_read > 1024) {
                    UC_LOG_C("to many bytes on {} .... {} {} {} {}",
                             sn.n,
                             *last_rx_write,
                             rx_write,
                             rx_read,
                             bytes_to_read);
                }

                if(bytes_to_read != 0 && tmp_recv_buffer.empty()) {
                    UC_LOG_D("read rdy {} {} {}", rx_read, rx_write, bytes_to_read);

                    tmp_recv_buffer.resize(std::min(bytes_to_read, tmp_recv_buffer.max_size()));

                    auto pos = Header{*rx_read,
                                      Header::ReadWrite::Read,
                                      Header::BlockSelect::SocketRxBuffer(sn.n),
                                      Header::Size::variable}
                                 .write(recv_command_buffer.begin(), recv_command_buffer.end());

                    w5500.selected([&]() {
                        w5500.command_q.push(
                          typename Commands::Write{recv_command_buffer.begin(), pos});
                        w5500.command_q.push(
                          typename Commands::Read{tmp_recv_buffer.data(), tmp_recv_buffer.size()});
                    });

                    auto cmdpos = pos;
                    pos         = Header{SocketRegister::RX_ReadPointer,
                                 Header::ReadWrite::Write,
                                 Header::BlockSelect::SocketRegister(sn.n),
                                 Header::Size::_2}
                            .write(pos, recv_command_buffer.end());
                    *rx_read += tmp_recv_buffer.size();
                    *pos++ = std::byte(*tx_write >> 8);
                    *pos++ = std::byte(*tx_write & 0xff);

                    pos = Header{SocketRegister::Command,
                                 Header::ReadWrite::Write,
                                 Header::BlockSelect::SocketRegister(sn.n),
                                 Header::Size::_1}
                            .write(pos, recv_command_buffer.end());
                    *pos++ = 0x40_b;

                    w5500.selected(
                      [&]() { w5500.command_q.push(typename Commands::Write{cmdpos, pos}); });

                    w5500.command_q.push(
                      typename Commands::Call_Socket{&SocketState::recv_ready, sn});
                } else {
                    w5500.append_yield();
                    auto pos = recv_command_buffer.begin();
                    w5500.selected([&]() {
                        pos = add_read_state(pos, recv_command_buffer.end(), w5500, sn);
                        w5500.command_q.push(
                          typename Commands::Call_Socket{&SocketState::print_sock_state, sn});

                        pos = add_read_rx_write(pos, recv_command_buffer.end(), w5500, sn);
                    });
                }
            } else {
                last_rx_write = rx_write;
                w5500.selected([&]() {
                    add_read_rx_write(recv_command_buffer.begin(),
                                      recv_command_buffer.end(),
                                      w5500,
                                      sn);
                });
            }
        }

        void recv_ready(W5500&       w5500,
                        SocketNumber sn) {
            if(!ready || free) {
                return;
            }

            w5500.selected([&]() {
                add_read_rx_write(recv_command_buffer.begin(),
                                  recv_command_buffer.end(),
                                  w5500,
                                  sn);
            });

            auto const old_size = recv_buffer.size();
            if(old_size + tmp_recv_buffer.size() > recv_buffer.max_size()) {
                UC_LOG_C("socket {} recv full", sn.n);
                return;
            }
            recv_buffer.resize(old_size + tmp_recv_buffer.size());
            std::copy(tmp_recv_buffer.begin(),
                      tmp_recv_buffer.end(),
                      recv_buffer.begin() + old_size);
            tmp_recv_buffer.clear();
        }

        void read_rx_read(W5500&,
                          SocketNumber) {
            if(!ready || free) {
                return;
            }
            auto old_rx_read = rx_read;

            rx_read
              = static_cast<std::uint16_t>((static_cast<std::uint32_t>(last_read_pos_recv[0]) << 8)
                                           | static_cast<std::uint32_t>(last_read_pos_recv[1]));

            if(old_rx_read) {
                UC_LOG_D("read {} assume {}", rx_read, old_rx_read);
            }
        }

        void check_send_ready(W5500&       w5500,
                              SocketNumber sn) {
            if(!ready || free) {
                return;
            }
            //   UC_LOG_D("ir {}", *last_read_pos);
            if(((*last_read_pos) & 0x10_b) != 0_b) {
                w5500.append_yield();
                w5500.selected(
                  [&]() { add_read_ir(command_buffer.begin(), command_buffer.end(), w5500, sn); });

                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_send_ready, sn});
            } else {
                //    UC_LOG_D("send ok");
                w5500.selected([&]() {
                    // add_read_tx_write(command_buffer.begin(),command_buffer.end(),w5500,sn);
                    add_clear_ir(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                send_in_progress = false;
            }
        }

        void check_sock_init(W5500&       w5500,
                             SocketNumber sn) {
            if(!ready || free) {
                return;
            }
            if(*last_state_pos != 0x13_b) {
                UC_LOG_D("not init {}", *last_state_pos);

                w5500.append_yield();
                w5500.selected([&]() {
                    add_read_state(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_init, sn});
            } else {
                UC_LOG_D("sock init");
                auto pos = Header{SocketRegister::Command,
                                  Header::ReadWrite::Write,
                                  Header::BlockSelect::SocketRegister(sn.n),
                                  Header::Size::_1}
                             .write(command_buffer.begin(), command_buffer.end());

                *pos++ = 0x4_b;

                w5500.selected([&]() {
                    w5500.command_q.push(typename Commands::Write{command_buffer.begin(), pos});
                    pos = add_read_state(pos, command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_established, sn});
            }
        }

        void check_sock_init_server(W5500&       w5500,
                                    SocketNumber sn) {
            if(!ready || free) {
                return;
            }
            if(*last_state_pos != 0x13_b) {
                UC_LOG_D("not init {}", *last_state_pos);

                w5500.append_yield();
                w5500.selected([&]() {
                    add_read_state(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_init_server, sn});
            } else {
                UC_LOG_D("sock init");
                auto pos = Header{SocketRegister::Command,
                                  Header::ReadWrite::Write,
                                  Header::BlockSelect::SocketRegister(sn.n),
                                  Header::Size::_1}
                             .write(command_buffer.begin(), command_buffer.end());

                *pos++ = 0x2_b;

                w5500.selected([&]() {
                    w5500.command_q.push(typename Commands::Write{command_buffer.begin(), pos});
                    pos = add_read_state(pos, command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_listen, sn});
            }
        }

        void print_sock_state(W5500&       w5500,
                              SocketNumber sn) {
            if(expectedSockState != *last_state_pos) {
                UC_LOG_T("sockstate {} != {} on {}", *last_state_pos, expectedSockState, sn.n);
                reset(w5500, sn);
            }
        }

        void check_sock_udp(W5500&       w5500,
                            SocketNumber sn) {
            if(!ready || free) {
                return;
            }
            if(*last_state_pos != 0x22_b) {
                UC_LOG_D("not init {}", *last_state_pos);

                w5500.append_yield();
                w5500.selected([&]() {
                    add_read_state(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_udp, sn});
            } else {
                UC_LOG_D("sock init {}", sn.n);
                connected         = true;
                expectedSockState = 0x22_b;
                w5500.selected([&]() {
                    auto pos = add_read_rx_write(recv_command_buffer.begin(),
                                                 recv_command_buffer.end(),
                                                 w5500,
                                                 sn);
                    add_read_tx_write(pos, recv_command_buffer.end(), w5500, sn);
                });
            }
        }

        void check_sock_established(W5500&       w5500,
                                    SocketNumber sn) {
            if(!ready || free) {
                return;
            }
            if(*last_state_pos != 0x17_b) {
                UC_LOG_D("not established {}", *last_read_pos);
                w5500.append_yield();
                w5500.selected([&]() {
                    add_read_state(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_established, sn});
            } else {
                UC_LOG_D("established");
                connected         = true;
                expectedSockState = 0x17_b;
                w5500.selected([&]() {
                    auto pos = add_read_rx_write(recv_command_buffer.begin(),
                                                 recv_command_buffer.end(),
                                                 w5500,
                                                 sn);
                    add_read_tx_write(pos, recv_command_buffer.end(), w5500, sn);
                });
            }
        }

        void check_sock_listen(W5500&       w5500,
                               SocketNumber sn) {
            if(!ready || free) {
                return;
            }
            if(*last_state_pos != 0x14_b) {
                UC_LOG_D("not listen {}", *last_read_pos);
                w5500.append_yield();
                w5500.selected([&]() {
                    add_read_state(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_listen, sn});
            } else {
                UC_LOG_D("listen");

                w5500.append_yield();
                w5500.selected([&]() {
                    add_read_state(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_accept, sn});

                connected = true;
            }
        }

        void check_sock_accept(W5500&       w5500,
                               SocketNumber sn) {
            if(!ready || free) {
                return;
            }
            if(*last_state_pos == 0x00_b) {
                UC_LOG_D("failed");
                connected = false;
            } else if(*last_state_pos == 0x17_b) {
                UC_LOG_D("accepted!");
                rdyAccept = true;
            } else {
                w5500.append_yield();
                w5500.selected([&]() {
                    add_read_state(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_accept, sn});
            }
        }

        void check_sock_closed(W5500&       w5500,
                               SocketNumber sn) {
            if(*last_state_pos != 0x00_b) {
                //       UC_LOG_D("not closed {}  {}", *last_read_pos, sn.n);
                w5500.append_yield();
                w5500.selected([&]() {
                    add_read_state(command_buffer.begin(), command_buffer.end(), w5500, sn);
                });
                w5500.command_q.push(
                  typename Commands::Call_Socket{&SocketState::check_sock_closed, sn});
            } else {
                UC_LOG_D("closed {}", sn.n);
                error = false;
                ready = true;
            }
        }
    };

    std::array<SocketState, NSockets> sockets{};

    struct Commands {
        struct Reset {};

        struct Yield {};

        struct Select {};

        struct Deselect {};

        struct Write {
            std::byte const* buffer;
            std::uint16_t    size;

            template<Kvasir::contiguous_byte_iterator I>
            Write(I first,
                  I last)
              : Write{first,
                      std::distance(first,
                                    last)} {}

            template<Kvasir::contiguous_byte_iterator I,
                     typename S>
            Write(I first,
                  S size_)
              : buffer{reinterpret_cast<std::byte const*>(first)}
              , size{static_cast<std::uint16_t>(size_)} {}
        };

        struct Read {
            std::byte*    buffer;
            std::uint16_t size;

            template<Kvasir::contiguous_byte_iterator I>
            Read(I first,
                 I last)
              : Read(first,
                     std::distance(first,
                                   last)) {}

            template<Kvasir::contiguous_byte_iterator I,
                     typename S>
            Read(I first,
                 S size_)
              : buffer{reinterpret_cast<std::byte*>(first)}
              , size{static_cast<std::uint16_t>(size_)} {}
        };

        struct Call_Socket {
            void (SocketState::*function)(W5500&,
                                          SocketNumber);
            SocketNumber n;
        };

        struct Call_Self {
            void (W5500::*function)();
        };

        using Command
          = std::variant<Yield, Reset, Select, Deselect, Write, Read, Call_Socket, Call_Self>;
    };

    Kvasir::Atomic::Queue<typename Commands::Command, 128> command_q;

    struct UDPSocket_ {
        W5500*       w5500{};
        SocketNumber sn{};

        SocketState& getSocket() { return w5500->sockets[sn.n]; }

        SocketState const& getSocket() const { return w5500->sockets[sn.n]; }

        bool send_ready() const { return !getSocket().send_in_progress; }

        auto& getRecvBuffer() { return getSocket().recv_buffer; }

        ~UDPSocket_() {
            if(!sn.isInvalid()) {
                w5500->sockets[sn.n].reset(*w5500, sn);
                assert(w5500->sockets[sn.n].free == false);
                w5500->sockets[sn.n].free = true;
            }
        }

        UDPSocket_& operator=(UDPSocket_ const&) = delete;
        UDPSocket_(UDPSocket_ const&)            = delete;

        UDPSocket_(UDPSocket_&& other)
          : w5500{other.w5500}
          , sn{other.sn} {
            other.sn.setInvalid();
        }

        UDPSocket_& operator=(UDPSocket_&& other) {
            if(std::addressof(other) != this) {
                assert(other.w5500 == w5500);
                sn = other.sn;
                other.sn.setInvalid();
            }
            return *this;
        }

        UDPSocket_(W5500&                   w5500_,
                   SocketNumber             sn_,
                   std::uint16_t            port,
                   std::optional<IPAddress> multicast)
          : w5500{std::addressof(w5500_)}
          , sn{sn_} {
            auto& socket = getSocket();
            auto& buffer = socket.command_buffer;
            socket.free  = false;

            auto cmdpos = buffer.begin();
            auto pos    = cmdpos;

            pos = Header{SocketRegister::Mode,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());
            std::byte v = 2_b;

            if(multicast) {
                v |= 0x80_b;   //enable multicast
            }

            *pos++ = v;

            pos = Header{SocketRegister::SourcePort,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());

            *pos++ = std::byte(port >> 8);
            *pos++ = std::byte(port & 0xff);
            /* Troubles with this blob while connecting under Windows
            if(multicast) {
                pos
                  = Header{SocketRegister::DestinationIPAddress, Header::ReadWrite::Write, Header::BlockSelect::SocketRegister(sn.n), Header::Size::_4}
                      .write(pos, buffer.end());
                pos = multicast->write(pos, buffer.end());

                pos
                  = Header{SocketRegister::DestinationPort, Header::ReadWrite::Write, Header::BlockSelect::SocketRegister(sn.n), Header::Size::_2}
                      .write(pos, buffer.end());

                *pos++ = std::byte(port >> 8);
                *pos++ = std::byte(port & 0xff);
            }
*/
            pos = Header{SocketRegister::Command,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());

            *pos++ = 0x1_b;

            assert(std::distance(pos, buffer.end()) >= 0);

            w5500->selected([&]() {
                w5500->command_q.push(typename Commands::Write{cmdpos, pos});
                pos = SocketState::add_read_tx_write(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_read_rx_read(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_clear_ir(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_read_state(pos, buffer.end(), *w5500, sn);
            });

            w5500->command_q.push(typename Commands::Call_Socket{&SocketState::check_sock_udp, sn});
        }

        std::expected<std::tuple<IPAddress,
                                 std::uint16_t,
                                 std::span<std::byte>>,
                      std::errc>
        recv_from(std::span<std::byte> buffer) {
            auto& packet = getRecvBuffer();
            if(packet.size() < 8) {
                return std::unexpected{std::errc::no_message_available};
            }

            IPAddress senderIp{packet.begin(), packet.end()};

            std::uint16_t senderPort;
            std::memcpy(&senderPort, packet.begin() + 4, 2);
            senderPort = std::byteswap(senderPort);

            std::uint16_t payloadSize;
            std::memcpy(&payloadSize, packet.begin() + 6, 2);
            payloadSize = std::byteswap(payloadSize);

            if(packet.size() - 8 < payloadSize) {
                return std::unexpected{std::errc::no_message_available};
            }

            if(buffer.size() < payloadSize) {
                return std::unexpected{std::errc::no_buffer_space};
            }
            std::copy(packet.begin() + 8, packet.begin() + 8 + payloadSize, buffer.begin());

            packet.erase(packet.begin(), packet.begin() + 8 + payloadSize);

            return std::make_tuple(senderIp, senderPort, std::span{buffer.begin(), payloadSize});
        }

        bool send_multi(IPAddress                  address,
                        std::uint16_t              port,
                        std::span<std::byte const> data) {
            auto& socket = getSocket();
            if(!socket.connected || socket.send_in_progress) {
                return false;
            }
            auto& buffer            = socket.command_buffer;
            socket.send_in_progress = true;
            auto pos                = buffer.begin();
            pos                     = Header{*socket.tx_write,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketTxBuffer(sn.n),
                         Header::Size::variable}
                    .write(pos, buffer.end());

            w5500->selected([&]() {
                w5500->command_q.push(typename Commands::Write{buffer.begin(), pos});
                w5500->command_q.push(typename Commands::Write{data.data(), data.size()});
            });

            auto const cmdpos1 = pos;

            auto const mac = address.toMulticastMac();
            pos            = Header{SocketRegister::DestinationHardwareAddress,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::variable}
                    .write(pos, buffer.end());
            pos = mac.write(pos, buffer.end());

            w5500->selected(
              [&]() { w5500->command_q.push(typename Commands::Write{cmdpos1, pos}); });

            auto const cmdpos2 = pos;

            pos = Header{SocketRegister::DestinationIPAddress,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_4}
                    .write(pos, buffer.end());
            pos = address.write(pos, buffer.end());

            pos = Header{SocketRegister::DestinationPort,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());

            *pos++ = std::byte(port >> 8);
            *pos++ = std::byte(port & 0xff);

            pos = Header{SocketRegister::SourcePort,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());

            *pos++ = std::byte(port >> 8);
            *pos++ = std::byte(port & 0xff);

            pos = Header{SocketRegister::TX_WritePointer,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());
            *socket.tx_write += data.size();
            *pos++ = std::byte(*socket.tx_write >> 8);
            *pos++ = std::byte(*socket.tx_write & 0xff);

            pos = Header{SocketRegister::Command,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());
            *pos++ = 0x21_b;

            pos = socket.add_read_ir(pos, buffer.end(), *w5500, sn);
            //            pos = socket.add_read_tx_write(pos, buffer.end(), *w5500, sn);

            w5500->selected(
              [&]() { w5500->command_q.push(typename Commands::Write{cmdpos2, pos}); });
            w5500->command_q.push(
              typename Commands::Call_Socket{&SocketState::check_send_ready, sn});
            return true;
        }

        bool send_to(IPAddress                  address,
                     std::uint16_t              port,
                     std::span<std::byte const> data) {
            auto& socket = getSocket();
            if(!socket.connected || socket.send_in_progress) {
                return false;
            }
            auto& buffer            = socket.command_buffer;
            socket.send_in_progress = true;
            auto pos                = Header{SocketRegister::DestinationPort,
                              Header::ReadWrite::Write,
                              Header::BlockSelect::SocketRegister(sn.n),
                              Header::Size::_2}
                         .write(buffer.begin(), buffer.end());

            *pos++ = std::byte(port >> 8);
            *pos++ = std::byte(port & 0xff);

            pos = Header{SocketRegister::DestinationIPAddress,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_4}
                    .write(pos, buffer.end());

            pos = address.write(pos, buffer.end());

            pos = Header{*socket.tx_write,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketTxBuffer(sn.n),
                         Header::Size::variable}
                    .write(pos, buffer.end());

            w5500->selected([&]() {
                w5500->command_q.push(typename Commands::Write{buffer.begin(), pos});
                w5500->command_q.push(typename Commands::Write{data.data(), data.size()});
            });

            auto cmdpos = pos;
            pos         = Header{SocketRegister::TX_WritePointer,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());
            *socket.tx_write += data.size();
            *pos++ = std::byte(*socket.tx_write >> 8);
            *pos++ = std::byte(*socket.tx_write & 0xff);

            pos = Header{SocketRegister::Command,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());
            *pos++ = 0x20_b;

            pos = socket.add_read_ir(pos, buffer.end(), *w5500, sn);
            //            pos = socket.add_read_tx_write(pos, buffer.end(), *w5500, sn);

            w5500->selected(
              [&]() { w5500->command_q.push(typename Commands::Write{cmdpos, pos}); });
            w5500->command_q.push(
              typename Commands::Call_Socket{&SocketState::check_send_ready, sn});
            return true;
        }
    };

    struct TCPSocket_ {
        W5500*       w5500{};
        SocketNumber sn{};

        SocketState& getSocket() { return w5500->sockets[sn.n]; }

        SocketState const& getSocket() const { return w5500->sockets[sn.n]; }

        bool send_ready() const { return getSocket().connected && !getSocket().send_in_progress; }

        auto& getRecvBuffer() { return getSocket().recv_buffer; }

        ~TCPSocket_() {
            if(!sn.isInvalid()) {
                w5500->sockets[sn.n].reset(*w5500, sn);
                assert(w5500->sockets[sn.n].free == false);
                w5500->sockets[sn.n].free = true;
            }
        }

        TCPSocket_& operator=(TCPSocket_ const&) = delete;
        TCPSocket_(TCPSocket_ const&)            = delete;

        TCPSocket_(TCPSocket_&& other)
          : w5500{other.w5500}
          , sn{other.sn} {
            other.sn.setInvalid();
        }

        TCPSocket_& operator=(TCPSocket_&& other) {
            if(std::addressof(other) != this) {
                assert(other.w5500 == w5500);
                sn = other.sn;
                other.sn.setInvalid();
            }
            return *this;
        }

        TCPSocket_(W5500&       w5500_,
                   SocketNumber sn_)
          : w5500{std::addressof(w5500_)}
          , sn{sn_} {
            auto& socket = getSocket();
            auto& buffer = socket.command_buffer;
            socket.free  = false;

            auto pos = buffer.begin();

            w5500->selected([&]() {
                w5500->command_q.push(typename Commands::Write{buffer.begin(), pos});
                pos = SocketState::add_read_tx_write(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_read_rx_read(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_clear_ir(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_read_state(pos, buffer.end(), *w5500, sn);
            });

            w5500->command_q.push(
              typename Commands::Call_Socket{&SocketState::check_sock_established, sn});
        }

        TCPSocket_(W5500&        w5500_,
                   SocketNumber  sn_,
                   IPAddress     peer,
                   std::uint16_t port,
                   std::uint16_t localPort)
          : w5500{std::addressof(w5500_)}
          , sn{sn_} {
            auto& socket = getSocket();
            auto& buffer = socket.command_buffer;
            socket.free  = false;

            auto pos = buffer.begin();
            pos      = Header{SocketRegister::Mode,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());
            *pos++ = 1_b;

            pos = Header{SocketRegister::SourcePort,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());

            *pos++ = std::byte(localPort >> 8);
            *pos++ = std::byte(localPort & 0xff);

            pos = Header{SocketRegister::DestinationIPAddress,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_4}
                    .write(pos, buffer.end());

            pos = peer.write(pos, buffer.end());

            pos = Header{SocketRegister::DestinationPort,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());

            *pos++ = std::byte(port >> 8);
            *pos++ = std::byte(port & 0xff);

            pos = Header{SocketRegister::Command,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());

            *pos++ = 0x1_b;

            assert(std::distance(pos, buffer.end()) >= 0);

            w5500->selected([&]() {
                w5500->command_q.push(typename Commands::Write{buffer.begin(), pos});
                pos = SocketState::add_read_tx_write(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_read_rx_read(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_clear_ir(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_read_state(pos, buffer.end(), *w5500, sn);
            });

            w5500->command_q.push(
              typename Commands::Call_Socket{&SocketState::check_sock_init, sn});
        }

        enum class OperationState { succeeded, failed, ongoing };

        bool has_error() {
            if(sn.isInvalid()) {
                return true;
            }
            auto& socket = getSocket();
            return socket.error;
        }

        bool send(std::span<std::byte const> data) {
            auto& socket = getSocket();
            if(!socket.connected || socket.send_in_progress) {
                return false;
            }
            auto& buffer            = socket.command_buffer;
            socket.send_in_progress = true;
            auto pos                = Header{*socket.tx_write,
                              Header::ReadWrite::Write,
                              Header::BlockSelect::SocketTxBuffer(sn.n),
                              Header::Size::variable}
                         .write(buffer.begin(), buffer.end());

            w5500->selected([&]() {
                w5500->command_q.push(typename Commands::Write{buffer.begin(), pos});
                w5500->command_q.push(typename Commands::Write{data.data(), data.size()});
            });
            //           UC_LOG_D("send on {} {}", socket.tx_write, span);
            auto cmdpos = pos;
            pos         = Header{SocketRegister::TX_WritePointer,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());
            *socket.tx_write += data.size();
            *pos++ = std::byte(*socket.tx_write >> 8);
            *pos++ = std::byte(*socket.tx_write & 0xff);

            pos = Header{SocketRegister::Command,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());
            *pos++ = 0x20_b;

            pos = socket.add_read_ir(pos, buffer.end(), *w5500, sn);
            //            pos = socket.add_read_tx_write(pos, buffer.end(), *w5500, sn);

            w5500->selected(
              [&]() { w5500->command_q.push(typename Commands::Write{cmdpos, pos}); });
            w5500->command_q.push(
              typename Commands::Call_Socket{&SocketState::check_send_ready, sn});
            return true;
        }

        std::optional<std::span<std::byte>> recv(std::span<std::byte> buffer) {
            auto& packet = getRecvBuffer();
            if(packet.empty() || buffer.empty()) {
                return std::nullopt;
            }
            std::size_t const size = std::min(buffer.size(), packet.size());

            std::copy(packet.begin(), packet.begin() + size, buffer.begin());
            packet.erase(packet.begin(), packet.begin() + size);

            return std::span{buffer.begin(), size};
        }
    };

    using TCPSocket = TCPSocket_;

    struct TCPServerSocket_ {
        W5500*        w5500{};
        SocketNumber  sn{};
        std::uint16_t port;

        SocketState& getSocket() { return w5500->sockets[sn.n]; }

        SocketState const& getSocket() const { return w5500->sockets[sn.n]; }

        bool send_ready() const { return getSocket().connected && !getSocket().send_in_progress; }

        auto& getRecvBuffer() { return getSocket().recv_buffer; }

        ~TCPServerSocket_() {
            if(!sn.isInvalid()) {
                w5500->sockets[sn.n].reset(*w5500, sn);
                assert(w5500->sockets[sn.n].free == false);
                w5500->sockets[sn.n].free = true;
            }
        }

        TCPServerSocket_& operator=(TCPServerSocket_ const&) = delete;
        TCPServerSocket_(TCPServerSocket_ const&)            = delete;

        TCPServerSocket_(TCPServerSocket_&& other)
          : w5500{other.w5500}
          , sn{other.sn}
          , port{other.port} {
            other.sn.setInvalid();
        }

        TCPServerSocket_& operator=(TCPServerSocket_&& other) {
            if(std::addressof(other) != this) {
                assert(other.w5500 == w5500);
                sn   = other.sn;
                port = other.port;
                other.sn.setInvalid();
            }
            return *this;
        }

        TCPServerSocket_(W5500&        w5500_,
                         SocketNumber  sn_,
                         std::uint16_t port_)
          : w5500{std::addressof(w5500_)}
          , sn{sn_}
          , port{port_} {
            auto& socket = getSocket();
            auto& buffer = socket.command_buffer;
            socket.free  = false;

            auto pos = buffer.begin();
            pos      = Header{SocketRegister::Mode,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());
            *pos++ = 1_b;

            pos = Header{SocketRegister::SourcePort,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_2}
                    .write(pos, buffer.end());

            *pos++ = std::byte(port >> 8);
            *pos++ = std::byte(port & 0xff);

            pos = Header{SocketRegister::Command,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::SocketRegister(sn.n),
                         Header::Size::_1}
                    .write(pos, buffer.end());

            *pos++ = 0x1_b;

            assert(std::distance(pos, buffer.end()) >= 0);

            w5500->selected([&]() {
                w5500->command_q.push(typename Commands::Write{buffer.begin(), pos});
                pos = SocketState::add_clear_ir(pos, buffer.end(), *w5500, sn);
                pos = SocketState::add_read_state(pos, buffer.end(), *w5500, sn);
            });

            w5500->command_q.push(
              typename Commands::Call_Socket{&SocketState::check_sock_init_server, sn});
        }

        bool has_error() {
            if(sn.isInvalid()) {
                return true;
            }
            auto& socket = getSocket();
            return socket.error;
        }

        bool has_pending_connection() {
            if(sn.isInvalid()) {
                return false;
            }
            auto& socket = getSocket();
            return socket.rdyAccept;
        }

        std::optional<TCPSocket> accept() {
            if(sn.isInvalid()) {
                return std::nullopt;
            }
            auto& socket = getSocket();
            if(!socket.rdyAccept) {
                return std::nullopt;
            }
            auto client = TCPSocket_{*w5500, sn};
            sn.setInvalid();
            auto newServer = w5500->tcpServerSocket(port);
            if(newServer) {
                *this = std::move(*newServer);
            } else {
                UC_LOG_D("server stoped now free socket");
            }
            return client;
        }
    };

    template<typename F>
    auto selected(F&& f) {
        struct Selector {
            W5500& w5500;

            Selector(W5500& w5500_) : w5500{w5500_} {
                w5500.command_q.push(typename Commands::Select{});
            }

            ~Selector() { w5500.command_q.push(typename Commands::Deselect{}); }
        };

        auto const selector = Selector{*this};
        return std::invoke(std::forward<F>(f));
    }

    enum class State { reset, reset_wait, reset_wait2, idle };

    State state{State::reset};

    bool linkDisconnectFlag{false};

    typename Clock::time_point timeout{};
    typename Clock::time_point checkWaitTime{};

    static constexpr auto ResetPulseTime    = std::chrono::milliseconds{1};
    static constexpr auto ResetWaitTime     = std::chrono::milliseconds{2};
    static constexpr auto CheckLinkWaitTime = std::chrono::seconds{2};

    void append_yield() { command_q.push(typename Commands::Yield{}); }

    void append_read_mode() {
        auto pos = Header{CommonRegister::Mode,
                          Header::ReadWrite::Read,
                          Header::BlockSelect::CommonRegister(),
                          Header::Size::_1}
                     .write(commandBuffer.begin(), commandBuffer.end());

        selected([&]() {
            command_q.push(typename Commands::Write{commandBuffer.begin(), pos});
            command_q.push(typename Commands::Read{commandBuffer.begin(), 1});
        });
    }

    void append_link_check() {
        auto pos = Header{CommonRegister::PHY_Configuration,
                          Header::ReadWrite::Read,
                          Header::BlockSelect::CommonRegister(),
                          Header::Size::_1}
                     .write(commandBuffer.begin(), commandBuffer.end());

        selected([&]() {
            command_q.push(typename Commands::Write{commandBuffer.begin(), pos});
            command_q.push(typename Commands::Read{commandBuffer.begin(), 1});
        });
        command_q.push(typename Commands::Call_Self{&W5500::check_link});
    }

    void append_link_disconnect_check() {
        auto pos = Header{CommonRegister::PHY_Configuration,
                          Header::ReadWrite::Read,
                          Header::BlockSelect::CommonRegister(),
                          Header::Size::_1}
                     .write(commandBuffer.begin(), commandBuffer.end());

        selected([&]() {
            command_q.push(typename Commands::Write{commandBuffer.begin(), pos});
            command_q.push(typename Commands::Read{commandBuffer.begin(), 1});
        });
        command_q.push(typename Commands::Call_Self{&W5500::check_link_disconnect});
    }

    void append_chip_check() {
        auto pos = Header{CommonRegister::ChipVersion,
                          Header::ReadWrite::Read,
                          Header::BlockSelect::CommonRegister(),
                          Header::Size::_1}
                     .write(commandBuffer.begin(), commandBuffer.end());

        selected([&]() {
            command_q.push(typename Commands::Write{commandBuffer.begin(), pos});
            command_q.push(typename Commands::Read{commandBuffer.begin(), 1});
        });
        command_q.push(typename Commands::Call_Self{&W5500::check_chip});
    }

    void append_read_common_regs() {
        auto pos = Header{CommonRegister::Mode,
                          Header::ReadWrite::Read,
                          Header::BlockSelect::CommonRegister(),
                          Header::Size::variable}
                     .write(commandBuffer.begin(), commandBuffer.end());

        selected([&]() {
            command_q.push(typename Commands::Write{commandBuffer.begin(), pos});
            command_q.push(typename Commands::Read{commandBuffer.begin(), 20});
        });
        command_q.push(typename Commands::Call_Self{&W5500::dump_common_regs});
    }

    void append_reset() {
        auto pos = Header{CommonRegister::Mode,
                          Header::ReadWrite::Write,
                          Header::BlockSelect::CommonRegister(),
                          Header::Size::_1}
                     .write(commandBuffer.begin(), commandBuffer.end());

        *pos++ = 0x80_b;
        selected([&]() { command_q.push(typename Commands::Write{commandBuffer.begin(), pos}); });
        command_q.push(typename Commands::Call_Self{&W5500::read_mode});
    }

    void dump_common_regs() {
        UC_LOG_D("{}", std::span{commandBuffer.begin(), commandBuffer.begin() + 20});
    }

    template<typename I>
    constexpr auto writeIpConfig(I         first,
                                 I         last,
                                 IPAddress ownIp,
                                 IPAddress gateway,
                                 IPAddress subnetMask) {
        auto pos = first;
        pos      = Header{CommonRegister::GatewayAddress,
                     Header::ReadWrite::Write,
                     Header::BlockSelect::CommonRegister(),
                     Header::Size::_4}
                .write(pos, last);
        pos = gateway.write(pos, last);

        pos = Header{CommonRegister::SubnetMaskAddress,
                     Header::ReadWrite::Write,
                     Header::BlockSelect::CommonRegister(),
                     Header::Size::_4}
                .write(pos, last);
        pos = subnetMask.write(pos, last);

        pos = Header{CommonRegister::SourceIpAddress,
                     Header::ReadWrite::Write,
                     Header::BlockSelect::CommonRegister(),
                     Header::Size::_4}
                .write(pos, last);
        pos = ownIp.write(pos, last);

        return pos;
    }

    void append_mac_config(MacAddress const& mac) {
        auto pos = commandBuffer.begin();
        selected([&]() {
            pos = Header{CommonRegister::SourceHardwareAddress,
                         Header::ReadWrite::Write,
                         Header::BlockSelect::CommonRegister(),
                         Header::Size::variable}
                    .write(pos, commandBuffer.end());
            pos = mac.write(pos, commandBuffer.end());
            command_q.push(typename Commands::Write{commandBuffer.begin(), pos});
        });
        command_q.push(typename Commands::Call_Self{&W5500::mac_set});
    }

    void append_ip_config(IPAddress ownIp,
                          IPAddress gateway,
                          IPAddress subnetMask) {
        auto pos = commandBuffer.begin();
        selected([&]() {
            UC_LOG_D("ip config ip: {} gw: {} sm: {} hasDHCP: {}",
                     ownIp.octets,
                     gateway.octets,
                     subnetMask.octets,
                     hasDHCP);

            pos = writeIpConfig(pos, commandBuffer.end(), ownIp, gateway, subnetMask);

            command_q.push(typename Commands::Write{commandBuffer.begin(), pos});
        });
        command_q.push(typename Commands::Call_Self{&W5500::ip_set});
    }

    void mac_set() {
        append_ip_config(Config::IpConfig.ownIp,
                         Config::IpConfig.gateway,
                         Config::IpConfig.subnetMask);
    }

    void ip_set() {
        ready = true;
        //UC_LOG_D("ip set");
        //append_read_common_regs();
    }

    bool                      ready = false;
    std::array<std::byte, 64> commandBuffer;

    void read_mode() {
        append_read_mode();
        command_q.push(typename Commands::Call_Self{&W5500::check_reset});
    }

    void check_reset() {
        if((commandBuffer[0] & 0x80_b) == 0_b) {
            UC_LOG_D("reset ok");
            append_chip_check();
        } else {
            UC_LOG_D("reset failed");
            append_yield();
            append_read_mode();
        }
    }

    void check_chip() {
        if(commandBuffer[0] == 4_b) {
            UC_LOG_D("chip ok");
            append_link_check();
        } else {
            UC_LOG_D("chip fail {}", commandBuffer[0]);
            command_q.push(typename Commands::Reset{});
        }
    }

    void check_link() {
        if((commandBuffer[0] & 0x1_b) == 0x1_b) {
            UC_LOG_D("link ok");
            append_mac_config(Config::Mac());
        } else {
            //UC_LOG_D("link not ok");
            append_yield();
            append_link_check();
        }
    }

    void check_link_disconnect() {
        if((commandBuffer[0] & 0x1_b) == 0x1_b) {
            UC_LOG_D("link ok");
            linkDisconnectFlag = false;
        } else {
            UC_LOG_D("link not ok");
            linkDisconnectFlag = true;
        }
    }

    std::atomic<bool>     command_in_progress = false;
    static constexpr auto dummyRecvByte       = 0xff_b;

    void clear_commands_of(SocketNumber) {}

    void processCommandQ() {
        auto const end = Clock::now() + 1ms;
        while(!command_in_progress && !command_q.empty() && end > Clock::now()
              && SPI::operationState() == SPI::OperationState::succeeded)
        {
            auto const cmd = command_q.front();
            command_q.pop();

            if(std::holds_alternative<typename Commands::Yield>(cmd)) {
                return;
            }

            std::visit([&](auto const& v) { return handle(v); }, cmd);
        }
    }

    void handle(typename Commands::Yield) {
        //   UC_LOG_D("yield");
    }

    void handle(typename Commands::Select) {
        //     UC_LOG_D("select");
        apply(clear(Cs{}));
    }

    void handle(typename Commands::Deselect) {
        //   UC_LOG_D("deselect");
        apply(set(Cs{}));
        Clock::template delay<std::chrono::nanoseconds, 35>();
    }

    void handle(typename Commands::Write const& c) {
        // UC_LOG_D("write");
        command_in_progress = true;
        SPI::send_nocopy(std::span{c.buffer, c.size}, [&]() { command_in_progress = false; });
    }

    void handle(typename Commands::Read const& c) {
        //    UC_LOG_D("read");
        command_in_progress = true;
        SPI::send_receive_nocopy_static(&dummyRecvByte, std::span{c.buffer, c.size}, [&]() {
            command_in_progress = false;
        });
    }

    void handle(typename Commands::Call_Self const& c) {
        //  UC_LOG_D("call self");
        std::invoke(c.function, this);
    }

    void handle(typename Commands::Call_Socket const& c) {
        // UC_LOG_D("call socket");
        std::invoke(c.function, sockets[c.n.n], *this, c.n);
    }

    void handle(typename Commands::Reset) {
        // UC_LOG_D("reset request");
        state               = State::reset;
        command_in_progress = true;
    }

    void handler() {
        auto const now = Clock::now();
        switch(state) {
        case State::reset:
            {
                command_in_progress = false;
                command_q.clear();
                for(auto& socket : sockets) {
                    socket.reset_raw();
                }
                apply(clear(Rst{}), set(Cs{}));
                timeout              = now + ResetPulseTime;
                checkWaitTime        = now + CheckLinkWaitTime;
                state                = State::reset_wait;
                ready                = false;
                linkDisconnectFlag   = false;
                currentEphemeralPort = EphemeralPortRange::min;
            }
            break;
        case State::reset_wait:
            {
                if(now >= timeout) {
                    apply(set(Rst{}));
                    timeout = now + ResetWaitTime;
                    state   = State::reset_wait2;
                }
            }
            break;
        case State::reset_wait2:
            {
                if(now >= timeout) {
                    timeout = now + ResetWaitTime;
                    state   = State::idle;
                    append_reset();
                }
            }
            break;
        case State::idle:
            {
                //checks every x(2) seconds the link
                if(now >= checkWaitTime) {
                    append_link_disconnect_check();
                    checkWaitTime = now + CheckLinkWaitTime;
                    //if disconnected more than x seconds reset
                    if(linkDisconnectFlag) {
                        //TODO reset after 10 secs when disconnected
                        if(now >= timeout) {
                            apply(Kvasir::SystemControl::SystemReset());
                        }
                    } else {
                        timeout = now + 10s;
                    }
                }
                //UC_LOG_D("state IDLE\n");
                processCommandQ();
                if(ready) {
                    dhcpHandler.handle();
                }
            }
            break;
        }
    }

    std::uint16_t currentEphemeralPort{};

    std::uint16_t nextEphemeralPort() {
        auto ret    = currentEphemeralPort;
        auto isFree = [](std::uint16_t) { return true; };   //TODO
        do {
            ++currentEphemeralPort;
            if(currentEphemeralPort > EphemeralPortRange::max) {
                currentEphemeralPort = EphemeralPortRange::min;
            }
        } while(!isFree(currentEphemeralPort));

        return ret;
    }

    using TCPServerSocket = TCPServerSocket_;
    using UDPSocket       = UDPSocket_;

    using DHCPHandler_t = std::conditional_t<hasDHCP, DHCPHandler<W5500>, EmptyDHCPHandler<W5500>>;

    DHCPHandler_t dhcpHandler{*this};

    std::optional<TCPSocket> tcpSocket(IPAddress const& ip,
                                       std::uint16_t    port) {
        if(!ready) {
            return std::nullopt;
        }

        if constexpr(hasDHCP) {
            if(!dhcpHandler.ready) {
                return std::nullopt;
            }
        }

        std::uint8_t n{};
        for(auto const& s : sockets) {
            if(s.free && s.ready) {
                auto ep = nextEphemeralPort();
                UC_LOG_D("TCP on {}", n);
                return TCPSocket{*this, SocketNumber{n}, ip, port, ep};
            }
            ++n;
        }
        return {};
    }

    std::optional<TCPServerSocket> tcpServerSocket(std::uint16_t port) {
        if(!ready) {
            return std::nullopt;
        }

        if constexpr(hasDHCP) {
            if(!dhcpHandler.ready) {
                return std::nullopt;
            }
        }

        std::uint8_t n{};
        for(auto const& s : sockets) {
            if(s.free && s.ready) {
                UC_LOG_D("TCPServer on {}", n);
                return TCPServerSocket{*this, SocketNumber{n}, port};
            }
            ++n;
        }
        return {};
    }

    std::optional<UDPSocket> udpSocket(std::uint16_t port) {
        if(!ready) {
            return std::nullopt;
        }

        if constexpr(hasDHCP) {
            if(!dhcpHandler.ready && port != DHCPHandler_t::ClientPort) {
                return std::nullopt;
            }
        }

        std::uint8_t n{};
        for(auto const& s : sockets) {
            if(s.free && s.ready) {
                UC_LOG_D("UDPSocket on {}", n);
                return UDPSocket{*this, SocketNumber{n}, port, std::nullopt};
            }
            ++n;
        }
        return {};
    }

    std::optional<UDPSocket> udpMulticastSocket(std::uint16_t port,
                                                IPAddress     multicast) {
        if(!ready) {
            UC_LOG_D("not ready");
            return std::nullopt;
        }

        if constexpr(hasDHCP) {
            if(!dhcpHandler.ready) {
                UC_LOG_D("no dhcp");
                return std::nullopt;
            }
        }

        std::uint8_t n{};
        for(auto const& s : sockets) {
            if(s.free && s.ready) {
                UC_LOG_D("UDPMulticast on {}", n);
                return UDPSocket{*this, SocketNumber{n}, port, multicast};
            }
            ++n;
        }
        UC_LOG_T("no free socket");
        return {};
    }
};
