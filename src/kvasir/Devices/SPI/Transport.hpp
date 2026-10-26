#pragma once
// A queued SPI master in the shape of the I2C master the chip-description engine expects: each
// engine step is one SPI frame - the register byte turned into the chip's read or write command,
// then the data straight into or out of the engine's buffer.
//
// An SPI chip description is written like an I2C one, plus:
//
//     static constexpr Kvasir::Hal::ClockMode Mode     = Kvasir::Hal::ClockMode::mode3;
//     static constexpr Kvasir::Units::Hertz   MaxClock = Kvasir::Units::hertz(5'000'000);
//     static constexpr std::uint8_t readCommand(std::uint8_t reg)  { return reg | 0x80; }  // optional
//     static constexpr std::uint8_t writeCommand(std::uint8_t reg) { return reg & 0x7F; }  // optional
//     static constexpr std::size_t  DummyBytes = 0;   // clocks between command and read data, optional
//
// There is no NAK: a part is there when its Identity matches (or setup() accepts a read-back).
// Log lines name the device "at 0x15": for SPI that is its chip-select GPIO.
#include "../BusTypes.hpp"
#include "../Engine/EngineFeatures.hpp"
#include "../Quantities.hpp"
#include "Client.hpp"
#include "QueueCore.hpp"
#include "kvasir/Hal/Spi.hpp"

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace SPI {

    template<typename C>
    concept SpiChip = requires {
        { C::Name } -> std::convertible_to<std::string_view>;
        { C::Mode } -> std::convertible_to<Hal::ClockMode>;
        C::MaxClock;
    } && (!requires { C::RegisterBytes; } || C::RegisterBytes <= 1);

    namespace detail {

        template<typename Chip>
        constexpr std::uint8_t readCommand(std::uint8_t reg) {
            if constexpr(requires { Chip::readCommand(reg); }) {
                return Chip::readCommand(reg);
            } else {
                return static_cast<std::uint8_t>(reg | 0x80U);
            }
        }

        template<typename Chip>
        constexpr std::uint8_t writeCommand(std::uint8_t reg) {
            if constexpr(requires { Chip::writeCommand(reg); }) {
                return Chip::writeCommand(reg);
            } else {
                return static_cast<std::uint8_t>(reg & 0x7FU);
            }
        }

        template<typename Chip>
        constexpr std::size_t dummyBytes() {
            if constexpr(requires { Chip::dummyBytes; }) {
                return Chip::dummyBytes;
            } else {
                return 0;
            }
        }

        /// For the log lines: port * 32 + pin, or a fake pin's id.
        template<template<int,
                          int> class L,
                 int Port,
                 int Pin>
        constexpr std::uint8_t pinNumber(L<Port,
                                           Pin>) {
            return static_cast<std::uint8_t>(Port * 32 + Pin);
        }

        template<typename Cs>
        constexpr std::uint8_t csId() {
            if constexpr(requires { Cs::id; }) {
                return static_cast<std::uint8_t>(Cs::id);
            } else if constexpr(requires { pinNumber(Cs{}); }) {
                return pinNumber(Cs{});
            } else {
                return 0;
            }
        }
    }   // namespace detail

    /// What the engine hands an SPI transport. Outside Transport, so it names nothing but the
    /// callback size: every SPI device of one callback size is on one engine port
    /// (Engine::detail::PortOf) and shares one Engine - nested, each device's Request was a type of
    /// its own and each device got an Engine copy of its own (1.5-2.2 KB of flash a device).
    ///
    /// The engine puts the register into `prefix` (the queued I2C drivers' shape) and the
    /// payload alone into sendData: the command and the data are two spans here already.
    template<std::size_t CallbackSize>
    struct TransportRequest {
        static constexpr std::size_t MaxPrefix = 2;

        std::uint8_t                                     address{};   // unused on SPI
        std::uint8_t                                     prefixBytes{};
        std::array<std::byte, MaxPrefix>                 prefix{};
        std::span<std::byte const>                       sendData{};
        std::span<std::byte>                             receiveData{};
        StaticFunction<void(Bus::Outcome), CallbackSize> callback{};
    };

    template<typename Master, typename Cs, SpiChip Chip, std::size_t CallbackCapacity = 16>
    struct Transport {
        static constexpr std::size_t CallbackSize = CallbackCapacity;
        using Result                              = Bus::Outcome;
        using MasterT                             = Master;
        /// SPI has no NAK, so nothing to park (Presence), and the queued master times out a
        /// transfer itself (QueueCore), so the engine needs no net of its own; no bridges.
        static constexpr Engine::EngineFeatures Features{.switchable  = false,
                                                         .presence    = false,
                                                         .inFlightNet = false};

        using Request = TransportRequest<CallbackSize>;

        /// What the engine asks a transport (Engine/Device.hpp): the log names a part by its
        /// chip-select GPIO, an SPI part has no clock of its own to put into a request.
        template<typename,
                 typename>
        static constexpr std::uint8_t logId() {
            return detail::csId<Cs>();
        }

        template<typename,
                 typename>
        static constexpr std::uint32_t ownClockHz() {
            return 0;
        }

        static constexpr bool perDeviceClock = false;

        template<typename>
        static constexpr bool accepts = true;

        static constexpr auto Setup = Master::setup(Chip::Mode, Chip::MaxClock);

        /// The engine has one step on the wire per device, so command and callback live here.
        static bool submit(Request const& r) {
            typename Master::Request m{.setup = Setup, .lines = csLines};
            std::size_t              c      = 0;
            bool const               isRead = !r.receiveData.empty();
            if(r.prefixBytes != 0) {
                auto const reg = std::to_integer<std::uint8_t>(r.prefix[0]);
                command_[c++]  = std::byte{isRead ? detail::readCommand<Chip>(reg)
                                                  : detail::writeCommand<Chip>(reg)};
                if(isRead) {
                    for(std::size_t i = 0; i < detail::dummyBytes<Chip>(); ++i) {
                        command_[c++] = std::byte{0xFF};
                    }
                }
                m.command = std::span<std::byte const>{command_}.first(c);
            }
            if(isRead) {
                m.rx = r.receiveData;
            } else if(!r.sendData.empty()) {
                m.tx = r.sendData;
            }
            callback_ = r.callback;
            m.callback
              = [](auto res) {   // Bus::Outcome, or on a tracked master Bus::TrackedOutcome
                    if constexpr(std::is_same_v<decltype(res), Result>) {
                        callback_(res);
                    } else {
                        callback_(res == decltype(res)::ok ? Result::ok : Result::failed);
                    }
                };
            return Master::submit(m);
        }

    private:
        static constexpr Lines csLines = ChipSelect<Cs>::lines;

        inline static std::array<std::byte, 1 + detail::dummyBytes<Chip>()> command_{};
        inline static StaticFunction<void(Result), CallbackSize>            callback_{};
    };

}}   // namespace Kvasir::SPI
