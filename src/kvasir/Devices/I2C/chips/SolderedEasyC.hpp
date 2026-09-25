#pragma once

#include "../Device.hpp"
#include "../Quantities.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace Kvasir::I2C::Chips {

/// The two Soldered "easyC" input boards. Neither is a sensor IC with a datasheet: each is
/// an ATtiny running Soldered's own firmware, and the protocol below is taken from their
/// Arduino libraries (Soldered-Slider-Potentiometer-with-easyC and
/// Soldered-Rotary-Encoder-With-easyC) and the shared easyC base class. Both default to
/// address 0x30, which is set by solder jumpers on the board.

/// Slider potentiometer with easyC (333131). One register: ANALOG_READ_REG (0), which
/// returns the 10-bit ADC reading as two little-endian bytes, 0..1023.
template<Address7 Addr = 0x30>
struct SolderedSlider {
    static constexpr std::string_view Name          = "easyC slider";
    static constexpr Address7         Address       = Addr;
    static constexpr std::size_t      RegisterBytes = 1;

    /// The jumpers set the address; only the one this instance is built for is known here.
    static constexpr std::array<Address7, 1> Addresses{Addr};

    static constexpr std::uint16_t FullScale = 1023;

    static constexpr std::array Init{Step::read({.reg = 0x00, .count = 2, .offset = 0})};

    struct Position {
        static constexpr auto       Period = std::chrono::milliseconds{50};
        static constexpr std::array Steps{Step::read({.reg = 0x00, .count = 2, .offset = 0})};

        struct Sample {
            std::uint16_t raw{};   ///< 0..1023

            /// The travel, 0..100 % of full scale.
            [[nodiscard]] constexpr Percent percent() const {
                return Units::percent(static_cast<std::uint32_t>(raw) * 100U / FullScale);
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            return {static_cast<std::uint16_t>(data.le16(0) & 0x03FFU)};
        }
    };

    using Reads = List<Position>;
};

/// Rotary encoder with easyC (333188). There is no register pointer at all: a bare five
/// byte read returns a signed 32-bit counter, little endian, followed by a one-byte event.
/// Writing the single byte 0xAA (170) zeroes the counter.
///
/// Whether the board clears the event byte once it has been read is not in the library (its
/// firmware is not published); a caller that wants every click polls faster than the user can
/// generate them either way, and `fresh<Motion>()` says whether the sample in hand is one that
/// has not been looked at yet.
template<Address7 Addr = 0x30>
struct SolderedRotary {
    static constexpr std::string_view Name          = "easyC rotary";
    static constexpr Address7         Address       = Addr;
    static constexpr std::size_t      RegisterBytes = 0;

    /// The jumpers set the address; only the one this instance is built for is known here.
    static constexpr std::array<Address7, 1> Addresses{Addr};

    enum class Event : std::uint8_t {
        idle         = 0,
        click        = 1,
        doubleClick  = 2,
        longPress    = 3,
        longRelease  = 4,
        counterClock = 5,
        clockwise    = 6,
    };

    static constexpr std::array Init{Step::receive({.count = 5, .offset = 0})};

    struct Motion {
        static constexpr auto       Period = std::chrono::milliseconds{20};
        static constexpr std::array Steps{Step::receive({.count = 5, .offset = 0})};

        struct Sample {
            std::int32_t count{};
            Event        event{Event::idle};

            [[nodiscard]] constexpr bool turned() const {
                return event == Event::clockwise || event == Event::counterClock;
            }

            [[nodiscard]] constexpr bool pressed() const {
                return event == Event::click || event == Event::doubleClick
                    || event == Event::longPress;
            }
        };

        [[nodiscard]] static constexpr Sample decode(Bytes data) {
            auto const raw = data.le32(0);
            auto const ev  = data.u8(4);
            return {static_cast<std::int32_t>(raw), ev <= 6 ? static_cast<Event>(ev) : Event::idle};
        }
    };

    /// Writes 0xAA, the byte the library's resetCounter() sends, whatever the Value.
    struct Reset {
        using Value                        = std::uint8_t;
        static constexpr std::size_t Bytes = 1;
        /// A one-shot command, not a state to restore after a reset.
        static constexpr bool Transient = true;

        [[nodiscard]] static constexpr Step encode(Value const&,
                                                   std::span<std::byte> buffer) {
            buffer[0] = std::byte{0xAA};
            return Step::commandBuffer({.offset = 0, .count = 1});
        }
    };

    using Reads  = List<Motion>;
    using Writes = List<Reset>;
};

}   // namespace Kvasir::I2C::Chips
