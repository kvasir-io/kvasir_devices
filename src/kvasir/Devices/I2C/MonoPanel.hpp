#pragma once

#include "Device.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

/// A one-bit OLED driven by a chip description (chips/Ssd1306.hpp), presented as the panel a
/// frame renderer draws through.
///
/// The description's Page write group *is* the frame: `items<Page>()` hands out its pages
/// and `touch<Page>(p)` says which of them changed, so a canvas draws straight into the
/// bytes that go on the wire and only the pages that moved are sent. Nothing here includes
/// the drawing library -- the contract is `pageRam()` and `pagesChanged()` over plain
/// arrays, which `gfx::MonoPanel` matches structurally.
///
/// `MonoPanelRef` holds the device by reference, so the device may be a local in main() or a
/// member of an application object -- which is where a device belongs, since nothing in the
/// engine needs it to be found by anyone:
///
///     Kvasir::I2C::Device<I2c1, Clock, Chips::Ssd1315<HW::Oled>> oled{};
///     gfx::MonoFrame<Kvasir::I2C::MonoPanelRef<decltype(oled)>, gfx::MonoCrispTraits>
///       screen{Kvasir::I2C::MonoPanelRef{oled}};
///     ...
///     oled.handler();                                   // once per loop turn
///     screen.frame([&](auto& c) { draw(c); });          // false while the last is on the wire
///
/// A device inside a `Bus` -- where a firmware with more than one usually keeps it -- is
/// reached with `monoPanel<Which>(bus)`:
///
///     Kvasir::I2C::Bus<I2c1, Clock, OledWide, OledSmall> bus{};
///     auto wide  = Kvasir::I2C::monoPanel<OledWide>(bus);
///     auto small = Kvasir::I2C::monoPanel<OledSmall>(bus);
///
/// There is deliberately no form that names the device as a template argument: a reference
/// template parameter only accepts an object with static storage duration and linkage, so such
/// a panel would oblige every device behind it to be a global.
namespace Kvasir::I2C {

/// The panel: it keeps a pointer to the device rather than naming it, so the device may live
/// wherever the application keeps it.
template<typename DeviceT>
struct MonoPanelRef {
    using Device = DeviceT;
    using Chip   = typename Device::Chip;
    using Page   = typename Chip::Page;

    static constexpr int Width  = Chip::Width;
    static constexpr int Height = Chip::Height;
    static constexpr int Pages  = Chip::Pages;

    constexpr explicit MonoPanelRef(Device& device) : device_{&device} {}

    /// Brought up, answering, and nothing owed or on the wire: a frame may be drawn.
    [[nodiscard]] bool ready() const { return device_->answering() && !device_->pending(); }

    /// An absent module is parked and probed by the engine (Presence.hpp), not given up on.
    [[nodiscard]] bool failed() const { return false; }

    void handler() { device_->handler(); }

    /// The frame, page 0 first, in the controller's own layout.
    [[nodiscard]] std::span<std::array<std::uint8_t,
                                       std::size_t(Width)>,
                            std::size_t(Pages)>
    pageRam() {
        return device_->template items<Page>();
    }

    /// A bitmask of the pages that were drawn into. Each one whose bytes differ from what was
    /// last sent becomes one transaction pair; a page drawn into but left as it was is not sent.
    ///
    /// "Drawn into" is all a canvas can say, and a frame that starts with clear() draws into
    /// every page: sent as it stood, a panel redrawn ten times a second would put every page on
    /// the bus each time, nearly all of them the same bytes again, and a mostly static panel
    /// behind a switch would hold its channel for much of the time. A 32-bit hash per page of
    /// what was last handed to the engine stands in for a copy of the frame, which this panel
    /// exists not to keep. A write that fails is re-sent by the engine, and a
    /// device reset re-sends everything the application wrote, so a page skipped here as
    /// unchanged is on the glass.
    void pagesChanged(std::uint32_t pages) {
        auto const ram = pageRam();
        for(std::size_t p = 0; p < std::size_t(Pages); ++p) {
            if(((pages >> p) & 1U) == 0) { continue; }
            auto const hash = hash_(ram[p]);
            if(((sent_ >> p) & 1U) != 0 && hashes_[p] == hash) { continue; }
            hashes_[p] = hash;
            sent_ |= 1UL << p;
            device_->template touch<Page>(p);
        }
    }

    [[nodiscard]] Device& device() const { return *device_; }

private:
    /// FNV-1a over one page.
    [[nodiscard]] static constexpr std::uint32_t hash_(std::span<std::uint8_t const> bytes) {
        std::uint32_t h = 2166136261U;
        for(auto const b : bytes) {
            h ^= b;
            h *= 16777619U;
        }
        return h;
    }

    Device*                                       device_;
    std::array<std::uint32_t, std::size_t(Pages)> hashes_{};   ///< of each page as last sent
    std::uint32_t                                 sent_{};     ///< pages that have a hash
};

/// The panel for one device of a `Bus`: `monoPanel<OledWide>(bus)`. A Bus member cannot be
/// named as a template argument, but it can be reached through the bus.
template<typename Which,
         typename Owner>
[[nodiscard]] constexpr MonoPanelRef<Which> monoPanel(Owner& owner) {
    return MonoPanelRef<Which>{owner.template get<Which>()};
}

}   // namespace Kvasir::I2C
