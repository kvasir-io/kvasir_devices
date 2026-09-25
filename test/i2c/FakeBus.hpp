#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <initializer_list>
#include <map>
#include <span>
#include <support/FakeClock.hpp>
#include <vector>

/// A queued I2C bus for host tests of the device drivers, in the shape of the SDK's
/// I2CBehaviorQueued (a `Request` of address, bytes to send, a receive buffer and a
/// completion callback; `submit()` true when it was queued), but answered by a *device model*:
/// `respond` is a function the test installs that sees every completed request -- address,
/// bytes sent, the receive buffer to fill -- and returns the result. `RegisterModel` below
/// is the model most chips need (a register file with auto-increment); command-and-wait
/// chips (Sensirion, AHT20) get a lambda of their own in their test.
///
/// `submit` records the request in `log` and keeps it pending; `complete()` answers every
/// pending request through the model, the way the I2C interrupt would, running the
/// request's callback. The faults a model cannot cause are injected on the oldest pending
/// request: `dataNak()` (the address was acknowledged, a data byte was not: the bus reports
/// `failed`), `timeout()` (the transfer timed out: the clock moves by the bus's timeout, then
/// `failed`), `partialRead(n)` (n bytes landed, then `failed`), `drain()` (a bus recovery:
/// everything pending fails at once) and `lose()` (the bus never answers; the request is
/// handed back so the test can complete it late). `refuse` / `refuseNext` make submit()
/// answer false: a full queue.
///
/// A bus is static, the way a Kvasir bus behavior is, so its state is per *type*:
/// `FakeBusFor<Tag>` is one bus per tag, and two tags are two buses with their own queue,
/// model and transcript (an I2C0 and an I2C1 in the same test). `FakeBus` is the untagged
/// one every single-bus test uses. A type derived from a FakeBusFor<Tag> (to add BaudRate or
/// QueueDepth) is the same bus as its base: tag the base to get a separate one.
namespace Kvasir::Test {

struct Transaction {
    /// A bus transaction, or an edge of a reset line: both go into the one transcript, so a
    /// test can assert that the pulse came before the first transaction and how long it was.
    enum class Kind : std::uint8_t { bus, hold, release };

    Kind                      kind{Kind::bus};
    std::uint8_t              address{};
    std::vector<std::uint8_t> sent{};
    std::size_t               recvLen{};
    std::vector<std::uint8_t> received{};   // filled in by complete()
    FakeClock::time_point     at{};

    [[nodiscard]] bool isBus() const { return kind == Kind::bus; }

    [[nodiscard]] bool isWrite() const { return isBus() && recvLen == 0; }

    [[nodiscard]] bool isRead() const { return isBus() && recvLen != 0; }
};

/// What every fake bus shares whatever its tag: the result, the request and the model's
/// signature. Out of the template so that one device model (RegisterModel, a responder
/// lambda) answers on any of them.
enum class FakeBusResult : std::uint8_t { failed, notAcknowledged, succeeded };

struct FakeBusRequest {
    std::uint8_t                       address{};
    std::span<std::byte const>         sendData{};
    std::span<std::byte>               receiveData{};
    std::function<void(FakeBusResult)> callback{};
};

using FakeBusResponder
  = std::function<FakeBusResult(std::uint8_t, std::span<std::byte const>, std::span<std::byte>)>;

template<typename Tag = void>
struct FakeBusFor {
    using Result    = FakeBusResult;
    using Request   = FakeBusRequest;
    using Responder = FakeBusResponder;

    /// What a real bus's transfer timeout is, for timeout(): tens of milliseconds.
    static constexpr auto TransferTimeout = std::chrono::milliseconds{25};

    static inline std::vector<Transaction> log{};
    static inline std::deque<Request>      pending{};
    static inline Responder                respond{};
    static inline bool                     refuse{false};   ///< every submit() is refused
    static inline int                      refuseNext{};    ///< the next n submit()s are
    static inline int                      submitted{};
    static inline int                      refused{};

    /// False, and nothing queued, while the queue is "full" (refuse / refuseNext).
    static bool submit(Request const& r) {
        ++submitted;
        if(refuse || refuseNext > 0) {
            if(refuseNext > 0) { --refuseNext; }
            ++refused;
            return false;
        }
        Transaction t{};
        t.address = r.address;
        for(auto const b : r.sendData) { t.sent.push_back(static_cast<std::uint8_t>(b)); }
        t.recvLen = r.receiveData.size();
        t.at      = FakeClock::now();
        log.push_back(t);
        pending.push_back(r);
        return true;
    }

    /// Every pending request answered by the model (a NAK when there is none).
    static void complete() {
        auto batch = std::move(pending);
        pending.clear();
        for(auto& r : batch) {
            auto const res
              = respond ? respond(r.address, r.sendData, r.receiveData) : Result::notAcknowledged;
            if(res == Result::succeeded) { received_(r, r.receiveData.size()); }
            ++answered;
            r.callback(res);
        }
    }

    /// The oldest pending request finished with `r` and no data: a fault the model did not
    /// cause. False, and nothing done, when nothing is pending.
    static bool complete(Result r) {
        if(pending.empty()) { return false; }
        auto req = std::move(pending.front());
        pending.pop_front();
        ++answered;
        req.callback(r);
        return true;
    }

    /// The address was acknowledged and a data byte was not: what the bus reports as `failed`
    /// (a NAK on the address alone is `notAcknowledged`).
    static bool dataNak() { return complete(Result::failed); }

    /// The transfer timed out: the bus's own timeout passes, then the request fails.
    static bool timeout() {
        if(pending.empty()) { return false; }
        FakeClock::current += TransferTimeout;
        return complete(Result::failed);
    }

    /// `n` bytes of the oldest pending read landed (from the model), then the transfer failed:
    /// a receive buffer that is partly written is what a driver must not take for an answer.
    static bool partialRead(std::size_t n) {
        if(pending.empty()) { return false; }
        auto req = std::move(pending.front());
        pending.pop_front();
        if(respond) {
            std::vector<std::byte> whole(req.receiveData.size());
            static_cast<void>(respond(req.address, req.sendData, std::span<std::byte>{whole}));
            auto const got = n < whole.size() ? n : whole.size();
            for(std::size_t i = 0; i < got; ++i) { req.receiveData[i] = whole[i]; }
            received_(req, got);
        }
        ++answered;
        req.callback(Result::failed);
        return true;
    }

    /// A bus recovery: everything pending fails at once, oldest first. How many did.
    static std::size_t drain() {
        std::size_t n = 0;
        while(complete(Result::failed)) { ++n; }
        return n;
    }

    /// The bus loses the oldest pending request: no callback comes. It is handed back so the
    /// test can complete it late -- after the client gave up on it -- and see that ignored.
    static Request lose() {
        auto req = std::move(pending.front());
        pending.pop_front();
        ++answered;
        return req;
    }

    static void reset() {
        log.clear();
        pending.clear();
        respond    = {};
        refuse     = false;
        refuseNext = 0;
        submitted  = 0;
        refused    = 0;
        answered   = 0;
    }

private:
    /// Requests leave the queue in the order they were submitted, so the transcript entry of
    /// the one completing now is the `answered`-th bus entry (the reset edges are not
    /// requests). Counted on every way off the queue, lose() included.
    static inline std::size_t answered{};

    /// The bytes the request completing now received, into its transcript entry.
    static void received_(Request const& r,
                          std::size_t    n) {
        std::size_t k = 0;
        for(auto& t : log) {
            if(!t.isBus()) { continue; }
            if(k++ != answered) { continue; }
            for(std::size_t i = 0; i < n; ++i) {
                t.received.push_back(static_cast<std::uint8_t>(r.receiveData[i]));
            }
            return;
        }
    }
};

/// The one bus of a single-bus test.
using FakeBus = FakeBusFor<>;

/// A reset line that writes its two edges into the transcript of the bus tagged `Tag`.
template<typename Tag = void>
struct FakeResetLineFor {
    static void hold() {
        FakeBusFor<Tag>::log.push_back(
          Transaction{Transaction::Kind::hold, 0, {}, 0, {}, FakeClock::now()});
    }

    static void release() {
        FakeBusFor<Tag>::log.push_back(
          Transaction{Transaction::Kind::release, 0, {}, 0, {}, FakeClock::now()});
    }
};

using FakeResetLine = FakeResetLineFor<>;

/// The device most chips are on the wire: a register file addressed by the first
/// `RegBytes` bytes of a write (big-endian), auto-incrementing, at one address. A register
/// is `Width` bytes (1 for the byte-register chips, 2 for the TI 16-bit ones), sent
/// MSB first. A write stores the payload from the register on; a read after a register
/// write (the queued bus's write-then-read) returns from the pointer, a bare read from the
/// pointer as it was left. Writes to registers listed in `readOnly` are dropped, which is
/// what a real chip does with a data register. `onWrite` sees every stored register, for a
/// test that wants a side effect (a conversion started by a config write).
template<std::size_t RegBytes = 1, std::size_t Width = 1>
struct RegisterModel {
    using Reg = std::array<std::uint8_t, Width>;

    std::uint8_t                                   address{};
    std::map<std::uint32_t, Reg>                   mem{};
    std::uint32_t                                  pointer{};
    std::vector<std::uint32_t>                     readOnly{};
    std::function<void(std::uint32_t, Reg const&)> onWrite{};
    int                                            writes{};
    int                                            reads{};

    explicit RegisterModel(std::uint8_t addr) : address{addr} {}

    /// Consecutive registers from `reg`, `Width` bytes each, MSB first.
    void set(std::uint32_t                       reg,
             std::initializer_list<std::uint8_t> bytes) {
        Reg         r{};
        std::size_t i = 0;
        for(auto const b : bytes) {
            r[i++] = b;
            if(i == Width) {
                mem[reg++] = r;
                i          = 0;
            }
        }
    }

    [[nodiscard]] Reg get(std::uint32_t reg) const {
        auto const it = mem.find(reg);
        return it == mem.end() ? Reg{} : it->second;
    }

    /// The register as one big-endian number.
    [[nodiscard]] std::uint32_t word(std::uint32_t reg) const {
        std::uint32_t v = 0;
        for(auto const b : get(reg)) { v = (v << 8) | b; }
        return v;
    }

    FakeBusResult operator()(std::uint8_t               addr,
                             std::span<std::byte const> sent,
                             std::span<std::byte>       recv) {
        if(addr != address) { return FakeBusResult::notAcknowledged; }
        if(!sent.empty()) {
            if(sent.size() < RegBytes) { return FakeBusResult::failed; }
            pointer = 0;
            for(std::size_t i = 0; i < RegBytes; ++i) {
                pointer = (pointer << 8) | static_cast<std::uint8_t>(sent[i]);
            }
            auto const payload = sent.size() - RegBytes;
            if(payload % Width != 0) { return FakeBusResult::failed; }
            for(std::size_t i = RegBytes; i < sent.size(); i += Width) {
                Reg r{};
                for(std::size_t k = 0; k < Width; ++k) {
                    r[k] = static_cast<std::uint8_t>(sent[i + k]);
                }
                bool ro = false;
                for(auto const x : readOnly) { ro = ro || x == pointer; }
                if(!ro) { mem[pointer] = r; }
                if(onWrite) { onWrite(pointer, r); }
                ++pointer;
            }
            if(payload != 0) { ++writes; }
        }
        if(!recv.empty()) {
            if(recv.size() % Width != 0) { return FakeBusResult::failed; }
            for(std::size_t i = 0; i < recv.size(); i += Width) {
                auto const r = get(pointer++);
                for(std::size_t k = 0; k < Width; ++k) {
                    recv[i + k] = static_cast<std::byte>(r[k]);
                }
            }
            ++reads;
        }
        return FakeBusResult::succeeded;
    }
};

/// Several models on one bus, by address.
struct Bus {
    std::vector<FakeBusResponder> devices{};

    FakeBusResult operator()(std::uint8_t               addr,
                             std::span<std::byte const> sent,
                             std::span<std::byte>       recv) {
        for(auto& d : devices) {
            auto const r = d(addr, sent, recv);
            if(r != FakeBusResult::notAcknowledged) { return r; }
        }
        return FakeBusResult::notAcknowledged;
    }
};

}   // namespace Kvasir::Test
