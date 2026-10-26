#pragma once
// The chip hooks of the device engine, written once. Every list the engine keeps of them is generated from
// KVASIR_ENGINE_HOOKS: the fields of the chip table (`detail::Ops`, Engine.hpp), its initializer (DeviceCore's
// `EngineOps`, Device.hpp), a device set's direct dispatch (`detail::SetOps`, SetOps.hpp) and the engine's constant
// accessors (Engine.hpp). Adding a hook is a line here and a static function of the same name in
// `DeviceCore::ChipHooks`; the order of the lines is the order of the table's fields in flash.
//
// The rows, in table order (`State` is the engine's view of the device, `TimePoint` the clock's):
//   script, buffer      the script the current run walks, and the buffer its steps read into or write from
//   sizeRead            a read group whose prepare() sized this run (only chips that need it)
//   countRead           a counted read whose length an earlier step read
//   ready               the running read group's ready(): a check or stopUnless step asks it
//   oracle, identify    the Identity registers against the data sheet, and setup() on a probe copy of the State:
//                       both say whether the script may go on
//   clearInit           the bring-up's buffer, cleared before a run: setup() and an identify step see only this run's
//   finishInit          the end of a bring-up; calls back into the engine (a thunk: a device set passes its own ops)
//   wakeReset           the wake-retry count, which every new transaction starts from
//   redirtyItem         the write item on the wire is still owed
//   unserve             the read on the wire is owed again with its requests still pending
//   forgetSamples       no sample stands for a part that is being brought up again
//   setupFinal          setup() over what the bring-up read: false is "answers, but is not this chip"
//   startupSchedule     every group's first deadline after a bring-up, and the values the chip owes again
//   wakeRetry           a NAK the part gives while it wakes: true when it went on the wire again, not counted
//   startupDelayMs, resetLowMs, resetSettleMs   the three waits of a power-on
//   reads               the chip's cyclic read groups: one ReadGroupInfo each
//   readSlot            the half of group g's slot the engine can touch without knowing the group's type
//   periodNow           the period group g runs at now (what `period<G>(ms)` set, or the chip State's)
//   dynamicPeriod       the one a group whose period follows its readings asks for after this sample
//   prepareRead         the group's prepare() and the buffer lengths this run works with
//   decode              the group's decode(): it stores the sample and says what the engine has to do about it
//   writes, writeSlot   the chip's write groups, the same way
//   encodeWrite         one item encoded into the script the engine then walks
//   afterWrite          the chip now holds the value: what the description makes of that (`applied`)
//   verifySlot          write group w's slot as a verifying one (only for a group that verifies)
//   encodeForVerify     item `item` of group w encoded again into the read-back buffer: what to read
//   verifyMatches       what came back is what was written (the group's verify(), else equal bytes)
//   applyInitial        group w's items that are not in `known` set to its Initial (a group that has one)
//   name, registerBytes, initEmpty   the small fields last, together (between the pointers each cost padding):
//                       the chip's name for the log, 0/1/2 register address bytes, and "sends nothing to come up"
//
// Row kinds: HOOK(name, Result, (args after State&)); HOOK_IF(..., Needs, none) - a device set none of whose chips
// `Needs` it gets `none` instead of a call; THUNK(name, Result, (args)) - SetOps calls `name##Of<Chip>`; CONST(name,
// Type) - data. Only `/* */` comments inside the macro: `//` would swallow the rest of the spliced line.

// clang-format off
#define KVASIR_ENGINE_HOOKS(HOOK, HOOK_IF, THUNK, CONST)                                                         \
    HOOK   (script,          std::span<Step const>,           ())                                               \
    HOOK   (buffer,          std::span<std::byte>,            ())                                               \
    HOOK_IF(sizeRead,        void,                            (),                     NeedsSizedRead, void())   \
    HOOK   (countRead,       void,                            (std::uint8_t))                                   \
    HOOK_IF(ready,           bool,                            (),                     NeedsReady,     true)     \
    HOOK_IF(oracle,          bool,                            (),                     NeedsOracle,    true)     \
    HOOK   (identify,        bool,                            ())                                               \
    HOOK   (clearInit,       void,                            ())                                               \
    THUNK  (finishInit,      void,                            (TimePoint))                                      \
    HOOK_IF(wakeReset,       void,                            (),                     NeedsWake,      void())   \
    HOOK   (redirtyItem,     void,                            ())                                               \
    HOOK   (unserve,         void,                            ())                                               \
    HOOK   (forgetSamples,   void,                            ())                                               \
    HOOK   (setupFinal,      bool,                            ())                                               \
    HOOK   (startupSchedule, void,                            (TimePoint))                                      \
    HOOK_IF(wakeRetry,       bool,                            (TimePoint),            NeedsWake,      false)    \
    CONST  (startupDelayMs,  std::uint32_t)                                                                     \
    CONST  (resetLowMs,      std::uint32_t)                                                                     \
    CONST  (resetSettleMs,   std::uint32_t)                                                                     \
    CONST  (reads,           std::span<ReadGroupInfo const>)                                                    \
    HOOK   (readSlot,        ReadSlotBaseT&,                  (std::uint8_t))                                   \
    HOOK   (periodNow,       std::chrono::milliseconds,       (std::uint8_t))                                   \
    HOOK_IF(dynamicPeriod,   std::chrono::milliseconds,       (std::uint8_t),         NeedsDynamicPeriod,       \
                                                                                      std::chrono::milliseconds{}) \
    HOOK   (prepareRead,     void,                            (std::uint8_t))                                   \
    HOOK   (decode,          DecodeResult,                    (std::uint8_t))                                   \
    CONST  (writes,          std::span<WriteGroupInfo const>)                                                   \
    HOOK   (writeSlot,       WriteSlotBase<Clock>&,           (std::uint8_t))                                   \
    HOOK   (encodeWrite,     void,                            (std::uint8_t, std::uint8_t))                     \
    HOOK   (afterWrite,      void,                            (std::uint8_t, std::uint8_t, TimePoint))          \
    HOOK   (verifySlot,      VerifiedWriteSlotBase<Clock>&,   (std::uint8_t))                                   \
    HOOK   (encodeForVerify, VerifyRead,                      (std::uint8_t, std::uint32_t))                    \
    HOOK   (verifyMatches,   bool,                            (std::uint8_t))                                   \
    HOOK   (applyInitial,    void,                            (std::uint8_t, std::uint32_t))                    \
    CONST  (name,            std::string_view)                                                                  \
    CONST  (registerBytes,   std::uint8_t)                                                                      \
    CONST  (initEmpty,       bool)
// clang-format on

/// The parameter list of a hook: the State first, then the row's arguments.
#define KVASIR_ENGINE_HOOK_PARAMS(...) State& __VA_OPT__(, ) __VA_ARGS__

/// A row that generates nothing, for the expansions that take only some kinds.
#define KVASIR_ENGINE_HOOK_SKIP(...)
