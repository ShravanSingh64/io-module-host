# io-module-host

A C++23 host that polls IO modules over a simulated half-duplex serial link,
parses the binary response frames, and handles error conditions (corruption,
truncation, desync) without crashing or hanging. Includes a simulator
emulating a Digital IO module (0x01) and a Push Button (0x02).

## Build & Run

Developed on **WSL2, Ubuntu 24.04.1 LTS**. The default toolchain didn't include
a C++23-capable GCC, so I installed and built with **GCC 12** (`g++-12`), which
provides the `std::expected` and `std::span` used here. Any compiler with
`<expected>` support (GCC 12+ / Clang 16+) should work.

```bash
mkdir build && cd build
cmake -DCMAKE_CXX_COMPILER=g++-12 .. && cmake --build . && ./bin/host && ctest --output-on-failure
```

This configures with GCC 12, builds the host/simulator/tests, runs the demo,
and runs the unit tests. The demo polls both devices, then deliberately drives
the error paths (corrupted payload CRC, truncated response) and an unaddressed
device, so the output exercises the robustness requirements in one run.

## Architecture

Three strictly separated layers, connected only through interfaces:

- **Transport** (`ISerialLink`) — a minimal read/write byte interface.
  `SimulatedLink` is an in-memory, half-duplex implementation (two directional
  queues). A real serial port would drop in behind the same interface without
  touching host or parser code.
- **Parsing** (`FrameParser`) — pure: bytes in, typed frames or errors out, no
  I/O. Push-based (`feed` / `next`), which is what makes truncation safe.
- **Orchestration** (`Host`, `Simulator`) — opposite ends of the link. The host
  owns the conversation (sequence counter, request/response matching); the
  simulator emulates a bus of reactive devices.

## Repository structure

The work is split across feature branches to keep each a reviewable unit: the
protocol layer (CRC, frame types, parser), the simulator and host, and the Push
Button bonus isolated on its own branch — partly so the bonus diff is itself
evidence of how cleanly the design extends to a new device. This structure is
CI-ready: a build-and-test workflow would gate each branch naturally.

---

## 1. Frame modeling

Frames are modeled as **typed structs**, never raw byte buffers. The common
7-byte header is a `FrameHeader` struct; each Inputs payload is its own struct
(`DigitalIoInputs`, `PushButtonInputs`). A parsed Inputs frame carries its
payload as `std::variant<DigitalIoInputs, PushButtonInputs>`.

The `std::variant` choice is driven directly by the protocol: the device type
is byte 7 of the payload, so the parser **cannot** know which variant applies
until it has read and CRC-validated the payload. A variant models exactly that
"resolved-after-the-fact" nature. Inheritance was rejected — there's no
behavioral polymorphism here, just a closed set of data shapes, which is the
textbook case for a variant over a class hierarchy.

SAP (frame type) and device type are modeled as scoped enums
(`enum class : std::uint8_t`) rather than loose constants. This gives type
safety at the boundaries and, crucially, **exhaustive-switch warnings**: adding
a new frame type or device means adding an enum value, and `-Wswitch` then flags
every dispatch site that hasn't handled it. Client addresses, by contrast, are a
*range* (0x01–0x1F), so they're validated with a predicate, not enumerated — a
specific device's address is configuration, not a protocol constant.

Bytes are marshaled to/from the wire **field-by-field**, never via `memcpy` or
`reinterpret_cast` over a packed struct, to avoid alignment/aliasing UB.

**Extending with a new device** is the four-step diff the Push Button bonus
already demonstrates: add a payload struct, add a variant arm, add an enum value,
add one case to the parser's device dispatch. The generic validation layer
(start token, header CRC, length, payload CRC) is device-agnostic and is never
touched. A new *frame type* (e.g. Outputs) is similar: an enum value plus an arm
in the SAP dispatch. I implemented an explicit dispatch seam for this rather than
a handler-registry framework — the explicit version is more legible for the next
engineer to extend, and a registry is a heavier architectural commitment that
belongs in a team discussion, not a take-home.

## 2. CRC strategy

CRC-8/SMBus is implemented as a **compile-time lookup table** (`consteval`
table generation into a `constexpr std::array<uint8_t, 256>`). The table holds
the result of the 8-round bit loop for every possible byte; at runtime each byte
costs one indexed lookup (`crc = table[crc ^ b]`) instead of eight shift/XOR
rounds — roughly an 8× speedup for 256 bytes of static storage and zero runtime
construction cost.

Correctness is enforced at **compile time**: a `static_assert` checks the
implementation against the spec's check value (CRC of `"123456789"` == `0xF4`).
A regression in the CRC fails the *build*, not a runtime test — the earliest
possible point to catch it, which matters because a wrong CRC corrupts every
frame silently.

The trade-off considered was table vs. bit-by-bit: bit-by-bit is marginally
simpler to read but runs the 8-round loop per byte. At this scale the table's
speed and the compile-time guarantee outweigh the simplicity, with negligible
memory cost.

## 3. Error recovery & resync

The parser is push-based and never blocks, which is the foundation for all three
error cases:

- **Starting mid-frame (bytes before the first 0xA5):** the parser scans forward
  one byte at a time to the next start token, discarding only the bytes *before*
  it — it never flushes the buffer behind a found 0xA5.

- **Corrupted CRC:** handled asymmetrically by design. A **header** CRC failure
  is treated as a false start token (a 0xA5 that appeared inside garbage or a
  payload) — the parser advances **exactly one byte** and resumes scanning. A
  **payload** CRC failure happens only after the header CRC has already validated
  the frame's boundaries, so the whole (length-valid) frame is discarded and a
  `PayloadCrcMismatch` is reported.

- **Stream stops mid-frame:** `next()` returns `nullopt` ("need more bytes") at
  either the header or payload boundary, leaving the partial frame buffered. When
  more bytes arrive it resumes and completes. The parser itself has no timeout —
  liveness is the host's concern: `Host::read_response` bounds the number of read
  attempts, so a stream that never completes surfaces as `NoResponse` rather than
  hanging.

**Avoiding permanent desync after a single bad byte** is the key invariant: a
header-CRC failure advances exactly *one* byte, never a guessed frame length (the
length field can't be trusted if the header is corrupt). So a single bad byte
costs at most one resync step, and a real frame sitting immediately behind a
false 0xA5 is always recovered. This is covered by a test that plants a 0xA5
inside garbage *before* a valid frame and asserts the valid frame is still parsed.

Note: a truncated response surfaces to the host as `NoResponse` rather than a
distinct "truncated" error — intentional, because on a real bus the host cannot
distinguish "device went silent" from "device sent a partial frame and stopped";
both are silence, both are handled by re-polling.

## 4. AI usage

I used Claude (Anthropic) substantially throughout, as a pair-programming and
design-review partner. Being specific about the division of labour, since that's
the point:

**What the AI did:** generated scaffolding and first drafts (the CRC table, the
parser state machine, the simulator and host), and acted as a sounding board for
design questions and for explaining C++23 features (`std::expected`, `std::span`,
`consteval` vs `constexpr`) I wanted to understand rather than just use.

**What I directed, corrected, or rejected:**
- **Architecture decisions were mine.** The `std::variant` modeling, the explicit
  SAP-dispatch seam over a handler registry (chosen for legibility and because a
  registry is a team-level decision), and the enum-vs-range distinction for SAP
  vs. addresses all came out of pushing on the design rather than accepting a
  first answer.
- **Rejected a TCP/UDP socket-backed simulator.** The AI and I discussed running
  the simulator as a separate process over sockets — architecturally closer to
  real hardware, but the spec explicitly asks for a *simple* read/write
  abstraction, and sockets add framing/connection complexity that isn't what's
  being evaluated. I kept the in-memory link and noted the socket path as future
  work instead. The `ISerialLink` seam means it could be added without disturbing
  the rest of the code.
- **Corrected a sequence-handling misconception.** I initially asked for a
  rollover guard on the sequence counter; the correct behavior is that a
  `uint8_t` wraps 0xFF→0x00 automatically, and the real check is the host
  verifying the client *echoed* the sequence it sent. That's where
  `SequenceMismatch` and the echo-check in `read_response` came from.
- **Split `Host` into send/receive.** An early draft fused sending the heartbeat
  and reading the response into one call, which double-sent against a reactive
  simulator. I separated `send_heartbeat` from `read_response` so the exchange
  models the real half-duplex flow (send → device reacts → read).
- I verified the CRC against the 0xF4 check value before building anything else,
  and reviewed all generated code rather than accepting it as-is.

In short: the AI accelerated the typing and explained the unfamiliar corners; the
architecture, the protocol-design reasoning, and the corrections were mine.