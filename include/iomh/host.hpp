#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <vector>

#include "iomh/frame_parser.hpp"
#include "iomh/protocol.hpp"
#include "iomh/serial_link.hpp"

namespace iomh {

// Outcome of a poll: a validated Inputs frame or a reason it failed. Extends
// ParseError with host-level conversation errors the parser cannot detect
// (the parser has no notion of "the sequence I sent").
enum class PollError {
    NoResponse,         // bus stayed silent within the read budget
    SequenceMismatch,   // response echoed the wrong sequence (stale/misrouted)
    HeaderCrcMismatch,
    PayloadCrcMismatch,
    LengthMismatch,
    UnknownSap,
    UnknownDeviceType,
};

// Polls a single client over an ISerialLink. Send and receive are separate
// operations: a real bus (or a concurrent/socket-backed simulator) responds
// autonomously between the two, so the host sends, then later reads. Owns
// the host's sequence counter.
class Host {
public:
    explicit Host(ISerialLink& link) : link_(link) {}

    // Build and transmit a Heartbeat to `client_address`. Returns the
    // sequence number used, so the caller can match the eventual response.
    std::uint8_t send_heartbeat(std::uint8_t client_address);

    // Read and parse one Inputs response to the heartbeat that used
    // `expected_sequence`. max_read_attempts bounds the wait, turning a
    // stream that stops mid-frame into NoResponse rather than a hang.
    [[nodiscard]] std::expected<InputsFrame, PollError> read_response(
        std::uint8_t expected_sequence, int max_read_attempts = 16);

private:
    [[nodiscard]] std::vector<std::uint8_t> build_heartbeat(
        std::uint8_t client_address, std::uint8_t sequence) const;

    ISerialLink& link_;
    std::uint8_t sequence_ = 0;  // wraps 0xFF -> 0x00 automatically (uint8_t)
};

[[nodiscard]] std::string to_string(PollError e);

}  // namespace iomh