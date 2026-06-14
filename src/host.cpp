#include "iomh/host.hpp"

#include <array>

#include "iomh/crc.hpp"

namespace iomh {
namespace {

PollError from_parse_error(ParseError e) {
    switch (e) {
        case ParseError::HeaderCrcMismatch:  return PollError::HeaderCrcMismatch;
        case ParseError::PayloadCrcMismatch: return PollError::PayloadCrcMismatch;
        case ParseError::LengthMismatch:     return PollError::LengthMismatch;
        case ParseError::UnknownSap:         return PollError::UnknownSap;
        case ParseError::UnknownDeviceType:  return PollError::UnknownDeviceType;
        case ParseError::SequenceMismatch:   return PollError::SequenceMismatch;
        case ParseError::Truncated:          return PollError::NoResponse;
    }
    return PollError::NoResponse;  // unreachable; satisfies the compiler
}

}  // namespace

std::vector<std::uint8_t> Host::build_heartbeat(std::uint8_t client_address,
                                                std::uint8_t sequence) const {
    std::vector<std::uint8_t> f;
    f.push_back(kStartToken);
    f.push_back(client_address);                            // destination: client
    f.push_back(kHostAddress);                              // source: host
    f.push_back(sequence);
    f.push_back(static_cast<std::uint8_t>(Sap::Heartbeat));
    f.push_back(0x00);                                      // payload length: 0
    f.push_back(crc8_smbus(std::span<const std::uint8_t>(f.data(), 6)));  // hdr CRC
    f.push_back(0x00);                                      // payload CRC (no payload)
    return f;  // 8 bytes
}

std::uint8_t Host::send_heartbeat(std::uint8_t client_address) {
    const std::uint8_t seq = sequence_++;  // use, then advance (uint8_t wraps)
    auto heartbeat = build_heartbeat(client_address, seq);
    link_.write(heartbeat);
    return seq;
}

std::expected<InputsFrame, PollError> Host::read_response(
    std::uint8_t expected_sequence, int max_read_attempts) {
    FrameParser parser;
    std::array<std::uint8_t, 64> chunk{};

    for (int attempt = 0; attempt < max_read_attempts; ++attempt) {
        const std::size_t n =
            link_.read(std::span<std::uint8_t>(chunk.data(), chunk.size()));
        if (n > 0) {
            parser.feed(std::span<const std::uint8_t>(chunk.data(), n));
        }

        auto event = parser.next();
        if (!event) {
            continue;  // need more bytes; try another read attempt
        }
        if (!event->has_value()) {
            return std::unexpected(from_parse_error(event->error()));
        }

        // Structurally valid frame. Host-level check the parser cannot do:
        // did the client echo the sequence we sent?
        InputsFrame frame = std::move(event->value());
        if (frame.header.sequence != expected_sequence) {
            return std::unexpected(PollError::SequenceMismatch);
        }
        return frame;
    }

    return std::unexpected(PollError::NoResponse);  // budget exhausted: no hang
}

std::string to_string(PollError e) {
    switch (e) {
        case PollError::NoResponse:         return "no response (bus silent)";
        case PollError::SequenceMismatch:   return "sequence mismatch (stale/misrouted)";
        case PollError::HeaderCrcMismatch:  return "header CRC mismatch";
        case PollError::PayloadCrcMismatch: return "payload CRC mismatch";
        case PollError::LengthMismatch:     return "payload length / device type mismatch";
        case PollError::UnknownSap:         return "unknown SAP (frame type)";
        case PollError::UnknownDeviceType:  return "unknown device type";
    }
    return "unknown error";
}

}  // namespace iomh