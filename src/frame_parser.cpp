#include "iomh/frame_parser.hpp"

#include <utility>

#include "iomh/crc.hpp"

namespace iomh {
namespace {

// p points at the first payload byte (device_type, i.e. frame byte 7).
// Payload CRC has already validated these bytes, so they are trusted here.
std::expected<InputsPayload, ParseError>
build_payload(const std::uint8_t* p, std::uint8_t device_type_byte,
              std::uint8_t payload_len) {
    const auto dt = device_type_from_byte(device_type_byte);
    if (!dt) {
        return std::unexpected(ParseError::UnknownDeviceType);
    }

    switch (*dt) {
        case DeviceType::DigitalIo: {
            if (payload_len != kDigitalIoPayloadLength) {
                return std::unexpected(ParseError::LengthMismatch);
            }
            DigitalIoInputs in{};
            in.hw_revision   = p[1];
            in.fw_major      = p[2];
            in.fw_minor      = p[3];
            in.digital_in[0] = p[4];
            in.digital_in[1] = p[5];
            in.digital_in[2] = p[6];
            in.digital_in[3] = p[7];
            return InputsPayload{in};
        }
        case DeviceType::PushButton: {
            if (payload_len != kPushButtonPayloadLength) {
                return std::unexpected(ParseError::LengthMismatch);
            }
            PushButtonInputs in{};
            in.hw_revision  = p[1];
            in.fw_major     = p[2];
            in.fw_minor     = p[3];
            in.black_button = p[4];
            in.white_button = p[5];
            return InputsPayload{in};
        }
    }
    // Unreachable: device_type_from_byte already rejected unknown values.
    // Present to satisfy the compiler's control-flow analysis.
    return std::unexpected(ParseError::UnknownDeviceType);
}

}  // namespace

void FrameParser::feed(std::span<const std::uint8_t> bytes) {
    // Compact consumed bytes so the buffer never grows unbounded. After a
    // feed, pos_ is always 0; consumed garbage/frames are reclaimed here.
    if (pos_ > 0) {
        buffer_.erase(buffer_.begin(),
                      buffer_.begin() + static_cast<std::ptrdiff_t>(pos_));
        pos_ = 0;
    }
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
}

void FrameParser::reset() {
    buffer_.clear();
    pos_ = 0;
}

std::optional<FrameParser::Result> FrameParser::next() {
    for (;;) {
        // 1. Resync: scan byte-by-byte to a start token. We drop only the
        //    bytes *before* the 0xA5 — never flush data behind it.
        while (pos_ < buffer_.size() && buffer_[pos_] != kStartToken) {
            ++pos_;
        }
        const std::size_t avail = buffer_.size() - pos_;
        if (avail == 0) {
            return std::nullopt;  // buffer drained / pure garbage
        }

        // 2. A full header is needed before anything can be trusted.
        if (avail < kHeaderSize) {
            return std::nullopt;  // truncated mid-header: wait for more bytes
        }

        const std::uint8_t* f = buffer_.data() + pos_;

        // 3. Header CRC covers bytes 0..5; byte 6 is the CRC.
        const std::uint8_t hdr_crc =
            crc8_smbus(std::span<const std::uint8_t>(f, 6));
        if (hdr_crc != f[6]) {
            // INVARIANT: false start token -> advance exactly one byte.
            // A single bad byte can never cost more than one resync step.
            ++pos_;
            continue;
        }

        // 4. Header valid -> dispatch on SAP. Generic validation above is
        //    SAP-agnostic; SAP-specific handling begins here. Adding a frame
        //    type = a new value in the Sap enum + an arm in this dispatch.
        const auto sap = sap_from_byte(f[4]);
        if (!sap) {
            // Aligned, header-valid, unknown frame type. Consume the header
            // and surface it loudly rather than dropping silently — a
            // half-wired new SAP then fails informatively for the next dev.
            pos_ += kHeaderSize;
            return Result{std::unexpected(ParseError::UnknownSap)};
        }

        switch (*sap) {
            case Sap::Inputs:
                break;  // fall through to payload validation below
            case Sap::Heartbeat:
                // A *received* Heartbeat is not expected on the host side in
                // this assignment, but the seam exists: route it to its own
                // handler here. For now, consume and report as unexpected.
                pos_ += kHeaderSize;
                return Result{std::unexpected(ParseError::UnknownSap)};
        }

        // 5. Inputs frame: payload length is byte 5; full frame includes the
        //    trailing payload CRC.
        const std::uint8_t payload_len = f[5];
        const std::size_t total = kHeaderSize + payload_len + 1;  // + payload CRC
        if (avail < total) {
            return std::nullopt;  // truncated mid-payload: wait for more bytes
        }

        // 6. Payload CRC covers the payload_len bytes between the two CRCs.
        const std::uint8_t pay_crc = crc8_smbus(
            std::span<const std::uint8_t>(f + kHeaderSize, payload_len));
        if (pay_crc != f[kHeaderSize + payload_len]) {
            // Aligned frame, corrupt payload: header CRC validated the
            // boundaries, so discard the whole frame and report it.
            pos_ += total;
            return Result{std::unexpected(ParseError::PayloadCrcMismatch)};
        }

        // 7. Payload trusted. device_type is the first payload byte (byte 7).
        const std::uint8_t device_type_byte = f[kHeaderSize];
        auto payload =
            build_payload(f + kHeaderSize, device_type_byte, payload_len);
        pos_ += total;  // consume the frame regardless of payload-level outcome

        if (!payload) {
            return Result{std::unexpected(payload.error())};
        }

        InputsFrame frame{};
        frame.header.start_token    = f[0];
        frame.header.destination    = f[1];
        frame.header.source         = f[2];
        frame.header.sequence       = f[3];
        frame.header.sap            = f[4];
        frame.header.payload_length = f[5];
        frame.header.header_crc     = f[6];
        frame.payload               = std::move(*payload);
        return Result{std::move(frame)};
    }
}

}  // namespace iomh