#pragma once

#include <cstdint>
#include <variant>

namespace iomh {

// ---- Protocol constants ----
inline constexpr std::uint8_t kStartToken     = 0xA5;
inline constexpr std::uint8_t kHostAddress    = 0x00;
inline constexpr std::uint8_t kSapHeartbeat   = 0xC1;
inline constexpr std::uint8_t kSapInputs      = 0xC2;
inline constexpr std::size_t  kHeaderSize     = 7;   // bytes 0-6 incl. header CRC

inline constexpr std::uint8_t kDeviceTypeDio        = 0x00;
inline constexpr std::uint8_t kDeviceTypePushButton = 0x01;

inline constexpr std::uint8_t kDioPayloadLength        = 0x08;
inline constexpr std::uint8_t kPushButtonPayloadLength = 0x06;

// ---- Header (bytes 0-6) ----
// Note: this is a logical model, not a wire-layout struct. We marshal
// field-by-field rather than memcpy a packed struct, to avoid any UB
// around alignment/padding/strict-aliasing.
struct FrameHeader {
    std::uint8_t start_token{};    // byte 0, always 0xA5
    std::uint8_t destination{};    // byte 1
    std::uint8_t source{};         // byte 2
    std::uint8_t sequence{};       // byte 3
    std::uint8_t sap{};            // byte 4
    std::uint8_t payload_length{}; // byte 5
    std::uint8_t header_crc{};     // byte 6
};

// ---- Typed Inputs payloads ----
struct DioInputs {
    std::uint8_t hw_revision{};
    std::uint8_t fw_major{};
    std::uint8_t fw_minor{};
    std::uint8_t digital_in[4]{};  // each 0x00 (low) or 0x01 (high)
};

struct PushButtonInputs {
    std::uint8_t hw_revision{};
    std::uint8_t fw_major{};
    std::uint8_t fw_minor{};
    std::uint8_t black_button{};   // 0x00 released / 0x01 pressed
    std::uint8_t white_button{};
};

// Device-type byte (7) selects the variant. The parser cannot know which
// arm applies until it has read that byte — exactly the spec's point.
using InputsPayload = std::variant<DioInputs, PushButtonInputs>;

// A fully parsed, validated Inputs frame: header + typed payload.
struct InputsFrame {
    FrameHeader   header{};
    InputsPayload payload{};
};

// ---- Parse error taxonomy ----
enum class ParseError {
    BadStartToken,        // resync case (handled inside parser, rarely surfaced)
    HeaderCrcMismatch,
    PayloadCrcMismatch,
    LengthMismatch,       // payload_length disagrees with device_type
    UnknownDeviceType,
    Truncated,            // stream ended mid-frame
};

}  // namespace iomh