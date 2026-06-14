#pragma once

#include <cstdint>
#include <optional>
#include <variant>

namespace iomh {

// ---- Frame-layout constants (individual wire-format singletons) ----
inline constexpr std::uint8_t kStartToken  = 0xA5;
inline constexpr std::uint8_t kHostAddress = 0x00;
inline constexpr std::size_t  kHeaderSize  = 7;   // bytes 0..6 incl. header CRC

inline constexpr std::uint8_t kDigitalIoPayloadLength  = 0x08;
inline constexpr std::uint8_t kPushButtonPayloadLength = 0x06;

// ---- Client address range (a RANGE, not a named set) ----
// Adding more clients adds zero constants — they're addresses within this
// range. A specific device's address is configuration/data, not a protocol
// constant.
inline constexpr std::uint8_t kClientAddrMin = 0x01;
inline constexpr std::uint8_t kClientAddrMax = 0x1F;

[[nodiscard]] constexpr bool is_valid_client_address(std::uint8_t a) noexcept {
    return a >= kClientAddrMin && a <= kClientAddrMax;
}

// ---- SAP: closed, named set of frame types -> scoped enum ----
// Underlying type pinned to uint8_t so it marshals to/from the wire cleanly.
// Extending the protocol = add an enum value; -Wswitch then flags every
// switch that hasn't handled it.
enum class Sap : std::uint8_t {
    Heartbeat = 0xC1,  // host -> client
    Inputs    = 0xC2,  // client -> host
};

enum class DeviceType : std::uint8_t {
    DigitalIo  = 0x00,
    PushButton = 0x01,
};

// ---- Boundary conversions: raw wire byte -> validated enum ----
// A byte off the wire can be ANY value (corrupt/unknown frames included),
// so we never blindly static_cast. nullopt => unrecognized -> a parse error.
[[nodiscard]] constexpr std::optional<Sap>
sap_from_byte(std::uint8_t b) noexcept {
    switch (b) {
        case static_cast<std::uint8_t>(Sap::Heartbeat): return Sap::Heartbeat;
        case static_cast<std::uint8_t>(Sap::Inputs):    return Sap::Inputs;
        default:                                        return std::nullopt;
    }
}

[[nodiscard]] constexpr std::optional<DeviceType>
device_type_from_byte(std::uint8_t b) noexcept {
    switch (b) {
        case static_cast<std::uint8_t>(DeviceType::DigitalIo):
            return DeviceType::DigitalIo;
        case static_cast<std::uint8_t>(DeviceType::PushButton):
            return DeviceType::PushButton;
        default:
            return std::nullopt;
    }
}

// ---- Header (logical model; marshaled field-by-field, never memcpy'd) ----
struct FrameHeader {
    std::uint8_t start_token{};
    std::uint8_t destination{};
    std::uint8_t source{};
    std::uint8_t sequence{};
    std::uint8_t sap{};            // raw byte; validated via sap_from_byte
    std::uint8_t payload_length{};
    std::uint8_t header_crc{};
};

// ---- Typed Inputs payloads ----
struct DigitalIoInputs {
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

// Device-type byte (frame byte 7) selects the variant. The parser cannot
// know which arm applies until it has read that byte — exactly the spec's
// constraint.
using InputsPayload = std::variant<DigitalIoInputs, PushButtonInputs>;

// A fully parsed, validated Inputs frame: header + typed payload.
struct InputsFrame {
    FrameHeader   header{};
    InputsPayload payload{};
};

// ---- Parse error taxonomy ----
enum class ParseError {
    HeaderCrcMismatch,
    PayloadCrcMismatch,
    LengthMismatch,       // payload_length disagrees with device type
    UnknownSap,           // SAP byte not a recognized frame type
    UnknownDeviceType,    // device-type byte not recognized
    SequenceMismatch,     // response sequence != the sequence the host sent
    Truncated,            // stream ended mid-frame (surfaced as nullopt by parser)
};

}  // namespace iomh