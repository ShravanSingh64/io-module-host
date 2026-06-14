#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "iomh/crc.hpp"
#include "iomh/frame_parser.hpp"

namespace {

using namespace iomh;

// Appends the header CRC (over bytes 0..5) to a 6-byte header prefix.
void finalize_header(std::vector<std::uint8_t>& f) {
    f.push_back(crc8_smbus(std::span<const std::uint8_t>(f.data(), 6)));
}

// Builds a well-formed DigitalIO Inputs frame (16 bytes) with correct CRCs.
std::vector<std::uint8_t> build_digital_io_frame(
    std::uint8_t src, std::uint8_t seq, std::array<std::uint8_t, 4> din) {
    std::vector<std::uint8_t> f{
        kStartToken,
        kHostAddress,
        src,
        seq,
        static_cast<std::uint8_t>(Sap::Inputs),
        kDigitalIoPayloadLength};
    finalize_header(f);

    std::vector<std::uint8_t> payload{
        static_cast<std::uint8_t>(DeviceType::DigitalIo),
        0x01, 0x02, 0x03,                       // hw rev, fw major, fw minor
        din[0], din[1], din[2], din[3]};        // digital inputs 0..3
    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back(crc8_smbus(payload));           // payload CRC over bytes 7..14
    return f;                                    // 16 bytes total
}

// Builds a well-formed Push Button Inputs frame (14 bytes) with correct CRCs.
std::vector<std::uint8_t> build_push_button_frame(
    std::uint8_t src, std::uint8_t seq, std::uint8_t black, std::uint8_t white) {
    std::vector<std::uint8_t> f{
        kStartToken,
        kHostAddress,
        src,
        seq,
        static_cast<std::uint8_t>(Sap::Inputs),
        kPushButtonPayloadLength};
    finalize_header(f);

    std::vector<std::uint8_t> payload{
        static_cast<std::uint8_t>(DeviceType::PushButton),
        0x01, 0x02, 0x03,                       // hw rev, fw major, fw minor
        black, white};                          // black + white button states
    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back(crc8_smbus(payload));           // payload CRC over bytes 7..12
    return f;                                    // 14 bytes total
}

}  // namespace

TEST_CASE("happy path: DigitalIO frame parses to typed payload") {
    auto bytes = build_digital_io_frame(0x01, 0x42, {1, 0, 1, 0});
    FrameParser p;
    p.feed(bytes);

    auto r = p.next();
    REQUIRE(r.has_value());          // got an event
    REQUIRE(r->has_value());         // it's a valid frame, not an error
    const auto& frame = r->value();
    CHECK(frame.header.sequence == 0x42);
    CHECK(frame.header.source == 0x01);

    const auto* dio = std::get_if<DigitalIoInputs>(&frame.payload);
    REQUIRE(dio != nullptr);
    CHECK(dio->digital_in[0] == 1);
    CHECK(dio->digital_in[1] == 0);
    CHECK(dio->digital_in[2] == 1);
    CHECK(dio->digital_in[3] == 0);

    CHECK_FALSE(p.next().has_value());  // buffer drained
}

TEST_CASE("happy path: Push Button frame parses to typed payload") {
    auto bytes = build_push_button_frame(0x02, 0x07, /*black=*/1, /*white=*/0);
    FrameParser p;
    p.feed(bytes);

    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE(r->has_value());
    const auto& frame = r->value();
    CHECK(frame.header.sequence == 0x07);
    CHECK(frame.header.source == 0x02);

    const auto* btn = std::get_if<PushButtonInputs>(&frame.payload);
    REQUIRE(btn != nullptr);
    CHECK(btn->black_button == 1);
    CHECK(btn->white_button == 0);
}

TEST_CASE("header CRC corruption: false start token, parser recovers next frame") {
    auto bad = build_digital_io_frame(0x01, 0x11, {0, 0, 0, 0});
    bad[6] ^= 0xFF;  // corrupt the header CRC byte

    auto good = build_digital_io_frame(0x01, 0x22, {1, 1, 1, 1});

    std::vector<std::uint8_t> stream;
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), good.begin(), good.end());

    FrameParser p;
    p.feed(stream);

    // The bad frame's 0xA5 is a false start token; the parser advances and
    // eventually locks onto the good frame. The bad header is never surfaced
    // as an event — it is silently resynced past.
    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE(r->has_value());
    CHECK(r->value().header.sequence == 0x22);  // recovered the good frame
}

TEST_CASE("payload CRC corruption is reported, not crashed") {
    auto bytes = build_digital_io_frame(0x01, 0x10, {0, 0, 0, 0});
    bytes[11] ^= 0xFF;  // flip a payload byte -> payload CRC now mismatches

    FrameParser p;
    p.feed(bytes);

    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE_FALSE(r->has_value());
    CHECK(r->error() == ParseError::PayloadCrcMismatch);
}

TEST_CASE("resync past garbage and a false 0xA5") {
    auto frame = build_digital_io_frame(0x01, 0x07, {1, 1, 0, 0});
    std::vector<std::uint8_t> stream{0xDE, 0xAD, kStartToken, 0xBE, 0xEF};
    stream.insert(stream.end(), frame.begin(), frame.end());

    FrameParser p;
    p.feed(stream);

    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE(r->has_value());
    CHECK(r->value().header.sequence == 0x07);  // recovered the real frame
}

TEST_CASE("truncated frame waits, then completes when bytes arrive") {
    auto bytes = build_digital_io_frame(0x01, 0x55, {0, 1, 0, 1});

    FrameParser p;
    p.feed(std::span<const std::uint8_t>(bytes.data(), 9));  // partial
    CHECK_FALSE(p.next().has_value());  // not enough yet, no hang

    p.feed(std::span<const std::uint8_t>(bytes.data() + 9, bytes.size() - 9));
    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE(r->has_value());
    CHECK(r->value().header.sequence == 0x55);
}

TEST_CASE("unknown SAP: header valid but frame type unrecognized") {
    // Build a header-valid frame whose SAP byte is not a recognized type.
    std::vector<std::uint8_t> f{
        kStartToken,
        kHostAddress,
        0x01,
        0x33,
        0x99,                        // bogus SAP (neither Heartbeat nor Inputs)
        0x00};                       // zero payload length
    finalize_header(f);             // header CRC is correct, so resync won't skip it
    f.push_back(0x00);              // trailing byte (would-be payload CRC slot)

    FrameParser p;
    p.feed(f);

    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE_FALSE(r->has_value());
    CHECK(r->error() == ParseError::UnknownSap);
}

TEST_CASE("length mismatch: DigitalIO device type with wrong payload length") {
    // device_type = DigitalIo (expects length 8) but header declares length 6.
    std::vector<std::uint8_t> f{
        kStartToken,
        kHostAddress,
        0x01,
        0x44,
        static_cast<std::uint8_t>(Sap::Inputs),
        kPushButtonPayloadLength};   // wrong length (6) for a DigitalIo payload
    finalize_header(f);

    std::vector<std::uint8_t> payload{
        static_cast<std::uint8_t>(DeviceType::DigitalIo),
        0x01, 0x02, 0x03, 0x00, 0x00};  // 6 payload bytes
    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back(crc8_smbus(payload));   // valid payload CRC over the 6 bytes

    FrameParser p;
    p.feed(f);

    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE_FALSE(r->has_value());
    CHECK(r->error() == ParseError::LengthMismatch);
}