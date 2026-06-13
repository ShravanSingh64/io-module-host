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

// Builds a well-formed DIO Inputs frame (16 bytes) with correct CRCs.
std::vector<std::uint8_t> build_dio_frame(std::uint8_t src, std::uint8_t seq,
                                          std::array<std::uint8_t, 4> din) {
    std::vector<std::uint8_t> f{kStartToken, kHostAddress, src, seq,
                                kSapInputs,  kDioPayloadLength};
    f.push_back(crc8_smbus(std::span<const std::uint8_t>(f.data(), 6)));

    std::vector<std::uint8_t> payload{kDeviceTypeDio, 0x01, 0x02, 0x03,
                                      din[0], din[1], din[2], din[3]};
    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back(crc8_smbus(payload));  // payload CRC over bytes 7..14
    return f;  // 16 bytes
}

}  // namespace

TEST_CASE("happy path: DIO frame parses to typed payload") {
    auto bytes = build_dio_frame(0x01, 0x42, {1, 0, 1, 0});
    FrameParser p;
    p.feed(bytes);

    auto r = p.next();
    REQUIRE(r.has_value());          // got an event
    REQUIRE(r->has_value());         // it's a valid frame, not an error
    const auto& frame = r->value();
    CHECK(frame.header.sequence == 0x42);
    CHECK(frame.header.source == 0x01);

    const auto* dio = std::get_if<DioInputs>(&frame.payload);
    REQUIRE(dio != nullptr);
    CHECK(dio->digital_in[0] == 1);
    CHECK(dio->digital_in[3] == 0);

    CHECK_FALSE(p.next().has_value());  // buffer drained
}

TEST_CASE("payload CRC corruption is reported, not crashed") {
    auto bytes = build_dio_frame(0x01, 0x10, {0, 0, 0, 0});
    bytes[11] ^= 0xFF;  // flip a payload byte -> payload CRC now mismatches

    FrameParser p;
    p.feed(bytes);

    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE_FALSE(r->has_value());
    CHECK(r->error() == ParseError::PayloadCrcMismatch);
}

TEST_CASE("resync past garbage and a false 0xA5") {
    auto frame = build_dio_frame(0x01, 0x07, {1, 1, 0, 0});
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
    auto bytes = build_dio_frame(0x01, 0x55, {0, 1, 0, 1});

    FrameParser p;
    p.feed(std::span<const std::uint8_t>(bytes.data(), 9));  // partial
    CHECK_FALSE(p.next().has_value());  // not enough yet, no hang

    p.feed(std::span<const std::uint8_t>(bytes.data() + 9, bytes.size() - 9));
    auto r = p.next();
    REQUIRE(r.has_value());
    REQUIRE(r->has_value());
    CHECK(r->value().header.sequence == 0x55);
}