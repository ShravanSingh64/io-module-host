#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <array>
#include <cstdint>

#include "iomh/crc.hpp"

namespace {
constexpr std::array<std::uint8_t, 9> kCheckData{
    0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};  // "123456789"

// Compile-time proof: spec check value is 0xF4.
static_assert(iomh::crc8_smbus(kCheckData) == 0xF4,
              "CRC-8/SMBus check value mismatch");
}  // namespace

TEST_CASE("CRC-8/SMBus check value") {
    CHECK(iomh::crc8_smbus(kCheckData) == 0xF4);
}

TEST_CASE("CRC over empty payload is zero") {
    // Why heartbeat payload CRC is 0x00: empty input, init 0x00, no final XOR.
    std::array<std::uint8_t, 0> empty{};
    CHECK(iomh::crc8_smbus(empty) == 0x00);
}