#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace iomh {
namespace detail {

// CRC-8/SMBus: poly 0x07, init 0x00, no reflection, final XOR 0x00.
// MSB-first table generation at compile time.
consteval std::array<std::uint8_t, 256> make_crc8_smbus_table() {
    std::array<std::uint8_t, 256> table{};
    for (int i = 0; i < 256; ++i) {
        auto crc = static_cast<std::uint8_t>(i);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<std::uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<std::uint8_t>(crc << 1);
        }
        table[static_cast<std::size_t>(i)] = crc;
    }
    return table;
}

inline constexpr auto kCrc8SmbusTable = make_crc8_smbus_table();

}  // namespace detail

[[nodiscard]] constexpr std::uint8_t crc8_smbus(
    std::span<const std::uint8_t> data) noexcept {
    std::uint8_t crc = 0x00;
    for (std::uint8_t b : data) {
        crc = detail::kCrc8SmbusTable[static_cast<std::size_t>(crc ^ b)];
    }
    return crc;
}

}  // namespace iomh