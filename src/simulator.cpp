#include "iomh/simulator.hpp"

#include <array>

#include "iomh/crc.hpp"

namespace iomh {
namespace {

void append_header_crc(std::vector<std::uint8_t>& f) {
    f.push_back(crc8_smbus(std::span<const std::uint8_t>(f.data(), 6)));
}

}  // namespace

std::vector<std::uint8_t> SimulatedDevice::build_response(
    std::uint8_t sequence) const {
    std::vector<std::uint8_t> f;

    const std::uint8_t payload_len =
        (type_ == DeviceType::DigitalIo) ? kDigitalIoPayloadLength
                                         : kPushButtonPayloadLength;

    // ---- Header (bytes 0..5) ----
    f.push_back(kStartToken);
    f.push_back(kHostAddress);                              // destination: host
    f.push_back(address_);                                  // source: this device
    f.push_back(sequence);                                  // echo sequence
    f.push_back(static_cast<std::uint8_t>(Sap::Inputs));
    f.push_back(payload_len);
    append_header_crc(f);                                   // byte 6

    // ---- Payload (starts at byte 7 with device type) ----
    std::vector<std::uint8_t> payload;
    payload.push_back(static_cast<std::uint8_t>(type_));
    payload.push_back(0x01);  // hardware revision
    payload.push_back(0x02);  // firmware major
    payload.push_back(0x03);  // firmware minor

    if (type_ == DeviceType::DigitalIo) {
        payload.push_back(digital_in_[0]);
        payload.push_back(digital_in_[1]);
        payload.push_back(digital_in_[2]);
        payload.push_back(digital_in_[3]);
    } else {
        payload.push_back(black_);
        payload.push_back(white_);
    }

    const std::uint8_t payload_crc = crc8_smbus(payload);

    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back(payload_crc);

    // ---- Apply corruption mode (for exercising the host's error path) ----
    switch (corrupt_) {
        case CorruptMode::None:
            break;
        case CorruptMode::BadHeaderCrc:
            f[6] ^= 0xFF;
            break;
        case CorruptMode::BadPayloadCrc:
            f.back() ^= 0xFF;  // corrupt the trailing payload CRC
            break;
        case CorruptMode::Truncate:
            if (f.size() > 4) {
                f.resize(f.size() - 4);  // drop the tail mid-frame
            }
            break;
    }

    return f;
}

std::size_t Simulator::service() {
    // Pull whatever the host has sent into our working buffer.
    std::array<std::uint8_t, 64> chunk{};
    std::size_t n = 0;
    while ((n = link_.read(std::span<std::uint8_t>(chunk.data(), chunk.size()))) > 0) {
        rx_buffer_.insert(rx_buffer_.end(), chunk.begin(), chunk.begin() + n);
    }

    std::size_t serviced = 0;
    std::size_t pos = 0;

    // A heartbeat is exactly 8 bytes: 7-byte header + 1-byte payload CRC.
    constexpr std::size_t kHeartbeatSize = kHeaderSize + 1;

    while (pos + kHeartbeatSize <= rx_buffer_.size()) {
        const std::uint8_t* h = rx_buffer_.data() + pos;

        // Resync to a start token (mirror of the host's robustness).
        if (h[0] != kStartToken) {
            ++pos;
            continue;
        }

        // Validate the heartbeat header CRC before acting on it.
        const std::uint8_t hdr_crc =
            crc8_smbus(std::span<const std::uint8_t>(h, 6));
        if (hdr_crc != h[6]) {
            ++pos;  // false start token; advance one byte
            continue;
        }

        // Only act on Heartbeat frames addressed to a device we host.
        const std::uint8_t destination = h[1];
        const std::uint8_t sequence    = h[3];
        const auto sap = sap_from_byte(h[4]);

        if (sap == Sap::Heartbeat) {
            if (SimulatedDevice* dev = device_at(destination)) {
                auto resp = dev->build_response(sequence);
                link_.write(resp);
                ++serviced;
            }
            // Addressed to an absent device -> ignore (no response).
        }

        pos += kHeartbeatSize;
    }

    // Drop consumed bytes; keep any partial trailing heartbeat.
    rx_buffer_.erase(rx_buffer_.begin(),
                     rx_buffer_.begin() + static_cast<std::ptrdiff_t>(pos));
    return serviced;
}

}  // namespace iomh