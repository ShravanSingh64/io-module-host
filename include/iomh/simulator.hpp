#pragma once

#include <cstdint>
#include <vector>

#include "iomh/protocol.hpp"
#include "iomh/serial_link.hpp"

namespace iomh {

// How the simulator should corrupt its next response, for exercising the
// host's error path. Modeled as an enum (not a bool) so one switch covers
// several corruption modes for cheap test coverage.
enum class CorruptMode {
    None,
    BadHeaderCrc,
    BadPayloadCrc,
    Truncate,       // drop the trailing bytes of the response
};

// Emulates one device on the bus at a fixed address. Responds to a Heartbeat
// addressed to itself with the appropriate Inputs frame; ignores heartbeats
// for other addresses. A bus of devices is just several of these sharing a
// link (see Simulator below).
class SimulatedDevice {
public:
    SimulatedDevice(std::uint8_t address, DeviceType type)
        : address_(address), type_(type) {}

    [[nodiscard]] std::uint8_t address() const noexcept { return address_; }

    void set_corrupt_mode(CorruptMode m) noexcept { corrupt_ = m; }

    // Digital IO state (only meaningful for a DigitalIo device).
    void set_digital_inputs(std::array<std::uint8_t, 4> in) noexcept {
        digital_in_ = in;
    }
    // Push button state (only meaningful for a PushButton device).
    void set_buttons(std::uint8_t black, std::uint8_t white) noexcept {
        black_ = black;
        white_ = white;
    }

    // Build the Inputs response for a heartbeat carrying `sequence`. The
    // sequence is echoed back unchanged, per the protocol.
    [[nodiscard]] std::vector<std::uint8_t> build_response(
        std::uint8_t sequence) const;

private:
    std::uint8_t address_;
    DeviceType   type_;
    CorruptMode  corrupt_ = CorruptMode::None;

    std::array<std::uint8_t, 4> digital_in_{};
    std::uint8_t black_ = 0;
    std::uint8_t white_ = 0;
};

// A bus of devices behind a single link. Reads heartbeats from the link,
// routes each to the addressed device, and writes that device's response
// back. Heartbeats addressed to an absent device are ignored.
class Simulator {
public:
    explicit Simulator(ISerialLink& link) : link_(link) {}

    void add_device(SimulatedDevice device) {
        devices_.push_back(std::move(device));
    }

    SimulatedDevice* device_at(std::uint8_t address) {
        for (auto& d : devices_) {
            if (d.address() == address) return &d;
        }
        return nullptr;
    }

    // Drain any pending heartbeats from the link and respond to each.
    // Returns the number of heartbeats serviced.
    std::size_t service();

private:
    ISerialLink&                  link_;
    std::vector<SimulatedDevice>  devices_;
    std::vector<std::uint8_t>     rx_buffer_;
};

}  // namespace iomh