#include <iostream>
#include <variant>

#include "iomh/host.hpp"
#include "iomh/protocol.hpp"
#include "iomh/simulated_link.hpp"
#include "iomh/simulator.hpp"

namespace {

using namespace iomh;

void print_response(std::uint8_t address, const InputsFrame& frame) {
    std::cout << "  client 0x" << std::hex << static_cast<int>(address)
              << " (seq 0x" << static_cast<int>(frame.header.sequence)
              << std::dec << "): ";

    if (const auto* dio = std::get_if<DigitalIoInputs>(&frame.payload)) {
        std::cout << "DigitalIO  fw " << static_cast<int>(dio->fw_major) << "."
                  << static_cast<int>(dio->fw_minor) << "  inputs ["
                  << static_cast<int>(dio->digital_in[0]) << ","
                  << static_cast<int>(dio->digital_in[1]) << ","
                  << static_cast<int>(dio->digital_in[2]) << ","
                  << static_cast<int>(dio->digital_in[3]) << "]\n";
    } else if (const auto* btn = std::get_if<PushButtonInputs>(&frame.payload)) {
        std::cout << "PushButton fw " << static_cast<int>(btn->fw_major) << "."
                  << static_cast<int>(btn->fw_minor) << "  black="
                  << static_cast<int>(btn->black_button) << " white="
                  << static_cast<int>(btn->white_button) << "\n";
    }
}

// One full half-duplex exchange against the in-memory simulator:
//   1. host sends heartbeat   2. simulator reacts (stands in for an
//   autonomous device)        3. host reads the response.
void exchange(Host& host, Simulator& sim, std::uint8_t address) {
    const std::uint8_t seq = host.send_heartbeat(address);  // 1
    sim.service();                                          // 2
    auto result = host.read_response(seq);                 // 3

    if (result) {
        print_response(address, *result);
    } else {
        std::cout << "  client 0x" << std::hex << static_cast<int>(address)
                  << std::dec << ": " << to_string(result.error()) << "\n";
    }
}

}  // namespace

int main() {
    using namespace iomh;

    SimulatedLink link;
    Host host(link);
    Simulator sim(link.peer());

    // Bus: Digital IO module at 0x01, Push Button at 0x02.
    SimulatedDevice dio(0x01, DeviceType::DigitalIo);
    dio.set_digital_inputs({1, 0, 1, 1});
    sim.add_device(dio);

    SimulatedDevice btn(0x02, DeviceType::PushButton);
    btn.set_buttons(/*black=*/1, /*white=*/0);
    sim.add_device(btn);

    std::cout << "== Valid responses ==\n";
    exchange(host, sim, 0x01);
    exchange(host, sim, 0x02);

    std::cout << "\n== Corrupted payload CRC (error path) ==\n";
    sim.device_at(0x01)->set_corrupt_mode(CorruptMode::BadPayloadCrc);
    exchange(host, sim, 0x01);
    sim.device_at(0x01)->set_corrupt_mode(CorruptMode::None);

    std::cout << "\n== Truncated response (error path) ==\n";
    sim.device_at(0x02)->set_corrupt_mode(CorruptMode::Truncate);
    exchange(host, sim, 0x02);
    sim.device_at(0x02)->set_corrupt_mode(CorruptMode::None);

    std::cout << "\n== Heartbeat to absent address 0x05 (ignored) ==\n";
    {
        const std::uint8_t seq = host.send_heartbeat(0x05);
        sim.service();  // no device at 0x05 -> no response queued
        auto r = host.read_response(seq, /*max_read_attempts=*/4);
        std::cout << "  client 0x05: "
                  << (r ? "unexpected response" : to_string(r.error())) << "\n";
    }

    return 0;
}