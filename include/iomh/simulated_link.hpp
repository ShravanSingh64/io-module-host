#pragma once

#include <cstdint>
#include <deque>
#include <span>

#include "iomh/serial_link.hpp"

namespace iomh {

// In-memory, half-duplex link standing in for a real serial port. Two byte
// queues model the two directions of the bus. The host writes into the
// host->client queue and reads from the client->host queue; the simulator
// does the mirror. A real serial port would replace this behind ISerialLink
// without touching host or parser code.
//
// One endpoint is constructed; the simulator gets the "other side" view via
// peer(). read() is non-blocking: it returns 0 when nothing is queued, which
// is what lets the host treat silence as a truncation/timeout case rather
// than hanging.
class SimulatedLink final : public ISerialLink {
public:
    SimulatedLink() : tx_(&a_to_b_), rx_(&b_to_a_) {}

    std::size_t write(std::span<const std::uint8_t> data) override {
        tx_->insert(tx_->end(), data.begin(), data.end());
        return data.size();
    }

    std::size_t read(std::span<std::uint8_t> buf) override {
        std::size_t n = 0;
        while (n < buf.size() && !rx_->empty()) {
            buf[n++] = rx_->front();
            rx_->pop_front();
        }
        return n;  // 0 == nothing available right now (not an error)
    }

    // The simulator's view of the same bus, with tx/rx swapped.
    [[nodiscard]] ISerialLink& peer() noexcept { return peer_view_; }

private:
    // A lightweight second endpoint sharing the same two queues, directions
    // swapped. Defined as a nested type so both sides see the same buffers.
    class PeerView final : public ISerialLink {
    public:
        PeerView(std::deque<std::uint8_t>* tx, std::deque<std::uint8_t>* rx)
            : tx_(tx), rx_(rx) {}

        std::size_t write(std::span<const std::uint8_t> data) override {
            tx_->insert(tx_->end(), data.begin(), data.end());
            return data.size();
        }
        std::size_t read(std::span<std::uint8_t> buf) override {
            std::size_t n = 0;
            while (n < buf.size() && !rx_->empty()) {
                buf[n++] = rx_->front();
                rx_->pop_front();
            }
            return n;
        }

    private:
        std::deque<std::uint8_t>* tx_;
        std::deque<std::uint8_t>* rx_;
    };

    std::deque<std::uint8_t> a_to_b_;  // host -> client
    std::deque<std::uint8_t> b_to_a_;  // client -> host

    std::deque<std::uint8_t>* tx_;  // host side writes here (a_to_b_)
    std::deque<std::uint8_t>* rx_;  // host side reads here (b_to_a_)

    // Simulator endpoint: writes b_to_a_, reads a_to_b_ (mirror of host).
    PeerView peer_view_{&b_to_a_, &a_to_b_};
};

}  // namespace iomh