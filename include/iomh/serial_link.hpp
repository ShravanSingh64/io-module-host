#pragma once

#include <cstdint>
#include <span>

namespace iomh {

// Byte-transport abstraction. The parser and host depend only on this, never
// on a concrete transport — so the in-memory simulator link today and a real
// /dev/tty serial port later are drop-in interchangeable. This is the seam
// that keeps parsing separated from transport.
class ISerialLink {
public:
    virtual ~ISerialLink() = default;

    // Write bytes toward the peer. Returns the number of bytes accepted.
    virtual std::size_t write(std::span<const std::uint8_t> data) = 0;

    // Read up to buf.size() bytes that have arrived from the peer into buf.
    // Returns the number of bytes copied; 0 means "nothing available right
    // now" (not an error, not end-of-stream). A non-blocking, best-effort
    // read like this is what lets the host treat a silent/dead peer as a
    // truncation case instead of hanging on it.
    virtual std::size_t read(std::span<std::uint8_t> buf) = 0;

protected:
    // Interface type: copy/move only through derived classes, never sliced.
    ISerialLink() = default;
    ISerialLink(const ISerialLink&) = default;
    ISerialLink(ISerialLink&&) = default;
    ISerialLink& operator=(const ISerialLink&) = default;
    ISerialLink& operator=(ISerialLink&&) = default;
};

}  // namespace iomh