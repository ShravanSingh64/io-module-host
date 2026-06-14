#pragma once

#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <vector>

#include "iomh/protocol.hpp"

namespace iomh {

// Push-based frame parser. Bytes are appended via feed(); completed frames
// (or aligned-but-corrupt frames) are drained via next(). The split decouples
// transport chunking from frame extraction, which is what makes truncation
// non-hanging: when next() cannot complete a frame it returns nullopt and the
// caller simply reads more bytes.
class FrameParser {
public:
    using Result = std::expected<InputsFrame, ParseError>;

    // Append raw bytes to the internal buffer.
    void feed(std::span<const std::uint8_t> bytes);

    // Extract the next event:
    //   - value           : a complete, CRC-valid frame
    //   - error            : an aligned frame that failed payload validation
    //   - std::nullopt     : need more bytes (incomplete / truncated / drained)
    // Header CRC failures are handled silently as part of resync — they are the
    // mechanism for finding sync, not an event the caller acts on.
    std::optional<Result> next();

    void reset();

    [[nodiscard]] std::size_t buffered() const noexcept {
        return buffer_.size() - pos_;
    }

private:
    std::vector<std::uint8_t> buffer_;
    std::size_t pos_{0};  // cursor: index of the current candidate start token
};

}  // namespace iomh