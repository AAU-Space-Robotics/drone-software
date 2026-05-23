#pragma once

#include <cstddef>
#include <cstdint>

// Max JPEG bytes per UDP datagram — well under the 65507-byte UDP limit.
// At typical 720p JPEG quality this is almost always a single fragment.
static constexpr size_t CAMERA_MAX_FRAG_PAYLOAD = 60000;

// Prepended to every camera UDP datagram (both single and fragmented frames).
#pragma pack(push, 1)
struct VideoFrameHeader {
    uint32_t frame_id;      // monotonically increasing per-frame counter
    uint16_t frag_idx;      // 0-based fragment index
    uint16_t total_frags;   // total fragments for this frame
    uint32_t frame_size;    // total bytes in the complete JPEG
    uint32_t payload_size;  // JPEG bytes carried in this datagram
};
#pragma pack(pop)
