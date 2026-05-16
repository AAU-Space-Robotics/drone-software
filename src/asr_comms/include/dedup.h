#pragma once

#include <bitset>
#include <cstdint>
#include <mutex>
#include <unordered_map>

// Rolling per-sender deduplication filter for MAVLink packets.
//
// When the same encoded packet arrives on both the SiK radio and the WiFi
// transport, both copies carry identical (sysid, compid, seq). This filter
// tracks the last ~128 sequence numbers per sender and returns false on the
// second arrival so handle_message is only called once.
//
// Thread-safe: check() may be called concurrently from two recv threads.
class DedupFilter {
public:
    // Returns true  → packet is new, call handle_message.
    // Returns false → duplicate, drop silently.
    bool check(uint8_t sysid, uint8_t compid, uint8_t seq)
    {
        const uint16_t key = static_cast<uint16_t>(sysid) << 8 | compid;
        std::lock_guard<std::mutex> lock(mutex_);
        auto& bits = seen_[key];
        if (bits.test(seq)) return false;
        bits.set(seq);
        // Clear the entry 128 steps ahead to prevent the far side of the
        // 256-wrap ring from causing false positives on very old seq numbers.
        bits.reset((static_cast<unsigned>(seq) + 128u) % 256u);
        return true;
    }

private:
    std::mutex                                      mutex_;
    std::unordered_map<uint16_t, std::bitset<256>> seen_;
};
