// RtkInjector.h
#pragma once
#include <cstdint>
#include <vector>
#include <atomic>

class MavlinkProtocolHandler;

struct RtkStats {
    std::atomic<size_t> valid_frames{ 0 };
    std::atomic<size_t> invalid_frames{ 0 };
    std::atomic<size_t> mavlink_packets_sent{ 0 };
    std::atomic<size_t> total_bytes{ 0 };
};

class RtkInjector {
public:
    explicit RtkInjector(MavlinkProtocolHandler* handler);

    // Inject a complete, CRC-validated RTCM frame
    // Splits into <=180 byte fragments, packs as GPS_RTCM_DATA, sends via MAVLink
    void inject(const std::vector<uint8_t>& rtcm_frame);

    RtkStats stats;

private:
    MavlinkProtocolHandler* handler_;
    uint8_t sequence_id_{ 0 };  // rolling sequence ID (0-31) for GPS_RTCM_DATA flags

    static constexpr size_t MAX_FRAGMENT_SIZE = 180;  // MAVLink GPS_RTCM_DATA payload max
    static constexpr uint8_t SYS_ID = 2;
    static constexpr uint8_t COMP_ID = 190;  // MAV_COMP_ID_GPS
};
