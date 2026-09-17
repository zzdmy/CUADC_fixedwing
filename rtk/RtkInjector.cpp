// RtkInjector.cpp
#include "RtkInjector.h"
#include "../MavlinkProtocolHandler.h"
#include <iostream>

RtkInjector::RtkInjector(MavlinkProtocolHandler* handler)
    : handler_(handler) {
}

void RtkInjector::inject(const std::vector<uint8_t>& rtcm_frame) {
    if (rtcm_frame.empty()) return;

    size_t total_len = rtcm_frame.size();
    size_t offset = 0;
    uint8_t seq = sequence_id_++;  // bump sequence for this frame
    bool fragmented = (total_len > MAX_FRAGMENT_SIZE);

    while (offset < total_len) {
        size_t frag_len = std::min(MAX_FRAGMENT_SIZE, total_len - offset);

        // Build flags byte:
        // bit 0: fragmented flag (1 = part of a fragmented sequence)
        // bits 1-2: fragment index (0-3)
        // bits 3-7: sequence ID (0-31)
        // NOTE: all fragments must keep bit 0 = 1 so ArduPilot reassembles
        // instead of injecting each fragment as a standalone RTCM frame.
        uint8_t flags = 0;
        if (fragmented) {
            uint8_t frag_idx = static_cast<uint8_t>(offset / MAX_FRAGMENT_SIZE);
            flags = 0x01;                              // bit 0 = 1
            flags |= ((frag_idx & 0x03) << 1);         // bits 1-2
            flags |= ((seq & 0x1F) << 3);              // bits 3-7
        }

        mavlink_message_t msg;
        uint8_t data_padded[MAX_FRAGMENT_SIZE] = { 0 };
        memcpy(data_padded, rtcm_frame.data() + offset, frag_len);

        mavlink_msg_gps_rtcm_data_pack(
            SYS_ID, COMP_ID, &msg,
            flags,
            static_cast<uint8_t>(frag_len),
            data_padded
        );

        handler_->sendMessage(msg);
        stats.mavlink_packets_sent++;

        offset += frag_len;
    }

    stats.valid_frames++;
    stats.total_bytes += total_len;
}
