// NtripClient.h
#pragma once
#include "RtkInjector.h"
#include "../config_loader.h"
#include <string>
#include <thread>
#include <atomic>
#include <memory>

class MavlinkProtocolHandler;

enum class NtripState {
    WaitingForGps,      // waiting for GPS accuracy
    Connecting,         // connecting to NTRIP caster
    Connected,          // receiving RTCM data
    Reconnecting,       // disconnected, waiting to retry
    MaxRetriesExceeded, // gave up after max retries
    Stopped,            // explicitly stopped
    Error               // unrecoverable error
};

class NtripClient {
public:
    NtripClient(const RtkConfig& cfg, MavlinkProtocolHandler* handler);
    ~NtripClient();

    // Start in background (non-blocking) -- waits for GPS, then connects
    void start();

    // Stop all threads
    void stop();

    NtripState getState() const { return state_.load(); }
    const RtkStats& getStats() const { return injector_.stats; }

private:
    void run();  // main thread function: wait GPS -> connect -> receive loop

    // RTCM processing
    void processRtcmBuffer();

    // RTCM parsing
    static uint32_t calcRtcmCrc24q(const uint8_t* data, size_t length);
    static std::string base64Encode(const std::string& input);

    // GGA
    std::string generateGga(double lat_deg, double lon_deg, double altitude);
    std::string getUtcTimeStr();

    // GPS wait
    bool isGpsAccurate();

    const RtkConfig& cfg_;
    MavlinkProtocolHandler* handler_;
    RtkInjector injector_;

    std::thread worker_thread_;
    std::atomic<bool> running_{ false };
    std::atomic<NtripState> state_{ NtripState::Stopped };

    // reconnect state
    int reconnect_attempts_{ 0 };
    double last_gps_lat_{ 0 };
    double last_gps_lon_{ 0 };

    // RTCM reassembly buffer
    std::vector<uint8_t> rtcm_buffer_;
};
