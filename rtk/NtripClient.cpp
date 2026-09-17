// NtripClient.cpp
#include "NtripClient.h"
#include "../MavlinkProtocolHandler.h"
#include "../AppLogger.h"
#include <boost/asio.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#endif

NtripClient::NtripClient(const RtkConfig& cfg, MavlinkProtocolHandler* handler)
    : cfg_(cfg), handler_(handler), injector_(handler) {
}

NtripClient::~NtripClient() {
    stop();
}

void NtripClient::start() {
    if (running_.exchange(true)) return;
    state_.store(NtripState::WaitingForGps);
    worker_thread_ = std::thread(&NtripClient::run, this);
    AppLogger::get().info("[RTK] Service started, waiting for GPS...");
}

void NtripClient::stop() {
    running_.store(false);
    state_.store(NtripState::Stopped);
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    AppLogger::get().info("[RTK] Service stopped.");
}

// ==================== Main Thread ====================

void NtripClient::run() {
    // Step 1: Wait for GPS accuracy
    while (running_) {
        if (isGpsAccurate()) {
            AppLogger::get().info("[RTK] GPS accuracy reached, connecting to NTRIP...");
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!running_) return;

    // Step 2: Connect + receive loop with reconnect
    reconnect_attempts_ = 0;
    while (running_) {
        if (reconnect_attempts_ >= cfg_.max_reconnect_attempts) {
            AppLogger::get().error("[RTK] Max reconnect attempts ({}) exceeded, giving up.",
                cfg_.max_reconnect_attempts);
            state_.store(NtripState::MaxRetriesExceeded);
            return;
        }

        state_.store(NtripState::Connecting);
        AppLogger::get().info("[RTK] Connecting to {}:{} (attempt {}/{}",
            cfg_.ntrip_host, cfg_.ntrip_port,
            (reconnect_attempts_ + 1), cfg_.max_reconnect_attempts);

        try {
            boost::asio::io_context io;
            boost::asio::ip::tcp::socket socket(io);
            boost::asio::ip::tcp::resolver resolver(io);

            auto endpoints = resolver.resolve(cfg_.ntrip_host,
                std::to_string(cfg_.ntrip_port));
            boost::asio::connect(socket, endpoints);

            // Send HTTP request
            std::string auth = "Basic " + base64Encode(cfg_.ntrip_username + ":" + cfg_.ntrip_password);
            std::string request =
                "GET /" + cfg_.ntrip_mountpoint + " HTTP/1.0\r\n"
                "User-Agent: NTRIPClient/1.0\r\n"
                "Authorization: " + auth + "\r\n"
                "Connection: close\r\n"
                "\r\n";

            boost::asio::write(socket, boost::asio::buffer(request));

            // Read response
            boost::asio::streambuf response;
            boost::system::error_code ec;
            boost::asio::read_until(socket, response, "\r\n", ec);
            if (ec) {
                AppLogger::get().error("[RTK] Failed to read response: {}", ec.message());
                throw std::runtime_error("NTRIP response read failed");
            }

            std::istream response_stream(&response);
            std::string first_line;
            std::getline(response_stream, first_line);
            AppLogger::get().info("[RTK] Server response: {}", first_line);

            if (first_line.find("200") == std::string::npos) {
                AppLogger::get().error("[RTK] NTRIP connection rejected!");
                throw std::runtime_error("NTRIP rejected");
            }

            // Connected
            state_.store(NtripState::Connected);
            reconnect_attempts_ = 0;
            AppLogger::get().info("[RTK] Connected, receiving RTCM data...");

            // === Receive Loop with GGA keepalive ===
            socket.non_blocking(true);
            std::vector<char> net_buffer(4096);
            auto last_gga_time = std::chrono::steady_clock::now();

            while (running_) {
                // Read from TCP
                boost::system::error_code recv_ec;
                size_t len = socket.read_some(boost::asio::buffer(net_buffer), recv_ec);

                if (recv_ec == boost::asio::error::would_block) {
                    // No data, check GGA timer
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                } else if (recv_ec == boost::asio::error::eof) {
                    AppLogger::get().info("[RTK] Server closed connection.");
                    break;
                } else if (recv_ec) {
                    AppLogger::get().error("[RTK] Network error: {}", recv_ec.message());
                    break;
                } else if (len > 0) {
                    // Append to RTCM reassembly buffer
                    rtcm_buffer_.insert(rtcm_buffer_.end(),
                        net_buffer.begin(), net_buffer.begin() + len);
                    processRtcmBuffer();
                }

                // Send GGA keepalive
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(
                    now - last_gga_time).count() >= cfg_.gga_interval_sec) {
                    // Update position from latest GPS
                    mavlink_gps_raw_int_t gps;
                    if (handler_->try_get_gps_raw_data(gps)) {
                        last_gps_lat_ = static_cast<double>(gps.lat) / 1e7;
                        last_gps_lon_ = static_cast<double>(gps.lon) / 1e7;
                    }
                    try {
                        std::string gga = generateGga(last_gps_lat_, last_gps_lon_, 10.0);
                        boost::asio::write(socket, boost::asio::buffer(gga), recv_ec);
                    } catch (...) {
                        // GGA send failed, socket probably closed
                    }
                    last_gga_time = now;
                }
            }
        } catch (const std::exception& e) {
            AppLogger::get().error("[RTK] Error: {}", e.what());
        }

        if (!running_) break;

        state_.store(NtripState::Reconnecting);
        reconnect_attempts_++;
        AppLogger::get().info("[RTK] Will retry in {}s (attempt {}/{})",
            cfg_.reconnect_interval_sec, reconnect_attempts_, cfg_.max_reconnect_attempts);
        std::this_thread::sleep_for(std::chrono::seconds(cfg_.reconnect_interval_sec));
    }
}

// ==================== RTCM Parsing ====================

uint32_t NtripClient::calcRtcmCrc24q(const uint8_t* data, size_t length) {
    uint32_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= (static_cast<uint32_t>(data[i]) << 16);
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000) crc ^= 0x1864CFB;
        }
    }
    return crc & 0xFFFFFF;
}

void NtripClient::processRtcmBuffer() {
    while (true) {
        if (rtcm_buffer_.size() < 6) break;

        bool found = false;
        for (size_t i = 0; i + 6 <= rtcm_buffer_.size(); ++i) {
            if (rtcm_buffer_[i] == 0xD3) {
                int payload_len = ((rtcm_buffer_[i + 1] & 0x03) << 8) | rtcm_buffer_[i + 2];
                size_t full_frame_len = 3 + payload_len + 3;

                if (i + full_frame_len <= rtcm_buffer_.size()) {
                    found = true;
                    std::vector<uint8_t> frame(rtcm_buffer_.begin() + i,
                        rtcm_buffer_.begin() + i + full_frame_len);

                    uint32_t received_crc = (frame[full_frame_len - 3] << 16)
                        | (frame[full_frame_len - 2] << 8)
                        | frame[full_frame_len - 1];
                    uint32_t calculated_crc = calcRtcmCrc24q(frame.data(), full_frame_len - 3);

                    if (received_crc == calculated_crc) {
                        injector_.inject(frame);
                    } else {
                        injector_.stats.invalid_frames++;
                    }

                    rtcm_buffer_.erase(rtcm_buffer_.begin(), rtcm_buffer_.begin() + i + full_frame_len);
                    break;
                } else {
                    break; // incomplete frame
                }
            }
        }

        if (!found) {
            if (rtcm_buffer_.size() > 4096) {
                AppLogger::get().error("[RTK] Buffer overflow, clearing.");
                rtcm_buffer_.clear();
            }
            break;
        }
    }
}

// ==================== GGA ====================

std::string NtripClient::getUtcTimeStr() {
    auto now = std::chrono::system_clock::now();
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm;
#ifdef _WIN32
    gmtime_s(&utc_tm, &time_t_now);
#else
    gmtime_r(&time_t_now, &utc_tm);
#endif
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << utc_tm.tm_hour
        << std::setw(2) << utc_tm.tm_min
        << std::setw(2) << utc_tm.tm_sec << "."
        << std::setw(2) << (ms.count() / 10);
    return oss.str();
}

std::string NtripClient::generateGga(double lat_deg, double lon_deg, double altitude) {
    auto to_nmea = [](double deg) -> std::string {
        int dd = static_cast<int>(std::abs(deg));
        double mm = (std::abs(deg) - dd) * 60.0;
        char buf[32];
        snprintf(buf, sizeof(buf), "%02d%07.4f", dd, mm);
        return std::string(buf);
    };

    std::string time_str = getUtcTimeStr();
    std::string lat_str = to_nmea(lat_deg);
    std::string lon_str = to_nmea(lon_deg);
    char lat_dir = lat_deg >= 0 ? 'N' : 'S';
    char lon_dir = lon_deg >= 0 ? 'E' : 'W';

    char gga_template[256];
    snprintf(gga_template, sizeof(gga_template),
        "$GPGGA,%s,%s,%c,%s,%c,1,08,1.0,%.1f,M,0.0,M,,",
        time_str.c_str(), lat_str.c_str(), lat_dir,
        lon_str.c_str(), lon_dir, altitude);

    uint8_t checksum = 0;
    for (const char* p = gga_template + 1; *p != '\0' && *p != '*'; ++p)
        checksum ^= static_cast<uint8_t>(*p);

    char final_sentence[300];
    snprintf(final_sentence, sizeof(final_sentence), "%s*%02X\r\n", gga_template, checksum);
    return std::string(final_sentence);
}

// ==================== Helpers ====================

bool NtripClient::isGpsAccurate() {
    mavlink_gps_raw_int_t gps;
    if (handler_->try_get_gps_raw_data(gps)) {
        double h_acc_m = static_cast<double>(gps.h_acc) / 1000.0;
        if (gps.h_acc != UINT32_MAX &&
            h_acc_m < cfg_.gps_accuracy_threshold &&
            gps.satellites_visible >= cfg_.min_satellites) {
            last_gps_lat_ = static_cast<double>(gps.lat) / 1e7;
            last_gps_lon_ = static_cast<double>(gps.lon) / 1e7;
            return true;
        }
    }
    return false;
}

std::string NtripClient::base64Encode(const std::string& input) {
    static const char* b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string encoded;
    int val = 0, valc = -6;
    for (unsigned char c : input) {
        val = (val << 8) + c;
        valc += 8;
        while (valc >= 0) {
            encoded.push_back(b64[(val >> valc) & 0x3F]);
            valc -= 6;
        }
    }
    if (valc > -6) encoded.push_back(b64[((val << 8) >> (valc + 8)) & 0x3F]);
    while (encoded.size() % 4) encoded.push_back('=');
    return encoded;
}
