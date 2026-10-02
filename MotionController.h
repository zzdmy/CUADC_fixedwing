// MotionController.h
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include "MavlinkProtocolHandler.h"

class MotionController {
public:
    explicit MotionController(std::shared_ptr<MavlinkProtocolHandler> mavlinkHandler);
    ~MotionController();

    void start();
    void stop();

    // 设置 GPS 位置目标（经纬度 + 高度），控制循环持续发送 GUIDED 全局位置指令
    void setGpsPositionTarget(double lat_deg, double lon_deg, float altitude_m, float yaw_deg = 0.0f);

private:
    void runControlLoop();

    enum class CommandMode {
        NONE,
        GPS_POSITION    // 全球坐标系 GPS 位置（经纬度 + 高度）
    };

    // GPS 目标状态
    double gps_target_lat_ = 0.0;
    double gps_target_lon_ = 0.0;
    float gps_target_alt_ = 0.0f;
    float gps_target_yaw_ = 0.0f;

    std::shared_ptr<MavlinkProtocolHandler> mavlink_handler_;

    std::atomic<bool> is_running_{ false };
    std::thread control_thread_;

    mutable std::mutex state_mutex_;

    std::atomic<CommandMode> current_command_mode_{ CommandMode::NONE };

    static constexpr int CONTROL_FREQUENCY_HZ_ = 15;
    static constexpr double GPS_EPSILON_ = 0.0000001;   // GPS变化阈值(度) ~1cm
    static constexpr float POSITION_EPSILON_ = 0.01f;   // 高度变化阈值(米)
    static constexpr float YAW_EPSILON_ = 0.1f;         // 偏航角变化阈值(度)

    // 标志位
    bool gps_command_pending_ = false;
};

#endif // MOTION_CONTROLLER_H
