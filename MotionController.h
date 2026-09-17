// MotionController.h
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include "MavlinkProtocolHandler.h"
extern std::atomic<float> doublefast;

class SerialPortHandler;
class MotionController {
public:
    explicit MotionController(std::shared_ptr<MavlinkProtocolHandler> mavlinkHandler);
    ~MotionController();

    void start();
    void stop();

    // ===== 视觉伺服（图像坐标系）=====
    void setVisualServoTarget(float pixel_x, float pixel_y, float vz_mps = 0.0f);

    // ===== 速度命令（机体坐标系）=====
    void setVelocityCommand(float vx, float vy, float vz);  // vx=前, vy=右, vz=下

    // ===== 位置目标（机体坐标系）=====
    void setPositionTarget(float x, float y, float z, float yaw_deg);      // x=前, y=右, z=下

    // ===== 模式控制 =====
    void enableTocMode(bool enable);
    void setImageResolution(int width, int height);


    // 设置 GPS 位置目标（经纬度 + 高度）
    void setGpsPositionTarget(double lat_deg, double lon_deg, float altitude_m, float yaw_deg = 0.0f);
    // Fly to GPS target and wait for arrival (altitude/threshold/timeout from config)
    bool flyToGpsAndWait(double target_lat_deg, double target_lon_deg, double altitude_m);

private:
    void runControlLoop();

    enum class CommandMode {
        NONE,
        POSITION,   // 位置目标（机体坐标系）
        VELOCITY,   // 速度命令（机体坐标系）
        VISUAL_SERVO, // 视觉伺服（图像坐标系）
        GPS_POSITION    // 全球坐标系 GPS 位置（经纬度 + 高度）
    };
    // GPS 目标状态
    double gps_target_lat_ = 0.0;
    double gps_target_lon_ = 0.0;
    float gps_target_alt_ = 0.0f;
    float gps_target_yaw_ = 0.0f;

    std::shared_ptr<MavlinkProtocolHandler> mavlink_handler_;

    std::atomic<bool> is_running_{ false };
    std::atomic<bool> is_toc_mode_{ false };
    std::thread control_thread_;

    mutable std::mutex state_mutex_;

    // 当前状态
    std::atomic<CommandMode> current_command_mode_{ CommandMode::NONE };
    float position_target_x_ = 0.0f;
    float position_target_y_ = 0.0f;
    float position_target_z_ = 0.0f;
    float position_target_yaw_ = 0.0f;
    float velocity_vx_ = 0.0f;
    float velocity_vy_ = 0.0f;
    float velocity_vz_ = 0.0f;

    // 视觉伺服状态
    bool has_visual_target_ = false;
    float visual_target_x_ = 0.0f;
    float visual_target_y_ = 0.0f;
    float visual_target_vz_ = 0.0f;

    // 图像尺寸
    int image_width_ = 640;
    int image_height_ = 480;

    // 控制参数（从配置加载）
    float kP_GAIN_;
    float MAX_SPEED_MPS_;
    static constexpr int CONTROL_FREQUENCY_HZ_ = 15;

    // 阈值检测参数
    static constexpr float POSITION_EPSILON_ = 0.01f;  // 位置变化阈值(米)
    static constexpr float YAW_EPSILON_ = 0.1f;        // 偏航角变化阈值(度)
    static constexpr double GPS_EPSILON_ = 0.0000001;   // GPS变化阈值(度) ~1cm

    // 标志位
    bool position_command_pending_ = false;
    bool gps_command_pending_ = false;
};

#endif // MOTION_CONTROLLER_H