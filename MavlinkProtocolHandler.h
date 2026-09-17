// MavlinkProtocolHandler.h
#pragma once
#include "ITransport.h"
#include "config_loader.h"
#include <common/mavlink.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <optional>
#include <queue>
#include <boost/lockfree/queue.hpp> // 或者用 std::queue + mutex
#include <memory>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

// 辅助类型
struct FlightModeData {
    uint8_t base_mode;
    uint32_t custom_mode;
};

struct HomePositionData {
    double latitude;
    double longitude;
    float altitude;
};

class MavlinkProtocolHandler {
public:
    explicit MavlinkProtocolHandler(const AppConfig& cfg);
    ~MavlinkProtocolHandler();// 析构函数
    void set_transport(std::shared_ptr<ITransport> transport);
    void start();

    // === 导航命令 ===
    void requestAllDataStreams();// 请求飞控所有数据流（GPS/姿态/舵机/遥控等）
    void requestGpsDataStreams();// 请求 GPS 数据流（GPS_RAW_INT + GLOBAL_POSITION_INT）
    bool uploadMissionWaypoints(double initial_lat, double initial_lon, double takeoff_heading);// 生成并上传航点到飞控

    // === 高层命令 ===
    bool setFlightMode(uint8_t flight_mode);// 设置飞行模式
    bool setMissionCurrent(uint16_t seq);// 设置当前任务序号（投弹后跳降落段）
    void sendGuidedPositionGPS(double lat_deg, double lon_deg, float alt_amsl, float yaw_deg);// 发送引导模式的全局位置控制命令
    void sendGuidedVelocity(float vx, float vy, float vz, float yaw_rate);// 发送引导模式速度控制命令
    void sendServoPWM(uint8_t servo_n, uint16_t pwm_value);// 发送舵机PWM控制命令
    void setup_guided_position_local(mavlink_message_t& msg, float x, float y, float z, float yaw);//基于本地坐标系位置
    void setup_guided_position_gps(mavlink_message_t& msg, double  lat_deg, double  lon_deg, float  alt_amsl, float yaw);// 基于GPS坐标系位置
    void setup_guided_position_body(mavlink_message_t& msg, float x, float y, float z, float yaw);// 基于机体坐标系位置
    void setup_guided_velocity(mavlink_message_t& msg, float vx, float vy, float vz, float yaw_rate);// 基于机体坐标系速度

    // 数据获取接口
    bool try_get_mission_ack_data(mavlink_mission_ack_t& mission_ack_data);// 获取任务完成状态
    bool try_get_mission_request_int_data(mavlink_mission_request_int_t& request_data);// 获取任务请求数据
    bool try_get_mission_request_data(mavlink_mission_request_t& request_data);// 获取任务请求数据

    // === 获取状态 ===
    bool try_get_gps_data(mavlink_global_position_int_t& out);// 获取GPS数据
    bool try_get_gps_raw_data(mavlink_gps_raw_int_t& out);// 获取原始GPS数据
    bool try_get_imu_data(mavlink_attitude_t& out);// 获取IMU数据
    bool try_get_rc_channels(mavlink_rc_channels_t& out);// 获取遥控数据

    bool try_get_flight_mode(FlightModeData& out);// 获取当前飞行模式

    bool try_get_home_position(HomePositionData& out);  // 获取家位置数据
    bool try_get_distance_sensor_data(mavlink_distance_sensor_t& out);// 获取距离传感器数据
    bool try_get_latest_mission_current(mavlink_mission_current_t& out);// 获取当前任务
    void clear_mission_current();// 清除当前任务
    void sendMessage(const mavlink_message_t& msg);// 发送MAVLink消息
    void sendSetGlobalOrigin(int32_t lat_e7, int32_t lon_e7, int32_t alt_mm);// 设置GPS全局原点
    void sendSetHomePosition();// 设置Home位置为当前RTK修正后的位置（MAV_CMD_DO_SET_HOME）

    // === 心跳监控 ===
    uint64_t getHeartbeatAgeMs() const;                     // 距离上次心跳的毫秒数
    double getHeartbeatRate() const;                         // 心跳频率(Hz)
    bool isAutopilotIdentified() const;                      // 是否已识别飞控
    bool try_get_heartbeat_info(uint8_t& autopilot, uint8_t& type, uint8_t& system_status) const;

    // === 静态转换函数 ===
    static const char* autopilotToString(uint8_t autopilot);
    static const char* vehicleTypeToString(uint8_t type);
    static const char* systemStatusToString(uint8_t status);
    static const char* gpsFixTypeToString(uint8_t fix_type);
    static std::string flightModeToString(uint8_t autopilot, uint8_t type, uint32_t custom_mode);

private:
    void on_raw_data_received(const uint8_t* buffer, size_t len);// 数据接收处理
    void parse_mavlink_message(uint8_t chan, const uint8_t* buf, size_t len);// 解析MAVLink消息
    void sendMessageLoop();// 发送消息循环

    // 航点生成与发送
    std::vector<mavlink_mission_item_t> buildMissionWaypoints(double takeoff_landing_lat, double takeoff_landing_lon, double takeoff_heading);// 构建航点列表
    void requestMessageInterval(uint16_t msg_id, float hz);// 请求单个MAVLink消息流

    // 发送队列（线程安全）
    boost::lockfree::queue<mavlink_message_t> messageQueue{ 1024 };
    std::atomic<bool> running{ true };
    std::thread senderThread;

    // 状态缓存
    std::optional<mavlink_mission_ack_t> latest_mission_ack_data_;
    std::optional<mavlink_mission_request_int_t> latest_mission_request_int_data_;
    std::optional<mavlink_mission_request_t> latest_mission_request_data_;
    std::optional<mavlink_global_position_int_t> latest_gps_data_;
    std::optional<mavlink_gps_raw_int_t> latest_gps_raw_data_;
    std::optional<mavlink_attitude_t> latest_imu_data_;
    std::optional<mavlink_rc_channels_t> latest_rc_channels_;
    std::optional<FlightModeData> latest_flight_mode_;
    std::optional<HomePositionData> latest_home_position_;
    std::optional<mavlink_distance_sensor_t> latest_distance_sensor_;
    static mavlink_mission_current_t latest_mission_current_;
    std::optional<mavlink_servo_output_raw_t> latest_servo_output_;
    static std::atomic<bool> has_new_mission_current_;

    // Mutexes
    mutable std::mutex gps_data_mutex_, gps_raw_data_mutex_, imu_data_mutex_,
        rc_channels_mutex_, flight_mode_mutex_, home_position_mutex_,
        distance_sensor_mutex_, mission_current_mutex_;
    std::mutex mission_ack_data_mutex_;
    std::mutex mission_request_int_data_mutex_;
    std::mutex mission_request_data_mutex_;
    std::mutex servo_output_mutex_;
    // 配置
    const AppConfig& cfg_;
    // ID 配置
    const uint8_t system_id_, component_id_, target_system_, target_component_;

    std::shared_ptr<ITransport> transport_;
    std::condition_variable data_cv_;
    std::mutex data_cv_mutex_;

    // 心跳监控
    mutable std::mutex heartbeat_mutex_;
    std::chrono::steady_clock::time_point latest_heartbeat_time_{};
    uint8_t latest_system_status_ = 0;
    bool autopilot_identified_ = false;
    uint8_t identified_autopilot_ = 0;
    uint8_t identified_type_ = 0;
    int64_t heartbeat_interval_ms_ = 0;
};