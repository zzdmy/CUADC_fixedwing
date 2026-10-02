// MavlinkProtocolHandler.cpp
#include "MavlinkProtocolHandler.h"
#define GEOGRAPHICLIB_SHARED_LIB 1
#include <GeographicLib/Geodesic.hpp>
#include "AppLogger.h"
#include <iomanip>
#include <bitset>
#include <cmath>
#include <sstream>

namespace
{
    // ArduPilot：custom_mode = 模式编号（依赖 vehicle type）
    static inline std::string ardupilot_mode_to_cn(uint8_t vehicle_type, uint32_t custom_mode)
    {
        // 固定翼（示例补几个常用）
        if (vehicle_type == MAV_TYPE_FIXED_WING)
        {
            // ArduPlane 常见：0 MANUAL, 2 CIRCLE, 3 STABILIZE, 4 TRAINING, 5 ACRO, 6 FBWA, 7 FBWB, 8 CRUISE, 10 AUTO, 11 RTL, 12 LOITER, 15 GUIDED, 16 INITIALISING, 17 QSTABILIZE, 18 QHOVER, 19 QLOITER, 20 QLAND, 21 QRTL, 22 QAUTOTUNE, 23 QACRO
            switch (custom_mode)
            {
            case 0: return "手动（Manual）";
            case 3: return "自稳（Stabilize）";
            case 6: return "FBWA";
            case 7: return "FBWB";
            case 8: return "巡航（Cruise）";
            case 10: return "自动任务（Auto）";
            case 11: return "返航（RTL）";
            case 12: return "盘旋（Loiter）";
            case 15: return "引导（Guided）";
            default:
            {
                std::ostringstream oss;
                oss << "ArduPlane模式未知(" << custom_mode << ")";
                return oss.str();
            }
            }
        }
        std::ostringstream oss;
        oss << "ArduPilot模式未知(type=" << (int)vehicle_type << ", custom=" << custom_mode << ")";
        return oss.str();
    }
}
// Static members
mavlink_mission_current_t MavlinkProtocolHandler::latest_mission_current_{};
std::atomic<bool> MavlinkProtocolHandler::has_new_mission_current_{ false };

const char* MavlinkProtocolHandler::autopilotToString(uint8_t autopilot) {
    switch (autopilot) {
    case MAV_AUTOPILOT_PX4: return "PX4";
    case MAV_AUTOPILOT_ARDUPILOTMEGA: return "ArduPilot";
    default: return "未知飞控";
    }
}

const char* MavlinkProtocolHandler::vehicleTypeToString(uint8_t type) {
    switch (type) {
    case MAV_TYPE_QUADROTOR: return "四旋翼";
    case MAV_TYPE_HEXAROTOR: return "六旋翼";
    case MAV_TYPE_OCTOROTOR: return "八旋翼";
    case MAV_TYPE_FIXED_WING: return "固定翼";
    case MAV_TYPE_GROUND_ROVER: return "地面车";
    case MAV_TYPE_SUBMARINE: return "潜航器";
    default: return "未知机型";
    }
}

const char* MavlinkProtocolHandler::systemStatusToString(uint8_t status) {
    switch (status) {
    case MAV_STATE_UNINIT: return "未初始化";
    case MAV_STATE_BOOT: return "启动中";
    case MAV_STATE_CALIBRATING: return "校准中";
    case MAV_STATE_STANDBY: return "待命";
    case MAV_STATE_ACTIVE: return "运行";
    case MAV_STATE_CRITICAL: return "严重";
    case MAV_STATE_EMERGENCY: return "紧急";
    case MAV_STATE_POWEROFF: return "关机";
    case MAV_STATE_FLIGHT_TERMINATION: return "终止飞行";
    default: return "未知状态";
    }
}

const char* MavlinkProtocolHandler::gpsFixTypeToString(uint8_t fix_type) {
    switch (fix_type) {
    case 0: return "无GPS";
    case 1: return "无定位";
    case 2: return "2D定位";
    case 3: return "3D定位";
    case 4: return "DGPS/RTK浮点";
    case 5: return "RTK浮点";
    case 6: return "RTK固定";
    default: return "未知";
    }
}

std::string MavlinkProtocolHandler::flightModeToString(uint8_t autopilot, uint8_t type, uint32_t custom_mode) {
    if (autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) return ardupilot_mode_to_cn(type, custom_mode);
    return "未知飞控模式";
}

MavlinkProtocolHandler::MavlinkProtocolHandler(const AppConfig& cfg)
    : cfg_(cfg),
    system_id_(static_cast<uint8_t>(cfg.mavlink.system_id)),
    component_id_(static_cast<uint8_t>(cfg.mavlink.component_id)),
    target_system_(cfg.mavlink.target_system),
    target_component_(cfg.mavlink.target_component) {
}

void MavlinkProtocolHandler::set_transport(std::shared_ptr<ITransport> transport) {
    transport_ = transport;
}

void MavlinkProtocolHandler::start() {
    if (!transport_) {
        throw std::runtime_error("Transport not set!");
    }
    senderThread = std::thread(&MavlinkProtocolHandler::sendMessageLoop, this);
    transport_->start_receive([this](const uint8_t* buf, size_t len) {
        on_raw_data_received(buf, len);
        });
}
void MavlinkProtocolHandler::sendMessage(const mavlink_message_t& msg) {
    {
        messageQueue.push(msg);//添加到队列
    }

    // 注意：不需要手动唤醒，发送线程应持续运行或使用条件变量
    // 如果你用的是“轮询”模式，确保 senderThread 在运行
}
void MavlinkProtocolHandler::on_raw_data_received(const uint8_t* buffer, size_t len) {
    parse_mavlink_message(MAVLINK_COMM_0, buffer, len);
}

void MavlinkProtocolHandler::parse_mavlink_message(uint8_t chan, const uint8_t* buffer, size_t len) {
    static mavlink_message_t msg;
    static mavlink_status_t status;

    for (size_t i = 0; i < len; ++i) {
        if (mavlink_parse_char(chan, buffer[i], &msg, &status)) {
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {//
                mavlink_global_position_int_t pos;
                mavlink_msg_global_position_int_decode(&msg, &pos);
                {
                    std::lock_guard<std::mutex> lock(gps_data_mutex_);
                    latest_gps_data_ = pos;
                  AppLogger::get().debug(" 对于起飞点: {}mm", pos.relative_alt);
                }
                data_cv_.notify_one();
                break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT: {
                mavlink_gps_raw_int_t raw;
                mavlink_msg_gps_raw_int_decode(&msg, &raw);
                {
                    std::lock_guard<std::mutex> lock(gps_raw_data_mutex_);
                    latest_gps_raw_data_ = raw;
                   
                }
                data_cv_.notify_one();
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE: {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&msg, &att);
                {
                    std::lock_guard<std::mutex> lock(imu_data_mutex_);
                    latest_imu_data_ = att;
                    
                }
                data_cv_.notify_one();
                break;
            }
            case MAVLINK_MSG_ID_RC_CHANNELS: {
                mavlink_rc_channels_t rc;
                mavlink_msg_rc_channels_decode(&msg, &rc);
                {
                    std::lock_guard<std::mutex> lock(rc_channels_mutex_);
                    latest_rc_channels_ = rc;
                }
                data_cv_.notify_one();
                break;
            }
            case MAVLINK_MSG_ID_HEARTBEAT: {
                // 只接受来自目标飞控的心跳
                if (msg.sysid != target_system_) break;

                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);

                {
                    std::lock_guard<std::mutex> lock(flight_mode_mutex_);
                    latest_flight_mode_ = FlightModeData{ hb.base_mode, hb.custom_mode };
                }

                // 心跳监控：记录时间戳、系统状态、识别飞控和机型
                {
                    auto now = std::chrono::steady_clock::now();
                    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                    if (autopilot_identified_) {
                        heartbeat_interval_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - latest_heartbeat_time_).count();
                    }
                    latest_heartbeat_time_ = now;
                    latest_system_status_ = hb.system_status;
                    if (!autopilot_identified_ && hb.autopilot != MAV_AUTOPILOT_INVALID) {
                        autopilot_identified_ = true;
                        identified_autopilot_ = hb.autopilot;
                        identified_type_ = hb.type;
                        AppLogger::get().info("飞控识别: {} / {}",
                            autopilotToString(hb.autopilot), vehicleTypeToString(hb.type));
                    }
                }

                const bool has_custom = (hb.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) != 0;

                std::string mode_cn;
                if (!has_custom)
                {
                    mode_cn = "未启用自定义模式（CUSTOM_MODE_DISABLED）";
                }
                else if (hb.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA)
                {
                    mode_cn = ardupilot_mode_to_cn(hb.type, hb.custom_mode);
                }
                else
                {
                    std::ostringstream oss;
                    oss << "未知飞控模式(custom_mode=" << hb.custom_mode << ")";
                    mode_cn = oss.str();
                }

                //std::cout
                //    << "心跳: 飞控为" << autopilot_to_cn(hb.autopilot)
                //    << "，机型为" << vehicle_type_to_cn(hb.type)
                //    << "，模式为" << mode_cn
                //   /* << "，base_mode=" << std::bitset<8>(hb.base_mode)
                //    << "，custom_mode=0x" << std::hex << hb.custom_mode << std::dec*/
                //    << std::endl;

                break;
            }
           
            case MAVLINK_MSG_ID_HOME_POSITION: {//首次定位
                mavlink_home_position_t hp;
                mavlink_msg_home_position_decode(&msg, &hp);
                HomePositionData data{
                    hp.latitude / 1e7,
                    hp.longitude / 1e7,
                    hp.altitude / 1000.0f
                };
                {
                    std::lock_guard<std::mutex> lock(home_position_mutex_);
                    latest_home_position_ = data;
                }
                break;
            }
            case MAVLINK_MSG_ID_DISTANCE_SENSOR: {//距离传感器
                mavlink_distance_sensor_t ds;
                mavlink_msg_distance_sensor_decode(&msg, &ds);
                {
                    std::lock_guard<std::mutex> lock(distance_sensor_mutex_);
                    latest_distance_sensor_ = ds;
                }
                break;
            }
            case MAVLINK_MSG_ID_MISSION_CURRENT: {//当前任务
                mavlink_mission_current_t mc;
                mavlink_msg_mission_current_decode(&msg, &mc);
                {
                    std::lock_guard<std::mutex> lock(mission_current_mutex_);
                    latest_mission_current_ = mc;
                    has_new_mission_current_ = true;
                }
                break;
            }
            case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
                mavlink_servo_output_raw_t servo_output = {};
                mavlink_msg_servo_output_raw_decode(&msg, &servo_output);
                {
                    std::lock_guard<std::mutex> lock(servo_output_mutex_);
                    latest_servo_output_ = servo_output;
                }
                //// 调试输出部分（可选地放在线程安全区域外）
                //std::cout << "[DEBUG] Servo Output Raw:" << std::endl;
                //std::cout << "  TimeUS: " << servo_output.time_usec << std::endl;
                //std::cout << "  Servo1: " << servo_output.servo1_raw << std::endl;
                //std::cout << "  Servo2: " << servo_output.servo2_raw << std::endl;
                //std::cout << "  Servo3: " << servo_output.servo3_raw << std::endl;
                //std::cout << "  Servo4: " << servo_output.servo4_raw << std::endl;
                //std::cout << "  Servo5: " << servo_output.servo5_raw << std::endl;
                //std::cout << "  Servo6: " << servo_output.servo6_raw << std::endl;
                //std::cout << "  Servo7: " << servo_output.servo7_raw << std::endl;
                //std::cout << "  Servo8: " << servo_output.servo8_raw << std::endl;

                break;
            }
            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: {
                mavlink_position_target_local_ned_t target_ned;
                mavlink_msg_position_target_local_ned_decode(&msg, &target_ned);

                // 解析类型掩码（确定哪些字段有效）
                uint16_t type_mask = target_ned.type_mask;
                std::bitset<16> mask_bits(type_mask);
                // 检查位置是否有效
                if (!(type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE)) {
                    AppLogger::get().debug("  目标位置: 北={}m, 东={}m, 地={}m", target_ned.x, target_ned.y, target_ned.z);
                }

                // 检查速度是否有效
                if (!(type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE)) {
                    AppLogger::get().debug("  目标速度: 北={}m/s, 东={}m/s, 地={}m/s", target_ned.vx, target_ned.vy, target_ned.vz);
                }

                // 检查偏航角是否有效
                if (!(type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)) {
                    AppLogger::get().debug("  目标偏航角: {}rad", target_ned.yaw);
                }
                if (type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) {
                    AppLogger::get().debug("  偏航角速率: {}rad/s", target_ned.yaw_rate);
                }

                break;
            }
            case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {//接受
                mavlink_mission_request_int_t request;
                mavlink_msg_mission_request_int_decode(&msg, &request);

                std::lock_guard<std::mutex> lock(mission_request_int_data_mutex_);
                latest_mission_request_int_data_ = request;
                break;
            }

            case MAVLINK_MSG_ID_MISSION_REQUEST: {//
                mavlink_mission_request_t request;
                mavlink_msg_mission_request_decode(&msg, &request);

                std::lock_guard<std::mutex> lock(mission_request_data_mutex_);
                latest_mission_request_data_ = request;
                break;
            }
            case MAVLINK_MSG_ID_MISSION_ACK: {
                mavlink_mission_ack_t mission_ack;
                mavlink_msg_mission_ack_decode(&msg, &mission_ack);
                {
                    std::lock_guard<std::mutex> lock(mission_ack_data_mutex_);
                    latest_mission_ack_data_ = mission_ack;//添加到队列中
                }
                
                AppLogger::get().info("任务确认:");
                AppLogger::get().info("目标系统:{}", (int)mission_ack.target_system);
                AppLogger::get().info("目标组件:{}", (int)mission_ack.target_component);
                if (mission_ack.type == MAV_MISSION_ACCEPTED) {
                    AppLogger::get().info("结果: 成功");
                }
                else {
                    AppLogger::get().info("结果:失败,错误码:{}", (int)mission_ack.type);
                }
                AppLogger::get().info("  任务类型: {}", (int)mission_ack.mission_type);
                if (mission_ack.opaque_id != 0) {
                    AppLogger::get().info("  任务ID: {}", mission_ack.opaque_id);
                }
                else {
                    AppLogger::get().info("  任务ID: 0");
                }
                break;
            }
            case MAVLINK_MSG_ID_MESSAGE_INTERVAL: {
                mavlink_message_interval_t interval_msg;
                mavlink_msg_message_interval_decode(&msg, &interval_msg);

                uint16_t msg_id = interval_msg.message_id;
                int32_t interval_us = interval_msg.interval_us;
               
                AppLogger::get().info("收到");
                AppLogger::get().info("消息ID:{}", msg_id);

                if (interval_us == -1) {
                    AppLogger::get().info(" ");
                }
                else if (interval_us == 0) {
                    AppLogger::get().info("  状态: 使用默认频率");
                }
                else {
                    float freq = 1e6f / interval_us;
                    AppLogger::get().info("  间隔: {} 微秒 ({} Hz)", interval_us, freq);
                }

                break;
            }








                                                // 可继续添加其他消息...
            }
        }
    }
}

void MavlinkProtocolHandler::sendMessageLoop() {
    while (running.load()) {
        mavlink_message_t msg;
        if (messageQueue.pop(msg)) {
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
            if (transport_ && transport_->is_connected()) {
                transport_->send(buffer, len);//发送消息
            }
        }
        else {
            std::this_thread::yield();// 让出CPU资源
        }
    }
}

// ===== 高层命令封装 =====
/*以下是常见的 ArduPlane（固定翼）飞行模式值：

       模式名称	值
       MANUAL	0
       STABILIZE	2
       FBWA	5
       CRUISE	7
       AUTO	10
       RTL	11
       GUIDED	15
       LOITER	12
       20*/
bool MavlinkProtocolHandler::setFlightMode(uint8_t flight_mode) {
    mavlink_message_t cmd;
    bool packed = mavlink_msg_command_long_pack(
        system_id_, component_id_, &cmd,
        target_system_, target_component_,
        MAV_CMD_DO_SET_MODE,
        0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        flight_mode,
        0, 0, 0, 0, 0
    );
    if (!packed) return false;

    return messageQueue.push(cmd);
}

bool MavlinkProtocolHandler::setMissionCurrent(uint16_t seq) {
    mavlink_message_t cmd;
    bool packed = mavlink_msg_command_long_pack(
        system_id_, component_id_, &cmd,
        target_system_, target_component_,
        MAV_CMD_DO_SET_MISSION_CURRENT,
        0,
        seq,           // param1: 要设为当前的航点序号
        0, 0, 0, 0, 0, 0
    );
    if (!packed) return false;
    AppLogger::get().info("[MISSION] 设置当前航点 seq={}", seq);
    return messageQueue.push(cmd);
}

void MavlinkProtocolHandler::sendServoPWM(uint8_t servo_n, uint16_t pwm_value) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        system_id_, component_id_, &msg,
        target_system_, target_component_,
        MAV_CMD_DO_SET_SERVO,
        0,
        servo_n,
        pwm_value,
        0, 0, 0, 0, 0
    );
	AppLogger::get().info("发送舵机PWM命令: 舵机编号={}, PWM值={}", (int)servo_n, pwm_value);
    messageQueue.push(msg);
}
void MavlinkProtocolHandler::setup_guided_position_gps(mavlink_message_t& msg, double  lat_deg, double  lon_deg, float  alt_amsl, float yaw) {
    uint32_t time_boot_ms = 0; // 可选：使用实际时间戳

    // 设置类型掩码：只使用位置（lat, lon, alt），忽略速度、加速度与 yaw_rate；yaw 取飞控当前姿态航向
    uint16_t type_mask = (POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE);
    //关键：转换为 degE7（int32_t）
    int32_t lat_e7 = static_cast<int32_t>(lat_deg * 1e7);
    int32_t lon_e7 = static_cast<int32_t>(lon_deg * 1e7);

    // 读取飞控当前姿态的 yaw（弧度），保持机头当前航向，不再硬编码 initial_heading
    float target_yaw_rad = static_cast<float>(yaw * M_PI / 180.0);  // 兜底：调用方传入的 yaw（度）
    mavlink_attitude_t att;
    if (try_get_imu_data(att)) {
        target_yaw_rad = att.yaw;  // 优先用飞控姿态 yaw（弧度）
    }
    // 使用 RELATIVE_ALT 坐标系（相对于起飞点高度，不依赖地形数据）
    mavlink_msg_set_position_target_global_int_pack(
        system_id_,
        component_id_,
        &msg,
        time_boot_ms,
        target_system_,
        target_component_,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,        // 相对起飞点高度
        type_mask,
        lat_e7,                     // 直接传入 int32_t lat (deg * 1e7)
        lon_e7,                // 直接传入 int32_t lon (deg * 1e7)
        alt_amsl,                  // 高度转为米
        0, 0, 0,                     // vx, vy, vz (ignored)
        0, 0, 0,                     // ax, ay, az (ignored)
        target_yaw_rad, 0                         // yaw（当前姿态航向）, yaw_rate（ignored）
    );
}
// ===== 获取状态 =====

#define TRY_GET_IMPL(var, mutex, out) \
    std::lock_guard<std::mutex> lock(mutex); \
    if (var.has_value()) { out = *var; return true; } \
    return false;
bool MavlinkProtocolHandler::try_get_mission_ack_data(mavlink_mission_ack_t& mission_ack_data) {
    std::lock_guard<std::mutex> lock(mission_ack_data_mutex_);
    if (latest_mission_ack_data_.has_value()) {
        mission_ack_data = *latest_mission_ack_data_;

        return true;
    }
    return false;
}
bool MavlinkProtocolHandler::try_get_mission_request_int_data(mavlink_mission_request_int_t& request_data) {
    std::lock_guard<std::mutex> lock(mission_request_int_data_mutex_);
    if (latest_mission_request_int_data_.has_value()) {
        request_data = *latest_mission_request_int_data_;

        return true;
    }
    return false;
}
bool MavlinkProtocolHandler::try_get_mission_request_data(mavlink_mission_request_t& request_data) {
    std::lock_guard<std::mutex> lock(mission_request_data_mutex_);
    if (latest_mission_request_data_.has_value()) {
        request_data = *latest_mission_request_data_;

        return true;
    }
    return false;
}

bool MavlinkProtocolHandler::try_get_gps_data(mavlink_global_position_int_t& out) {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    if (latest_gps_data_.has_value()) {
        out = *latest_gps_data_;
		//std::cout << " GPS数据: lat=" << out.lat / 1e7 << "°, lon=" << out.lon / 1e7 << "°, relative_alt=" << out.relative_alt / 1000.0f << "m" << std::endl;
        return true;
    }
    return false;
}

bool MavlinkProtocolHandler::try_get_gps_raw_data(mavlink_gps_raw_int_t& out) {
    std::lock_guard<std::mutex> lock(gps_raw_data_mutex_);
    if (latest_gps_raw_data_.has_value()) {
        out = *latest_gps_raw_data_;
        return true;
    }
    return false;
}

bool MavlinkProtocolHandler::try_get_imu_data(mavlink_attitude_t& out) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    if (latest_imu_data_.has_value()) {
        out = *latest_imu_data_;
        return true;
    }
    return false;
}

bool MavlinkProtocolHandler::try_get_rc_channels(mavlink_rc_channels_t& out) {
    std::lock_guard<std::mutex> lock(rc_channels_mutex_);
    if (latest_rc_channels_.has_value()) {
        out = *latest_rc_channels_;
        return true;
    }
    return false;
}

bool MavlinkProtocolHandler::try_get_flight_mode(FlightModeData& out) {
    std::lock_guard<std::mutex> lock(flight_mode_mutex_);
    if (latest_flight_mode_.has_value()) {
        out = *latest_flight_mode_;
        return true;
    }
    return false;
}

bool MavlinkProtocolHandler::try_get_home_position(HomePositionData& out) {
    std::lock_guard<std::mutex> lock(home_position_mutex_);
    if (latest_home_position_.has_value()) {
        out = *latest_home_position_;
        return true;
    }
    return false;
}

bool MavlinkProtocolHandler::try_get_distance_sensor_data(mavlink_distance_sensor_t& out) {
    std::lock_guard<std::mutex> lock(distance_sensor_mutex_);
    if (latest_distance_sensor_.has_value()) {
        out = *latest_distance_sensor_;
        return true;
    }
    return false;
}

bool MavlinkProtocolHandler::try_get_latest_mission_current(mavlink_mission_current_t& out) {
    if (has_new_mission_current_) {
        std::lock_guard<std::mutex> lock(mission_current_mutex_);
        out = latest_mission_current_;
        has_new_mission_current_ = false;
        return true;
    }
    return false;
}

void MavlinkProtocolHandler::clear_mission_current() {
    has_new_mission_current_ = false;
}

// ==================== 心跳监控 ====================

uint64_t MavlinkProtocolHandler::getHeartbeatAgeMs() const {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    if (latest_heartbeat_time_ == std::chrono::steady_clock::time_point{}) {
        return UINT64_MAX;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - latest_heartbeat_time_).count();
}

double MavlinkProtocolHandler::getHeartbeatRate() const {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    if (heartbeat_interval_ms_ <= 0) return 0.0;
    return 1000.0 / heartbeat_interval_ms_;
}

bool MavlinkProtocolHandler::isAutopilotIdentified() const {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    return autopilot_identified_;
}

bool MavlinkProtocolHandler::try_get_heartbeat_info(uint8_t& autopilot, uint8_t& type, uint8_t& system_status) const {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    if (!autopilot_identified_) return false;
    autopilot = identified_autopilot_;
    type = identified_type_;
    system_status = latest_system_status_;
    return true;
}

bool MavlinkProtocolHandler::is_armed() const {
    std::lock_guard<std::mutex> lock(flight_mode_mutex_);
    if (!latest_flight_mode_.has_value()) return false;
    return (latest_flight_mode_->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
}

// ==================== 航点与导航方法 ====================

void MavlinkProtocolHandler::requestMessageInterval(uint16_t msg_id, float hz) {
    double time_us = 1e6 / hz;
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        system_id_, component_id_, &msg,
        target_system_, target_component_,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0, msg_id, time_us,
        0, 0, 0, 0, 0
    );
    sendMessage(msg);
    mavlink_msg_command_long_pack(
        system_id_, component_id_, &msg,
        target_system_, target_component_,
        MAV_CMD_GET_MESSAGE_INTERVAL,
        0, msg_id, 0,
        0, 0, 0, 0, 0
    );
    sendMessage(msg);
}

void MavlinkProtocolHandler::requestAllDataStreams() {
    mavlink_message_t msg{};
    // 关闭所有自动消息流
    mavlink_msg_request_data_stream_pack(
        system_id_, component_id_, &msg,
        target_system_, target_component_,
        MAV_DATA_STREAM_ALL, 0, 0
    );
    sendMessage(msg);
	requestGpsDataStreams();// GPS数据流重新请求
    requestMessageInterval(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    requestMessageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    requestMessageInterval(MAVLINK_MSG_ID_DISTANCE_SENSOR, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    requestMessageInterval(MAVLINK_MSG_ID_MISSION_CURRENT, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void MavlinkProtocolHandler::requestGpsDataStreams() {
    requestMessageInterval(MAVLINK_MSG_ID_GPS_RAW_INT, 5);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5);
}

void MavlinkProtocolHandler::sendSetGlobalOrigin(int32_t lat_e7, int32_t lon_e7, int32_t alt_mm) {
    mavlink_message_t msg;
    mavlink_msg_set_gps_global_origin_pack(
        system_id_, component_id_, &msg,
        target_system_,
        lat_e7, lon_e7, alt_mm,
        0  // time_usec: 0 lets FC determine timestamp
    );
    sendMessage(msg);
    AppLogger::get().info("[ORIGIN] SET_GPS_GLOBAL_ORIGIN sent: lat={:.7f} lon={:.7f} alt={}mm", lat_e7 / 1e7, lon_e7 / 1e7, alt_mm);
}
void MavlinkProtocolHandler::sendSetHomePosition() {
	mavlink_message_t msg;
	mavlink_msg_command_long_pack(
	    system_id_, component_id_, &msg,
	    target_system_, target_component_,
	    MAV_CMD_DO_SET_HOME,
	    0,  // confirmation
	    1,  // param1: 1 = use current position
	    0, 0, 0,  // param2-4: unused
	    0, 0, 0   // param5-7: unused (lat, lon, alt)
	);
	sendMessage(msg);
	AppLogger::get().info("[HOME] MAV_CMD_DO_SET_HOME sent (use current position)");
}

std::vector<mavlink_mission_item_t> MavlinkProtocolHandler::buildMissionWaypoints(
    double takeoff_landing_lat,
    double takeoff_landing_lon,
    double takeoff_heading
) {
    std::vector<mavlink_mission_item_t> waypoints;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    auto base = [&]() {
        mavlink_mission_item_t it{};
        it.target_system = target_system_;
        it.target_component = target_component_;
        it.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        it.autocontinue = true;
        return it;
    };

    // 0. 起降区 HOME 点（用 RTK 实测坐标）
    mavlink_mission_item_t item = base();
    item.seq = 0;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.current = true;
    item.x = takeoff_landing_lat;
    item.y = takeoff_landing_lon;
    item.z = 0.0f;
    waypoints.push_back(item);

    // 1. 手抛起飞爬升（ArduPlane TAKEOFF）
    item = base();
    item.seq = 1;
    item.command = MAV_CMD_NAV_TAKEOFF;
    item.param1 = static_cast<float>(cfg_.fixedwing.takeoff_pitch_deg);   // 俯仰角 18°
    item.param4 = static_cast<float>(takeoff_heading);                     // 手抛方向（与航线基准一致）
    item.z = static_cast<float>(cfg_.fixedwing.takeoff_climb_alt_m);      // 爬升高度 38m
    waypoints.push_back(item);

    // 2..N：循环 config 航线表，刚体变换（相对起飞航向 + 相对 HOME）
    uint16_t seq = 2;
    for (size_t i = 0; i < cfg_.fixedwing.route.size(); ++i) {
        const auto& r = cfg_.fixedwing.route[i];
        double lat, lon;
        geod.Direct(takeoff_landing_lat, takeoff_landing_lon,
                    takeoff_heading + r.bearing_offset_deg, r.distance_m, lat, lon);
        item = base();
        item.seq = seq++;
        item.command = (i == cfg_.fixedwing.route.size() - 1) ? MAV_CMD_NAV_LAND : MAV_CMD_NAV_WAYPOINT;
        item.x = lat;
        item.y = lon;
        item.z = static_cast<float>(r.alt_m);
        waypoints.push_back(item);
    }

    return waypoints;
}

bool MavlinkProtocolHandler::uploadMissionWaypoints(
    double initial_lat, double initial_lon, double takeoff_heading
) {
    return uploadMissionItems(buildMissionWaypoints(initial_lat, initial_lon, takeoff_heading));
}

bool MavlinkProtocolHandler::uploadFallbackMission(
    double initial_lat, double initial_lon, double takeoff_heading
) {
    return uploadMissionItems(buildFallbackMissionWaypoints(initial_lat, initial_lon, takeoff_heading));
}

std::vector<mavlink_mission_item_t> MavlinkProtocolHandler::buildFallbackMissionWaypoints(
    double takeoff_landing_lat,
    double takeoff_landing_lon,
    double takeoff_heading
) {
    std::vector<mavlink_mission_item_t> waypoints;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    auto base = [&]() {
        mavlink_mission_item_t it{};
        it.target_system = target_system_;
        it.target_component = target_component_;
        it.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
        it.autocontinue = true;
        return it;
    };

    // 兜底航线：route[fallback_start_index]（巡航6）起到尾；巡航7 之后紧跟 DO_SET_SERVO 投弹
    const auto& route = cfg_.fixedwing.route;
    int start = cfg_.fixedwing.fallback_start_index;
    if (start < 0) start = 0;
    if (start >= static_cast<int>(route.size())) start = static_cast<int>(route.size()) - 1;

    uint16_t seq = 0;
    for (int i = start; i < static_cast<int>(route.size()); ++i) {
        const auto& r = route[i];
        double lat, lon;
        geod.Direct(takeoff_landing_lat, takeoff_landing_lon,
                    takeoff_heading + r.bearing_offset_deg, r.distance_m, lat, lon);
        mavlink_mission_item_t item = base();
        item.seq = seq++;
        item.command = (i == static_cast<int>(route.size()) - 1) ? MAV_CMD_NAV_LAND : MAV_CMD_NAV_WAYPOINT;
        item.x = lat;
        item.y = lon;
        item.z = static_cast<float>(r.alt_m);
        waypoints.push_back(item);

        // 巡航7（fallback_start_index+1）之后紧跟一条 DO_SET_SERVO 投弹
        if (i == start + 1) {
            mavlink_mission_item_t servo = base();
            servo.seq = seq++;
            servo.command = MAV_CMD_DO_SET_SERVO;
            servo.param1 = static_cast<float>(cfg_.fixedwing.servo_channel);
            servo.param2 = static_cast<float>(cfg_.fixedwing.servo_release_pwm);
            waypoints.push_back(servo);
        }
    }

    return waypoints;
}

bool MavlinkProtocolHandler::uploadMissionItems(const std::vector<mavlink_mission_item_t>& waypoints) {
    // 1. 发送 MISSION_COUNT
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(
        system_id_, component_id_, &msg,
        target_system_, target_component_,
        static_cast<uint16_t>(waypoints.size()),
        MAV_MISSION_TYPE_MISSION, 0
    );
    sendMessage(msg);
    AppLogger::get().info("已发送 MISSION_COUNT = {}", static_cast<uint16_t>(waypoints.size()));

    uint16_t total_count = static_cast<uint16_t>(waypoints.size());
    const int REQUEST_TIMEOUT_MS = 2000;
    const int SLEEP_BETWEEN_SEND_MS = 50;

    // 2. 循环：等待 MISSION_REQUEST 并发送对应航点
    for (uint16_t expected_seq = 0; expected_seq < total_count; ) {
        bool request_received = false;
        auto start_time = std::chrono::steady_clock::now();

        while (true) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            if (elapsed.count() > REQUEST_TIMEOUT_MS) break;

            mavlink_mission_request_t request;
            if (try_get_mission_request_data(request)) {
                if (request.target_system == system_id_ &&
                    request.target_component == component_id_ &&
                    request.mission_type == MAV_MISSION_TYPE_MISSION) {
                    if (request.seq == expected_seq) {
                        request_received = true;
                        break;
                    } else {
                        uint16_t req_seq = request.seq;  // 拷贝局部变量，避免引用未对齐 packed 字段
                        AppLogger::get().debug("飞控请求序号 {}，期望 {}，跳过", req_seq, expected_seq);
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!request_received) {
            AppLogger::get().info("超时：未收到 MISSION_REQUEST for seq={}", expected_seq);
            if (expected_seq == 0) {
                mavlink_msg_mission_count_pack(
                    system_id_, component_id_, &msg,
                    target_system_, target_component_,
                    total_count, MAV_MISSION_TYPE_MISSION, 0
                );
                sendMessage(msg);
                AppLogger::get().info("重发 MISSION_COUNT");
            }
            continue;
        }

        mavlink_msg_mission_item_encode(system_id_, component_id_, &msg, &waypoints[expected_seq]);
        sendMessage(msg);
        uint16_t cmd_id = waypoints[expected_seq].command;  // 拷贝局部变量，避免引用未对齐 packed 字段
        AppLogger::get().info("发送航点 seq={} (cmd={})", expected_seq, cmd_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_BETWEEN_SEND_MS));
        expected_seq++;
    }

    // 3. 等待 MISSION_ACK
    {
        auto start_time = std::chrono::steady_clock::now();
        while (true) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            if (elapsed.count() > REQUEST_TIMEOUT_MS) {
                AppLogger::get().info("超时：未收到 MISSION_ACK");
                return false;
            }

            mavlink_mission_ack_t ack;
            if (try_get_mission_ack_data(ack)) {
                if (ack.target_system == system_id_ &&
                    ack.target_component == component_id_ &&
                    ack.mission_type == MAV_MISSION_TYPE_MISSION) {
                    if (ack.type == MAV_MISSION_ACCEPTED) {
                        AppLogger::get().info("航点任务上传成功！飞控返回: MAV_MISSION_ACCEPTED");
                        return true;
                    } else {
                        AppLogger::get().info("任务上传被拒绝，错误码: {}", static_cast<int>(ack.type));
                        return false;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return true;
}

// 析构函数（确保线程退出）

MavlinkProtocolHandler::~MavlinkProtocolHandler() {
    running = false;
    if (senderThread.joinable()) {
        senderThread.join();
    }
}
