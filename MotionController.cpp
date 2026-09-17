#include "MotionController.h"
#include "AppLogger.h"
#include "WatchdogHelper.h"
#include "ConfigManager.h"
#include "config_loader.h"
#include <cmath>
#include <algorithm>
#define GEOGRAPHICLIB_SHARED_LIB 1
#include <GeographicLib/Geodesic.hpp>

MotionController::MotionController(std::shared_ptr<MavlinkProtocolHandler> mavlinkHandler)
    : mavlink_handler_(std::move(mavlinkHandler)) {
    const auto& cfg = ConfigManager::getInstance().getConfig();
    kP_GAIN_ = cfg.control.kp;
    MAX_SPEED_MPS_ = cfg.control.max_speed;
}

MotionController::~MotionController() {
    stop();
}

void MotionController::start() {
    if (!is_running_.exchange(true)) {
        control_thread_ = std::thread(&MotionController::runControlLoop, this);
    }
}

void MotionController::stop() {
    if (is_running_.exchange(false)) {
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
    }
}

void MotionController::setVelocityCommand(float vx, float vy, float vz) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    velocity_vx_ = vx;
    velocity_vy_ = vy;
    velocity_vz_ = vz;
    current_command_mode_.store(CommandMode::VELOCITY);
}

void MotionController::setVisualServoTarget(float pixel_x, float pixel_y, float vz_mps) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    visual_target_x_ = pixel_x;
    visual_target_y_ = pixel_y;
    visual_target_vz_ = vz_mps;
    has_visual_target_ = true;
    current_command_mode_.store(CommandMode::VISUAL_SERVO);
}

void MotionController::setPositionTarget(float x, float y, float z, float yaw_deg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 检查位置是否发生变化
    bool position_changed = (std::abs(position_target_x_ - x) > POSITION_EPSILON_) ||
        (std::abs(position_target_y_ - y) > POSITION_EPSILON_) ||
        (std::abs(position_target_z_ - z) > POSITION_EPSILON_) ||
        (std::abs(position_target_yaw_ - yaw_deg) > YAW_EPSILON_);

    if (position_changed || current_command_mode_.load() != CommandMode::POSITION) {
        position_target_x_ = x;
        position_target_y_ = y;
        position_target_z_ = z;
        position_target_yaw_ = yaw_deg;
        position_command_pending_ = true; // 标记需要发送新位置指令
        current_command_mode_.store(CommandMode::POSITION);
    }
}

void MotionController::enableTocMode(bool enable) {
    is_toc_mode_.store(enable);
}

void MotionController::setImageResolution(int width, int height) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    image_width_ = width;
    image_height_ = height;
}



void MotionController::setGpsPositionTarget(double lat_deg, double lon_deg, float altitude_m, float yaw_deg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 检查GPS位置是否发生变化
    bool gps_changed = (std::abs(gps_target_lat_ - lat_deg) > GPS_EPSILON_) ||
        (std::abs(gps_target_lon_ - lon_deg) > GPS_EPSILON_) ||
        (std::abs(gps_target_alt_ - altitude_m) > POSITION_EPSILON_) ||
        (std::abs(gps_target_yaw_ - yaw_deg) > YAW_EPSILON_);

    if (gps_changed || current_command_mode_.load() != CommandMode::GPS_POSITION) {
        gps_target_lat_ = lat_deg;
        gps_target_lon_ = lon_deg;
        gps_target_alt_ = altitude_m;
        gps_target_yaw_ = yaw_deg;
        gps_command_pending_ = true; // 标记需要发送新GPS指令
        current_command_mode_.store(CommandMode::GPS_POSITION);
    }
}

bool MotionController::flyToGpsAndWait(double target_lat_deg, double target_lon_deg, double altitude_m)
{
    const auto& cfg = ConfigManager::getInstance().getConfig();
    const double distance_threshold_m = cfg.control.gps_arrival_threshold_m;
    const float altitude_threshold_m = 0.5f;
    const int timeout_ms = cfg.control.gps_fly_timeout_ms;

    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
    auto stable_start_time = std::chrono::steady_clock::time_point::max();

    // Set GPS target; control loop will keep sending the command
    setGpsPositionTarget(target_lat_deg, target_lon_deg, static_cast<float>(altitude_m));

    AppLogger::get().info("Flying to GPS target: lat={}, lon={} alt={}m", target_lat_deg, target_lon_deg, altitude_m);

    auto result = WatchdogHelper::waitUntil(
        [this, &geod, target_lat_deg, target_lon_deg, altitude_m,
            distance_threshold_m, altitude_threshold_m, &stable_start_time]() {
            // 使用 EKF 融合位置（比原始 GPS 更平滑）
            mavlink_global_position_int_t current_pos;
            if (mavlink_handler_->try_get_gps_data(current_pos)) {
                double current_lat = static_cast<double>(current_pos.lat) / 1e7;
                double current_lon = static_cast<double>(current_pos.lon) / 1e7;
                float current_alt = current_pos.relative_alt / 1000.0f;

                double horizontal_error;
                geod.Inverse(target_lat_deg, target_lon_deg, current_lat, current_lon, horizontal_error);
                float altitude_error = std::abs(current_alt - static_cast<float>(altitude_m));

                AppLogger::get().debug("Distance to target: h={:.3f}m v={:.3f}m", horizontal_error, altitude_error);

                // 持续在阈值内 500ms 才确认到达
                bool in_zone = horizontal_error < distance_threshold_m && altitude_error < altitude_threshold_m;
                if (in_zone) {
                    if (stable_start_time == std::chrono::steady_clock::time_point::max()) {
                        stable_start_time = std::chrono::steady_clock::now();
                    }
                    return (std::chrono::steady_clock::now() - stable_start_time) >= std::chrono::milliseconds(200);
                } else {
                    stable_start_time = std::chrono::steady_clock::time_point::max();
                }
            }
            return false;
        },
        is_running_,
        std::chrono::milliseconds(timeout_ms),
        std::chrono::milliseconds(200));

    if (result == WaitResult::ConditionMet) {
        AppLogger::get().info("Target reached, exiting wait.");
        return true;
    }
    if (result == WaitResult::Timeout) {
        AppLogger::get().warn("GPS fly-to timeout!");
    } else {
        AppLogger::get().info("GPS fly-to stopped.");
    }
    return false;
}

void MotionController::runControlLoop() {
    const auto interval = std::chrono::milliseconds(1000 / CONTROL_FREQUENCY_HZ_);
    mavlink_message_t msg;

    while (is_running_.load()) {
        if (is_toc_mode_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        auto start_time = std::chrono::steady_clock::now();
        CommandMode mode = CommandMode::NONE;
        bool position_pending = false;
        bool gps_pending = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            mode = current_command_mode_.load();
            position_pending = position_command_pending_;
            gps_pending = gps_command_pending_;
        }

        // 优先级: POSITION > VELOCITY > VISUAL_SERVO
        if (mode == CommandMode::POSITION) {
            // 持续发送位置指令，否则飞控3秒超时会停止
            mavlink_handler_->setup_guided_position_body(
                msg,
                position_target_x_,
                position_target_y_,
                position_target_z_,
                position_target_yaw_
            );
            mavlink_handler_->sendMessage(msg);

            std::lock_guard<std::mutex> lock(state_mutex_);
            position_command_pending_ = false;
        }
        else if (mode == CommandMode::VELOCITY) {
            // 速度命令需要持续发送
            mavlink_handler_->setup_guided_velocity(
                msg,
                velocity_vx_,
                velocity_vy_,
                velocity_vz_,
                0.0f
            );
            mavlink_handler_->sendMessage(msg);
        }
        else if (mode == CommandMode::VISUAL_SERVO) {
            // 视觉伺服需要持续发送
            float vx = 0.0f, vy = 0.0f, vz = 0.0f;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                float dx = visual_target_x_ - static_cast<float>(image_width_) / 2.0f;
                float dy = static_cast<float>(image_height_) / 2.0f - visual_target_y_; // Y轴翻转

                vx = std::clamp(dx * kP_GAIN_ * doublefast.load(), -MAX_SPEED_MPS_, MAX_SPEED_MPS_);
                vy = std::clamp(dy * kP_GAIN_ * doublefast.load(), -MAX_SPEED_MPS_, MAX_SPEED_MPS_);
                vz = visual_target_vz_;

                float height_scale = 300.0f / 150.0f;//
                vx *= height_scale;
                vy *= height_scale;
            }

            // 交换坐标: 视觉计算的vx(右向)→vy, vy(前向)→vx
            std::swap(vx, vy);

            // 发送速度指令 (机体坐标系)
            mavlink_handler_->setup_guided_velocity(
                msg,
                vx,
                vy,
                vz,
                0.0f
            );
            mavlink_handler_->sendMessage(msg);
        }
        else if (mode == CommandMode::GPS_POSITION) {
            // 持续发送GPS位置指令，否则飞控3秒超时会停止
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                mavlink_handler_->setup_guided_position_gps(
                    msg,
                    gps_target_lat_,
                    gps_target_lon_,
                    gps_target_alt_,
                    gps_target_yaw_
                );
            }
            mavlink_handler_->sendMessage(msg);

            std::lock_guard<std::mutex> lock(state_mutex_);
            gps_command_pending_ = false;
        }
        else {
            continue;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (auto sleep_time = interval - elapsed; sleep_time > std::chrono::microseconds::zero()) {
            std::this_thread::sleep_for(sleep_time);
        }
    }
}
