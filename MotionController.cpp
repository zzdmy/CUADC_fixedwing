#include "MotionController.h"
#include <cmath>
#include <utility>

MotionController::MotionController(std::shared_ptr<MavlinkProtocolHandler> mavlinkHandler)
    : mavlink_handler_(std::move(mavlinkHandler)) {
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

void MotionController::runControlLoop() {
    const auto interval = std::chrono::milliseconds(1000 / CONTROL_FREQUENCY_HZ_);
    mavlink_message_t msg;

    while (is_running_.load()) {
        auto start_time = std::chrono::steady_clock::now();
        CommandMode mode = CommandMode::NONE;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            mode = current_command_mode_.load();
        }

        if (mode == CommandMode::GPS_POSITION) {
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
