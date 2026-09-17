// MissionScheduler.h
#pragma once
#ifndef MISSION_SCHEDULER_H
#define MISSION_SCHEDULER_H

#include <atomic>
#include <mutex>
#include <thread>
#include "MavlinkProtocolHandler.h"
#include "CaptureThread.h"
#include "yolodecet/yolo_tracker.h"
#include "MotionController.h"
#include <memory>
#include "ConfigManager.h"
#include "PixelToGPSConverter.h"
using namespace cv;
using namespace std;

bool try_get_latest_image(Mat& latest_image);
class SerialPortHandler;

// Global time stored as nanoseconds
static std::atomic<std::chrono::nanoseconds::rep> global_time_ns;

// Mission state enum
enum class MissionState {
    WaitingForInitialization,
    TakingOff,
    Mission1InProgress,
    LandingInProgress
};


class MissionScheduler {
public:
    explicit MissionScheduler(std::shared_ptr<MavlinkProtocolHandler> mavlinkHandler,
                               std::shared_ptr<PixelToGPSConverter> cameraConverter);
    ~MissionScheduler();

    void start();
    void stop();
    MissionState getCurrentState() const;

    // 侦察模型切换：MissionScheduler 请求 → 主线程执行
    bool isReconModelRequested() const { return switch_to_recon_requested_.load(); }
    void setReconModelReady(bool v) { recon_model_ready_.store(v); switch_to_recon_requested_.store(false); }

private:
    const AppConfig& cfg_;
    double home_lat_ = 0.0;
    double home_lon_ = 0.0;
    double takeoff_heading_ = 0.0;   // 已解析的起飞真北航向（自动读或手动），供上传与兜底点共用
    std::vector<TargetGPS> target_gps_list_;
    bool data_stream_requested_ = true;

    void runScheduler();
    void handleWaitingState();
    void handleTakeoffState();
    void handleMission1State();
    void handleLandingState();
    void handleManualObserve();   // 测试用: 手动飞行、仅检测+输出

    std::shared_ptr<MavlinkProtocolHandler> mavlink_handler_;
    std::shared_ptr<PixelToGPSConverter> camera_converter_;
    std::thread scheduler_thread_;
    std::atomic<bool> running_{ false };
    std::atomic<MissionState> current_state_{ MissionState::WaitingForInitialization };

    vector<yoloout> latest_frame_;
    mutable std::mutex frame_mutex_;
    bool new_frame_available_{ false };
    std::unique_ptr<MotionController> motion_ctrl_;

    // 侦察模型切换标志
    std::atomic<bool> switch_to_recon_requested_{ false };
    std::atomic<bool> recon_model_ready_{ false };
};

#endif // MISSION_SCHEDULER_H