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

class YoloV8TensorRT; // 前向声明（任务一第二级图案检测器）
using namespace cv;
using namespace std;

bool try_get_latest_image(Mat& latest_image);

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

    // 任务一第二级图案检测器（任务二为空）；主线程创建后注入
    void setPatternDetector(std::shared_ptr<YoloV8TensorRT> p) { pattern_detector_ = std::move(p); }

private:
    const AppConfig& cfg_;
    double home_lat_ = 0.0;
    double home_lon_ = 0.0;
    double takeoff_heading_ = 0.0;   // 已解析的起飞真北航向（自动读或手动），供上传与兜底点共用
    int landing_last_seq_ = 0;       // 降落监控终点 seq（正常流程=1+route.size()，兜底流程=route.size()-fallback_start_index）
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

    // 任务一第二级图案检测器（可空；任务二不使用）
    std::shared_ptr<YoloV8TensorRT> pattern_detector_;
};

#endif // MISSION_SCHEDULER_H