// CaptureThread.h
#pragma once

#include <thread>
#include <atomic>
#include <string>
#include <opencv2/opencv.hpp>
#include "FrameDispatcher.h"


class CaptureThread {
public:
    struct Config {
        int width = 1920;
        int height = 1080;
        int fps = 60;
        bool useMjpg = true;

        // --- 软件 AE 配置 ---
        bool ae_enable = true;
        double ae_target_brightness = 0.4;
        int ae_exposure_min = -10;
        int ae_exposure_max = -2;
        double ae_kp = 0.3;
        double ae_ki = 0.02;
        int ae_update_interval = 5;          // 调节冷却帧数
        double ae_deadband = 0.15;           // 亮度死区
    };

    explicit CaptureThread(int cameraIndex, const Config& config = {});
    ~CaptureThread();

    void start();
    void stop();
    bool isRunning() const;
    double getCaptureFPS() const;

private:
    void run();

    int cameraIndex_;
    Config config_;

    cv::VideoCapture cap_;

    std::thread thread_;
    std::atomic<bool> running_{ false };
    std::atomic<double> capture_fps_{ 0.0 };

    // --- AE 状态 ---
    int current_exposure_{ -6 };
    double ae_integral_{ 0.0 };
    int ae_cooldown_{ 0 };                   // 冷却计数器（递减到0后允许下次调节）
};
