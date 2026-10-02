#pragma once
#include "IDetector.h"
#include <string>
#include <thread>
#include <atomic>
#include <stop_token> // C++20

class YoloV8TensorRT; // 前向声明

class TensorRTDetector : public IDetector {
public:
    TensorRTDetector(const std::string& enginePath, const std::string& classPath, int frameStride = 5);
    ~TensorRTDetector();

    void startDetectionLoop(int width, int height) override;
    void stop() override;
    bool isInitialized() const override;
    bool isRunning() const override { return m_running.load(); }

private:
    std::unique_ptr<YoloV8TensorRT> m_impl;
    std::jthread m_thread;               // C++20: 自动 join
    std::atomic<bool> m_running{ false };
    int m_frame_stride = 5;              // 抽帧：每 N 帧推理一次
};