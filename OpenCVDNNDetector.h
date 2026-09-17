#pragma once
#include "IDetector.h"
#include <string>
#include <thread>
#include <atomic>
#include <stop_token> // C++20

class Inference; // 前向声明

class OpenCVDNNDetector : public IDetector {
public:
    OpenCVDNNDetector(const std::string& modelPath, const std::string& classPath, bool useGPU);
    ~OpenCVDNNDetector();

    void startDetectionLoop(int width, int height) override;
    void stop() override;
    bool isInitialized() const override;
    bool isRunning() const override { return m_running.load(); }

private:
    void detectionThread(std::stop_token st);

    std::string m_modelPath;
    std::string m_classPath;
    bool m_useGPU;
    std::unique_ptr<Inference> m_infer;
    std::jthread m_thread;               // ← C++20 jthread
    std::atomic<bool> m_running{ false };
    bool m_initialized = false;
};