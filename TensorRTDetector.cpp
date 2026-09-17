#include "TensorRTDetector.h"
#include "yolov8_trt_infer.hpp"

TensorRTDetector::TensorRTDetector(const std::string& enginePath, const std::string& classPath) {
    m_impl = std::make_unique<YoloV8TensorRT>(enginePath, classPath);
}

TensorRTDetector::~TensorRTDetector() {
    stop(); // jthread 析构会自动 join
}

void TensorRTDetector::startDetectionLoop(int width, int height) {
    if (!isInitialized() || m_running) return;
    m_running = true;
    m_thread = std::jthread([this, width, height](std::stop_token st) {
        m_impl->detectionLoop(width, height, std::move(st));
        });
}

void TensorRTDetector::stop() {
    m_running = false;
    // std::jthread 会在析构时自动 request_stop + join
}

bool TensorRTDetector::isInitialized() const {
    return m_impl && m_impl->isInitialized();
}