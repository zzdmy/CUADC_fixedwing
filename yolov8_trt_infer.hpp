#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <vector>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <stop_token> // C++20
#include "yolodecet/inference.h"
#include "FrameDispatcher.h"

// ==================== 日志器 ====================
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override;
};

// ==================== YOLOv8 配置参数 ====================
namespace YoloConfig {
    constexpr int INPUT_H = 640;
    constexpr int INPUT_W = 640;
    constexpr int CHANNELS = 3;
    constexpr int BATCH_SIZE = 1;

    constexpr float CONF_THRESHOLD = 0.25f;
    constexpr float Score_THRESHOLD = 0.45f;
    constexpr float NMS_THRESHOLD = 0.5f;

    constexpr const char* INPUT_NAME_T = "images";
    constexpr const char* OUTPUT_NAME_T = "output0";
}

// ==================== TensorRT推理类 ====================
class YoloV8TensorRT {
public:
    explicit YoloV8TensorRT(const std::string& engine_file_path, const std::string& classes_path);
    void detectionLoop(int actualWidth, int actualHeight, int frame_stride, std::stop_token st = {});
    ~YoloV8TensorRT();

    std::vector<Detection> infer(const cv::Mat& frame, int orig_img_w, int orig_img_h);
    bool isInitialized() const { return m_ready; }
    void loadClassesFromFile();

private:
    YoloV8TensorRT(const YoloV8TensorRT&) = delete;
    YoloV8TensorRT& operator=(const YoloV8TensorRT&) = delete;

    bool loadEngine(const std::string& file_path, const std::string& classes_path);

private:
    Logger m_logger;
    nvinfer1::IRuntime* m_runtime = nullptr;
    nvinfer1::ICudaEngine* m_engine = nullptr;
    nvinfer1::IExecutionContext* m_context = nullptr;

    void* m_device_input = nullptr;
    void* m_device_output = nullptr;
    size_t m_input_bytes = 0;
    size_t m_output_bytes = 0;

    std::vector<float> m_host_output; // pre-allocated output buffer

    cudaStream_t m_stream = 0; // 专用 CUDA stream

    // pre-allocated CPU Mats to avoid per-frame allocation
    cv::Mat m_resized;      // resize result
    cv::Mat m_letterboxed;  // 模型输入尺寸 CV_8UC3 (letterbox)
    cv::Mat m_float;        // 模型输入尺寸 CV_32FC3
    cv::Mat m_rgb;          // 模型输入尺寸 CV_32FC3 (RGB)

    // pre-allocated split channel outputs (模型输入尺寸 CV_32FC1 each)
    cv::Mat m_channel_r;
    cv::Mat m_channel_g;
    cv::Mat m_channel_b;

    int m_input_w = 640;    // 模型输入宽（从引擎读取；天井1280 / 图案640）
    int m_input_h = 640;    // 模型输入高

    bool m_ready = false;
    std::string classesPath{};
    std::vector<std::string> classes;
};