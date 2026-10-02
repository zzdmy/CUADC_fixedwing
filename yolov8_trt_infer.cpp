// File: yolov8_trt_infer.cpp
#include "yolov8_trt_infer.hpp"

#include <fstream>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include "AppLogger.h"

#include "realtime_pipeline.h"

void Logger::log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept {
    if (severity != nvinfer1::ILogger::Severity::kINFO) {
        AppLogger::get().info("[TensorRT] {}", msg);
    }
}
using namespace std;
void YoloV8TensorRT::detectionLoop(int actualWidth, int actualHeight, int frame_stride, std::stop_token st) {
    init_pipeline();
    if (frame_stride < 1) frame_stride = 1;
    uint64_t last_frame_id = 0;       // 上一帧编号（去重）
    uint64_t last_processed_id = 0;   // 上次推理的帧编号（抽帧）

    thread_local std::mt19937 gen{ std::random_device{}() };//创建一个随机数生成器
    std::uniform_int_distribution<int> dis(100, 255);//随机数生成器

    while (!st.stop_requested()) {
        auto loop_start = std::chrono::high_resolution_clock::now(); //开始计时
        auto frame_ptr = frameDispatcher.getFrame();//获取最新帧
        if (frame_ptr && !frame_ptr->empty()) {
            uint64_t current_frame_id = frameDispatcher.getFrameCounter();//获取帧编号
            if (current_frame_id > last_frame_id) {
                last_frame_id = current_frame_id;
                // 抽帧：每 frame_stride 帧推理一次，防输入>输出导致数据堆积
                if (current_frame_id - last_processed_id < static_cast<uint64_t>(frame_stride)) {
                    continue;   // 跳过本帧，仅推进帧引用
                }
                last_processed_id = current_frame_id;
                const cv::Mat& frame = *frame_ptr; // 无需 clone

                auto detections = infer(frame, actualWidth, actualHeight);//推理

                // ===== 识别输出日志（限速 ≥400ms 一条）：检出明细，供飞行复盘 =====
                if (!detections.empty()) {
                    static auto last_det_log = std::chrono::steady_clock::time_point{};
                    auto now_log = std::chrono::steady_clock::now();
                    if (now_log - last_det_log >= std::chrono::milliseconds(400)) {
                        last_det_log = now_log;
                        std::ostringstream oss;
                        for (size_t i = 0; i < detections.size() && i < 8; ++i) {
                            const auto& d = detections[i];
                            oss << " " << (d.className.empty() ? "?" : d.className)
                                << " " << std::fixed << std::setprecision(2) << d.confidence
                                << "@(" << d.box.x << "," << d.box.y << ","
                                << d.box.width << "x" << d.box.height << ")";
                        }
                        AppLogger::get().info("[检测] 帧{} 检出{}个:{}",
                            current_frame_id, detections.size(), oss.str());
                    }
                }

                std::vector<yoloout> output;
                output.reserve(detections.size());

                for (const auto& det : detections) {
                    cv::Rect box = det.box;
                    cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
                    yoloout out{
                        .name = det.className,
                        .confidence = det.confidence,
                        .tjImage = {},          // 保留空 Mat
                        .corner = center,
                        .box = box,
                        .classId = det.class_id
                    };
                    output.push_back(std::move(out));
                }

                push_yolo(std::move(output)); // 触发 move 重载
                // 计算并输出本轮总耗时（仅在处理新帧时）
/*                auto loop_end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
                std::cout << "[DEBUG] Frame " << current_frame_id
                    << " processed in " << duration.count() << " us ("
                    << (duration.count()/ 1000.0) << " ms)" << std::endl;*/

            }
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

// ==================== 构造 & 析构 ====================
YoloV8TensorRT::YoloV8TensorRT(const std::string& engine_file_path, const std::string& classes_path) {
    if (!loadEngine(engine_file_path, classes_path)) {
        AppLogger::get().error("Failed to load engine!");
        return;
    }
    m_ready = true;
}

YoloV8TensorRT::~YoloV8TensorRT() {
    if (m_stream) cudaStreamDestroy(m_stream);
    cudaFree(m_device_input);
    cudaFree(m_device_output);
}

// ==================== 加载 Engine ====================
bool YoloV8TensorRT::loadEngine(const std::string& file_path, const std::string& classes_path) {
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        AppLogger::get().error("Cannot open engine file: {}", file_path);
        return false;
    }
    classesPath = classes_path;
    loadClassesFromFile();//加载类别文件
    auto size = file.tellg();
    file.seekg(0);
    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);// 读取文件
    file.close();

    m_runtime = nvinfer1::createInferRuntime(m_logger);// 创建运行时
    if (!m_runtime) {
        AppLogger::get().error("Failed to create IRuntime!");
        return false;
    }

    m_engine = m_runtime->deserializeCudaEngine(engine_data.data(), size);// 反序引擎
    if (!m_engine) {
        AppLogger::get().error("Failed to deserialize engine!");
        return false;
    }

    m_context = m_engine->createExecutionContext();// 创建执行上下文
    if (!m_context) {
        AppLogger::get().error("Failed to create execution context!");
        return false;
    }
 
    // 创建 CUDA stream（成员变量）
    if (cudaStreamCreate(&m_stream) != cudaSuccess) {
        AppLogger::get().error("Failed to create CUDA stream!");
        return false;
    }

    // 从引擎读取输入尺寸（静态引擎固定形状；天井 1280x1280、图案 640x640 均自适应）
    {
        nvinfer1::Dims engine_in = m_engine->getTensorShape(YoloConfig::INPUT_NAME_T);
        if (engine_in.nbDims >= 4) {
            m_input_h = engine_in.d[2];
            m_input_w = engine_in.d[3];
        }
    }
    // 动态引擎才需要 setInputShape；静态引擎(无优化 profile)返回 false 则沿用固定形状
    nvinfer1::Dims input_dims{ 4, {YoloConfig::BATCH_SIZE, YoloConfig::CHANNELS, m_input_h, m_input_w} };
    if (!m_context->setInputShape(YoloConfig::INPUT_NAME_T, input_dims)) {
        AppLogger::get().warn("setInputShape 失败（静态引擎无优化 profile），沿用引擎固定输入形状 [{},{},{},{}]",
                              YoloConfig::BATCH_SIZE, YoloConfig::CHANNELS, m_input_h, m_input_w);
    }

    // 获取输出大小
    nvinfer1::Dims output_dims = m_context->getTensorShape(YoloConfig::OUTPUT_NAME_T);
    m_output_bytes = m_context->getMaxOutputSize(YoloConfig::OUTPUT_NAME_T);// 输出字节数

    if (m_output_bytes == 0) {
        AppLogger::get().error("Invalid output size!");
        return false;
    }

    m_host_output.resize(m_output_bytes / sizeof(float));  // 预分配，避免每帧 alloc

    m_input_bytes = static_cast<size_t>(YoloConfig::BATCH_SIZE) * YoloConfig::CHANNELS * m_input_h * m_input_w * sizeof(float);// 输入字节数

    // 分配 GPU 内存
    if (cudaMalloc(&m_device_input, m_input_bytes) != cudaSuccess ||
        cudaMalloc(&m_device_output, m_output_bytes) != cudaSuccess) {
        AppLogger::get().error("CUDA malloc failed!");
        return false;
    }

    m_context->setTensorAddress(YoloConfig::INPUT_NAME_T, m_device_input);
    m_context->setTensorAddress(YoloConfig::OUTPUT_NAME_T, m_device_output);

    AppLogger::get().info("Engine loaded successfully!");
    return true;

}

void YoloV8TensorRT::loadClassesFromFile()
{
    std::ifstream inputFile(classesPath);
    if (inputFile.is_open())
    {
        std::string classLine;
        while (std::getline(inputFile, classLine))

            classes.push_back(classLine);
        inputFile.close();
    }
}

// ==================== GPU预处理和推理函数 ====================
std::vector<Detection> YoloV8TensorRT::infer(const cv::Mat& frame, int orig_img_w, int orig_img_h) {
    auto total_start = std::chrono::high_resolution_clock::now();

    if (!m_ready) {
        AppLogger::get().error("Model not ready!");
        return {};
    }

    //1. 确保 CUDA stream 已创建
    if (!m_stream) {
        AppLogger::get().error("CUDA stream not created!");
        return {};
    }

    auto preprocess_start = std::chrono::high_resolution_clock::now();

    //2. GPU预处理（使用预分配 GpuMat，避免每帧 GPU alloc）
    float orig_ratio = static_cast<float>(orig_img_w) / orig_img_h;
    float model_ratio = static_cast<float>(m_input_w) / m_input_h;

    cv::Size new_size;
    int top_pad = 0, bottom_pad = 0, left_pad = 0, right_pad = 0;

    if (orig_ratio > model_ratio) {
        new_size.width = m_input_w;
        new_size.height = static_cast<int>(m_input_w / orig_ratio);
        top_pad = (m_input_h - new_size.height) / 2;
        bottom_pad = m_input_h - new_size.height - top_pad;
    }
    else {
        new_size.height = m_input_h;
        new_size.width = static_cast<int>(m_input_h * orig_ratio);
        left_pad = (m_input_w - new_size.width) / 2;
        right_pad = m_input_w - new_size.width - left_pad;
    }

    // resize + copyMakeBorder (letterbox in 1 step: 替代 setTo+copyTo)
    cv::resize(frame, m_resized, new_size, 0, 0, cv::INTER_LINEAR);
    cv::copyMakeBorder(m_resized, m_letterboxed,
        top_pad, bottom_pad, left_pad, right_pad,
        cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    // BGR uint8 -> RGB float[0,1]（预分配 GpuMat）
    m_letterboxed.convertTo(m_float, CV_32FC3, 1.0 / 255.0);
    cv::cvtColor(m_float, m_rgb, cv::COLOR_BGR2RGB);

    // HWC -> CHW: split（预分配 channel Mat，无 per-frame alloc）+ H2D memcpy
    // 修复：split() 只写入数组元素（Mat 头拷贝），成员 m_channel_* 之前从未分配，
    //      原先从空 Mat 取 ptr<float>()（空指针）导致 GPU 输入一直是垃圾数据 → 零检出。
    //      这里显式预分配成员（数组拷贝与成员共享缓冲），并直接从数组元素取数据。
    if (m_channel_r.empty()) {
        m_channel_r.create(m_input_h, m_input_w, CV_32FC1);
        m_channel_g.create(m_input_h, m_input_w, CV_32FC1);
        m_channel_b.create(m_input_h, m_input_w, CV_32FC1);
    }
    cv::Mat channels[] = { m_channel_r, m_channel_g, m_channel_b };
    cv::split(m_rgb, channels);
    size_t channel_size = m_input_w * m_input_h * sizeof(float);
    cudaMemcpyAsync(static_cast<char*>(m_device_input), channels[0].ptr<float>(),
        channel_size, cudaMemcpyHostToDevice, m_stream);
    cudaMemcpyAsync(static_cast<char*>(m_device_input) + channel_size, channels[1].ptr<float>(),
        channel_size, cudaMemcpyHostToDevice, m_stream);
    cudaMemcpyAsync(static_cast<char*>(m_device_input) + 2 * channel_size, channels[2].ptr<float>(),
        channel_size, cudaMemcpyHostToDevice, m_stream);

    auto preprocess_end = std::chrono::high_resolution_clock::now();
    auto preprocess_time = std::chrono::duration<double, std::milli>(preprocess_end - preprocess_start);
   // std::cout << "GPU预处理耗时: " << preprocess_time.count() << " 毫秒" << std::endl;

    //3. 执行推理（异步提交）
    bool enqueued = m_context->enqueueV3(m_stream);
    if (!enqueued) {
        AppLogger::get().error("enqueueV3 failed!");
        return {};
    }

    //4. D2H: Device -> Host (异步，复用预分配 buffer)
    cudaMemcpyAsync(m_host_output.data(), m_device_output,
        m_output_bytes,
        cudaMemcpyDeviceToHost,
        m_stream);

    //5. 同步等待 stream 完成所有操作
    cudaError_t syncStatus = cudaStreamSynchronize(m_stream);
    if (syncStatus != cudaSuccess) {
        AppLogger::get().error("CUDA stream synchronize failed: {}", cudaGetErrorString(syncStatus));
        return {};
    }

    auto inference_end = std::chrono::high_resolution_clock::now();
    auto inference_time = std::chrono::duration<double, std::milli>(inference_end - preprocess_end);
   // std::cout << "GPU推理耗时: " << inference_time.count() << " 毫秒" << std::endl;

    //6. 后处理
    auto post_start = std::chrono::high_resolution_clock::now();

    // 获取输出维度信息
    nvinfer1::Dims output_dims = m_context->getTensorShape(YoloConfig::OUTPUT_NAME_T);
    int num_classes = output_dims.d[1];  // 4 + class_count (e.g. 6 for 2-class model)
    int num_det = output_dims.d[2];      // number of detection candidates (e.g. 8400)

    // 原始数据为 [num_classes, num_det] 布局（列优先）
    // 跳过转置，直接用 stride 索引读取每个检测
    float* raw_data = m_host_output.data();

    // 计算 Letterbox 的缩放和填充（padding 复用预处理阶段的值）
    float scale;
    if (orig_ratio > model_ratio) {
        scale = static_cast<float>(m_input_w) / orig_img_w;
    }
    else {
        scale = static_cast<float>(m_input_h) / orig_img_h;
    }

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    class_ids.reserve(num_det);
    confidences.reserve(num_det);
    boxes.reserve(num_det);

    // 解析每个检测：直接 stride 索引，无转置
    for (int i = 0; i < num_det; ++i) {
        float* det = raw_data + i;  // 指向第 i 个检测的 x

        // 手动找最大类别分数（idx 4..num_classes-1）
        float maxClassScore = det[4 * num_det];
        int class_id = 0;
        for (int c = 1; c < num_classes - 4; ++c) {
            float score = det[(4 + c) * num_det];
            if (score > maxClassScore) {
                maxClassScore = score;
                class_id = c;
            }
        }

        if (maxClassScore <= YoloConfig::CONF_THRESHOLD) continue;

        float x = det[0];           // = raw_data[i + 0 * num_det]
        float y = det[1 * num_det];  // = raw_data[i + 1 * num_det]
        float w = det[2 * num_det];
        float h = det[3 * num_det];

        // 逆 letterbox：减去填充，除以缩放比例
        x = (x - left_pad) / scale;
        y = (y - top_pad) / scale;
        w = w / scale;
        h = h / scale;

        int left = static_cast<int>(x - 0.5f * w);
        int top = static_cast<int>(y - 0.5f * h);
        int width = static_cast<int>(w);
        int height = static_cast<int>(h);

        left = std::max(0, std::min(left, orig_img_w - 1));
        top = std::max(0, std::min(top, orig_img_h - 1));
        width = std::max(1, std::min(width, orig_img_w - left));
        height = std::max(1, std::min(height, orig_img_h - top));

        confidences.push_back(maxClassScore);
        class_ids.push_back(class_id);
        boxes.push_back(cv::Rect(left, top, width, height));
    }

    //7. 应用非极大值抑制 (NMS)
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences,
        YoloConfig::Score_THRESHOLD,
        YoloConfig::NMS_THRESHOLD,
        nms_result);

    //8. 构建检测结果
    std::vector<Detection> detections;
    detections.reserve(nms_result.size());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(100, 255);

    for (unsigned long i = 0; i < nms_result.size(); ++i) {
        int idx = nms_result[i];

        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));

        if (result.class_id >= classes.size()) {
            result.className = "unknown";
        }
        else {
            result.className = classes[result.class_id];
        }
        result.box = boxes[idx];

        detections.push_back(result);
    }

    auto post_end = std::chrono::high_resolution_clock::now();
    auto post_time = std::chrono::duration<double, std::milli>(post_end - post_start);
   // std::cout << "后处理耗时（含NMS）: " << post_time.count() << " 毫秒" << std::endl;

    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_time = std::chrono::duration<double, std::milli>(total_end - total_start);
  //  std::cout << "总耗时: " << total_time.count() << " 毫秒" << std::endl;

    return detections;
}