#include "OpenCVDNNDetector.h"
#include "FrameDispatcher.h"
#include "yolodecet/yolo_tracker.h"

OpenCVDNNDetector::OpenCVDNNDetector(const std::string& modelPath, const std::string& classPath, bool useGPU)
    : m_modelPath(modelPath), m_classPath(classPath), m_useGPU(useGPU) {

    fristyolo(m_modelPath, m_classPath, m_useGPU);
    m_infer = std::make_unique<Inference>(m_modelPath, cv::Size(640, 640), m_classPath, m_useGPU);
    m_initialized = true;
}

OpenCVDNNDetector::~OpenCVDNNDetector() {
    stop(); // jthread 自动 join
}

void OpenCVDNNDetector::startDetectionLoop(int width, int height) {
    if (!m_initialized || m_running) return;
    m_running = true;
    m_thread = std::jthread([this](std::stop_token st) {
        detectionThread(std::move(st));
        });
}

void OpenCVDNNDetector::detectionThread(std::stop_token st) {
    init_pipeline(); // 初始化管道（每个 detector 独立调用，但内部有保护）

    uint64_t last_frame_id = 0; // ← 局部变量，线程安全

    while (!st.stop_requested()) {
        auto frame_ptr = frameDispatcher.getFrame();
        if (!frame_ptr || frame_ptr->empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        uint64_t current_frame_id = frameDispatcher.getFrameCounter();
        if (current_frame_id <= last_frame_id) {
            continue;
        }
        last_frame_id = current_frame_id;

        // ⚠️ 注意：runObjectTracking 内部会清空 output 并填充
        std::vector<yoloout> output;
        bool dummy_stop = true; // yolostop 恒为 true，可移除
        runObjectTracking(*frame_ptr, *m_infer, output, dummy_stop);

        // 👇 关键：使用 move 语义推送（需 realtime_pipeline 支持）
        push_yolo(std::move(output));
    }

    signal_stop(); // 通知消费者退出
}

void OpenCVDNNDetector::stop() {
    m_running = false;
    // jthread 析构时自动 request_stop + join
}

bool OpenCVDNNDetector::isInitialized() const {
    return m_initialized;
}