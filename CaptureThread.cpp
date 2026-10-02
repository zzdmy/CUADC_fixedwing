// CaptureThread.cpp
#include "CaptureThread.h"
#include <chrono>
#include <cmath>
#include "AppLogger.h"
#include <stdexcept>

extern int actualWidth, actualHeight, actualFPS;

CaptureThread::CaptureThread(int cameraIndex)
    : CaptureThread(cameraIndex, Config{}) {
}

CaptureThread::CaptureThread(int cameraIndex, const Config& config)
    : cameraIndex_(cameraIndex), config_(config), running_(false) {
}

CaptureThread::~CaptureThread() {
    stop();
}

void CaptureThread::start() {
    if (running_.exchange(true)) return;

    try {
        // 相机可能被地面录像(camera_recorder)短暂持有：它约2秒轮询一次并主动让出。
        // 这里重试等待最多 ~13.5 秒，避免"程序先启动、录像后让出"时启动即失败（飞行关键路径）。
        for (int attempt = 0; attempt < 10; ++attempt) {
            if (attempt > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            }
#ifdef _WIN32
            cap_.open(cameraIndex_ + cv::CAP_DSHOW);
#else
            cap_.open(cameraIndex_, cv::CAP_V4L2);
#endif
            if (cap_.isOpened()) break;
        }
        if (!cap_.isOpened()) {
            throw std::runtime_error("无法打开摄像头索引: " + std::to_string(cameraIndex_));
        }

        AppLogger::get().info("后端: {}", cap_.getBackendName());

        if (config_.useMjpg) {
            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        }
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
        cap_.set(cv::CAP_PROP_FPS, config_.fps);

        int w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        int h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        int fps = static_cast<int>(cap_.get(cv::CAP_PROP_FPS));

        if (w != config_.width || h != config_.height) {
            AppLogger::get().error("警告：分辨率设置失败，实际: {}x{}", w, h);
        }
        else {
            AppLogger::get().info("分辨率设置成功: {}x{}", w, h);
        }
        actualWidth=w;
        actualHeight=h;

        // 曝光模式初始化
        if (config_.ae_enable) {
            cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);          // 切手动，由 PI 控制器接管
            int probe = static_cast<int>(cap_.get(cv::CAP_PROP_EXPOSURE));
            AppLogger::get().info("AE: 当前曝光值={} (配置范围 {}~{}, 现场可通过 ae.exposure_min/max 调整)",
                probe, config_.ae_exposure_min, config_.ae_exposure_max);
            current_exposure_ = std::clamp(probe, config_.ae_exposure_min, config_.ae_exposure_max);
        } else {
            cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);          // 恢复摄像头自带自动曝光
            AppLogger::get().info("AE: 软件 AE 已禁用，使用摄像头自动曝光");
        }

        thread_ = std::thread(&CaptureThread::run, this);
    }
    catch (...) {
        running_ = false;
        cap_.release();
        throw;
    }
}

void CaptureThread::stop() {
    if (running_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
        cap_.release();
    }
}

bool CaptureThread::isRunning() const {
    return running_.load();
}

double CaptureThread::getCaptureFPS() const {
    return capture_fps_.load();
}

void CaptureThread::run() {
    cv::Mat frame;
    int frameCount = 0;
    auto startTime = std::chrono::steady_clock::now();
    auto ae_log_time = std::chrono::steady_clock::now();

    while (running_) {
        cap_ >> frame;
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // ---- 软件 AE（PI 控制器） ----
        if (config_.ae_enable) {
            // 冷却期内跳过 PI 计算，等待相机响应上次调节
            if (ae_cooldown_ > 0) {
                ae_cooldown_--;
            } else {
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                double brightness = cv::mean(gray)[0] / 255.0;

                double error = config_.ae_target_brightness - brightness;

                // 死区判断：误差在死区内不调节，积分归零防累积
                if (std::abs(error) < config_.ae_deadband) {
                    ae_integral_ = 0.0;
                } else {
                    ae_integral_ += error;
                    ae_integral_ = std::clamp(ae_integral_, -5.0, 5.0);

                    // 曝光效果≈线性于曝光时长：按"目标/当前亮度"比例步进(半步长)，
                    // 相机真实量程 1~10000 下数步收敛（原加法步进只适配已废弃的负数小量程）
                    double ratio = config_.ae_target_brightness / std::max(brightness, 0.02);
                    ratio = std::clamp(ratio, 0.25, 4.0);   // 单次最多 ±4 倍，防过冲
                    int target_exposure = static_cast<int>(current_exposure_ * std::sqrt(ratio));
                    if (target_exposure == current_exposure_) {
                        // 最小步进：曝光值很小时乘法取整会原地踏步，强制 ±1 保证能爬出边界
                        target_exposure = current_exposure_ + (error > 0 ? 1 : -1);
                    }
                    target_exposure = std::clamp(target_exposure, config_.ae_exposure_min, config_.ae_exposure_max);

                    if (target_exposure != current_exposure_) {
                        cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
                        cap_.set(cv::CAP_PROP_EXPOSURE, target_exposure);
                        AppLogger::get().info("AE_ADJ: exposure {} -> {} (brightness={:.3f} target={:.2f})",
                            current_exposure_, target_exposure, brightness, config_.ae_target_brightness);
                        current_exposure_ = target_exposure;
                        ae_cooldown_ = config_.ae_update_interval;  // 调完进入冷却
                    }
                }
            }

            // 每 2 秒输出一次 AE 状态日志
            auto now_ae = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now_ae - ae_log_time).count() >= 2.0) {
                // brightness 在冷却期内没算，重新取一下
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                double brightness = cv::mean(gray)[0] / 255.0;
                AppLogger::get().info("AE: brightness={:.3f} target={:.1f} exposure={} cooldown={}",
                    brightness, config_.ae_target_brightness, current_exposure_, ae_cooldown_);
                ae_log_time = now_ae;
            }
        }

        frameDispatcher.updateFrame(std::move(frame));

        frameCount++;
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - startTime).count() >= 1.0) {
            capture_fps_.store(static_cast<double>(frameCount) /
                std::chrono::duration<double>(now - startTime).count());
            frameCount = 0;
            startTime = now;
        }
    }
}
