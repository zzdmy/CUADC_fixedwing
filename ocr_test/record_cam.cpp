// ocr_test/record_cam.cpp
//
// CUADC 机载相机录像程序（dashcam 式连续录制）
//
// 冲突处理（重要）：
//   - 相机同一时刻只能被一个进程打开。本程序每 2 秒检测主程序 cuadc_fixedwing
//     是否在运行：在运行 → 立即释放相机让给它；退出 → 自动重新接管继续录。
//   - 相机未插/打开失败 → 每 3 秒重试，插上就录。
//
// 其它：
//   - 分段录制：默认 10 分钟一段，MJPG AVI，写到 ~/cuadc/recordings/rec_*.avi
//   - 磁盘保护：目录总大小超过 80GB 自动删最旧分段
//   - 相机参数与主程序一致：1920x1080 + 相机自动曝光(0.75)
//
// 编译（机载）: g++ -std=c++20 -O2 -I/usr/include/opencv4 record_cam.cpp \
//   -o bin/camera_recorder -L/usr/lib/aarch64-linux-gnu \
//   -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio
#include <cstdio>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace fs = std::filesystem;
using namespace std::chrono;

// ===== 可调参数 =====
static const char* REC_DIR = "/home/nvidia/cuadc/recordings";
static const int    SEGMENT_SEC = 300;                    // 每段 5 分钟（断电最多损失当前段）
static const uint64_t MAX_TOTAL_BYTES = 80ull << 30;      // 总上限 80GB
static const int    WIDTH = 1920, HEIGHT = 1080, FPS = 30;

// 信号处理：收到停止信号后优雅退出（关闭当前分段，MP4 索引正常写入）
static volatile std::sig_atomic_t g_stop = 0;
static void onSignal(int) { g_stop = 1; }

// 主程序是否在运行（用 /proc/PID/comm 精确匹配进程名，
// 避免命令行里仅仅"提到" cuadc_fixedwing 字样的脚本被误判）
static bool mainProgRunning() {
    std::error_code ec;
    for (const auto& e : fs::directory_iterator("/proc", ec)) {
        const std::string name = e.path().filename().string();
        if (name.empty() || !std::isdigit(static_cast<unsigned char>(name[0]))) continue;
        std::ifstream f(e.path() / "comm");
        if (!f.is_open()) continue;
        std::string comm;
        std::getline(f, comm);
        if (comm == "cuadc_fixedwing") return true;
    }
    return false;
}

// 生成不重名的分段文件名（机载时钟不稳，同名时自动加序号）
static std::string makeSegmentPath() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char base[64];
    std::strftime(base, sizeof(base), "rec_%Y-%m-%d_%H-%M-%S", &tm);
    for (int i = 0; i < 100; ++i) {
        std::string p = std::string(REC_DIR) + "/" + base +
            (i ? "_" + std::to_string(i) : "") + ".mp4";
        if (!fs::exists(p)) return p;
    }
    return std::string(REC_DIR) + "/" + base + "_x.mp4";
}

// 超过总量上限时删最旧的分段（跳过当前正在写的）
static void cleanupOld(const std::string& curPath) {
    std::error_code ec;
    std::vector<std::pair<fs::file_time_type, fs::path>> files;
    uint64_t total = 0;
    for (const auto& e : fs::directory_iterator(REC_DIR, ec)) {
        const auto ext = e.path().extension().string();
        if (ext != ".avi" && ext != ".mp4") continue;   // 兼容旧 avi 分段
        total += fs::file_size(e.path(), ec);
        files.emplace_back(fs::last_write_time(e.path(), ec), e.path());
    }
    if (total <= MAX_TOTAL_BYTES) return;
    std::sort(files.begin(), files.end());
    for (auto& [ft, p] : files) {
        if (total <= MAX_TOTAL_BYTES) break;
        if (p.string() == curPath) continue;
        const uint64_t sz = fs::file_size(p, ec);
        if (fs::remove(p, ec)) {
            total -= sz;
            std::printf("[rec] 清理旧分段: %s\n", p.filename().string().c_str());
        }
    }
}

int main() {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGINT, onSignal);
    fs::create_directories(REC_DIR);
    std::printf("[rec] 录像程序启动，输出目录: %s\n", REC_DIR);

    cv::VideoCapture cap;
    cv::VideoWriter  writer;
    auto segStart = steady_clock::now();
    auto lastYieldCheck = steady_clock::now();
    int  emptyReads = 0;
    bool warnedNoCam = false;

    while (!g_stop) {
        // ---- 每 2 秒：主程序优先，让出相机 ----
        const auto now = steady_clock::now();
        if (now - lastYieldCheck > seconds(2)) {
            lastYieldCheck = now;
            if (mainProgRunning()) {
                if (writer.isOpened()) {
                    writer.release();
                    std::printf("[rec] 主程序运行中，暂停录制（已让出相机）\n");
                }
                if (cap.isOpened()) cap.release();
                std::this_thread::sleep_for(seconds(2));
                continue;
            }
        }

        // ---- 确保相机打开 ----
        if (!cap.isOpened()) {
            cap.open(0, cv::CAP_V4L2);
            if (!cap.isOpened()) {
                if (!warnedNoCam) {
                    std::printf("[rec] 未找到相机/打开失败，每3秒重试...\n");
                    warnedNoCam = true;
                }
                std::this_thread::sleep_for(seconds(3));
                continue;
            }
            cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
            cap.set(cv::CAP_PROP_FRAME_WIDTH, WIDTH);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, HEIGHT);
            cap.set(cv::CAP_PROP_FPS, FPS);
            cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);   // 与主程序一致，防全黑
            warnedNoCam = false;
            std::printf("[rec] 相机已打开: %dx%d\n",
                static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
                static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
        }

        // ---- 取帧 ----
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            if (++emptyReads >= 30) {
                std::printf("[rec] 连续取帧失败，重新打开相机\n");
                writer.release();
                cap.release();
                emptyReads = 0;
            }
            std::this_thread::sleep_for(milliseconds(50));
            continue;
        }
        emptyReads = 0;

        // ---- 分段 ----
        if (!writer.isOpened() || now - segStart > seconds(SEGMENT_SEC)) {
            if (writer.isOpened()) {
                writer.release();
                std::printf("[rec] 分段结束\n");
            }
            const std::string path = makeSegmentPath();
            // GStreamer x264 → MP4（qtmux 已验证可解码；分段5分钟，断电最多损失当前段）
            const std::string pipe =
                "appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc bitrate=6000 speed-preset=veryfast key-int-max=60 "
                "! h264parse ! qtmux ! filesink location=" + path;
            if (writer.open(pipe, cv::CAP_GSTREAMER, 0, FPS, frame.size())) {
                segStart = now;
                cleanupOld(path);
                std::printf("[rec] 新分段: %s\n", path.c_str());
            } else {
                std::printf("[rec] 分段创建失败: %s\n", path.c_str());
                std::this_thread::sleep_for(seconds(2));
                continue;
            }
        }
        writer.write(frame);
    }

    // 优雅退出：关闭当前分段（写入 MP4 索引），释放相机
    if (writer.isOpened()) writer.release();
    if (cap.isOpened()) cap.release();
    std::printf("[rec] 收到退出信号，当前分段已正常关闭\n");
    return 0;
}
