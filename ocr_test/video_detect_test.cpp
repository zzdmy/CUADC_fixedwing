// ocr_test/video_detect_test.cpp
//
// 离线识别测试（视频或单张图片）：用与主程序完全相同的检测器（YoloV8TensorRT）
// 跑 天井(best_well.engine) 检测，命中时对最大框做 图案/OCR 二级识别。
//
// 用法（在 /home/nvidia/cuadc 下运行，模型都在根目录）:
//   video_detect_test <video|image> [--stride N] [--pattern] [--ocr] [--dump <dir>] [--crop x y w h]
//   --crop 仅对单张图片生效：先裁剪再检测（放大目标排查小目标漏检）
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include "AppLogger.h"
#include "yolov8_trt_infer.hpp"
#include "ocr/OcrPipeline.h"

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    if (argc < 2) {
        std::printf("usage: video_detect_test <video|image> [--stride N] [--pattern] [--ocr] [--dump <dir>] [--crop x y w h]\n");
        return 2;
    }

    std::string input = argv[1];
    int stride = 1;
    bool usePattern = false, useOcr = false;
    std::string dumpDir;
    int cx = 0, cy = 0, cw = 0, ch = 0;
    bool cropSet = false;
    for (int i = 2; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--stride") && i + 1 < argc) stride = std::max(1, std::atoi(argv[++i]));
        else if (!std::strcmp(argv[i], "--pattern")) usePattern = true;
        else if (!std::strcmp(argv[i], "--ocr")) useOcr = true;
        else if (!std::strcmp(argv[i], "--dump") && i + 1 < argc) dumpDir = argv[++i];
        else if (!std::strcmp(argv[i], "--crop") && i + 4 < argc) {
            cx = std::atoi(argv[++i]); cy = std::atoi(argv[++i]);
            cw = std::atoi(argv[++i]); ch = std::atoi(argv[++i]);
            cropSet = true;
        }
    }

    AppLogger::get().setLevel(spdlog::level::warn);

    // ---- 一级：天井检测器（与主程序同一引擎）----
    YoloV8TensorRT well("best_well.engine", "best_well.txt");
    if (!well.isInitialized()) { std::printf("[FAIL] 天井检测器初始化失败\n"); return 1; }
    std::printf("[OK] 天井检测器就绪 (best_well.engine)\n");

    // ---- 二级（可选）----
    std::unique_ptr<YoloV8TensorRT> pattern;
    if (usePattern) {
        pattern = std::make_unique<YoloV8TensorRT>("best_detect.engine", "best_detect.txt");
        if (!pattern->isInitialized()) { std::printf("[FAIL] 图案检测器初始化失败\n"); return 1; }
        std::printf("[OK] 图案检测器就绪 (best_detect.engine)\n");
    }
    std::unique_ptr<ocr::OcrPipeline> ocrPipe;
    if (useOcr) {
        ocr::OcrPipelineConfig cfg;
        cfg.rec.onnxPath = "PP-OCRv6_medium_rec.onnx";
        cfg.rec.enginePath = "PP-OCRv6_medium_rec.engine";
        cfg.rec.dictPath = "ppocr_keys_v6.txt";
        cfg.rec.recHeight = 48;
        cfg.rec.recMinWidth = 16;
        cfg.rec.recOptWidth = 320;
        cfg.rec.recMaxWidth = 320;
        cfg.rec.useFp16 = true;
        cfg.rec.digitOnly = true;
        cfg.useDet = true;
        cfg.det.onnxPath = "PP-OCRv6_medium_det.onnx";
        cfg.det.enginePath = "PP-OCRv6_medium_det.engine";
        cfg.det.limitSide = 960;
        cfg.det.boxThresh = 0.45f;
        cfg.det.thresh = 0.2f;
        cfg.det.unclipRatio = 1.4f;
        cfg.det.useFp16 = true;
        cfg.useOri = true;
        cfg.ori.onnxPath = "PP-LCNet_x1_0_textline_ori.onnx";
        cfg.ori.enginePath = "PP-LCNet_x1_0_textline_ori.engine";
        cfg.ori.useFp16 = true;
        ocrPipe = std::make_unique<ocr::OcrPipeline>(cfg);
        if (!ocrPipe->isReady()) { std::printf("[FAIL] OCR 管线初始化失败\n"); return 1; }
        std::printf("[OK] OCR 管线就绪\n");
    }

    // ---- 单帧检测+二级识别（返回天井数量）----
    float bestConf = 0.0f;
    auto processFrame = [&](const cv::Mat& frame, long idx) -> long {
        auto dets = well.infer(frame, frame.cols, frame.rows);
        if (dets.empty()) return 0;

        std::printf("[帧 %4ld] 天井x%zu:", idx, dets.size());
        for (const auto& d : dets) {
            if (d.confidence > bestConf) bestConf = d.confidence;
            std::printf(" [%d,%d %dx%d c=%.2f %s]",
                d.box.x, d.box.y, d.box.width, d.box.height,
                d.confidence, d.className.c_str());
        }

        const Detection* big = &*std::max_element(dets.begin(), dets.end(),
            [](const Detection& a, const Detection& b) { return a.box.area() < b.box.area(); });
        cv::Rect r = big->box;
        const int px = static_cast<int>(r.width * 0.08);
        const int py = static_cast<int>(r.height * 0.08);
        r = cv::Rect(r.x - px, r.y - py, r.width + 2 * px, r.height + 2 * py)
            & cv::Rect(0, 0, frame.cols, frame.rows);
        if (r.area() > 0) {
            cv::Mat crop = frame(r).clone();
            if (pattern) {
                auto pd = pattern->infer(crop, crop.cols, crop.rows);
                if (!pd.empty()) {
                    const Detection* best = &*std::max_element(pd.begin(), pd.end(),
                        [](const Detection& a, const Detection& b) { return a.confidence < b.confidence; });
                    std::printf(" | 图案=%s(%.2f)", best->className.c_str(), best->confidence);
                } else {
                    std::printf(" | 图案=无");
                }
            }
            if (ocrPipe) {
                auto res = ocrPipe->run(crop);
                std::printf(" | OCR='%s'(%.2f)", res.text.c_str(), res.confidence);
            }
            if (!dumpDir.empty()) {
                char path[512];
                std::snprintf(path, sizeof(path), "%s/hit_%05ld.jpg", dumpDir.c_str(), idx);
                if (cv::imwrite(path, frame)) std::printf(" [dump %s]", path);
            }
        }
        std::printf("\n");
        return static_cast<long>(dets.size());
    };

    // ---- 图片模式 ----
    cv::Mat single = cv::imread(input, cv::IMREAD_COLOR);
    if (!single.empty()) {
        if (cropSet) {
            cv::Rect cr(cx, cy, cw, ch);
            cr &= cv::Rect(0, 0, single.cols, single.rows);
            if (cr.area() > 0) single = single(cr).clone();
            std::printf("[图片] %s 裁剪后 %dx%d (crop %d,%d,%d,%d)\n", input.c_str(), single.cols, single.rows, cx, cy, cw, ch);
        } else {
            std::printf("[图片] %s %dx%d\n", input.c_str(), single.cols, single.rows);
        }
        const long n = processFrame(single, 0);
        std::printf("\n==== 汇总(单图) ==== 天井数=%ld | 最高conf=%.3f\n", n, bestConf);
        return 0;
    }

    // ---- 视频模式 ----
    cv::VideoCapture cap(input);
    if (!cap.isOpened()) { std::printf("[FAIL] 无法打开视频: %s\n", input.c_str()); return 1; }
    const int total = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const double fps = cap.get(cv::CAP_PROP_FPS);
    std::printf("[视频] %s  %dx%d @ %.1ffps  共%d帧  抽帧步长=%d\n",
        input.c_str(),
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
        fps, total, stride);

    long idx = 0, used = 0, hitFrames = 0, detTotal = 0;
    cv::Mat frame;
    while (cap.read(frame)) {
        if (idx % stride != 0) { ++idx; continue; }
        ++used;
        const long n = processFrame(frame, idx);
        if (n > 0) { ++hitFrames; detTotal += n; }
        if (used % 200 == 0) std::printf("...进度 %ld/%d\n", idx, total);
        ++idx;
    }

    std::printf("\n==== 汇总 ==== 视频共%d帧 | 处理(抽帧%d)=%ld | 含天井帧=%ld | 天井实例总数=%ld | 最高conf=%.3f\n",
        total, stride, used, hitFrames, detTotal, bestConf);
    return 0;
}
