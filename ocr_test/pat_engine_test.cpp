// pat_engine_test.cpp — 用项目自己的 YoloV8TensorRT 类对新图案引擎跑一次真实推理
// 用法: pat_engine_test <engine> <classes.txt> <image>
#include <thread>
#include <stop_token>
#include "yolov8_trt_infer.hpp"
#include <opencv2/opencv.hpp>
#include <cstdio>

// 平时由 main.cpp 定义的全局量（链接时需要）
int actualWidth = 1920, actualHeight = 1080, actualFPS = 30;

int main(int argc, char** argv) {
    if (argc < 4) { std::printf("usage: pat_engine_test <engine> <classes.txt> <image>\n"); return 2; }
    YoloV8TensorRT yolo(argv[1], argv[2]);
    if (!yolo.isInitialized()) { std::printf("FAIL: engine init\n"); return 1; }
    cv::Mat img = cv::imread(argv[3]);
    if (img.empty()) { std::printf("FAIL: image\n"); return 2; }
    std::printf("image %dx%d, running inference...\n", img.cols, img.rows);
    auto dets = yolo.infer(img, img.cols, img.rows);
    std::printf("RESULT: %zu detections\n", dets.size());
    for (const auto& d : dets) {
        std::printf("  class=%s id=%d conf=%.3f box=(%d,%d,%dx%d)\n",
            d.className.c_str(), d.class_id, d.confidence,
            d.box.x, d.box.y, d.box.width, d.box.height);
    }
    return 0;
}
