// ocr_test/onnx_dnn_test.cpp
//
// 用 OpenCV DNN 直接跑 YOLO ONNX（完全绕开 TensorRT 管线），验证模型本身。
// 输出：输出张量维度 + 各类别最高原始分数（未经阈值过滤）。
//
// 用法: onnx_dnn_test <onnx> <image> [inputW inputH]
#include <cstdio>
#include <algorithm>

#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    if (argc < 3) { std::printf("usage: onnx_dnn_test <onnx> <image> [W H]\n"); return 2; }
    const int W = argc >= 5 ? std::atoi(argv[3]) : 1280;
    const int H = argc >= 5 ? std::atoi(argv[4]) : 1280;

    cv::dnn::Net net;
    try {
        net = cv::dnn::readNetFromONNX(argv[1]);
    } catch (const std::exception& e) {
        std::printf("[FAIL] readNetFromONNX 异常: %s\n", e.what());
        return 1;
    }
    if (net.empty()) { std::printf("[FAIL] 无法加载 onnx\n"); return 1; }

    cv::Mat img = cv::imread(argv[2], cv::IMREAD_COLOR);
    if (img.empty()) { std::printf("[FAIL] 无法读取图片\n"); return 1; }

    // letterbox 到模型输入
    const float r = std::min(static_cast<float>(W) / img.cols, static_cast<float>(H) / img.rows);
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(), r, r, cv::INTER_LINEAR);
    cv::Mat lb(H, W, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(lb(cv::Rect(0, 0, resized.cols, resized.rows)));

    cv::Mat blob = cv::dnn::blobFromImage(lb, 1.0 / 255.0, cv::Size(W, H), cv::Scalar(), true, false);
    net.setInput(blob);
    cv::Mat out;
    try {
        out = net.forward();
    } catch (const std::exception& e) {
        std::printf("[FAIL] forward 异常: %s\n", e.what());
        return 1;
    }

    std::printf("输出维度: ");
    for (int i = 0; i < out.dims; ++i) std::printf("%d%s", out.size[i], i + 1 < out.dims ? "," : "");
    std::printf("\n");

    // 期望 [1, 4+nc, N]；自动判断两种情况
    const int n1 = out.size[1], n2 = out.size[2];
    bool layoutCN = (n1 < n2);   // [1, 4+nc, N]
    const int nc = layoutCN ? n1 : n2;
    const int nd = layoutCN ? n2 : n1;
    std::printf("布局: %s  类别数(含4)=%d 候选数=%d\n", layoutCN ? "[1,C,N]" : "[1,N,C]", nc, nd);

    const float* p = reinterpret_cast<const float*>(out.data);
    float best[16] = { 0 };
    for (int i = 0; i < nd; ++i) {
        for (int c = 4; c < std::min(nc, 16); ++c) {
            const float s = layoutCN ? p[static_cast<size_t>(c) * nd + i] : p[static_cast<size_t>(i) * nc + c];
            if (s > best[c]) best[c] = s;
        }
    }
    for (int c = 4; c < std::min(nc, 16); ++c) {
        std::printf("类别%d 最高原始分数 = %.4f %s\n", c - 4, best[c], best[c] > 0.25f ? " <== 有检测!" : "");
    }
    return 0;
}
