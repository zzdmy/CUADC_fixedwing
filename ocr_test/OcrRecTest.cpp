// ocr_test/OcrRecTest.cpp
//
// 端到端验证 PP-OCRv5 识别链路（不依赖相机 / 飞控）：
//   1. 加载字符字典
//   2. 从 ONNX 构建 TensorRT engine（或复用缓存）
//   3. 张量解析：确认输入是 4 维、输出最后一维 == 字典大小 + 2
//   4. 合成数字图跑通预处理 + 推理 + CTC 解码
//
// 用法: OcrRecTest <onnx> <engine> <dict> [yml]
#include <cstdio>
#include <opencv2/imgproc.hpp>

#include "AppLogger.h"
#include "ocr/PaddleOCRRec.h"

// 生成一张黑字白底的数字图，送入识别链路。
static cv::Mat makeDigitImage(const std::string& text, int scale = 4) {
    const int W = 64 + 40 * static_cast<int>(text.size());
    const int H = 128;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255)); // 白底

    int baseline = 0;
    const double fontScale = 2.2;
    const int thick = 5;
    cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fontScale, thick, &baseline);
    cv::Point org((W - ts.width) / 2, (H + ts.height) / 2);
    cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, fontScale,
        cv::Scalar(0, 0, 0), thick, cv::LINE_AA);

    if (scale > 1) {
        cv::resize(img, img, cv::Size(W * scale, H * scale), 0, 0, cv::INTER_LINEAR);
    }
    return img;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::printf("usage: OcrRecTest <onnx> <engine> <dict> [inference.yml]\n");
        return 2;
    }

    AppLogger::get().setLevel(spdlog::level::info);

    ocr::PaddleRecConfig cfg;
    cfg.onnxPath = argv[1];
    cfg.enginePath = argv[2];
    cfg.dictPath = argv[3];
    if (argc >= 5) cfg.inferenceYmlPath = argv[4];
    cfg.recMaxWidth = 320;
    cfg.recOptWidth = 320;
    cfg.useFp16 = true;
    cfg.digitOnly = true;

    std::printf("\n########## 构造识别器（可能触发 engine 构建）##########\n");
    ocr::PaddleOCRRec rec(cfg);

    if (!rec.isReady()) {
        std::printf("\n[FAIL] 识别器未就绪\n");
        return 1;
    }

    int minW = 0, maxW = 0;
    rec.widthRange(minW, maxW);
    std::printf("\n########## 引擎就绪 ##########\n");
    std::printf("宽度区间      : [%d, %d]\n", minW, maxW);
    std::printf("字典类别数    : %d\n", rec.charset().numClasses());
    std::printf("数字候选数    : %d\n", rec.charset().digitCount());

    int failures = 0;

    // ---- 用例 1: 只含数字 ----
    for (const std::string& t : { std::string("7"), std::string("42"), std::string("138") }) {
        cv::Mat img = makeDigitImage(t);
        ocr::RecResult r = rec.run(img);
        bool hit = (r.text == t);
        std::printf("[%s] 期望 '%s' -> 实得 '%s' (conf=%.3f, 图 %dx%d)\n",
            hit ? "PASS" : "MISS", t.c_str(), r.text.c_str(), r.confidence,
            img.cols, img.rows);
        if (!hit) ++failures;
    }

    // ---- 用例 2: 数字+字母，验证 digitOnly 只保留数字 ----
    {
        cv::Mat img = makeDigitImage("A5B");
        ocr::RecResult r = rec.run(img);
        bool hit = (r.text == "5");
        std::printf("[%s] 'A5B' 仅保留数字 -> 实得 '%s' (conf=%.3f)\n",
            hit ? "PASS" : "MISS", r.text.c_str(), r.confidence);
        if (!hit) ++failures;
    }

    // ---- 用例 3: 空图 / 极小图，不应崩溃 ----
    {
        cv::Mat tiny(4, 4, CV_8UC3, cv::Scalar(255, 255, 255));
        ocr::RecResult r = rec.run(tiny);
        std::printf("[INFO] 4x4 极小图 -> '%s' (未崩溃)\n", r.text.c_str());
        cv::Mat empty;
        ocr::RecResult r2 = rec.run(empty);
        std::printf("[INFO] 空图 -> '%s' (未崩溃)\n", r2.text.c_str());
    }

    std::printf("\n########## 结果 ##########\n");
    std::printf("数字识别失败用例: %d\n", failures);
    std::printf("%s\n", failures == 0 ? "== 端到端 PASS ==" : "== 端到端 部分失败 ==");
    return failures == 0 ? 0 : 1;
}
