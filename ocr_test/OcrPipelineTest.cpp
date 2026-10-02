// ocr_test/OcrPipelineTest.cpp
//
// 任务二「数字靶标」完整管线端到端自测：
//   det(DB 找数字行) -> ori(纠 180°) -> rec(读数字)
//
// 用法（在项目根目录运行，模型文件都在根目录）:
//   OcrPipelineTest.exe
//
// 生成合成数字图（白底黑字），逐张跑完整管线，打印期望 vs 实际。
#include <cstdio>
#include <string>

#include <opencv2/imgproc.hpp>

#include "AppLogger.h"
#include "ocr/OcrPipeline.h"

namespace {

    // 白底黑字合成图。字号较大，方便 DB 检测到文本行。
    cv::Mat makeDigitImage(const std::string& text, int width = 320, int height = 160) {
        cv::Mat img(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
        const double scale = 4.0;
        const int thickness = 8;
        int baseline = 0;
        cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
        cv::Point org((width - sz.width) / 2, (height + sz.height) / 2);
        cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, scale,
            cv::Scalar(0, 0, 0), thickness, cv::LINE_AA);
        return img;
    }

} // namespace

int main() {
    AppLogger::get().setLevel(spdlog::level::info);

    ocr::OcrPipelineConfig cfg;

    // rec（PP-OCRv6_medium_rec）
    cfg.rec.onnxPath = "PP-OCRv6_medium_rec.onnx";
    cfg.rec.enginePath = "PP-OCRv6_medium_rec.engine";
    cfg.rec.dictPath = "ppocr_keys_v6.txt";
    cfg.rec.recHeight = 48;
    cfg.rec.recMinWidth = 16;
    cfg.rec.recOptWidth = 320;
    cfg.rec.recMaxWidth = 320;
    cfg.rec.useFp16 = true;
    cfg.rec.digitOnly = true;

    // det（PP-OCRv6_medium_det）
    cfg.useDet = true;
    cfg.det.onnxPath = "PP-OCRv6_medium_det.onnx";
    cfg.det.enginePath = "PP-OCRv6_medium_det.engine";
    cfg.det.limitSide = 960;
    cfg.det.boxThresh = 0.45f;
    cfg.det.thresh = 0.2f;
    cfg.det.unclipRatio = 1.4f;
    cfg.det.useFp16 = true;

    // ori（PP-LCNet_x1_0_textline_ori）
    cfg.useOri = true;
    cfg.ori.onnxPath = "PP-LCNet_x1_0_textline_ori.onnx";
    cfg.ori.enginePath = "PP-LCNet_x1_0_textline_ori.engine";
    cfg.ori.useFp16 = true;

    ocr::OcrPipeline pipe(cfg);
    if (!pipe.isReady()) {
        std::fprintf(stderr, "[FAIL] OcrPipeline 初始化失败，引擎未就绪\n");
        return 1;
    }
    std::fprintf(stderr, "[ OK ] OcrPipeline 初始化成功\n");

    int fail = 0;
    const std::string cases[] = { "7", "42", "138" };
    for (const auto& expect : cases) {
        cv::Mat img = makeDigitImage(expect);
        ocr::RecResult r = pipe.run(img);
        const bool ok = (r.text == expect);
        if (!ok) ++fail;
        std::fprintf(stderr, "[%s] expect='%s' got='%s' conf=%.3f\n",
            ok ? "PASS" : "MISS", expect.c_str(), r.text.c_str(), r.confidence);
    }

    const int total = static_cast<int>(sizeof(cases) / sizeof(cases[0]));
    std::fprintf(stderr, "=== %d/%d 通过 ===\n", total - fail, total);
    return fail == 0 ? 0 : 2;
}
