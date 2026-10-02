// ocr/OcrPipeline.h
//
// 任务二「数字靶标」完整识别管线组合器：
//   YOLO 检出天井 -> 裁剪 -> det(DB 找数字行) -> 四向 rec(0/90/180/270 取最高置信度)
//
// 对单张图暴露与 PaddleOCRRec 相同的 run() 接口，输出按阅读顺序拼接的数字。
#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include "ocr/PaddleDet.h"
#include "ocr/PaddleOCRRec.h"
#include "ocr/TextLineOri.h"

namespace ocr {

    struct OcrPipelineConfig {
        PaddleRecConfig       rec;
        PaddleDet::DetConfig  det;
        TextLineOri::OriConfig ori;
        bool  useDet = true;        // 启用文本行检测阶段
        bool  useOri = true;        // 已弃用：四向 rec 已覆盖 180° 倒放，run() 不再调用 ori（字段保留以便回退）
        float cropPadding = 0.0f;   // 文本行裁剪外扩比例（det 框已外扩，可再保险）
    };

    class OcrPipeline {
    public:
        explicit OcrPipeline(const OcrPipelineConfig& cfg);
        ~OcrPipeline();

        OcrPipeline(const OcrPipeline&) = delete;
        OcrPipeline& operator=(const OcrPipeline&) = delete;

        bool isReady() const;

        /// @brief 识别单张 BGR 图（天井裁剪），返回按阅读顺序拼接的数字。
        RecResult run(const cv::Mat& bgr);

    private:
        OcrPipelineConfig m_cfg;
        std::unique_ptr<PaddleDet>     m_det;
        std::unique_ptr<TextLineOri>   m_ori;
        std::unique_ptr<PaddleOCRRec>  m_rec;
    };

} // namespace ocr
