// ocr/PaddleDet.h
//
// PP-OCRv6_medium_det (DB) 文本行检测的 TensorRT 封装。
// 输入 BGR 图，输出文本行四角框（原图坐标，按阅读顺序排序）。
//
// 模型 I/O（trtexec 实测）：
//   输入 x            : float32 [1,3,H,W]（动态，长边 <= limitSide 且为 32 的倍数）
//   输出 fetch_name_0 : float32 [1,1,H,W]（概率图，与输入同分辨率，已含 sigmoid）
// 后处理              : DBPostProcess（box_thresh=0.45, thresh=0.2, unclip_ratio=1.4）
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "ocr/TrtEngine.h"

namespace ocr {

    class PaddleDet {
    public:
        struct DetConfig {
            std::string onnxPath;
            std::string enginePath;
            int   limitSide = 960;      // 输入长边上限
            float boxThresh = 0.45f;    // 文本框置信度阈值
            float thresh = 0.2f;        // 概率图二值化阈值
            float unclipRatio = 1.4f;   // 框外扩比例
            bool  useFp16 = true;
            int   deviceId = 0;
        };

        struct TextBox {
            std::vector<cv::Point2f> pts;  // 4 角点（原图坐标）
            float score = 0.0f;

            cv::Rect bbox() const {
                if (pts.empty()) return {};
                float x0 = pts[0].x, x1 = pts[0].x, y0 = pts[0].y, y1 = pts[0].y;
                for (const auto& p : pts) {
                    x0 = std::min(x0, p.x); x1 = std::max(x1, p.x);
                    y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
                }
                return cv::Rect(cv::Point((int)x0, (int)y0), cv::Point((int)std::ceil(x1), (int)std::ceil(y1)));
            }
        };

        explicit PaddleDet(const DetConfig& cfg);
        ~PaddleDet();

        PaddleDet(const PaddleDet&) = delete;
        PaddleDet& operator=(const PaddleDet&) = delete;

        bool isReady() const { return m_engine && m_engine->ready(); }

        /// @brief 在 BGR 图上检测文本行，返回原图坐标系的文本框（按阅读顺序排序）。
        std::vector<TextBox> detect(const cv::Mat& bgr);

    private:
        DetConfig m_cfg;
        std::unique_ptr<TrtEngine> m_engine;
    };

} // namespace ocr
