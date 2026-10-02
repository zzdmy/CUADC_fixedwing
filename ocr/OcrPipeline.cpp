// ocr/OcrPipeline.cpp
#include "ocr/OcrPipeline.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "AppLogger.h"

namespace ocr {

    OcrPipeline::OcrPipeline(const OcrPipelineConfig& cfg) : m_cfg(cfg) {
        m_rec = std::make_unique<PaddleOCRRec>(m_cfg.rec);
        if (m_cfg.useDet) {
            m_det = std::make_unique<PaddleDet>(m_cfg.det);
        }
        if (m_cfg.useOri) {
            m_ori = std::make_unique<TextLineOri>(m_cfg.ori);
        }
    }

    OcrPipeline::~OcrPipeline() = default;

    bool OcrPipeline::isReady() const {
        if (!m_rec || !m_rec->isReady()) return false;
        if (m_cfg.useDet && (!m_det || !m_det->isReady())) return false;
        if (m_cfg.useOri && (!m_ori || !m_ori->isReady())) return false;
        return true;
    }

    RecResult OcrPipeline::run(const cv::Mat& bgr) {
        RecResult result;
        if (!isReady() || bgr.empty() || bgr.channels() != 3) {
            return result;
        }

        // 待识别的一行文字裁剪（可能有多行，按阅读顺序拼接）
        std::vector<cv::Mat> lines;

        if (m_cfg.useDet) {
            auto boxes = m_det->detect(bgr);
            const cv::Rect whole(0, 0, bgr.cols, bgr.rows);
            for (const auto& tb : boxes) {
                cv::Rect b = tb.bbox() & whole;
                if (b.area() <= 0) continue;
                // 外扩一点，防止贴边数字被切
                const int padX = (int)(b.width * m_cfg.cropPadding);
                const int padY = (int)(b.height * m_cfg.cropPadding);
                b = cv::Rect(b.x - padX, b.y - padY, b.width + 2 * padX, b.height + 2 * padY) & whole;
                if (b.area() <= 0) continue;
                lines.push_back(bgr(b));
            }
            // det 没框到任何文本行时回退到整图识别，避免漏读
            if (lines.empty()) {
                lines.push_back(bgr);
            }
        }
        else {
            lines.push_back(bgr);
        }

        double scoreSum = 0.0;
        int kept = 0;
        std::string text;
        // 方向对齐（方案B）：数字可能横放/侧放，rec 假设文字接近水平，
        // 对每行 0°/90°/180°/270° 各跑一次 rec，取置信度最高者。
        // 已覆盖 90°/270°/180° 倒放，不再依赖 ori 的 0/180 分类。
        static const int kRotateFlags[] = {
            cv::ROTATE_90_CLOCKWISE, cv::ROTATE_180, cv::ROTATE_90_COUNTERCLOCKWISE
        };
        for (cv::Mat& line : lines) {
            RecResult best;
            for (int i = 0; i < 4; ++i) {
                cv::Mat cand = line;                 // i==0：原图
                if (i > 0) {
                    cv::rotate(line, cand, kRotateFlags[i - 1]);
                }
                RecResult r = m_rec->run(cand);
                if (!r.text.empty() && r.confidence > best.confidence) {
                    best = std::move(r);
                }
            }
            if (!best.text.empty()) {
                text += best.text;
                scoreSum += best.confidence;
                ++kept;
            }
        }

        result.text = std::move(text);
        result.confidence = kept > 0 ? (float)(scoreSum / kept) : 0.0f;
        return result;
    }

} // namespace ocr
