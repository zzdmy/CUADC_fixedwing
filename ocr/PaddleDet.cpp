// ocr/PaddleDet.cpp
#include "ocr/PaddleDet.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <opencv2/imgproc.hpp>

#include "AppLogger.h"

namespace ocr {
    namespace {

        /// @brief 手工计算 RotatedRect 的 4 个顶点（等价 cv::boxPoints，顺序顺时针）。
        /// 直接实现避免依赖 cv::boxPoints（某些 OpenCV 构建下对空 vector 输出异常）。
        std::vector<cv::Point2f> rectPoints(const cv::RotatedRect& rr) {
            float angle = rr.angle;
            if (angle < -45.0f) angle += 90.0f;
            const float theta = angle * static_cast<float>(CV_PI) / 180.0f;
            const float b = std::cos(theta) * 0.5f;
            const float a = std::sin(theta) * 0.5f;
            const float cx = rr.center.x, cy = rr.center.y;
            const float w = rr.size.width, h = rr.size.height;
            std::vector<cv::Point2f> pts(4);
            pts[0] = cv::Point2f(cx - a * h - b * w, cy + b * h - a * w);
            pts[1] = cv::Point2f(cx + a * h - b * w, cy - b * h - a * w);
            pts[2] = cv::Point2f(2 * cx - pts[0].x, 2 * cy - pts[0].y);
            pts[3] = cv::Point2f(2 * cx - pts[1].x, 2 * cy - pts[1].y);
            return pts;
        }

        /// @brief 概率图上计算框内平均分（等价 PaddleOCR box_score_fast）。
        float boxScoreFast(const float* prob, int W, int H, const std::vector<cv::Point2f>& box) {
            if (box.size() < 4) return 0.0f;
            float xminF = box[0].x, xmaxF = box[0].x, yminF = box[0].y, ymaxF = box[0].y;
            for (const auto& p : box) {
                xminF = std::min(xminF, p.x); xmaxF = std::max(xmaxF, p.x);
                yminF = std::min(yminF, p.y); ymaxF = std::max(ymaxF, p.y);
            }
            int xmin = std::clamp((int)std::floor(xminF), 0, W - 1);
            int xmax = std::clamp((int)std::ceil(xmaxF), 0, W - 1);
            int ymin = std::clamp((int)std::floor(yminF), 0, H - 1);
            int ymax = std::clamp((int)std::ceil(ymaxF), 0, H - 1);

            cv::Mat mask(ymax - ymin + 1, xmax - xmin + 1, CV_8UC1, cv::Scalar(0));
            std::vector<cv::Point> poly;
            poly.reserve(box.size());
            for (const auto& p : box) {
                poly.emplace_back((int)std::round(p.x - xmin), (int)std::round(p.y - ymin));
            }
            cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{poly}, cv::Scalar(1));

            double sum = 0.0;
            int cnt = 0;
            for (int y = ymin; y <= ymax; ++y) {
                const float* row = prob + (size_t)y * W;
                const uchar* m = mask.ptr<uchar>(y - ymin);
                for (int x = xmin; x <= xmax; ++x) {
                    if (m[x - xmin]) { sum += row[x]; ++cnt; }
                }
            }
            return cnt > 0 ? (float)(sum / cnt) : 0.0f;
        }

    } // namespace

    PaddleDet::PaddleDet(const DetConfig& cfg) : m_cfg(cfg) {
        TrtEngineProfile prof;
        // 输入 [1,3,H,W]：长边上限 limitSide，短边下限 32，均为 32 的倍数
        prof.minShape = nvinfer1::Dims{ 4, {1, 3, 32, 32} };
        prof.optShape = nvinfer1::Dims{ 4, {1, 3, 640, 640} };
        prof.maxShape = nvinfer1::Dims{ 4, {1, 3, cfg.limitSide, cfg.limitSide} };
        m_engine = std::make_unique<TrtEngine>(cfg.onnxPath, cfg.enginePath, prof, cfg.useFp16, cfg.deviceId);
        if (!m_engine->ready()) {
            AppLogger::get().error("PaddleDet 引擎初始化失败");
        }
    }

    PaddleDet::~PaddleDet() = default;

    std::vector<PaddleDet::TextBox> PaddleDet::detect(const cv::Mat& bgr) {
        std::vector<TextBox> out;
        if (!isReady() || bgr.empty() || bgr.channels() != 3) {
            return out;
        }

        const int srcH = bgr.rows;
        const int srcW = bgr.cols;

        // ---- 1. DetResizeForTest：长边限幅 + 32 对齐 ----
        const float ratio = (std::max(srcH, srcW) > m_cfg.limitSide)
            ? (float)m_cfg.limitSide / std::max(srcH, srcW) : 1.0f;
        int rh = std::max((int)std::round((int)(srcH * ratio) / 32.0) * 32, 32);
        int rw = std::max((int)std::round((int)(srcW * ratio) / 32.0) * 32, 32);
        const float ratioH = (float)rh / srcH;
        const float ratioW = (float)rw / srcW;

        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(rw, rh), 0, 0, cv::INTER_LINEAR);

        // ---- 2. 归一化 (x/255 - mean) / std，BGR 通道序 ----
        cv::Mat f;
        resized.convertTo(f, CV_32FC3, 1.0 / 255.0);
        std::vector<cv::Mat> ch(3);
        cv::split(f, ch);
        ch[0] = (ch[0] - 0.485f) / 0.229f;   // B
        ch[1] = (ch[1] - 0.456f) / 0.224f;   // G
        ch[2] = (ch[2] - 0.406f) / 0.225f;   // R

        std::vector<float> input(3ULL * rh * rw);
        for (int c = 0; c < 3; ++c) {
            std::memcpy(input.data() + (size_t)c * rh * rw, ch[c].ptr<float>(),
                (size_t)rh * rw * sizeof(float));
        }

        // ---- 3. 推理 ----
        nvinfer1::Dims inD{ 4, {1, 3, rh, rw} };
        std::vector<float> hostOut;
        nvinfer1::Dims outD{};
        if (!m_engine->run(input.data(), inD, hostOut, outD)) {
            return out;
        }
        // 期望 [1,1,rh,rw]
        if (outD.nbDims != 4 || outD.d[2] != rh || outD.d[3] != rw) {
            AppLogger::get().error("PaddleDet 输出形状异常 (期望 1x1x{}x{}, 实得 {}x{}x{}x{})",
                rh, rw, outD.d[0], outD.d[1], outD.d[2], outD.d[3]);
            return out;
        }
        const float* prob = hostOut.data();

        // ---- 4. DBPostProcess ----
        // 4.1 二值化
        cv::Mat bit(rh, rw, CV_8UC1);
        for (int y = 0; y < rh; ++y) {
            uchar* row = bit.ptr<uchar>(y);
            const float* p = prob + (size_t)y * rw;
            for (int x = 0; x < rw; ++x) {
                row[x] = p[x] > m_cfg.thresh ? 255 : 0;
            }
        }

        // 4.2 轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bit, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            cv::RotatedRect rr = cv::minAreaRect(contour);
            float minSide = std::min(rr.size.width, rr.size.height);
            if (minSide < 3.0f) continue;   // min_size = 3

            std::vector<cv::Point2f> pts = rectPoints(rr);
            float score = boxScoreFast(prob, rw, rh, pts);
            if (score < m_cfg.boxThresh) continue;

            // unclip：矩形外扩 distance = area*ratio/perimeter
            const float w = rr.size.width, h = rr.size.height;
            const float distance = (w * h) * m_cfg.unclipRatio / (2.0f * (w + h));
            cv::RotatedRect rrEx(rr.center, cv::Size2f(w + 2.0f * distance, h + 2.0f * distance), rr.angle);
            std::vector<cv::Point2f> ex = rectPoints(rrEx);
            float minSideEx = std::min(rrEx.size.width, rrEx.size.height);
            if (minSideEx < 5.0f) continue; // min_size + 2

            TextBox tb;
            tb.score = score;
            for (const auto& p : ex) {
                float x = std::clamp(p.x, 0.0f, (float)rw) / ratioW;
                float y = std::clamp(p.y, 0.0f, (float)rh) / ratioH;
                tb.pts.emplace_back(x, y);
            }
            out.push_back(std::move(tb));
        }

        // ---- 5. 按阅读顺序排序（先上到下，再左到右） ----
        std::sort(out.begin(), out.end(), [](const TextBox& a, const TextBox& b) {
            cv::Rect ra = a.bbox(), rb = b.bbox();
            const int ay = ra.y + ra.height / 2;
            const int by = rb.y + rb.height / 2;
            if (std::abs(ay - by) > std::max(ra.height, rb.height) / 2) {
                return ay < by;
            }
            return ra.x < rb.x;
        });

        return out;
    }

} // namespace ocr
