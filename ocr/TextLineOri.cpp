// ocr/TextLineOri.cpp
#include "ocr/TextLineOri.h"

#include <cstring>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "AppLogger.h"

namespace ocr {

    TextLineOri::TextLineOri(const OriConfig& cfg) : m_cfg(cfg) {
        TrtEngineProfile prof;
        prof.minShape = nvinfer1::Dims{ 4, {1, 3, 80, 160} };
        prof.optShape = nvinfer1::Dims{ 4, {1, 3, 80, 160} };
        prof.maxShape = nvinfer1::Dims{ 4, {1, 3, 80, 160} };
        m_engine = std::make_unique<TrtEngine>(cfg.onnxPath, cfg.enginePath, prof, cfg.useFp16, cfg.deviceId);
        if (!m_engine->ready()) {
            AppLogger::get().error("TextLineOri 引擎初始化失败");
        }
    }

    TextLineOri::~TextLineOri() = default;

    bool TextLineOri::isUpsideDown(const cv::Mat& bgr) {
        if (!isReady() || bgr.empty() || bgr.channels() != 3) {
            return false;
        }

        // ResizeImage：固定 160(宽) x 80(高)
        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(160, 80), 0, 0, cv::INTER_LINEAR);

        // NormalizeImage: (x/255 - mean) / std
        cv::Mat f;
        resized.convertTo(f, CV_32FC3, 1.0 / 255.0);
        std::vector<cv::Mat> ch(3);
        cv::split(f, ch);
        ch[0] = (ch[0] - 0.485f) / 0.229f;   // B
        ch[1] = (ch[1] - 0.456f) / 0.224f;   // G
        ch[2] = (ch[2] - 0.406f) / 0.225f;   // R

        std::vector<float> input(3ULL * 80 * 160);
        for (int c = 0; c < 3; ++c) {
            std::memcpy(input.data() + (size_t)c * 80 * 160, ch[c].ptr<float>(),
                (size_t)80 * 160 * sizeof(float));
        }

        nvinfer1::Dims inD{ 4, {1, 3, 80, 160} };
        std::vector<float> hostOut;
        nvinfer1::Dims outD{};
        if (!m_engine->run(input.data(), inD, hostOut, outD)) {
            return false;
        }
        if (hostOut.size() < 2) {
            return false;
        }
        // 输出 [1,2] softmax：0=0_degree, 1=180_degree
        return hostOut[1] > hostOut[0];
    }

} // namespace ocr
