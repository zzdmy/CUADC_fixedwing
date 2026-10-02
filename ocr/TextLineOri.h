// ocr/TextLineOri.h
//
// PP-LCNet_x1_0_textline_ori 文本方向分类（0° / 180°）的 TensorRT 封装。
// 用于纠正数字行被倒放（6/9 上下颠倒）的情况。
//
// 模型 I/O（trtexec 实测）：
//   输入 x            : float32 [1,3,80,160]（静态）
//   输出 fetch_name_0 : float32 [1,2]（softmax：0=0_degree, 1=180_degree）
#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "ocr/TrtEngine.h"

namespace ocr {

    class TextLineOri {
    public:
        struct OriConfig {
            std::string onnxPath;
            std::string enginePath;
            bool useFp16 = true;
            int  deviceId = 0;
        };

        explicit TextLineOri(const OriConfig& cfg);
        ~TextLineOri();

        TextLineOri(const TextLineOri&) = delete;
        TextLineOri& operator=(const TextLineOri&) = delete;

        bool isReady() const { return m_engine && m_engine->ready(); }

        /// @brief 判断文本行是否倒放。
        /// @return true = 180° 倒放（需旋转）；false = 正常（或推理失败，按正常处理）。
        bool isUpsideDown(const cv::Mat& bgr);

    private:
        OriConfig m_cfg;
        std::unique_ptr<TrtEngine> m_engine;
    };

} // namespace ocr
