// ocr/PaddleOCRRec.h
//
// PP-OCRv5 文本行识别模型的 TensorRT 推理封装（纯 C++，无 Python 依赖）。
//
// 模型：PaddlePaddle/PP-OCRv5_mobile_rec_onnx （16.5MB）
//   实测图结构（用 protobuf 直接解析该 ONNX 得到）：
//     输入 x           : float32 [N, 3, 48, 320]，高 48；宽/批 为符号维度
//                        (Paddle 导出为 DynamicDimension.0/.1，ONNX 里读不到数字)
//     输出 fetch_name_0: float32 [N, T, C]，已含 Softmax
//                        T = W/8；C = 18385 = 字典 18383 + blank + 空格
//                        维度顺序由末尾 Transpose.8 的 perm=[0,2,1] 保证（[N,C,T]->[N,T,C]）
//   后处理          : CTCLabelDecode（贪心解码：去 blank、去连续重复）
//
// 设计要点：
//   1. 首次运行若没有 .engine，则用 nvonnxparser 直接从 .onnx 构建并落盘缓存；
//      之后每次启动只做 deserialize，秒级加载。
//   2. 张量名不硬编码 —— 直接按张量索引向 engine 查询，兼容不同的 ONNX 导出命名。
//   3. 输出张量通过「最后一维 == 字典大小 + 2」自动识别，避开兼容用的
//      另一个分支输出。
//   4. 宽度区间来自本配置的 optimization profile，不依赖 ONNX 里的符号形状。
#pragma once

#include <NvInfer.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "ocr/OcrCharset.h"

namespace ocr {

    struct RecResult {
        std::string text;        // 解码出的完整文本（仅保留数字，见 setDigitOnly）
        float confidence = 0.0f; // 被保留字符的平均置信度
        bool empty() const { return text.empty(); }
    };

    struct PaddleRecConfig {
        std::string onnxPath;          // PP-OCRv5_mobile_rec.onnx
        std::string enginePath;        // 缓存的 .engine（不存在则自动构建）
        std::string dictPath;          // 字符字典 txt（每行一个字符）
        std::string inferenceYmlPath;  // 备选：直接从 inference.yml 解析字典
        int   recHeight = 48;          // 模型输入高度，固定 48
        int   recMaxWidth = 320;       // 动态宽度上限（也是 profile 的 max）
        int   recMinWidth = 16;        // profile 的 min
        int   recOptWidth = 320;       // profile 的 opt
        bool  useFp16 = true;          // 构建 engine 时启用 FP16
        int   deviceId = 0;
        bool  digitOnly = true;        // 只保留数字字符（竞赛编号场景）
    };

    /// @brief TensorRT 日志器（仅本模块使用，避免与 YoloV8 的 Logger 冲突）
    class RecLogger : public nvinfer1::ILogger {
    public:
        void log(Severity severity, const char* msg) noexcept override;
    };

    class PaddleOCRRec {
    public:
        explicit PaddleOCRRec(const PaddleRecConfig& cfg);
        ~PaddleOCRRec();

        PaddleOCRRec(const PaddleOCRRec&) = delete;
        PaddleOCRRec& operator=(const PaddleOCRRec&) = delete;

        /// @brief 引擎是否加载成功。
        bool isReady() const { return m_ready; }

        /// @brief 识别单张 BGR 图（应为已裁好的单行文字图）。
        /// @note 非线程安全：请在同一个线程内调用（OcrDigitReader 已保证）。
        RecResult run(const cv::Mat& bgr);

        /// @brief 模型实际支持的动态宽度区间 [min, max]，用于日志与裁剪校验。
        void widthRange(int& minW, int& maxW) const { minW = m_minWidth; maxW = m_maxWidth; }

        /// @brief 字典信息（用于启动日志）。
        const OcrCharset& charset() const { return m_charset; }

    private:
        bool buildEngineFromOnnx();
        bool loadEngineFromFile();
        bool deserializeEngine(const std::vector<char>& data);
        bool createContextAndBuffers();
        bool resolveTensors();
        /// @brief 预处理：写入 m_hostInput，返回实际使用的对齐后宽度。
        int preprocess(const cv::Mat& bgr);
        RecResult decode(const float* logits, int T, int C) const;

        PaddleRecConfig m_cfg;
        RecLogger m_logger;
        OcrCharset m_charset;

        nvinfer1::IRuntime* m_runtime = nullptr;
        nvinfer1::ICudaEngine* m_engine = nullptr;
        nvinfer1::IExecutionContext* m_context = nullptr;

        int m_inputIndex = -1;
        int m_outputIndex = -1;
        int m_numClasses = -1;

        int m_minWidth = 16;
        int m_optWidth = 320;
        int m_maxWidth = 320;

        void* m_devInput = nullptr;
        void* m_devOutput = nullptr;
        size_t m_inputBytes = 0;
        size_t m_outputBytes = 0;
        std::vector<float> m_hostInput;
        std::vector<float> m_hostOutput;

        cudaStream_t m_stream = nullptr;
        bool m_ready = false;
    };

} // namespace ocr
