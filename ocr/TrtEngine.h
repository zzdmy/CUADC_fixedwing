// ocr/TrtEngine.h
//
// 通用「单输入 / 单输出」TensorRT 引擎封装，供 det / ori 两个新增阶段复用。
// 与 PaddleOCRRec 同一套 auto-build 骨架：
//   1. 首次运行若无 .engine，则用 nvonnxparser 从 .onnx 构建并落盘缓存；
//   2. 之后每次启动只 deserialize，秒级加载；
//   3. 输入为单个 4 维 float 张量（NCHW），输出为单个 float 张量（任意形状）；
//   4. 动态输入形状由 optimization profile 决定（min/opt/max）。
#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cstddef>
#include <string>
#include <vector>

namespace ocr {

    /// @brief TensorRT 日志器（det/ori 共用；rec 用自己的 RecLogger 避免冲突）。
    class TrtLogger : public nvinfer1::ILogger {
    public:
        void log(Severity severity, const char* msg) noexcept override;
    };

    struct TrtEngineProfile {
        nvinfer1::Dims minShape{};
        nvinfer1::Dims optShape{};
        nvinfer1::Dims maxShape{};
    };

    class TrtEngine {
    public:
        TrtEngine(std::string onnxPath, std::string enginePath,
                  TrtEngineProfile profile, bool useFp16, int deviceId);
        ~TrtEngine();

        TrtEngine(const TrtEngine&) = delete;
        TrtEngine& operator=(const TrtEngine&) = delete;

        bool ready() const { return m_ready; }
        const std::string& inputName() const { return m_inputName; }
        const std::string& outputName() const { return m_outputName; }

        /// @brief 设定本次输入形状（动态），返回对应输出形状（各维 > 0）。
        nvinfer1::Dims setInputShape(const nvinfer1::Dims& inDims);

        /// @brief 推理。hostInput 已是 CHW float（元素数 = inDims 各维之积），
        ///        结果写入 hostOutput，outDims 返回实际输出形状。
        bool run(const float* hostInput, const nvinfer1::Dims& inDims,
                 std::vector<float>& hostOutput, nvinfer1::Dims& outDims);

    private:
        bool buildEngineFromOnnx();
        bool loadEngineFromFile();
        bool deserializeEngine(const std::vector<char>& data);
        bool createContextAndBuffers();

        std::string m_onnxPath;
        std::string m_enginePath;
        TrtEngineProfile m_profile;
        bool m_useFp16;
        int m_deviceId;

        TrtLogger m_logger;
        nvinfer1::IRuntime* m_runtime = nullptr;
        nvinfer1::ICudaEngine* m_engine = nullptr;
        nvinfer1::IExecutionContext* m_context = nullptr;

        std::string m_inputName;
        std::string m_outputName;
        int m_inputIndex = -1;
        int m_outputIndex = -1;

        void* m_devInput = nullptr;
        void* m_devOutput = nullptr;
        size_t m_inputBytes = 0;
        size_t m_outputBytes = 0;

        cudaStream_t m_stream = nullptr;
        bool m_ready = false;
    };

} // namespace ocr
