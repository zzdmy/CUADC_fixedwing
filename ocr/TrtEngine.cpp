// ocr/TrtEngine.cpp
#include "ocr/TrtEngine.h"

#include <NvOnnxParser.h>

#include <cstring>
#include <fstream>
#include <sstream>

#include "AppLogger.h"

namespace ocr {

    namespace {
        bool fileExists(const std::string& p) {
            if (p.empty()) return false;
            std::ifstream f(p, std::ios::binary);
            return f.good();
        }

        size_t prodDims(const nvinfer1::Dims& d) {
            size_t n = 1;
            for (int i = 0; i < d.nbDims; ++i) n *= static_cast<size_t>(d.d[i]);
            return n;
        }

        std::string dimsToStr(const nvinfer1::Dims& d) {
            std::ostringstream oss;
            for (int k = 0; k < d.nbDims; ++k) {
                if (k) oss << "x";
                if (d.d[k] > 0) oss << d.d[k]; else oss << "dyn";
            }
            return oss.str();
        }
    } // namespace

    void TrtLogger::log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept {
        if (severity == nvinfer1::ILogger::Severity::kINTERNAL_ERROR ||
            severity == nvinfer1::ILogger::Severity::kERROR) {
            AppLogger::get().error("[TRT-OCR] {}", msg ? msg : "");
        }
        else if (severity == nvinfer1::ILogger::Severity::kWARNING) {
            AppLogger::get().warn("[TRT-OCR] {}", msg ? msg : "");
        }
    }

    TrtEngine::TrtEngine(std::string onnxPath, std::string enginePath,
                         TrtEngineProfile profile, bool useFp16, int deviceId)
        : m_onnxPath(std::move(onnxPath)), m_enginePath(std::move(enginePath)),
          m_profile(profile), m_useFp16(useFp16), m_deviceId(deviceId) {
        bool loaded = false;
        if (fileExists(m_enginePath)) {
            loaded = loadEngineFromFile();
        }
        if (!loaded) {
            if (!fileExists(m_onnxPath)) {
                AppLogger::get().error("OCR 模型缺失: onnx 与 engine 都不存在 (onnx={}, engine={})",
                    m_onnxPath, m_enginePath);
                return;
            }
            AppLogger::get().info("OCR 未找到缓存 engine，开始从 ONNX 构建（一次性）: {}", m_onnxPath);
            if (!buildEngineFromOnnx()) {
                AppLogger::get().error("OCR engine 构建失败: {}", m_onnxPath);
                return;
            }
        }
        if (!createContextAndBuffers()) {
            return;
        }
        m_ready = true;
        AppLogger::get().info("OCR 引擎就绪: in='{}' out='{}'", m_inputName, m_outputName);
    }

    TrtEngine::~TrtEngine() {
        if (m_stream) { cudaStreamDestroy(m_stream); m_stream = nullptr; }
        if (m_devInput) { cudaFree(m_devInput); m_devInput = nullptr; }
        if (m_devOutput) { cudaFree(m_devOutput); m_devOutput = nullptr; }
        if (m_context) { delete m_context; m_context = nullptr; }
        if (m_engine) { delete m_engine; m_engine = nullptr; }
        if (m_runtime) { delete m_runtime; m_runtime = nullptr; }
    }

    bool TrtEngine::buildEngineFromOnnx() {
        std::ifstream f(m_onnxPath, std::ios::binary | std::ios::ate);
        if (!f.is_open()) {
            AppLogger::get().error("无法打开 ONNX: {}", m_onnxPath);
            return false;
        }
        std::streamsize size = f.tellg();
        f.seekg(0);
        std::vector<char> onnxData(static_cast<size_t>(size));
        f.read(onnxData.data(), size);
        f.close();

        m_runtime = nvinfer1::createInferRuntime(m_logger);
        if (!m_runtime) {
            AppLogger::get().error("createInferRuntime 失败");
            return false;
        }

        nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(m_logger);
        if (!builder) {
            AppLogger::get().error("createInferBuilder 失败");
            return false;
        }
        nvinfer1::INetworkDefinition* network = builder->createNetworkV2(0);
        if (!network) {
            AppLogger::get().error("createNetworkV2 失败");
            delete builder;
            return false;
        }
        nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, m_logger);
        if (!parser) {
            AppLogger::get().error("createParser 失败");
            delete network;
            delete builder;
            return false;
        }
        if (!parser->parse(onnxData.data(), static_cast<int>(onnxData.size()))) {
            AppLogger::get().error("ONNX 解析失败 ({} 个子错误)", parser->getNbErrors());
            for (int i = 0; i < parser->getNbErrors(); ++i) {
                AppLogger::get().error("  {}", parser->getError(i)->desc());
            }
            delete parser;
            delete network;
            delete builder;
            return false;
        }

        nvinfer1::ITensor* inputTensor = network->getInput(0);
        if (inputTensor->getDimensions().nbDims != 4) {
            AppLogger::get().error("期望 4 维输入 (N,C,H,W)，实际 {} 维",
                inputTensor->getDimensions().nbDims);
            delete parser;
            delete network;
            delete builder;
            return false;
        }

        nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
        if (!config) {
            AppLogger::get().error("createBuilderConfig 失败");
            delete parser;
            delete network;
            delete builder;
            return false;
        }

        // 注意：IOptimizationProfile 析构为 protected，不能 delete；addOptimizationProfile 后由 config 接管所有权。
        auto* profile = builder->createOptimizationProfile();
        if (!profile) {
            AppLogger::get().error("createOptimizationProfile 失败");
            delete config;
            delete parser;
            delete network;
            delete builder;
            return false;
        }
        profile->setDimensions(inputTensor->getName(), nvinfer1::OptProfileSelector::kMIN, m_profile.minShape);
        profile->setDimensions(inputTensor->getName(), nvinfer1::OptProfileSelector::kOPT, m_profile.optShape);
        profile->setDimensions(inputTensor->getName(), nvinfer1::OptProfileSelector::kMAX, m_profile.maxShape);

        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30); // 1GB
        if (config->addOptimizationProfile(profile) < 0) {
            AppLogger::get().error("addOptimizationProfile 失败");
            delete config;
            delete parser;
            delete network;
            delete builder;
            return false;
        }
        if (m_useFp16 && builder->platformHasFastFp16()) {
            config->setFlag(nvinfer1::BuilderFlag::kFP16);
        }

        AppLogger::get().info("开始构建 TensorRT engine ({} -> {} -> {})...",
            dimsToStr(m_profile.minShape), dimsToStr(m_profile.optShape), dimsToStr(m_profile.maxShape));
        nvinfer1::IHostMemory* serialized = builder->buildSerializedNetwork(*network, *config);
        if (!serialized) {
            AppLogger::get().error("buildSerializedNetwork 失败");
            delete config;
            delete parser;
            delete network;
            delete builder;
            return false;
        }

        if (!m_enginePath.empty()) {
            std::ofstream out(m_enginePath, std::ios::binary);
            if (out.is_open()) {
                out.write(static_cast<const char*>(serialized->data()), serialized->size());
                out.close();
                AppLogger::get().info("OCR engine 已缓存: {} ({} MB)", m_enginePath,
                    serialized->size() / (1024 * 1024));
            }
            else {
                AppLogger::get().warn("engine 缓存写入失败（不影响本次运行）: {}", m_enginePath);
            }
        }

        std::vector<char> blob(static_cast<size_t>(serialized->size()));
        std::memcpy(blob.data(), serialized->data(), static_cast<size_t>(serialized->size()));
        delete serialized;
        delete config;
        delete parser;
        delete network;
        delete builder;

        return deserializeEngine(blob);
    }

    bool TrtEngine::loadEngineFromFile() {
        std::ifstream f(m_enginePath, std::ios::binary | std::ios::ate);
        if (!f.is_open()) return false;
        std::streamsize size = f.tellg();
        if (size <= 0) return false;
        f.seekg(0);
        std::vector<char> data(static_cast<size_t>(size));
        f.read(data.data(), size);
        f.close();

        AppLogger::get().info("加载缓存 OCR engine: {}", m_enginePath);
        m_runtime = nvinfer1::createInferRuntime(m_logger);
        if (!m_runtime) {
            AppLogger::get().error("createInferRuntime 失败");
            return false;
        }
        if (!deserializeEngine(data)) {
            AppLogger::get().warn("缓存 engine 反序列化失败，将回退到 ONNX 重建");
            if (m_runtime) { delete m_runtime; m_runtime = nullptr; }
            return false;
        }
        return true;
    }

    bool TrtEngine::deserializeEngine(const std::vector<char>& data) {
        m_engine = m_runtime->deserializeCudaEngine(data.data(), data.size());
        return m_engine != nullptr;
    }

    bool TrtEngine::createContextAndBuffers() {
        if (!m_engine) return false;

        m_context = m_engine->createExecutionContext();
        if (!m_context) {
            AppLogger::get().error("createExecutionContext 失败");
            return false;
        }
        if (cudaStreamCreate(&m_stream) != cudaSuccess) {
            AppLogger::get().error("cudaStreamCreate 失败");
            return false;
        }
        if (m_engine->getNbOptimizationProfiles() > 0) {
            m_context->setOptimizationProfileAsync(0, m_stream);
        }

        // 解析输入/输出张量
        m_inputIndex = -1;
        m_outputIndex = -1;
        const int n = m_engine->getNbIOTensors();
        for (int i = 0; i < n; ++i) {
            const char* name = m_engine->getIOTensorName(i);
            if (m_engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT &&
                m_inputIndex < 0) {
                m_inputIndex = i;
                m_inputName = name;
            }
            else if (m_engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT &&
                m_outputIndex < 0) {
                m_outputIndex = i;
                m_outputName = name;
            }
        }
        if (m_inputIndex < 0 || m_outputIndex < 0) {
            AppLogger::get().error("未找到输入/输出张量 (in={} out={})", m_inputIndex, m_outputIndex);
            return false;
        }

        // 输入缓冲：按 profile 最大形状分配
        m_inputBytes = prodDims(m_profile.maxShape) * sizeof(float);
        if (cudaMalloc(&m_devInput, m_inputBytes) != cudaSuccess) {
            AppLogger::get().error("cudaMalloc 输入失败");
            return false;
        }

        // 输出缓冲：先按最大输入形状求输出上界
        if (!m_context->setInputShape(m_inputName.c_str(), m_profile.maxShape)) {
            AppLogger::get().error("设定最大输入形状失败 ({})", dimsToStr(m_profile.maxShape));
            return false;
        }
        nvinfer1::Dims outMax = m_context->getTensorShape(m_outputName.c_str());
        m_outputBytes = prodDims(outMax) * sizeof(float);
        if (m_outputBytes == 0) {
            AppLogger::get().error("输出缓冲大小为 0（输出形状异常: {}）", dimsToStr(outMax));
            return false;
        }
        if (cudaMalloc(&m_devOutput, m_outputBytes) != cudaSuccess) {
            AppLogger::get().error("cudaMalloc 输出失败");
            return false;
        }

        m_context->setTensorAddress(m_inputName.c_str(), m_devInput);
        m_context->setTensorAddress(m_outputName.c_str(), m_devOutput);
        return true;
    }

    nvinfer1::Dims TrtEngine::setInputShape(const nvinfer1::Dims& inDims) {
        nvinfer1::Dims out{};
        if (!m_ready || !m_context) return out;
        if (!m_context->setInputShape(m_inputName.c_str(), inDims)) {
            AppLogger::get().error("setInputShape 失败 ({})", dimsToStr(inDims));
            return out;
        }
        out = m_context->getTensorShape(m_outputName.c_str());
        return out;
    }

    bool TrtEngine::run(const float* hostInput, const nvinfer1::Dims& inDims,
                        std::vector<float>& hostOutput, nvinfer1::Dims& outDims) {
        if (!m_ready || !m_context || !hostInput) return false;

        const size_t inBytes = prodDims(inDims) * sizeof(float);
        if (inBytes > m_inputBytes) {
            AppLogger::get().error("输入超出预分配缓冲 ({} > {})", inBytes, m_inputBytes);
            return false;
        }

        if (!m_context->setInputShape(m_inputName.c_str(), inDims)) {
            AppLogger::get().error("run setInputShape 失败 ({})", dimsToStr(inDims));
            return false;
        }
        outDims = m_context->getTensorShape(m_outputName.c_str());
        const size_t outBytes = prodDims(outDims) * sizeof(float);
        if (outBytes == 0 || outBytes > m_outputBytes) {
            AppLogger::get().error("输出形状异常或超出缓冲 ({} > {})", outBytes, m_outputBytes);
            return false;
        }

        // 按实际输出大小扩好 host 缓冲（调用方可能传入空 vector）。
        hostOutput.resize(outBytes / sizeof(float));

        cudaError_t err = cudaMemcpyAsync(m_devInput, hostInput, inBytes, cudaMemcpyHostToDevice, m_stream);
        if (err != cudaSuccess) {
            AppLogger::get().error("H2D 拷贝失败: {}", cudaGetErrorString(err));
            return false;
        }
        if (!m_context->enqueueV3(m_stream)) {
            AppLogger::get().error("enqueueV3 失败");
            return false;
        }
        err = cudaMemcpyAsync(hostOutput.data(), m_devOutput, outBytes, cudaMemcpyDeviceToHost, m_stream);
        if (err != cudaSuccess) {
            AppLogger::get().error("D2H 拷贝失败: {}", cudaGetErrorString(err));
            return false;
        }
        if (cudaStreamSynchronize(m_stream) != cudaSuccess) {
            AppLogger::get().error("stream 同步失败");
            return false;
        }
        return true;
    }

} // namespace ocr
