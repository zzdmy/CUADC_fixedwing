// ocr/PaddleOCRRec.cpp
#include "ocr/PaddleOCRRec.h"

#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>

#include "AppLogger.h"

namespace ocr {

    void RecLogger::log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept {
        // 只保留警告及以上，避免构建 engine 时刷屏
        if (severity == nvinfer1::ILogger::Severity::kINTERNAL_ERROR ||
            severity == nvinfer1::ILogger::Severity::kERROR) {
            AppLogger::get().error("[TRT-OCR] {}", msg ? msg : "");
        }
        else if (severity == nvinfer1::ILogger::Severity::kWARNING) {
            AppLogger::get().warn("[TRT-OCR] {}", msg ? msg : "");
        }
    }

    namespace {
        bool fileExists(const std::string& p) {
            if (p.empty()) return false;
            std::ifstream f(p, std::ios::binary);
            return f.good();
        }
    } // namespace

    PaddleOCRRec::PaddleOCRRec(const PaddleRecConfig& cfg) : m_cfg(cfg) {
        // ---- 1. 字符字典：优先 txt，其次 inference.yml ----
        if (!m_cfg.dictPath.empty() && m_charset.loadDict(m_cfg.dictPath)) {
            AppLogger::get().info("OCR 字符字典已加载(文件): {}", m_charset.describeFirst(3));
        }
        else if (!m_cfg.inferenceYmlPath.empty() &&
                 m_charset.loadFromInferenceYml(m_cfg.inferenceYmlPath)) {
            AppLogger::get().info("OCR 字符字典已加载(inference.yml): {}", m_charset.describeFirst(3));
        }
        else {
            AppLogger::get().error("OCR 字符字典加载失败! dict={} yml={}",
                m_cfg.dictPath, m_cfg.inferenceYmlPath);
            return;
        }
        if (m_cfg.digitOnly && m_charset.digitCount() == 0) {
            AppLogger::get().error("字符字典中没有数字字符，请检查字典文件是否正确。");
            return;
        }

        // ---- 2. 引擎：优先读缓存，缺失则从 onnx 构建 ----
        bool loaded = false;
        if (fileExists(m_cfg.enginePath)) {
            loaded = loadEngineFromFile();
        }
        if (!loaded) {
            if (!fileExists(m_cfg.onnxPath)) {
                AppLogger::get().error("OCR 模型缺失: onnx 与 engine 都不存在 (onnx={}, engine={})",
                    m_cfg.onnxPath, m_cfg.enginePath);
                return;
            }
            AppLogger::get().info("OCR 未找到缓存 engine，开始从 ONNX 构建（实测约 10~15 分钟，一次性）...");
            if (!buildEngineFromOnnx()) {
                AppLogger::get().error("OCR engine 构建失败");
                return;
            }
        }

        if (!createContextAndBuffers()) {
            return;
        }
        m_ready = true;
        AppLogger::get().info("OCR 识别引擎就绪: W=[{},{}] classes={} backend=TensorRT",
            m_minWidth, m_maxWidth, m_numClasses);
    }

    PaddleOCRRec::~PaddleOCRRec() {
        if (m_stream) { cudaStreamDestroy(m_stream); m_stream = nullptr; }
        if (m_devInput) { cudaFree(m_devInput); m_devInput = nullptr; }
        if (m_devOutput) { cudaFree(m_devOutput); m_devOutput = nullptr; }
        if (m_context) { delete m_context; m_context = nullptr; }
        if (m_engine) { delete m_engine; m_engine = nullptr; }
        if (m_runtime) { delete m_runtime; m_runtime = nullptr; }
    }

    // ============================================================
    // engine 构建 / 加载
    // ============================================================
    bool PaddleOCRRec::buildEngineFromOnnx() {
        std::ifstream f(m_cfg.onnxPath, std::ios::binary | std::ios::ate);
        if (!f.is_open()) {
            AppLogger::get().error("无法打开 ONNX: {}", m_cfg.onnxPath);
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

        // 注意：network 必须由 IBuilder 创建（IRuntime 没有 createNetworkV2）
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

        // 打印网络张量信息，便于核对
        // 注意：Paddle 导出的 ONNX 用符号维度（DynamicDimension.N）表达动态轴，
        // 所以这里可能打印 -1 或 0 —— 那不是错误，宽度区间由下面的 profile 决定。
        AppLogger::get().info("ONNX 解析成功: 输入 {} 个 / 输出 {} 个",
            network->getNbInputs(), network->getNbOutputs());
        auto dimsToStr = [](const nvinfer1::Dims& d) {
            std::ostringstream oss;
            for (int k = 0; k < d.nbDims; ++k) {
                if (k) oss << "x";
                if (d.d[k] > 0) oss << d.d[k];
                else oss << "dyn";   // 符号维度
            }
            return oss.str();
        };
        for (int i = 0; i < network->getNbInputs(); ++i) {
            auto* t = network->getInput(i);
            AppLogger::get().info("  in[{}] name='{}' shape={}", i,
                t->getName(), dimsToStr(t->getDimensions()));
        }
        for (int i = 0; i < network->getNbOutputs(); ++i) {
            auto* t = network->getOutput(i);
            AppLogger::get().info("  out[{}] name='{}' shape={}", i,
                t->getName(), dimsToStr(t->getDimensions()));
        }

        // 动态 shape profile：输入 x=[1,3,48,W]
        auto* profile = builder->createOptimizationProfile();
        if (!profile) {
            AppLogger::get().error("createOptimizationProfile 失败");
            delete parser;
            delete network;
            delete builder;
            return false;
        }

        nvinfer1::ITensor* inputTensor = network->getInput(0);
        int inDims = inputTensor->getDimensions().nbDims;
        if (inDims != 4) {
            AppLogger::get().error("期望 4 维输入 (N,C,H,W)，实际 {} 维", inDims);
            delete parser;
            delete network;
            return false;
        }

        m_minWidth = std::max(8, m_cfg.recMinWidth);
        m_optWidth = std::max(m_minWidth, m_cfg.recOptWidth);
        m_maxWidth = std::max(m_optWidth, m_cfg.recMaxWidth);

        nvinfer1::Dims minD{ 4, {1, 3, m_cfg.recHeight, m_minWidth} };
        nvinfer1::Dims optD{ 4, {1, 3, m_cfg.recHeight, m_optWidth} };
        nvinfer1::Dims maxD{ 4, {1, 3, m_cfg.recHeight, m_maxWidth} };
        profile->setDimensions(inputTensor->getName(), nvinfer1::OptProfileSelector::kMIN, minD);
        profile->setDimensions(inputTensor->getName(), nvinfer1::OptProfileSelector::kOPT, optD);
        profile->setDimensions(inputTensor->getName(), nvinfer1::OptProfileSelector::kMAX, maxD);

        // profile 必须挂到 config 上（TRT 10）
        nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
        if (!config) {
            AppLogger::get().error("createBuilderConfig 失败");
            delete parser;
            delete network;
            delete builder;
            return false;
        }
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30); // 1GB
        if (config->addOptimizationProfile(profile) < 0) {
            AppLogger::get().error("addOptimizationProfile 失败");
            delete config;
            delete parser;
            delete network;
            delete builder;
            return false;
        }

        // 注意：这里不调用 network->markOutput()。ONNX parser 已将图输出标记为
        // network-level output，重复标记会触发 TRT API Usage Error。

        if (m_cfg.useFp16 && builder->platformHasFastFp16()) {
            config->setFlag(nvinfer1::BuilderFlag::kFP16);
            AppLogger::get().info("OCR engine 启用 FP16");
        }

        AppLogger::get().info("开始构建 TensorRT engine（W: {} -> {} -> {}）...",
            m_minWidth, m_optWidth, m_maxWidth);
        nvinfer1::IHostMemory* serialized = builder->buildSerializedNetwork(*network, *config);
        if (!serialized) {
            AppLogger::get().error("buildSerializedNetwork 失败");
            delete config;
            delete parser;
            delete network;   // network 由 builder 创建，须先于 builder 销毁
            delete builder;
            return false;
        }

        // 落盘缓存
        if (!m_cfg.enginePath.empty()) {
            std::ofstream out(m_cfg.enginePath, std::ios::binary);
            if (out.is_open()) {
                out.write(static_cast<const char*>(serialized->data()), serialized->size());
                out.close();
                AppLogger::get().info("OCR engine 已缓存: {} ({} MB)", m_cfg.enginePath,
                    serialized->size() / (1024 * 1024));
            }
            else {
                AppLogger::get().warn("engine 缓存写入失败（不影响本次运行）: {}", m_cfg.enginePath);
            }
        }

        std::vector<char> blob(static_cast<size_t>(serialized->size()));
        std::memcpy(blob.data(), serialized->data(), static_cast<size_t>(serialized->size()));
        delete serialized;
        delete config;
        delete parser;
        delete network;   // 顺序很重要：network/parser 必须先于 builder 销毁
        delete builder;

        return deserializeEngine(blob);
    }

    bool PaddleOCRRec::loadEngineFromFile() {
        std::ifstream f(m_cfg.enginePath, std::ios::binary | std::ios::ate);
        if (!f.is_open()) {
            return false;
        }
        std::streamsize size = f.tellg();
        if (size <= 0) {
            return false;
        }
        f.seekg(0);
        std::vector<char> data(static_cast<size_t>(size));
        f.read(data.data(), size);
        f.close();

        AppLogger::get().info("加载缓存 OCR engine: {}", m_cfg.enginePath);
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

    bool PaddleOCRRec::deserializeEngine(const std::vector<char>& data) {
        m_engine = m_runtime->deserializeCudaEngine(data.data(), data.size());
        if (!m_engine) {
            return false;
        }

        // 读取 profile 里记录的宽度区间（便于日志/校验）
        for (int p = 0; p < m_engine->getNbOptimizationProfiles(); ++p) {
            for (int i = 0; i < m_engine->getNbIOTensors(); ++i) {
                const char* name = m_engine->getIOTensorName(i);
                if (m_engine->getTensorIOMode(name) != nvinfer1::TensorIOMode::kINPUT) continue;
                auto d = m_engine->getProfileShape(name, p, nvinfer1::OptProfileSelector::kMAX);
                if (d.nbDims == 4 && d.d[3] > 0) {
                    m_maxWidth = static_cast<int>(d.d[3]);
                }
                auto dmin = m_engine->getProfileShape(name, p, nvinfer1::OptProfileSelector::kMIN);
                if (dmin.nbDims == 4 && dmin.d[3] > 0) {
                    m_minWidth = static_cast<int>(dmin.d[3]);
                }
                auto dopt = m_engine->getProfileShape(name, p, nvinfer1::OptProfileSelector::kOPT);
                if (dopt.nbDims == 4 && dopt.d[3] > 0) {
                    m_optWidth = static_cast<int>(dopt.d[3]);
                }
            }
        }
        return true;
    }

    bool PaddleOCRRec::createContextAndBuffers() {
        if (!m_engine) {
            return false;
        }
        m_context = m_engine->createExecutionContext();
        if (!m_context) {
            AppLogger::get().error("createExecutionContext 失败");
            return false;
        }

        // 专用 CUDA stream：避免与检测线程共用默认 stream 造成隐式同步。
        if (cudaStreamCreate(&m_stream) != cudaSuccess) {
            AppLogger::get().error("OCR cudaStreamCreate 失败");
            return false;
        }
        // 输入形状是动态的，必须先声明"用 profile 0"，否则 TRT 拒绝设置形状。
        if (m_engine->getNbOptimizationProfiles() > 0) {
            m_context->setOptimizationProfileAsync(0, m_stream);
        }

        if (!resolveTensors()) {
            return false;
        }

        // 输入缓冲：按 profile 的最大宽度分配，后续窄图复用
        m_inputBytes = static_cast<size_t>(1) * 3 * m_cfg.recHeight * m_maxWidth * sizeof(float);
        m_hostInput.resize(m_inputBytes / sizeof(float));

        // 输出缓冲大小：不能依赖 getMaxOutputSize —— 在未设定输入形状前调用它
        // 会直接返回 0（TRT 报 "Not all shapes are specified"），据此分配 0 字节
        // 会导致后续 D2H 拷贝越界崩溃。
        // 正确做法：先把输入设为 profile 的最大宽度求出输出上界，再查询输出形状。
        const char* inName = m_engine->getIOTensorName(m_inputIndex);
        const char* outName = m_engine->getIOTensorName(m_outputIndex);

        nvinfer1::Dims maxInD{ 4, {1, 3, m_cfg.recHeight, m_maxWidth} };
        if (!m_context->setInputShape(inName, maxInD)) {
            AppLogger::get().error("OCR 设定最大输入形状失败 (W={})", m_maxWidth);
            return false;
        }
        nvinfer1::Dims outD = m_context->getTensorShape(outName);
        if (outD.nbDims != 3 || outD.d[1] <= 0 || outD.d[2] <= 0) {
            AppLogger::get().error("OCR 输出张量形状非法 (dims={})", outD.nbDims);
            return false;
        }
        m_numClasses = static_cast<int>(outD.d[2]);
        const int maxT = static_cast<int>(outD.d[1]);
        m_outputBytes = static_cast<size_t>(1) * maxT * m_numClasses * sizeof(float);
        AppLogger::get().info("OCR 输出缓冲: [1 x {} x {}] = {} KB",
            maxT, m_numClasses, m_outputBytes / 1024);
        m_hostOutput.resize(m_outputBytes / sizeof(float));

        if (cudaMalloc(&m_devInput, m_inputBytes) != cudaSuccess) {
            AppLogger::get().error("OCR cudaMalloc 输入失败");
            return false;
        }
        if (cudaMalloc(&m_devOutput, m_outputBytes) != cudaSuccess) {
            AppLogger::get().error("OCR cudaMalloc 输出失败");
            return false;
        }

        m_context->setTensorAddress(inName, m_devInput);
        m_context->setTensorAddress(outName, m_devOutput);
        return true;
    }

    bool PaddleOCRRec::resolveTensors() {
        const int n = m_engine->getNbIOTensors();
        m_inputIndex = -1;
        m_outputIndex = -1;

        // 输入：第一个 4 维 float 输入
        for (int i = 0; i < n; ++i) {
            const char* name = m_engine->getIOTensorName(i);
            if (m_engine->getTensorIOMode(name) != nvinfer1::TensorIOMode::kINPUT) continue;
            auto d = m_engine->getTensorShape(name);
            if (d.nbDims == 4) {
                m_inputIndex = i;
                AppLogger::get().info("OCR 输入张量: '{}'", name);
                break;
            }
        }
        if (m_inputIndex < 0) {
            AppLogger::get().error("未找到 4 维输入张量");
            return false;
        }

        // 输出：3 维且最后一维 == 字典大小 + 2 的那个（排除 gtc 分支）
        const int expectC = m_charset.numClasses();
        for (int i = 0; i < n; ++i) {
            const char* name = m_engine->getIOTensorName(i);
            if (m_engine->getTensorIOMode(name) != nvinfer1::TensorIOMode::kOUTPUT) continue;
            auto d = m_engine->getTensorShape(name);
            if (d.nbDims == 3 && d.d[2] == expectC) {
                m_outputIndex = i;
                m_numClasses = static_cast<int>(d.d[2]);
                AppLogger::get().info("OCR 输出张量(CTC): '{}' C={}", name, m_numClasses);
                break;
            }
        }
        if (m_outputIndex < 0) {
            // 退化路径：engine 未给出具体 C（理论上不应发生），
            // 退而选择 3 维输出中最后一维最大的那个，并打印全部候选。
            int bestC = 0;
            for (int i = 0; i < n; ++i) {
                const char* name = m_engine->getIOTensorName(i);
                if (m_engine->getTensorIOMode(name) != nvinfer1::TensorIOMode::kOUTPUT) continue;
                auto d = m_engine->getTensorShape(name);
                std::ostringstream oss;
                for (int k = 0; k < d.nbDims; ++k) {
                    if (k) oss << "x";
                    if (d.d[k] > 0) oss << d.d[k]; else oss << "dyn";
                }
                AppLogger::get().warn("  候选输出 '{}' shape={}", name, oss.str());
                if (d.nbDims == 3 && d.d[2] > bestC) {
                    bestC = static_cast<int>(d.d[2]);
                    m_outputIndex = i;
                    m_numClasses = static_cast<int>(d.d[2]);
                }
            }
            if (m_outputIndex < 0) {
                AppLogger::get().error("未找到 3 维 CTC 输出张量 (期望 C={})", expectC);
                return false;
            }
            AppLogger::get().warn("回退选择输出 '{}' C={}（期望 {}，请核对字典文件）",
                m_engine->getIOTensorName(m_outputIndex), m_numClasses, expectC);
        }
        return true;
    }

    // ============================================================
    // 预处理 / 推理 / 解码
    // ============================================================
    int PaddleOCRRec::preprocess(const cv::Mat& bgr) {
        // 按高度 48 等比缩放，宽度对齐到 8 的倍数并 clamp 到 profile 区间
        const int h = m_cfg.recHeight;
        int w = static_cast<int>(std::lround(static_cast<double>(bgr.cols) * h / bgr.rows));
        w = std::max(8, w);
        w = (w + 7) / 8 * 8;                       // 对齐 stride
        w = std::min(w, m_maxWidth);
        w = std::max(w, 8);
        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);

        // PaddleOCR RecResizeImg 归一化: (x/255 - 0.5) / 0.5  -> [-1,1]，保持 BGR 顺序
        cv::Mat f;
        resized.convertTo(f, CV_32FC3, 1.0 / 255.0, 0.0);
        f = (f - 0.5f) * 2.0f;

        // HWC -> CHW 展开到预分配的 host 缓冲
        std::vector<cv::Mat> ch(3);
        cv::split(f, ch);
        for (int c = 0; c < 3; ++c) {
            std::memcpy(m_hostInput.data() + static_cast<size_t>(c) * h * w,
                ch[c].ptr<float>(), static_cast<size_t>(h) * w * sizeof(float));
        }

        // 数据已写入 m_hostInput，返回本次使用的宽度
        return w;
    }

    RecResult PaddleOCRRec::run(const cv::Mat& bgr) {
        RecResult result;
        if (!m_ready || !m_context) {
            return result;
        }
        if (bgr.empty() || bgr.rows < 2 || bgr.cols < 2 || bgr.channels() != 3) {
            return result;
        }

        int w = preprocess(bgr);
        if (w <= 0) {
            return result;
        }

        // 设定本次输入形状
        nvinfer1::Dims inD{ 4, {1, 3, m_cfg.recHeight, w} };
        const char* inName = m_engine->getIOTensorName(m_inputIndex);
        if (!m_context->setInputShape(inName, inD)) {
            AppLogger::get().error("setInputShape 失败 (w={})", w);
            return result;
        }

        const size_t inBytes = static_cast<size_t>(3) * m_cfg.recHeight * w * sizeof(float);
        cudaError_t err = cudaMemcpyAsync(m_devInput, m_hostInput.data(), inBytes,
            cudaMemcpyHostToDevice, m_stream);
        if (err != cudaSuccess) {
            AppLogger::get().error("OCR H2D 拷贝失败: {}", cudaGetErrorString(err));
            return result;
        }

        if (!m_context->enqueueV3(m_stream)) {
            AppLogger::get().error("OCR enqueueV3 失败");
            return result;
        }

        // 输出实际形状 [1, T, C]
        const char* outName = m_engine->getIOTensorName(m_outputIndex);
        nvinfer1::Dims outD = m_context->getTensorShape(outName);
        if (outD.nbDims != 3 || outD.d[1] <= 0 || outD.d[2] <= 0) {
            AppLogger::get().error("OCR 输出形状非法");
            return result;
        }
        const int T = static_cast<int>(outD.d[1]);
        const int C = static_cast<int>(outD.d[2]);
        const size_t outBytes = static_cast<size_t>(T) * C * sizeof(float);
        if (outBytes > m_outputBytes) {
            AppLogger::get().error("OCR 输出超出预分配缓冲 ({} > {})", outBytes, m_outputBytes);
            return result;
        }

        err = cudaMemcpyAsync(m_hostOutput.data(), m_devOutput, outBytes,
            cudaMemcpyDeviceToHost, m_stream);
        if (err != cudaSuccess) {
            AppLogger::get().error("OCR D2H 拷贝失败: {}", cudaGetErrorString(err));
            return result;
        }
        if (cudaStreamSynchronize(m_stream) != cudaSuccess) {
            AppLogger::get().error("OCR stream 同步失败");
            return result;
        }

        return decode(m_hostOutput.data(), T, C);
    }

    RecResult PaddleOCRRec::decode(const float* logits, int T, int C) const {
        RecResult result;
        result.confidence = 0.0f;
        if (!logits || T <= 0 || C <= 0) {
            return result;
        }

        int lastIndex = 0;
        int kept = 0;
        double scoreSum = 0.0;
        std::string text;
        text.reserve(16);

        for (int t = 0; t < T; ++t) {
            const float* row = logits + static_cast<size_t>(t) * C;
            // 贪心取最大
            int best = 0;
            float bestScore = row[0];
            for (int c = 1; c < C; ++c) {
                if (row[c] > bestScore) {
                    bestScore = row[c];
                    best = c;
                }
            }
            // CTC：跳过 blank(0) 与连续重复
            if (best == 0 || best == lastIndex) {
                lastIndex = best;
                continue;
            }
            lastIndex = best;

            if (m_cfg.digitOnly) {
                char d = m_charset.toDigit(best);
                if (d == '\0') {
                    continue; // 非数字字符直接丢弃
                }
                text.push_back(d);
                scoreSum += bestScore;
                ++kept;
            }
            else {
                const std::string& s = m_charset.at(best);
                if (!s.empty()) {
                    text += s;
                    scoreSum += bestScore;
                    ++kept;
                }
            }
        }

        result.text = std::move(text);
        result.confidence = kept > 0 ? static_cast<float>(scoreSum / kept) : 0.0f;
        return result;
    }

} // namespace ocr
