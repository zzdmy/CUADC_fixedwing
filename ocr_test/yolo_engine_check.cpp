// yolo_engine_check.cpp — 验证重建后的检测 engine 能否被 yolov8_trt_infer.cpp 的 loadEngine 正常加载。
// 关键点：代码里对输入调用了 setInputShape("images", 1x3x640x640)；静态 engine 上这步可能返回 false。
#include <NvInfer.h>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

class L : public nvinfer1::ILogger {
public:
    void log(Severity s, const char* m) noexcept override {
        if (s <= Severity::kWARNING) std::fprintf(stderr, "[TRT] %s\n", m);
    }
};

int main(int argc, char** argv) {
    if (argc < 2) { std::printf("usage: yolo_engine_check <engine>\n"); return 2; }
    std::ifstream f(argv[1], std::ios::binary | std::ios::ate);
    if (!f.is_open()) { std::printf("FAIL: open %s\n", argv[1]); return 1; }
    auto size = f.tellg(); f.seekg(0);
    std::vector<char> data(size);
    f.read(data.data(), size);
    f.close();

    L logger;
    nvinfer1::IRuntime* rt = nvinfer1::createInferRuntime(logger);
    nvinfer1::ICudaEngine* eng = rt->deserializeCudaEngine(data.data(), size);
    if (!eng) { std::printf("FAIL: deserialize\n"); return 1; }
    auto ctx = eng->createExecutionContext();
    if (!ctx) { std::printf("FAIL: createExecutionContext\n"); return 1; }

    // 与 yolov8_trt_infer.cpp:122-126 完全一致的调用
    nvinfer1::Dims in{ 4, {1, 3, 640, 640} };
    bool ok = ctx->setInputShape("images", in);
    std::printf("setInputShape('images', 1x3x640x640) = %s\n", ok ? "TRUE" : "FALSE");

    nvinfer1::Dims out = ctx->getTensorShape("output0");
    std::printf("output0 dims = [");
    for (int i = 0; i < out.nbDims; ++i) std::printf("%s%d", i ? "," : "", out.d[i]);
    std::printf("]\n");

    bool dynamic = false;
    for (int i = 0; i < eng->getNbIOTensors(); ++i) {
        const char* nm = eng->getIOTensorName(i);
        if (nm && std::string(nm) == "images") {
            nvinfer1::Dims d = eng->getTensorShape(nm);
            for (int k = 0; k < d.nbDims; ++k) if (d.d[k] < 0) dynamic = true;
        }
    }
    std::printf("engine has dynamic input (optimization profile) = %s\n", dynamic ? "YES" : "NO");

    std::printf("%s\n", ok ? "== CHECK PASS ==" : "== CHECK FAIL: setInputShape failed ==");
    return ok ? 0 : 1;
}
