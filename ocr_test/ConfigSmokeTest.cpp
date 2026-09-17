// ocr_test/ConfigSmokeTest.cpp
// 验证 config.yaml 的 ocr 段能被 config_loader 正确解析（不需要硬件/相机）。
#include <cstdio>
#include <string>

#include "config_loader.h"

int main(int argc, char** argv) {
    std::string path = (argc > 1) ? argv[1] : "config.yaml";
    AppConfig cfg = load_config_from_yaml(path);

    const auto& o = cfg.ocr;
    std::printf("--- ocr 段解析结果 ---\n");
    std::printf("enabled              = %d\n", (int)o.enabled);
    std::printf("enable_on_recon      = %d\n", (int)o.enable_on_recon);
    std::printf("enable_on_bomb       = %d\n", (int)o.enable_on_bomb);
    std::printf("onnx_path            = %s\n", o.onnx_path.c_str());
    std::printf("engine_path          = %s\n", o.engine_path.c_str());
    std::printf("dict_path            = %s\n", o.dict_path.c_str());
    std::printf("inference_yml_path   = %s\n", o.inference_yml_path.c_str());
    std::printf("digit_only           = %d\n", (int)o.digit_only);
    std::printf("use_fp16             = %d\n", (int)o.use_fp16);
    std::printf("rec_max_width        = %d\n", o.rec_max_width);
    std::printf("min_box_width        = %.2f\n", o.min_box_width);
    std::printf("min_box_height       = %.2f\n", o.min_box_height);
    std::printf("max_boxes_per_cycle  = %d\n", o.max_boxes_per_cycle);
    std::printf("min_infer_interval_ms= %d\n", o.min_infer_interval_ms);
    std::printf("crop_padding         = %.3f\n", o.crop_padding);
    std::printf("vote_window          = %d\n", o.vote_window);
    std::printf("min_votes            = %d\n", o.min_votes);
    std::printf("entry_ttl_ms         = %d\n", o.entry_ttl_ms);

    // 校验：这些值必须来自 config.yaml，而不是结构体默认值
    bool pass = true;
    if (o.rec_max_width != 320) { std::printf("FAIL: rec_max_width 未从 yaml 读取\n"); pass = false; }
    if (o.vote_window != 5) { std::printf("FAIL: vote_window 未从 yaml 读取\n"); pass = false; }
    if (o.min_box_width != 16.0f) { std::printf("FAIL: min_box_width 未从 yaml 读取\n"); pass = false; }
    if (o.entry_ttl_ms != 3000) { std::printf("FAIL: entry_ttl_ms 未从 yaml 读取\n"); pass = false; }
    if (o.onnx_path != "PP-OCRv5_mobile_rec.onnx") { std::printf("FAIL: onnx_path 未从 yaml 读取\n"); pass = false; }
    if (!o.enabled) { std::printf("FAIL: enabled 应为 true\n"); pass = false; }

    std::printf("%s\n", pass ? "== PASS ==" : "== FAIL ==");
    return pass ? 0 : 1;
}
