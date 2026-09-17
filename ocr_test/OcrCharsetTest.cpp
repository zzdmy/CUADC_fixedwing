// ocr_test/OcrCharsetTest.cpp
// 独立验证 OcrCharset 对 inference.yml 的解析与数字索引映射。
#include <cstdio>
#include <string>

#include "ocr/OcrCharset.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: OcrCharsetTest <inference.yml|dict.txt>\n");
        return 2;
    }
    std::string path = argv[1];

    ocr::OcrCharset cs;
    bool ok = false;
    if (path.size() > 4 && path.substr(path.size() - 4) == ".yml") {
        ok = cs.loadFromInferenceYml(path);
    }
    else {
        ok = cs.loadDict(path);
    }

    if (!ok) {
        std::printf("FAIL: 无法加载 %s\n", path.c_str());
        return 1;
    }

    std::printf("OK: %s\n", cs.describeFirst(3).c_str());
    std::printf("numClasses(= dict+2) = %d\n", cs.numClasses());
    if (cs.indexRepaired()) {
        std::printf("NOTE: 检测到字典首项全角空格被上游 trim 掉，已自动补回（索引已修正）\n");
    }

    // 打印所有数字的 CTC 索引
    int found = 0;
    for (int i = 0; i < cs.numClasses(); ++i) {
        char d = cs.toDigit(i);
        if (d != '\0') {
            std::printf("  ctc[%d] -> '%c'\n", i, d);
            ++found;
            if (found >= 30) break;
        }
    }
    std::printf("digit entries = %d (期望 20)\n", cs.digitCount());

    // 校验若干已知位置的字符，确认索引对齐（0=blank, 1=字典第0项）
    std::printf("at(0)  = '%s'  (应为空, blank)\n", cs.at(0).c_str());
    std::printf("at(1)  = '%s'  (字典第 1 项)\n", cs.at(1).c_str());

    // 校验：至少要有 10 个半角数字（完整 PP-OCRv5 字典还有 10 个全角数字，共 20）
    bool pass = (cs.digitCount() >= 10) && (cs.toDigit(0) == '\0');
    // 反查：'0'..'9' 必须都能被某个索引映射到
    for (char c = '0'; c <= '9'; ++c) {
        bool hit = false;
        for (int i = 0; i < cs.numClasses() && !hit; ++i) {
            if (cs.toDigit(i) == c) hit = true;
        }
        if (!hit) {
            std::printf("FAIL: 数字 '%c' 没有对应索引\n", c);
            pass = false;
        }
    }

    std::printf("%s\n", pass ? "== PASS ==" : "== FAIL ==");
    return pass ? 0 : 1;
}
