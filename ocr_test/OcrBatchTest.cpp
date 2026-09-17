// ocr_test/OcrBatchTest.cpp
//
// 批量真图识别测试。
//
// 用法:
//   OcrBatchTest <imagesDir> <onnx> <engine> <dict> [inference.yml] [labels.txt]
//
// 期望值来源（按优先级）:
//   1. labels.txt：每行 "文件名或相对路径<TAB>期望数字"
//   2. 文件名中最长的连续数字串（例如 "target_42_a.jpg" -> "42"）
//   3. 都没有则只识别不判分（记作 ?）
//
// 输出: 控制台报告 + CSV（便于用 Excel 分析）
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "AppLogger.h"
#include "ocr/PaddleOCRRec.h"

namespace {

    const std::set<std::string> kImageExt = {
        ".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"
    };

    std::string toLower(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }

    std::string extOf(const std::string& p) {
        const size_t d = p.find_last_of('.');
        return d == std::string::npos ? std::string() : toLower(p.substr(d));
    }

    std::string baseName(const std::string& p) {
        const size_t s = p.find_last_of("/\\");
        return s == std::string::npos ? p : p.substr(s + 1);
    }

    void listImages(const std::string& dir, std::vector<std::string>& out) {
        namespace fs = std::filesystem;
        std::error_code ec;
        for (const auto& e : fs::recursive_directory_iterator(dir, ec)) {
            if (!e.is_regular_file(ec)) continue;
            const std::string p = e.path().string();
            if (kImageExt.count(extOf(p))) out.push_back(p);
        }
        std::sort(out.begin(), out.end());
    }

    /// 取文件名中最长的连续数字串作为期望编号
    std::string expectedFromName(const std::string& path) {
        std::string stem = baseName(path);
        const size_t d = stem.find_last_of('.');
        if (d != std::string::npos) stem = stem.substr(0, d);

        std::string best, cur;
        for (char c : stem) {
            if (c >= '0' && c <= '9') {
                cur.push_back(c);
            }
            else {
                if (cur.size() > best.size()) best = cur;
                cur.clear();
            }
        }
        if (cur.size() > best.size()) best = cur;
        return best;
    }

    std::map<std::string, std::string> loadLabels(const std::string& path) {
        std::map<std::string, std::string> m;
        std::ifstream in(path);
        if (!in.is_open()) return m;
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            const size_t t = line.find('\t');
            if (t == std::string::npos) continue;
            std::string key = line.substr(0, t);
            std::string val = line.substr(t + 1);
            while (!val.empty() && (val.back() == '\r' || val.back() == ' ')) val.pop_back();
            m[key] = val;
            m[baseName(key)] = val;   // 同时支持只用文件名匹配
        }
        return m;
    }

    struct Row {
        std::string file;
        std::string expect;
        std::string got;
        float conf = 0.0f;
        double ms = 0.0;
        bool labeled = false;
        bool correct = false;
    };

    /// 自动定位亮区（模拟 YOLO 检测框裁剪）。
    /// 真实 pipeline 里 OCR 只吃检测框内的裁剪图，直接送整图会让数字过小、
    /// 无法反映真实精度，所以这里默认先裁一个 ROI。
    cv::Rect autoRoi(const cv::Mat& bgr) {
        cv::Mat gray, mask;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        const int thr = std::max(140, static_cast<int>(cv::mean(gray)[0] * 1.9));
        cv::threshold(gray, mask, thr, 255, cv::THRESH_BINARY);

        if (cv::countNonZero(mask) < 30) {
            return cv::Rect(0, 0, bgr.cols, bgr.rows);   // 找不到亮区就用整图
        }

        // 取最大连通域
        cv::Mat labels, stats, cents;
        const int n = cv::connectedComponentsWithStats(mask, labels, stats, cents, 8);
        int best = -1, bestArea = 0;
        for (int i = 1; i < n; ++i) {
            const int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > bestArea) { bestArea = area; best = i; }
        }
        if (best < 0) {
            return cv::Rect(0, 0, bgr.cols, bgr.rows);
        }

        cv::Rect r(stats.at<int>(best, cv::CC_STAT_LEFT),
                   stats.at<int>(best, cv::CC_STAT_TOP),
                   stats.at<int>(best, cv::CC_STAT_WIDTH),
                   stats.at<int>(best, cv::CC_STAT_HEIGHT));

        // 略微外扩，避免贴边数字被切
        const int px = std::max(2, static_cast<int>(r.width * 0.06));
        const int py = std::max(2, static_cast<int>(r.height * 0.06));
        r.x -= px; r.y -= py; r.width += 2 * px; r.height += 2 * py;
        return r & cv::Rect(0, 0, bgr.cols, bgr.rows);
    }

} // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::printf("usage: OcrBatchTest <imagesDir> <onnx> <engine> <dict> [inference.yml] [labels.txt] [--full]\n");
        std::printf("  --full : 不裁 ROI，直接把整张图送识别（默认会自动裁亮区模拟 YOLO 检测框）\n");
        return 2;
    }

    bool fullImage = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--full") == 0) fullImage = true;
    }

    const std::string imagesDir = argv[1];
    ocr::PaddleRecConfig cfg;
    cfg.onnxPath = argv[2];
    cfg.enginePath = argv[3];
    cfg.dictPath = argv[4];
    if (argc >= 6) cfg.inferenceYmlPath = argv[5];
    cfg.recMaxWidth = 320;
    cfg.recOptWidth = 320;
    cfg.useFp16 = true;
    cfg.digitOnly = true;

    std::map<std::string, std::string> labels;
    if (argc >= 7) {
        labels = loadLabels(argv[6]);
        std::printf("已加载标注文件: %s (%zu 条)\n", argv[6], labels.size());
    }

    std::vector<std::string> files;
    listImages(imagesDir, files);
    if (files.empty()) {
        std::printf("[ERROR] 目录中没有图片: %s\n", imagesDir.c_str());
        std::printf("        支持的扩展名: .jpg .jpeg .png .bmp .webp .tif .tiff\n");
        return 2;
    }
    std::printf("发现 %zu 张图片，开始识别...\n\n", files.size());

    AppLogger::get().setLevel(spdlog::level::warn);   // 压低日志，只留报告
    ocr::PaddleOCRRec rec(cfg);
    if (!rec.isReady()) {
        std::printf("[ERROR] 识别器初始化失败\n");
        return 1;
    }

    std::vector<Row> rows;
    rows.reserve(files.size());

    for (const auto& f : files) {
        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
        Row r;
        r.file = f;

        if (img.empty()) {
            std::printf("  [SKIP ] %-40s 无法读取\n", baseName(f).c_str());
            r.got = "<read-error>";
            rows.push_back(r);
            continue;
        }

        const auto t0 = std::chrono::steady_clock::now();
        cv::Rect roi(0, 0, img.cols, img.rows);
        cv::Mat feed = img;
        if (!fullImage) {
            roi = autoRoi(img);
            feed = img(roi);
        }
        ocr::RecResult res = rec.run(feed);
        const auto t1 = std::chrono::steady_clock::now();
        r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        r.got = res.text;
        r.conf = res.confidence;

        // 期望值
        auto it = labels.find(f);
        if (it == labels.end()) it = labels.find(baseName(f));
        if (it != labels.end()) {
            r.expect = it->second;
            r.labeled = true;
        }
        else {
            r.expect = expectedFromName(f);
            r.labeled = !r.expect.empty();
        }
        r.correct = r.labeled && (r.got == r.expect);

        const char* mark = !r.labeled ? "  ?   " : (r.correct ? " PASS " : " FAIL ");
        std::printf("[%s] %-30s %4dx%-4d roi=%dx%-4d %6.1fms  exp=%-5s got=%-5s conf=%.3f\n",
            mark, baseName(f).c_str(), img.cols, img.rows, feed.cols, feed.rows, r.ms,
            r.labeled ? r.expect.c_str() : "?", r.got.empty() ? "(空)" : r.got.c_str(),
            r.conf);

        rows.push_back(r);
    }

    // ---- 汇总 ----
    int labeled = 0, correct = 0, unlabeled = 0, readErr = 0;
    double totalMs = 0.0;
    std::map<std::string, int> confusion;   // "exp->got"
    for (const auto& r : rows) {
        totalMs += r.ms;
        if (r.got == "<read-error>") { ++readErr; continue; }
        if (!r.labeled) { ++unlabeled; continue; }
        ++labeled;
        if (r.correct) ++correct;
        else confusion[r.expect + " -> " + (r.got.empty() ? "(空)" : r.got)]++;
    }

    std::printf("\n================ 汇总 ================\n");
    std::printf("图片总数      : %zu\n", rows.size());
    std::printf("可判分样本    : %d\n", labeled);
    std::printf("判分正确      : %d\n", correct);
    std::printf("判分错误      : %d\n", labeled - correct);
    std::printf("未标注(仅识别): %d\n", unlabeled);
    std::printf("读取失败      : %d\n", readErr);
    if (labeled > 0) {
        std::printf("准确率        : %.2f%% (%d/%d)\n",
            100.0 * correct / labeled, correct, labeled);
    }
    if (!rows.empty()) {
        std::printf("平均单张耗时  : %.1f ms\n", totalMs / rows.size());
    }

    if (!confusion.empty()) {
        std::printf("\n---- 错误明细 (期望 -> 实得 : 次数) ----\n");
        for (const auto& kv : confusion) {
            std::printf("  %-24s : %d\n", kv.first.c_str(), kv.second);
        }
    }

    // ---- CSV ----
    std::ofstream csv("ocr_batch_result.csv");
    if (csv.is_open()) {
        csv << "file,expect,got,confidence,ms,labeled,correct\n";
        for (const auto& r : rows) {
            csv << '"' << r.file << "\","
                << '"' << r.expect << "\","
                << '"' << r.got << "\","
                << r.conf << "," << r.ms << ","
                << (r.labeled ? 1 : 0) << "," << (r.correct ? 1 : 0) << "\n";
        }
        csv.close();
        std::printf("\n明细已写入: ocr_batch_result.csv\n");
    }

    return (labeled > 0 && correct == labeled) ? 0 : 1;
}
