// ocr/OcrDigitReader.cpp
#include "ocr/OcrDigitReader.h"

#include <algorithm>
#include <chrono>

#include "AppLogger.h"
#include "FrameDispatcher.h"
#include "realtime_pipeline.h"

namespace ocr {

    namespace {
        int64_t nowMs() {
            using namespace std::chrono;
            return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
        }

        /// @brief 对齐到 8 像素网格的字符串 key，保证相邻帧同一目标落在同一桶。
        std::string makeBoxKey(const cv::Rect& r) {
            const int gx = (r.x + 4) / 8;
            const int gy = (r.y + 4) / 8;
            const int gw = std::max(1, (r.width + 4) / 8);
            const int gh = std::max(1, (r.height + 4) / 8);
            return std::to_string(gx) + "_" + std::to_string(gy) + "_" +
                   std::to_string(gw) + "_" + std::to_string(gh);
        }

        float iou(const cv::Rect& a, const cv::Rect& b) {
            const int x1 = std::max(a.x, b.x);
            const int y1 = std::max(a.y, b.y);
            const int x2 = std::min(a.x + a.width, b.x + b.width);
            const int y2 = std::min(a.y + a.height, b.y + b.height);
            const int iw = x2 - x1;
            const int ih = y2 - y1;
            if (iw <= 0 || ih <= 0) {
                return 0.0f;
            }
            const float inter = static_cast<float>(iw) * ih;
            const float uni = static_cast<float>(a.area()) + static_cast<float>(b.area()) - inter;
            return uni > 0.0f ? inter / uni : 0.0f;
        }
    } // namespace

    bool isPhaseTriggered(const std::vector<std::string>& triggers, const std::string& currentPhase) {
        if (triggers.empty()) {
            return true; // 未配置则视为全部允许
        }
        for (const auto& t : triggers) {
            if (t == currentPhase) {
                return true;
            }
        }
        return false;
    }

    std::string OcrDigitReader::boxKey(const cv::Rect& r) {
        return makeBoxKey(r);
    }

    OcrDigitReader::OcrDigitReader(OcrConfig cfg, OcrPipelineConfig pipelineCfg)
        : m_cfg(std::move(cfg)), m_pipelineCfg(std::move(pipelineCfg)) {
        m_pipeline = std::make_unique<OcrPipeline>(m_pipelineCfg);
        m_enabled.store(m_cfg.startEnabled);
        if (!m_pipeline->isReady()) {
            AppLogger::get().error("OCR 识别器初始化失败，编号识别将不可用");
        }
    }

    OcrDigitReader::~OcrDigitReader() {
        stop();
    }

    void OcrDigitReader::start() {
        if (m_running.load()) {
            return;
        }
        if (!m_pipeline || !m_pipeline->isReady()) {
            AppLogger::get().error("OCR 引擎未就绪，跳过启动");
            return;
        }
        m_running.store(true);
        m_thread = std::jthread([this](std::stop_token st) {
            worker(std::move(st));
            });

        std::string triggers;
        for (const auto& t : m_cfg.triggers) {
            if (!triggers.empty()) triggers += ",";
            triggers += t;
        }
        AppLogger::get().info("OCR 识别线程已启动 (触发场景: {})",
            triggers.empty() ? std::string("全部") : triggers);
    }

    void OcrDigitReader::stop() {
        m_running.store(false);
        // std::jthread 析构时自动 request_stop + join
    }

    void OcrDigitReader::pruneLocked(int64_t now) const {
        for (auto it = m_entries.begin(); it != m_entries.end();) {
            if (now - it->second.stampMs > m_cfg.entryTtlMs) {
                m_votes.erase(it->first);   // mutable，允许在 const 查询里清理
                it = m_entries.erase(it);
            }
            else {
                ++it;
            }
        }
    }

    void OcrDigitReader::worker(std::stop_token st) {
        int64_t lastInferMs = 0;

        while (!st.stop_requested()) {
            // ---- 按需触发：未启用时几乎不占 CPU ----
            if (!m_enabled.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            const int64_t t0 = nowMs();
            if (t0 - lastInferMs < m_cfg.minInferIntervalMs) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            auto framePtr = frameDispatcher.getFrame();
            if (!framePtr || framePtr->empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            auto detPtr = get_latest_yolo();
            if (!detPtr || detPtr->empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            const cv::Mat& frame = *framePtr;
            int processed = 0;
            uint64_t cycleHits = 0;

            for (const auto& det : *detPtr) {
                if (st.stop_requested() || !m_enabled.load()) {
                    break;
                }
                if (processed >= m_cfg.maxBoxesPerCycle) {
                    break;
                }

                // 类别过滤：只对配置的类别做 OCR（空=全部；任务二只取 bucket 类）
                if (!m_cfg.keepClassIds.empty()) {
                    bool keep = false;
                    for (int cid : m_cfg.keepClassIds) {
                        if (det.classId == cid) { keep = true; break; }
                    }
                    if (!keep) continue;
                }

                cv::Rect box = det.box & cv::Rect(0, 0, frame.cols, frame.rows);
                if (box.area() <= 0) {
                    continue;
                }
                if (box.width < m_cfg.minBoxWidth || box.height < m_cfg.minBoxHeight) {
                    continue;
                }

                // ---- 向外扩一点，避免把贴边的数字切掉 ----
                const int padX = static_cast<int>(box.width * m_cfg.cropPadding);
                const int padY = static_cast<int>(box.height * m_cfg.cropPadding);
                cv::Rect roi(box.x - padX, box.y - padY,
                    box.width + 2 * padX, box.height + 2 * padY);
                roi &= cv::Rect(0, 0, frame.cols, frame.rows);
                if (roi.area() <= 0) {
                    continue;
                }

                cv::Mat crop = frame(roi);
                if (crop.empty()) {
                    continue;
                }

                // ---- 识别 ----
                const int64_t inferStart = nowMs();
                RecResult res = m_pipeline->run(crop);
                lastInferMs = nowMs();
                m_inferences.fetch_add(1);
                ++processed;

                const double costMs = static_cast<double>(lastInferMs - inferStart);
                if (!res.empty()) {
                    cycleHits++;
                    m_hits.fetch_add(1);
                }

                // ---- 累计投票 ----
                const std::string key = makeBoxKey(roi);
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    pruneLocked(lastInferMs);

                    auto& votes = m_votes[key];
                    votes.push_back(res.text);
                    if (static_cast<int>(votes.size()) > m_cfg.voteWindow) {
                        votes.erase(votes.begin(),
                            votes.begin() + (votes.size() - m_cfg.voteWindow));
                    }

                    // 统计众数
                    std::string bestDigits;
                    int bestCount = 0;
                    for (size_t i = 0; i < votes.size(); ++i) {
                        if (votes[i].empty()) {
                            continue;
                        }
                        int cnt = 0;
                        for (const auto& v : votes) {
                            if (v == votes[i]) ++cnt;
                        }
                        if (cnt > bestCount) {
                            bestCount = cnt;
                            bestDigits = votes[i];
                        }
                    }

                    auto& entry = m_entries[key];
                    entry.box = roi;
                    entry.stampMs = lastInferMs;
                    entry.framesSeen += 1;
                    entry.rawVotes = bestCount;

                    if (!bestDigits.empty() && bestCount >= m_cfg.minVotes) {
                        if (entry.digits != bestDigits) {
                            AppLogger::get().info("[OCR] 确认编号 \"{}\" (票数 {}/{}, conf={:.2f}, roi {},{},{},{})",
                                bestDigits, bestCount, static_cast<int>(votes.size()),
                                res.confidence, roi.x, roi.y, roi.width, roi.height);
                        }
                        entry.digits = bestDigits;
                        entry.confidence = res.confidence;
                    }
                    else if (m_cfg.minVotes <= 1 && !res.text.empty()) {
                        // 投票门槛为 1 时，单帧即生效
                        entry.digits = res.text;
                        entry.confidence = res.confidence;
                    }
                }

                if (costMs > 200.0) {
                    AppLogger::get().debug("OCR 单张推理耗时 {:.1f}ms (roi {}x{})",
                        costMs, roi.width, roi.height);
                }
            }

            m_cycles.fetch_add(1);
            m_lastCostMs.store(static_cast<double>(nowMs() - t0));
            (void)cycleHits;

            // 识别完一轮主动让出，避免与检测线程抢 GPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        AppLogger::get().info("OCR 识别线程已退出");
    }

    bool OcrDigitReader::lookup(const cv::Rect& detBox, OcrEntry& entry) const {
        const int64_t now = nowMs();
        OcrEntry best;
        float bestIou = 0.0f;
        bool found = false;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            pruneLocked(now);
            for (const auto& kv : m_entries) {
                const OcrEntry& e = kv.second;
                if (e.digits.empty()) {
                    continue;
                }
                const float v = iou(detBox, e.box);
                if (v > bestIou) {
                    bestIou = v;
                    best = e;
                    found = true;
                }
            }
        }
        if (!found || bestIou < 0.3f) {
            return false;
        }
        entry = best;
        return true;
    }

    std::vector<OcrEntry> OcrDigitReader::snapshot() const {
        const int64_t now = nowMs();
        std::vector<OcrEntry> out;
        std::lock_guard<std::mutex> lock(m_mutex);
        pruneLocked(now);
        out.reserve(m_entries.size());
        for (const auto& kv : m_entries) {
            out.push_back(kv.second);
        }
        std::sort(out.begin(), out.end(), [](const OcrEntry& a, const OcrEntry& b) {
            return a.stampMs > b.stampMs;
            });
        return out;
    }

    OcrDigitReader::Stats OcrDigitReader::getStats() const {
        Stats s;
        s.cycles = m_cycles.load();
        s.inferences = m_inferences.load();
        s.hits = m_hits.load();
        s.lastCostMs = m_lastCostMs.load();
        return s;
    }

    // ===== 全局 OCR 读取器（供 MissionScheduler 等其它线程按框查数字） =====
    static std::atomic<OcrDigitReader*> g_global_reader{ nullptr };

    void setGlobalReader(OcrDigitReader* r) {
        g_global_reader.store(r);
    }

    OcrDigitReader* globalReader() {
        return g_global_reader.load();
    }

    std::string lookupDigit(const cv::Rect& box) {
        OcrDigitReader* r = g_global_reader.load();
        if (!r) {
            return {};
        }
        OcrEntry entry;
        if (r->lookup(box, entry)) {
            return entry.digits;
        }
        return {};
    }

} // namespace ocr
