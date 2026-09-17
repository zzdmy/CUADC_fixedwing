// ocr/OcrDigitReader.h
//
// 编号数字识别调度器。
//
// 职责：
//   1. 按需触发 —— 只在任务进入「侦察 / 投弹」状态时才真正调用识别模型；
//   2. 对 YOLO 检出的候选框裁剪后进行 PP-OCRv5 识别；
//   3. 多帧投票，输出稳定数字，供 MissionScheduler 做目标判据；
//   4. 结果通过 lookup() 以「框 IoU 匹配」方式查询，供任务调度按检测框取编号。
//
// 说明：识别结果不进画面，只看数据（按需求）。
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "ocr/PaddleOCRRec.h"

namespace ocr {

    /// @brief 一条 OCR 识别结果（按检测框缓存，可跨帧复用若干毫秒）。
    struct OcrEntry {
        cv::Rect box;
        std::string digits;             // 稳定后的编号（只含数字，可能为空）
        float confidence = 0.0f;        // 对应置信度
        int   rawVotes = 0;             // 达成该结论的原始票数
        int   framesSeen = 0;           // 累计观察帧数
        int64_t stampMs = 0;            // 时间戳(ms)，用于过期判断
        bool  stable() const { return !digits.empty(); }
    };

    struct OcrConfig {
        bool enabled = true;
        bool startEnabled = false;          // 初始是否允许识别（由任务状态驱动）
        std::vector<std::string> triggers;  // 触发场景：recon / bomb（空=全部允许）

        float minBoxWidth = 16.0f;          // 过滤过小的框（像素）
        float minBoxHeight = 8.0f;
        int   maxBoxesPerCycle = 24;        // 每轮最多识别多少个候选框
        int   minInferIntervalMs = 80;      // 两次识别之间的最小间隔（限流）
        float cropPadding = 0.08f;          // 裁剪时向外扩的比例
        int   minTextHeight = 8;            // 送入识别前的上采样目标高度下限

        int   voteWindow = 5;               // 滑动投票窗口大小
        int   minVotes = 2;                 // 至少多少次一致才认为稳定
        int64_t entryTtlMs = 3000;          // 结果保留时间
    };

    class OcrDigitReader {
    public:
        OcrDigitReader(OcrConfig cfg, PaddleRecConfig recCfg);
        ~OcrDigitReader();

        OcrDigitReader(const OcrDigitReader&) = delete;
        OcrDigitReader& operator=(const OcrDigitReader&) = delete;

        /// @brief 引擎是否可用（false 时 start() 不会起线程）。
        bool isReady() const { return m_rec && m_rec->isReady(); }

        /// @brief 启动后台识别线程。
        void start();

        /// @brief 停止后台识别线程并等待退出。
        void stop();

        /// @brief 是否允许识别（由 MissionScheduler 按任务状态设置）。
        void setEnabled(bool v) { m_enabled.store(v); }
        bool isEnabled() const { return m_enabled.load(); }

        /// @brief 配置的触发场景列表（空表示全部允许）。
        const std::vector<std::string>& triggers() const { return m_cfg.triggers; }

        /// @brief 查询某个检测框对应的编号。
        /// @param[out] entry 命中的结果（可空）
        /// @return 找到有效结果返回 true
        bool lookup(const cv::Rect& detBox, OcrEntry& entry) const;

        /// @brief 取当前所有未过期的识别结果（快照）。
        std::vector<OcrEntry> snapshot() const;

        /// @brief 统计信息，供状态日志使用。
        struct Stats {
            uint64_t cycles = 0;        // 识别轮次
            uint64_t inferences = 0;    // 单张推理次数
            uint64_t hits = 0;          // 成功解出非空编号的次数
            double lastCostMs = 0.0;    // 最近一轮耗时
        };
        Stats getStats() const;

        bool isRunning() const { return m_running.load(); }

    private:
        void worker(std::stop_token st);
        void pruneLocked(int64_t nowMs) const;

        OcrConfig m_cfg;
        PaddleRecConfig m_recCfg;
        std::unique_ptr<PaddleOCRRec> m_rec;

        std::jthread m_thread;
        std::atomic<bool> m_running{ false };
        std::atomic<bool> m_enabled{ false };

        mutable std::mutex m_mutex;
        mutable std::unordered_map<std::string, OcrEntry> m_entries;
        mutable std::unordered_map<std::string, std::vector<std::string>> m_votes;

        std::atomic<uint64_t> m_cycles{ 0 };
        std::atomic<uint64_t> m_inferences{ 0 };
        std::atomic<uint64_t> m_hits{ 0 };
        std::atomic<double> m_lastCostMs{ 0.0 };

        static std::string boxKey(const cv::Rect& r);
    };

    /// @brief 判断当前任务场景是否在 OCR 触发列表中。
    /// @param triggers 配置里的触发场景名（空表示全部允许）
    /// @param currentPhase 当前场景名（"init"/"takeoff"/"bomb"/"recon"）
    bool isPhaseTriggered(const std::vector<std::string>& triggers, const std::string& currentPhase);

    // ===== 全局访问：供 MissionScheduler 等其它线程按框查数字 =====
    /// @brief 注册全局 OCR 读取器（main 线程创建并 start 后调用一次）。
    void setGlobalReader(OcrDigitReader* r);
    /// @brief 取全局 OCR 读取器（未注册返回 nullptr）。
    OcrDigitReader* globalReader();
    /// @brief 便捷查询：按检测框查稳定数字字符串（未命中返回空串）。
    std::string lookupDigit(const cv::Rect& box);

} // namespace ocr
