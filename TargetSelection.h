// TargetSelection.h
// 投弹目标选择底层接口：按 CUADC「固定翼侦察与打击」规则从候选目标中选出打击目标。
//
// 设计意图：选择逻辑只依赖「每个候选目标带一个可排序的 value」，与具体模型解耦。
// 模型尚未就绪时，上层在构建 TargetCandidate 时通过 class_id -> value 注入
// （见 MissionScheduler.cpp 里的 resolveTargetValue 接线点），本模块只负责「按规则挑一个」。
//
// 规则：
//   任务一（第一轮）：3 个图片靶标中打「目标价值最高」的一个。
//     价值表：机枪兵1 火箭兵2 多旋翼3 固定翼4 卡车5 防空炮6 坦克7
//             直升机8 战斗机9 运输机10 侦察机11 轰炸机12
//   任务二（第二轮）：3 个数字靶标（两位数 0~99）中打「中位数」的一个。

#pragma once
#include <algorithm>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace strike {

// 任务类型
enum class StrikeTask {
    HighestValue = 1,   // 任务一：价值最高
    Median = 2          // 任务二：中位数
};

// 一个候选打击目标
struct TargetCandidate {
    double lat = 0.0;        // 目标 GPS（来自聚类）
    double lon = 0.0;
    int class_id = -1;       // 模型输出的类别 id（0 基，未知 = -1）
    std::string name;        // 模型输出的类别名（备用，当前未用）
    double value = 0.0;      // 参与排序的值：任务一=价值，任务二=数字
};

// 任务一 图片靶标价值表：class_id(0 基) -> 价值
// 注意：class_id 是模型 classes 文件的 0 基下标；默认假设文件按规则顺序排列，
// 若真实模型类别顺序不同，只需改这张表。
inline const std::map<int, double>& pictureValueTable() {
    static const std::map<int, double> table = {
        {0,  1.0},   // 机枪兵
        {1,  2.0},   // 火箭兵
        {2,  3.0},   // 多旋翼
        {3,  4.0},   // 固定翼
        {4,  5.0},   // 卡车
        {5,  6.0},   // 防空炮
        {6,  7.0},   // 坦克
        {7,  8.0},   // 直升机
        {8,  9.0},   // 战斗机
        {9,  10.0},  // 运输机
        {10, 11.0},  // 侦察机
        {11, 12.0},  // 轰炸机
    };
    return table;
}

// 目标选择器：按规则从候选里挑一个
class TargetSelector {
public:
    explicit TargetSelector(StrikeTask task = StrikeTask::HighestValue) : task_(task) {}

    void setTask(StrikeTask task) { task_ = task; }
    StrikeTask task() const { return task_; }

    // 返回选中的候选下标；候选为空返回 -1。
    // 任务一：value 最大；任务二：value 中位数（排序后取中间，默认 3 个靶标）。
    int select(const std::vector<TargetCandidate>& candidates) const {
        if (candidates.empty()) return -1;
        if (candidates.size() == 1) return 0;

        if (task_ == StrikeTask::HighestValue) {
            int best = 0;
            for (std::size_t i = 1; i < candidates.size(); ++i) {
                if (candidates[i].value > candidates[best].value) best = static_cast<int>(i);
            }
            return best;
        }

        // Median：按 value 排序取中间下标（3 个靶标时取 index 1）
        std::vector<std::size_t> idx(candidates.size());
        for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
        std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
            return candidates[a].value < candidates[b].value;
        });
        return static_cast<int>(idx[idx.size() / 2]);
    }

private:
    StrikeTask task_;
};

} // namespace strike
