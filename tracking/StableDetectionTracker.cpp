#include "StableDetectionTracker.h"
#include <algorithm>

StableDetectionTracker::StableDetectionTracker(int max_frames, double max_time)
    : max_frames_(max_frames), max_time_(max_time) {
}

void StableDetectionTracker::addDetections(const std::vector<yoloout>& detections, double timestamp) {
    for (const auto& det : detections) {
        history_.push_back({
            det.name,
            det.confidence,
            det.box,
            current_frame_,
            timestamp
            });
    }
    current_frame_++;
    pruneOldDetections(timestamp);
}

std::vector<yoloout> StableDetectionTracker::getStableSortedDetections() {
    if (history_.empty()) return {};

    std::vector<cv::Rect> stable_boxes = getStableBoxes();

    std::sort(stable_boxes.begin(), stable_boxes.end(),
        [](const cv::Rect& a, const cv::Rect& b) {
            float center_a = a.x + a.width / 2.0f;
            float center_b = b.x + b.width / 2.0f;
            return center_a < center_b;
        });

    std::vector<yoloout> result;
    for (int i = 0; i < stable_boxes.size(); ++i) {
        yoloout out;
        out.name = "target";
        out.confidence = 0.9f;
        out.box = stable_boxes[i];
        out.classId = i + 1;
        result.push_back(out);
    }

    return result;
}

void StableDetectionTracker::pruneOldDetections(double current_time) {
    while (!history_.empty()) {
        auto& front = history_.front();
        bool by_time = (current_time - front.timestamp) > max_time_;
        bool by_frame = (current_frame_ - front.frame_id) > max_frames_;
        if (by_time || by_frame) {
            history_.pop_front();
        }
        else {
            break;
        }
    }
}

std::vector<cv::Rect> StableDetectionTracker::getStableBoxes() {
    std::map<std::string, std::vector<cv::Rect>> clusters;

    for (const auto& snap : history_) {
        std::string key = std::to_string(snap.box.x / 10) + "_" +
            std::to_string(snap.box.y / 10) + "_" +
            std::to_string(snap.box.width / 5);
        clusters[key].push_back(snap.box);
    }

    std::vector<cv::Rect> stable_boxes;
    for (auto& [key, boxes] : clusters) {
        if (boxes.size() < (size_t)max_frames_ * 0.5) {
            continue;
        }

        cv::Rect avg_box(0, 0, 0, 0);
        for (const auto& b : boxes) {
            avg_box.x += b.x;
            avg_box.y += b.y;
            avg_box.width += b.width;
            avg_box.height += b.height;
        }
        int n = boxes.size();
        avg_box.x /= n;
        avg_box.y /= n;
        avg_box.width /= n;
        avg_box.height /= n;

        stable_boxes.push_back(avg_box);
    }

    return stable_boxes;
}
