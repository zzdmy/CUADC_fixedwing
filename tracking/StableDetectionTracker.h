#pragma once
#include <deque>
#include <vector>
#include <map>
#include <string>
#include <opencv2/core.hpp>
#include "../yolodecet/yolo_tracker.h"

struct DetectionSnapshot {
    std::string name;
    float confidence;
    cv::Rect box;
    int frame_id;
    double timestamp;
};

class StableDetectionTracker {
public:
    explicit StableDetectionTracker(int max_frames = 10, double max_time = 2.0);

    void addDetections(const std::vector<yoloout>& detections, double timestamp = 0.0);
    std::vector<yoloout> getStableSortedDetections();

private:
    std::deque<DetectionSnapshot> history_;
    int max_frames_;
    double max_time_;
    int current_frame_ = 0;

    void pruneOldDetections(double current_time);
    std::vector<cv::Rect> getStableBoxes();
};
