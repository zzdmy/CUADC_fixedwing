#pragma once
#include <opencv2/core.hpp>

struct TrackedObject {
    int classId;
    cv::Rect box;
};
