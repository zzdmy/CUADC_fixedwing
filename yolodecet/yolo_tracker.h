// yolo_tracker.h
#ifndef YOLO_TRACKER_H
#define YOLO_TRACKER_H

#include <opencv2/opencv.hpp>
#include "inference.h"
#include "../realtime_pipeline.h"
#include <vector>
#include <string>

cv::Mat extractSafeSubRect(cv::Mat& workingImage, const cv::Rect& box);
void yolo(Inference inf, cv::Mat& frame, std::vector<yoloout>& getout);
bool fileExists(const std::string& filename);
void runInThread(const std::string& modelPath, const std::string& classPath,
    bool runOnGPU, std::atomic<bool>& live);

void fristyolo( const std::string& modelPath, const std::string& classPath, bool runOnGPU);
void runObjectTracking(cv::Mat inframe,  Inference inf, std::vector<yoloout>&output,bool&yolostop);

void drawCompassOnImage(cv::Mat& image, float yaw);


class YoloTracker {
public:
   

private:
  
};



#endif // YOLO_TRACKER_H
