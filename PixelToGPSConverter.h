// PixelToGPSConverter.h

#ifndef PIXEL_TO_GPS_CONVERTER_H
#define PIXEL_TO_GPS_CONVERTER_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#define GEOGRAPHICLIB_SHARED_LIB 1
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>

struct TargetGPS {
    int classId;
    cv::Rect box;
    double lat;
    double lon;
    float confidence;

    //机体坐标系：X前, Y右, Z下（单位：米）
    double body_x;
    double body_y;
    double body_z;

    TargetGPS() : classId(-1), confidence(0.0f), body_x(0), body_y(0), body_z(0) {}
};

class PixelToGPSConverter {
public:
    PixelToGPSConverter();

    // Read camera parameters from ConfigManager (统一入口，避免各处重复构造矩阵)
    void configureFromConfig();

    void setCameraIntrinsics(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    void setCameraExtrinsics(
        double roll_rad = 0.0,
        double pitch_rad = 0.0,            // 相机直接朝下安装，无需俯仰旋转
        double yaw_rad = 0.0,
        double tx = 0.0,
        double ty = 0.0,
        double tz = 0.4  // 相机在飞控下方 0.4 米
    );

    std::vector<TargetGPS> convertDetectionsToGPS(
        const std::vector<cv::Rect>& boxes,
        const std::vector<int>& classIds,
        double drone_lat,
        double drone_lon,
        float drone_alt,       // AGL height in meters
        double drone_roll_rad, // FC roll (rad)
        double drone_pitch_rad,// FC pitch (rad)
        double drone_yaw_rad   // FC yaw (rad)
    ) const;

    // 像素→机体FRD坐标（不转GPS，用于Precision Loiter）
    bool pixelToBodyFrame(float pixel_x, float pixel_y,
                          float drone_alt_m,
                          double drone_roll_rad, double drone_pitch_rad, double drone_yaw_rad,
                          double& body_x, double& body_y, double& body_z) const;

    bool isReady() const;

private:
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Mat R_;
    cv::Mat T_;
    bool hasIntrinsics_;
    bool hasExtrinsics_;

    mutable GeographicLib::LocalCartesian localCartesian_;

    cv::Mat eulerAnglesToRotationMatrix(double roll, double pitch, double yaw) const;

    //内部函数：同时计算 GPS 和 机体坐标
    struct PixelToGroundResult {
        cv::Point2d gps;
        double body_x, body_y, body_z;
    };

    PixelToGroundResult pixelToGroundWithBody(
        const cv::Point2d& pixel,
        double drone_lat,
        double drone_lon,
        float drone_alt,
        double drone_roll_rad,
        double drone_pitch_rad,
        double drone_yaw_rad
    ) const;
};

#endif // PIXEL_TO_GPS_CONVERTER_H