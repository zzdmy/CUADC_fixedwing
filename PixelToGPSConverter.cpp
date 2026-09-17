// PixelToGPSConverter.cpp

#include "PixelToGPSConverter.h"
#include "ConfigManager.h"
#include <cmath>
#include <opencv2/calib3d.hpp>
#include "AppLogger.h"

PixelToGPSConverter::PixelToGPSConverter()
    : hasIntrinsics_(false), hasExtrinsics_(false) {}

void PixelToGPSConverter::configureFromConfig() {
    const auto& cam = ConfigManager::getInstance().getConfig().camera;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        cam.fx, 0.0, cam.cx,
        0.0, cam.fy, cam.cy,
        0.0, 0.0, 1.0);
    cv::Mat distCoeffs(cam.dist_coeffs.size(), 1, CV_64F);
    for (size_t i = 0; i < cam.dist_coeffs.size(); ++i)
        distCoeffs.at<double>(i) = cam.dist_coeffs[i];
    setCameraIntrinsics(cameraMatrix, distCoeffs);

    setCameraExtrinsics(
        cam.mount_roll  * CV_PI / 180.0,
        cam.mount_pitch * CV_PI / 180.0,
        cam.mount_yaw   * CV_PI / 180.0,
        cam.tx, cam.ty, cam.tz);
}

void PixelToGPSConverter::setCameraIntrinsics(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    cameraMatrix_ = cameraMatrix.clone();
    distCoeffs_ = distCoeffs.clone();
    hasIntrinsics_ = true;
}

cv::Mat PixelToGPSConverter::eulerAnglesToRotationMatrix(double roll, double pitch, double yaw) const {
    // Camera-to-body fixed rotation (team-tested version)
    // Camera frame: X right, Y down, Z forward
    // Body frame:    X forward, Y right, Z down
    cv::Mat R_cam2body = (cv::Mat_<double>(3, 3) <<
        0, -1, 0,  // Camera X → Body Y
        1,  0, 0,  // Camera Y → Body -X
        0,  0, 1   // Camera Z → Body Z
        );

    // Body-to-world rotation: Ry(pitch) * Rx(roll) (team-tested order)
    // (yaw is applied separately in the ENU conversion step)
    cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, std::cos(roll), -std::sin(roll),
        0, std::sin(roll), std::cos(roll));
    cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
        std::cos(pitch), 0, std::sin(pitch),
        0, 1, 0,
        -std::sin(pitch), 0, std::cos(pitch));

    return Ry * Rx * R_cam2body;
}

void PixelToGPSConverter::setCameraExtrinsics(
    double roll_rad,
    double pitch_rad,
    double yaw_rad,
    double tx,
    double ty,
    double tz
) {
    R_ = eulerAnglesToRotationMatrix(roll_rad, pitch_rad, yaw_rad);
    T_ = (cv::Mat_<double>(3, 1) << tx, ty, tz);
    hasExtrinsics_ = true;
}

bool PixelToGPSConverter::isReady() const {
    return hasIntrinsics_ && hasExtrinsics_ &&
        !cameraMatrix_.empty() && !distCoeffs_.empty() &&
        !R_.empty() && !T_.empty();
}

PixelToGPSConverter::PixelToGroundResult PixelToGPSConverter::pixelToGroundWithBody(
    const cv::Point2d& pixel,
    double drone_lat,
    double drone_lon,
    float drone_alt,
    double drone_roll_rad,
    double drone_pitch_rad,
    double drone_yaw_rad
) const {
    PixelToGroundResult result;
    result.gps = cv::Point2d(0, 0);
    result.body_x = result.body_y = result.body_z = 0.0;

    if (!isReady()) {
        AppLogger::get().error("Error: Camera intrinsics or extrinsics not set!");
        return result;
    }

    // Step 1: Undistort pixel (team-tested approach: keep pixel coords)
    cv::Point2f corrected;
    {
        std::vector<cv::Point2f> in_pts = { cv::Point2f(static_cast<float>(pixel.x), static_cast<float>(pixel.y)) };
        std::vector<cv::Point2f> out_pts;
        cv::Mat cam_64f, dist_64f;
        cameraMatrix_.convertTo(cam_64f, CV_64F);
        if (!distCoeffs_.empty()) {
            distCoeffs_.convertTo(dist_64f, CV_64F);
        }
        cv::undistortPoints(in_pts, out_pts, cam_64f, dist_64f, cv::noArray(), cam_64f);
        corrected = out_pts.empty() ? in_pts[0] : out_pts[0];
    }

    // Step 2: Normalize to camera coordinates (team-tested approach)
    cv::Mat cam_64f;
    cameraMatrix_.convertTo(cam_64f, CV_64F);
    double fx = cam_64f.at<double>(0, 0);
    double fy = cam_64f.at<double>(1, 1);
    double cx = cam_64f.at<double>(0, 2);
    double cy = cam_64f.at<double>(1, 2);
    double x_norm = (corrected.x - cx) / fx;
    double y_norm = (corrected.y - cy) / fy;
    cv::Mat ray_cam = (cv::Mat_<double>(3, 1) << x_norm, y_norm, 1.0);

    // Step 3: Camera → Body rotation (mount calibration + FC attitude)
    // Compose FC roll/pitch on top of fixed mount calibration (team-tested approach)
    cv::Mat R_fc_roll = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, std::cos(drone_roll_rad), -std::sin(drone_roll_rad),
        0, std::sin(drone_roll_rad), std::cos(drone_roll_rad));
    cv::Mat R_fc_pitch = (cv::Mat_<double>(3, 3) <<
        std::cos(drone_pitch_rad), 0, std::sin(drone_pitch_rad),
        0, 1, 0,
        -std::sin(drone_pitch_rad), 0, std::cos(drone_pitch_rad));
    cv::Mat point_body = R_fc_pitch * R_fc_roll * R_ * ray_cam;
    double dir_x = point_body.at<double>(0);
    double dir_y = point_body.at<double>(1);
    double dir_z = point_body.at<double>(2);

    if (dir_z <= 0.0) {
        AppLogger::get().error("Error: Ray parallel to ground or pointing upward!");
        return result;
    }

    // Step 4: Ray-ground intersection (ground at Z=0, relative to drone AGL)
    // Rotate camera offset T_ by FC attitude to get true vertical offset
    cv::Mat T_world = R_fc_pitch * R_fc_roll * T_;
    double t = (drone_alt - T_world.at<double>(2)) / dir_z;

    //保存机体坐标
    result.body_x = t * dir_x;
    result.body_y = t * dir_y;
    result.body_z = t * dir_z;  // 应接近 drone_alt - tz

    // Step 5: 机体 → 世界坐标系（ENU：东、北）
    double world_east = result.body_x * sin(drone_yaw_rad) + result.body_y * cos(drone_yaw_rad);
    double world_north = result.body_x * cos(drone_yaw_rad) - result.body_y * sin(drone_yaw_rad);

    // Step 6: 使用 GeographicLib 转换为 GPS
    try {
        localCartesian_ = GeographicLib::LocalCartesian(drone_lat, drone_lon, drone_alt);
        double target_lat, target_lon, target_alt;
        localCartesian_.Reverse(world_east, world_north, 0.0, target_lat, target_lon, target_alt);
        result.gps = cv::Point2d(target_lon, target_lat);
    }
    catch (const std::exception& e) {
        AppLogger::get().error("GeographicLib Error: {}", e.what());
    }

    return result;
}

bool PixelToGPSConverter::pixelToBodyFrame(
    float pixel_x, float pixel_y,
    float drone_alt_m,
    double drone_roll_rad, double drone_pitch_rad, double drone_yaw_rad,
    double& body_x, double& body_y, double& body_z) const
{
    if (!isReady()) {
        AppLogger::get().error("PixelToGPSConverter::pixelToBodyFrame: not ready!");
        return false;
    }
    // 复用内部转换：传入占位GPS即可，仅取机体坐标
    PixelToGroundResult res = pixelToGroundWithBody(
        cv::Point2d(pixel_x, pixel_y),
        0.0, 0.0,  // 占位lat/lon（不影响机体坐标计算）
        drone_alt_m,
        drone_roll_rad, drone_pitch_rad, drone_yaw_rad);
    if (res.body_z <= 0.0) {
        return false;
    }
    body_x = res.body_x;
    body_y = res.body_y;
    body_z = res.body_z;
    return true;
}

std::vector<TargetGPS> PixelToGPSConverter::convertDetectionsToGPS(
    const std::vector<cv::Rect>& boxes,
    const std::vector<int>& classIds,
    double drone_lat,
    double drone_lon,
    float drone_alt,
    double drone_roll_rad,
    double drone_pitch_rad,
    double drone_yaw_rad
) const {
    std::vector<TargetGPS> results;

    if (boxes.size() != classIds.size()) {
        AppLogger::get().error("Warning: boxes and classIds size mismatch!");
        return results;
    }

    for (size_t i = 0; i < boxes.size(); ++i) {
        const cv::Rect& box = boxes[i];
        int classId = classIds[i];

        float cx = box.x + box.width / 2.0f;
        float cy = box.y + box.height / 2.0f;

        PixelToGroundResult res = pixelToGroundWithBody(
            cv::Point2d(cx, cy),
            drone_lat,
            drone_lon,
            drone_alt,
            drone_roll_rad,
            drone_pitch_rad,
            drone_yaw_rad
        );

        TargetGPS tg;
        tg.classId = classId;
        tg.box = box;
        tg.lon = res.gps.x;
        tg.lat = res.gps.y;
        tg.confidence = 1.0f;

        tg.body_x = res.body_x;
        tg.body_y = res.body_y;
        tg.body_z = res.body_z;

        results.push_back(tg);
    }

    return results;
}