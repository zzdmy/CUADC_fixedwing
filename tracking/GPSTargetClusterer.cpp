#include "GPSTargetClusterer.h"
#include "../ConfigManager.h"
#define GEOGRAPHICLIB_SHARED_LIB 1
#include <GeographicLib/Geodesic.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>
#include "../AppLogger.h"

GPSTargetClusterer::GPSTargetClusterer(std::shared_ptr<PixelToGPSConverter> converter,
                                       const std::string& target_class,
                                       int max_targets)
    : converter_(std::move(converter)), target_class_(target_class) {
    const auto& cfg = ConfigManager::getInstance().getConfig();
    const auto& trk = cfg.tracking;

    cluster_radius_m_ = trk.cluster_radius_m;
    min_samples_     = trk.min_samples;
    max_targets_     = max_targets > 0 ? max_targets : trk.max_targets;
    max_age_frames_  = trk.max_age_frames;
}

void GPSTargetClusterer::update(const std::vector<yoloout>& detections,
                                double drone_lat, double drone_lon,
                                float drone_alt_m,
                                double drone_roll_rad, double drone_pitch_rad,
                                double drone_yaw_rad) {
    frame_count_++;

    // Collect boxes and class IDs from detections matching target_class_
    // (task2: "bucket"=天井；task1: 空字符串 = 全类别，class_id 已随簇保存，
    //  价值最高/中位数规则在 MissionScheduler 侧用簇的 class_id 完成)
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;
    for (const auto& det : detections) {
        if (target_class_.empty() || det.name == target_class_) {
            boxes.push_back(det.box);
            classIds.push_back(det.classId);
        }
    }
    if (boxes.empty()) {
        // Age all clusters
        for (auto& c : clusters_) {
            c.frames_since_update++;
        }
        // Remove stale
        for (const auto& c : clusters_) {
            if (c.frames_since_update > max_age_frames_) {
                AppLogger::get().info("[聚类] C{} 丢失 ({}帧未更新)", c.id, c.frames_since_update);
            }
        }
        clusters_.erase(
            std::remove_if(clusters_.begin(), clusters_.end(),
                [this](const Cluster& c) {
                    return c.frames_since_update > max_age_frames_;
                }),
            clusters_.end());
        return;
    }

    // Convert pixel detections to GPS
    auto targets = converter_->convertDetectionsToGPS(
        boxes, classIds, drone_lat, drone_lon, drone_alt_m,
        drone_roll_rad, drone_pitch_rad, drone_yaw_rad);

    // Match each detection to nearest existing cluster
    for (const auto& t : targets) {
        int best_idx = -1;
        double best_dist = cluster_radius_m_;

        for (int i = 0; i < static_cast<int>(clusters_.size()); ++i) {
            double d = gpsDistanceMeters(t.lat, t.lon,
                clusters_[i].lat, clusters_[i].lon);
            if (d < best_dist) {
                best_dist = d;
                best_idx = i;
            }
        }

        if (best_idx >= 0) {
            // Update existing cluster with running mean
            auto& c = clusters_[best_idx];
            double n = static_cast<double>(c.sample_count);
            c.lat = (c.lat * n + t.lat) / (n + 1.0);
            c.lon = (c.lon * n + t.lon) / (n + 1.0);
            c.body_x = (c.body_x * n + t.body_x) / (n + 1.0);
            c.body_y = (c.body_y * n + t.body_y) / (n + 1.0);
            c.avg_box.x = static_cast<int>((c.avg_box.x * n + t.box.x) / (n + 1.0));
            c.avg_box.y = static_cast<int>((c.avg_box.y * n + t.box.y) / (n + 1.0));
            c.avg_box.width  = static_cast<int>((c.avg_box.width  * n + t.box.width)  / (n + 1.0));
            c.avg_box.height = static_cast<int>((c.avg_box.height * n + t.box.height) / (n + 1.0));
            c.class_id = t.classId;   // 记录类别（多类模型就绪后用）
            c.sample_count++;
            c.frames_since_update = 0;
            if (c.sample_count == min_samples_) {
                AppLogger::get().info("[聚类] C{} 达到稳定 (样本{}): GPS({:.7f},{:.7f}) body=({:.2f},{:.2f})m",
                    c.id, c.sample_count, c.lat, c.lon, c.body_x, c.body_y);
            }
        }
        else if (static_cast<int>(clusters_.size()) < max_targets_) {
            // Create new cluster
            Cluster c;
            c.id = next_id_++;
            c.lat = t.lat;
            c.lon = t.lon;
            c.body_x = t.body_x;
            c.body_y = t.body_y;
            c.avg_box = t.box;
            c.class_id = t.classId;   // 记录类别（多类模型就绪后用）
            c.sample_count = 1;
            c.frames_since_update = 0;
            clusters_.push_back(c);
            AppLogger::get().info("[聚类] 新目标 C{}: GPS({:.7f},{:.7f}) body=({:.2f},{:.2f})m box({},{},{}x{})",
                c.id, c.lat, c.lon, c.body_x, c.body_y,
                c.avg_box.x, c.avg_box.y, c.avg_box.width, c.avg_box.height);
        }
        // else: detection too far from any cluster and max_targets reached — discard
    }

    // Age all clusters
    for (auto& c : clusters_) {
        c.frames_since_update++;
    }

    // Remove stale clusters
    for (const auto& c : clusters_) {
        if (c.frames_since_update > max_age_frames_) {
            AppLogger::get().info("[聚类] C{} 丢失 ({}帧未更新)", c.id, c.frames_since_update);
        }
    }
    clusters_.erase(
        std::remove_if(clusters_.begin(), clusters_.end(),
            [this](const Cluster& c) {
                return c.frames_since_update > max_age_frames_;
            }),
        clusters_.end());

    // 周期状态（约每秒一条，防刷屏）：各簇 body 位置/样本数
    if (!clusters_.empty() && frame_count_ % 5 == 0) {
        std::ostringstream oss;
        for (const auto& c : clusters_) {
            oss << " C" << c.id << "(" << std::fixed << std::setprecision(1)
                << c.body_x << "," << c.body_y << ")n=" << c.sample_count;
        }
        AppLogger::get().info("[聚类] 状态:{}", oss.str());
    }
}

std::vector<GPSTargetClusterer::Cluster> GPSTargetClusterer::getStableTargets() const {
    std::vector<Cluster> stable;
    for (const auto& c : clusters_) {
        if (c.sample_count >= min_samples_) {
            stable.push_back(c);
        }
    }
    // Sort by average box center X: left to right on screen
    std::sort(stable.begin(), stable.end(),
        [](const Cluster& a, const Cluster& b) {
            double cx_a = a.avg_box.x + a.avg_box.width / 2.0;
            double cx_b = b.avg_box.x + b.avg_box.width / 2.0;
            return cx_a < cx_b;
        });
    return stable;
}

std::vector<GPSTargetClusterer::Cluster> GPSTargetClusterer::getAllClusters() const {
    return clusters_;
}

void GPSTargetClusterer::reset() {
    clusters_.clear();
    next_id_ = 1;
}

double GPSTargetClusterer::gpsDistanceMeters(double lat1, double lon1,
                                              double lat2, double lon2) {
    double s12;
    GeographicLib::Geodesic::WGS84().Inverse(lat1, lon1, lat2, lon2, s12);
    return s12;
}
