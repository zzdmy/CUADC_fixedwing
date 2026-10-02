#pragma once
#include "../PixelToGPSConverter.h"
#include <memory>
#include <vector>
#include "../yolodecet/yolo_tracker.h"
class GPSTargetClusterer {
public:
    struct Cluster {
        int id;
        double lat;           // running mean GPS
        double lon;
        cv::Rect avg_box;     // running mean bounding box (for CSRT init + X sorting)
        int sample_count;
        int frames_since_update;
        double body_x = 0.0;  // running mean body-frame position (m)
        double body_y = 0.0;
        int class_id = -1;    // 目标类别 id（多类模型就绪后用于规则选目标，见 TargetSelection.h）
    };

    // Accept shared camera converter (configured once in main thread)
    // target_class: 聚类的类别名（默认 "toudan" 旧占位；任务二传 "bucket"=天井）
    // max_targets: max clusters to maintain (0 = use config value)
    GPSTargetClusterer(std::shared_ptr<PixelToGPSConverter> converter,
                       const std::string& target_class = "toudan",
                       int max_targets = 0);

    // Per-frame update: convert detections to GPS, match to clusters
    void update(const std::vector<yoloout>& detections,
                double drone_lat, double drone_lon,
                float drone_alt_m,
                double drone_roll_rad, double drone_pitch_rad,
                double drone_yaw_rad);

    // Returns clusters with sample_count >= min_samples, sorted by avg_pixel_x
    std::vector<Cluster> getStableTargets() const;

    // Returns all clusters (including unstable)
    std::vector<Cluster> getAllClusters() const;

    void reset();

private:
    std::shared_ptr<PixelToGPSConverter> converter_;
    std::string target_class_;
    double cluster_radius_m_;
    int min_samples_;
    int max_targets_;
    int max_age_frames_;
    int next_id_ = 1;
    int frame_count_ = 0;
    std::vector<Cluster> clusters_;

    static double gpsDistanceMeters(double lat1, double lon1,
                                    double lat2, double lon2);
};
