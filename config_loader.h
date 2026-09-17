// config_loader.h
#pragma once
#include <string>
#include <vector>
#include <cstdint> // for uint8_t
#include<yaml-cpp/yaml.h>
// 通信子配置
struct SerialConfig {
    std::string port = "COM6";
    unsigned int baud_rate = 115200;
};

struct TcpConfig {
    std::string host = "127.0.0.1";
    unsigned short port = 5762;
};

struct UdpConfig {
    std::string host = "192.168.142.137";
    unsigned short port = 14555;
};

struct CommunicationConfig {
    std::string transport = "tcp"; // "serial", "tcp", "udp"
    SerialConfig serial;
    TcpConfig tcp;
    UdpConfig udp;
};

// MAVLink 子配置
struct MavlinkConfig {
    int system_id = 2;
    int component_id = 190;
    uint8_t target_system = 1;
    uint8_t target_component = 0;
};

// 曝光控制子配置
struct AeConfig {
    bool enable = true;                  // 软件 AE 总开关
    double target_brightness = 0.4;      // 目标亮度 (归一化 0~1)
    int exposure_min = -10;              // 曝光下限
    int exposure_max = -2;               // 曝光上限
    double kp = 0.3;                     // PI 比例系数
    double ki = 0.02;                    // PI 积分系数
    int update_interval = 5;             // 调节冷却帧数（每次调完等 N 帧再调）
    double deadband = 0.15;              // 亮度死区（误差在此范围内不调节）
};

// 相机子配置
struct CameraConfig {
    int index = 0;
    int width = 1920;
    int height = 1080;
    int fps = 30;
    // Camera intrinsics (defaults for 1080p, calibrate for actual camera)
    double fx = 1000.0;
    double fy = 1000.0;
    double cx = 960.0;
    double cy = 540.0;
    // Distortion coefficients (OpenCV format: k1,k2,p1,p2,k3[,k4,k5,k6[,s1-s4[,tx,ty]]])
    std::vector<double> dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};
    // Camera mount offset from FC (meters, body frame: X fwd Y right Z down)
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.4;
    // Camera mount angles (degrees, gimbal calibration)
    double mount_roll = 0.0;
    double mount_pitch = 0.0;            // 相机直接朝下安装
    double mount_yaw = 0.0;
    AeConfig ae;                         // 曝光控制与预处理配置
};

// 控制参数子配置
struct ControlConfig {
    float kp = 0.001f;
    float deadzone_x = 15.0f;          // 最终死区阈值 (像素, X方向)
    float deadzone_y = 15.0f;          // 最终死区阈值 (像素, Y方向)
    float deadzone_big = 50.0f;
    float max_speed = 0.3f;
    double initial_heading = 347.9;
    bool auto_heading = false;          // true=自动读机头航向；false=用手动 initial_heading
    double magnetic_declination = 0.0;  // 磁偏角(度)：真北航向 = 磁航向 + 磁偏角（飞控已报真北则填0）
    int toudan_time_out = 120000;
    bool manual_observe = false;   // 测试用: 拦截状态机，仅检测+输出，不发控制
    double gps_arrival_threshold_m = 0.3;       // GPS到达阈值 (meters)
    int gps_fly_timeout_ms = 10000;             // GPS飞行超时 (ms)
    double max_search_radius_m = 35.0;          // 搜索阶段最大距离起飞点距离 (米)
    // 舵机参数
    int servo_channel_left = 6;                 // 左舵机通道号
    int servo_channel_right = 7;                // 右舵机通道号
    int servo_offset_left_x = 50;               // 左舵机X轴偏移 (像素)
    int servo_offset_right_x = -70;             // 右舵机X轴偏移 (像素)
    int servo_release_pwm_left = 1200;          // 左舵机释放PWM值
    int servo_release_pwm_right = 1700;         // 右舵机释放PWM值
};

// 航点规划子配置
struct WaypointConfig {
    double default_altitude = 2.5;
    double distance_to_drop_zone = 30.0;
    double distance_to_recon1 = 56.0;
    double distance_to_recon2 = 59.0;
    double recon_side_distance = 2.6;
    float delay_takeoff = 1.0f;
    float delay_drop_point = 2.0f;
    float delay_after_drop = 2.0f;
    float delay_return = 1.0f;
    float delay_recon_main = 5.0f;
    float delay_recon_secondary = 3.0f;
    float delay_recon_side = 3.0f;
};

// YOLO 模型子配置
struct YolomodeConfig {
    std::string backend="onnx";
    std::string model_path = "best.onnx";
    std::string engine_path = "best_cuadc10.10.2.engine";
    std::string class_path = "classes2.txt";
    bool use_gpu = false;

    // 侦察阶段 ONNX 模型 (Mission2 使用，检测不同类别)
    std::string recon_model_path = "recon.onnx";
    std::string recon_class_path = "classes_recon.txt";
    std::string recon_target_class = "";   // 从类文件第一行自动读取
};

// RTK 网络服务子配置
struct RtkConfig {
    bool enabled = false;
    std::string ntrip_host;
    int ntrip_port = 8002;
    std::string ntrip_mountpoint;
    std::string ntrip_username;
    std::string ntrip_password;
    double gps_accuracy_threshold = 1.0;  // h_acc 阈值(米)
    int min_satellites = 10;
    int gga_interval_sec = 5;
    int reconnect_interval_sec = 5;
    int max_reconnect_attempts = 3;
    int fix_timeout_sec = 60;
    bool allow_gps_flight = true;
};

// 目标跟踪/聚类子配置
struct TrackingConfig {
    double cluster_radius_m = 0.7;
    int min_samples = 5;
    int max_targets = 3;
    int max_age_frames = 30;
};

// 固定翼航线表：每条 = 一个航点（相对起飞航向 + 相对 HOME 的距离 + 高度）
struct RouteWaypointConfig {
    double bearing_offset_deg = 0.0;   // 相对起飞航向的方位角偏移（度）
    double distance_m = 0.0;           // 相对 HOME 的水平距离（米）
    double alt_m = 0.0;                // 高度（米，相对起飞点）
};

// 固定翼子配置（ArduPlane）
struct FixedWingConfig {
    int mode_auto = 10;                 // ArduPlane AUTO
    int mode_guided = 15;               // ArduPlane GUIDED
    int mode_rtl = 11;                  // ArduPlane RTL
    int servo_channel = 8;              // 投弹舵机通道
    int servo_release_pwm = 1995;       // 投弹舵机 PWM
    double takeoff_pitch_deg = 18.0;    // 手抛爬升俯仰角（TAKEOFF param1）
    double takeoff_climb_alt_m = 38.0;  // 起飞爬升高度（TAKEOFF z）
    double drop_altitude = 30.0;        // 投弹引导飞越高度 (m)
    double entry_extension_distance_m = 80.0; // 入口点=目标沿 Home→目标 方向延长距离 (m)，须 > 投弹提前量
    double entry_reach_radius_m = 40.0;       // 到达入口点附近后切入返航直线的半径 (m)，须 > 飞控 WP_RADIUS
    double drop_fall_time_s = 0.0;      // 弹道落体时间；0=按高度算 sqrt(2h/g)
    double release_lead_offset_m = 0.0; // 投弹提前量附加偏移 (m)
    double fallback_bearing_offset_deg = -63.3; // 兜底点相对起飞航向的方位角偏移
    double fallback_distance_m = 66.1;          // 兜底点相对 HOME 的距离 (m)
    int strike_handoff_seq = 8;         // 巡航末点 MISSION 序号（切 GUIDED 时机）
    int landing_start_seq = 9;          // 投弹后跳转的降落段起始 MISSION 序号
    int task_type = 1;                  // 打击目标选择规则：1=任务一(价值最高) 2=任务二(中位数)
    std::vector<RouteWaypointConfig> route; // 航线表（最后一条自动作为 NAV_LAND）
};

// 编号 OCR 识别子配置 (PP-OCRv5_mobile_rec + TensorRT)
struct OcrConfig {
    bool enabled = false;                          // OCR 总开关
    bool enable_on_recon = true;                   // 侦察阶段启用识别
    bool enable_on_bomb = true;                    // 投弹阶段启用识别
    std::string onnx_path = "PP-OCRv5_mobile_rec.onnx";
    std::string engine_path = "PP-OCRv5_mobile_rec.engine";
    std::string dict_path = "ppocr_keys_v5.txt";
    std::string inference_yml_path = "inference.yml";
    bool digit_only = true;                        // 只保留数字字符
    bool use_fp16 = true;
    int rec_max_width = 320;                       // 动态宽度上限
    float min_box_width = 16.0f;                   // 候选框最小宽(像素)
    float min_box_height = 8.0f;                   // 候选框最小高(像素)
    int max_boxes_per_cycle = 24;                  // 每轮最多识别框数
    int min_infer_interval_ms = 80;                // 识别限流间隔(ms)
    float crop_padding = 0.08f;                    // 裁剪外扩比例
    int vote_window = 5;                           // 多帧投票窗口
    int min_votes = 2;                             // 稳定所需最少票数
    int entry_ttl_ms = 3000;                       // 结果保留时间(ms)
};

// 主配置结构体
struct AppConfig {
    CommunicationConfig communication;
    MavlinkConfig mavlink;
    CameraConfig camera;
    ControlConfig control;
    TrackingConfig tracking;
    YolomodeConfig yolo;
    WaypointConfig waypoint;
    RtkConfig rtk;
    FixedWingConfig fixedwing;
    OcrConfig ocr;
};
AppConfig load_config_from_yaml(const std::string& path = "config.yaml");