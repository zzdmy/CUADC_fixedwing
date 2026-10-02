// config_loader.cpp
#include "config_loader.h"
#include <stdexcept>
#include <fstream>
#include "AppLogger.h"

AppConfig load_config_from_yaml(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);

    AppConfig cfg;

    // communication
    auto comm = root["communication"];
    cfg.communication.transport = comm["transport"].as<std::string>(cfg.communication.transport);

    auto serial = comm["serial"];
    cfg.communication.serial.port = serial["port"].as<std::string>(cfg.communication.serial.port);
    cfg.communication.serial.baud_rate = serial["baud_rate"].as<unsigned int>(cfg.communication.serial.baud_rate);

    auto tcp = comm["tcp"];
    cfg.communication.tcp.host = tcp["host"].as<std::string>(cfg.communication.tcp.host);
    cfg.communication.tcp.port = tcp["port"].as<unsigned short>(cfg.communication.tcp.port);

    auto udp = comm["udp"];
    cfg.communication.udp.host = udp["host"].as<std::string>(cfg.communication.udp.host);
    cfg.communication.udp.port = udp["port"].as<unsigned short>(cfg.communication.udp.port);

    // mavlink
    auto ml = root["mavlink"];
    cfg.mavlink.system_id = ml["system_id"].as<int>(cfg.mavlink.system_id);
    cfg.mavlink.component_id = ml["component_id"].as<int>(cfg.mavlink.component_id);
    cfg.mavlink.target_system = static_cast<uint8_t>(ml["target_system"].as<int>(cfg.mavlink.target_system));
    cfg.mavlink.target_component = static_cast<uint8_t>(ml["target_component"].as<int>(cfg.mavlink.target_component));

    // camera
    auto cam = root["camera"];
    cfg.camera.index = cam["index"].as<int>(cfg.camera.index);
    cfg.camera.width = cam["width"].as<int>(cfg.camera.width);
    cfg.camera.height = cam["height"].as<int>(cfg.camera.height);
    cfg.camera.fps = cam["fps"].as<int>(cfg.camera.fps);
    cfg.camera.fx = cam["fx"].as<double>(cfg.camera.fx);
    cfg.camera.fy = cam["fy"].as<double>(cfg.camera.fy);
    cfg.camera.cx = cam["cx"].as<double>(cfg.camera.cx);
    cfg.camera.cy = cam["cy"].as<double>(cfg.camera.cy);
    if (cam["dist_coeffs"].IsSequence()) {
        cfg.camera.dist_coeffs = cam["dist_coeffs"].as<std::vector<double>>(cfg.camera.dist_coeffs);
    }
    cfg.camera.tx = cam["tx"].as<double>(cfg.camera.tx);
    cfg.camera.ty = cam["ty"].as<double>(cfg.camera.ty);
    cfg.camera.tz = cam["tz"].as<double>(cfg.camera.tz);
    cfg.camera.mount_roll = cam["mount_roll"].as<double>(cfg.camera.mount_roll);
    cfg.camera.mount_pitch = cam["mount_pitch"].as<double>(cfg.camera.mount_pitch);
    cfg.camera.mount_yaw = cam["mount_yaw"].as<double>(cfg.camera.mount_yaw);

    // ae (auto exposure)
    auto ae = cam["ae"];
    cfg.camera.ae.enable = ae["enable"].as<bool>(cfg.camera.ae.enable);
    cfg.camera.ae.target_brightness = ae["target_brightness"].as<double>(cfg.camera.ae.target_brightness);
    cfg.camera.ae.exposure_min = ae["exposure_min"].as<int>(cfg.camera.ae.exposure_min);
    cfg.camera.ae.exposure_max = ae["exposure_max"].as<int>(cfg.camera.ae.exposure_max);
    cfg.camera.ae.kp = ae["kp"].as<double>(cfg.camera.ae.kp);
    cfg.camera.ae.ki = ae["ki"].as<double>(cfg.camera.ae.ki);
    cfg.camera.ae.update_interval = ae["update_interval"].as<int>(cfg.camera.ae.update_interval);
    cfg.camera.ae.deadband = ae["deadband"].as<double>(cfg.camera.ae.deadband);

    // tracking
    auto trk = root["tracking"];
    cfg.tracking.cluster_radius_m = trk["cluster_radius_m"].as<double>(cfg.tracking.cluster_radius_m);
    cfg.tracking.min_samples = trk["min_samples"].as<int>(cfg.tracking.min_samples);
    cfg.tracking.max_targets = trk["max_targets"].as<int>(cfg.tracking.max_targets);
    cfg.tracking.max_age_frames = trk["max_age_frames"].as<int>(cfg.tracking.max_age_frames);

    // control
    auto ctrl = root["control"];
    cfg.control.initial_heading = ctrl["initial_heading"].as<double>(cfg.control.initial_heading);
    cfg.control.auto_heading = ctrl["auto_heading"].as<bool>(cfg.control.auto_heading);
    cfg.control.magnetic_declination = ctrl["magnetic_declination"].as<double>(cfg.control.magnetic_declination);
    cfg.control.toudan_time_out = ctrl["toudan_time_out"].as<int>(cfg.control.toudan_time_out);
    cfg.control.manual_observe = ctrl["manual_observe"].as<bool>(cfg.control.manual_observe);
    cfg.control.enable_display = ctrl["enable_display"].as<bool>(cfg.control.enable_display);
    cfg.control.gps_arrival_threshold_m = ctrl["gps_arrival_threshold_m"].as<double>(cfg.control.gps_arrival_threshold_m);
    cfg.control.gps_fly_timeout_ms = ctrl["gps_fly_timeout_ms"].as<int>(cfg.control.gps_fly_timeout_ms);
    cfg.control.max_search_radius_m = ctrl["max_search_radius_m"].as<double>(cfg.control.max_search_radius_m);
	cfg.control.simulate_home = ctrl["simulate_home"].as<bool>(cfg.control.simulate_home);
	cfg.control.sim_home_lat = ctrl["sim_home_lat"].as<double>(cfg.control.sim_home_lat);
	cfg.control.sim_home_lon = ctrl["sim_home_lon"].as<double>(cfg.control.sim_home_lon);

    // yolo
    auto yolo = root["yolo"];
    cfg.yolo.backend = yolo["backend"].as<std::string>(cfg.yolo.backend);
    cfg.yolo.model_path = yolo["model_path"].as<std::string>(cfg.yolo.model_path);
    cfg.yolo.engine_path = yolo["engine_path"].as<std::string>(cfg.yolo.engine_path);
    cfg.yolo.class_path = yolo["class_path"].as<std::string>(cfg.yolo.class_path);
    cfg.yolo.use_gpu = yolo["use_gpu"].as<bool>(cfg.yolo.use_gpu);
    cfg.yolo.well_model_path = yolo["well_model_path"].as<std::string>(cfg.yolo.well_model_path);
    cfg.yolo.well_engine_path = yolo["well_engine_path"].as<std::string>(cfg.yolo.well_engine_path);
    cfg.yolo.well_class_path = yolo["well_class_path"].as<std::string>(cfg.yolo.well_class_path);
    cfg.yolo.pattern_model_path = yolo["pattern_model_path"].as<std::string>(cfg.yolo.pattern_model_path);
    cfg.yolo.pattern_engine_path = yolo["pattern_engine_path"].as<std::string>(cfg.yolo.pattern_engine_path);
    cfg.yolo.pattern_class_path = yolo["pattern_class_path"].as<std::string>(cfg.yolo.pattern_class_path);
    cfg.yolo.detect_frame_stride = yolo["detect_frame_stride"].as<int>(cfg.yolo.detect_frame_stride);

    // rtk
    auto rtk = root["rtk"];
    cfg.rtk.enabled = rtk["enabled"].as<bool>(cfg.rtk.enabled);
    cfg.rtk.ntrip_host = rtk["ntrip_host"].as<std::string>(cfg.rtk.ntrip_host);
    cfg.rtk.ntrip_port = rtk["ntrip_port"].as<int>(cfg.rtk.ntrip_port);
    cfg.rtk.ntrip_mountpoint = rtk["ntrip_mountpoint"].as<std::string>(cfg.rtk.ntrip_mountpoint);
    cfg.rtk.ntrip_username = rtk["ntrip_username"].as<std::string>(cfg.rtk.ntrip_username);
    cfg.rtk.ntrip_password = rtk["ntrip_password"].as<std::string>(cfg.rtk.ntrip_password);
    cfg.rtk.gps_accuracy_threshold = rtk["gps_accuracy_threshold"].as<double>(cfg.rtk.gps_accuracy_threshold);
    cfg.rtk.min_satellites = rtk["min_satellites"].as<int>(cfg.rtk.min_satellites);
    cfg.rtk.gga_interval_sec = rtk["gga_interval_sec"].as<int>(cfg.rtk.gga_interval_sec);
    cfg.rtk.reconnect_interval_sec = rtk["reconnect_interval_sec"].as<int>(cfg.rtk.reconnect_interval_sec);
    cfg.rtk.max_reconnect_attempts = rtk["max_reconnect_attempts"].as<int>(cfg.rtk.max_reconnect_attempts);
    cfg.rtk.fix_timeout_sec = rtk["fix_timeout_sec"].as<int>(cfg.rtk.fix_timeout_sec);
    cfg.rtk.allow_gps_flight = rtk["allow_gps_flight"].as<bool>(cfg.rtk.allow_gps_flight);

    // record（机内录像，素材收集用）
    auto rec = root["record"];
    cfg.record.enable = rec["enable"].as<bool>(cfg.record.enable);
    cfg.record.dir = rec["dir"].as<std::string>(cfg.record.dir);
    cfg.record.segment_sec = rec["segment_sec"].as<int>(cfg.record.segment_sec);
    cfg.record.max_total_gb = rec["max_total_gb"].as<double>(cfg.record.max_total_gb);

    // fixedwing（固定翼/ArduPlane）
    auto fw = root["fixedwing"];
    cfg.fixedwing.mode_auto = fw["mode_auto"].as<int>(cfg.fixedwing.mode_auto);
    cfg.fixedwing.mode_guided = fw["mode_guided"].as<int>(cfg.fixedwing.mode_guided);
    cfg.fixedwing.mode_rtl = fw["mode_rtl"].as<int>(cfg.fixedwing.mode_rtl);
    cfg.fixedwing.servo_channel = fw["servo_channel"].as<int>(cfg.fixedwing.servo_channel);
    cfg.fixedwing.servo_release_pwm = fw["servo_release_pwm"].as<int>(cfg.fixedwing.servo_release_pwm);
    cfg.fixedwing.takeoff_pitch_deg = fw["takeoff_pitch_deg"].as<double>(cfg.fixedwing.takeoff_pitch_deg);
    cfg.fixedwing.takeoff_climb_alt_m = fw["takeoff_climb_alt_m"].as<double>(cfg.fixedwing.takeoff_climb_alt_m);
    cfg.fixedwing.drop_altitude = fw["drop_altitude"].as<double>(cfg.fixedwing.drop_altitude);
    cfg.fixedwing.entry_extension_distance_m = fw["entry_extension_distance_m"].as<double>(cfg.fixedwing.entry_extension_distance_m);
    cfg.fixedwing.entry_reach_radius_m = fw["entry_reach_radius_m"].as<double>(cfg.fixedwing.entry_reach_radius_m);
    cfg.fixedwing.drop_fall_time_s = fw["drop_fall_time_s"].as<double>(cfg.fixedwing.drop_fall_time_s);
    cfg.fixedwing.release_lead_offset_m = fw["release_lead_offset_m"].as<double>(cfg.fixedwing.release_lead_offset_m);
    cfg.fixedwing.fallback_start_index = fw["fallback_start_index"].as<int>(cfg.fixedwing.fallback_start_index);
    cfg.fixedwing.strike_handoff_seq = fw["strike_handoff_seq"].as<int>(cfg.fixedwing.strike_handoff_seq);
    cfg.fixedwing.landing_start_seq = fw["landing_start_seq"].as<int>(cfg.fixedwing.landing_start_seq);
    cfg.fixedwing.task_type = fw["task_type"].as<int>(cfg.fixedwing.task_type);
    cfg.fixedwing.toudan_time_out_task1 = fw["toudan_time_out_task1"].as<int>(cfg.fixedwing.toudan_time_out_task1);
    cfg.fixedwing.toudan_time_out_task2 = fw["toudan_time_out_task2"].as<int>(cfg.fixedwing.toudan_time_out_task2);
    cfg.fixedwing.guided_entry_timeout_ms = fw["guided_entry_timeout_ms"].as<int>(cfg.fixedwing.guided_entry_timeout_ms);
    cfg.fixedwing.guided_release_timeout_ms = fw["guided_release_timeout_ms"].as<int>(cfg.fixedwing.guided_release_timeout_ms);
    if (fw["route"].IsSequence()) {
        cfg.fixedwing.route.clear();
        for (const auto& n : fw["route"]) {
            RouteWaypointConfig w;
            w.bearing_offset_deg = n["bearing_offset_deg"].as<double>(0.0);
            w.distance_m         = n["distance_m"].as<double>(0.0);
            w.alt_m              = n["alt_m"].as<double>(0.0);
            cfg.fixedwing.route.push_back(w);
        }
    }
    // ocr (编号识别)
    auto ocr = root["ocr"];
    cfg.ocr.enabled = ocr["enabled"].as<bool>(cfg.ocr.enabled);
    cfg.ocr.enable_on_recon = ocr["enable_on_recon"].as<bool>(cfg.ocr.enable_on_recon);
    cfg.ocr.enable_on_bomb = ocr["enable_on_bomb"].as<bool>(cfg.ocr.enable_on_bomb);
    cfg.ocr.onnx_path = ocr["onnx_path"].as<std::string>(cfg.ocr.onnx_path);
    cfg.ocr.engine_path = ocr["engine_path"].as<std::string>(cfg.ocr.engine_path);
    cfg.ocr.dict_path = ocr["dict_path"].as<std::string>(cfg.ocr.dict_path);
    cfg.ocr.inference_yml_path = ocr["inference_yml_path"].as<std::string>(cfg.ocr.inference_yml_path);
    cfg.ocr.digit_only = ocr["digit_only"].as<bool>(cfg.ocr.digit_only);
    cfg.ocr.use_fp16 = ocr["use_fp16"].as<bool>(cfg.ocr.use_fp16);
    cfg.ocr.rec_max_width = ocr["rec_max_width"].as<int>(cfg.ocr.rec_max_width);
    cfg.ocr.use_det = ocr["use_det"].as<bool>(cfg.ocr.use_det);
    cfg.ocr.det_onnx_path = ocr["det_onnx_path"].as<std::string>(cfg.ocr.det_onnx_path);
    cfg.ocr.det_engine_path = ocr["det_engine_path"].as<std::string>(cfg.ocr.det_engine_path);
    cfg.ocr.det_limit_side = ocr["det_limit_side"].as<int>(cfg.ocr.det_limit_side);
    cfg.ocr.det_box_thresh = ocr["det_box_thresh"].as<float>(cfg.ocr.det_box_thresh);
    cfg.ocr.det_thresh = ocr["det_thresh"].as<float>(cfg.ocr.det_thresh);
    cfg.ocr.det_unclip_ratio = ocr["det_unclip_ratio"].as<float>(cfg.ocr.det_unclip_ratio);
    cfg.ocr.use_ori = ocr["use_ori"].as<bool>(cfg.ocr.use_ori);
    cfg.ocr.ori_onnx_path = ocr["ori_onnx_path"].as<std::string>(cfg.ocr.ori_onnx_path);
    cfg.ocr.ori_engine_path = ocr["ori_engine_path"].as<std::string>(cfg.ocr.ori_engine_path);
    cfg.ocr.min_box_width = ocr["min_box_width"].as<float>(cfg.ocr.min_box_width);
    cfg.ocr.min_box_height = ocr["min_box_height"].as<float>(cfg.ocr.min_box_height);
    cfg.ocr.max_boxes_per_cycle = ocr["max_boxes_per_cycle"].as<int>(cfg.ocr.max_boxes_per_cycle);
    cfg.ocr.min_infer_interval_ms = ocr["min_infer_interval_ms"].as<int>(cfg.ocr.min_infer_interval_ms);
    cfg.ocr.crop_padding = ocr["crop_padding"].as<float>(cfg.ocr.crop_padding);
    cfg.ocr.vote_window = ocr["vote_window"].as<int>(cfg.ocr.vote_window);
    cfg.ocr.min_votes = ocr["min_votes"].as<int>(cfg.ocr.min_votes);
    cfg.ocr.entry_ttl_ms = ocr["entry_ttl_ms"].as<int>(cfg.ocr.entry_ttl_ms);

    AppLogger::get().info("配置文件加载完成（含固定翼配置 {} 条航线）", cfg.fixedwing.route.size());
    return cfg;
}