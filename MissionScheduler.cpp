#include "AppLogger.h"
#include "WatchdogHelper.h"
#include "MissionScheduler.h"
#include "tracking/TrackedObject.h"
#include "tracking/StableDetectionTracker.h"
#include "tracking/GPSTargetClusterer.h"
#include "TargetSelection.h"
#include "ocr/OcrDigitReader.h"
#include "FrameDispatcher.h"
#include "config_loader.h"
#include "ConfigManager.h"
#include <stdlib.h>
#define GEOGRAPHICLIB_SHARED_LIB 1
#include <GeographicLib/Geodesic.hpp>
#include <iostream>
#include <cmath>
using namespace cv;

extern int actualWidth, actualHeight;
using namespace std;

// =====================================================================
// 模型接线点（模型就绪后只需确认/补充这里，选择逻辑不用改）
// 现在检测模型只有单类 "toudan"，无法区分靶标类型/价值/数字，
// 所以 value 由 class_id 查价值表得到（未知类别返回 0，退化为取首个稳定目标）。
// 任务二「数字靶标→中位数」的 value 解析待数字识别模型就绪后补：
//   - 若模型直接输出 0~99 的 class_id，直接 return class_id；
//   - 若按单个数字检出，需先按 GPS 聚合成两位数再填 value。
// =====================================================================
static double resolveTargetValue(int class_id, strike::StrikeTask task) {
    if (task == strike::StrikeTask::HighestValue) {
        const auto& table = strike::pictureValueTable();
        auto it = table.find(class_id);
        return it != table.end() ? it->second : 0.0;
    }
    return 0.0;  // 任务二：不走这里（编号 OCR 路径填 value，见候选循环）
}

MissionScheduler::MissionScheduler(std::shared_ptr<MavlinkProtocolHandler> mavlinkHandler,
	std::shared_ptr<PixelToGPSConverter> cameraConverter)
	: cfg_(ConfigManager::getInstance().getConfig()),
	mavlink_handler_(std::move(mavlinkHandler)),
	camera_converter_(std::move(cameraConverter))
{

	// 初始化代码（如果需要）
}

MissionScheduler::~MissionScheduler() {
	stop();
}

void MissionScheduler::start() {
	if (!running_) {
		running_ = true;
		scheduler_thread_ = std::thread(&MissionScheduler::runScheduler, this);//创建一个线程运行任务调度器

		//正确创建 MotionController 成员
		motion_ctrl_ = std::make_unique<MotionController>(mavlink_handler_);
		motion_ctrl_->setImageResolution(actualWidth, actualHeight);
		motion_ctrl_->start();
	}
}
// 更新时间
void updateGlobalTime() {
	auto now = std::chrono::steady_clock::now();
	auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
		now.time_since_epoch()
	).count();
	global_time_ns.store(ns, std::memory_order_release);
}

// 获取时间点
std::chrono::steady_clock::time_point getGlobalTime() {
	auto ns = global_time_ns.load(std::memory_order_acquire);
	return std::chrono::steady_clock::time_point(
		std::chrono::nanoseconds(ns)
	);
}

// 计算与当前时间的间隔（纳秒）
std::chrono::nanoseconds getElapsedTime() {
	auto now = std::chrono::steady_clock::now();
	auto ns = global_time_ns.load(std::memory_order_acquire);
	auto stored_time = std::chrono::steady_clock::time_point(
		std::chrono::nanoseconds(ns)
	);
	return now - stored_time;
}

// 计算与当前时间的间隔（秒）
double getElapsedSeconds() {
	return std::chrono::duration<double>(getElapsedTime()).count();
}

void MissionScheduler::stop() {
	if (running_.exchange(false)) { // 原子性地设置 running_ 为 false
		AppLogger::get().info("任务调度器已停止");
	}

	//停止并销毁 MotionController
	if (motion_ctrl_) {
		motion_ctrl_->stop();
		motion_ctrl_.reset(); // 或自动析构
	}
}

MissionState MissionScheduler::getCurrentState() const {
	return current_state_.load();//返回当前状态
}

void MissionScheduler::runScheduler() {//任务调度器实现

	// === 测试拦截: manual_observe 模式，跳过全部状态机 ===
	if (cfg_.control.manual_observe) {
		handleManualObserve();
		return;
	}
	// === 拦截结束 ===

	// 更新时间
	updateGlobalTime();
	while (running_) {
		switch (current_state_) {
		case MissionState::WaitingForInitialization:
		{
			handleWaitingState();
			break;
		}
		case MissionState::TakingOff:
		{
			handleTakeoffState();
			break;
		}
		case MissionState::Mission1InProgress:
		{
			handleMission1State();
			break;
		}
		case MissionState::LandingInProgress:
		{
			handleLandingState();
			break;
		}

		}

	}
}

// ============================================================
// 测试用: 手动飞行观察模式
// - 等待 RTK FIX 后进入循环
// - 循环: YOLO检测 → GPS转换 → 输出 body_x/body_y
// - 全程不发控制指令，不切飞行模式
// - 测完删掉 runScheduler 里的拦截块 + 本函数即可
// ============================================================
void MissionScheduler::handleManualObserve() {
	AppLogger::get().info("[ManualObserve] Waiting for RTK FIX (fix_type >= 6)...");

	mavlink_gps_raw_int_t gps_raw;
	while (running_) {
		if (mavlink_handler_->try_get_gps_raw_data(gps_raw)) {
			AppLogger::get().debug("  Lat: {:.7f}  Lon: {:.7f}  h_acc: {:.3f}m  fix_type: {}",
				(double)gps_raw.lat / 1e7, (double)gps_raw.lon / 1e7,
				gps_raw.h_acc / 1000.0f, static_cast<int>(gps_raw.fix_type));
			if (gps_raw.fix_type >= 6) {
				AppLogger::get().info("[ManualObserve] RTK FIX achieved, starting observation loop.");
				break;
			}
		}
		this_thread::sleep_for(chrono::milliseconds(500));
	}

	AppLogger::get().info("===== 精度验证模式 =====");
	AppLogger::get().info("请将飞机举到约 1.2m 高度，对准地面目标");
	AppLogger::get().info("输出格式: body_x=前(+)后(-)  body_y=右(+)左(-)  单位: 米");
	AppLogger::get().info("用尺子测量目标实际位置，与输出值对比验证精度。");
	AppLogger::get().info("=========================");

	int frame_count = 0;
	while (running_) {
		// 1. Get YOLO detections
		vector<yoloout> detections;
		auto data = get_latest_yolo();
		if (data) {
			detections = *data;
		}

		// 2. Get drone GPS + altitude
		mavlink_global_position_int_t gps;
		float drone_alt_m = 0;
		if (mavlink_handler_->try_get_gps_data(gps)) {
			if (gps.relative_alt > 0) {
				drone_alt_m = gps.relative_alt / 1000.0f;
			}
		}

		// 3. Get drone attitude
		mavlink_attitude_t att;
		double roll = 0, pitch = 0, yaw = 0;
		if (mavlink_handler_->try_get_imu_data(att)) {
			roll = att.roll;
			pitch = att.pitch;
			yaw = att.yaw;
		}

		// 4. 对每个检测框，计算 body-frame 偏移（含舵机偏移量）
		if (!detections.empty()) {
			AppLogger::get().info("--- [帧{}] 高度={:.2f}m 姿态(r={:.1f} p={:.1f} y={:.1f}) deg ---",
				frame_count, drone_alt_m, roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
			for (size_t i = 0; i < detections.size(); ++i) {
				const auto& d = detections[i];
				float cx = d.box.x + d.box.width / 2.0f;
				float cy = d.box.y + d.box.height / 2.0f;
				auto printBody = [&](const char* label, float shifted_cx) {
					double body_x = 0, body_y = 0, body_z = 0;
					if (camera_converter_->pixelToBodyFrame(shifted_cx, cy,
						drone_alt_m, roll, pitch, yaw,
						body_x, body_y, body_z))
					{
						AppLogger::get().info("  [目标{}][{}] 像素=({:.0f},{:.0f})  "
							"body=({:.3f}, {:.3f})m  ->  前{:.0f}cm  右{:.0f}cm",
							i, label, shifted_cx, cy,
							body_x, body_y,
							body_x * 100, body_y * 100);
					}
				};
				printBody("相机", cx);
				printBody("左舵", cx + cfg_.control.servo_offset_left_x);
				printBody("右舵", cx + cfg_.control.servo_offset_right_x);
			}
		}
		frame_count++;
		this_thread::sleep_for(chrono::milliseconds(100));
	}

	AppLogger::get().info("[ManualObserve] Exited.");
}

//WaitingForInitialization

void MissionScheduler::handleWaitingState() {//等待初始化

	mavlink_handler_->setFlightMode(static_cast<uint8_t>(cfg_.fixedwing.mode_rtl));//切换为引导模式
	bool initialized = false;
	if (data_stream_requested_) {
		mavlink_handler_->requestAllDataStreams();
		data_stream_requested_ = false;
	}
	bool waiting_for_gps = true;
	bool waiting_for_upload = true;
	mavlink_gps_raw_int_t global_pos_data;
	while (waiting_for_upload) {
		if (cfg_.rtk.enabled) {
			// RTK mode: wait for RTK FIX (implies GPS accuracy + satellite count)
			AppLogger::get().info("[RTK] Waiting for RTK FIX (fix_type >= 6)...");
			auto rtk_timeout = std::chrono::seconds(cfg_.rtk.fix_timeout_sec);

			while (running_ && waiting_for_gps) {
				auto result = WatchdogHelper::waitUntil(
					[this, &global_pos_data]() {
						if (mavlink_handler_->try_get_gps_raw_data(global_pos_data)) {
							AppLogger::get().info("  纬度: {:.7f}, 经度: {:.7f}, 水平精度: {:.3f}m, 卫星: {}, fix_type: {}",
								(double)global_pos_data.lat / 1e7, (double)global_pos_data.lon / 1e7,
								global_pos_data.h_acc / 1000.0f, (int)global_pos_data.satellites_visible,
								static_cast<int>(global_pos_data.fix_type));
							if (global_pos_data.fix_type >= 5) {
								mavlink_handler_->sendSetHomePosition();
								return true;
							}
						}
						return false;
					},
					running_, rtk_timeout, std::chrono::milliseconds(500));

				if (result == WaitResult::ConditionMet) {
					AppLogger::get().info("[RTK] FIX achieved (type={}), global origin reset done.",
						static_cast<int>(global_pos_data.fix_type));
					waiting_for_gps = false;
					break;
			}
				if (result == WaitResult::Stopped) {
					waiting_for_gps = false;
					break;
			}
				// Timeout
				if (cfg_.rtk.allow_gps_flight) {
					AppLogger::get().warn("[RTK] FIX timeout after {}s, proceeding with GPS-only.", cfg_.rtk.fix_timeout_sec);
					waiting_for_gps = false;
				} else {
					AppLogger::get().warn("[RTK] FIX timeout, waiting indefinitely...");
					// Reset timeout and continue waiting
					continue;
			}
				break;
			}
		} else {
			// GPS-only mode: wait for accuracy + satellites
			while (waiting_for_gps) {
				if (mavlink_handler_->try_get_gps_raw_data(global_pos_data)) {
					AppLogger::get().info("  纬度: {:.7f}, 经度: {:.7f}",
						(double)global_pos_data.lat / 1e7, (double)global_pos_data.lon / 1e7);
					if (global_pos_data.h_acc != UINT32_MAX) {
						AppLogger::get().info("水平精度: {:.3f}m", global_pos_data.h_acc / 1000.0f);
					}
					AppLogger::get().info("卫星数量: {}", (int)global_pos_data.satellites_visible);
					if (global_pos_data.h_acc / 1000.0f < 1 && (int)global_pos_data.satellites_visible >= 10) {
						waiting_for_gps = false;
						break;
					}
					AppLogger::get().info("等待GPS定位中...");
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		}
		home_lat_ = (double)global_pos_data.lat / 1e7;
		home_lon_ = (double)global_pos_data.lon / 1e7;
		double heading = cfg_.control.initial_heading;  // 兜底：手动真北航向
		// 自动读机头航向（罗盘磁航向 → 真北）
		if (cfg_.control.auto_heading) {
			mavlink_global_position_int_t gps_hdg;
			if (mavlink_handler_->try_get_gps_data(gps_hdg) && gps_hdg.hdg <= 36000) {
				double mag = gps_hdg.hdg / 100.0;                  // 磁航向
				heading = mag + cfg_.control.magnetic_declination; // 真北 = 磁北 + 磁偏角
				if (heading < 0.0) heading += 360.0;
				if (heading >= 360.0) heading -= 360.0;
				AppLogger::get().info("[航向] 自动读机头磁航向 {:.1f}° + 磁偏角 {:.1f}° = 真北航向 {:.1f}°",
					mag, cfg_.control.magnetic_declination, heading);
			} else {
				AppLogger::get().warn("[航向] 自动读航向失败，退回手动 initial_heading={:.1f}°", heading);
			}
		}
		takeoff_heading_ = heading;

		if (mavlink_handler_->uploadMissionWaypoints(home_lat_, home_lon_, heading)) {
			waiting_for_upload = false;
			break;// 发送成功
		}
	}
	mavlink_handler_->setFlightMode(static_cast<uint8_t>(cfg_.fixedwing.mode_auto));//切换为自动模式
	initialized = true;//初始化成功
	if (initialized) {
		current_state_ = MissionState::TakingOff;
	}
}

void MissionScheduler::handleTakeoffState() {
	// 检查是否完成起飞
	bool takeoff_completed = false;
	bool takeoff_msg_printed = true;
	while (!takeoff_completed) {
		mavlink_mission_current_t current;
		if (mavlink_handler_->try_get_latest_mission_current(current)) {
			if (current.seq == 1 && takeoff_msg_printed) {
				AppLogger::get().info("等待起飞");
				takeoff_msg_printed = false;//输出一次
			}
			else if (current.seq >= 2) {
				AppLogger::get().info("起飞完成");
				takeoff_completed = true;

			}
		}
	}

	if (takeoff_completed) {
		current_state_ = MissionState::Mission1InProgress;
	}

}

void MissionScheduler::handleMission1State() {
	// 1. 等待巡航末点（seq 达到 strike_handoff_seq）
	while (running_) {
		mavlink_mission_current_t current;
		if (mavlink_handler_->try_get_latest_mission_current(current)) {
			if (current.seq >= static_cast<uint16_t>(cfg_.fixedwing.strike_handoff_seq)) {
				AppLogger::get().info("到达巡航末点 seq={}，切 GUIDED 投弹", current.seq);
				break;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	if (!running_) return;

	// 2. 切 GUIDED
	mavlink_handler_->setFlightMode(static_cast<uint8_t>(cfg_.fixedwing.mode_guided));
	AppLogger::get().info("已发送进入引导模式指令 (mode={})", cfg_.fixedwing.mode_guided);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// 3. 启动控制回路
	if (motion_ctrl_) motion_ctrl_->start();

	const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
	const double g = 9.81;
	double fall_time = cfg_.fixedwing.drop_fall_time_s;
	if (fall_time <= 0.0) {
		fall_time = std::sqrt(2.0 * cfg_.fixedwing.drop_altitude / g);
	}

	// 引导飞越 + 释放（返回是否已释放）
	// 简化逻辑：飞到「Home→目标」延长线上的入口点，再沿直线笔直飞回 Home，
	// 目标恰好落在返航航迹上，无需按瞬时航迹角推算引导点。
	auto flyThroughAndRelease = [&](double tgt_lat, double tgt_lon) -> bool {
		// 1) Home→目标 的方位角
		double s_ht = 0.0, azi_ht = 0.0, azi_th = 0.0;
		geod.Inverse(home_lat_, home_lon_, tgt_lat, tgt_lon, s_ht, azi_ht, azi_th);
		// 2) 入口点 = 目标沿 Home→目标 方向延长 entry_extension_distance_m
		double entry_lat = 0.0, entry_lon = 0.0;
		geod.Direct(tgt_lat, tgt_lon, azi_ht, cfg_.fixedwing.entry_extension_distance_m, entry_lat, entry_lon);

		// 阶段一：飞向入口点，进入切换半径后切入返航直线
		if (motion_ctrl_) motion_ctrl_->setGpsPositionTarget(
			entry_lat, entry_lon, static_cast<float>(cfg_.fixedwing.drop_altitude), 0.0f);
		AppLogger::get().info("[投弹] 入口点 lat={:.7f} lon={:.7f}（沿 Home→目标 延长 {:.0f}m）",
			entry_lat, entry_lon, cfg_.fixedwing.entry_extension_distance_m);

		auto phase_start = std::chrono::steady_clock::now();
		while (running_) {
			mavlink_global_position_int_t gps;
			if (!mavlink_handler_->try_get_gps_data(gps)) {
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
				continue;
			}
			double d_entry = 0.0;
			geod.Inverse(gps.lat / 1e7, gps.lon / 1e7, entry_lat, entry_lon, d_entry);
			if (d_entry <= cfg_.fixedwing.entry_reach_radius_m) {
				break;
			}
			auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::steady_clock::now() - phase_start).count();
			// 固定翼从巡航末点飞到入口点通常 <20s，留足裕量；仅异常兜底，避免永久卡住
			if (elapsed_ms > 60000) {
				AppLogger::get().warn("[投弹] 未到达入口点，超时 60s，直接切入返航直线");
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}

		// 阶段二：沿直线笔直朝 Home 飞，途中在目标正上方（提前量 lead）释放
		if (motion_ctrl_) motion_ctrl_->setGpsPositionTarget(
			home_lat_, home_lon_, static_cast<float>(cfg_.fixedwing.drop_altitude), 0.0f);
		AppLogger::get().info("[投弹] 切入返航直线，目标 (lat={:.7f} lon={:.7f}) 位于航迹上", tgt_lat, tgt_lon);

		phase_start = std::chrono::steady_clock::now();
		while (running_) {
			mavlink_global_position_int_t gps;
			if (!mavlink_handler_->try_get_gps_data(gps)) {
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
				continue;
			}
			double drone_lat = gps.lat / 1e7;
			double drone_lon = gps.lon / 1e7;
			double ground_speed = std::hypot(static_cast<double>(gps.vx), static_cast<double>(gps.vy)) / 100.0; // cm/s -> m/s

			// 弹道提前量（沿返航直线，飞机到目标水平距离即沿航迹到目标距离）
			double lead = ground_speed * fall_time + cfg_.fixedwing.release_lead_offset_m;
			double d = 0.0;
			geod.Inverse(drone_lat, drone_lon, tgt_lat, tgt_lon, d);

			if (d <= lead) {
				mavlink_handler_->sendServoPWM(static_cast<uint8_t>(cfg_.fixedwing.servo_channel),
					static_cast<uint16_t>(cfg_.fixedwing.servo_release_pwm));
				AppLogger::get().info("[投弹] 释放舵机 ch{} pwm={} (d={:.2f}m lead={:.2f}m)",
					cfg_.fixedwing.servo_channel, cfg_.fixedwing.servo_release_pwm, d, lead);
				return true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		return false;
	};

	// 4. 视觉锁定目标（同时辅助侦察），并按竞赛规则选择打击目标
	const strike::StrikeTask strike_task =
		(cfg_.fixedwing.task_type == 2) ? strike::StrikeTask::Median : strike::StrikeTask::HighestValue;
	GPSTargetClusterer clusterer(camera_converter_);
	bool locked = false;
	double tgt_lat = 0.0, tgt_lon = 0.0;
	auto lock_start = std::chrono::steady_clock::now();

	while (running_) {
		auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - lock_start).count();
		if (elapsed_ms > cfg_.control.toudan_time_out) {
			AppLogger::get().warn("[投弹] 识别超时 ({}ms)，切换兜底投弹", cfg_.control.toudan_time_out);
			break;
		}

		vector<yoloout> dets;
		auto data = get_latest_yolo();
		if (data) dets = *data;

		mavlink_global_position_int_t gps;
		double lat = 0, lon = 0; float alt_m = 0;
		if (mavlink_handler_->try_get_gps_data(gps) && gps.relative_alt > 0) {
			lat = gps.lat / 1e7; lon = gps.lon / 1e7; alt_m = gps.relative_alt / 1000.0f;
		}
		mavlink_attitude_t att;
		double roll = 0, pitch = 0, yaw = 0;
		if (mavlink_handler_->try_get_imu_data(att)) {
			roll = att.roll; pitch = att.pitch; yaw = att.yaw;
		}
		clusterer.update(dets, lat, lon, alt_m, roll, pitch, yaw);

		auto stable = clusterer.getStableTargets();
		if (!stable.empty()) {
			// 构建候选目标并按规则选择（模型接线点：resolveTargetValue 填 value）
			std::vector<strike::TargetCandidate> candidates;
			candidates.reserve(stable.size());
			for (const auto& c : stable) {
				strike::TargetCandidate cand;
				cand.lat = c.lat;
				cand.lon = c.lon;
				cand.class_id = c.class_id;
				if (strike_task == strike::StrikeTask::HighestValue) {
					cand.value = resolveTargetValue(c.class_id, strike_task);
				} else {
					// 任务二（中位数）：用编号 OCR 读目标数字作为 value
					std::string digits = ocr::lookupDigit(c.avg_box);
					if (digits.empty()) {
						continue;  // 该目标还没读出编号，跳过，等下一轮
					}
					cand.value = std::stod(digits);
				}
				candidates.push_back(cand);
			}

			// 任务二需集齐 3 个靶标的编号才能取中位数；任务一有候选即可
			bool enough = (strike_task == strike::StrikeTask::HighestValue)
				? !candidates.empty()
				: (candidates.size() >= 3);

			if (enough) {
				strike::TargetSelector selector(strike_task);
				int idx = selector.select(candidates);
				if (idx >= 0) {
					tgt_lat = candidates[idx].lat;
					tgt_lon = candidates[idx].lon;
					locked = true;
					AppLogger::get().info("[投弹] 锁定目标 lat={:.7f} lon={:.7f} (候选数={} 选中={} 规则={})",
						tgt_lat, tgt_lon, candidates.size(), idx,
						strike_task == strike::StrikeTask::HighestValue ? "价值最高" : "中位数");
					break;
				}
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	// 5. 引导飞越 + 释放（锁定失败则走兜底点）
	bool released = false;
	if (locked) {
		released = flyThroughAndRelease(tgt_lat, tgt_lon);
	} else {
		double fb_lat, fb_lon;
		geod.Direct(home_lat_, home_lon_,
			takeoff_heading_ + cfg_.fixedwing.fallback_bearing_offset_deg,
			cfg_.fixedwing.fallback_distance_m, fb_lat, fb_lon);
		AppLogger::get().info("[投弹] 兜底投弹点 lat={:.7f} lon={:.7f}", fb_lat, fb_lon);
		released = flyThroughAndRelease(fb_lat, fb_lon);
	}

	// 6. 投弹后：跳降落段，切回 AUTO
	if (motion_ctrl_) motion_ctrl_->stop();
	mavlink_handler_->setMissionCurrent(static_cast<uint16_t>(cfg_.fixedwing.landing_start_seq));
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	mavlink_handler_->setFlightMode(static_cast<uint8_t>(cfg_.fixedwing.mode_auto));
	AppLogger::get().info("[投弹] 完成 (released={})，跳降落段 seq={} 切 AUTO",
		released, cfg_.fixedwing.landing_start_seq);
	current_state_ = MissionState::LandingInProgress;
}

void MissionScheduler::handleLandingState() {
	// 监控降落：seq 到达最后一条 NAV_LAND，或高度 < 1m，或超时
	const int LAND_TIMEOUT_MS = 60000;
	int last_seq = 1 + static_cast<int>(cfg_.fixedwing.route.size()); // HOME=0, TAKEOFF=1, route[0]=2...
	auto land_start = std::chrono::steady_clock::now();

	while (running_) {
		mavlink_mission_current_t current;
		if (mavlink_handler_->try_get_latest_mission_current(current)) {
			if (current.seq >= last_seq) {
				AppLogger::get().info("[降落] 到达最后航点 seq={}", current.seq);
				break;
			}
		}
		mavlink_global_position_int_t gps;
		if (mavlink_handler_->try_get_gps_data(gps)) {
			if (gps.relative_alt >= 0 && gps.relative_alt / 1000.0f < 1.0) {
				AppLogger::get().info("[降落] 高度 < 1m，判定落地");
				break;
			}
		}
		auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - land_start).count();
		if (elapsed_ms > LAND_TIMEOUT_MS) {
			AppLogger::get().warn("[降落] 监控超时，退出");
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	if (motion_ctrl_) motion_ctrl_->stop();
	AppLogger::get().info("任务完成，调度器退出");
	stop();
}

bool try_get_latest_image(Mat& latest_image) {//
	auto frame_ptr = frameDispatcher.getFrame();

	if (frame_ptr && !frame_ptr->empty()) {
		const cv::Mat& frame = *frame_ptr;
		latest_image = frame.clone();
		return true;
	}
	return false;
}
