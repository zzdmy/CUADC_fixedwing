//main.cpp
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif
#include "AppLogger.h"
#include <vector>
#include <string>

#include "MavlinkProtocolHandler.h"
#include "SerialTransport.h"
#include "TcpTransport.h"
#include "UdpTransport.h"
#include <thread>//线程头文件
#include <atomic>//用于原子操作
#include <mutex>
#include "FrameDispatcher.h"//帧分发头文件
#include "CaptureThread.h"//帧捕获头文件
#ifdef _WIN32
#include "WinDeviceEnumerator.h"
using DeviceEnumeratorImpl = WinDeviceEnumerator;
#else
#include "LinuxDeviceEnumerator.h"
using DeviceEnumeratorImpl = LinuxDeviceEnumerator;
#endif
#include "TensorRTDetector.h"//tensorrt检测方法头文件
#include "OpenCVDNNDetector.h"//opencv检测方法头文件
#include "IDetector.h"//统一检测接口
#include "yolov8_trt_infer.hpp"//YoloV8TensorRT 类定义（任务一第二级图案检测器）

#include <boost/asio/io_context.hpp>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <utility>
#include "MissionScheduler.h"
#include "PixelToGPSConverter.h"
#include "rtk/NtripClient.h"
//#include "yolodecet/yolo_tracker.h"
#include <common/mavlink_msg_attitude.h>
#include <mavlink_types.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include "ConfigManager.h"//
#include "ocr/OcrDigitReader.h"
#include <fstream>
#include <csignal>
#include <filesystem>

using namespace std;
using namespace cv;
cv::AccessFlag access_mask = cv::ACCESS_MASK;//避免accessmask不明确
int actualWidth, actualHeight, actualFPS;// 全局变量定义分辨率
cv::Mat g_cameraMatrix, g_distCoeffs;
double movex = 0, movey = 0;//初始化x和y移动速度xc

// 主线程与显示线程共享的状态数据
struct SharedStatus {
	std::mutex mtx;
	std::string text;
	int color_code = 0; // 0=白 1=黄 2=红
};

// 显示线程：从 FrameDispatcher 取帧并绘制
void displayThreadFunc(std::stop_token st,
	std::shared_ptr<MavlinkProtocolHandler> mavlink_handler,
	std::shared_ptr<std::atomic<bool>> running_flag,
	std::shared_ptr<SharedStatus> shared_status) {
	*running_flag = true;

	try {
		cv::namedWindow("Video", 0.25);

		while (!st.stop_requested()) {
			mavlink_attitude_t imu_data;
			float yaw = 0.0f;
			if (mavlink_handler->try_get_imu_data(imu_data)) {
				yaw = (float)imu_data.yaw * 180.00000f / 3.1415926f;
			}

			auto frame_ptr = frameDispatcher.getFrame();
			if (frame_ptr && !frame_ptr->empty()) {
				cv::Mat display = frame_ptr->clone();
				drawCompassOnImage(display, yaw);

				// 绘制顶部状态栏（半透明黑底 + 白色文字）
				const int bar_height = 22;
				std::string status_text;
				int color_code = 0;
				{
					std::lock_guard<std::mutex> lock(shared_status->mtx);
					status_text = shared_status->text;
					color_code = shared_status->color_code;
				}
				if (!status_text.empty()) {
					cv::Mat roi = display(cv::Rect(0, 0, display.cols, bar_height));
					cv::addWeighted(roi, 0.4, cv::Mat::zeros(roi.size(), roi.type()), 0.6, 0, roi);
					double font_scale = (display.cols >= 1280) ? 0.45 : 0.4;
					cv::Scalar text_color;
					switch (color_code) {
					case 1: text_color = cv::Scalar(0, 255, 255); break;
					case 2: text_color = cv::Scalar(0, 0, 255); break;
					default: text_color = cv::Scalar(255, 255, 255); break;
					}
					cv::putText(display, status_text, cv::Point(5, bar_height - 5),
						cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, 1);
				}

				cv::imshow("Video", display);
			}

			cv::waitKey(1);
		}

		cv::destroyWindow("Video");
	}
	catch (...) {
		*running_flag = false;
		throw;
	}
	*running_flag = false;
}

// ===== 机内录像（素材收集；config.record.enable 控制）=====
// 从 frameDispatcher 取帧写 H.264 MP4（GStreamer x264），与识别/预览共用同一路画面，
// 不额外占用相机。收到 SIGTERM/SIGINT 时先关闭当前分段再放行进程退出，保住 MP4 索引。
static volatile std::sig_atomic_t g_termSig = 0;
static void onTermSignal(int sig) { g_termSig = sig; }

static void recCleanupOld(const std::string& dir, double max_gb, const std::string& curPath) {
    std::error_code ec;
    namespace fs = std::filesystem;
    const uint64_t maxB = static_cast<uint64_t>(max_gb * 1024.0 * 1024.0 * 1024.0);
    uint64_t total = 0;
    std::vector<std::pair<fs::file_time_type, fs::path>> files;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        const auto ext = e.path().extension().string();
        if (ext != ".mp4" && ext != ".avi") continue;
        total += fs::file_size(e.path(), ec);
        files.emplace_back(fs::last_write_time(e.path(), ec), e.path());
    }
    if (total <= maxB) return;
    std::sort(files.begin(), files.end());
    for (auto& [ft, p] : files) {
        if (total <= maxB) break;
        if (p.string() == curPath) continue;
        const uint64_t sz = fs::file_size(p, ec);
        if (fs::remove(p, ec)) {
            total -= sz;
            AppLogger::get().info("[record] 清理旧录像: {}", p.filename().string());
        }
    }
}

void recordThreadFunc(std::stop_token st, RecordConfig rc) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(rc.dir, ec);
    AppLogger::get().info("[record] 机内录像线程启动: 目录={} 分段={}s 上限={}GB",
        rc.dir, rc.segment_sec, rc.max_total_gb);

    auto segStart = std::chrono::steady_clock::now();
    auto lastWrite = std::chrono::steady_clock::now();
    uint64_t lastFid = 0;
    cv::VideoWriter writer;
    std::string curPath;

    while (!st.stop_requested()) {
        // 终止信号：先关分段（写 MP4 索引），再按默认行为退出
        if (g_termSig) {
            if (writer.isOpened()) {
                writer.release();
                AppLogger::get().info("[record] 收到终止信号，当前分段已关闭");
            }
            std::signal(g_termSig, SIG_DFL);
            std::raise(g_termSig);
        }

        auto fp = frameDispatcher.getFrame();
        const uint64_t fid = frameDispatcher.getFrameCounter();
        if (fp && !fp->empty() && fid != lastFid) {
            lastFid = fid;
            const auto now = std::chrono::steady_clock::now();
            if (now - lastWrite >= std::chrono::milliseconds(33)) {   // 限速到 ~30fps
                lastWrite = now;
                if (!writer.isOpened() || now - segStart > std::chrono::seconds(rc.segment_sec)) {
                    if (writer.isOpened()) writer.release();
                    std::time_t t = std::time(nullptr);
                    std::tm tm{};
#ifdef _WIN32
                    localtime_s(&tm, &t);
#else
                    localtime_r(&t, &tm);
#endif
                    char base[64];
                    std::strftime(base, sizeof(base), "flt_%Y-%m-%d_%H-%M-%S", &tm);
                    curPath = rc.dir + "/" + base + ".mp4";
                    for (int i = 1; i < 100 && fs::exists(curPath, ec); ++i)
                        curPath = rc.dir + "/" + base + "_" + std::to_string(i) + ".mp4";

                    const std::string pipe =
                        "appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc bitrate=6000 speed-preset=veryfast key-int-max=60 "
                        "! h264parse ! qtmux ! filesink location=" + curPath;
                    bool opened = writer.open(pipe, cv::CAP_GSTREAMER, 0, 30, fp->size());
#ifdef _WIN32
                    if (!opened)
                        opened = writer.open(curPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, fp->size());
#endif
                    if (opened) {
                        segStart = now;
                        recCleanupOld(rc.dir, rc.max_total_gb, curPath);
                        AppLogger::get().info("[record] 新分段: {}", curPath);
                    } else {
                        AppLogger::get().error("[record] 分段创建失败: {}", curPath);
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                    }
                }
                if (writer.isOpened()) writer.write(*fp);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (writer.isOpened()) writer.release();
    AppLogger::get().info("[record] 机内录像线程退出，当前分段已关闭");
}

int main(int argc, char** argv)
{
	try
	{
#ifdef _WIN32
		SetConsoleOutputCP(CP_UTF8); // 控制台输出使用 UTF-8，避免中文日志乱码
		SetConsoleCP(CP_UTF8);
#endif
		AppConfig config = load_config_from_yaml(); //加载配置文件
		ConfigManager::getInstance().setConfig(config);//设置配置文件到全局
		AppLogger::get().info("配置文件加载完成, 通信方式: {}", config.communication.transport);
		std::vector<std::thread> threadList;//建立线程列

		auto enumerator = std::make_unique<DeviceEnumeratorImpl>();//创建设备枚举器


		// 查找摄像头
		int camIndex = -1;
		if (auto idx = enumerator->findCameraByVidPid("345F", "2130")) {
			camIndex = *idx;
		}
		else if (auto idx = enumerator->findCameraByVidPid("534d", "2109")) {
			camIndex = *idx;
		}
		else {
			auto cams = enumerator->enumerateCameras();
			if (!cams.empty()) {
				AppLogger::get().info("未找到采集卡设备，使用第一个可用摄像头: {}", cams[0].friendlyName);
				camIndex = cams[0].index;
			}
		}

		if (camIndex < 0) {
			AppLogger::get().info("VID/PID 枚举失败，使用配置文件 camera.index: {}", config.camera.index);
			camIndex = config.camera.index;
		}


		// 创建并启动捕获线程（使用 YAML 配置的相机参数）
		CaptureThread::Config camConfig;
		camConfig.width = config.camera.width;
		camConfig.height = config.camera.height;
		camConfig.fps = config.camera.fps;
		// AE 参数
		camConfig.ae_enable = config.camera.ae.enable;
		camConfig.ae_target_brightness = config.camera.ae.target_brightness;
		camConfig.ae_exposure_min = config.camera.ae.exposure_min;
		camConfig.ae_exposure_max = config.camera.ae.exposure_max;
		camConfig.ae_kp = config.camera.ae.kp;
		camConfig.ae_ki = config.camera.ae.ki;
		camConfig.ae_update_interval = config.camera.ae.update_interval;
		camConfig.ae_deadband = config.camera.ae.deadband;
		CaptureThread captureThread(camIndex, camConfig);
		try {
			captureThread.start();
			AppLogger::get().info("捕获线程已启动 ({}x{}@{}fps)", actualWidth, actualHeight, actualFPS);
		}
		catch (const std::exception& e) {
			AppLogger::get().error("捕获线程启动失败: {}", e.what());
			return -1;
		}
		// 配置控制参数
		const int FRAME_CENTER_X = actualWidth / 2;   // 实际宽度的一半
		const int FRAME_CENTER_Y = actualHeight / 2;  // 实际高度的一半
		int actualWidth_int = actualWidth;
		int actualHeight_int = actualHeight;
		// 初始化发送msg
		mavlink_message_t msg;

		// ===== 初始化飞控通信 =====
		boost::asio::io_context io;

		// === 选择一种连接方式 ===

		  // 创建传输层
        std::shared_ptr<ITransport> transport;
        if (config.communication.transport == "serial") {
			// 查找串口
			if (auto port = enumerator->findComPortByVidPid("10C4", "EA60")) {
				AppLogger::get().info("串口已找到: {}", *port);
				transport = std::make_shared<SerialTransport>(
					io, *port, config.communication.serial.baud_rate
				);
			}
			else {
				AppLogger::get().info("未找到串口设备,将使用配置文件中串口: {}", config.communication.serial.port);
				transport = std::make_shared<SerialTransport>(
					io, config.communication.serial.port, config.communication.serial.baud_rate
				);
			}


        } else if (config.communication.transport == "tcp") {
            transport = std::make_shared<TcpTransport>(
                io, config.communication.tcp.host, config.communication.tcp.port
            );
        }
		else if (config.communication.transport == "udp") {
			transport = std::make_shared<UdpTransport>(
				io, config.communication.udp.host, config.communication.udp.port);
		}

		// ===== 创建 MAVLink 协议处理器 =====
		auto mavlink_handler = std::make_shared<MavlinkProtocolHandler>(config);
		mavlink_handler->set_transport(transport);//设置传输方式
		mavlink_handler->start(); // 启动接收回调
		AppLogger::get().info("MAVLink处理器已启动 (system_id={}, transport={})", config.mavlink.system_id, config.communication.transport);

		// ===== 启动 IO 线程 =====
		std::thread mavlink_thread([&io]() {
			AppLogger::get().info("连接建立，IO 线程运行中...");
			io.run(); // 驱动所有异步操作（串口/TCP/UDP 收发）
			});

		// 将线程加入管理列表
		threadList.push_back(std::move(mavlink_thread));

		// ===== 提前请求 GPS 数据流，保证 RTK/Mission 启动时 GPS 已就绪 =====
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		mavlink_handler->requestGpsDataStreams();

		auto camera_converter = std::make_shared<PixelToGPSConverter>();//创建像素到GPS转换器
		camera_converter->configureFromConfig();//从配置文件中读取相机参数
		AppLogger::get().info("相机参数加载完成");
		MissionScheduler missionScheduler(mavlink_handler, camera_converter);//创建任务调度器

		missionScheduler.start();//启动任务调度器
		AppLogger::get().info("启动任务调度器完成");
		// ===== 启动 RTK 网络服务 =====
		std::unique_ptr<NtripClient> rtkClient;
		if (config.rtk.enabled) {
			rtkClient = std::make_unique<NtripClient>(config.rtk, mavlink_handler.get());
			rtkClient->start();
			AppLogger::get().info("RTK客户端已启动, NTRIP服务器: {}:{}", config.rtk.ntrip_host, config.rtk.ntrip_port);
		}

		// 主循环只负责显示和控制
	auto last_fps_display = std::chrono::steady_clock::now();
	auto last_display_update = std::chrono::steady_clock::now();



    // ===== 检测器接线（两段式：先检天井 → 再检图案/数字）=====
    // 主检测器 = 天井（两轮比赛共用，单类 bucket，imgsz=1280）
    const std::string well_model_path  = config.yolo.well_model_path;
    const std::string well_engine_path = config.yolo.well_engine_path;
    const std::string well_class_path  = config.yolo.well_class_path;

    // 任务一：额外加载图案检测器（12 类，只做第二级裁剪识别，不起独立检测 loop）
    std::shared_ptr<YoloV8TensorRT> patternDetector;
    if (config.fixedwing.task_type == 1) {
        patternDetector = std::make_shared<YoloV8TensorRT>(
            config.yolo.pattern_engine_path, config.yolo.pattern_class_path);
        if (!patternDetector->isInitialized()) {
            AppLogger::get().error("图案检测器加载失败: {}", config.yolo.pattern_engine_path);
            return -1;
        }
        AppLogger::get().info("图案检测器已加载（任务一第二级）: {}", config.yolo.pattern_engine_path);
        missionScheduler.setPatternDetector(patternDetector);   // 注入任务调度器做第二级识别
    }

    std::unique_ptr<IDetector> detector;
    if (config.yolo.backend=="tensorrt") {

        detector = std::make_unique<TensorRTDetector>(
            well_engine_path, well_class_path, config.yolo.detect_frame_stride);//创建天井tensorrt检测器（含抽帧）
    } else{
        detector = std::make_unique<OpenCVDNNDetector>(well_model_path, well_class_path, config.yolo.use_gpu);//创建opencv检测器
    }

    if (!detector || !detector->isInitialized()) {
        AppLogger::get().error("Detector initialization failed!");
        return -1;
    }

    AppLogger::get().info("Detector initialized successfully.");

    // 启动检测线程
    detector->startDetectionLoop(actualWidth, actualHeight);
    AppLogger::get().info("天井检测线程已启动, 后端: {}, 引擎: {}, 抽帧: 每{}帧",
        config.yolo.backend, well_engine_path, config.yolo.detect_frame_stride);

	// 启动显示线程
	auto display_running = std::make_shared<std::atomic<bool>>(false);
	auto shared_status = std::make_shared<SharedStatus>();
	std::jthread displayThread;
	if (config.control.enable_display) {
		displayThread = std::jthread(displayThreadFunc, mavlink_handler, display_running, shared_status);
		AppLogger::get().info("显示线程已启动");
	} else {
		AppLogger::get().info("显示线程已禁用（enable_display=false，机载无屏模式）");
	}

	// ===== 启动机内录像（素材收集；比赛时把 config 的 record.enable 设为 false）=====
	std::jthread recordThread;
	if (config.record.enable) {
		std::signal(SIGTERM, onTermSignal);
		std::signal(SIGINT, onTermSignal);
		recordThread = std::jthread(recordThreadFunc, config.record);
		AppLogger::get().info("机内录像已启用: 目录={} 分段={}s", config.record.dir, config.record.segment_sec);
	} else {
		AppLogger::get().info("机内录像未启用（record.enable=false）");
	}

	// ===== 启动编号 OCR 识别（PP-OCRv5_mobile_rec + TensorRT，按需触发）=====
	std::unique_ptr<ocr::OcrDigitReader> ocrReader;
	if (config.ocr.enabled && config.fixedwing.task_type == 2) {
		// 前置检查：缺文件时给出可执行的提示，避免 TensorRT 抛出难懂的错
		auto fileOk = [](const std::string& p) {
			std::ifstream f(p, std::ios::binary);
			return f.good();
		};
		const bool hasEngine = fileOk(config.ocr.engine_path);
		const bool hasOnnx = fileOk(config.ocr.onnx_path);
		const bool hasDict = fileOk(config.ocr.dict_path);
		const bool hasYml = fileOk(config.ocr.inference_yml_path);

		if (!hasEngine && !hasOnnx) {
			AppLogger::get().error(
				"编号 OCR 已启用但缺少模型: 既没有 {} 也没有 {}。"
				"请先运行 setup_ocr_model.ps1 下载模型。", config.ocr.engine_path, config.ocr.onnx_path);
		}
		else if (!hasDict && !hasYml) {
			AppLogger::get().error(
				"编号 OCR 缺少字符字典: {} 与 {} 都不存在。"
				"请先运行 setup_ocr_model.ps1。", config.ocr.dict_path, config.ocr.inference_yml_path);
		}
		else if (!hasEngine) {
			AppLogger::get().info("编号 OCR 首次运行，将从 ONNX 构建 TensorRT 引擎（实测约 10~15 分钟，仅一次）...");
		}

		ocr::OcrPipelineConfig pipelineCfg;
		// --- rec（PP-OCRv6_medium_rec）---
		pipelineCfg.rec.onnxPath = config.ocr.onnx_path;
		pipelineCfg.rec.enginePath = config.ocr.engine_path;
		pipelineCfg.rec.dictPath = config.ocr.dict_path;
		pipelineCfg.rec.inferenceYmlPath = config.ocr.inference_yml_path;
		pipelineCfg.rec.recMaxWidth = config.ocr.rec_max_width;
		pipelineCfg.rec.recOptWidth = config.ocr.rec_max_width;
		pipelineCfg.rec.useFp16 = config.ocr.use_fp16;
		pipelineCfg.rec.digitOnly = config.ocr.digit_only;
		// --- det（PP-OCRv6_medium_det / DB）---
		pipelineCfg.useDet = config.ocr.use_det;
		pipelineCfg.det.onnxPath = config.ocr.det_onnx_path;
		pipelineCfg.det.enginePath = config.ocr.det_engine_path;
		pipelineCfg.det.limitSide = config.ocr.det_limit_side;
		pipelineCfg.det.boxThresh = config.ocr.det_box_thresh;
		pipelineCfg.det.thresh = config.ocr.det_thresh;
		pipelineCfg.det.unclipRatio = config.ocr.det_unclip_ratio;
		pipelineCfg.det.useFp16 = config.ocr.use_fp16;
		// --- ori（PP-LCNet_x1_0_textline_ori）---
		pipelineCfg.useOri = config.ocr.use_ori;
		pipelineCfg.ori.onnxPath = config.ocr.ori_onnx_path;
		pipelineCfg.ori.enginePath = config.ocr.ori_engine_path;
		pipelineCfg.ori.useFp16 = config.ocr.use_fp16;
		pipelineCfg.cropPadding = 0.0f; // det 框已外扩，管线内不再额外扩

		ocr::OcrConfig ocrCfg;
		ocrCfg.enabled = true;
		ocrCfg.startEnabled = false;
		ocrCfg.minBoxWidth = config.ocr.min_box_width;
		ocrCfg.minBoxHeight = config.ocr.min_box_height;
		ocrCfg.maxBoxesPerCycle = config.ocr.max_boxes_per_cycle;
		ocrCfg.minInferIntervalMs = config.ocr.min_infer_interval_ms;
		ocrCfg.cropPadding = config.ocr.crop_padding;
		ocrCfg.voteWindow = config.ocr.vote_window;
		ocrCfg.minVotes = config.ocr.min_votes;
		ocrCfg.entryTtlMs = config.ocr.entry_ttl_ms;
		// 触发场景：按任务阶段决定何时开启识别
		if (config.ocr.enable_on_bomb)  ocrCfg.triggers.push_back("bomb");
		if (config.ocr.enable_on_recon) ocrCfg.triggers.push_back("recon");
		// 任务二：只对天井类(bucket, class_id=0)做 OCR（天井单类模型的 class_id=0）
		if (config.fixedwing.task_type == 2) {
			ocrCfg.keepClassIds = { 0 };
		}

		ocrReader = std::make_unique<ocr::OcrDigitReader>(ocrCfg, pipelineCfg);
		if (ocrReader->isReady()) {
			ocr::setGlobalReader(ocrReader.get());   // 注册全局读取器，供 MissionScheduler 查编号
			ocrReader->start();
			AppLogger::get().info("编号 OCR 已就绪: 触发场景={}",
				ocrCfg.triggers.empty() ? std::string("全部") :
				(ocrCfg.triggers.size() == 2 ? std::string("bomb,recon") : ocrCfg.triggers[0]));
		} else {
			AppLogger::get().warn("编号 OCR 初始化失败（缺少模型或引擎构建失败），将不影响其他功能");
			ocrReader.reset();
		}
	} else {
		AppLogger::get().info("编号 OCR 已在配置中关闭");
	}

	while (true) {//主线程：调度与监控
		// ===== OCR 按需触发：仅在投弹阶段开启识别 =====
		if (ocrReader) {
			const char* ocr_phase = "init";
			switch (missionScheduler.getCurrentState()) {
			case MissionState::Mission1InProgress: ocr_phase = "bomb"; break;
			default: break;
			}
			const bool want = ocr::isPhaseTriggered(ocrReader->triggers(), ocr_phase);
			if (want != ocrReader->isEnabled()) {
				ocrReader->setEnabled(want);
				AppLogger::get().info("编号 OCR {} (阶段: {})", want ? "已开启" : "已关闭", ocr_phase);
			}
		}

		auto now = std::chrono::steady_clock::now();

		// 构建状态字符串（英文，供画面显示）
		auto build_status = [&](int& color) {
			std::string s;
			int worst = 0; // 0=白 1=黄 2=红
			char buf[32];
			auto hb_age = mavlink_handler->getHeartbeatAgeMs();
			uint8_t ap = 0, vt = 0, ss = 0;
			if (mavlink_handler->try_get_heartbeat_info(ap, vt, ss)) {
				auto rate = mavlink_handler->getHeartbeatRate();
				if (rate > 0) { snprintf(buf, sizeof(buf), "%.1fHz ", rate); s += buf; }
				switch (ap) {
				case MAV_AUTOPILOT_PX4: s += "PX4"; break;
				case MAV_AUTOPILOT_ARDUPILOTMEGA: s += "Ardu"; break;
				default: s += "FC?"; break;
				}
				switch (vt) {
				case MAV_TYPE_QUADROTOR: s += "/Quad"; break;
				case MAV_TYPE_HEXAROTOR: s += "/Hexa"; break;
				case MAV_TYPE_OCTOROTOR: s += "/Octo"; break;
				case MAV_TYPE_FIXED_WING: s += "/Wing"; break;
				default: s += "/?"; break;
				}
				switch (ss) {
				case MAV_STATE_UNINIT: s += "/Uninit"; break;
				case MAV_STATE_BOOT: s += "/Boot"; break;
				case MAV_STATE_CALIBRATING: s += "/Calib"; break;
				case MAV_STATE_STANDBY: s += "/Standby"; break;
				case MAV_STATE_ACTIVE: s += "/Active"; break;
				case MAV_STATE_CRITICAL: s += "/CRIT"; break;
				case MAV_STATE_EMERGENCY: s += "/EMRG"; break;
				default: s += "/?"; break;
				}
				FlightModeData fm;
				if (mavlink_handler->try_get_flight_mode(fm)) {
					s += "/";
					if (ap == MAV_AUTOPILOT_PX4) {
						switch ((fm.custom_mode >> 16) & 0xFF) {
						case 1: s += "MANUAL"; break;
						case 2: s += "ALTCTL"; break;
						case 3: s += "POSCTL"; break;
						case 4: s += "AUTO"; break;
						case 5: s += "ACRO"; break;
						case 6: s += "OFFBOARD"; break;
						case 7: s += "STABILIZED"; break;
						case 8: s += "RATTITUDE"; break;
						default: s += "M" + std::to_string((fm.custom_mode >> 16) & 0xFF); break;
						}
					} else if (ap == MAV_AUTOPILOT_ARDUPILOTMEGA) {
						switch (fm.custom_mode) {
						case 0: s += "Manual"; break;
						case 3: s += "Stabilize"; break;
						case 6: s += "FBWA"; break;
						case 8: s += "Cruise"; break;
						case 10: s += "Auto"; break;
						case 11: s += "RTL"; break;
						case 12: s += "Loiter"; break;
						case 15: s += "Guided"; break;
						default: s += "M" + std::to_string(fm.custom_mode); break;
						}
					} else {
						s += "M" + std::to_string(fm.custom_mode);
					}
				}
			} else {
				s += "HB";
			}
			if (hb_age > 5000) { s += " LOST!"; worst = 2; }
			else if (hb_age > 2000) { s += " LOST?"; if (worst < 1) worst = 1; }

			// 离地高度(mm)
			mavlink_global_position_int_t gpos;
			if (mavlink_handler->try_get_gps_data(gpos) && gpos.relative_alt > 0) {
				snprintf(buf, sizeof(buf), " %dmm", gpos.relative_alt);
				s += buf;
			}

			mavlink_gps_raw_int_t gps_raw{};
			if (mavlink_handler->try_get_gps_raw_data(gps_raw)) {
				s += " | GPS";
				switch (gps_raw.fix_type) {
				case 0: s += "NoGPS"; break;
				case 1: s += "NoFix"; break;
				case 2: s += "2D"; break;
				case 3: s += "3D"; break;
				case 4: s += "DGPS"; break;
				case 5: s += "RTK Float"; break;
				case 6: s += "RTK Fixed"; break;
				default: s += "?"; break;
				}
				if (gps_raw.fix_type < 2) worst = 2;
				else if (gps_raw.fix_type < 3 && worst < 1) worst = 1;
				s += "/" + std::to_string(gps_raw.satellites_visible) + "s";
				if (gps_raw.eph > 0) {
					snprintf(buf, sizeof(buf), "/HDOP%.1f", gps_raw.eph / 100.0f);
					s += buf;
				}
			} else {
				s += " | NoGPS";
			}

			if (rtkClient) {
				s += " | RTK";
				switch (rtkClient->getState()) {
				case NtripState::WaitingForGps: s += "WaitGPS"; break;
				case NtripState::Connecting: s += "Connecting"; break;
				case NtripState::Connected: s += "OK"; break;
				case NtripState::Reconnecting: s += "Reconn"; if (worst < 1) worst = 1; break;
				case NtripState::MaxRetriesExceeded: s += "Fail"; worst = 2; break;
				case NtripState::Stopped: s += "Stopped"; break;
				case NtripState::Error: s += "Error"; worst = 2; break;
				}
			}

			s += " | " + std::to_string(static_cast<int>(captureThread.getCaptureFPS())) + "fps";
			s += "/" + std::string(detector->isRunning() ? "DET" : "DEToff");
			s += "/" + std::string(display_running->load() ? "DISP" : "DISPoff");
			s += "/";
			switch (missionScheduler.getCurrentState()) {
			case MissionState::WaitingForInitialization: s += "Init"; break;
			case MissionState::TakingOff: s += "Takeoff"; break;
			case MissionState::Mission1InProgress: s += "Bomb"; break;
			case MissionState::LandingInProgress: s += "Land"; break;
			}
			color = worst;
			return s;
		};

		// 更新显示状态（每秒）
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_display_update).count() >= 1000) {
			int cc = 0;
			auto status = build_status(cc);
			{
				std::lock_guard<std::mutex> lock(shared_status->mtx);
				shared_status->text = std::move(status);
				shared_status->color_code = cc;
			}
			last_display_update = now;
		}

		// 记录日志（每5秒，中文）
		if (std::chrono::duration_cast<std::chrono::seconds>(now - last_fps_display).count() >= 5) {
			std::string log;
			char logbuf[32];
			auto hb_age = mavlink_handler->getHeartbeatAgeMs();
			uint8_t ap = 0, vt = 0, ss = 0;
			if (mavlink_handler->try_get_heartbeat_info(ap, vt, ss)) {
				auto rate = mavlink_handler->getHeartbeatRate();
				log += "心跳(";
				if (rate > 0) { snprintf(logbuf, sizeof(logbuf), "%.1fHz/", rate); log += logbuf; }
				log += MavlinkProtocolHandler::autopilotToString(ap);
				log += "/";
				log += MavlinkProtocolHandler::vehicleTypeToString(vt);
				log += "/";
				log += MavlinkProtocolHandler::systemStatusToString(ss);
				FlightModeData fm;
				if (mavlink_handler->try_get_flight_mode(fm)) {
					log += "/";
					log += MavlinkProtocolHandler::flightModeToString(ap, vt, fm.custom_mode);
				}
				log += ") ";
			} else {
				log += "心跳(等待中) ";
			}
			if (hb_age > 3000) log += "断联! ";

			mavlink_gps_raw_int_t gps_raw{};
			if (mavlink_handler->try_get_gps_raw_data(gps_raw)) {
				log += "GPS(";
				log += MavlinkProtocolHandler::gpsFixTypeToString(gps_raw.fix_type);
				log += "/" + std::to_string(gps_raw.satellites_visible) + "星";
				if (gps_raw.eph > 0) {
					snprintf(logbuf, sizeof(logbuf), "/HDOP=%.1f", gps_raw.eph / 100.0f);
					log += logbuf;
				}
				log += ") ";
			} else {
				log += "GPS(无数据) ";
			}

			if (rtkClient) {
				auto rtk_state = rtkClient->getState();
				auto& rtk_s = rtkClient->getStats();
				log += "RTK(";
				switch (rtk_state) {
				case NtripState::WaitingForGps: log += "待GPS"; break;
				case NtripState::Connecting: log += "连接中"; break;
				case NtripState::Connected: log += "已连接"; break;
				case NtripState::Reconnecting: log += "重连"; break;
				case NtripState::MaxRetriesExceeded: log += "重连耗尽"; break;
				case NtripState::Stopped: log += "已停止"; break;
				case NtripState::Error: log += "错误"; break;
				}
				log += "/" + std::to_string(rtk_s.valid_frames.load()) + "帧) ";
			}

			log += "线程(";
			log += "捕获" + std::to_string(static_cast<int>(captureThread.getCaptureFPS())) + "fps";
			log += "/检测" + std::string(detector->isRunning() ? "运行中" : "停止");
			log += "/显示" + std::string(display_running->load() ? "运行中" : "停止");
			log += "/任务";
			switch (missionScheduler.getCurrentState()) {
			case MissionState::WaitingForInitialization: log += "待初始化"; break;
			case MissionState::TakingOff: log += "起飞中"; break;
			case MissionState::Mission1InProgress: log += "投弹任务"; break;
			case MissionState::LandingInProgress: log += "降落监控"; break;
			}
			log += ")";

			if (ocrReader) {
				auto os = ocrReader->getStats();
				log += " OCR(";
				log += ocrReader->isEnabled() ? "开/" : "待触发/";
				log += std::to_string(os.inferences) + "次推理";
				log += "/命中" + std::to_string(os.hits);
				char ocrbuf[32];
				snprintf(ocrbuf, sizeof(ocrbuf), "/%.0fms", os.lastCostMs);
				log += ocrbuf;
				log += ")";
			}

			AppLogger::get().info("系统状态: {}", log);
			last_fps_display = now;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

		// 等待子线程完成
		// 当你需要等待所有线程完成时
		for (auto& th : threadList) {
			if (th.joinable()) {
				th.join();
			}
		}
		missionScheduler.stop();//停止任务调度器
	}
	catch (const std::exception& e) {
		AppLogger::get().error("发生错误：{}", e.what());
	}
	return 0;
}
