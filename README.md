# CUADC_fixedwing

一个基于 C++ 的固定翼无人机侦察与打击项目，集成了 MAVLink 通信、相机采集、目标检测（YOLO）、目标追踪、视觉伺服、RTK 差分定位和任务调度能力。面向 Windows + Visual Studio 环境，适合用于飞控联调、视觉识别投放和航点任务实验。

## 功能概览

- **通信**：支持 `serial`、`tcp`、`udp` 三种方式连接飞控（Boost.Asio 抽象传输层）
- **MAVLink 交互**：飞行模式切换、状态读取、航点上传、舵机 PWM 控制、GPS_RTCM_DATA 注入
- **相机采集**：设备自动枚举（VID/PID + DirectShow）+ 独立采集线程 + 帧分发
- **目标检测**：支持 TensorRT 引擎推理 或 OpenCV DNN 推理（YOLO）
- **目标追踪**：GPS 聚类目标关联（GPSTargetClusterer）、像素坐标 → GPS 坐标转换（PixelToGPSConverter）、OpenCV CSRT 追踪
- **视觉伺服**：基于像素误差的比例速度控制，分阶段对准（水平 → 高度 → 稳定 → 投放）
- **任务调度**：多状态状态机（WaitingForInitialization → TakingOff → Mission1InProgress → Mission2InProgress），独立线程运行
- **曝光控制**：软件 AE，PI 调节亮度，可配置目标亮度 / 曝光限幅 / 更新间隔
- **RTK 差分定位**：NTRIP 客户端自动连接、RTCM v3 接收、CRC24Q 校验、GPS_RTCM_DATA 注入飞控

## 项目结构

- `main.cpp` — 程序入口，初始化配置、通信、检测器、采集线程和主显示循环
- `MissionScheduler.*` — 任务调度核心状态机，负责航点任务、视觉对准、投放控制与后处理
- `MavlinkProtocolHandler.*` — MAVLink 消息收发、飞行模式切换、航点上传、GPS/IMU 数据缓存
- `MotionController.*` — 15Hz 控制循环，支持 POSITION / VELOCITY / VISUAL_SERVO 三种模式
- `CaptureThread.*` / `FrameDispatcher.*` — 视频采集与帧分发
- `TensorRTDetector.*` / `OpenCVDNNDetector.*` — 检测器实现
- `GPSTargetClusterer.*` — GPS 坐标聚类匹配与稳定目标判定
- `PixelToGPSConverter.*` — 像素坐标到 GPS 坐标的几何转换（基于相机内参 + 姿态 + 高度）
- `config_loader.*` / `ConfigManager.h` — YAML 配置加载与全局单例管理
- `yolodecet/` — YOLO 推理封装与检测结果结构体
- `rtk/NtripClient.*` — NTRIP 客户端，RTCM 接收与自动重连
- `rtk/RtkInjector.*` — RTCM 帧分片 → MAVLink GPS_RTCM_DATA 注入

## 运行流程

```
config.yaml → load_config_from_yaml() → ConfigManager (singleton)
                                              ↓
main.cpp: io_context → MavlinkProtocolHandler → Serial/TCP/UDP 传输 → 飞控
              ↓
         CaptureThread → FrameDispatcher → IDetector (TensorRT / OpenCV DNN)
              ↓                                    ↓
         main display loop              YOLO 检测结果 → get_latest_yolo()
                                              ↓
         MissionScheduler (独立线程) ← MavlinkProtocolHandler
              ↓                           ↓
         MotionController          NtripClient (RTK, 若启用)
         (速度 / 位置命令)            (RTCM → GPS_RTCM_DATA 注入)
              ↓
         MAVLink SET_POSITION / SET_VELOCITY → 飞控
```

### 任务调度状态流

```
WaitingForInitialization → TakingOff → Mission1InProgress → Mission2InProgress
     (等待GPS)              (起飞)    (目标识别/对准/投放)   (后处理：聚类目标 → 依次飞至 → RTL)
```

其中 Mission1InProgress 内部包含四个视觉伺服阶段：
```
水平对准 (HORIZONTAL_APPROACH) → 高度调整 (HEIGHT_ADJUSTMENT) → 稳定保持 (STABILIZATION) → 投放 (DROP_COMMAND)
```

## 环境与依赖

| 依赖 | 版本 | 链接方式 |
|------|------|----------|
| Visual Studio | 2022+ | 打开 `.vcxproj` |
| MSVC 工具集 | v145 | C++20 |
| OpenCV | 4.13.0 | `opencv_world4130.lib` |
| TensorRT | 10.x | `nvinfer.lib`, `nvinfer_plugin.lib` |
| CUDA | 12.x | `cudart.lib` |
| GeographicLib | 2.4 | `GeographicLib.lib` |
| yaml-cpp | — | `yaml-cpp.lib` |
| Boost.Asio | 1.x | 仅头文件 |
| MAVLink | — | `<common/mavlink.h>` |
| Windows SDK | 10.0 | `SetupAPI.lib` |

> 工程文件中的依赖路径为本地绝对路径，迁移到其他机器时需重新配置包含目录、库目录和运行时 DLL。

## 构建方式

1. 使用 Visual Studio 打开 `CUADC_fixedwing.slnx`（或 `CUADC_fixedwing.vcxproj`）
2. 选择 `x64` / `Release` 配置
3. 确保以下依赖已正确配置：
   - OpenCV 头文件、库和 DLL
   - TensorRT 头文件和库
   - CUDA 头文件和库
   - GeographicLib 头文件、库和 DLL
   - yaml-cpp 与 MAVLink 头文件
4. 将模型文件、类别文件和 `config.yaml` 放在可执行目录
5. 编译并运行

## 配置说明

所有参数通过根目录下的 `config.yaml` 加载，程序启动时读取。

### 1. 通信配置 (`communication`)

| 参数 | 说明 |
|------|------|
| `transport` | 传输方式：`serial`、`tcp`、`udp` |
| `serial.port` | 串口号 |
| `serial.baud_rate` | 波特率 |
| `tcp.host` / `tcp.port` | TCP 连接参数 |
| `udp.host` / `udp.port` | UDP 连接参数 |

### 2. MAVLink 配置 (`mavlink`)

| 参数 | 说明 |
|------|------|
| `system_id` | 本机系统 ID |
| `component_id` | 本机组件 ID（默认 MAV_COMP_ID_GPS） |
| `target_system` | 目标飞控系统 ID |
| `target_component` | 目标组件 ID |

### 3. 相机配置 (`camera`)

| 参数 | 说明 |
|------|------|
| `index` | 摄像头索引（VID/PID 枚举失败时备用） |
| `width` / `height` / `fps` | 采集分辨率与帧率 |
| `fx` / `fy` / `cx` / `cy` | 相机内参（标定值） |
| `dist_coeffs` | 畸变系数（OpenCV 格式） |
| `tx` / `ty` / `tz` | 相机安装偏移（米，机体坐标系） |
| `mount_roll` / `mount_pitch` / `mount_yaw` | 云台安装角（度） |
| `ae.enable` | 软件自动曝光总开关 |
| `ae.target_brightness` | 目标亮度（归一化 0~1） |
| `ae.exposure_min` / `ae.exposure_max` | 曝光限幅 |
| `ae.kp` / `ae.ki` | PI 调节系数 |
| `ae.update_interval` | 调节冷却帧数 |
| `ae.deadband` | 亮度死区（防极限环） |

### 4. 飞行控制参数 (`control`)

| 参数 | 说明 |
|------|------|
| `kp` | 比例系数 |
| `deadzone_x` | 最终死区阈值（像素，X 方向） |
| `deadzone_y` | 最终死区阈值（像素，Y 方向） |
| `deadzone_big` | 一级死区阈值（像素） |
| `max_speed` | 最大速度限制（m/s） |
| `initial_heading` | 起飞初始航向角（度） |
| `toudan_time_out` | 投弹超时时间（毫秒） |
| `manual_observe` | 测试模式：仅检测输出，不发送控制指令 |
| `gps_arrival_threshold_m` | GPS 到达判定距离阈值（米） |
| `gps_fly_timeout_ms` | GPS 飞行超时（毫秒） |
| `max_search_radius_m` | 搜索阶段最大距离（米，相对起飞点） |
| `servo_channel_left` / `servo_channel_right` | 左右舵机通道号 |
| `servo_offset_left_x` / `servo_offset_right_x` | 左右舵机 X 轴偏移（像素） |
| `servo_release_pwm_left` / `servo_release_pwm_right` | 左右舵机释放 PWM 值 |

### 5. YOLO 模型配置 (`yolo`)

| 参数 | 说明 |
|------|------|
| `backend` | 推理后端：`TensorRT` / `ONNX` |
| `model_path` | ONNX 模型路径 |
| `engine_path` | TensorRT 引擎路径 |
| `class_path` | 类别文件路径 |
| `use_gpu` | 是否使用 GPU 推理 |

### 6. 航点规划参数 (`waypoint`)

| 参数 | 说明 |
|------|------|
| `default_altitude` | 默认飞行高度（米） |
| `distance_to_drop_zone` | 起飞点到投弹区距离（米） |
| `distance_to_recon1` | 起飞点到侦察点1距离（米） |
| `distance_to_recon2` | 起飞点到侦察点2距离（米） |
| `recon_side_distance` | 侦察侧向偏移距离（米） |
| `delay_takeoff` | 起飞前延时（秒） |
| `delay_drop_point` | 投弹点停留延时（秒） |
| `delay_after_drop` | 投弹后延时（秒） |
| `delay_return` | 返航延时（秒） |
| `delay_recon_main` / `delay_recon_secondary` / `delay_recon_side` | 各侦察点延时（秒） |

### 7. RTK 差分定位 (`rtk`)

| 参数 | 说明 |
|------|------|
| `enabled` | 是否启用 RTK |
| `ntrip_host` / `ntrip_port` | NTRIP 服务器地址和端口 |
| `ntrip_mountpoint` | NTRIP 挂载点 |
| `ntrip_username` / `ntrip_password` | NTRIP 认证 |
| `gps_accuracy_threshold` | GPS 水平精度阈值（米） |
| `min_satellites` | 最少卫星数 |
| `gga_interval_sec` | GGA 上报间隔（秒） |
| `reconnect_interval_sec` | 断线重连间隔（秒） |
| `max_reconnect_attempts` | 最大重连次数 |
| `fix_timeout_sec` | RTK FIX 超时时间（秒） |
| `allow_gps_flight` | 超时后是否允许仅用 GPS 飞行 |

### 8. 目标跟踪 / 聚类 (`tracking`)

| 参数 | 说明 |
|------|------|
| `cluster_radius_m` | GPS 聚类匹配半径（米） |
| `min_samples` | 最少检测帧数才算稳定目标 |
| `max_targets` | 最多追踪目标数 |
| `max_age_frames` | 簇无更新帧数后删除 |

## 使用注意事项

- 工程较依赖本地硬件环境（摄像头、飞控、串口、模型文件），首次使用需根据实际设备调整配置
- `initial_heading`、航点距离、投放超时等参数直接影响任务行为，建议结合场地重新标定
- 若使用 TensorRT，引擎文件需与 CUDA / TensorRT 版本匹配
- `config_loader.h` 中的 C++ 默认值与 `config.yaml` 对应，缺失键会回退到默认值
- 舵机 PWM 值、死区阈值、视觉伺服增益等参数建议在实飞前通过地面测试确定
- 若在新机器上构建，建议优先整理 `.vcxproj` 中的依赖路径与运行时 DLL 部署方式
