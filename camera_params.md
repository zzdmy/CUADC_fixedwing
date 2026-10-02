# CUADC 所有调用摄像头的程序 · 参数总表

> 生成时间：2026-10-06。相机 = 采集卡（"Global Shutter Camera" USB），Linux 下即 `/dev/video0`。
> **V4L2 是独占的：同一时刻只有一个进程能打开相机** —— 下表各程序的互斥规则见文末。

## 一、总表

| # | 程序 | 源码文件 | 运行方式 | 后端 | 设备选择 | 分辨率 | 帧率 | FOURCC | BUFFERSIZE | 曝光(AUTO_EXPOSURE) | 预热 | 输出 |
|---|------|---------|---------|------|---------|--------|------|--------|-----------|--------------------|------|------|
| 1 | **主程序** cuadc_fixedwing | `CaptureThread.cpp` + `main.cpp` | systemd 开机自启（cuadc.service） | V4L2（Win: DSHOW） | ①VID/PID `345F:2130` → ②`534d:2109` → ③第一个可用相机 → ④兜底 `camera.index`(=0) | 1920×1080（config `camera.width/height`） | 60（config `camera.fps`） | **MJPG**（代码默认 `useMjpg=true`，无 config 项） | 1 | `ae.enable=false`（当前）→ **0.75 相机自动曝光**；`true`→ 0.25 手动 + PI 控制 | 无（长跑连续采） | 识别输入 + 预览窗口 + `flt_*.mp4` |
| 2 | 机内录像线程（主程序内） | `main.cpp` `recordThreadFunc` | 主程序运行中且 `record.enable=true` | — **不开相机** | 从 frameDispatcher 取主程序已采的帧 | 同主程序 | 按 30fps 节奏写 | — | — | 沿用主程序 | — | `recordings/flt_*.mp4`（x264 6Mbps，120s/段） |
| 3 | 画面预览 preview.sh | `preview.sh` | 机载桌面双击「画面预览」 | — **不开相机** | 只把 `enable_display` 改 true 后直接跑主程序（同一二进制） | 同主程序 | 同主程序 | — | — | 沿用主程序 | — | 屏幕 Video 窗口（无飞控时自动起假飞控保活） |
| 4 | **地面录像** camera_recorder | `ocr_test/record_cam.cpp` | systemd 开机自启（cuadc-recorder.service） | V4L2 | 固定 index 0 | 1920×1080（源码常量） | **30**（源码常量） | **未设置**（走 V4L2 默认 = YUYV） | 1 | **0.75 相机自动曝光**（注释：与主程序一致，防全黑） | 无（连续采） | `recordings/rec_*.mp4`（x264 6Mbps，300s/段，80GB 轮转） |
| 5 | snapshot 抓图工具 | `ocr_test/snapshot.cpp` | 手动 | V4L2 | 固定 index 0 | 1920×1080 | 60 | 未设置（默认 YUYV） | 1 | 0.75 相机自动曝光 | **90 帧 ≈ 3s**（防抓全黑） | `<out.jpg>`（可 `x y w h` 裁剪） |

## 二、不碰相机的程序（只处理文件/合成图）

| 程序 | 源码 | 输入 |
|------|------|------|
| video_detect_test | `ocr_test/video_detect_test.cpp` | 视频文件/图片路径（`VideoCapture(input)`，含 `--stride/--crop/--ocr`） |
| OcrPipelineTest / OcrBatchTest | `ocr_test/*.cpp` | 合成图/图片目录 |
| onnx_dnn_test / yolo_engine_check | `ocr_test/*.cpp` | ONNX/engine 文件 |
| probe_mp4 / probe2_mp4 | `ocr_test/*.cpp` | MP4 文件（探帧数/解码） |

## 三、关键差异与注意点

1. **主程序（识别/预览/机内录像）实际只开一次相机**：CaptureThread 采帧 → frameDispatcher → 检测线程 / 显示线程 / 录像线程各自取帧，三者不重复打开设备。
2. **只有"地面录像"和"snapshot"会作为第二个进程抢相机**：
   - 地面录像内置互斥：每 2s 查主程序进程，主程序运行 → 释放相机让出；主程序退出 → 自动接管（2s 内）。
   - snapshot 是手动工具，用之前请确认主程序没在跑，否则会"打开失败"。
3. **帧率不一致是故意的**：主程序 60fps 供识别（高帧率利于跟踪/投弹窗口），地面录像 30fps 省 CPU/体积，snapshot 60fps 只抓单帧无所谓。
4. **FOURCC 差异**：只有主程序设置 MJPG（1080p60 必须，YUYV 带宽不够）；地面录像 30fps + YUYV、snapshot 单帧 + YUYV 都可行。⚠️ 若把地面录像帧率提到 60，需同步加 MJPG，否则会打不开/掉帧。
5. **曝光统一策略**：所有程序默认都是"相机自动曝光"（0.75）；只有主程序开了软件 AE（`ae.enable=true`）才会切 0.25 手动接管（当前 config 为 false）。
6. **preview.sh 与 systemd 服务的关系**：预览脚本直接跑 `./build/cuadc_fixedwing`；若此时 cuadc.service 已真正运行（拿到了飞控链路），会上演"两个主程序抢相机"→ 预览实例打开失败。预览前确保服务处于"等待链路"状态或先 `systemctl stop cuadc`。
7. **画面预览不含检测框**：显示的是原始帧 + 罗盘 + 状态栏（检测框的叠加显示只在带 display 的调试路径里）。

## 四、主程序打开相机的完整调用序列（CaptureThread.cpp）

```
open(index, CAP_V4L2)                          # 索引来自 VID/PID 枚举
set(CAP_PROP_FOURCC, 'MJPG')                   # useMjpg=true（默认）
set(CAP_PROP_BUFFERSIZE, 1)                    # 只留最新帧，防延迟堆积
set(CAP_PROP_FRAME_WIDTH, 1920)                # config.camera.width
set(CAP_PROP_FRAME_HEIGHT, 1080)               # config.camera.height
set(CAP_PROP_FPS, 60)                          # config.camera.fps
if ae.enable:  set(CAP_PROP_AUTO_EXPOSURE, 0.25); current = clamp(get(CAP_PROP_EXPOSURE), -10, -3)
else:          set(CAP_PROP_AUTO_EXPOSURE, 0.75)   # ← 当前生效
# 之后每帧计亮度，误差超 deadband(0.15) 且冷却到期才微调（kp=1.0, ki=0.001）
```

（config.yaml 的 `camera.ae`: enable=false, target 0.4, exposure -10~-3, kp 1.0, ki 0.001, interval 30, deadband 0.15）

## 五、互斥关系一览

| 场景 | 谁占着相机 | 其他程序 |
|------|-----------|---------|
| 待机（主程序在等飞控链路） | 地面录像（rec_*.mp4） | 主程序尚未开相机 |
| 主程序运行（飞行/预览） | 主程序（识别+预览+flt_*.mp4 共用） | 地面录像 2s 内让出，主程序退出后自动接管 |
| 手动跑 snapshot / video_detect_test | 该工具 | 请先停主程序/地面录像 |
| 比赛模式 | 主程序（录像功能关闭） | 地面录像服务被停用+取消自启 |
