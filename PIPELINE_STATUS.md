# CUADC_fixedwing 链路完成度清单

> 生成日期：2026-10-03。目的：梳理「哪些链路已跑通、哪些还没处理完」，方便逐项收尾。
> 每条标注 ✅已跑通 / ⚠️代码完成未实测 / ❌未完成。

---

## 一、已跑通的链路 ✅

| # | 链路 | 说明 | 验证情况 |
|---|------|------|----------|
| 1 | 配置加载 | `config.yaml` → `config_loader` → `ConfigManager` 单例 | 启动日志正常 |
| 2 | MAVLink 通信 | `serial/tcp/udp` 三种传输 → `MavlinkProtocolHandler` | SITL TCP(5762) 已验证 |
| 3 | 航点程序化生成 + 上传 | `buildMissionWaypoints()`（HOME+TAKEOFF+route 刚体变换）→ `MISSION_COUNT/REQUEST/ITEM/ACK` 握手 | SITL 21 航点 `MAV_MISSION_ACCEPTED` |
| 4 | 任务状态机 | `WaitingForInitialization → TakingOff → Mission1InProgress → LandingInProgress` | SITL 跑到「等待起飞」 |
| 5 | 相机采集 + 帧分发 | `CaptureThread` → `FrameDispatcher` | 依赖真机摄像头 |
| 6 | YOLO 检测 | TensorRT / OpenCV DNN → 检测线程 → `yoloout` | 12 类(best_detect) + 2 类(bucket) 引擎均可加载 |
| 7 | OCR 数字识别 | PP-OCRv6 `det→rec`（4 方向取置信度最高，ori 弃用）→ `OcrDigitReader` 投票 → `lookupDigit` | 端到端已跑通（`OcrPipelineTest`） |
| 8 | 像素→GPS + GPS 聚类 | `PixelToGPSConverter` + `GPSTargetClusterer`（已改多类过滤） | 单元级 OK |
| 9 | 目标选择 | `TargetSelection.h`：任务一价值最高 / 任务二中位数 + `pictureValueTable` | 纯逻辑，已接 |
| 10 | 投弹引导 | `flyThroughAndRelease`（入口点延长线 + 提前量 + 舵机释放） | ⚠️ 代码完成，未实飞 |

---

## 二、未完成 / 待办链路 ❌

### A. 架构层面 —— 需你定夺（最关键）

**1. 任务一到底是「单模型直检」还是「先检天井→再检图案」两段式？** ⚠️⚠️

存在两处不一致，必须先对齐，否则任务一会错：

- **本次实现（当前代码）**：`task_type==1` 直接用 12 类模型全图检测（`main.cpp:274-278`），聚类不按类名过滤（`cluster_target_class=""`），按 `pictureValueTable` 取价值最高。
- **此前你确认的语义（2026-10-02 记录）**：两任务流程一致，应「先 `best.pt`(2 类) 检天井(bucket) → 任务一在天井内跑 12 类图案 / 任务二在天井内跑 OCR」。

如果两段式是对的，还需要补：
- 双检测器（或顺序切换模型）+「检出天井 → 裁剪天井区域 → 再跑 12 类」的串联逻辑（当前是单检测器、按 `task_type` 一次只跑一个模型，`main.cpp:286-299` 是「替换」非「并行」）。
- 命中后把「天井内的图案类别」喂给聚类/价值表。

**结论：先确认走哪条，再决定要不要补两段式。**

---

### B. 上机部署（Jetson Orin 重建引擎）

| # | 事项 | 说明 |
|---|------|------|
| 2 | `best_detect.engine` 上机重建 | 当前是 Windows RTX5060(sm120) 构建，Orin(sm87) 不通用；用 Jetson 侧 trtexec 对 `best_detect.onnx` 重建 |
| 3 | `best_bucket.engine` 上机重建 | 同上，任务二天井检测引擎需 Orin 版 |
| 4 | OCR `det/rec` engine 上机重建 | PP-OCRv6 的 det/rec 引擎同样需 Orin 重建（`TrtEngine` 会自动从 ONNX 建，但 Windows 与 Orin 架构不同） |

> 注意：YOLO 检测器 `YoloV8TensorRT::loadEngine` 只反序列化现成 `.engine`，**不会**像 `ocr::TrtEngine` 那样自动从 ONNX 构建，所以检测引擎必须手动重排。

---

### C. 实机联调（未完成）

| # | 事项 | 说明 |
|---|------|------|
| 5 | **RTK / NTRIP 连不上** | 服务器对账号密码返回 `HTTP 401 Unauthorized`，真实 RTK FIX 拿不到，影响真机定位精度。需换账号/改密码（凭据已在公开仓库泄漏过，建议后台改密）。 |
| 6 | 真机串口切换 | `transport` 现为 `tcp`(SITL 用)，真机要改 `serial`，并确认 3 个 `/dev/ttyUSB*` 哪个是飞控 |
| 7 | 摄像头 USB | Jetson bench 无 `/dev/video0`，采集线程会先于检测器退出，需接 USB 摄像头 |
| 8 | 投弹引导实飞验证 | 入口点(`entry_extension_distance_m`)、切入半径(`entry_reach_radius_m`)、提前量(`release_lead_offset_m`/`drop_fall_time_s`)、舵机 PWM(通道8/1995) 均只在纸面/SITL 验证过 |

---

### D. 端到端未验证（需实机/实拍画面）

| # | 链路 | 说明 |
|---|------|------|
| 9 | 任务一全链 | 「识别 12 类图案 → 聚类 → 价值最高 → 投弹」需实拍画面验证 |
| 10 | 任务二全链 | 「检天井 → OCR 读 3 个两位数 → 中位数 → 投弹」OCR 单测通过，整链未实机验证 |

---

### E. 代码卫生 / 过时注释 / 死代码

| # | 位置 | 问题 |
|---|------|------|
| 11 | `MissionScheduler.cpp:21-28` | `resolveTargetValue` 注释仍写「单类 toudan 占位、12 类未训练」，已过时 |
| 12 | `tracking/GPSTargetClusterer.h:21` | 注释仍写默认 `"toudan"` 旧占位 |
| 13 | `main.cpp:400-413` + `MissionScheduler.h:44-45` | 侦察模型切换 `isReconModelRequested/setReconModelReady` 是**死代码**（atomic 从不置 true），且硬编码用 `OpenCVDNNDetector`。应清理或真正接线 |
| 14 | `main.cpp:376-388, 415-427` | OCR 的 `enable_on_recon`（"recon" 触发）在固定翼流程永不生效（状态机只有 "bomb" 阶段）；任务一时 OCR 仍会在 bomb 阶段空跑（对图案找不到数字，浪费算力），可对任务一直接关 OCR |
| 15 | `README.md` | 严重过时：仍描述多旋翼 `Mission1/Mission2`、4 阶段视觉伺服（水平/高度/稳定/投放）、ArduCopter 模式号 3/4/6，与当前固定翼 ArduPlane 流程（`AUTO=10/GUIDED=15/RTL=11`、GUIDED 引导投弹）不符 |
| 16 | `config.yaml:86` | `engine_path` 注释「SITL 临时用，上机改回 best_7.26_orin.engine」——实际 `task_type=1/2` 时该默认值都被分支覆盖用不到，容易误导 |

---

## 三、收尾建议顺序

1. **先定 A1（任务一架构）** —— 这是唯一影响正确性的决策。
2. 做 B（Jetson 重排 3 个引擎）+ C5（RTK 账号）—— 真机联调的前置。
3. 做 C6/C7/C8 真机联调。
4. D9/D10 实拍端到端验证。
5. 最后清 E（过时注释 + 死代码 + README）。
