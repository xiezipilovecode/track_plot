# track_plot（CARLA 轨迹复现与隧道数据集采集系统）

本仓库围绕 CARLA 仿真引擎，提供三大核心能力：

- **轨迹复现**：外部车辆轨迹数据解析、质量校验、地图对齐可视化、CARLA 世界中自动回放
- **隧道仿真与数据集采集**：三车道代理车流生成、多相机图像/2D框/实例分割采集、批量自动化导出
- **HoloLens 2 WebRTC 推流**：将选中代理车驾驶员视角实时推流到 HoloLens 2，接收头部姿态控制相机旋转

---

## 项目目录结构

```text
track_plot/
├─ auto_control_main.py             # 轨迹回放入口
├─ run_all_batches.bat              # 一键批量采集脚本
├─ tp_replay/                       # 轨迹复现核心包
├─ tp_tunnel_traffic/               # 隧道仿真 + 数据集采集
│  ├─ main.py                       # 隧道场景主入口
│  ├─ collector.py                  # 帧级数据集采集器（RGB/COCO/instance）
│  ├─ video_collector.py            # 视频录制器（连续帧 MP4）
│  ├─ hololens_server.py            # HoloLens 2 WebRTC 推流服务端
│  ├─ config.py                     # 全部环境变量配置 (TT_*/TP_*)
│  ├─ control.py                    # 自动驾驶控制器
│  ├─ lane_sampling.py              # 三车道提取
│  ├─ gui.py                        # Pygame GUI 控制台
│  ├─ spawning.py                   # 车辆生成与净空判定
│  ├─ vehicle_plan.py               # 代理车计划
│  ├─ autopilot.py                  # Traffic Manager 集成
│  ├─ merge_coco.py                 # 全局 COCO 合并工具
│  ├─ validate_dataset_run.py       # run 级离线校验工具
│  ├─ dataset_cameras.json          # 相机布局配置
│  ├─ camera_offsets.json           # 视角微调配置
│  ├─ docs/                         # 设计文档
│  ├─ tests/                        # 测试入口
│  └─ README.md                     # 模块详细文档
├─ holens_websocket/                # HoloLens 推流原型（opencvdriver）
│  ├─ opencvdriver.py               # 独立推流服务端（已停用）
│  ├─ hololens_websocket_server.py  # WebRTC 模拟客户端
│  └─ README.md                     # 推流模块文档
├─ tools/                           # 独立工具脚本
├─ dataset/                         # 数据集输出目录
├─ dataset_video/                   # 视频录制输出目录
├─ website/                         # 项目文档网站 (VitePress)
├─ __pycache__/
└─ README.md
```

---

## 一、轨迹复现（tp_replay）

外部车辆轨迹数据在 CARLA 世界中的自动回放。

### 快速开始

```powershell
conda activate carla
cd E:\code\track_plot
$env:TP_PRESERVE_EXISTING_WORLD='1'
python .\auto_control_main.py
```

### 关键配置

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `TP_DATA_FILE_PATH` | — | 轨迹数据文件 |
| `TP_DATA_SCALE` | 0.01 | 厘米→米缩放 |
| `TP_REVERSE_DIRECTION` | True | 隧道逆向行驶 |
| `TP_SNAP_THRESHOLD` | 15.0 | 车道吸附距离（米） |

---

## 二、隧道仿真与数据集采集（tp_tunnel_traffic）

### 核心能力

- 三车道代理车流（左快中稳右慢，持续补车）
- 7 相机 RGB + 7 相机实例分割 PNG
- COCO 2D 车辆检测框自动标注
- 控制量标签（steer/throttle/brake + ego state）
- **HoloLens 2 WebRTC 推流**（选中车驾驶员视角实时推流）
- **视频录制**（连续帧 MP4，FFmpeg 背景线程编码）
- 全局 COCO 合并（`merge_coco.py`）

### 各功能快速运行

**前提**：CARLA Server 已启动，conda 环境已激活。

#### 1) GUI 手动采集（含 HoloLens 推流 + 视频录制）

```bat
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla
cd /d E:\code\track_plot

set TP_PRESERVE_EXISTING_WORLD=1
set TT_SYNC_MODE=1
set TT_GUI_ENABLE=1
set TT_PROXY_ENABLE=1
set TT_PROXY_TARGET_PER_LANE=8

rem 数据集采集（7 相机 + COCO + instance）
set TT_COLLECT_OUTPUT_DIR=dataset
set TT_COLLECT_FRAME_STRIDE=10
set TT_COLLECT_RECORD_LABELS=1
set TT_COLLECT_ENABLE_INSTANCE_SEGMENTATION=1
set TT_COLLECT_WRITE_COCO=1
set TT_COLLECT_COCO_MIN_AREA_PX2=200
set TT_COLLECT_COCO_MAX_HEIGHT_RATIO=0.9

rem 视频录制
set TT_VIDEO_ENABLE=1
set TT_VIDEO_OUTPUT_DIR=dataset_video

rem HoloLens 推流
set TT_HOLOLENS_ENABLE=1
set TT_HOLOLENS_PORT=8765

python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
```

GUI 操作：
- 左侧 `Proxies` 列表点选代理车 → 切到该车驾驶员视角
- `Collect Selected` → 帧级数据集采集
- `Record Video` → 连续视频录制
- `HoloLens Stream` → WebRTC 推流到 HoloLens 2
- 三个按钮**互斥**，切换代理车时推流和录制自动跟随

#### 2) 批量自动化采集

```bat
E:\code\track_plot\run_all_batches.bat
```

六批次不同车流配方（密度/速度/TM 模式），自动选车→采集→换车循环。

#### 3) HoloLens 推流本地测试

```bat
python -m tp_tunnel_traffic.tests.test_hololens_client
```

客户端弹出 OpenCV 窗口显示实时画面。HoloLens 2 连接 `ws://<PC_IP>:8765`。

#### 4) 数据集校验

```bat
python -m tp_tunnel_traffic.validate_dataset_run --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset
```

---

## 三、HoloLens 2 WebRTC 推流

将选中代理车的驾驶员视角通过 WebRTC 实时推流到 HoloLens 2，同时接收头部姿态（yaw/pitch）控制相机旋转。

### 架构

```
CARLA 仿真 tick
  └─ 世界相机（不绑定车辆，回调中手动跟踪选中车位置）
       ├─ VideoStreamTrack → aiortc → WebRTC → HoloLens 2 显示
       └─ DataChannel "controls" ← 头部 yaw/pitch ← HoloLens 2
```

### 技术要点

- **单一世界相机**：不绑定任何车辆，切车时无创建/销毁操作，彻底避免 CARLA 同步模式下崩溃
- **相机回调实时跟踪**：读取当前 `self._vehicle` 的位姿 + 驾驶位偏移 + 头部旋转 → 计算世界坐标
- **DataChannel 接收姿态**：8 字节 LE binary `[yaw: f32] [pitch: f32]`
- **asyncio 后台线程**：WebRTC 信令和推流不阻塞 CARLA 主循环

---

## 四、轨迹复现详细流程

### Step 1：数据质量校验

```powershell
python .\tools\validate_trace_data.py --data-file .\trace_data\test_data\data_6lu2.txt --output-dir run_logs/validator
```

### Step 2：地图对齐可视化

```powershell
python .\tools\trace_display.py
```

蓝色=原始，绿色=吸附；偏差线过长说明需调 `MANUAL_ROTATION_FIX`。

### Step 3：正式回放

```powershell
$env:TP_PRESERVE_EXISTING_WORLD='1'
python .\auto_control_main.py
```

---

## 五、常见问题

| 问题 | 排查 |
|---|---|
| GUI 不弹窗 | `pip install pygame numpy` |
| 没有代理车 | 确认 `TT_PROXY_ENABLE=1`；观察控制台 `代理车详情` |
| HoloLens 无画面 | 确认 `TT_HOLOLENS_ENABLE=1`；先选代理车再点 HoloLens Stream |
| 采集不写盘 | 看控制台 `采集状态: 持续采集已启动`；若大量 `miss_no_complete_frame`，降低车流密度 |

---

## 参考命令汇总

```bat
rem 激活环境
call E:\Programs\miniconda\Scripts\activate.bat
conda activate carla
cd /d E:\code\track_plot

rem --- 轨迹复现 ---
python .\auto_control_main.py

rem --- 隧道仿真（GUI 采集 + HoloLens 推流 + 视频录制）---
python -m tp_tunnel_traffic.tests.test_tunnel_autodrive

rem --- 批量自动化采集 ---
run_all_batches.bat

rem --- 数据集校验 ---
python -m tp_tunnel_traffic.validate_dataset_run --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
python -m tp_tunnel_traffic.merge_coco --dataset-dir dataset

rem --- HoloLens 推流测试 ---
python -m tp_tunnel_traffic.tests.test_hololens_client
```
