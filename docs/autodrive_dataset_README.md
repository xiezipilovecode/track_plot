# 隧道自动驾驶数据集采集规范（CARLA）

本文档为在 CARLA 隧道场景中构建自动驾驶数据集的工程化规范，目标是利用一辆采样车（Ego）采集 7 路摄像头数据（6 路外环 + 1 路车内）并生成训练/测试用的数据集（兼容 KITTI/nuScenes 思路）。文档包含配置、同步、校准、存盘格式、标注与 QA 检查清单，便于你审阅并据此实现脚本/流水线。

---

## 一、背景与目标
- 背景：已有功能：可让车辆在仿真隧道中沿车道自动驾驶（autopilot/agent 已就绪）。
- 目标：基于该车辆生成一个高质量的、可复用的自动驾驶数据集，每帧包含 7 路同步 RGB 图像 + 元数据（ego pose、内外参、时间戳），并能生成 2D/3D 标注（仿真环境可直接获得 GT）。
- 最终交付：按 scene 划分的数据集目录（图片、校准文件、annotations、metadata、splits、README）。

> 请先在文档顶部替换关键参数（下面示例值可按需调整）：
> - CARLA_VERSION: 0.9.xx
> - FRAME_RATE: 20 (Hz)
> - IMAGE_SIZE: 1280x720
> - FOV: 90 (deg)
> - OUTPUT_ROOT: /data/autodrive_dataset/

---

## 二、前置条件
- 已安装并能运行指定版本的 CARLA，并能用 Python API 连接（client/server 通信正常）。
- 有权限保存大量数据（估算见后文）。
- 已准备好 Ego 车辆 spawn 与 autopilot 流程（用于让车在隧道中行驶采样）。
- 推荐 Python 环境：3.8/3.9，常用依赖：numpy, opencv-python, scipy, Pillow, pyquaternion（或 transforms3d）。

---

## 三、总体架构概览
1. 在 CARLA 中 spawn Ego，挂载 7 个相机 sensor（sensor.camera.rgb）。
2. 开启 World 的同步模式（Synchronous Mode）：`fixed_delta_seconds = 1.0 / FRAME_RATE`。
3. 每次 `world.tick()`：从多个摄像头回调队列收集当前 frame 的 7 张图像（frame_id 对齐），同时读取 ego pose 与 actor 列表生成标注。
4. 保存图片与元数据到 disk（按 scene 分目录），并按需生成 KITTI/nuScenes 风格的标注文件。
5. 结束时正确回收 actor 并恢复 world 原有设置。

---

## 四、传感器配置（建议）
- 摄像头命名：
  - cam_front
  - cam_front_left
  - cam_front_right
  - cam_back
  - cam_back_left
  - cam_back_right
  - cam_cabin（车内驾驶位/仪表盘）
- 建议相对 transform（vehicle-local，单位 m / deg，可根据车模型微调）：
  - cam_front:     (x=1.6,  y=0.0,  z=1.6, pitch=0,   yaw=0,    roll=0)
  - cam_front_left:(x=1.2,  y=-0.6, z=1.6, pitch=0,   yaw=-60,  roll=0)
  - cam_front_right:(x=1.2, y=0.6,  z=1.6, pitch=0,   yaw=60,   roll=0)
  - cam_back:      (x=-1.6, y=0.0,  z=1.6, pitch=0,   yaw=180,  roll=0)
  - cam_back_left: (x=-1.2, y=-0.6, z=1.6, pitch=0,   yaw=-120, roll=0)
  - cam_back_right:(x=-1.2, y=0.6,  z=1.6, pitch=0,   yaw=120,  roll=0)
  - cam_cabin:     (x=0.5,  y=0.3,  z=1.4, pitch=-5,  yaw=0,    roll=0)
- 相机内参（示例计算）：
  - width = IMAGE_WIDTH，height = IMAGE_HEIGHT，FOV = FOV（deg）
  - fx = fy = 0.5 * width / tan(FOV * π / 360)
  - cx = width / 2, cy = height / 2
- 建议图像格式：PNG（无损）或 JPG_quality=90（节省空间）。深度/语义可选。

---

## 五、同步与采集策略（关键）
- 开启同步模式：
  - settings.synchronous_mode = True
  - settings.fixed_delta_seconds = 1.0 / FRAME_RATE
- 采集队列机制：
  - 每个摄像头的 `.listen(callback)` 将 image 对象（带 frame/frame_id、timestamp、sensor_name）放入线程安全队列（按 frame_id 分组）。
  - 主循环：`world.tick()` -> wait 收齐该 frame_id 的 7 张图像（超时策略 e.g. 200 ms）。收齐则标记为完整帧并保存；若超时则记录为丢帧并可选择跳过或保存不完整帧。
- 时间戳：记录 image.timestamp（float seconds），以及 world snapshot timestamp。写入 `timestamps.txt`（frame_idx timestamp_seconds）。

---

## 六、校准（内参 + 外参）与单位约定
- 单位：坐标使用米（m），速度 m/s，时间秒（s）。
- 内参文件 `camera_intrinsics.json` 示例条目：
```json
{
  "cam_front": { "width": 1280, "height": 720, "fx": 1200.0, "fy": 1200.0, "cx": 640.0, "cy": 360.0, "fov": 90 }
}
```
- 外参文件 `camera_extrinsics.json`：保存每个 camera 相对于 ego body 的 4x4 变换矩阵或 translation + quaternion（world->camera 或 camera->world，需在使用处明确约定）。
- 如何获取：在 CARLA 中用 `camera.get_transform()`（相对 ego），并把欧拉角转为 R 矩阵或四元数构造 4x4。
- 保存 ego_pose（world frame）每帧：translation (x,y,z) + rotation quaternion (qx,qy,qz,qw) 或 yaw/pitch/roll。

---

## 七、标注策略（仿真优势）
- 每帧遍历 `world.get_actors().filter('vehicle.*')`（剔除 ego）：
  1. 获取 actor 的 3D bounding box（actor.bounding_box）8 个顶点，变换到 world 坐标。
  2. 将 world 坐标点转换到 camera 坐标（使用 camera extrinsic），再用 intrinsics 投影到像素平面得到 2D box。
  3. 若所有顶点在相机后方或投影位于图像外，则跳过（不可见）。
  4. 遮挡判定：若 depth map 可用，用 depth 值比较（或使用 raycast）判断 occlusion。
- 标注格式输出：
  - KITTI-style（每帧一行或每帧一个 txt）用于 2D/3D 检测训练（Type,h,w,l,x,y,z,rotation_y,...）
  - JSON（nuScenes-like）便于保存更多元数据（track_id, velocity, attributes）
- Track ID：使用 actor.id 映射为 dataset 中的 track id，保存映射表用于后续处理。

---

## 八、数据落盘目录示例（推荐）
```
dataset_root/
├── scene_0000/
│   ├── cameras/
│   │   ├── cam_front/          000000.png 000001.png ...
│   │   ├── cam_front_left/
│   │   └── cam_cabin/
│   ├── json/
│   │   ├── timestamps.txt      (frame_idx timestamp)
│   │   ├── ego_pose.json       (per-frame ego transform)
│   │   ├── camera_intrinsics.json
│   │   └── camera_extrinsics.json
│   ├── annotations/
│   │   ├── 000000.txt (KITTI) or 000000.json
│   ├── depth/ (optional)
│   └── seg/ (optional)
├── splits/
│   ├── train.txt
│   └── val.txt
└── README.md
```

---

## 九、验证（QA）与可视化检查
- 同步完整性：
  - 统计每 scene 的总帧数、完整帧数、丢帧率（目标 < 0.3%）。
  - 随机抽取 N 帧，检查 7 张图片的 timestamp 差异（目标 < 1 ms）。
- 标注准确性：
  - 随机抽 100 帧：把投影的 2D bbox 叠加在图片上，人工检查是否对齐。
  - 计算投影误差：选若干 3D 点，将其投影 -> 反投 -> 与原 world 坐标比较（期望小于 5 cm 或像素误差 < 5 px）。
- 轨迹连贯性：检查目标 track 在时间上的连贯性（速度、加速度异常值检测）。
- 自动检查脚本包含：frame completeness, timestamp std/max, reprojection error stats（p50/p90）。

---

## 十、性能与存储估算（供参考）
- 设定：IMAGE 1280x720 JPG@90 ~ 150 KB/张（视场景复杂度）
- 7 cam 每帧 ~ 7*150KB = 1.05 MB；20 Hz => 21 MB/s；1 小时 ≈ 75 GB（raw images）。
- 建议：先用 10–20 分钟短样本做验证，再评估正式采集时长与压缩策略（PNG->JPG, 降分辨率, 降帧率）。

---

## 十一、运行流程（简要步骤）
1. 准备 CARLA 并启动 server。
2. 准备 config.json（CARLA host/port, scene name, output_root, fps, image size, camera transforms）。
3. 启动数据采集脚本（data_capture_carla.py）：
   - 脚本 spawn ego，attach sensors，apply sync settings。
   - 启动主循环若干秒/若干帧后退出。
4. 运行标注生成（annotation_generator.py）或实时在主循环内生成并写盘。
5. 使用 visualize_frame.py 抽检若干帧。
6. 运行 validate_dataset.py 生成 QA 报告。
7. 使用 data_packager.py 导出 KITTI/nuScenes-lite 格式并切分 train/val。

---

## 十二、质量控制与交付清单（采集前/采集后）
采集前：
- [ ] 确认 CARLA 版本与 API 兼容性
- [ ] 确认 output_root 可用空间 >= 预计需求
- [ ] 确认 camera transforms、FOV、分辨率、FPS
采集过程中：
- [ ] monitor 丢帧率、队列延迟、磁盘写入速率
采集后：
- [ ] 完整帧率/丢帧率报告
- [ ] 随机 100 帧的 overlay 可视化图（含 bbox）
- [ ] camera_intrinsics/extrinsics 与 ego_pose 文件
- [ ] dataset README（包含采集参数、CARLA seed、地图名）

---

## 十三、扩展（可选）
- 添加 LIDAR：可同步采集点云并保存为 bin（nuScenes/Waymo 风格）。
- 添加 IMU/GNSS/CAN：用于定位/控制学习。
- 支持 depth/seg：为语义/深度任务提供 GT 图像。
- 导出为 nuScenes 格式（更复杂，需要 token/scene 概念）。

---

## 附录 A：示例 config.json 模板
```json
{
  "carla_host": "127.0.0.1",
  "carla_port": 2000,
  "carla_map": "YourTunnelMap",
  "output_root": "/data/autodrive_dataset/",
  "frame_rate": 20,
  "image_width": 1280,
  "image_height": 720,
  "fov": 90,
  "cameras": {
    "cam_front": {"x":1.6, "y":0.0, "z":1.6, "pitch":0, "yaw":0, "roll":0},
    "cam_front_left": {"x":1.2, "y":-0.6, "z":1.6, "pitch":0, "yaw":-60, "roll":0}
    // ... 其它相机
  },
  "save_depth": false,
  "save_seg": false,
  "timeout_ms": 200
}
```

---

## 附录 B：建议给代码生成器的 prompt（若要我直接生成脚本）
- 若需要我自动生成 `data_capture_carla.py`、`calibs_exporter.py`、`annotation_generator.py` 三个脚本以及 QA 脚本，请回复 **“生成脚本”** 并确认：
  - CARLA 版本、Python 版本
  - frame_rate、image_size、output_root
  - 是否保存 depth/seg / 是否包含 LiDAR
我收到确认后会基于本规范生成可运行脚本并附带说明和示例 config。

---

## 检查点：请确认你要我进一步做哪项（勾选一个）：
1. 把上面规范直接生成成 README.md（该文档即为 README）并生成示例脚本骨架；  
2. 仅生成 `data_capture_carla.py`（含 config 模板）；  
3. 生成完整脚本包（capture + calib exporter + annotation + validate + visualize）；  
4. 只需把该 Markdown 输出为本地文件（我给出下载指引/内容， 你自行保存）。  

选项回复序号或给出修改点，我立即按你的偏好继续。
