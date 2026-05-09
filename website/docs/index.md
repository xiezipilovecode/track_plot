---
layout: home
hero:
  name: "TAT Dataset"
  text: TunnelAutopilot‑Tunnel
  tagline: 61 辆车 · 3,504 帧/相机 · 24,528 张 RGB 图像 · 24,528 张实例掩码 · 27,763 个 COCO 标注框
  image:
    src: /muti-view-sample.png
    alt: TAT 多视角相机样本
  actions:
    - theme: brand
      text: 浏览数据格式
      link: /data-format
    - theme: alt
      text: 下载数据集
      link: /download
features:
  - icon: 📊
    title: 61 辆车 · 61 次采集
    details: 分 6 个批次采集，涵盖不同车流密度和速度。每次采集约 180 秒，在逼真的 CARLA 三车道隧道中自动驾驶运行——总计超过 3 小时的驾驶数据。
  - icon: 📷
    title: 7 相机环视
    details: 鸟瞰（自车）、前向、左前/右前、左侧/右侧、后向——全部由 <code>world_frame</code> 硬同步。每相机 3,504 帧，为行为克隆和 3D 感知提供丰富的空间上下文。
  - icon: 🎯
    title: 实例分割（24K 掩码）
    details: 像素级车辆实例掩码与<strong>每一张</strong> RGB 图像配对——24,528 张掩码，覆盖率 100%。每辆车实例标注唯一颜色 ID，可直接用于分割模型训练。
  - icon: 📦
    title: COCO 2D 检测（27K 标注框）
    details: 所有帧共 27,763 个标注框。提供每次采集的 COCO 文件和全局合并的 <code>coco_annotations.json</code>。可直接接入 Detectron2、MMDetection、YOLO 等 COCO 兼容框架。
  - icon: 🕹️
    title: 完整控制监督
    details: 每帧提供转向/油门/刹车指令、自车世界位姿 (x, y, z, yaw) 和速度 (m/s)。每次采集 65 帧标注数据，为行为克隆和控制回归提供密集控制信号。
  - icon: 🤖
    title: 自动化采集流水线
    details: 6 批次采集，支持可配置的车流密度/速度范围。完全可复现——<code>batch_summary.json</code> 记录每次采集详情，<code>auto_collect_config.json</code> 支持暂停与恢复。
---

## 核心统计

<div class="tat-stats-grid">
  <div class="tat-stat-card">
    <div class="stat-number">61</div>
    <div class="stat-label">辆车<br/>6 个采集批次</div>
  </div>
  <div class="tat-stat-card">
    <div class="stat-number">24,528</div>
    <div class="stat-label">RGB 图像<br/>800×600 像素</div>
  </div>
  <div class="tat-stat-card">
    <div class="stat-number">24,528</div>
    <div class="stat-label">实例掩码<br/>100% 配对</div>
  </div>
  <div class="tat-stat-card">
    <div class="stat-number">27,763</div>
    <div class="stat-label">COCO 标注框<br/>平均 1.13 / 张</div>
  </div>
</div>

---

## 关于数据集

<div class="tat-narrative">

**TunnelAutopilot‑Tunnel (TAT)** 是一个纯视觉自动驾驶数据集，采集自逼真的 CARLA 三车道隧道环境。与晴朗天气的高速公路基准不同，TAT 捕捉了隧道驾驶的独特感知挑战：

- **低照度与不均匀照明**——隧道灯光产生强烈阴影和眩光斑点，对视觉模型构成挑战。
- **结构重复性**——统一的墙壁、车道线和天花板图案使特征匹配和定位变得困难。
- **动态多车交通流**——61 辆代理车辆同时行驶，具备逼真的跟车与换道行为。

本数据集为每辆车提供 **7 台同步 RGB 相机**、每帧**实例分割掩码**，以及全部采集共 **27,763 个 COCO 2D 标注框**。全局合并的 COCO 文件可直接接入标准检测框架。

</div>

<div class="tat-info-box">
  <div class="box-title">📖 采集方法</div>
  数据采用<strong>自动化均衡策略</strong>采集：系统依次遍历代理车辆，每辆车记录约 180 秒的驾驶数据，然后切换到下一辆。共运行 6 个批次，每次使用不同的车流密度和车速范围，以保证行为多样性。所有采集参数均记录在 <code>batch_summary.json</code> 和 <code>auto_collect_config.json</code> 中，确保完全可复现。
</div>

---

## 详细统计

| 统计项 | 数值 |
|-------|-------|
| **采集次数** | 61 |
| **车辆数** | 61（每车 1 次采集） |
| **RGB 相机数** | 每车 7 台 |
| **每相机帧数** | 3,504 |
| **每次采集帧数** | 65 |
| **RGB 图像总数** | 24,528 |
| **实例掩码** | 24,528（100% 配对） |
| **COCO 标注** | 27,763 |
| **平均标注数/张** | 1.13 |
| **平均采集时长** | ~180 秒 |
| **图像分辨率** | 800 × 600 像素 |
| **采集日期** | 2026‑05‑06 19:26 (UTC+8) |
| **采集持续时间** | ~3 小时（6 批次） |
| **仿真器** | CARLA（同步模式，`fixed_delta_seconds`） |
| **地图** | QingShiLing.xodr——三车道隧道 |

---

## 可视化概览

<div class="tat-img-grid cols-2" style="margin-top:0">
  <div class="tat-img-figure">
    <img src="/multi-view-grid.png" alt="全部 7 相机" loading="lazy" />
    <div class="caption">7 相机同步视图——proxy_1796，帧 14253598</div>
  </div>
  <div class="tat-img-figure">
    <img src="/instance-comparison.png" alt="实例分割对比" loading="lazy" />
    <div class="caption">RGB 与实例分割掩码对比——自车、前向、右前</div>
  </div>
  <div class="tat-img-figure">
    <img src="/coco-detection-front.png" alt="COCO 检测 前向——9 辆车" loading="lazy" />
    <div class="caption">COCO 2D 检测——前向视图（9 辆车）</div>
  </div>
  <div class="tat-img-figure">
    <img src="/coco-detection-front_left.png" alt="COCO 检测 左前——9 辆车" loading="lazy" />
    <div class="caption">COCO 2D 检测——左前视图（9 辆车）</div>
  </div>
</div>

---

## 新闻

- **2026‑05‑06**——v2.0 发布。6 批次共 61 辆车，24.5K RGB 图像 + 实例掩码 + 27.7K COCO 标注框。提供全局合并 `coco_annotations.json` 供检测框架使用。
- **2026‑05‑06（更早）**——v1.0 试点批次。84 辆车，13K 图像（已被 v2.0 取代）。

## 快速链接

- **[数据格式](./data-format)**——完整 Schema、COCO 标注（每次采集 + 全局合并）和字段表。
- **[传感器](./sensors)**——相机规格、实例分割和多视角示例。
- **[下载](./download)**——数据包及训练/验证/测试划分（按采集次 70/15/15）。
- **[开发工具包](./devkit)**——Python 代码片段：读取、校验和合并 COCO。
- **[任务](./tasks)**——行为克隆、2D 检测、实例分割基准。

## 引用

```bibtex
@misc{tat2026,
  title        = {{TunnelAutopilot-Tunnel (TAT) Dataset}},
  author       = {{TAT Dataset Contributors}},
  year         = {2026},
  howpublished = {\url{https://your-website-url}},
  note         = {61‑vehicle multi‑view tunnel driving dataset with 24K+ instance masks and 27K+ COCO 2D annotations.},
}
```

<br />

---

<div style="text-align:center;color:var(--tat-text-light);font-size:.9rem;margin-top:2rem">
  <em>灵感源自 KITTI Vision Benchmark Suite——追求清晰性、可复现性和学术严谨性。</em>
</div>
