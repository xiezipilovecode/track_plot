# 隧道数据集网站（VitePress / Vue 3）设计文档

> 目标：基于 `dataset/` 目录中 **TunnelAutopilot-Tunnel (TAT)** 数据格式，建设一个风格参考 KITTI（https://www.cvlibs.net/datasets/kitti/）的“学术数据集发布站”。

## 1. 定位与原则

### 1.1 站点定位
- **学术数据集主页**：清晰解释数据内容/组织方式/标注格式/可用任务，并提供下载、引用与开发工具。
- **静态站为主**：优先可被搜索引擎收录（SEO），便于长期维护与部署。

### 1.2 设计原则（对标 KITTI）
- 信息密度高、页面朴素、少动效：**“像论文附录一样可靠”**。
- 结构稳定：导航栏固定，内容页以“章节 + 表格 + 代码块”为主。
- 复现友好：下载、数据对齐、字段说明必须“可抄到代码里”。

---

## 2. 数据集概览（从 dataset/README.md 抽象）

### 2.1 Dataset Card（网站展示字段）
- Name：TunnelAutopilot‑Tunnel (TAT)
- Domain：CARLA tunnel, 3‑lane traffic
- Collection mode：proxy‑only（GUI 选择代理车作为采集目标）
- Sensors：7×RGB cameras + 可选 instance segmentation cameras
- Supervision：control + ego state + 可选 COCO 2D bbox/instance mask
- Primary key：`world_frame`
- Recommended tasks：Behavior Cloning、speed/yaw regression、控制质量分析

### 2.2 目录结构（run 组织）
数据按 `proxy_<actor_id>/run_YYYYmmdd_HHMMSS/` 组织：

```text
dataset/
  proxy_<actor_id>/
    run_YYYYmmdd_HHMMSS/
      images/
        ego/
        front/
        front_left/
        front_right/
        left/
        right/
        rear/
        ego_instance/       (optional)
        front_instance/     (optional)
      labels.jsonl
      labels_2d/
        coco_instances.json
      metadata.json
      target.json
      effective_config.json (optional)
      actors.json           (optional)
```

对齐主键：`world_frame`（图像命名 `images/<camera>/<frame:06d>.png`）。

---

## 3. 信息架构（IA）与路由

参考 KITTI 的“顶层栏目 + 内容页”结构，建议 7 个一级栏目：

1. **Home**（首页）
2. **Sensors**（采集平台/传感器）
3. **Data Format**（数据格式）
4. **Tasks**（任务与基准）
5. **Download**（下载）
6. **DevKit**（开发工具包）
7. **License & Citation**（许可与引用）

对应 VitePress 路由（建议）：

```text
/
/sensors
/data-format
/tasks
/download
/devkit
/license
```

---

## 4. 页面设计（内容草案）

### 4.1 Home（/）
**目标**：让访问者 10 秒内理解“这是什么、能做什么、怎么引用、怎么拿到数据”。

模块建议：
- 顶部：数据集名称 + 一句话简介
- Highlights：数据集特性要点（camera‑only、多视角、隧道难点、world_frame 对齐）
- News：版本更新（例如 v1.0 / v1.1）
- Quick Links：Data Format / Download / DevKit
- Citation：BibTeX（占位，后续补齐）

素材建议：
- 放 1 张“多视角拼接示例图”（后续可从任意 `run_*/images/*/*.png` 里抽帧生成）

### 4.2 Sensors（/sensors）
**目标**：解释 7 个相机的含义、命名与 metadata 字段。

内容：
- 相机列表：`ego/front/front_left/front_right/left/right/rear`
- `metadata.json -> cameras[]` 字段表：
  - name
  - pose（x,y,z,pitch,yaw,roll，相对车体）
  - imaging（width,height,fov）
- Instance Segmentation 相机规则：`TT_COLLECT_INSTANCE_SUFFIX`，输出到 `images/<cam>_instance/`

### 4.3 Data Format（/data-format）
**目标**：把 `dataset/README.md` 里最关键的 schema 变成网页版“权威说明”。

章节：
1) Directory Structure（目录树）
2) Synchronization（对齐规则与伪代码）
3) labels.jsonl（逐行字段表 + 示例 JSON）
4) metadata.json（run 级元信息字段表）
5) effective_config.json / actors.json（可选文件说明）
6) labels_2d/coco_instances.json（COCO 输出的约定）
7) Limitations（对标 KITTI/nuScenes 缺口：timestamp、标定矩阵、3D 标注等）

建议配套组件：
- “代码块复制”按钮（VitePress 默认支持）
- 字段表格（Markdown 表格足够；后续可用自定义组件增强）

### 4.4 Tasks（/tasks）
**目标**：声明推荐任务、指标与 baseline 建议（可先写“占位 + 未来扩展点”）。

建议内容（v1.0 最小集）：
- Behavior Cloning：输入（单/多相机）→ 输出（steer/throttle/brake）
- 辅助回归：speed_mps、vehicle_yaw
- 可选感知：2D vehicle detection（COCO）、instance segmentation

若未来做 leaderboard：
- 提供提交格式（results JSON）+ 评价脚本链接 + 排行榜表格

### 4.5 Download（/download）
**目标**：清晰列出下载包与校验方式。

建议分包（示例）：
- TAT‑Images‑RGB（按 run 打包）
- TAT‑Labels（labels.jsonl / metadata.json / target.json）
- TAT‑Vision‑Optional（COCO / instance masks）

配套：
- SHA256 校验值
- 使用条款简述（跳转 License 页面）
- “按 run 粒度切分”的建议（避免泄漏）

### 4.6 DevKit（/devkit）
**目标**：给出最小可用的 Python 读取示例与可视化思路。

内容建议：
- 读取 run：枚举 `proxy_*/run_*`
- 读取 labels：逐行解析 JSONL（按 world_frame 对齐图片）
- 可视化：给定相机名与 frame，拼图/抽样导出
- 校验：引用现有校验工具（`python -m tp_tunnel_traffic.validate_dataset_run --run-dir ...`）

注：DevKit 可先只提供“示例代码块”，后续再单独开仓库或放到本仓库 `tools/`。

### 4.7 License & Citation（/license）
**目标**：明确许可与引用。

内容：
- License（当前 dataset README 标注“未声明”，建议尽快补齐：非商业/研究用途/禁止再分发等）
- Citation：BibTeX（占位）
- Contact：维护者邮箱/主页（占位）

---

## 5. 站点实现方案（VitePress）

### 5.1 技术栈
- Vue 3 + VitePress
- Markdown 为主，少量自定义组件（可选）

### 5.2 代码组织（建议）
```text
website/
  docs/
    .vitepress/
      config.mts
      theme/
        index.ts
        custom.css
    index.md
    sensors.md
    data-format.md
    tasks.md
    download.md
    devkit.md
    license.md
  package.json
  .gitignore
```

### 5.3 UI/样式（复刻 KITTI 的“学术风”）
- 颜色：减少彩色块，强调链接与标题层级
- 版心：偏窄（阅读友好）
- 字体：系统默认 + 清晰的等宽字体用于代码块

---

## 6. 内容生产流程（建议）

### 6.1 从真实数据抽样生成示例图（后续工作）
- 选择一个代表性 run（例如 `dataset/proxy_299/run_20260506_003936`）
- 抽取若干 world_frame（稀疏采样），生成：
  - 单相机示例
  - 多相机拼接示例（front/left/right/rear）
  - instance mask 对比示例
- 将示例图放到 `website/docs/public/samples/`，在页面引用

### 6.2 文档与 schema 的一致性
- 数据格式页直接以 `dataset/README.md` 为“单一事实来源（SSOT）”
- 每次 schema 演进：同步更新 `dataset/README.md` 与网页 `data-format.md`

---

## 7. 部署（建议）

### 7.1 本地预览
```bash
npm install
npm run docs:dev
```

### 7.2 构建与产物
```bash
npm run docs:build
```

### 7.3 托管
- GitHub Pages：输出目录 `website/docs/.vitepress/dist`
- Vercel：直接指向 `website`，build command `npm run docs:build`

---

## 8. 里程碑（建议）

- v0.1：信息架构 + 6 个内容页（占位也可）上线
- v0.2：补齐 Data Format 的字段表与示例图
- v1.0：下载包、校验说明、引用与许可齐全
