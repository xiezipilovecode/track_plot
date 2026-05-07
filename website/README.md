# TAT Dataset Website

> 基于 [VitePress](https://vitepress.dev) (Vue 3) 构建的 **TunnelAutopilot‑Tunnel (TAT)** 隧道数据集发布网站。
> 风格参考 [KITTI Vision Benchmark Suite](https://www.cvlibs.net/datasets/kitti/)，追求**学术极简、信息密度高、复现友好**。

---

## 快速开始

```bash
cd website
npm install
npm run docs:dev
```

浏览器访问 `http://localhost:5173`。

---

## 可用命令

| 命令 | 说明 |
|------|------|
| `npm run docs:dev` | 本地开发服务器（热更新） |
| `npm run docs:build` | 生产构建 → `docs/.vitepress/dist/` |
| `npm run docs:preview` | 本地预览生产构建 |

---

## 技术栈

| 层级 | 选型 |
|------|------|
| 框架 | Vue 3 |
| SSG | VitePress 1.x |
| 样式 | 自定义 CSS 变量（学术蓝配色） |
| 托管 | 静态站点（GitHub Pages / Vercel） |

---

## 目录结构

```text
website/
├── README.md
├── DESIGN.md                         # 网站设计文档
├── package.json
├── .gitignore
├── generate_samples.py               # 示例图生成脚本（从真实 run 采样）
│
└── docs/
    ├── index.md                      # 首页（Hero 图 + 统计表 + 4 图画廊）
    ├── sensors.md                     # 传感器页（多视角网格 + RGB/Instance 对比）
    ├── data-format.md                 # 数据格式页（COCO 检测图 + 全局合并 COCO）
    ├── tasks.md                       # 任务页
    ├── download.md                    # 下载页
    ├── devkit.md                      # 开发工具包（含 Auto Collection）
    ├── license.md                     # 许可与引用
    │
    ├── public/
    │   ├── muti-view-sample.png       # 多视角拼接图（Hero 展示）
    │   ├── multi-view-grid.png        # 7 相机网格（由脚本生成）
    │   ├── instance-comparison.png    # RGB vs Instance 对比（由脚本生成）
    │   ├── coco-detection-front.png   # front COCO 检测（9 bboxes）
    │   └── coco-detection-front_left.png  # front_left COCO 检测（9 bboxes）
    │
    └── .vitepress/
        ├── config.mts                 # 导航/侧边栏/页脚/搜索
        └── theme/
            ├── index.ts
            └── custom.css             # 学术蓝主题 + 图片网格样式
```

---

## 数据集版本

| 版本 | 日期 | 规模 |
|------|------|------|
| v2.0 | 2026‑05‑06 | 61 辆车、3,504 帧/相机、24,528 RGB + 24,528 instance masks + 27,763 COCO bboxes |
| v1.0 | 2026‑05‑06 | 84 辆车、1,868 帧/相机、13,076 RGB（已淘汰） |

---

## 网站页面清单

| 页面 | 核心内容 |
|------|---------|
| Home | Hero + 16 行统计表 + 4 图画廊（多视角/Instance/COCO×2） + 6 Feature 卡片 |
| Sensors | 7 相机网格图 + RGB/Instance 三相机对比图 + 相机表格 + Instance 属性表 |
| Data Format | COCO 检测双图 + 全局合并 COCO 章节 + labels.jsonl 字段表 + 目录树 |
| Tasks | BC / 2D 检测 / 实例分割 / 速度回归 结构化说明 + ML 规模数据 |
| Download | 5 个数据包 + 70/15/15 拆分 + 校验命令 |
| DevKit | COCO 合并命令 + 5 段 Python 示例 + Auto Collection 配置表 |
| License | 许可草案 + BibTeX + 联系方式 + CARLA 致谢 |

---

## 如何更新示例图

```bash
cd E:\code\track_plot

# 修改 generate_samples.py 中的 RUN_DIR 和 FRAME_MAIN/FRAME_MIDDLE
& "E:\programs\miniconda\envs\carla\python.exe" "website\generate_samples.py"
```

---

## 部署

```bash
npm run docs:build
# 产出: docs/.vitepress/dist/
```

**Vercel**: Root = `website`, Build = `npm run docs:build`, Output = `docs/.vitepress/dist`

---

## 后续优化

- [x] COCO detection 可视化
- [x] RGB vs Instance 对比图
- [x] 全局合并 COCO (coco_annotations.json) 章节
- [x] 首页 4 图 Visual Overview 画廊
- [ ] 补充下载包实际链接和 SHA256
- [ ] 正式 License 文本
- [ ] 在线排行榜（Leaderboard）

---

## 关联资源

- 主仓库：`E:\code\track_plot`
- 数据集 Schema：`E:\code\track_plot\dataset\README.md`
- 全局 COCO：`E:\code\track_plot\dataset\coco_annotations.json`
- 采集统计：`E:\code\track_plot\dataset\batch_summary.json`
- 网站设计文档：`E:\code\track_plot\website\DESIGN.md`
