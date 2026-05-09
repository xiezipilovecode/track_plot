# 下载

数据包按组件划分。数据反映 **v2.0 版本**（61 辆车，24,528 张图像）。

::: warning 使用条款
所有下载均需接受数据集许可协议（仅限研究/非商业用途）。详见 [许可与引用](./license)。
:::

---

## 数据集规模（v2.0）

| 指标 | 数值 |
|--------|-------|
| 车辆（代理角色） | 61 |
| 运行次数 | 61 |
| RGB 相机 | 每车 7 个 |
| 每相机帧数 | 3,504 |
| 每次运行帧数 | 65 |
| **Total RGB images** | **24,528** |
| **Instance masks** | **24,528** |
| **COCO annotations** | **27,763** |
| 平均运行时长 | ~180 秒 |
| 采集日期 | 2026‑05‑06 19:26 (UTC+8) |
| 采集批次 | 6（不同密度/速度） |

---

## 数据包

| 数据包 | 内容 | 说明 |
|---------|---------|-------|
| **TAT‑Images‑RGB** | 所有 `images/` —— 7 相机 × 3,504 帧 | 24,528 张 PNG（800×600）。建议每次运行一个归档文件 |
| **TAT‑Instance‑Masks** | 所有 `images/*_instance/` —— 7 相机 × 3,504 掩码 | 24,528 张 PNG 掩码，100% 与 RGB 配对 |
| **TAT‑Labels** | `labels.jsonl`、`metadata.json`、`target.json`（×61 次运行） | 每次运行约 KB 级 |
| **TAT‑COCO** | `coco_annotations.json`（全局合并）+ 每次运行 `labels_2d/*.json` | 27,763 个边界框，1 个类别（vehicle） |
| **TAT‑Config** | `effective_config.json`、`batch_summary.json`、`auto_collect_config.json` | 可复现性快照 |

---

## 划分策略

按**运行次数**划分，而非按帧划分：

✅ **正确**：`run_20260506_163611` → 整个划入训练集  
❌ **错误**：同一次运行的部分帧在训练集，其余在验证集

推荐划分（61 次运行）：

| 划分 | 运行次数 | 帧数（每相机） | 占比 |
|-------|------|---------------------|---|
| 训练集 | 43 | ~2,795 | ~70% |
| 验证集 | 9 | ~585 | ~15% |
| 测试集 | 9 | ~585 | ~15% |

---

## 校验

```bat
conda activate carla
cd /d E:\code\track_plot

python -m tp_tunnel_traffic.validate_dataset_run ^
  --run-dir dataset\proxy_<id>\run_YYYYmmdd_HHMMSS
```

---

## 下载状态

| 数据包 | 状态 |
|------|--------|
| TAT‑Images‑RGB | 🟡 准备中 |
| TAT‑Instance‑Masks | 🟡 准备中 |
| TAT‑Labels | 🟡 准备中 |
| TAT‑COCO | 🟡 准备中 |
| TAT‑Config | 🟡 准备中 |
