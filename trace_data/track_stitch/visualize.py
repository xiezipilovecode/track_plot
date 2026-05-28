"""拼接轨迹可视化工具。

生成 4 面板综合可视化：
  1. 时距图（时间-y坐标，按摄像头着色）
  2. 摄像头覆盖分布（柱状图）
  3. 质量评分分布（直方图）
  4. 示例拼接轨迹（放大展示前 N 条高质量轨迹）

用法：
    python -m trace_data.track_stitch.visualize
    python -m trace_data.track_stitch.visualize --sample 500 --output my_plot.png
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter

import matplotlib
matplotlib.use("Agg")  # 无 GUI 后端
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import matplotlib.font_manager as fm
import numpy as np
import os

# ── 中文字体设置（Windows 环境） ──
_CN_FONT = None
# 按优先级尝试常见中文字体
for _font_name in ["Microsoft YaHei", "SimHei", "SimSun", "KaiTi"]:
    for _fp in fm.findSystemFonts():
        if _font_name.lower().replace(" ", "") in os.path.basename(_fp).lower().replace(" ", ""):
            _CN_FONT = fm.FontProperties(fname=_fp)
            break
    if _CN_FONT is not None:
        break

if _CN_FONT is not None:
    plt.rcParams["font.family"] = _CN_FONT.get_name()
else:
    # 兜底：直接扫描所有字体找含中文的
    for _fp in fm.findSystemFonts():
        try:
            _prop = fm.FontProperties(fname=_fp)
            _name = _prop.get_name()
            if any(kw in _name.lower() for kw in ["hei", "song", "kai", "ming", "yahei", "chinese"]):
                _CN_FONT = _prop
                plt.rcParams["font.family"] = _name
                break
        except Exception:
            continue

plt.rcParams["axes.unicode_minus"] = False
print(f"中文字体: {_CN_FONT.get_name() if _CN_FONT else '未找到（标签将显示为英文）'}")

# ── 配色（按摄像头） ──
CAMERA_COLORS = {
    "TV023": "#e41a1c",  # 红
    "TV024": "#377eb8",  # 蓝
    "TV025": "#4daf4a",  # 绿
    "TV026": "#984ea3",  # 紫
    "TV027": "#ff7f00",  # 橙
    "TV028": "#a65628",  # 棕
    "INTERP": "#cccccc",  # 灰（插值点）
}

CAMERA_LABELS = {
    "TV023": "TV023 (K615+703)",
    "TV024": "TV024 (K615+833)",
    "TV025": "TV025 (K615+963)",
    "TV026": "TV026 (K616+093)",
    "TV027": "TV027 (K616+223)",
    "TV028": "TV028 (K616+323)",
    "INTERP": "插值点",
}


def load_trajectories(json_path: str, sample: int = 0):
    """加载拼接轨迹 JSON。sample=0 表示全部加载。"""
    print(f"加载 {json_path} ...")
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    trajs = data.get("trajectories", [])
    if sample > 0 and len(trajs) > sample:
        # 按质量评分降序采样
        trajs_sorted = sorted(trajs, key=lambda t: t.get("quality_score", 0), reverse=True)
        # 同时保留一些低分样本
        high_n = min(sample // 2, len(trajs_sorted))
        low_n = min(sample - high_n, len(trajs_sorted))
        trajs = trajs_sorted[:high_n] + trajs_sorted[-low_n:]

    print(f"  加载 {len(trajs)} 条轨迹")
    return trajs


def plot_all(trajs: list, output_path: str, dpi: int = 150, use_english: bool = False):
    """生成 2×2 综合可视化。"""
    fig, axes = plt.subplots(2, 2, figsize=(18, 12))

    title = "Tunnel 6-Camera Trajectory Stitching Visualization" if use_english \
            else "隧道六路摄像头轨迹拼接结果可视化"
    fig.suptitle(title, fontsize=16, fontweight="bold", y=0.98)

    # ── Panel 1: 时距图 ──
    ax1 = axes[0, 0]
    _plot_time_space(ax1, trajs, use_english)

    # ── Panel 2: 摄像头覆盖分布 ──
    ax2 = axes[0, 1]
    _plot_camera_distribution(ax2, trajs, use_english)

    # ── Panel 3: 质量评分分布 ──
    ax3 = axes[1, 0]
    _plot_quality_distribution(ax3, trajs, use_english)

    # ── Panel 4: 精选轨迹放大 ──
    ax4 = axes[1, 1]
    _plot_featured_trajectories(ax4, trajs, use_english)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(output_path, dpi=dpi, bbox_inches="tight")
    print(f"图表已保存: {output_path}")
    plt.close()


# ═══════════════════════════════════════════════════════════
#  Panel 1: 时距图
# ═══════════════════════════════════════════════════════════

def _plot_time_space(ax, trajs, use_english=False):
    """时距图：X=时间(相对秒), Y=y坐标(cm), 每条线=一辆车。"""
    # 使用全局最早时间作为零点
    all_ts = []
    for t in trajs:
        for n in t.get("nodes", []):
            all_ts.append(n["timestamp"])
    if not all_ts:
        return
    t0 = min(all_ts)

    # 限制绘图轨迹数（避免过密）
    max_plot = min(2000, len(trajs))
    plotted = 0

    for t in trajs[:max_plot]:
        nodes = t.get("nodes", [])
        if len(nodes) < 2:
            continue

        times = [(n["timestamp"] - t0) / 1000.0 for n in nodes]  # 相对秒
        ys = [n["y"] for n in nodes]
        cams = [n.get("camera", "INTERP") for n in nodes]

        # 按摄像头分段绘制（不同颜色）
        seg_start = 0
        for i in range(1, len(cams) + 1):
            if i == len(cams) or cams[i] != cams[seg_start]:
                cam = cams[seg_start]
                color = CAMERA_COLORS.get(cam, "#999999")
                alpha = 0.3 if cam == "INTERP" else 0.7
                lw = 0.5 if cam == "INTERP" else 1.2
                ax.plot(times[seg_start:i], ys[seg_start:i],
                        color=color, alpha=alpha, linewidth=lw)
                seg_start = i

        plotted += 1
        if plotted >= max_plot:
            break

    # 标注摄像头 y 坐标范围
    cam_ranges = {
        "TV023": 61_571_000, "TV024": 61_585_000,
        "TV025": 61_597_000, "TV026": 61_610_000,
        "TV027": 61_623_000, "TV028": 61_633_000,
    }
    for cam, y_pos in cam_ranges.items():
        ax.axhline(y=y_pos, color=CAMERA_COLORS.get(cam, "#999"),
                   linestyle="--", linewidth=0.6, alpha=0.4)

    # 图例
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=CAMERA_COLORS[c],
              label=CAMERA_LABELS.get(c, c) if not use_english else c)
        for c in ["TV023", "TV024", "TV025", "TV026", "TV027", "TV028", "INTERP"]
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=7, ncol=2)

    ax.set_xlabel("Time (relative seconds)" if use_english else "时间（相对秒）", fontsize=10)
    ax.set_ylabel("Y Coordinate (cm)" if use_english else "y 坐标（cm）", fontsize=10)
    ax.set_title(
        f"Time-Space Diagram ({min(len(trajs), max_plot)} trajectories)" if use_english
        else f"时距图（{min(len(trajs), max_plot)} 条轨迹）",
        fontsize=11, fontweight="bold",
    )
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{x/1e6:.1f}M"))
    ax.grid(True, alpha=0.2)


# ═══════════════════════════════════════════════════════════
#  Panel 2: 摄像头覆盖分布
# ═══════════════════════════════════════════════════════════

def _plot_camera_distribution(ax, trajs, use_english=False):
    """摄像头覆盖数柱状图。"""
    cam_counts = Counter(t.get("camera_count", 0) for t in trajs)
    x, y = sorted(cam_counts.keys()), [cam_counts[k] for k in sorted(cam_counts.keys())]
    colors = ["#fbb4ae" if k <= 2 else "#b3cde3" if k <= 4 else "#ccebc5" for k in x]
    bars = ax.bar(x, y, color=colors, edgecolor="white", linewidth=0.8)
    for bar, val in zip(bars, y):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(y)*0.01,
                f"{val}\n({val/max(1,sum(y))*100:.1f}%)", ha="center", va="bottom", fontsize=8)
    ax.set_xlabel("Camera Count" if use_english else "覆盖摄像头数", fontsize=10)
    ax.set_ylabel("Trajectory Count" if use_english else "轨迹数", fontsize=10)
    ax.set_title("Camera Coverage Distribution" if use_english else "摄像头覆盖分布", fontsize=11, fontweight="bold")
    ax.set_xticks(x); ax.grid(axis="y", alpha=0.2)


def _plot_quality_distribution(ax, trajs, use_english=False):
    """质量评分直方图。"""
    scores = [t.get("quality_score", 0) for t in trajs]
    if not scores: return
    ax.hist(scores, bins=40, color="#6baed6", edgecolor="white", alpha=0.85)
    mean_s, median_s = np.mean(scores), np.median(scores)
    ax.axvline(mean_s, color="#e41a1c", linestyle="--", linewidth=1.5,
               label=f"{'Mean' if use_english else '均值'}={mean_s:.3f}")
    ax.axvline(median_s, color="#377eb8", linestyle="-.", linewidth=1.5,
               label=f"{'Median' if use_english else '中位数'}={median_s:.3f}")
    ax.set_xlabel("Quality Score" if use_english else "质量评分", fontsize=10)
    ax.set_ylabel("Trajectory Count" if use_english else "轨迹数", fontsize=10)
    ax.set_title(f"{'Quality Score Distribution' if use_english else '质量评分分布'} (n={len(scores)})", fontsize=11, fontweight="bold")
    ax.legend(fontsize=8); ax.grid(axis="y", alpha=0.2)


def _plot_featured_trajectories(ax, trajs, use_english=False):
    """选取几条高质量全摄像头轨迹放大展示。"""
    top_trajs = sorted(
        [t for t in trajs if t.get("camera_count", 0) >= 5 and t.get("quality_score", 0) > 0.8],
        key=lambda t: t.get("quality_score", 0), reverse=True)[:8]
    if not top_trajs:
        top_trajs = sorted(trajs, key=lambda t: t.get("quality_score", 0), reverse=True)[:8]
    all_ts, t0 = [], 0
    for t in top_trajs:
        for n in t.get("nodes", []): all_ts.append(n["timestamp"])
    t0 = min(all_ts) if all_ts else 0
    colors = plt.cm.tab10(np.linspace(0, 1, len(top_trajs)))
    for i, t in enumerate(top_trajs):
        nodes = t.get("nodes", [])
        times = [(n["timestamp"] - t0) / 1000.0 for n in nodes]
        ys = [n["y"] for n in nodes]
        label = f"{t['trajectory_id']} (Q={t.get('quality_score',0):.2f} C={t.get('camera_count',0)} {t.get('vehicle_type','?')})"
        ax.plot(times, ys, color=colors[i], linewidth=1.5, alpha=0.85, label=label)
    ax.set_xlabel("Time (relative seconds)" if use_english else "时间（相对秒）", fontsize=10)
    ax.set_ylabel("Y Coordinate (cm)" if use_english else "y 坐标（cm）", fontsize=10)
    title = f"Featured Stitched Trajectories (Top {len(top_trajs)})" if use_english else f"精选拼接轨迹（Top {len(top_trajs)}）"
    ax.set_title(title, fontsize=11, fontweight="bold")
    ax.legend(fontsize=6, loc="lower right")
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{x/1e6:.1f}M"))
    ax.grid(True, alpha=0.2)


# ═══════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="拼接轨迹可视化")
    parser.add_argument("--input", default=None,
                        help="轨迹 JSON 路径（默认使用 output/stitched_trajectories.json）")
    parser.add_argument("--output", default=None,
                        help="输出图片路径（默认 output/stitching_visualization.png）")
    parser.add_argument("--sample", type=int, default=2000,
                        help="采样轨迹数（0=全部，默认 2000）")
    parser.add_argument("--dpi", type=int, default=150,
                        help="输出图片 DPI（默认 150）")
    parser.add_argument("--english", action="store_true",
                        help="使用英文标签（避免中文字体问题）")
    args = parser.parse_args()

    # 默认路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_input = os.path.join(script_dir, "output", "stitched_trajectories.json")
    default_output = os.path.join(script_dir, "output", "stitching_visualization.png")

    input_path = args.input or default_input
    output_path = args.output or default_output

    if not os.path.exists(input_path):
        print(f"错误：找不到输入文件 {input_path}")
        sys.exit(1)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    trajs = load_trajectories(input_path, sample=args.sample)
    plot_all(trajs, output_path, dpi=args.dpi, use_english=args.english)


if __name__ == "__main__":
    main()
