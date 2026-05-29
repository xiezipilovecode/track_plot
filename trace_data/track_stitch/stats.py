import json, numpy as np
from collections import Counter

d = json.load(open(r"trace_data\track_stitch\output\stitched_trajectories.json", "r", encoding="utf-8"))
t = d["trajectories"]
meta = d["metadata"]

print(f"=== 拼接结果统计 ===")
print(f"总轨迹数: {len(t)}")
print(f"来源文件: {meta['source']}")
print()

scores = [x["quality_score"] for x in t]
print(f"质量评分:")
print(f"  最低: {min(scores):.3f}")
print(f"  最高: {max(scores):.3f}")
print(f"  均值: {np.mean(scores):.3f}")
print(f"  中位数: {np.median(scores):.3f}")
print()

cd = Counter(x["camera_count"] for x in t)
print(f"摄像头覆盖分布:")
for k in sorted(cd.keys()):
    print(f"  {k}个摄像头: {cd[k]} 条 ({cd[k]/len(t)*100:.1f}%)")
print()

vt = Counter(x["vehicle_type"] for x in t)
print(f"车辆类型 (Top 5):")
for tp, cnt in vt.most_common(5):
    print(f"  {tp}: {cnt} ({cnt/len(t)*100:.1f}%)")
print()

top = sorted(t, key=lambda x: x["quality_score"], reverse=True)[:5]
print(f"Top 5 高质量轨迹:")
for x in top:
    nodes = x["nodes"]
    real_n = sum(1 for n in nodes if not n["interpolated"])
    interp_n = sum(1 for n in nodes if n["interpolated"])
    ts = [n["timestamp"] for n in nodes]
    dur = (max(ts) - min(ts)) / 1000 if ts else 0
    print(f"  {x['trajectory_id']}: Q={x['quality_score']:.3f} "
          f"cams={x['camera_count']} "
          f"nodes={len(nodes)}(实测{real_n}+插值{interp_n}) "
          f"dur={dur:.1f}s type={x['vehicle_type']}")
