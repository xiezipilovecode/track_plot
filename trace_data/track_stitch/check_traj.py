import json
d=json.load(open("trace_data/track_stitch/output/stitched_trajectories.json","r",encoding="utf-8"))
trajs=d["trajectories"]

# 检查多条轨迹
for idx in [0, 1, 2, 5, 10]:
    if idx >= len(trajs): break
    t=trajs[idx]
    nodes=t["nodes"]
    ys=[n["y"] for n in nodes]
    diffs=[ys[i+1]-ys[i] for i in range(len(ys)-1)]
    neg_count=sum(1 for d in diffs if d<0)
    
    tss=[n["timestamp"] for n in nodes]
    ts_diffs=[tss[i+1]-tss[i] for i in range(len(tss)-1)]
    neg_ts=sum(1 for d in ts_diffs if d<=0)
    
    cameras=list(set(n.get("camera","?") for n in nodes))
    print(f"TRAJ[{idx}] {t['trajectory_id']}: nodes={len(nodes)} y_rev={neg_count} ts_rev={neg_ts} cams={sorted(cameras)} Q={t['quality_score']:.3f}")
    if neg_count>0:
        for i,d in enumerate(diffs):
            if d<0:
                print(f"  y反向@i={i}: {ys[i]:.0f}->{ys[i+1]:.0f} cam={nodes[i]['camera']}->{nodes[i+1]['camera']}")
                break
    if neg_ts>0:
        for i,d in enumerate(ts_diffs):
            if d<0:
                print(f"  ts反向@i={i}: {tss[i]}->{tss[i+1]} diff={d}")
                break
