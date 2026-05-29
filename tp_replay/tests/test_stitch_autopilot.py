"""拼接轨迹回放——直接 teleport 驱动，简单可靠。"""

from __future__ import annotations
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tp_replay import config
from tp_replay.carla_compat import require_carla
from tp_replay.engine import ReplayEngine

def main():
    carla=require_carla()
    js=os.getenv("TP_STITCH_JSON_PATH","")
    if not js: print("set TP_STITCH_JSON_PATH"); return
    c=carla.Client(os.getenv("TP_CARLA_HOST","localhost"),int(os.getenv("TP_CARLA_PORT","2000")))
    c.set_timeout(15.0); w=c.get_world()
    s=w.get_settings(); s.synchronous_mode=True; s.fixed_delta_seconds=0.05; w.apply_settings(s)

    eng=ReplayEngine(c,config.XODR_PATH)
    anchor=os.getenv("TP_DATA_FILE_PATH",config.DATA_FILE_PATH)
    if anchor and os.path.exists(anchor): eng.process_data(anchor)

    ca=eng._stitch_cos_a; sa=eng._stitch_sin_a
    ox=eng._stitch_off_x; oy=eng._stitch_off_y
    ds=eng._data_start_point
    sc=float(config.DATA_SCALE); sy=float(config.SCALE_Y)

    nth=int(os.getenv("TP_STITCH_NTH","1"))
    with open(js,"r",encoding="utf-8") as f: data=json.load(f)
    all_trajs=data.get("trajectories",data if isinstance(data,list) else [])
    candidates=[t for t in sorted(all_trajs,key=lambda x:x["quality_score"],reverse=True)
                 if t["camera_count"]>=5 and t["quality_score"]>0.7]
    st_data=candidates[min(nth-1,len(candidates)-1)] if candidates else None
    if not st_data: print("无轨迹"); return
    nodes=st_data["nodes"]
    print(f"轨迹:{st_data['trajectory_id']} Q={st_data['quality_score']:.3f} {st_data['vehicle_type']} {len(nodes)}节点")

    # ── 所有节点 → CARLA 世界坐标（路点吸附） ──
    wpts=[]
    for n in nodes:
        y=n["y"]; x=n.get("x") or ds[0]
        rx=(x-ds[0])*sc; ry=(y-ds[1])*sc*sy
        loc=carla.Location(rx*ca-ry*sa+eng.entry_loc.x+ox, rx*sa+ry*ca+eng.entry_loc.y+oy, eng.entry_loc.z)
        try:
            wp=eng.map.get_waypoint(loc,project_to_road=True,lane_type=carla.LaneType.Driving)
            if wp: loc=wp.transform.location
        except: pass
        spd=n.get("speed") or 0
        wpts.append((loc, spd, n.get("camera","INTERP"), n["timestamp"]))

    # 画轨迹
    CAM={"TV023":(231,76,60),"TV024":(52,152,219),"TV025":(46,204,113),
         "TV026":(155,89,182),"TV027":(243,156,18),"TV028":(211,84,0)}
    w.tick(); prev=None
    for pt,_,cam,_ in wpts:
        rgb=CAM.get(cam,(180,180,180))
        if prev: w.debug.draw_line(prev,pt,thickness=0.06,color=carla.Color(*rgb),life_time=600.0)
        prev=pt

    # spawn
    bp=w.get_blueprint_library().find("vehicle.tesla.model3")
    if bp is None: bp=w.get_blueprint_library().filter("vehicle.*")[0]
    bp.set_attribute("role_name","test")
    p0=wpts[0][0]; p1=wpts[1][0]
    yaw0=math.degrees(math.atan2(p1.y-p0.y,p1.x-p0.x))
    v=w.try_spawn_actor(bp,carla.Transform(p0,carla.Rotation(yaw=yaw0)))
    for off in [5,10,20] if v is None else []:
        pp=carla.Location(p0.x,p0.y+off,p0.z)
        v=w.try_spawn_actor(bp,carla.Transform(pp,carla.Rotation(yaw=yaw0)))
    if v is None: print("spawn fail"); return

    # Spectator
    spec=w.get_spectator()
    spec.set_transform(carla.Transform(
        carla.Location(p0.x,p0.y,p0.z+8),carla.Rotation(pitch=-20,yaw=yaw0)))

    # ── 逐 tick 插值回放（平滑无跳变） ──
    # 预计算段距和速度
    segs=[]
    for i in range(len(wpts)-1):
        d=wpts[i][0].distance(wpts[i+1][0])
        spd=max(0.1,(wpts[i][1]+wpts[i+1][1])/2)/3.6
        segs.append((d,spd))
    total_d=sum(s[0] for s in segs)

    print(f"总距离:{total_d:.0f}m | 回放中 (Ctrl+C 退出)")
    sim_dist=0.0; idx=0
    smooth_yaw=yaw0

    try:
        while idx<len(segs):
            w.tick()
            seg_d,seg_spd=segs[idx]
            step=seg_spd*0.05
            sim_dist+=step

            while idx<len(segs) and sim_dist>seg_d:
                sim_dist-=seg_d; idx+=1
                if idx<len(segs): seg_d,seg_spd=segs[idx]

            if idx>=len(segs): break
            alpha=max(0,min(1,sim_dist/max(seg_d,0.01)))
            p0,p1=wpts[idx][0],wpts[min(idx+1,len(wpts)-1)][0]
            ix=p0.x+(p1.x-p0.x)*alpha; iy=p0.y+(p1.y-p0.y)*alpha

            raw_yaw=math.degrees(math.atan2(p1.y-p0.y,p1.x-p0.x))
            diff=(raw_yaw-smooth_yaw+180)%360-180
            smooth_yaw+=diff*0.3

            v.set_transform(carla.Transform(carla.Location(ix,iy,p0.z),carla.Rotation(yaw=smooth_yaw)))

            if int(sim_dist*10)%20==0:
                pct=sum(s[0] for s in segs[:idx])/total_d*100
                print(f"  {pct:.0f}% {seg_spd*3.6:.0f}km/h",end="\r")

        print("\n完成 | Ctrl+C 退出")
        while True: w.tick()
    except KeyboardInterrupt: print("\n退出")
    finally:
        if v.is_alive: v.destroy()
        w.apply_settings(w.get_settings())

if __name__=="__main__": main()
