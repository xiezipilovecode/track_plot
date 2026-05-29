"""拼接轨迹平滑回放。"""

from __future__ import annotations
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tp_replay import config
from tp_replay.carla_compat import require_carla
from tp_replay.engine import ReplayEngine

CAMS = {"TV023":(231,76,60),"TV024":(52,152,219),"TV025":(46,204,113),
        "TV026":(155,89,182),"TV027":(243,156,18),"TV028":(211,84,0),"INTERP":(160,160,160)}

def main():
    carla = require_carla()
    js=os.getenv("TP_STITCH_JSON_PATH","")
    if not js: print("set TP_STITCH_JSON_PATH"); return
    c=carla.Client(os.getenv("TP_CARLA_HOST","localhost"),int(os.getenv("TP_CARLA_PORT","2000")))
    c.set_timeout(15.0); w=c.get_world()
    s=w.get_settings(); s.synchronous_mode=True; s.fixed_delta_seconds=0.05; w.apply_settings(s)

    eng=ReplayEngine(c,config.XODR_PATH)
    anchor=os.getenv("TP_DATA_FILE_PATH",config.DATA_FILE_PATH)
    if anchor and os.path.exists(anchor): eng.process_data(anchor)

    ca=getattr(eng,"_stitch_cos_a",1.0); sa=getattr(eng,"_stitch_sin_a",0.0)
    ox=getattr(eng,"_stitch_off_x",0.0); oy=getattr(eng,"_stitch_off_y",0.0)
    ds=getattr(eng,"_data_start_point",(0.0,0.0))
    sc=float(config.DATA_SCALE); sy=float(config.SCALE_Y)

    with open(js,"r",encoding="utf-8") as f: data=json.load(f)
    # 选第 N 条高质量轨迹（默认第 1 条，可通过 TP_STITCH_NTH 环境变量切换）
    nth = int(os.getenv("TP_STITCH_NTH","1"))
    candidates=[t for t in sorted(data["trajectories"],key=lambda x:x["quality_score"],reverse=True)
                if t["camera_count"]>=6 and t["quality_score"]>0.85]
    st=candidates[min(nth-1,len(candidates)-1)] if candidates else None
    if not st: print("无轨迹"); return
    nodes=st["nodes"]
    print(f"轨迹: {st['trajectory_id']} Q={st['quality_score']:.3f} {st['vehicle_type']} {len(nodes)}节点")

    real_xs=[(i,n.get("x")) for i,n in enumerate(nodes) if n.get("x") and abs(float(n.get("x",0)))>=1.0]
    for i,n in enumerate(nodes):
        if n.get("x") is None or abs(float(n.get("x",0)))<1.0:
            before=[rx for rx in real_xs if rx[0]<i]; after=[rx for rx in real_xs if rx[0]>i]
            if before and after:
                bi,bx=before[-1]; ai,ax=after[0]; n["x"]=float(bx)+(float(ax)-float(bx))*(i-bi)/(ai-bi)
            elif before: n["x"]=float(before[-1][1])
            elif after: n["x"]=float(after[0][1])

    def tw(x,y):
        rx=(x-ds[0])*sc; ry=(y-ds[1])*sc*sy
        return carla.Location(rx*ca-ry*sa+eng.entry_loc.x+ox, rx*sa+ry*ca+eng.entry_loc.y+oy, eng.entry_loc.z+1.0)

    raw_pts=[]; snapped=[]
    for n in nodes:
        y=n["y"]; x=n.get("x") or ds[0]
        raw=tw(float(x),float(y)); raw_pts.append(raw)
        snap=raw
        try:
            wp=eng.map.get_waypoint(raw,project_to_road=True,lane_type=carla.LaneType.Driving)
            if wp: snap=wp.transform.location; snap.z+=0.5
        except: pass
        snapped.append((snap, n.get("camera","INTERP"), n.get("speed",0)))

    segs=[]
    for i in range(len(raw_pts)-1):
        d=raw_pts[i].distance(raw_pts[i+1])
        spd_a=snapped[i][2]; spd_b=snapped[i+1][2]
        segs.append((d, max(0.1,spd_a), max(0.1,spd_b)))
    total_dist=sum(s[0] for s in segs)
    print(f"总距离: {total_dist:.0f}m, {len(segs)}段")

    w.tick(); pl=None
    for pt,cam,_ in snapped:
        rgb=CAMS.get(cam,(180,180,180))
        if pl: w.debug.draw_line(pl,pt,thickness=0.04,color=carla.Color(*rgb),life_time=10.0)
        pl=pt

    p0=snapped[0][0]; p1=snapped[1][0]
    yaw0=(math.degrees(math.atan2(p1.y-p0.y,p1.x-p0.x))+180)%360
    bp=w.get_blueprint_library().find("vehicle.tesla.model3")
    if bp is None: bp=w.get_blueprint_library().filter("vehicle.*")[0]
    bp.set_attribute("role_name","test")
    v=w.try_spawn_actor(bp,carla.Transform(p0,carla.Rotation(yaw=yaw0)))
    for off in [5,10,20,50] if v is None else []:
        pp=carla.Location(p0.x,p0.y+off,p0.z)
        v=w.try_spawn_actor(bp,carla.Transform(pp,carla.Rotation(yaw=yaw0)))
    if v is None: print("spawn fail"); return

    spec=w.get_spectator()
    sim_dist=0.0; idx=0; smooth_yaw=yaw0; completed=False

    try:
        while not completed:
            w.tick()
            if idx>=len(segs): completed=True; break

            seg_d,spd_a,spd_b=segs[idx]
            alpha0=sim_dist/max(seg_d,0.001)
            cur_spd_kmh=spd_a+(spd_b-spd_a)*min(1.0,alpha0)
            step=cur_spd_kmh/3.6*0.05
            sim_dist+=step

            while idx<len(segs) and sim_dist>seg_d:
                sim_dist-=seg_d; idx+=1
                if idx<len(segs): seg_d,spd_a,spd_b=segs[idx]
            if idx>=len(segs): completed=True
            i=min(idx,len(segs)-1)
            alpha=max(0,min(1,sim_dist/max(seg_d,0.001)))
            sp0,_,_=snapped[i]; sp1,_,_=snapped[min(i+1,len(snapped)-1)]

            ix=sp0.x+(sp1.x-sp0.x)*alpha; iy=sp0.y+(sp1.y-sp0.y)*alpha
            # yaw: 路点方向 + 180（REVERSE_DIRECTION 修正）
            try:
                wp=eng.map.get_waypoint(carla.Location(ix,iy,sp0.z),
                    project_to_road=True,lane_type=carla.LaneType.Driving)
                raw_yaw=(wp.transform.rotation.yaw+180)%360 if wp else 0
            except: raw_yaw=0

            diff=(raw_yaw-smooth_yaw+180)%360-180; smooth_yaw+=diff*0.3

            v.set_transform(carla.Transform(
                carla.Location(ix,iy,sp0.z), carla.Rotation(yaw=smooth_yaw)))

            rad=math.radians(smooth_yaw)
            spec.set_transform(carla.Transform(
                carla.Location(ix-math.cos(rad)*8,iy-math.sin(rad)*8,sp0.z+4),
                carla.Rotation(pitch=-8,yaw=smooth_yaw)))

            if int(sim_dist*10)%20==0:
                pct=sum(s[0] for s in segs[:idx])/total_dist*100
                print(f"  {pct:.0f}% {cur_spd_kmh:.0f}km/h yaw={smooth_yaw:.0f}",end="\r")

        print("\n完成 | Ctrl+C 退出")
        while True: w.tick()
    except KeyboardInterrupt: print("\n退出")
    finally:
        if v.is_alive: v.destroy()
        w.apply_settings(w.get_settings())

if __name__=="__main__": main()
