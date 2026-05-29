"""拼接轨迹回放测试 — 使用引擎原生 kinematic 驱动。"""

from __future__ import annotations
import json, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tp_replay import config
from tp_replay.carla_compat import require_carla
from tp_replay.engine import ReplayEngine
from tp_replay.stitch_adapter import StitchAdapter

CAMS = {"TV023":(231,76,60),"TV024":(52,152,219),"TV025":(46,204,113),
        "TV026":(155,89,182),"TV027":(243,156,18),"TV028":(211,84,0),"INTERP":(160,160,160)}

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

    # 加载拼接轨迹（全帧模式）
    adapter=StitchAdapter(eng)
    nth=int(os.getenv("TP_STITCH_NTH","1"))
    tracks=adapter.load_for_kinematic(js, min_quality=0.7, min_cameras=5)
    if not tracks: print("无轨迹"); return
    st=tracks[min(nth-1,len(tracks)-1)]
    print(f"轨迹: {st.id} frames={len(st.frames)} type={st.type_str} duration={st.end_time-st.start_time:.1f}s")

    # 画轨迹线
    pl=None
    cam=lambda n: n.get("camera","INTERP")  # dummy for debug draw
    # 直接用引擎的 frames（已吸附到路点）
    for f in st.frames:
        rgb=(180,180,180)
        if pl: w.debug.draw_line(pl,f.loc,thickness=0.04,color=carla.Color(*rgb),life_time=10.0)
        pl=f.loc

    # 注入引擎
    st.next_spawn_time=-1.0  # 立即生成
    eng.pending_tracks=[st]
    eng.active_tracks=[]

    spec=w.get_spectator(); tick=0; dt=float(s.fixed_delta_seconds)
    try:
        while True:
            w.tick(); active=eng.tick(dt); tick+=1
            # spectator 跟随
            for t in eng.active_tracks:
                if t.actor and t.actor.is_alive:
                    tr=t.actor.get_transform()
                    spec.set_transform(carla.Transform(
                        carla.Location(tr.location.x-tr.get_forward_vector().x*8,
                                       tr.location.y-tr.get_forward_vector().y*8,
                                       tr.location.z+4),
                        carla.Rotation(pitch=-8,yaw=tr.rotation.yaw)))
            if tick%20==0:
                print(f"  t={eng.current_traj_time:.1f}s active={active}",end="\r")
            if active==0 and tick>10 and eng.current_traj_time>st.end_time:
                print("\n完成"); break
    except KeyboardInterrupt: print("\n退出")
    finally:
        for t in eng.active_tracks:
            if t.actor and t.actor.is_alive: t.actor.destroy()
        w.apply_settings(w.get_settings())

if __name__=="__main__": main()
