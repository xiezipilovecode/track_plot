"""tp_replay 主入口（从历史 replay_main.py 迁移）。

设计目标：
- 入口尽量“薄”：只做 env 覆盖、日志初始化、CARLA world 设置保护，以及调用 ReplayEngine。
- 保持 import 时无副作用：auto_control_main.py 会延迟导入本模块。
"""

from __future__ import annotations

import io
import json
import logging
import math
import os
import sys
from typing import Optional
from pathlib import Path

from . import config
from .carla_compat import require_carla
from .engine import ReplayEngine, _info
from .env_utils import (
    _ensure_logging_to_file,
    _get_bool_from_env,
    _get_float_from_env,
    _get_float_list_from_env,
    _get_int_from_env,
    _get_speed_factor_from_env,
    _get_str_from_env,
)
from .world_utils import _maybe_generate_opendrive_world


logger = logging.getLogger(__name__)


def _run_stitch_simple(world, client, engine, settings) -> None:
    """拼接轨迹回放——先单车验证环境。"""

    carla = require_carla()
    import json
    stitch_json = os.getenv("TP_STITCH_JSON_PATH") or config.STITCH_JSON_PATH
    if not stitch_json: return
    anchor_file = os.getenv("TP_DATA_FILE_PATH") or config.DATA_FILE_PATH
    if anchor_file and os.path.exists(anchor_file):
        engine.process_data(anchor_file)
        engine.pending_tracks = []
    with open(stitch_json,"r",encoding="utf-8") as f: data=json.load(f)
    trajs=data.get("trajectories",data if isinstance(data,list) else [])
    # 选一条车：高质量 + 单调 y（防 Z 字形轨迹导致运动混乱）
    def _check_monotonic(tr,max_rev=3):
        nds=tr.get("nodes",[])
        return sum(1 for i in range(len(nds)-1) if nds[i+1]["y"]<nds[i]["y"])<=max_rev
    nth=_get_int_from_env("TP_STITCH_NTH",1)
    cand=sorted([t for t in trajs if t["quality_score"]>0.7 and t["camera_count"]>=5 and _check_monotonic(t)],
                key=lambda x:x["quality_score"],reverse=True)
    st=cand[min(nth-1,len(cand)-1)] if cand else None
    if not st: return
    nodes=st["nodes"]
    from .lane_align import cluster_x_to_lanes, assign_lane
    clusters = cluster_x_to_lanes([st])
    # 坐标变换参数防护
    if not hasattr(engine,'_stitch_cos_a') or engine._stitch_cos_a is None:
        _info("错误: engine 未初始化坐标变换参数，请设置 TP_DATA_FILE_PATH 指向锚点文件"); return
    ca=engine._stitch_cos_a; sa=engine._stitch_sin_a; ox=engine._stitch_off_x; oy=engine._stitch_off_y; ds=engine._data_start_point
    sc=float(config.DATA_SCALE); sy=float(config.SCALE_Y)
    # ── 路点构建（CARLA waypoint 网络导航到目标车道）──
    def _nav_lane(base_wp, target_id):
        try:
            cur = int(base_wp.lane_id); tgt = int(target_id); wp = base_wp
            while cur < tgt:
                nxt = wp.get_right_lane()
                if nxt and nxt.lane_type == carla.LaneType.Driving: wp = nxt; cur += 1
                else: break
            while cur > tgt:
                nxt = wp.get_left_lane()
                if nxt and nxt.lane_type == carla.LaneType.Driving: wp = nxt; cur -= 1
                else: break
            return wp
        except Exception: return base_wp

    wpts = []
    last_lane = "-2"
    for n in nodes:
        cam_id = n.get("camera_id", "")
        if cam_id == "INTERP":
            n_lane = last_lane
        elif clusters:
            x_val = n.get("x")
            n_lane = assign_lane(x_val, cam_id, clusters) if x_val is not None else last_lane
            last_lane = n_lane
        else:
            n_lane = None

        y_n = n["y"]; x_n = n.get("x") or ds[0]
        rx = (x_n - ds[0]) * sc; ry = (y_n - ds[1]) * sc * sy
        rough_x = rx * ca - ry * sa + engine.entry_loc.x + ox
        rough_y = rx * sa + ry * ca + engine.entry_loc.y + oy
        rough_loc = carla.Location(rough_x, rough_y, engine.entry_loc.z)

        try:
            base_wp = engine.map.get_waypoint(rough_loc, project_to_road=True,
                                               lane_type=carla.LaneType.Driving)
        except Exception:
            base_wp = None
        if base_wp is None:
            continue
        if rough_loc.distance(base_wp.transform.location) > 20.0:
            continue

        if n_lane is not None and clusters:
            try:
                loc = _nav_lane(base_wp, n_lane).transform.location
            except Exception:
                loc = base_wp.transform.location
        else:
            loc = base_wp.transform.location
        wpts.append((loc, n.get("speed") or 0, n["timestamp"]))
    if len(wpts) < 2: _info("wpts too short"); return
    segs=[]
    for i in range(len(wpts)-1):
        d=wpts[i][0].distance(wpts[i+1][0])
        spd=max(0.1,(wpts[i][1]+wpts[i+1][1])/2)/3.6
        segs.append((d,spd))
    # ── 诊断输出 ──
    _info(f"  DEBUG traj={st['trajectory_id']} Q={st['quality_score']:.3f} nodes={len(nodes)}")
    _info(f"  DEBUG wpts[0]=({wpts[0][0].x:.1f},{wpts[0][0].y:.1f},{wpts[0][0].z:.1f}) entry_z={engine.entry_loc.z:.1f}")
    _info(f"  DEBUG ca={ca:.6f} sa={sa:.6f} ds=({ds[0]:.1f},{ds[1]:.1f}) scale_y={sy}")
    _info(f"  DEBUG speeds[0:5]={[wpts[i][1] for i in range(min(5,len(wpts)))]} km/h")
    total_d=sum(s[0] for s in segs)
    _info(f"  DEBUG segs={len(segs)} total_d={total_d:.0f}m PLAYBACK={config.PLAYBACK_SPEED} dt={float(settings.fixed_delta_seconds)}")
    if total_d<0.5: _info(f"  WARN: total distance too small ({total_d:.2f}m)! Vehicle won't move.")
    # ── 同步模式 tick + 调试轨迹线 ──
    world.tick()
    prev_pt=None
    for pt,_,_ in wpts:
        if prev_pt: world.debug.draw_line(prev_pt,pt,thickness=0.05,color=carla.Color(255,200,0),life_time=300.0)
        prev_pt=pt
    bp=world.get_blueprint_library().find("vehicle.tesla.model3")
    if bp is None: bp=world.get_blueprint_library().filter("vehicle.*")[0]
    bp.set_attribute("role_name","stitch")
    p0,p1=wpts[0][0],wpts[1][0]
    yaw0=math.degrees(math.atan2(p1.y-p0.y,p1.x-p0.x))
    actor=world.try_spawn_actor(bp,carla.Transform(p0,carla.Rotation(yaw=yaw0)))
    for off in [5,10,20] if actor is None else []:
        actor=world.try_spawn_actor(bp,carla.Transform(carla.Location(p0.x,p0.y+off,p0.z),carla.Rotation(yaw=yaw0)))
    if actor is None: _info("spawn fail"); return
    actor.set_simulate_physics(False)  # 关键：禁用物理，纯瞬移驱动（匹配 engine kinematic 模式）
    _info(f"单车测试: {st['trajectory_id']} nodes={len(nodes)}")
    # 设置观众视角
    spec=world.get_spectator()
    spec.set_transform(carla.Transform(carla.Location(p0.x,p0.y,p0.z+8),carla.Rotation(pitch=-20,yaw=yaw0)))
    # 断言关键参数
    assert float(config.PLAYBACK_SPEED)>0, f"PLAYBACK_SPEED={config.PLAYBACK_SPEED} must be >0"
    assert len(segs)>0, "segs is empty - no waypoints to follow"
    assert total_d>0.5, f"total_d={total_d:.2f}m too small"
    sd=0.0; idx=0; syaw=yaw0; dt=float(settings.fixed_delta_seconds)
    try:
        while idx<len(segs):
            world.tick()
            d,spd=segs[idx]; sd+=spd*dt*float(config.PLAYBACK_SPEED)
            while idx<len(segs) and sd>d:
                sd-=d; idx+=1
                if idx<len(segs): d,spd=segs[idx]
            if idx>=len(segs): break
            a=max(0,min(1,sd/max(d,0.01)))
            p0,p1=wpts[idx][0],wpts[min(idx+1,len(wpts)-1)][0]
            ix=p0.x+(p1.x-p0.x)*a; iy=p0.y+(p1.y-p0.y)*a
            ry=math.degrees(math.atan2(p1.y-p0.y,p1.x-p0.x))
            df=(ry-syaw+180)%360-180; syaw+=df*0.3
            actor.set_transform(carla.Transform(carla.Location(ix,iy,p0.z),carla.Rotation(yaw=syaw)))
            if idx%20==0: print(f"\r{idx}/{len(segs)}",end="")
        _info("\n完成")
    finally:
        if actor.is_alive: actor.destroy()

def _run_stitch_kinematic(world, client, engine, settings) -> None:
    """拼接轨迹回放 —— 多车 PID 路点跟随自动驾驶。

    每辆车独立计算 steering（朝向下一路点）+ throttle/brake（匹配目标速度），
    逐帧 apply_control 驱动，physics=True 保证自然悬挂/车轮效果。
    """
    carla = require_carla()

    # ── 车型 → CARLA 蓝图映射 ──
    _VEHICLE_BP = {
        "car": "vehicle.tesla.model3",
        "truck": "vehicle.carlamotors.carlacola",
        "tanker": "vehicle.carlamotors.carlacola",
        "van": "vehicle.volkswagen.t2",
        "bus": "vehicle.carlamotors.carlacola",
        "pika": "vehicle.ford.mustang",
        "motorbike": "vehicle.harley-davidson.low_rider",
    }

    def _get_bp(w, vtype: str):
        bp_name = _VEHICLE_BP.get(vtype, "vehicle.tesla.model3")
        bp = w.get_blueprint_library().find(bp_name)
        if bp is None:
            bp = w.get_blueprint_library().filter("vehicle.*")[0]
        return bp

    stitch_json = os.getenv("TP_STITCH_JSON_PATH") or config.STITCH_JSON_PATH
    if not stitch_json:
        _info("错误：stitch_kinematic 模式需要设置 TP_STITCH_JSON_PATH")
        return

    # ── Phase 1: 锚点 & 坐标变换 ──
    anchor_file = os.getenv("TP_DATA_FILE_PATH") or config.DATA_FILE_PATH
    if anchor_file and os.path.exists(anchor_file):
        engine.process_data(anchor_file)
        engine.pending_tracks = []
    if not hasattr(engine, '_stitch_cos_a') or engine._stitch_cos_a is None:
        _info("错误：engine 未初始化坐标变换参数，请设置 TP_DATA_FILE_PATH")
        return
    ca = engine._stitch_cos_a; sa = engine._stitch_sin_a
    ox = engine._stitch_off_x; oy = engine._stitch_off_y
    ds = engine._data_start_point
    sc = float(config.DATA_SCALE); sy = float(config.SCALE_Y)

    # ── Phase 2: 加载 & 过滤 ──
    with open(stitch_json, "r", encoding="utf-8") as f:
        raw = json.load(f)
    all_trajs = raw.get("trajectories", raw if isinstance(raw, list) else [])
    min_q = _get_float_from_env("TP_STITCH_MIN_QUALITY", float(config.STITCH_MIN_QUALITY))
    min_c = _get_int_from_env("TP_STITCH_MIN_CAMERAS", int(config.STITCH_MIN_CAMERAS))
    max_n = _get_int_from_env("TP_TRACK_LIMIT", 0) or None

    filtered = []
    for t in all_trajs:
        if t.get("quality_score", 0) < min_q: continue
        if t.get("camera_count", 0) < min_c: continue
        filtered.append(t)
        if max_n and len(filtered) >= max_n: break

    if not filtered:
        _info("错误：无符合条件的拼接轨迹"); return
    filtered.sort(key=lambda t: t["nodes"][0]["timestamp"])
    global_min_ts = filtered[0]["nodes"][0]["timestamp"] / 1000.0
    time_win = _get_float_from_env("TP_STITCH_MAX_START_S", 600.0)
    filtered = [t for t in filtered
                if t["nodes"][0]["timestamp"] / 1000.0 - global_min_ts < time_win]
    _info(f"加载 {len(filtered)} 条轨迹 (Q>={min_q} cam>={min_c} window<={time_win}s)")

    from .lane_align import cluster_x_to_lanes, assign_lane

    clusters = cluster_x_to_lanes(filtered)
    if clusters:
        _info(f"x→车道聚类完成: {len(clusters)} 个摄像头")

    # ── Phase 3: 构建每车路点 ──
    class _VS:
        __slots__ = ('tid','wpts','actor','wpidx','stime','done','vtype','_phys_off')
        def __init__(self, tid, wpts, stime, vtype):
            self.tid = tid; self.wpts = wpts
            self.actor = None; self.wpidx = 1
            self.stime = stime; self.done = False
            self.vtype = vtype; self._phys_off = True

    def _nav_to_lane(base_wp, target_lane_id: str):
        """在 CARLA 道路网络中从 base_wp 导航到目标车道，返回对应 waypoint。"""
        try:
            cur = int(base_wp.lane_id)
            tgt = int(target_lane_id)
            wp = base_wp
            while cur < tgt:  # 向右走
                nxt = wp.get_right_lane()
                if nxt and nxt.lane_type == carla.LaneType.Driving:
                    wp = nxt; cur += 1
                else: break
            while cur > tgt:  # 向左走
                nxt = wp.get_left_lane()
                if nxt and nxt.lane_type == carla.LaneType.Driving:
                    wp = nxt; cur -= 1
                else: break
            return wp
        except Exception:
            return base_wp

    states = []
    for t in filtered:
        nodes = t["nodes"]
        stime = nodes[0]["timestamp"] / 1000.0 - global_min_ts

        wpts = []
        last_lane = "-2"
        for n in nodes:
            cam_id = n.get("camera_id", "")
            if cam_id == "INTERP":
                n_lane = last_lane
            elif clusters:
                x_val = n.get("x")
                n_lane = assign_lane(x_val, cam_id, clusters) if x_val is not None else last_lane
                last_lane = n_lane
            else:
                n_lane = None

            # 原坐标变换 → rough_loc
            y_n = n["y"]
            x_n = n.get("x") or ds[0]
            rx = (x_n - ds[0]) * sc
            ry = (y_n - ds[1]) * sc * sy
            rough_x = rx * ca - ry * sa + engine.entry_loc.x + ox
            rough_y = rx * sa + ry * ca + engine.entry_loc.y + oy
            rough_loc = carla.Location(rough_x, rough_y, engine.entry_loc.z)

            # 投影到道路——失败则跳过该节点
            try:
                base_wp = engine.map.get_waypoint(rough_loc, project_to_road=True,
                                                   lane_type=carla.LaneType.Driving)
            except Exception:
                base_wp = None
            if base_wp is None:
                continue  # 不在道路上，跳过
            # 投影距离太大 → 吸附到远处无关路点 → 丢弃
            snap_dist = rough_loc.distance(base_wp.transform.location)
            if snap_dist > 20.0:
                if len(states) < 3 and len(wpts) == 0:
                    _info(f"    node snap reject: rough=({rough_x:.0f},{rough_y:.0f}) snap_dist={snap_dist:.0f}m")
                continue

            if n_lane is not None and clusters:
                try:
                    target_wp = _nav_to_lane(base_wp, n_lane)
                    loc = target_wp.transform.location
                except Exception:
                    loc = base_wp.transform.location
            else:
                loc = base_wp.transform.location
            wpts.append((loc, n.get("speed") or 0, n["timestamp"]))

        if len(wpts) < 2:
            if len(states) < 3:
                _info(f"  丢弃 {t['trajectory_id']}: {len(nodes)}节点 → {len(wpts)}wpt (不足2个)")
            continue
        # 诊断前3条轨迹
        if len(states) < 3:
            _info(f"  {t['trajectory_id']}: {len(nodes)}节点 → {len(wpts)}wpt (过滤{len(nodes)-len(wpts)}个)")
            _info(f"    entry_loc=({engine.entry_loc.x:.0f},{engine.entry_loc.y:.0f},{engine.entry_loc.z:.1f}) map_angle={engine.map_angle:.1f}")
            _info(f"    ds=({ds[0]:.1f},{ds[1]:.1f}) ca={ca:.4f} sa={sa:.4f}")
            # 显示前3个原始节点和变换后的rough_loc
            for j in range(min(3, len(nodes))):
                nn = nodes[j]
                y_n = nn["y"]; x_n = nn.get("x") or ds[0]
                rx = (x_n - ds[0]) * sc; ry = (y_n - ds[1]) * sc * sy
                rx_loc = rx * ca - ry * sa + engine.entry_loc.x + ox
                ry_loc = rx * sa + ry * ca + engine.entry_loc.y + oy
                _info(f"    node[{j}]: raw({x_n:.0f},{y_n:.0f}) → ({rx:.2f},{ry:.2f})m → CARLA({rx_loc:.0f},{ry_loc:.0f})")
            for j in range(min(3, len(wpts))):
                loc = wpts[j][0]
                d = loc.distance(engine.entry_loc)
                _info(f"    wpt[{j}]=({loc.x:.0f},{loc.y:.0f},{loc.z:.1f}) dist_entry={d:.0f}m")
        if len(wpts) < 2: continue
        # 沿路点链推进至少 MIN_D 米后才 spawn
        MIN_D = _get_float_from_env("TP_SPAWN_MIN_ENTRY_DIST_M", 30.0)
        cumul = 0.0; si = 0
        for i in range(len(wpts) - 1):
            cumul += wpts[i][0].distance(wpts[i + 1][0])
            if cumul >= MIN_D: si = i + 1; break
        if si > 0:
            if len(states) < 3: _info(f"    跳过前{si}个路点 (累计{cumul:.0f}m)")
            wpts = wpts[si:]
        if len(wpts) < 2: continue
        states.append(_VS(t["trajectory_id"], wpts, stime, t.get("vehicle_type", "car")))

    states.sort(key=lambda s: s.stime)
    max_active = _get_int_from_env("TP_STITCH_TM_MAX_ACTIVE", int(config.STITCH_TM_MAX_ACTIVE))
    _info(f"准备回放: {len(states)} 条轨迹 max_active={max_active}")

    # ── PID 参数 ──
    KP_STEER = _get_float_from_env("TP_PID_STEER_KP", 2.0)
    KP_SPEED = _get_float_from_env("TP_PID_SPEED_KP", 0.05)
    KP_BRAKE = _get_float_from_env("TP_PID_SPEED_BRAKE_KP", 0.1)
    WP_THRESHOLD = _get_float_from_env("TP_PID_WAYPOINT_THRESHOLD_M", 4.0)

    # ── Phase 4: 主循环 ──
    spawn_q = list(states)
    active = []
    sim_time = 0.0
    dt = float(settings.fixed_delta_seconds)
    ps = float(config.PLAYBACK_SPEED)
    tick_idx = 0
    spawned = 0; spawn_fail = 0

    _try_set_spectator_view(world, engine)

    try:
        while spawn_q or active:
            world.tick()
            sim_time += dt * ps
            tick_idx += 1

            # ── spawn ──
            while spawn_q and spawn_q[0].stime <= sim_time and len(active) < max_active:
                vs = spawn_q.pop(0)
                p0, p1 = vs.wpts[0][0], vs.wpts[1][0]
                yaw0 = math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))
                bp = _get_bp(world, vs.vtype)
                bp.set_attribute("role_name", "stitch")
                actor = world.try_spawn_actor(bp, carla.Transform(p0, carla.Rotation(yaw=yaw0)))
                for off in [5, 10, 20] if actor is None else []:
                    actor = world.try_spawn_actor(bp,
                        carla.Transform(carla.Location(p0.x, p0.y + off, p0.z), carla.Rotation(yaw=yaw0)))
                if actor is None:
                    spawn_fail += 1; vs.done = True; continue
                # 先禁用物理，精确放置到 spawn 位置，避免首帧下落
                actor.set_simulate_physics(False)
                actor.set_transform(carla.Transform(carla.Location(p0.x, p0.y, p0.z + 0.5), carla.Rotation(yaw=yaw0)))
                vs.actor = actor
                active.append(vs); spawned += 1

            # ── 推进所有活跃车辆 ──
            for vs in list(active):
                if vs.done: continue
                # 首帧：启用物理（spawn 时先禁用防掉落）
                if vs._phys_off:
                    vs.actor.set_simulate_physics(True)
                    vs._phys_off = False
                t = vs.actor.get_transform()
                vel = vs.actor.get_velocity()
                speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2)

                # 前进到下一个路点（已到则跳过）
                while vs.wpidx < len(vs.wpts) - 1:
                    if t.location.distance(vs.wpts[vs.wpidx][0]) < WP_THRESHOLD:
                        vs.wpidx += 1
                    else:
                        break

                # 到最后路点 → 完成
                if vs.wpidx >= len(vs.wpts) - 1:
                    if t.location.distance(vs.wpts[-1][0]) < WP_THRESHOLD:
                        vs.done = True
                        if vs.actor.is_alive: vs.actor.destroy()
                        continue

                target = vs.wpts[vs.wpidx][0]
                target_spd = max(5.0, vs.wpts[vs.wpidx][1]) / 3.6  # km/h → m/s

                # ── Steering: 朝向目标路点 ──
                dx = target.x - t.location.x
                dy = target.y - t.location.y
                target_angle = math.atan2(dy, dx)
                vehicle_angle = math.radians(t.rotation.yaw)
                angle_diff = (target_angle - vehicle_angle + math.pi) % (2 * math.pi) - math.pi
                steer = max(-1.0, min(1.0, KP_STEER * angle_diff))

                # ── Speed: 匹配目标速度 ──
                spd_err = target_spd - speed_ms
                if spd_err > 0:
                    throttle = min(1.0, KP_SPEED * spd_err)
                    brake = 0.0
                else:
                    throttle = 0.0
                    brake = min(1.0, KP_BRAKE * abs(spd_err))

                vs.actor.apply_control(
                    carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

            active = [v for v in active if not v.done]

            if tick_idx % max(1, int(config.PRINT_EVERY_N_TICKS)) == 0:
                print(f"\rTime: {sim_time:.1f}s | Active: {len(active)} | "
                      f"Spawned: {spawned} | Fail: {spawn_fail} | Queue: {len(spawn_q)}   ", end="")

        _info(f"\n回放完成: spawned={spawned} fail={spawn_fail}")
    finally:
        for vs in active:
            if vs.actor and vs.actor.is_alive:
                vs.actor.destroy()


def _run_stitch_autopilot(world, client, engine, settings) -> None:
    """TM 自动驾驶模式：拼接轨迹 → TM 自动驾驶回放。"""
    from .stitch_adapter import StitchAdapter
    from .stitch_autopilot import StitchAutopilot

    stitch_json = os.getenv("TP_STITCH_JSON_PATH") or config.STITCH_JSON_PATH
    if not stitch_json:
        _info("错误：stitch_autopilot 模式需要设置 TP_STITCH_JSON_PATH")
        return

    # Phase 1: 运行 process_data 获取坐标变换参数
    anchor_file = config.DATA_FILE_PATH
    if not anchor_file or not os.path.exists(anchor_file):
        _info(f"警告：锚点文件不存在 {anchor_file}，使用默认变换")
    else:
        engine.process_data(anchor_file)
        engine.pending_tracks = []

    # Phase 2: 加载拼接轨迹
    adapter = StitchAdapter(engine)
    stitch_tracks = adapter.load_and_convert(
        stitch_json,
        min_quality=_get_float_from_env("TP_STITCH_MIN_QUALITY", float(config.STITCH_MIN_QUALITY)),
        min_cameras=_get_int_from_env("TP_STITCH_MIN_CAMERAS", int(config.STITCH_MIN_CAMERAS)),
        max_tracks=_get_int_from_env("TP_TRACK_LIMIT", 0) or None,
    )
    if not stitch_tracks:
        _info("错误：未加载到有效的拼接轨迹")
        return

    # 过滤：只保留前 N 秒内开始的轨迹（避免 11 天跨度导致车辆永不生成）
    max_start_s = _get_float_from_env("TP_STITCH_MAX_START_S", 600.0)
    stitch_tracks = sorted(stitch_tracks, key=lambda t: t.start_time)
    earliest = stitch_tracks[0].start_time if stitch_tracks else 0.0
    stitch_tracks = [t for t in stitch_tracks if t.start_time - earliest < max_start_s]
    _info(f"Time-filtered to {len(stitch_tracks)} tracks (first {max_start_s}s window)")

    # Phase 3: 创建 autopilot 编排器
    autopilot = StitchAutopilot(world, client, engine)
    autopilot.load(stitch_tracks)

    _try_set_spectator_view(world, engine)
    _info(f"Stitch-Autopilot: {len(stitch_tracks)} trajectories | "
          f"TM active={autopilot._spawned_total} max={config.STITCH_TM_MAX_ACTIVE}")

    # Phase 4: 主循环
    fixed_dt = float(settings.fixed_delta_seconds)
    tick_idx = 0

    while True:
        world.tick()
        active_cnt = autopilot.tick(fixed_dt)
        tick_idx += 1
        s = autopilot.stats  # 确保 s 在每次迭代都定义（避免条件块内赋值导致未定义）

        if tick_idx % max(1, int(config.PRINT_EVERY_N_TICKS)) == 0:
            print(
                f"\rTime: {s['current_time']:.1f}s | Active: {s['active']} "
                f"| Spawned: {s['spawned']} | Finished: {s['finished']} "
                f"| Pending: {s['pending']} | Fail: {s['failures']}   ",
                end="",
            )

        if tick_idx % max(1, int(config.STATUS_LOG_EVERY_N_TICKS)) == 0:
            logger.info(
                "Status: time=%.1fs active=%d pending=%d spawned=%d finished=%d failures=%d",
                s["current_time"], s["active"], s["pending"],
                s["spawned"], s["finished"], s["failures"],
            )

        if s["pending"] == 0 and s["active"] == 0:
            if config.MAX_TRAJ_TIME_SECONDS and s["current_time"] < float(config.MAX_TRAJ_TIME_SECONDS):
                continue
            _info("\nStitch-Autopilot playback finished.")
            s_final = autopilot.stats
            logger.info(
                "Final stats: spawned=%d finished=%d failures=%d",
                s_final["spawned"], s_final["finished"], s_final["failures"],
            )
            break

        if config.MAX_TRAJ_TIME_SECONDS and s["current_time"] >= float(config.MAX_TRAJ_TIME_SECONDS):
            _info(f"\nStop: reached TP_MAX_TRAJ_TIME={float(config.MAX_TRAJ_TIME_SECONDS):.2f}s")
            break

    autopilot.cleanup()


def _apply_env_overrides_to_config() -> None:
    """Mutate tp_replay.config module-level defaults using env vars.

    This mirrors the legacy behavior in replay_main.main() where globals were
    overridden in-place.
    """

    # Track limit (allow -1 to mean None)
    track_limit = _get_int_from_env(
        "TP_TRACK_LIMIT",
        config.TRACK_LIMIT if config.TRACK_LIMIT is not None else -1,
    )
    if track_limit == -1:
        config.TRACK_LIMIT = None
    else:
        config.TRACK_LIMIT = track_limit

    config.PRINT_EVERY_N_TICKS = _get_int_from_env(
        "TP_PRINT_EVERY_N_TICKS",
        int(config.PRINT_EVERY_N_TICKS),
    )
    config.STATUS_LOG_EVERY_N_TICKS = _get_int_from_env(
        "TP_STATUS_LOG_EVERY_N_TICKS",
        int(config.STATUS_LOG_EVERY_N_TICKS),
    )
    config.MAX_TRAJ_TIME_SECONDS = _get_float_from_env(
        "TP_MAX_TRAJ_TIME",
        float(config.MAX_TRAJ_TIME_SECONDS),
    )

    config.FINISH_BEHAVIOR = _get_str_from_env("TP_FINISH_BEHAVIOR", config.FINISH_BEHAVIOR)
    config.FINISH_TELEPORT_Z = _get_float_from_env(
        "TP_FINISH_TELEPORT_Z",
        float(config.FINISH_TELEPORT_Z),
    )

    config.SPAWN_RETRY_DELAY_SECONDS = _get_float_from_env(
        "TP_SPAWN_RETRY_DELAY_SECONDS",
        float(config.SPAWN_RETRY_DELAY_SECONDS),
    )
    config.SPAWN_MAX_ATTEMPTS = _get_int_from_env(
        "TP_SPAWN_MAX_ATTEMPTS",
        int(config.SPAWN_MAX_ATTEMPTS),
    )
    config.SPAWN_OVERLAP_BLOCK_DIST_M = _get_float_from_env(
        "TP_SPAWN_OVERLAP_BLOCK_DIST_M",
        float(config.SPAWN_OVERLAP_BLOCK_DIST_M),
    )
    config.SPAWN_CANDIDATE_OFFSETS_M = _get_float_list_from_env(
        "TP_SPAWN_CANDIDATE_OFFSETS_M",
        list(config.SPAWN_CANDIDATE_OFFSETS_M),
    )
    config.SPAWN_TRY_ADJACENT_LANES = _get_bool_from_env(
        "TP_SPAWN_TRY_ADJACENT_LANES",
        bool(config.SPAWN_TRY_ADJACENT_LANES),
    )
    config.SPAWN_RETRY_MIN_REMAINING_SECONDS = _get_float_from_env(
        "TP_SPAWN_RETRY_MIN_REMAINING_SECONDS",
        float(config.SPAWN_RETRY_MIN_REMAINING_SECONDS),
    )
    config.SPAWN_RETRY_MIN_DELAY_SECONDS = _get_float_from_env(
        "TP_SPAWN_RETRY_MIN_DELAY_SECONDS",
        float(config.SPAWN_RETRY_MIN_DELAY_SECONDS),
    )
    config.SPAWN_Z_OFFSET_M = _get_float_from_env(
        "TP_SPAWN_Z_OFFSET_M",
        float(config.SPAWN_Z_OFFSET_M),
    )
    config.SPAWN_MAX_CANDIDATES = _get_int_from_env(
        "TP_SPAWN_MAX_CANDIDATES",
        int(config.SPAWN_MAX_CANDIDATES),
    )

    # MAX_ACTIVE_VEHICLES: empty -> keep default; <=0 -> None
    max_active_raw = os.getenv("TP_MAX_ACTIVE_VEHICLES")
    if max_active_raw is not None and str(max_active_raw).strip() != "":
        v = _get_int_from_env("TP_MAX_ACTIVE_VEHICLES", 0)
        config.MAX_ACTIVE_VEHICLES = None if v <= 0 else v

    config.SPAWN_DEFER_SECONDS = _get_float_from_env(
        "TP_SPAWN_DEFER_SECONDS",
        float(config.SPAWN_DEFER_SECONDS),
    )

    config.PLAYBACK_SPEED = _get_float_from_env(
        "TP_PLAYBACK_SPEED",
        float(config.PLAYBACK_SPEED),
    )
    config.ENABLE_PHYSICS = _get_bool_from_env(
        "TP_ENABLE_PHYSICS",
        bool(config.ENABLE_PHYSICS),
    )

    config.DATA_SPEED_IN_KMH = _get_bool_from_env(
        "TP_DATA_SPEED_IN_KMH",
        bool(config.DATA_SPEED_IN_KMH),
    )
    config.SPEED_FACTOR = _get_speed_factor_from_env(float(config.SPEED_FACTOR))

    config.REPLAY_CONTROL_MODE = _get_str_from_env(
        "TP_REPLAY_CONTROL_MODE",
        config.REPLAY_CONTROL_MODE,
    ).strip().lower()
    if config.REPLAY_CONTROL_MODE == "kinematic":
        config.ENABLE_PHYSICS = False

    config.PHYSICS_SYNC_DIST = _get_float_from_env(
        "TP_PHYSICS_SYNC_DIST",
        float(config.PHYSICS_SYNC_DIST),
    )
    config.LOOKAHEAD_TIME = _get_float_from_env(
        "TP_LOOKAHEAD_TIME",
        float(config.LOOKAHEAD_TIME),
    )
    config.LOOKAHEAD_SPEED_GAIN = _get_float_from_env(
        "TP_LOOKAHEAD_SPEED_GAIN",
        float(config.LOOKAHEAD_SPEED_GAIN),
    )
    config.CATCH_UP_GAIN = _get_float_from_env(
        "TP_CATCH_UP_GAIN",
        float(config.CATCH_UP_GAIN),
    )
    config.SNAP_THRESHOLD = _get_float_from_env(
        "TP_SNAP_THRESHOLD",
        float(config.SNAP_THRESHOLD),
    )
    config.SNAP_STRICT_DIST = _get_float_from_env(
        "TP_SNAP_STRICT_DIST",
        float(config.SNAP_STRICT_DIST),
    )
    config.LANE_CHANGE_PENALTY_M = _get_float_from_env(
        "TP_LANE_CHANGE_PENALTY_M",
        float(config.LANE_CHANGE_PENALTY_M),
    )
    config.LANE_SWITCH_MIN_IMPROVEMENT_M = _get_float_from_env(
        "TP_LANE_SWITCH_MIN_IMPROVEMENT_M",
        float(config.LANE_SWITCH_MIN_IMPROVEMENT_M),
    )
    config.ENFORCE_SAME_ROAD_ID = _get_bool_from_env(
        "TP_ENFORCE_SAME_ROAD_ID",
        bool(config.ENFORCE_SAME_ROAD_ID),
    )
    config.ROTATION_BIAS_DEG = _get_float_from_env(
        "TP_ROTATION_BIAS_DEG",
        float(config.ROTATION_BIAS_DEG),
    )
    config.DATA_ANGLE_BASELINE_M = _get_float_from_env(
        "TP_DATA_ANGLE_BASELINE_M",
        float(config.DATA_ANGLE_BASELINE_M),
    )

    config.XODR_PATH = os.getenv("TP_XODR_PATH") or config.XODR_PATH
    config.DATA_FILE_PATH = os.getenv("TP_DATA_FILE_PATH") or config.DATA_FILE_PATH


def _try_set_spectator_view(world, engine) -> None:
    """Apply spectator view from a JSON file if present; fallback to anchor view."""

    carla = require_carla()

    try:
        spectator = world.get_spectator()
    except Exception as e:
        logger.debug("get_spectator failed: %s", e)
        return

    cam_path = _get_str_from_env("TP_SPECTATOR_VIEW_JSON", "")
    repo_root = Path(__file__).resolve().parents[1]
    default_cam = str(repo_root / "camera_view.json")
    candidates = [p for p in [cam_path, default_cam] if p]

    last_error: Optional[Exception] = None
    for p in candidates:
        try:
            if not os.path.exists(p):
                raise FileNotFoundError(p)
            with open(p, "r", encoding="utf-8") as f:
                cam = json.load(f)
            cam_loc = carla.Location(
                cam["location"]["x"],
                cam["location"]["y"],
                cam["location"]["z"],
            )
            cam_rot = carla.Rotation(
                pitch=cam["rotation"]["pitch"],
                yaw=cam["rotation"]["yaw"],
                roll=cam["rotation"]["roll"],
            )
            spectator.set_transform(carla.Transform(cam_loc, cam_rot))
            logger.info("Applied spectator view from %s", p)
            return
        except (OSError, json.JSONDecodeError, KeyError) as e:
            last_error = e

    if last_error is not None:
        logger.info("Use fallback spectator view: %s", last_error)

    try:
        fwd = engine.entry_fwd
        vf = -1.0 if bool(config.REVERSE_DIRECTION) else 1.0
        cam_loc = carla.Location(
            engine.entry_loc.x - fwd.x * 10 * vf,
            engine.entry_loc.y - fwd.y * 10 * vf,
            engine.entry_loc.z + 8,
        )
        yaw = math.degrees(math.atan2(fwd.y * vf, fwd.x * vf))
        spectator.set_transform(
            carla.Transform(cam_loc, carla.Rotation(pitch=-10, yaw=yaw))
        )
    except Exception as e:
        logger.debug("fallback spectator set_transform failed: %s", e)


def _maybe_apply_weather_override(world) -> None:
    """Optionally override weather using env vars.

    Preserves legacy semantics:
    - By default does nothing.
    - If TP_WEATHER_PRESET is set, apply the named carla.WeatherParameters preset.
    - Else if TP_FORCE_WEATHER=1, apply partial TP_WEATHER_* overrides.
    """

    carla = require_carla()

    force_weather = _get_bool_from_env("TP_FORCE_WEATHER", False)
    weather_preset = _get_str_from_env("TP_WEATHER_PRESET", "")
    if not (force_weather or weather_preset):
        return

    try:
        if weather_preset:
            preset_name = weather_preset.strip()
            preset = getattr(carla.WeatherParameters, preset_name, None)
            if preset is None:
                logger.warning(
                    "Unknown TP_WEATHER_PRESET=%r; expected a carla.WeatherParameters preset name (e.g. ClearNoon)",
                    preset_name,
                )
            else:
                world.set_weather(preset)
                logger.info("Applied TP_WEATHER_PRESET=%s", preset_name)
            return

        if force_weather:
            w = world.get_weather()
            cloud = os.getenv("TP_WEATHER_CLOUDINESS")
            if cloud not in (None, ""):
                w.cloudiness = float(cloud)
            rain = os.getenv("TP_WEATHER_PRECIPITATION")
            if rain not in (None, ""):
                w.precipitation = float(rain)
            fog = os.getenv("TP_WEATHER_FOG_DENSITY")
            if fog not in (None, ""):
                w.fog_density = float(fog)
            sun_alt = os.getenv("TP_WEATHER_SUN_ALTITUDE_ANGLE")
            if sun_alt not in (None, ""):
                w.sun_altitude_angle = float(sun_alt)
            sun_az = os.getenv("TP_WEATHER_SUN_AZIMUTH_ANGLE")
            if sun_az not in (None, ""):
                w.sun_azimuth_angle = float(sun_az)
            world.set_weather(w)
            logger.info("Applied TP_FORCE_WEATHER=1 (partial TP_WEATHER_* overrides)")
    except Exception as e:
        logger.warning("Failed to apply weather override: %s", e)


def main() -> None:
    """Run CARLA replay."""

    # Windows 控制台经常是 GBK：尽量让输出用 UTF-8，减少乱码
    engine = None

    try:
        if isinstance(sys.stdout, io.TextIOWrapper):
            sys.stdout.reconfigure(encoding="utf-8")
        if isinstance(sys.stderr, io.TextIOWrapper):
            sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

    log_path = _ensure_logging_to_file()
    if log_path:
        print(f"Logging to: {log_path}")

    _apply_env_overrides_to_config()

    logger.info(
        "Run config: XODR_PATH=%s DATA_FILE_PATH=%s TRACK_LIMIT=%s TP_SELECT_MODE=%s TP_MAX_TRAJ_TIME=%s TP_REBASE_TIME=%s "
        "PLAYBACK_SPEED=%s TP_REPLAY_CONTROL_MODE=%s ENABLE_PHYSICS=%s TP_DATA_SPEED_IN_KMH=%s TP_DATA_SPEED_FACTOR=%s PHYSICS_SYNC_DIST=%s SNAP_THRESHOLD=%s "
        "TP_SPAWN_MAX_ATTEMPTS=%s TP_SPAWN_RETRY_DELAY_SECONDS=%s TP_SPAWN_OVERLAP_BLOCK_DIST_M=%s "
        "TP_SPAWN_CANDIDATE_OFFSETS_M=%s TP_SPAWN_TRY_ADJACENT_LANES=%s TP_SPAWN_MAX_CANDIDATES=%s "
        "TP_SPAWN_RETRY_MIN_REMAINING_SECONDS=%s TP_SPAWN_RETRY_MIN_DELAY_SECONDS=%s TP_SPAWN_Z_OFFSET_M=%s "
        "TP_FINISH_BEHAVIOR=%s TP_FINISH_TELEPORT_Z=%s TP_MAX_ACTIVE_VEHICLES=%s TP_SPAWN_DEFER_SECONDS=%s",
        config.XODR_PATH,
        config.DATA_FILE_PATH,
        config.TRACK_LIMIT,
        (os.getenv("TP_SELECT_MODE") or "none"),
        config.MAX_TRAJ_TIME_SECONDS,
        (os.getenv("TP_REBASE_TIME") or "0"),
        config.PLAYBACK_SPEED,
        (os.getenv("TP_REPLAY_CONTROL_MODE") or config.REPLAY_CONTROL_MODE),
        config.ENABLE_PHYSICS,
        config.DATA_SPEED_IN_KMH,
        config.SPEED_FACTOR,
        config.PHYSICS_SYNC_DIST,
        config.SNAP_THRESHOLD,
        config.SPAWN_MAX_ATTEMPTS,
        config.SPAWN_RETRY_DELAY_SECONDS,
        config.SPAWN_OVERLAP_BLOCK_DIST_M,
        ",".join(str(x) for x in config.SPAWN_CANDIDATE_OFFSETS_M),
        config.SPAWN_TRY_ADJACENT_LANES,
        config.SPAWN_MAX_CANDIDATES,
        config.SPAWN_RETRY_MIN_REMAINING_SECONDS,
        config.SPAWN_RETRY_MIN_DELAY_SECONDS,
        config.SPAWN_Z_OFFSET_M,
        config.FINISH_BEHAVIOR,
        config.FINISH_TELEPORT_Z,
        config.MAX_ACTIVE_VEHICLES,
        config.SPAWN_DEFER_SECONDS,
    )

    if _get_bool_from_env("TP_SPAWN_SCHEDULE_COMPRESS", False):
        logger.info(
            "Spawn schedule config: enabled=1 mode=%s base=%.3f spread=%.3f jitter=%.3f seed=%r preview_n=%d",
            _get_str_from_env("TP_SPAWN_SCHEDULE_MODE", "uniform"),
            float(_get_float_from_env("TP_SPAWN_SCHEDULE_BASE_SECONDS", 0.0)),
            float(_get_float_from_env("TP_SPAWN_SCHEDULE_SPREAD_SECONDS", 10.0)),
            float(_get_float_from_env("TP_SPAWN_SCHEDULE_JITTER_SECONDS", 0.0)),
            os.getenv("TP_SPAWN_SCHEDULE_SEED"),
            int(_get_int_from_env("TP_SPAWN_SCHEDULE_PREVIEW_N", 10)),
        )

    carla = require_carla()

    try:
        host = _get_str_from_env("TP_CARLA_HOST", "localhost")
        port = _get_int_from_env("TP_CARLA_PORT", 2000)
        timeout = _get_float_from_env("TP_CARLA_TIMEOUT_SECONDS", 10.0)
        client = carla.Client(host, int(port))
        client.set_timeout(float(timeout))
        world = _maybe_generate_opendrive_world(client, config.XODR_PATH)
    except Exception as e:
        logger.error(
            "Failed to connect to CARLA server at %s:%s: %s. Is CARLA Server running?",
            os.getenv("TP_CARLA_HOST") or "localhost",
            os.getenv("TP_CARLA_PORT") or "2000",
            e,
        )
        raise

    # Keep a baseline for restoration.
    original_settings = None
    try:
        original_settings = world.get_settings()
    except Exception as e:
        logger.warning("Failed to get world settings: %s", e)

    try:
        # Weather override is optional and should apply to the final world.
        _maybe_apply_weather_override(world)

        # Apply deterministic synchronous settings (legacy behavior)
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = _get_float_from_env(
            "TP_FIXED_DELTA_SECONDS",
            0.05,
        )

        if _get_bool_from_env("TP_ENABLE_SUBSTEPPING", False):
            if hasattr(settings, "substepping"):
                settings.substepping = True
            if hasattr(settings, "max_substep_delta_time"):
                settings.max_substep_delta_time = _get_float_from_env(
                    "TP_MAX_SUBSTEP_DELTA_TIME",
                    0.01,
                )
            if hasattr(settings, "max_substeps"):
                settings.max_substeps = _get_int_from_env("TP_MAX_SUBSTEPS", 2)

        world.apply_settings(settings)

        engine = ReplayEngine(client, config.XODR_PATH)

        # ── 清理所有残留车辆 ──
        for actor in list(world.get_actors().filter("vehicle.*")):
            try: actor.destroy()
            except: pass

        # —— 拼接轨迹回放模式 ——
        replay_mode = _get_str_from_env("TP_REPLAY_MODE", "kinematic").strip().lower()
        if replay_mode == "stitch_simple":
            _run_stitch_simple(world, client, engine, settings)
            return
        if replay_mode == "stitch_kinematic":
            _run_stitch_kinematic(world, client, engine, settings)
            return
        if replay_mode == "stitch_autopilot":
            _run_stitch_autopilot(world, client, engine, settings)
            return

        engine.process_data(config.DATA_FILE_PATH)
        if not engine.pending_tracks:
            logger.warning("No tracks loaded; exiting")
            return

        _try_set_spectator_view(world, engine)

        mode_label = os.getenv("TP_REPLAY_CONTROL_MODE") or config.REPLAY_CONTROL_MODE
        _info(f"开始回放 (mode={mode_label})...")

        tick_idx = 0
        fixed_dt = float(settings.fixed_delta_seconds)
        while True:
            world.tick()
            active_cnt = engine.tick(fixed_dt)
            tick_idx += 1

            if tick_idx % max(1, int(config.PRINT_EVERY_N_TICKS)) == 0:
                print(
                    f"\rTrajTime: {engine.current_traj_time:.2f}s | Active: {active_cnt} "
                    f"| Teleports: {engine.stats['teleports']} "
                    f"| SpawnOK: {engine.stats.get('spawn_success', 0)} "
                    f"| SpawnFail: {engine.stats['spawn_failures']} "
                    f"(None: {engine.stats.get('spawn_fail_none', 0)} Exc: {engine.stats.get('spawn_fail_exception', 0)}) "
                    f"| SkippedWP: {engine.stats['skipped_no_waypoint']} "
                    f"| ParsedLaneChg: {engine.stats.get('parsed_lane_changes', 0)} "
                    f"| Finished: {engine.stats.get('finished_tracks', 0)} Destroyed: {engine.stats.get('destroyed_tracks', 0)} "
                    f"| Pending: {len(engine.pending_tracks)}   ",
                    end="",
                )

            if tick_idx % max(1, int(config.STATUS_LOG_EVERY_N_TICKS)) == 0:
                logger.info(
                    "Status: traj_time=%.2fs active=%d pending=%d teleports=%d spawn_ok=%d spawn_fail=%d finished=%d destroyed=%d extended=%d parsed_lane_changes=%d snap_fallback_wp=%d skipped_wp=%d",
                    engine.current_traj_time,
                    active_cnt,
                    len(engine.pending_tracks),
                    engine.stats["teleports"],
                    engine.stats.get("spawn_success", 0),
                    engine.stats["spawn_failures"],
                    engine.stats.get("finished_tracks", 0),
                    engine.stats.get("destroyed_tracks", 0),
                    engine.stats.get("replays_extended", 0),
                    engine.stats.get("parsed_lane_changes", 0),
                    engine.stats.get("snap_fallback_waypoint", 0),
                    engine.stats["skipped_no_waypoint"],
                )

            if not engine.pending_tracks and active_cnt == 0:
                if config.MAX_TRAJ_TIME_SECONDS and engine.current_traj_time < float(
                    config.MAX_TRAJ_TIME_SECONDS
                ):
                    continue

                _info("\nPlayback finished.")
                logger.info(
                    "Final stats: teleports=%d spawn_ok=%d spawn_fail=%d (none=%d exc=%d) blocked_overlap=%d deferred_no_state=%d "
                    "skipped_past_end=%d abandoned_max_attempts=%d abandoned_near_end=%d candidates_tried=%d finished=%d destroyed=%d "
                    "extended=%d parsed_lane_changes=%d snap_fallback_wp=%d skipped_wp=%d parse_errors=%d",
                    engine.stats["teleports"],
                    engine.stats.get("spawn_success", 0),
                    engine.stats["spawn_failures"],
                    engine.stats.get("spawn_fail_none", 0),
                    engine.stats.get("spawn_fail_exception", 0),
                    engine.stats.get("spawn_blocked_overlap", 0),
                    engine.stats.get("spawn_deferred_no_state", 0),
                    engine.stats.get("spawn_skipped_past_end", 0),
                    engine.stats.get("spawn_abandoned_max_attempts", 0),
                    engine.stats.get("spawn_abandoned_near_end", 0),
                    engine.stats.get("spawn_candidates_tried", 0),
                    engine.stats.get("finished_tracks", 0),
                    engine.stats.get("destroyed_tracks", 0),
                    engine.stats.get("replays_extended", 0),
                    engine.stats.get("parsed_lane_changes", 0),
                    engine.stats.get("snap_fallback_waypoint", 0),
                    engine.stats["skipped_no_waypoint"],
                    engine.stats["parse_errors"],
                )
                if float(config.POST_PLAYBACK_HOLD_SECONDS) > 0.0:
                    hold_ticks = int(float(config.POST_PLAYBACK_HOLD_SECONDS) / fixed_dt)
                    for _ in range(max(0, hold_ticks)):
                        world.tick()
                break

            if config.MAX_TRAJ_TIME_SECONDS and engine.current_traj_time >= float(
                config.MAX_TRAJ_TIME_SECONDS
            ):
                print(
                    f"\nStop: reached TP_MAX_TRAJ_TIME={float(config.MAX_TRAJ_TIME_SECONDS):.2f}s"
                )
                logger.info("Stop due to max traj time: %.3fs", engine.current_traj_time)
                break

    except KeyboardInterrupt:
        _info("\nStop")
        if engine is not None:
            logger.info(
                "Interrupted: traj_time=%.3fs active=%d pending=%d teleports=%d spawn_ok=%d spawn_fail=%d (none=%d exc=%d) "
                "blocked_overlap=%d deferred_no_state=%d skipped_past_end=%d abandoned_max_attempts=%d abandoned_near_end=%d "
                "candidates_tried=%d finished=%d destroyed=%d parsed_lane_changes=%d snap_fallback_wp=%d skipped_wp=%d parse_errors=%d",
                engine.current_traj_time,
                len(engine.active_tracks),
                len(engine.pending_tracks),
                engine.stats.get("teleports", 0),
                engine.stats.get("spawn_success", 0),
                engine.stats.get("spawn_failures", 0),
                engine.stats.get("spawn_fail_none", 0),
                engine.stats.get("spawn_fail_exception", 0),
                engine.stats.get("spawn_blocked_overlap", 0),
                engine.stats.get("spawn_deferred_no_state", 0),
                engine.stats.get("spawn_skipped_past_end", 0),
                engine.stats.get("spawn_abandoned_max_attempts", 0),
                engine.stats.get("spawn_abandoned_near_end", 0),
                engine.stats.get("spawn_candidates_tried", 0),
                engine.stats.get("finished_tracks", 0),
                engine.stats.get("destroyed_tracks", 0),
                engine.stats.get("parsed_lane_changes", 0),
                engine.stats.get("snap_fallback_waypoint", 0),
                engine.stats.get("skipped_no_waypoint", 0),
                engine.stats.get("parse_errors", 0),
            )
    except Exception:
        logger.exception("Fatal error")
        raise
    finally:
        _info("Cleaning up...")

        # Best-effort destroy
        if engine is not None:
            try:
                for t in list(engine.active_tracks):
                    if t.actor:
                        t.actor.destroy()
            except Exception as e:
                logger.debug("destroy active actors failed: %s", e)
            try:
                for t in list(engine.pending_tracks):
                    if t.actor:
                        t.actor.destroy()
            except Exception as e:
                logger.debug("destroy pending actors failed: %s", e)

        if original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception as e:
                logger.warning("Failed to restore world settings: %s", e)


if __name__ == "__main__":
    main()
