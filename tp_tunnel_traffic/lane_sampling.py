from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple


@dataclass(frozen=True)
class PathPoint:
    s: float
    # carla.Transform（运行时由 CARLA 提供，这里用 Any 避免静态分析误报）
    transform: Any


def _line_xy_hdg(x: float, y: float, hdg: float, ds: float) -> Tuple[float, float, float]:
    return x + ds * math.cos(hdg), y + ds * math.sin(hdg), hdg


def _arc_xy_hdg(x: float, y: float, hdg: float, curvature: float, ds: float) -> Tuple[float, float, float]:
    k = curvature
    if abs(k) < 1e-9:
        return _line_xy_hdg(x, y, hdg, ds)
    dh = k * ds
    nx = x + (math.sin(hdg + dh) - math.sin(hdg)) / k
    ny = y - (math.cos(hdg + dh) - math.cos(hdg)) / k
    return nx, ny, hdg + dh


def _lane_offset_at(road_elem, s: float) -> float:
    lane_offsets = list(road_elem.findall("lanes/laneOffset"))
    if not lane_offsets:
        return 0.0
    lane_offset = lane_offsets[0]
    for candidate in lane_offsets:
        if float(candidate.attrib.get("s", "0")) <= s:
            lane_offset = candidate
        else:
            break
    s0 = float(lane_offset.attrib.get("s", "0"))
    ds = max(0.0, s - s0)
    a = float(lane_offset.attrib.get("a", "0"))
    b = float(lane_offset.attrib.get("b", "0"))
    c = float(lane_offset.attrib.get("c", "0"))
    d = float(lane_offset.attrib.get("d", "0"))
    return a + b * ds + c * ds * ds + d * ds * ds * ds


def _lane_width_at(lane_elem, s: float) -> float:
    if lane_elem is None:
        return 0.0
    widths = list(lane_elem.findall("width"))
    if not widths:
        return 0.0
    width = widths[0]
    for candidate in widths:
        if float(candidate.attrib.get("sOffset", "0")) <= s:
            width = candidate
        else:
            break
    s0 = float(width.attrib.get("sOffset", "0"))
    ds = max(0.0, s - s0)
    a = float(width.attrib.get("a", "0"))
    b = float(width.attrib.get("b", "0"))
    c = float(width.attrib.get("c", "0"))
    d = float(width.attrib.get("d", "0"))
    return a + b * ds + c * ds * ds + d * ds * ds * ds


def _road_has_three_driving_lanes(road) -> bool:
    right_sections = road.findall("lanes/laneSection/right")
    if not right_sections:
        return False
    for right in right_sections:
        lane_ids = {lane.attrib.get("id") for lane in right.findall("lane")}
        if {"-1", "-2", "-3"}.issubset(lane_ids):
            return True
    return False


def _road_length_m(road) -> float:
    try:
        return float(road.attrib.get("length", "0"))
    except ValueError:
        return 0.0


def _sample_road_reference_points(road_elem, step_m: float) -> List[Tuple[float, float, float, float]]:
    plan = road_elem.find("planView")
    if plan is None:
        return []

    geometries = []
    for geom in plan.findall("geometry"):
        s0 = float(geom.attrib["s"])
        x0 = float(geom.attrib["x"])
        y0 = float(geom.attrib["y"])
        hdg0 = float(geom.attrib["hdg"])
        length = float(geom.attrib["length"])
        kind = "line"
        curvature = 0.0
        if geom.find("arc") is not None:
            kind = "arc"
            curvature = float(geom.find("arc").attrib["curvature"])
        geometries.append((s0, x0, y0, hdg0, length, kind, curvature))

    pts = []
    for idx, (s0, x0, y0, hdg0, length, kind, curvature) in enumerate(geometries):
        steps = max(1, int(math.ceil(length / step_m)))
        for i in range(steps + 1):
            ds = min(i * step_m, length)
            if idx > 0 and i == 0:
                continue
            if kind == "line":
                x, y, hdg = _line_xy_hdg(x0, y0, hdg0, ds)
            else:
                x, y, hdg = _arc_xy_hdg(x0, y0, hdg0, curvature, ds)
            pts.append((s0 + ds, x, y, hdg))
    return pts


def _road_ids_in_order(root) -> List[str]:
    return [r.attrib["id"] for r in root.findall("road")]


def _choose_road_sequence(root) -> List[str]:
    roads = []
    for road in root.findall("road"):
        if _road_has_three_driving_lanes(road):
            roads.append((
                _road_length_m(road),
                road.attrib["id"].strip(),
            ))
    if roads:
        roads.sort(reverse=True)
        return [roads[0][1]]

    road_ids = _road_ids_in_order(root)
    preferred = ["5", "2", "9", "10"]
    seq = [r for r in preferred if r in road_ids]
    if not seq:
        seq = road_ids[:1]
    return seq


def _lane_section_at(road, s: float):
    lane_sections = list(road.findall("lanes/laneSection"))
    if not lane_sections:
        return None
    chosen = lane_sections[0]
    for section in lane_sections:
        if float(section.attrib.get("s", "0")) <= s:
            chosen = section
        else:
            break
    return chosen


def _lane_by_id(section, lane_id: str):
    if section is None:
        return None
    right = section.find("right")
    if right is None:
        return None
    for lane in right.findall("lane"):
        if lane.attrib.get("id") == lane_id:
            return lane
    return None


def build_center_lane_path(carla, xodr_path: str, step_m: float, spawn_z: float = 1.0) -> List[PathPoint]:
    if not xodr_path or not os.path.exists(xodr_path):
        raise RuntimeError(f"XODR 文件不存在: {xodr_path}")

    tree = ET.parse(xodr_path)
    root = tree.getroot()
    road_map = {r.attrib["id"].strip(): r for r in root.findall("road")}
    seq = _choose_road_sequence(root)

    points: List[PathPoint] = []
    last_world = None
    s_acc = 0.0

    for road_id in seq:
        road = road_map.get(str(road_id))
        if road is None:
            continue

        ref_pts = _sample_road_reference_points(road, step_m)
        if not ref_pts:
            continue

        for s, x, y, hdg in ref_pts:
            lane_section = _lane_section_at(road, s)
            lane_minus1 = _lane_by_id(lane_section, "-1")
            lane_minus2 = _lane_by_id(lane_section, "-2")
            lane_offset = _lane_offset_at(road, s)
            if lane_minus2 is not None:
                s_local = max(0.0, s - float(lane_section.attrib.get("s", "0")) if lane_section is not None else s)
                w1 = _lane_width_at(lane_minus1, s_local)
                w2 = _lane_width_at(lane_minus2, s_local)
                lateral = lane_offset - (w1 + w2 / 2.0)
            else:
                lateral = lane_offset - 1.75

            # XODR meters -> CARLA meters; only flip Y to match your imported tunnel
            wx = x - lateral * math.sin(hdg)
            wy = -y + lateral * math.cos(hdg)

            if last_world is not None:
                dx = wx - last_world[0]
                dy = wy - last_world[1]
                if abs(dx) < 1e-3 and abs(dy) < 1e-3:
                    continue

            yaw = math.degrees(hdg)
            loc = carla.Location(wx, wy, spawn_z)
            rot = carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0)
            points.append(PathPoint(s=s_acc, transform=carla.Transform(loc, rot)))
            last_world = (wx, wy)
            s_acc += step_m

    if not points:
        raise RuntimeError("未从 XODR 提取到可用中心线")
    return points


def build_three_lane_paths(carla, xodr_path: str, step_m: float, spawn_z: float = 1.0) -> Dict[str, List[PathPoint]]:
    if not xodr_path or not os.path.exists(xodr_path):
        raise RuntimeError(f"XODR 文件不存在: {xodr_path}")

    tree = ET.parse(xodr_path)
    root = tree.getroot()
    road_map = {r.attrib["id"].strip(): r for r in root.findall("road")}
    seq = _choose_road_sequence(root)

    # 归一化三车道：以当前正确的中间车道（-2）为基准，左右车道使用常量车道宽度平移合成。
    # 这样可保证三条车道在可视化/代理车生成时始终平行且等间距。
    lane_width_m = float(os.getenv("TT_LANE_WIDTH_M", "3.5"))

    center_samples: List[Tuple[float, float, float]] = []
    last_center_xy: Tuple[float, float] | None = None
    started = False
    min_lane_width_m = 0.10

    for road_id in seq:
        road = road_map.get(str(road_id))
        if road is None:
            continue

        for s, x, y, hdg in _sample_road_reference_points(road, step_m):
            lane_section = _lane_section_at(road, s)
            if lane_section is None:
                continue
            lane_m1 = _lane_by_id(lane_section, "-1")
            lane_m2 = _lane_by_id(lane_section, "-2")
            lane_m3 = _lane_by_id(lane_section, "-3")
            lane_offset = _lane_offset_at(road, s)
            s_local = max(0.0, s - float(lane_section.attrib.get("s", "0")))
            w1 = _lane_width_at(lane_m1, s_local)
            w2 = _lane_width_at(lane_m2, s_local)
            w3 = _lane_width_at(lane_m3, s_local)

            is_valid_three_lane = (
                lane_m1 is not None
                and lane_m2 is not None
                and lane_m3 is not None
                and min(w1, w2, w3) >= min_lane_width_m
            )
            if not started:
                if not is_valid_three_lane:
                    continue
                started = True
            elif not is_valid_three_lane:
                break

            lateral2 = lane_offset - (w1 + w2 / 2.0)

            # 先构建中间车道（-2）的中心点（作为 source of truth）
            center_x = x - lateral2 * math.sin(hdg)
            center_y = -y + lateral2 * math.cos(hdg)
            if last_center_xy is not None and abs(center_x - last_center_xy[0]) < 1e-3 and abs(center_y - last_center_xy[1]) < 1e-3:
                continue
            last_center_xy = (center_x, center_y)

            center_samples.append((s, center_x, center_y))

    center_samples.reverse()
    if not center_samples:
        raise RuntimeError("未从 XODR 提取到可用中心线")

    lane_m1_points: List[PathPoint] = []
    lane_m2_points: List[PathPoint] = []
    lane_m3_points: List[PathPoint] = []
    total = len(center_samples)
    for idx, (s, center_x, center_y) in enumerate(center_samples):
        if total == 1:
            tangent_x, tangent_y = 1.0, 0.0
        elif idx == 0:
            next_x, next_y = center_samples[idx + 1][1], center_samples[idx + 1][2]
            tangent_x, tangent_y = next_x - center_x, next_y - center_y
        elif idx == total - 1:
            prev_x, prev_y = center_samples[idx - 1][1], center_samples[idx - 1][2]
            tangent_x, tangent_y = center_x - prev_x, center_y - prev_y
        else:
            prev_x, prev_y = center_samples[idx - 1][1], center_samples[idx - 1][2]
            next_x, next_y = center_samples[idx + 1][1], center_samples[idx + 1][2]
            tangent_x, tangent_y = next_x - prev_x, next_y - prev_y

        tangent_len = math.hypot(tangent_x, tangent_y)
        if tangent_len < 1e-9:
            tangent_x, tangent_y = 1.0, 0.0
            tangent_len = 1.0
        tangent_x /= tangent_len
        tangent_y /= tangent_len

        yaw_deg = math.degrees(math.atan2(tangent_y, tangent_x))
        rot = carla.Rotation(pitch=0.0, yaw=yaw_deg, roll=0.0)

        # 用中间车道自身切线构造左法向，确保弯道中也与中心线垂直
        left_nx = -tangent_y
        left_ny = tangent_x

        loc2 = carla.Location(center_x, center_y, spawn_z)
        loc1 = carla.Location(center_x + left_nx * lane_width_m, center_y + left_ny * lane_width_m, spawn_z)
        loc3 = carla.Location(center_x - left_nx * lane_width_m, center_y - left_ny * lane_width_m, spawn_z)

        lane_m2_points.append(PathPoint(s=s, transform=carla.Transform(loc2, rot)))
        lane_m1_points.append(PathPoint(s=s, transform=carla.Transform(loc1, rot)))
        lane_m3_points.append(PathPoint(s=s, transform=carla.Transform(loc3, rot)))

    return {"-1": lane_m1_points, "-2": lane_m2_points, "-3": lane_m3_points}
