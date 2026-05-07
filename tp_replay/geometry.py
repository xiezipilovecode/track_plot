import math

import numpy as np

from . import config


def get_vector_angle_degrees(dx, dy):
    return math.degrees(math.atan2(dy, dx))


def is_in_front(point_loc, entry_loc, forward_vec):
    p_vec = np.array([point_loc.x, point_loc.y])
    e_vec = np.array([entry_loc.x, entry_loc.y])
    f_vec = np.array([forward_vec.x, forward_vec.y])
    return np.dot(p_vec - e_vec, f_vec) >= -10.0  # 放宽判定区域


def _extract_xy_points_from_tokens(tokens, max_points=None):
    """从 6 元组 token 序列中提取 (x,y) 原始点（未缩放），用于方向估计。"""

    points = []
    for i in range(0, len(tokens), 6):
        if i + 3 >= len(tokens):
            break
        try:
            x = float(tokens[i + 1])
            y = float(tokens[i + 2])
        except (ValueError, IndexError):
            continue
        if abs(x) < 1.0:
            continue
        points.append((x, y))
        if max_points is not None and len(points) >= max_points:
            break
    return points


def _calculate_stable_data_angle(points):
    """用长基线估计 data_angle，降低短基线噪声导致的全局旋转误差。"""

    if len(points) < 2:
        return 0.0

    p_start = points[0]
    target_idx = min(5, len(points) - 1)
    for i in range(5, len(points)):
        dist = math.hypot(points[i][0] - p_start[0], points[i][1] - p_start[1]) * float(config.DATA_SCALE)
        if dist >= float(config.DATA_ANGLE_BASELINE_M):
            target_idx = i
            break

    p_end = points[target_idx]
    vec_dx = (p_end[0] - p_start[0]) * float(config.DATA_SCALE)
    vec_dy = (p_end[1] - p_start[1]) * float(config.DATA_SCALE) * float(config.SCALE_Y)
    return get_vector_angle_degrees(vec_dx, vec_dy)
