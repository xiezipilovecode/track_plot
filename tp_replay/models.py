from .carla_compat import require_carla


class TrackFrame:
    __slots__ = ["ts", "loc", "rot", "v"]

    def __init__(self, ts, loc, rot, v):
        self.ts = ts
        self.loc = loc
        self.rot = rot
        self.v = v


class VehicleTrack:
    def __init__(self, vehicle_id, type_str):
        self.id = vehicle_id
        self.type_str = type_str
        self.frames = []
        self.actor = None
        self.spawned = False
        self.finished = False
        self.spawn_attempts = 0
        # 仅用于“何时尝试生成”的调度时间，避免修改 start_time 影响回放时间轴
        self.next_spawn_time = float("inf")
        self.just_spawned = False
        self.start_time = float("inf")
        self.end_time = -float("inf")
        self.finish_timer = 0.0

        # 可选：时间轴偏移（用于回放调度/轨迹复用）。
        self.time_offset = 0.0
        self.loop_count = 0

        # control state
        self.teleport_over_ticks = 0
        self.teleport_cooldown_ticks = 0
        self.cmd_vx = None
        self.cmd_vy = None
        self.cmd_speed = None
        self.catchup_err_f = 0.0
        self.cmd_wz = 0.0

    def add_frame(self, frame):
        if self.frames and frame.ts <= self.frames[-1].ts:
            return
        self.frames.append(frame)
        if frame.ts < self.start_time:
            self.start_time = frame.ts
        if frame.ts > self.end_time:
            self.end_time = frame.ts

    def sort_frames(self):
        self.frames.sort(key=lambda x: x.ts)

    def get_state_at_time(self, rel_time):
        if not self.frames:
            return None

        local_time = float(rel_time) - float(getattr(self, "time_offset", 0.0))
        if local_time < self.start_time:
            return None
        if local_time > self.end_time:
            return "FINISHED"

        for i in range(len(self.frames) - 1):
            f1 = self.frames[i]
            f2 = self.frames[i + 1]
            if f1.ts <= local_time <= f2.ts:
                total = f2.ts - f1.ts
                if total <= 1e-5:
                    return f1
                ratio = (local_time - f1.ts) / total

                # 位置插值
                ix = f1.loc.x + (f2.loc.x - f1.loc.x) * ratio
                iy = f1.loc.y + (f2.loc.y - f1.loc.y) * ratio
                iz = f1.loc.z + (f2.loc.z - f1.loc.z) * ratio

                # 速度插值
                iv = f1.v + (f2.v - f1.v) * ratio

                # 角度插值 (最短路径)
                diff = f2.rot.yaw - f1.rot.yaw
                if diff > 180:
                    diff -= 360
                elif diff < -180:
                    diff += 360
                iyaw = f1.rot.yaw + diff * ratio

                carla = require_carla()
                return TrackFrame(
                    rel_time,
                    carla.Location(ix, iy, iz),
                    carla.Rotation(0, iyaw, 0),
                    iv,
                )
        return "FINISHED"

    def global_start_time(self):
        return float(self.start_time) + float(getattr(self, "time_offset", 0.0))

    def global_end_time(self):
        return float(self.end_time) + float(getattr(self, "time_offset", 0.0))
