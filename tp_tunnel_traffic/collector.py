from __future__ import annotations

import json
import math
import os
import platform
import sys
import time
import queue
from dataclasses import asdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


@dataclass(frozen=True)
class CameraSpec:
    name: str
    x: float
    y: float
    z: float
    pitch: float
    yaw: float
    roll: float
    width: int = 800
    height: int = 600
    fov: int = 90


class DatasetCollector:
    def __init__(self, carla, world, vehicle, config, lane_points) -> None:
        self.carla = carla
        self.world = world
        self.vehicle = vehicle
        self.config = config
        self.lane_points = lane_points
        self.sensors = []
        self.frame_queue: "queue.Queue[tuple[str, int, Any]]" = queue.Queue(maxsize=2000)
        self.root_dir = Path(config.collect_output_dir)
        self.images_dir = self.root_dir / "images"
        self.labels_path = self.root_dir / "labels.jsonl"
        self.metadata_path = self.root_dir / "metadata.json"
        self.cameras: List[CameraSpec] = []
        self.enabled: bool = True
        self._output_prepared: bool = False
        self._callbacks_enabled: bool = False
        self._callback_generation: int = 0
        self._pending_frames: Dict[int, Dict[str, Any]] = {}
        self._missed_stride_count: int = 0
        self._sensors_by_name: Dict[str, Any] = {}
        self._camera_spec_by_name: Dict[str, CameraSpec] = {}
        self._active_cameras: List[CameraSpec] = []
        self._coco_state: Dict[str, Any] = {
            "images": [],
            "annotations": [],
            "categories": [{"id": 1, "name": "vehicle"}],
            "image_id": 0,
            "annotation_id": 0,
        }
        self._stats: Dict[str, int] = {
            "ticks": 0,
            "stride_ticks": 0,
            "writes": 0,
            "miss_no_complete_frame": 0,
            "miss_missing_camera": 0,
            "label_write_fail": 0,
            "image_write_fail": 0,
        }
        self._load_camera_specs()

    def get_output_root(self) -> Path:
        return Path(self.root_dir)

    def _target_dir_name(self) -> str:
        target_id = getattr(self.vehicle, "id", None)
        if target_id is None:
            return "target_unknown"
        return f"target_{int(target_id)}"

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = bool(enabled)
        if not self.enabled:
            self._stop_sensors()
            self._pending_frames.clear()
            self.frame_queue = queue.Queue(maxsize=2000)
            self._missed_stride_count = 0
            for k in list(self._stats.keys()):
                self._stats[k] = 0

    def _stop_sensors(self) -> None:
        self._callbacks_enabled = False
        for sensor in self.sensors:
            try:
                sensor.stop()
            except Exception:
                pass
            try:
                sensor.destroy()
            except Exception:
                pass
        self.sensors.clear()
        self._sensors_by_name.clear()

    def set_vehicle(self, vehicle, lane_points=None, output_root: Path | None = None) -> None:
        self.vehicle = vehicle
        if lane_points is not None:
            self.lane_points = lane_points
        if output_root is not None:
            self.root_dir = Path(output_root)
        # main.py already passes a per-target output_root; do not auto-nest again.
        self.images_dir = self.root_dir / "images"
        self.labels_path = self.root_dir / "labels.jsonl"
        self.metadata_path = self.root_dir / "metadata.json"
        self._output_prepared = False
        self._callback_generation += 1
        self._stop_sensors()
        self._pending_frames.clear()
        self.frame_queue = queue.Queue(maxsize=2000)
        self._camera_spec_by_name.clear()
        self._active_cameras = []
        self._reset_coco_state()

    def _reset_coco_state(self) -> None:
        self._coco_state = {
            "images": [],
            "annotations": [],
            "categories": [{"id": 1, "name": "vehicle"}],
            "image_id": 0,
            "annotation_id": 0,
        }

    def _prepare_output(self) -> None:
        if self._output_prepared:
            return
        _ensure_dir(self.root_dir)
        _ensure_dir(self.images_dir)
        _ensure_dir(self.root_dir / "labels_2d")
        self._output_prepared = True

    def _load_camera_specs(self) -> None:
        cfg_path = Path(self.config.collect_cameras_json)
        cameras: List[CameraSpec] = []
        if cfg_path.exists():
            with open(cfg_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            cameras = [
                CameraSpec(
                    name=str(item["name"]),
                    x=float(item["x"]),
                    y=float(item["y"]),
                    z=float(item["z"]),
                    pitch=float(item.get("pitch", 0.0)),
                    yaw=float(item.get("yaw", 0.0)),
                    roll=float(item.get("roll", 0.0)),
                    width=int(item.get("width", 800)),
                    height=int(item.get("height", 600)),
                    fov=int(item.get("fov", 90)),
                )
                for item in data
            ]
        else:
            cameras = [CameraSpec("ego", 0.65, -0.18, 1.22, -7.5, 0.0, 0.0)]

        # 默认第一人称视角优先使用 camera_offsets.json 中的驾驶视角配置
        ego_cfg_path = Path(self.config.camera_tune_json)
        if ego_cfg_path.exists():
            with open(ego_cfg_path, "r", encoding="utf-8") as f:
                ego = json.load(f)
            ego_spec = CameraSpec(
                name="ego",
                x=float(ego.get("cam_forward_m", 0.65)),
                y=float(ego.get("cam_right_m", -0.18)),
                z=float(ego.get("cam_up_m", 1.22)),
                pitch=float(ego.get("cam_pitch_deg", -7.5)),
                yaw=float(ego.get("cam_yaw_offset_deg", 0.0)),
                roll=0.0,
                width=800,
                height=600,
                fov=90,
            )
            cameras = [c for c in cameras if c.name != "ego"]
            cameras.insert(0, ego_spec)

        self.cameras = cameras

    def _build_active_cameras(self) -> List[CameraSpec]:
        enabled_instance = bool(getattr(self.config, "collect_enable_instance_segmentation", False))
        if not enabled_instance:
            return list(self.cameras)
        suffix = str(getattr(self.config, "collect_instance_suffix", "_instance"))
        names = {spec.name for spec in self.cameras}
        active = list(self.cameras)
        for spec in list(self.cameras):
            if spec.name.endswith(suffix):
                continue
            inst_name = f"{spec.name}{suffix}"
            if inst_name in names:
                continue
            inst_spec = CameraSpec(
                name=inst_name,
                x=spec.x,
                y=spec.y,
                z=spec.z,
                pitch=spec.pitch,
                yaw=spec.yaw,
                roll=spec.roll,
                width=spec.width,
                height=spec.height,
                fov=spec.fov,
            )
            active.append(inst_spec)
            names.add(inst_name)
        return active

    def _resolve_camera_type(self, spec: CameraSpec) -> str:
        suffix = str(getattr(self.config, "collect_instance_suffix", "_instance"))
        if not bool(getattr(self.config, "collect_enable_instance_segmentation", False)):
            return "sensor.camera.rgb"
        if spec.name.endswith(suffix):
            return "sensor.camera.instance_segmentation"
        return "sensor.camera.rgb"

    def _is_instance_camera(self, name: str) -> bool:
        suffix = str(getattr(self.config, "collect_instance_suffix", "_instance"))
        return bool(getattr(self.config, "collect_enable_instance_segmentation", False)) and name.endswith(suffix)

    def _build_projection_matrix(self, width: int, height: int, fov: float) -> List[List[float]]:
        focal = float(width) / (2.0 * math.tan(float(fov) * math.pi / 360.0))
        return [
            [focal, 0.0, float(width) / 2.0],
            [0.0, focal, float(height) / 2.0],
            [0.0, 0.0, 1.0],
        ]

    def _world_to_camera_matrix(self, camera_tf) -> List[List[float]]:
        return camera_tf.get_inverse_matrix()

    def _project_point(self, world_point, k, w2c):
        x, y, z = float(world_point.x), float(world_point.y), float(world_point.z)
        cx = w2c[0][0] * x + w2c[0][1] * y + w2c[0][2] * z + w2c[0][3]
        cy = w2c[1][0] * x + w2c[1][1] * y + w2c[1][2] * z + w2c[1][3]
        cz = w2c[2][0] * x + w2c[2][1] * y + w2c[2][2] * z + w2c[2][3]
        # CARLA (UE) to camera coordinates: (x, y, z) -> (y, -z, x)
        cx, cy, cz = cy, -cz, cx
        if cz <= 0:
            return None
        u = (k[0][0] * cx / cz) + k[0][2]
        v = (k[1][1] * cy / cz) + k[1][2]
        return float(u), float(v), float(cz)

    def _compute_actor_bbox_2d(self, actor, camera_tf, k, width, height):
        try:
            bb = actor.bounding_box
            actor_tf = actor.get_transform()
        except Exception:
            return None
        try:
            vertices = bb.get_world_vertices(actor_tf)
        except Exception:
            return None

        w2c = self._world_to_camera_matrix(camera_tf)
        xs: List[float] = []
        ys: List[float] = []
        for v in vertices:
            proj = self._project_point(v, k, w2c)
            if proj is None:
                continue
            u, v2, _ = proj
            xs.append(u)
            ys.append(v2)
        if not xs or not ys:
            return None
        x_min = max(0.0, min(xs))
        x_max = min(float(width - 1), max(xs))
        y_min = max(0.0, min(ys))
        y_max = min(float(height - 1), max(ys))
        if x_max <= x_min or y_max <= y_min:
            return None
        return x_min, y_min, x_max, y_max

    def _write_coco_annotation(self, frame: int, camera_name: str, width: int, height: int, image_path: str, bboxes):
        coco = self._coco_state
        coco["image_id"] += 1
        image_id = coco["image_id"]
        coco["images"].append(
            {
                "id": image_id,
                "file_name": image_path.replace("\\", "/"),
                "width": int(width),
                "height": int(height),
                "frame": int(frame),
                "camera": camera_name,
            }
        )
        for bbox in bboxes:
            coco["annotation_id"] += 1
            ann_id = coco["annotation_id"]
            x_min, y_min, x_max, y_max, actor_id = bbox
            w = float(x_max - x_min)
            h = float(y_max - y_min)
            coco["annotations"].append(
                {
                    "id": ann_id,
                    "image_id": image_id,
                    "category_id": 1,
                    "bbox": [float(x_min), float(y_min), w, h],
                    "area": float(w * h),
                    "iscrowd": 0,
                    "actor_id": int(actor_id),
                }
            )

    def _flush_coco(self) -> None:
        if not bool(getattr(self.config, "collect_write_coco", True)):
            return
        coco_path = self.root_dir / "labels_2d" / "coco_instances.json"
        try:
            with open(coco_path, "w", encoding="utf-8") as f:
                json.dump(self._coco_state, f, ensure_ascii=False, indent=2)
        except Exception:
            return

    def spawn_sensors(self):
        if self.vehicle is None:
            return self.sensors
        self._prepare_output()
        self._callbacks_enabled = True
        bp_lib = self.world.get_blueprint_library()
        generation = int(self._callback_generation)
        active_cameras = self._build_active_cameras()
        self._active_cameras = active_cameras
        for spec in active_cameras:
            camera_bp = bp_lib.find(self._resolve_camera_type(spec))
            camera_bp.set_attribute("image_size_x", str(spec.width))
            camera_bp.set_attribute("image_size_y", str(spec.height))
            camera_bp.set_attribute("fov", str(spec.fov))
            # Ensure deterministic capture rate in sync mode.
            try:
                camera_bp.set_attribute("sensor_tick", "0.0")
            except Exception:
                pass
            transform = self.carla.Transform(
                self.carla.Location(spec.x, spec.y, spec.z),
                self.carla.Rotation(pitch=spec.pitch, yaw=spec.yaw, roll=spec.roll),
            )
            sensor = self.world.spawn_actor(camera_bp, transform, attach_to=self.vehicle)
            self._sensors_by_name[spec.name] = sensor
            self._camera_spec_by_name[spec.name] = spec
            q = self.frame_queue

            def _on_image(image, name=spec.name, expected_generation=generation, target_queue=q):
                if not self._callbacks_enabled:
                    return
                if expected_generation != self._callback_generation:
                    return
                try:
                    target_queue.put_nowait((name, int(image.frame), image))
                except queue.Full:
                    pass
                except Exception:
                    pass

            sensor.listen(_on_image)
            self.sensors.append(sensor)
        self._write_metadata()
        return self.sensors

    def _ensure_capture_ready(self) -> bool:
        if self.vehicle is None:
            return False
        if not bool(getattr(self.vehicle, "is_alive", False)):
            return False
        if not self.sensors:
            try:
                self.spawn_sensors()
            except Exception:
                self.enabled = False
                self._stop_sensors()
                return False
        return bool(self.sensors)

    def _drain_frame_queue(self) -> None:
        while not self.frame_queue.empty():
            try:
                name, image_frame, image = self.frame_queue.get_nowait()
            except Exception:
                break
            bucket = self._pending_frames.setdefault(int(image_frame), {})
            bucket[str(name)] = image
        if not self._pending_frames:
            return
        latest = max(self._pending_frames.keys())
        min_keep = latest - 30
        obsolete = [f for f in self._pending_frames.keys() if f < min_keep]
        for f in obsolete:
            self._pending_frames.pop(f, None)

    def _select_frame_for_write(self, target_frame: int, camera_names: List[str]) -> tuple[int | None, Dict[str, Any] | None]:
        if not self._pending_frames:
            return None, None
        complete_frames = [
            f for f, images in self._pending_frames.items()
            if all(name in images for name in camera_names)
        ]
        if not complete_frames:
            return None, None

        not_ahead = [f for f in complete_frames if f <= int(target_frame)]
        if not_ahead:
            chosen_frame = max(not_ahead)
        else:
            # 兜底：若暂时只有略超前帧，取最接近的完整帧，避免采集中断。
            chosen_frame = min(complete_frames)

        frame_images = self._pending_frames.pop(chosen_frame, None)
        if frame_images is None:
            return None, None

        # 清理更旧的历史帧，避免重复占用内存。
        obsolete = [f for f in self._pending_frames.keys() if f < chosen_frame - 5]
        for f in obsolete:
            self._pending_frames.pop(f, None)
        return int(chosen_frame), frame_images

    def _log_stats_if_needed(self) -> None:
        ticks = int(self._stats.get("ticks", 0))
        if ticks <= 0:
            return
        if ticks % 200 != 0:
            return
        try:
            stride_ticks = int(self._stats.get("stride_ticks", 0))
            writes = int(self._stats.get("writes", 0))
            miss_no_complete = int(self._stats.get("miss_no_complete_frame", 0))
            miss_missing_cam = int(self._stats.get("miss_missing_camera", 0))
            label_fail = int(self._stats.get("label_write_fail", 0))
            img_fail = int(self._stats.get("image_write_fail", 0))
            print(
                "采集统计: "
                f"ticks={ticks} stride_ticks={stride_ticks} writes={writes} "
                f"miss_no_complete={miss_no_complete} miss_missing_cam={miss_missing_cam} "
                f"label_fail={label_fail} image_fail={img_fail}"
            )
        except Exception:
            pass

    def _write_effective_config_snapshot(self) -> None:
        """Write a run-level config snapshot for reproducibility.

        This captures:
        - The effective (resolved) TunnelTrafficConfig values
        - The relevant TT_/TP_ environment variables
        - Basic runtime metadata (argv, python, conda env)
        """
        try:
            env = {k: v for k, v in os.environ.items() if k.startswith(("TT_", "TP_"))}
            env_sorted = {k: env[k] for k in sorted(env.keys())}
        except Exception:
            env_sorted = {}

        try:
            effective = asdict(self.config)
        except Exception:
            # Fallback: best-effort dict view
            effective = dict(getattr(self.config, "__dict__", {}))

        payload = {
            "created_at_unix": float(time.time()),
            "output_dir": str(self.root_dir),
            "effective_config": effective,
            "env": env_sorted,
            "runtime": {
                "argv": list(sys.argv) if hasattr(sys, "argv") else [],
                "python": str(platform.python_version()),
                "python_executable": str(getattr(sys, "executable", "")),
                "conda_default_env": str(os.getenv("CONDA_DEFAULT_ENV", "")),
            },
        }
        try:
            with open(self.root_dir / "effective_config.json", "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
        except Exception:
            return

    def _write_metadata(self) -> None:
        try:
            world_map = self.world.get_map()
            map_name = str(getattr(world_map, "name", ""))
        except Exception:
            map_name = ""

        try:
            client_version = str(getattr(self.carla, "__version__", ""))
        except Exception:
            client_version = ""

        meta = {
            "xodr_path": self.config.xodr_path,
            "map_name": map_name,
            "carla_client_version": client_version,
            "platform": {
                "python": str(platform.python_version()),
                "system": str(platform.system()),
                "release": str(platform.release()),
            },
            "created_at_unix": float(time.time()),
            "output_dir": str(self.root_dir),
            "cameras": [spec.__dict__ for spec in self.cameras],
            "instance_segmentation": {
                "enabled": bool(getattr(self.config, "collect_enable_instance_segmentation", False)),
                "suffix": str(getattr(self.config, "collect_instance_suffix", "_instance")),
            },
            "coco_2d": {
                "enabled": bool(getattr(self.config, "collect_write_coco", True)),
                "category_ids": {"vehicle": 1},
                "min_area_px2": float(getattr(self.config, "collect_coco_min_area_px2", 0.0)),
                "max_height_ratio": float(getattr(self.config, "collect_coco_max_height_ratio", 0.0)),
            },
            "step_m": self.config.step_m,
            "collect_frame_stride": self.config.collect_frame_stride,
            "fixed_delta_seconds": float(getattr(self.config, "fixed_delta_seconds", 0.0)),
            "sync_mode": bool(getattr(self.config, "sync_mode", True)),
            "target_actor_id": int(getattr(self.vehicle, "id", -1)) if self.vehicle is not None else -1,
        }
        with open(self.metadata_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        # Run-level reproducibility snapshot.
        self._write_effective_config_snapshot()

        # Also write a run-level marker for downstream tooling.
        try:
            marker = {
                "target_actor_id": meta["target_actor_id"],
                "output_dir": meta["output_dir"],
            }
            with open(self.root_dir / "target.json", "w", encoding="utf-8") as f:
                json.dump(marker, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def on_tick(self, frame_idx: int, nearest_idx: int, control) -> None:
        self._stats["ticks"] = int(self._stats.get("ticks", 0)) + 1
        self._drain_frame_queue()
        if not bool(self.enabled):
            return
        if not self._ensure_capture_ready():
            return
        if frame_idx % max(1, int(self.config.collect_frame_stride)) != 0:
            return
        self._stats["stride_ticks"] = int(self._stats.get("stride_ticks", 0)) + 1

        camera_names = [spec.name for spec in self._active_cameras]
        chosen_frame, frame_images = self._select_frame_for_write(int(frame_idx), camera_names)
        if chosen_frame is None or frame_images is None:
            self._stats["miss_no_complete_frame"] = int(self._stats.get("miss_no_complete_frame", 0)) + 1
            self._missed_stride_count += 1
            if self._missed_stride_count % 20 == 0:
                print(
                    f"采集提示: 连续{self._missed_stride_count}次采样点未凑齐完整相机帧，"
                    f"latest_tick={int(frame_idx)}"
                )
            self._log_stats_if_needed()
            return
        self._missed_stride_count = 0

        if any(name not in frame_images for name in camera_names):
            self._stats["miss_missing_camera"] = int(self._stats.get("miss_missing_camera", 0)) + 1
            self._log_stats_if_needed()
            return

        try:
            vehicle_loc = self.vehicle.get_location()
            vehicle_tf = self.vehicle.get_transform()
            try:
                vel = self.vehicle.get_velocity()
                speed_mps = (float(vel.x) ** 2 + float(vel.y) ** 2 + float(vel.z) ** 2) ** 0.5
            except Exception:
                speed_mps = 0.0
            label = {
                "frame": int(chosen_frame),
                "world_frame": int(chosen_frame),
                "nearest_idx": int(nearest_idx),
                "actor_id": int(getattr(self.vehicle, "id", -1)),
                "throttle": float(control.throttle),
                "steer": float(control.steer),
                "brake": float(control.brake),
                "hand_brake": bool(control.hand_brake),
                "reverse": bool(control.reverse),
                "speed_mps": float(speed_mps),
                "vehicle_x": float(vehicle_loc.x),
                "vehicle_y": float(vehicle_loc.y),
                "vehicle_z": float(vehicle_loc.z),
                "vehicle_yaw": float(getattr(vehicle_tf.rotation, "yaw", 0.0)),
            }
            if bool(getattr(self.config, "collect_record_labels", True)):
                with open(self.labels_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(label, ensure_ascii=False) + "\n")
        except Exception:
            self._stats["label_write_fail"] = int(self._stats.get("label_write_fail", 0)) + 1
            self._log_stats_if_needed()
            return

        for name in camera_names:
            image = frame_images.get(name)
            if image is None:
                continue
            camera_dir = self.images_dir / name
            try:
                _ensure_dir(camera_dir)
                image.save_to_disk(str(camera_dir / f"{int(chosen_frame):06d}.png"))
            except Exception:
                self._stats["image_write_fail"] = int(self._stats.get("image_write_fail", 0)) + 1
                continue

        if bool(getattr(self.config, "collect_write_coco", True)):
            self._write_coco_for_frame(int(chosen_frame), frame_images)
            self._flush_coco()

        self._stats["writes"] = int(self._stats.get("writes", 0)) + 1
        self._log_stats_if_needed()

    def destroy(self) -> None:
        self._stop_sensors()

    def _write_coco_for_frame(self, frame: int, frame_images: Dict[str, Any]) -> None:
        try:
            actors = self.world.get_actors().filter("vehicle.*")
        except Exception:
            return
        target_actor_id = int(getattr(self.vehicle, "id", -1)) if self.vehicle is not None else -1
        for name in list(frame_images.keys()):
            if self._is_instance_camera(name):
                continue
            spec = self._camera_spec_by_name.get(name)
            if spec is None:
                continue
            sensor = self._sensors_by_name.get(name)
            if sensor is None:
                continue
            try:
                camera_tf = sensor.get_transform()
            except Exception:
                continue
            k = self._build_projection_matrix(spec.width, spec.height, spec.fov)
            bboxes = []
            for actor in actors:
                if target_actor_id >= 0 and int(getattr(actor, "id", -2)) == target_actor_id:
                    continue
                bbox = self._compute_actor_bbox_2d(actor, camera_tf, k, spec.width, spec.height)
                if bbox is None:
                    continue
                x_min, y_min, x_max, y_max = bbox
                area = float((x_max - x_min) * (y_max - y_min))
                if area < float(getattr(self.config, "collect_coco_min_area_px2", 0.0)):
                    continue
                max_h_ratio = float(getattr(self.config, "collect_coco_max_height_ratio", 0.0))
                if max_h_ratio > 0.0:
                    h_ratio = float((y_max - y_min) / max(1.0, float(spec.height)))
                    if h_ratio > max_h_ratio:
                        continue
                bboxes.append((x_min, y_min, x_max, y_max, int(actor.id)))
            image_path = str(Path("images") / name / f"{int(frame):06d}.png")
            self._write_coco_annotation(frame, name, spec.width, spec.height, image_path, bboxes)
