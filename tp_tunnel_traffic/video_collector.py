from __future__ import annotations

import json
import platform
import queue
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List


def _ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


class _CameraWriter:
    """Per-camera background writer.

    Uses FFmpeg for real-time MP4 encoding when available; falls back
    to PNG frames written from the background thread to avoid blocking
    the simulation tick.
    """

    def __init__(self, output_dir: Path, name: str, width: int, height: int, fps: float):
        self.name = name
        self.width = width
        self.height = height
        self.fps = max(1.0, float(fps))
        self.output_dir = output_dir
        self._queue: queue.Queue[tuple[int, bytes] | None] = queue.Queue(maxsize=600)
        self._thread: threading.Thread | None = None
        self._running = False
        self._ffmpeg: subprocess.Popen | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def feed(self, frame: int, raw_data: bytes) -> None:
        """Non-blocking: push a frame into the write queue."""
        try:
            self._queue.put_nowait((int(frame), bytes(raw_data)))
        except queue.Full:
            pass  # drop frame rather than block the simulation

    def stop(self) -> None:
        self._running = False
        try:
            self._queue.put_nowait(None)  # sentinel
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=15.0)

    def _run(self) -> None:
        ffmpeg = self._start_ffmpeg()
        png_dir = None

        while self._running or not self._queue.empty():
            try:
                item = self._queue.get(timeout=0.3)
            except queue.Empty:
                continue
            if item is None:  # sentinel
                break

            frame, raw_data = item
            if ffmpeg is not None:
                try:
                    ffmpeg.stdin.write(raw_data)  # type: ignore[union-attr]
                except Exception:
                    ffmpeg = None
                    png_dir = self._ensure_png_dir()
            if ffmpeg is None:
                if png_dir is None:
                    png_dir = self._ensure_png_dir()
                try:
                    png_path = png_dir / f"{int(frame):06d}.png"
                    # raw_data is BGRA; write raw bytes then let user open
                    with open(png_path, "wb") as f:
                        f.write(raw_data)
                except Exception:
                    pass

        if ffmpeg is not None and ffmpeg.stdin is not None:
            try:
                ffmpeg.stdin.close()
                ffmpeg.wait(timeout=10)
            except Exception:
                pass

    def _start_ffmpeg(self) -> subprocess.Popen | None:
        out_path = self.output_dir / f"{self.name}.mp4"
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo", "-vcodec", "rawvideo",
            "-s", f"{self.width}x{self.height}",
            "-pix_fmt", "bgra",
            "-r", str(self.fps),
            "-i", "pipe:0",
            "-c:v", "libx264", "-preset", "ultrafast",
            "-crf", "23", "-pix_fmt", "yuv420p",
            str(out_path),
        ]
        try:
            proc = subprocess.Popen(
                cmd, stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            return proc
        except Exception:
            return None

    def _ensure_png_dir(self) -> Path:
        d = self.output_dir / "images" / self.name
        _ensure_dir(d)
        return d


class VideoCollector:
    """Continuous video recorder with background async encoding.

    - on_tick() only pushes raw frame bytes into a lock-free queue.
    - A background thread + FFmpeg (or png fallback) handles all I/O.
    - Follows the currently selected proxy vehicle; switching vehicle
      automatically starts a new recording session.
    """

    def __init__(self, carla, world, config, lane_points) -> None:
        self.carla = carla
        self.world = world
        self.config = config
        self.lane_points = lane_points
        self.sensors: list = []
        self._sensors_by_name: Dict[str, Any] = {}
        self.frame_queue: "queue.Queue[tuple[str, int, Any]]" = queue.Queue(maxsize=3000)
        self.root_dir = Path(getattr(config, "video_output_dir", "dataset_video"))
        self.writers: Dict[str, _CameraWriter] = {}
        self.recording = False
        self._callbacks_enabled = False
        self._callback_generation = 0
        self._pending_frames: Dict[int, Dict[str, Any]] = {}
        self._current_vehicle = None
        self._current_run_dir: Path | None = None
        self._start_time = 0.0
        self._frame_count = 0
        self._camera_specs: List[Dict[str, Any]] = []
        self._load_camera_specs()

    # ------------------------------------------------------------------
    #  public API
    # ------------------------------------------------------------------

    def set_vehicle(self, vehicle, lane_points=None) -> None:
        """Called when the user selects a different proxy in the GUI.

        If already recording, the previous session is finalized and a new
        one starts automatically.
        """
        was_recording = self.recording
        if was_recording:
            self._finalize_session()
        self._current_vehicle = vehicle
        if lane_points is not None:
            self.lane_points = lane_points
        if was_recording:
            self._start_session()

    def start(self) -> None:
        """Begin recording (called from GUI Record Video ON)."""
        if self.recording:
            return
        self._start_session()
        print(f"VideoCollector: recording started -> {self._current_run_dir}")

    def stop(self) -> None:
        """Stop recording (called from GUI Record Video OFF)."""
        if not self.recording:
            return
        self._finalize_session()
        print(f"VideoCollector: recording stopped ({self._frame_count} frames)")

    def on_tick(self, world_frame: int) -> None:
        """Call every simulation tick. Fast path: drain sensor frames into queues only."""
        if not self.recording:
            return
        self._drain_queue()
        chosen_frame, frame_images = self._select_frame(world_frame)
        if chosen_frame is None or frame_images is None:
            return
        # Non-blocking: feed raw bytes to background writers
        self._feed_writers(chosen_frame, frame_images)

    def destroy(self) -> None:
        self.stop()
        self._stop_sensors()

    # ------------------------------------------------------------------
    #  session management
    # ------------------------------------------------------------------

    def _start_session(self) -> None:
        if self._current_vehicle is None:
            return
        timestamp = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self._current_run_dir = self.root_dir / timestamp
        self._current_run_dir.mkdir(parents=True, exist_ok=True)
        self._start_time = time.time()
        self._frame_count = 0
        self.writers.clear()
        # Create one writer per camera
        fps = 1.0 / max(0.01, float(getattr(self.config, "fixed_delta_seconds", 0.05)))
        for spec in self._camera_specs:
            w = spec["width"]
            h = spec["height"]
            name = spec["name"]
            writer = _CameraWriter(self._current_run_dir, name, w, h, fps)
            writer.start()
            self.writers[name] = writer
        self._spawn_sensors()
        self._write_metadata()
        self.recording = True

    def _finalize_session(self) -> None:
        self.recording = False
        self._stop_sensors()
        self._pending_frames.clear()
        for writer in self.writers.values():
            writer.stop()
        self.writers.clear()

    # ------------------------------------------------------------------
    #  sensor management
    # ------------------------------------------------------------------

    def _load_camera_specs(self) -> None:
        cfg_path = Path(getattr(self.config, "video_cameras_json",
                                r"tp_tunnel_traffic\dataset_cameras.json"))
        specs: List[Dict[str, Any]] = []
        if cfg_path.exists():
            import json as _json
            with open(cfg_path, "r", encoding="utf-8") as f:
                data = _json.load(f)
            instance_suffix = str(getattr(self.config, "collect_instance_suffix", "_instance"))
            for item in data:
                name = str(item["name"])
                if name.endswith(instance_suffix):
                    continue
                specs.append({
                    "name": name,
                    "x": float(item["x"]), "y": float(item["y"]), "z": float(item["z"]),
                    "pitch": float(item.get("pitch", 0.0)),
                    "yaw": float(item.get("yaw", 0.0)),
                    "roll": float(item.get("roll", 0.0)),
                    "width": int(item.get("width", 800)),
                    "height": int(item.get("height", 600)),
                    "fov": int(item.get("fov", 90)),
                })
        if not specs:
            specs = [{"name": "ego", "x": 0.65, "y": -0.18, "z": 1.22,
                       "pitch": -7.5, "yaw": 0.0, "roll": 0.0,
                       "width": 800, "height": 600, "fov": 90}]
        self._camera_specs = specs

    def _spawn_sensors(self) -> None:
        if self._current_vehicle is None:
            return
        self._callbacks_enabled = True
        bp_lib = self.world.get_blueprint_library()
        generation = int(self._callback_generation)
        for spec in self._camera_specs:
            camera_bp = bp_lib.find("sensor.camera.rgb")
            camera_bp.set_attribute("image_size_x", str(spec["width"]))
            camera_bp.set_attribute("image_size_y", str(spec["height"]))
            camera_bp.set_attribute("fov", str(spec["fov"]))
            try:
                camera_bp.set_attribute("sensor_tick", "0.0")
            except Exception:
                pass
            transform = self.carla.Transform(
                self.carla.Location(spec["x"], spec["y"], spec["z"]),
                self.carla.Rotation(pitch=spec["pitch"], yaw=spec["yaw"], roll=spec["roll"]),
            )
            sensor = self.world.spawn_actor(camera_bp, transform, attach_to=self._current_vehicle)
            self._sensors_by_name[spec["name"]] = sensor
            q = self.frame_queue

            def _on_image(image, name=spec["name"], expected_generation=generation, target_queue=q):
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
        self._callback_generation += 1

    # ------------------------------------------------------------------
    #  frame routing
    # ------------------------------------------------------------------

    def _drain_queue(self) -> None:
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
        obsolete = [f for f in self._pending_frames.keys() if f < latest - 30]
        for f in obsolete:
            self._pending_frames.pop(f, None)

    def _select_frame(self, target_frame: int) -> tuple[int | None, Dict[str, Any] | None]:
        if not self._pending_frames:
            return None, None
        cam_names = [s["name"] for s in self._camera_specs]
        complete = [f for f, imgs in self._pending_frames.items()
                    if all(name in imgs for name in cam_names)]
        if not complete:
            return None, None
        not_ahead = [f for f in complete if f <= target_frame]
        chosen = max(not_ahead) if not_ahead else min(complete)
        frame_images = self._pending_frames.pop(chosen, None)
        return (int(chosen), frame_images)

    def _feed_writers(self, frame: int, frame_images: Dict[str, Any]) -> None:
        """Push raw BGRA bytes to each camera's background writer (non-blocking)."""
        for spec in self._camera_specs:
            name = spec["name"]
            writer = self.writers.get(name)
            if writer is None:
                continue
            image = frame_images.get(name)
            if image is None:
                continue
            try:
                raw = bytes(image.raw_data)
            except Exception:
                continue
            writer.feed(frame, raw)

    # ------------------------------------------------------------------
    #  metadata
    # ------------------------------------------------------------------

    def _write_metadata(self) -> None:
        if self._current_run_dir is None:
            return
        try:
            map_name = str(getattr(self.world.get_map(), "name", ""))
        except Exception:
            map_name = ""
        try:
            client_version = str(getattr(self.carla, "__version__", ""))
        except Exception:
            client_version = ""
        fps = 1.0 / max(0.01, float(getattr(self.config, "fixed_delta_seconds", 0.05)))
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
            "output_dir": str(self._current_run_dir),
            "cameras": self._camera_specs,
            "fps": round(float(fps), 2),
            "fixed_delta_seconds": float(getattr(self.config, "fixed_delta_seconds", 0.0)),
            "sync_mode": bool(getattr(self.config, "sync_mode", True)),
            "format": "mp4 (ffmpeg) or png fallback",
        }
        meta_path = self._current_run_dir / "metadata.json"
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
