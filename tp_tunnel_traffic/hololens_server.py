from __future__ import annotations

import asyncio
import json
import logging
import struct
import threading
import time
from fractions import Fraction

import numpy as np

try:
    from aiortc import VideoStreamTrack
except ImportError:
    VideoStreamTrack = object

logger = logging.getLogger("HoloLensServer")

# Shared frame buffer
_HOLO_FRAME = None
_HOLO_ROTATION = {"yaw": 0.0, "pitch": 0.0}
_HOLO_ROTATION_LOCK = threading.Lock()


class HoloLensVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, width=896, height=504, fps=30):
        super().__init__()
        self._w, self._h = width, height
        self._fps = max(1, fps)
        self._fc = 0
        self._start: float | None = None
        self._pts_step = 90000 // self._fps

    async def recv(self):
        from av import VideoFrame

        now = time.time()
        if self._start is None:
            self._start = now
        target = self._start + self._fc / self._fps
        wait = target - now
        if self._fc % 100 == 0 and abs(wait) > 0.5:
            self._start = now - self._fc / self._fps
            wait = 0
        if wait > 0.001:
            await asyncio.sleep(wait)

        data = _HOLO_FRAME
        if data is not None:
            try:
                arr = np.frombuffer(data, dtype=np.uint8).reshape((self._h, self._w, 4))
                rgb = arr[:, :, :3][:, :, ::-1]
                frame = VideoFrame.from_ndarray(np.ascontiguousarray(rgb), format="rgb24")
            except Exception:
                frame = VideoFrame.from_ndarray(
                    np.full((self._h, self._w, 3), 128, dtype=np.uint8), format="rgb24")
        else:
            frame = VideoFrame.from_ndarray(
                np.full((self._h, self._w, 3), 128, dtype=np.uint8), format="rgb24")
        self._fc += 1
        frame.pts = self._fc * self._pts_step
        frame.time_base = Fraction(1, 90000)
        return frame


class HoloLensServer:
    """WebRTC streaming with a SINGLE world camera — no create/destroy on switch."""

    def __init__(self, carla, world, config) -> None:
        self.carla = carla
        self.world = world
        self.config = config
        self.port = int(getattr(config, "hololens_port", 8765))
        self.res_w = int(getattr(config, "hololens_res_w", 896))
        self.res_h = int(getattr(config, "hololens_res_h", 504))
        self.fps = int(getattr(config, "hololens_fps", 30))
        self._running = False
        self._vehicle = None  # current target vehicle (set by GUI)
        self._camera = None   # spawned ONCE, never destroyed
        self._thread: threading.Thread | None = None

    # ---- public ----
    def start(self, vehicle=None) -> None:
        if self._running:
            return
        # Wait for old thread to exit gracefully (non-blocking short timeout)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self._vehicle = vehicle
        self._running = True
        if self._camera is None:
            self._spawn_camera_once()
        self._thread = threading.Thread(target=self._bg_loop, daemon=True)
        self._thread.start()
        logger.info("HoloLensServer started (port %d)", self.port)

    def stop(self) -> None:
        global _HOLO_FRAME
        self._running = False
        _HOLO_FRAME = None
        # Don't destroy camera or block — return immediately

    def set_vehicle(self, vehicle) -> None:
        """Switch camera to follow a different vehicle.  No create/destroy."""
        if self._vehicle is vehicle:
            return
        self._vehicle = vehicle
        # That's it — the callback reads self._vehicle live

    def pre_tick(self) -> None:
        """Position camera BEFORE world.tick() — zero-lag."""
        v = self._vehicle
        cam = self._camera
        if v is None or cam is None or not v.is_alive:
            return
        t = getattr(self, "_tc", 0) + 1
        setattr(self, "_tc", t)
        try:
            vt = v.get_transform()
            with _HOLO_ROTATION_LOCK:
                y = float(_HOLO_ROTATION["yaw"])
                p = float(_HOLO_ROTATION["pitch"])
            fwd_m, right_m, up_m = self._driver_offset(v)
            fwd = vt.get_forward_vector()
            right = vt.get_right_vector()
            cam.set_transform(self.carla.Transform(
                self.carla.Location(
                    vt.location.x + fwd.x * fwd_m - right.x * abs(right_m),
                    vt.location.y + fwd.y * fwd_m - right.y * abs(right_m),
                    vt.location.z + up_m,
                ),
                self.carla.Rotation(
                    pitch=vt.rotation.pitch - p,
                    yaw=vt.rotation.yaw + y,
                    roll=0.0,
                ),
            ))
        except Exception:
            pass

    def destroy(self) -> None:
        self.stop()

    # ---- camera (created ONCE) ----
    # Driver seat offsets per vehicle type (meters: forward, right, up)
    _DRIVER_OFFSETS = {
        # Default (fallback)
        "default":               (0.65, -0.18, 1.22),
        # Sedans / compact cars
        "audi.etron":            (0.65, -0.18, 1.22),
        "audi.a2":               (0.60, -0.16, 1.18),
        "bmw":                   (0.65, -0.18, 1.20),
        "citroen":               (0.60, -0.16, 1.18),
        "ford.crown":            (0.65, -0.18, 1.25),
        "ford.mustang":          (0.65, -0.20, 1.15),
        "lincoln.mkz":           (0.68, -0.18, 1.22),
        "mercedes.coupe":        (0.65, -0.20, 1.15),
        "mercedes.sprinter":     (0.70, -0.22, 1.65),
        "mini.cooper":           (0.55, -0.16, 1.15),
        "nissan.micra":          (0.58, -0.16, 1.18),
        "nissan.patrol":         (0.70, -0.20, 1.50),
        "seat.leon":             (0.62, -0.17, 1.20),
        "tesla.model3":          (0.65, -0.18, 1.20),
        "toyota.prius":          (0.62, -0.17, 1.22),
        # SUVs
        "jeep":                  (0.68, -0.20, 1.45),
        "landrover":             (0.70, -0.20, 1.50),
        "range":                 (0.70, -0.20, 1.50),
        # Trucks / vans
        "carlamotors":           (0.75, -0.22, 1.70),
        "volkswagen.t2":         (0.65, -0.20, 1.35),
    }

    def _driver_offset(self, vehicle) -> tuple:
        """Return (forward_m, right_m, up_m) for the given vehicle blueprint."""
        if vehicle is None:
            return (0.65, -0.18, 1.22)
        try:
            tid = str(vehicle.type_id).removeprefix("vehicle.").lower()
        except Exception:
            return self._DRIVER_OFFSETS["default"]
        # Match by prefix (e.g. "vehicle.audi.etron" → "audi.etron", then match "audi")
        for key in sorted(self._DRIVER_OFFSETS, key=lambda x: -len(x)):
            if key == "default":
                continue
            if tid.startswith(key):
                return self._DRIVER_OFFSETS[key]
        return self._DRIVER_OFFSETS["default"]
    def _spawn_camera_once(self) -> None:
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self.res_w))
        bp.set_attribute("image_size_y", str(self.res_h))
        bp.set_attribute("fov", "90")
        # World camera at spectator position — not attached to any vehicle
        self._camera = self.world.spawn_actor(bp, self.world.get_spectator().get_transform())
        self._listen()

    def _listen(self) -> None:
        global _HOLO_FRAME
        count = 0

        def _cb(image):
            nonlocal count
            global _HOLO_FRAME
            # Just store frame — camera position already set by pre_tick()
            _HOLO_FRAME = image.raw_data
            count += 1

        self._camera.listen(_cb)
        print(f"[HoloLens] camera listening ({self.res_w}x{self.res_h})")

    def _destroy_camera(self) -> None:
        global _HOLO_FRAME
        _HOLO_FRAME = None
        if self._camera is not None:
            try:
                self._camera.destroy()
            except Exception:
                pass
            self._camera = None

    # ---- asyncio background thread ----
    def _bg_loop(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._serve())
        except Exception:
            logger.exception("server loop error")
        finally:
            loop.close()

    async def _serve(self) -> None:
        import websockets
        from aiortc import RTCPeerConnection, RTCSessionDescription
        from aiortc.sdp import candidate_from_sdp

        rw, rh, fps = self.res_w, self.res_h, self.fps

        async def handler(websocket):
            pc = RTCPeerConnection()
            pc.addTrack(HoloLensVideoTrack(rw, rh, fps))

            @pc.on("datachannel")
            def _dc(ch):
                @ch.on("message")
                def _msg(m):
                    if isinstance(m, bytes) and len(m) == 8:
                        try:
                            y, p = struct.unpack("<ff", m)
                            with _HOLO_ROTATION_LOCK:
                                _HOLO_ROTATION["yaw"] = float(y)
                                _HOLO_ROTATION["pitch"] = float(p)
                        except Exception:
                            pass

            @pc.on("connectionstatechange")
            async def _cs():
                if pc.connectionState in ("failed", "closed"):
                    await pc.close()

            try:
                async for msg in websocket:
                    try:
                        d = json.loads(msg)
                    except Exception:
                        continue
                    mt = d.get("msg")
                    if mt == "sdp":
                        s = RTCSessionDescription(sdp=d.get("sdp", ""), type=d.get("type", ""))
                        await pc.setRemoteDescription(s)
                        if s.type == "offer":
                            a = await pc.createAnswer()
                            await pc.setLocalDescription(a)
                            await websocket.send(json.dumps({"msg": "sdp", "type": "answer", "sdp": pc.localDescription.sdp}))
                    elif mt == "ice":
                        cs = d.get("candidate", "")
                        if cs:
                            if "candidate:" in cs:
                                cs = cs.split(":", 1)[1]
                            try:
                                ice = candidate_from_sdp(cs)
                                ice.sdpMid = d.get("sdpMid", "0")
                                ice.sdpMLineIndex = d.get("sdpMlineIndex", 0)
                                await pc.addIceCandidate(ice)
                            except Exception:
                                pass
            except Exception:
                pass
            finally:
                await pc.close()

        async with websockets.serve(handler, "0.0.0.0", self.port, ping_interval=None,
                                       close_timeout=1):
            while self._running:
                await asyncio.sleep(0.5)
