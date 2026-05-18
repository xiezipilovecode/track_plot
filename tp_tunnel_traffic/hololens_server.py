from __future__ import annotations

import asyncio
import json
import logging
import struct
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

# Ensure track is recognized by aiortc
try:
    from aiortc import MediaStreamTrack
except ImportError:
    MediaStreamTrack = object  # type: ignore[assignment,misc]

logger = logging.getLogger("HoloLensServer")


# ------------------------------------------------------------------
#  HoloLensVideoTrack  — 将 CARLA 相机帧封装为 WebRTC VideoTrack
# ------------------------------------------------------------------

class HoloLensVideoTrack(MediaStreamTrack):
    """aiortc VideoStreamTrack — pulls frames from a shared CARLA camera buffer.

    Frames are fed from the CARLA sensor callback (main thread) via
    :meth:`feed_frame` and consumed by aiortc's RTCRtpSender via
    :meth:`recv`.
    """

    kind = "video"

    def __init__(self, fps: int = 30):
        super().__init__()
        self._fps = max(1, fps)
        self._latest_bytes: bytes | None = None
        self._latest_lock = threading.Lock()
        self._frame_count = 0
        self._start_time: float | None = None
        self._pts_step = 90000 // self._fps
        self._width = 0
        self._height = 0
        self._fed_count = 0  # debug: frames received from CARLA
        self._sent_count = 0  # debug: frames returned by recv()
        self._sent_with_data = 0  # debug: recv calls that had real data

    @property
    def readyState(self) -> str:
        return "live"

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    def configure(self, width: int, height: int) -> None:
        self._width = width
        self._height = height

    def feed_frame(self, raw_data: bytes) -> None:
        """Called from CARLA sensor callback (main thread). Thread-safe."""
        with self._latest_lock:
            self._latest_bytes = bytes(raw_data)
        self._fed_count += 1

    async def recv(self):
        from av import VideoFrame

        # Rate-limit to ~30 fps so the encoder doesn't flood
        await asyncio.sleep(1.0 / max(1, self._fps))

        self._sent_count += 1
        with self._latest_lock:
            data = self._latest_bytes
        if self._sent_count % 100 == 1:
            has_data = data is not None and len(data) > 0
            if has_data:
                self._sent_with_data += 1
            print(f"[HoloLens] recv() #{self._sent_count}: has_data={has_data}, "
                  f"fed={self._fed_count}, sent_with_data={self._sent_with_data}")

        if data is None or len(data) == 0:
            arr = np.full((self._height or 504, self._width or 896, 3), 128, dtype=np.uint8)
            frame = VideoFrame.from_ndarray(arr, format="rgb24")
        else:
            h = self._height or 504
            w = self._width or 896
            try:
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 4))
            except ValueError:
                arr = np.full((h, w, 4), 128, dtype=np.uint8)
            rgb = arr[:, :, :3][:, :, ::-1]  # BGRA → RGB
            frame = VideoFrame.from_ndarray(np.ascontiguousarray(rgb), format="rgb24")

        self._frame_count += 1
        frame.pts = self._frame_count * self._pts_step
        frame.time_base = 1 / 90000
        return frame


# ------------------------------------------------------------------
#  HoloLensServer  — 管理 WebRTC 连接、信令、相机
# ------------------------------------------------------------------

class HoloLensServer:
    """WebRTC streaming server for HoloLens 2.

    - Shares the main process's CARLA client/world.
    - Manages a dedicated ego camera on the selected proxy vehicle.
    - Runs asyncio event loop in a background thread (non-blocking).
    """

    def __init__(self, carla, world, config) -> None:
        self.carla = carla
        self.world = world
        self.config = config
        self.port: int = int(getattr(config, "hololens_port", 8765))
        self.res_w: int = int(getattr(config, "hololens_res_w", 896))
        self.res_h: int = int(getattr(config, "hololens_res_h", 504))
        self.fps: int = int(getattr(config, "hololens_fps", 30))

        self._running = False
        self._vehicle = None
        self._camera_sensor = None
        self._track = HoloLensVideoTrack(self.fps)
        self._latest_rotation = {"yaw": 0.0, "pitch": 0.0}
        self._rotation_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

        # WebSocket JPEG streaming (scheme A: video via WS, controls via WebRTC)
        self._ws_clients: list = []
        self._latest_jpeg: bytes | None = None
        self._jpeg_lock = threading.Lock()
        self._jpeg_quality: int = 70

    # ------------------------------------------------------------------
    #  public API (called from main thread)
    # ------------------------------------------------------------------

    def start(self, vehicle=None) -> None:
        """Begin streaming. If no vehicle, uses a world-fixed spectator camera."""
        if self._running:
            return
        self._vehicle = vehicle
        self._running = True
        self._spawn_camera()
        self._thread = threading.Thread(target=self._thread_loop, daemon=True)
        self._thread.start()
        logger.info("HoloLensServer started on port %d", self.port)

    def stop(self) -> None:
        """Stop streaming and clean up."""
        self._running = False
        self._destroy_camera()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        logger.info("HoloLensServer stopped")

    def set_vehicle(self, vehicle) -> None:
        """Re-target the stream to a different proxy vehicle.

        If ``vehicle`` is None, the camera is re-spawned at its current
        world position (detached mode) rather than being destroyed.
        """
        if self._vehicle is vehicle:
            return
        # Remember the current camera's world transform before destroying
        last_tf = None
        if self._camera_sensor is not None:
            try:
                last_tf = self._camera_sensor.get_transform()
            except Exception:
                pass
        self._destroy_camera()
        self._vehicle = vehicle
        if self._running:
            if vehicle is None and last_tf is not None:
                self._spawn_camera_at(last_tf)
            elif vehicle is not None:
                self._spawn_camera()

    def _spawn_camera_at(self, world_transform) -> None:
        """Spawn a world-fixed camera at the given transform."""
        bp_lib = self.world.get_blueprint_library()
        camera_bp = bp_lib.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self.res_w))
        camera_bp.set_attribute("image_size_y", str(self.res_h))
        camera_bp.set_attribute("fov", "90")
        self._camera_sensor = self.world.spawn_actor(camera_bp, world_transform)
        self._track.configure(self.res_w, self.res_h)
        self._listen_camera()

    def feed_frame(self, raw_data: bytes) -> None:
        """Deliver latest camera frame to the WebRTC track."""
        self._track.feed_frame(raw_data)

    def update_rotation(self, yaw: float, pitch: float) -> None:
        """Update head rotation from HoloLens DataChannel."""
        with self._rotation_lock:
            self._latest_rotation["yaw"] = float(yaw)
            self._latest_rotation["pitch"] = float(pitch)

    # ------------------------------------------------------------------
    #  camera management (main thread, called from start/set_vehicle)
    # ------------------------------------------------------------------

    def _spawn_camera(self) -> None:
        bp_lib = self.world.get_blueprint_library()
        camera_bp = bp_lib.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self.res_w))
        camera_bp.set_attribute("image_size_y", str(self.res_h))
        camera_bp.set_attribute("fov", "90")

        if self._vehicle is not None and self._vehicle.is_alive:
            # Attach to selected proxy vehicle (driver's seat view)
            transform = self.carla.Transform(
                self.carla.Location(0.65, -0.18, 1.22),
                self.carla.Rotation(pitch=-7.5, yaw=0.0, roll=0.0),
            )
            self._camera_sensor = self.world.spawn_actor(
                camera_bp, transform, attach_to=self._vehicle
            )
        else:
            # No vehicle selected → use world spectator position
            spectator = self.world.get_spectator()
            sp_tf = spectator.get_transform()
            self._camera_sensor = self.world.spawn_actor(
                camera_bp, sp_tf
            )
        self._track.configure(self.res_w, self.res_h)
        self._listen_camera()

    def _listen_camera(self) -> None:
        """Attach the frame-feed listener to the current camera sensor."""
        rotation_lock = self._rotation_lock
        latest_rotation = self._latest_rotation
        camera_sensor_ref = self._camera_sensor
        track_ref = self._track
        carla_ref = self.carla
        attached = self._vehicle is not None and self._vehicle.is_alive
        fed = [0]  # mutable counter for closure
        jpeg_q = int(self._jpeg_quality)
        jpeg_lock = self._jpeg_lock

        def _on_image(image):
            # Update camera transform with latest rotation (attached mode only)
            if attached:
                try:
                    with rotation_lock:
                        yaw = float(latest_rotation.get("yaw", 0.0))
                        pitch = float(latest_rotation.get("pitch", 0.0))
                    camera_sensor_ref.set_transform(carla_ref.Transform(
                        carla_ref.Location(0.65, -0.18, 1.22),
                        carla_ref.Rotation(pitch=float(-pitch), yaw=float(yaw), roll=0.0),
                    ))
                except Exception:
                    pass
            # Feed raw bytes to track  (WebRTC, kept for future)
            track_ref.feed_frame(image.raw_data)
            # Encode JPEG for WebSocket streaming
            try:
                arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4))
                ok, jpeg = cv2.imencode(".jpg", arr[:, :, :3],
                                         [cv2.IMWRITE_JPEG_QUALITY, jpeg_q])
                if ok:
                    data = jpeg.tobytes()
                    header = struct.pack(">I", len(data))
                    with jpeg_lock:
                        self._latest_jpeg = header + data  # store in self
            except Exception:
                pass
            fed[0] += 1
            if fed[0] % 200 == 1:
                print(f"[HoloLens] camera callback OK (frame #{fed[0]})")

        self._camera_sensor.listen(_on_image)
        logger.info("Camera sensor listening (res=%dx%d)", self.res_w, self.res_h)

    def _destroy_camera(self) -> None:
        if self._camera_sensor is not None:
            try:
                self._camera_sensor.stop()
            except Exception:
                pass
            try:
                self._camera_sensor.destroy()
            except Exception:
                pass
            self._camera_sensor = None

    # ------------------------------------------------------------------
    #  WebRTC + WebSocket signalling (background thread)
    # ------------------------------------------------------------------

    def _thread_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except Exception:
            logger.exception("HoloLens server loop error")
        finally:
            self._loop.close()

    async def _serve(self) -> None:
        import websockets
        from aiortc import RTCPeerConnection, RTCSessionDescription
        from aiortc.sdp import candidate_from_sdp

        async def handler(websocket):
            # Register client for JPEG broadcasting
            self._ws_clients.append(websocket)

            # ---- WebRTC (controls via DataChannel, kept for compatibility) ----
            pc = RTCPeerConnection()
            pc.addTrack(self._track)

            @pc.on("datachannel")
            def _on_datachannel(channel):
                @channel.on("message")
                def _on_message(message):
                    if isinstance(message, bytes) and len(message) == 8:
                        try:
                            yaw, pitch_val = struct.unpack("<ff", message)
                            self.update_rotation(yaw, pitch_val)
                        except Exception:
                            pass

            @pc.on("connectionstatechange")
            async def _on_state():
                if pc.connectionState in ("failed", "closed"):
                    await pc.close()

            # ---- Run JPEG broadcaster + WebRTC signaling concurrently ----
            async def broadcast_jpeg():
                """Push latest JPEG to this client at ~30fps."""
                jpeg_lock = self._jpeg_lock
                while self._running:
                    with jpeg_lock:
                        data = self._latest_jpeg
                    if data is not None:
                        try:
                            await websocket.send(data)
                        except Exception:
                            break
                    await asyncio.sleep(0.033)  # ~30 fps

            broadcaster = asyncio.ensure_future(broadcast_jpeg())

            try:
                async for message in websocket:
                    try:
                        data = json.loads(message)
                    except Exception:
                        continue
                    msg_type = data.get("msg")
                    if msg_type == "sdp":
                        sdp_str = data.get("sdp", "")
                        sdp_type = data.get("type", "")
                        offer = RTCSessionDescription(sdp=sdp_str, type=sdp_type)
                        await pc.setRemoteDescription(offer)
                        if offer.type == "offer":
                            answer = await pc.createAnswer()
                            await pc.setLocalDescription(answer)
                            await websocket.send(json.dumps({
                                "msg": "sdp",
                                "type": "answer",
                                "sdp": pc.localDescription.sdp,
                            }))
                    elif msg_type == "ice":
                        candidate_str = data.get("candidate", "")
                        sdpMid = data.get("sdpMid", "0")
                        sdpMLineIndex = data.get("sdpMlineIndex", 0)
                        if candidate_str:
                            if "candidate:" in candidate_str:
                                candidate_str = candidate_str.split(":", 1)[1]
                            try:
                                ice = candidate_from_sdp(candidate_str)
                                ice.sdpMid = sdpMid
                                ice.sdpMLineIndex = sdpMLineIndex
                                await pc.addIceCandidate(ice)
                            except Exception:
                                pass
            except Exception:
                pass
            finally:
                broadcaster.cancel()
                try:
                    self._ws_clients.remove(websocket)
                except ValueError:
                    pass
                await pc.close()

        async with websockets.serve(handler, "0.0.0.0", self.port, ping_interval=None):
            while self._running:
                await asyncio.sleep(0.5)
