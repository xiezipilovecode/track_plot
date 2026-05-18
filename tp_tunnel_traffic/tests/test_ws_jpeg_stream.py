from __future__ import annotations

"""Standalone WebSocket JPEG streaming test — server + client in one process.

No aiortc, no CARLA needed.  Verifies that raw WebSocket can deliver
video frames.  Once this passes, we know the approach works.

Usage:
    python E:\code\track_plot\tp_tunnel_traffic\tests\test_ws_jpeg_stream.py
"""

import asyncio
import json
import struct
import sys
import time
from pathlib import Path

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

PORT = 19878
FRAME_COUNT = 60  # 60 frames = ~2s at 30fps


# ---- Server: generates test pattern frames, sends via WebSocket ----
class JpegServer:
    """Minimal WebSocket server that pushes JPEG frames."""

    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self._count = 0
        self._clients: list = []

    async def start(self, host="127.0.0.1", port=PORT):
        import websockets

        async def handler(ws):
            self._clients.append(ws)
            print(f"[Server] client connected (total={len(self._clients)})")
            try:
                async for _ in ws:  # keep-alive
                    pass
            finally:
                self._clients.remove(ws)

        self._server = await websockets.serve(handler, host, port)
        print(f"[Server] listening on {host}:{port}")

    async def push_loop(self, total_frames=FRAME_COUNT):
        """Generate test pattern frames and broadcast to all clients."""
        import cv2

        for i in range(total_frames):
            await asyncio.sleep(1.0 / self.fps)

            # Generate color gradient test pattern
            arr = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            shift = (i * 5) % 255
            arr[:, :, 0] = shift  # R
            arr[:, :, 1] = (255 - shift)  # G
            arr[:, :, 2] = (shift + 85) % 255  # B

            ok, jpeg = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue

            # Protocol: 4-byte length (big-endian) + JPEG bytes
            data = jpeg.tobytes()
            header = struct.pack(">I", len(data))
            payload = header + data

            for ws in self._clients:
                try:
                    await ws.send(payload)
                except Exception:
                    pass

            self._count += 1
            if i % 30 == 0:
                print(f"[Server] frame #{i}/{total_frames} ({len(data)} bytes)")

        print(f"[Server] done. {self._count} frames sent")

    async def stop(self):
        if self._server:
            self._server.close()
            await self._server.wait_closed()


# ---- Client: receives JPEG frames, decodes, displays via OpenCV ----
async def run_client(host="127.0.0.1", port=PORT):
    import cv2
    import websockets

    frames_received = 0
    start_time = None
    cv_ok = False
    try:
        cv2.namedWindow("WS JPEG Client", cv2.WINDOW_NORMAL)
        cv_ok = True
    except Exception:
        pass

    uri = f"ws://{host}:{port}"
    print(f"[Client] connecting to {uri} ...")
    try:
        async with websockets.connect(uri, ping_interval=None) as ws:
            print("[Client] connected")
            start_time = time.time()
            while True:
                msg = await ws.recv()
                if isinstance(msg, bytes):
                    # Parse: 4-byte length + JPEG data
                    data_len = struct.unpack(">I", msg[:4])[0]
                    jpeg_data = msg[4 : 4 + data_len]

                    arr = np.frombuffer(jpeg_data, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        frames_received += 1
                        if frames_received % 30 == 1:
                            elapsed = time.time() - start_time
                            fps = frames_received / elapsed if elapsed > 0 else 0
                            print(f"[Client] frame #{frames_received} fps={fps:.1f}")
                        if cv_ok:
                            cv2.imshow("WS JPEG Client", img)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                break

                    if frames_received >= FRAME_COUNT:
                        break
    except Exception as e:
        print(f"[Client] error: {e}")
    finally:
        if cv_ok:
            cv2.destroyAllWindows()

    elapsed = time.time() - start_time if start_time else 0
    fps = frames_received / elapsed if elapsed > 0 else 0
    print(f"[Client] done. {frames_received} frames in {elapsed:.1f}s ({fps:.1f} fps)")
    return frames_received, fps


# ---- Main ----
async def main():
    server = JpegServer(640, 480, 30)
    await server.start()

    # Run server push + client concurrently
    client_task = asyncio.create_task(run_client())
    await asyncio.sleep(0.5)
    await server.push_loop(FRAME_COUNT)

    frames, fps = await client_task
    await server.stop()

    if frames >= FRAME_COUNT * 0.9:
        print(f"\nPASS: {frames} frames at {fps:.1f} fps")
        return 0
    else:
        print(f"\nFAIL: only {frames} frames")
        return 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
