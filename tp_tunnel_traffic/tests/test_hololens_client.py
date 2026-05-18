from __future__ import annotations

"""WebRTC test client — matches HoloLensSimulator from hololens_websocket_server.py.

Usage:
    python -m tp_tunnel_traffic.tests.test_hololens_client --host 127.0.0.1 --port 8765
"""

import argparse
import asyncio
import json
import struct
import sys
import time
from pathlib import Path

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


async def main(host: str, port: int):
    import cv2
    import numpy as np
    import websockets
    from aiortc import RTCPeerConnection, RTCSessionDescription

    uri = f"ws://{host}:{port}"
    print(f"Connecting to {uri} ...")

    pc = RTCPeerConnection()
    # Create DataChannel BEFORE offer (critical — matches HoloLensSimulator)
    ctrl = pc.createDataChannel("controls")
    pc.addTransceiver("video", direction="recvonly")

    # Fake head rotation (same as HoloLensSimulator)
    @ctrl.on("open")
    def _on_open():
        async def send():
            t = 0.0
            while True:
                await asyncio.sleep(0.033)
                t += 0.05
                try:
                    ctrl.send(struct.pack("<ff", float(np.sin(t) * 45.0), float(np.cos(t * 0.5) * 10.0)))
                except Exception:
                    break
        asyncio.ensure_future(send())

    frames = 0
    start_time: float | None = None
    cv_ok = False
    try:
        cv2.namedWindow("HoloLens WebRTC", cv2.WINDOW_NORMAL)
        cv_ok = True
    except Exception:
        print("(opencv window not available)")

    @pc.on("track")
    def on_track(track):
        nonlocal frames, start_time
        print(f"Track received: kind={track.kind}")
        if track.kind != "video":
            return

        async def consume():
            nonlocal frames, start_time
            start_time = time.time()
            first_saved = False
            while True:
                try:
                    frame = await track.recv()
                    frames += 1
                    if frames == 1:
                        # Diagnostic: check pixel values and save first frame
                        img = frame.to_ndarray(format="bgr24")
                        mean_val = img.mean()
                        print(f"  [DIAG] first frame: shape={img.shape}, mean={mean_val:.1f}", flush=True)
                        if mean_val < 10:
                            print("  [DIAG] WARNING: frame is nearly BLACK (mean < 10)", flush=True)
                        elif 120 < mean_val < 136:
                            print("  [DIAG] WARNING: frame is nearly GRAY (mean ~128) — fallback frame?", flush=True)
                        else:
                            print(f"  [DIAG] frame looks OK (mean={mean_val:.1f})", flush=True)
                        import cv2
                        cv2.imwrite("_test_frame.png", img)
                        print("  [DIAG] saved _test_frame.png — open it to check visually", flush=True)
                        first_saved = True
                    if frames % 30 == 1:
                        e = time.time() - start_time
                        fps = frames / e if e > 0 else 0
                        print(f"  frame #{frames:>4d}  fps={fps:.1f}  {frame.width}x{frame.height}")
                    if cv_ok and frames > 1:
                        img2 = frame.to_ndarray(format="bgr24")
                        cv2.imshow("HoloLens WebRTC", img2)
                        cv2.waitKey(1)
                except Exception as ex:
                    print(f"Frame error: {ex}")
                    break

        asyncio.ensure_future(consume())

    try:
        async with websockets.connect(uri, ping_interval=None) as ws:
            print("WebSocket connected")
            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            await ws.send(json.dumps({"msg": "sdp", "type": "offer", "sdp": pc.localDescription.sdp}))
            print("Offer sent")

            async for msg in ws:
                d = json.loads(msg)
                if d.get("msg") == "sdp" and d.get("type") == "answer":
                    await pc.setRemoteDescription(RTCSessionDescription(sdp=d["sdp"], type="answer"))
                    print("Answer received — waiting for video...")
                    break

            while True:
                await asyncio.sleep(1)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if cv_ok:
            cv2.destroyAllWindows()

    e = time.time() - start_time if start_time else 0
    fps = frames / e if e > 0 else 0
    print(f"\nDone. {frames} frames in {e:.1f}s ({fps:.1f} fps)")
    return 0 if frames > 0 else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()
    raise SystemExit(asyncio.run(main(args.host, args.port)))
