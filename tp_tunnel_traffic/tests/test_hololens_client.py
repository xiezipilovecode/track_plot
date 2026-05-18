from __future__ import annotations

"""Test client for HoloLens WebSocket JPEG streaming.

Connects to HoloLensServer, receives JPEG video frames via WebSocket,
displays via OpenCV.

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

import numpy as np


async def main(host: str, port: int):
    import cv2
    import websockets

    uri = f"ws://{host}:{port}"
    print(f"Connecting to {uri} ...")
    start_time: float | None = None
    frames = 0

    cv_ok = False
    try:
        cv2.namedWindow("HoloLens WS Client", cv2.WINDOW_NORMAL)
        cv_ok = True
    except Exception:
        print("(opencv window not available)")

    try:
        async with websockets.connect(uri, ping_interval=None) as ws:
            print("Connected. Waiting for video ...")
            start_time = time.time()

            while True:
                msg = await ws.recv()

                if isinstance(msg, str):
                    # Text = SDP signaling (passed through, logged only)
                    try:
                        data = json.loads(msg)
                        k = data.get("msg", "?")
                        if k == "sdp":
                            print(f"  [SIG] {data.get('type','?')}")
                    except Exception:
                        pass
                    continue

                if isinstance(msg, bytes):
                    # Binary = JPEG frame (4B length + JPEG)
                    try:
                        dlen = struct.unpack(">I", msg[:4])[0]
                        jpeg = msg[4: 4 + dlen]
                    except Exception:
                        continue

                    arr = np.frombuffer(jpeg, dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is None:
                        continue

                    frames += 1
                    if frames % 30 == 1:
                        e = time.time() - start_time
                        fps = frames / e if e > 0 else 0
                        print(f"  frame #{frames:>4d}  fps={fps:5.1f}  size={len(jpeg):>5d}B")

                    if cv_ok:
                        cv2.imshow("HoloLens WS Client", img)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break

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
