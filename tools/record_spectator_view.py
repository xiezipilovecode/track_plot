import json
import os
import time
from pathlib import Path

import msvcrt

from tp_replay.carla_compat import require_carla


_DEFAULT_OUT = str(Path(__file__).resolve().parent / "camera_view.json")
OUTPUT_PATH = os.getenv("TP_SPECTATOR_VIEW_JSON", _DEFAULT_OUT)

def capture_once(spectator):
    t = spectator.get_transform()
    data = {
        "location": {"x": t.location.x, "y": t.location.y, "z": t.location.z},
        "rotation": {"pitch": t.rotation.pitch, "yaw": t.rotation.yaw, "roll": t.rotation.roll}
    }
    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"Saved: {OUTPUT_PATH}")
    print(f"loc=({t.location.x:.3f},{t.location.y:.3f},{t.location.z:.3f}) rot=(pitch={t.rotation.pitch:.2f}, yaw={t.rotation.yaw:.2f}, roll={t.rotation.roll:.2f})")

def main():
    carla = require_carla()
    host = os.getenv("TP_CARLA_HOST", "localhost")
    port = int(os.getenv("TP_CARLA_PORT", "2000"))
    timeout_seconds = float(os.getenv("TP_CARLA_TIMEOUT_SECONDS", "10.0"))

    client = carla.Client(host, port)
    client.set_timeout(timeout_seconds)

    try:
        world = client.get_world()
    except Exception as e:
        print(
            f"Failed to connect to CARLA at {host}:{port} within {timeout_seconds:.3f}s: {e}"
        )
        return
    spectator = world.get_spectator()
    print("Press SPACE to save current spectator view; Q or ESC to quit.")
    while True:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key in (b" ",):
                capture_once(spectator)
            elif key in (b"q", b"Q", b"\x1b"):
                break
        time.sleep(0.05)

if __name__ == "__main__":
    main()
