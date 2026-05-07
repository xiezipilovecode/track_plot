from __future__ import annotations

"""隧道数据集采集测试入口（阶段 A）。

用途：
- 运行当前隧道自动驾驶流程
- 在开启 TT_COLLECT_ENABLE=1 时，采集多视角图像与标签

运行：
    $env:TT_COLLECT_ENABLE='1'
    python -m tp_tunnel_traffic.tests.test_dataset_capture
"""

from pathlib import Path
import importlib
import sys

if __package__ is None or __package__ == "":
    sys.path.append(str(Path(__file__).resolve().parents[2]))


def run_dataset_capture() -> None:
    run_main = importlib.import_module("tp_tunnel_traffic.main").main
    run_main()


if __name__ == "__main__":
    run_dataset_capture()
