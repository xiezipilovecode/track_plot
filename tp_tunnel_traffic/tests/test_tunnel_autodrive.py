from __future__ import annotations

"""隧道自动驾驶流程测试入口（使用反转后的三车道起点）。

用途：
- 复用 tp_tunnel_traffic.main 中的三车道提取、车辆生成、自动驾驶与视角跟随逻辑
- 便于在 CARLA 中直接跑一个可验证的隧道自动驾驶测试
 - 当前以反转后的三车道起点作为车辆起点与视角对准目标

运行：
    python -m tp_tunnel_traffic.tests.test_tunnel_autodrive
"""

from pathlib import Path
import sys
import importlib

if __package__ is None or __package__ == "":
    sys.path.append(str(Path(__file__).resolve().parents[2]))

run_tunnel_autodrive = importlib.import_module("tp_tunnel_traffic.main").main


def test_tunnel_autodrive_entry() -> None:
    """保存当前隧道自动驾驶功能的测试入口。

    说明：这个测试本身是一个运行入口，不做断言式单元测试，
    直接执行主流程，用于人工在 CARLA 中验证车辆是否能在隧道中自动驾驶。
    """

    run_tunnel_autodrive()


if __name__ == "__main__":
    test_tunnel_autodrive_entry()
