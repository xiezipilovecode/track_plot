"""tp_replay: CARLA 轨迹复现（回放）模块化实现。

说明：本包是从历史单文件 `replay_main.py` 拆分而来，便于维护与按职责分层。
仓库推荐入口仍为根目录 `auto_control_main.py`。
"""

# NOTE: Do not import heavy CARLA-dependent modules at package import time.
# Keep this package importable under `python -m compileall .` even if `carla`
# isn't available on the machine.


def main(*args, **kwargs):
    from .main import main as _main

    return _main(*args, **kwargs)


__all__ = ["main"]
