"""轨迹拼接模块。

将六路隧道摄像头的车辆轨迹片段拼接为完整跨摄像头轨迹。

用法：
    python -m trace_data.track_stitch.main --quick
"""

from .config import StitchingConfig
from .stitcher import Stitcher

__all__ = ["StitchingConfig", "Stitcher"]
