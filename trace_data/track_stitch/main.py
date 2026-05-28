"""轨迹拼接 CLI 入口。

用法：
    python -m trace_data.track_stitch.main
    python -m trace_data.track_stitch.main --quick
    python -m trace_data.track_stitch.main --data-root <path> --output <path>
"""

from __future__ import annotations

import argparse
import logging
import sys

from .config import StitchingConfig
from .stitcher import Stitcher


def main() -> None:
    parser = argparse.ArgumentParser(
        description="六路摄像头隧道车辆轨迹拼接工具",
    )
    parser.add_argument(
        "--data-root",
        default=None,
        help="轨迹数据目录（默认使用 config 中的路径）",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="输出目录（默认使用 config 中的路径）",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="快速扫描模式（仅处理每摄像头前20个文件）",
    )
    parser.add_argument(
        "--max-files",
        type=int,
        default=0,
        help="最大处理文件数（0=全部）",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="详细日志输出",
    )
    args = parser.parse_args()

    # 日志设置
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    logger = logging.getLogger(__name__)

    # 配置
    config = StitchingConfig()
    if args.data_root:
        config.data_root = args.data_root
    if args.output:
        config.output_dir = args.output

    if args.quick:
        config.max_files_to_scan = 10  # 每摄像头 10 个文件
        logger.info("Quick scan mode: %d files per camera", config.max_files_to_scan)

    if args.max_files >= 0:
        config.max_files_to_scan = args.max_files
        if args.max_files == 0:
            logger.info("Full scan mode: all files")

    logger.info("Data root: %s", config.data_root)
    logger.info("Output dir: %s", config.output_dir)
    logger.info("Cameras: %s", ", ".join(config.cameras))

    # 执行拼接
    stitcher = Stitcher(config)
    stitched = stitcher.run()

    if not stitched:
        logger.warning("No stitched trajectories produced!")
        sys.exit(1)

    # 导出
    output_path = stitcher.export(stitched, config.output_dir)

    # 摘要
    print("\n" + "=" * 50)
    print("拼接完成！")
    print(f"  输出文件: {output_path}")
    print(f"  拼接轨迹数: {len(stitched)}")
    print(f"  总耗时: {stitcher.stats['elapsed_seconds']:.1f} 秒")
    print(f"  解析轨迹数: {stitcher.stats['total_tracks_parsed']}")
    print(f"  合并后轨迹数: {stitcher.stats['tracks_after_merge']}")
    print(f"  匹配对: {stitcher.stats['matched_pairs']}")
    print(f"  插值节点: {stitcher.stats['interpolated_nodes']}")

    # 质量分布
    if stitched:
        scores = [s.quality_score for s in stitched]
        print(f"  质量评分: min={min(scores):.3f} max={max(scores):.3f} "
              f"avg={sum(scores)/len(scores):.3f}")
        cam_counts = [s.camera_count for s in stitched]
        from collections import Counter
        dist = Counter(cam_counts)
        print(f"  摄像头覆盖分布: {dict(sorted(dist.items()))}")


if __name__ == "__main__":
    main()
