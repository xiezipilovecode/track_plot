"""回放入口（推荐）：直接从仓库根目录运行。

用法：
    python .\auto_control_main.py

注意：该脚本保持“延迟导入”，避免仅导入本模块时就强依赖 carla。
"""


def main() -> None:
    # 延迟导入，避免仅导入 auto_control_main 时强依赖 carla/numpy
    from tp_replay.main import main as impl

    impl()


if __name__ == "__main__":
    main()
