# tests — 测试入口

## 隧道仿真

| 文件 | 用途 | 运行命令 |
|---|---|---|
| `test_tunnel_autodrive.py` | 隧道代理车流 + GUI 选车采集（主入口） | `python -m tp_tunnel_traffic.tests.test_tunnel_autodrive` |
| `test_tunnel_lanes.py` | 单独显示三车道车道线，验证采样是否正确 | `python -m tp_tunnel_traffic.tests.test_tunnel_lanes` |

## 数据集采集

| 文件 | 用途 | 运行命令 |
|---|---|---|
| `test_auto_collect.py` | 无 GUI 批量自动化采集，挂机跑数据 | `python -m tp_tunnel_traffic.tests.test_auto_collect` |
| `test_dataset_capture.py` | 数据集采集触发入口（`TT_COLLECT_ENABLE=1` 时采集多视角图像与标签） | `python -m tp_tunnel_traffic.tests.test_dataset_capture` |
| `test_dataset_vision_outputs.py` | 离线校验 COCO 标注与实例分割输出是否完整，可导出抽检样本 | `python -m tp_tunnel_traffic.tests.test_dataset_vision_outputs --run-dir <RUN_DIR>` |

## HoloLens 推流

| 文件 | 用途 | 运行命令 |
|---|---|---|
| `test_hololens_client.py` | WebRTC 测试客户端，连接推流服务端，OpenCV 显示画面 | `python -m tp_tunnel_traffic.tests.test_hololens_client` |

## 测试流程建议

1. **先验证车道**：`test_tunnel_lanes` → 确保三车道正常
2. **跑 GUI 采集**：`test_tunnel_autodrive` → 手动选车采集
3. **离线校验**：`test_dataset_vision_outputs` → 确认数据完整
4. **批量跑数据**：`test_auto_collect` → 挂机自动化采集
5. **HoloLens 推流**：`test_hololens_client` → 验证实时画面推流
