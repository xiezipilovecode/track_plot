import asyncio
import logging
import struct
import json
import numpy as np
import websockets
import carla
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceCandidate
from aiortc.sdp import candidate_from_sdp
from av import VideoFrame
from fractions import Fraction
import time
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK
# --- 配置日志 ---
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("HolocarServer")

# --- 全局变量 ---
vehicle = None
camera = None
latest_frame_data = None 
latest_rotation = {'yaw': 0.0, 'pitch': 0.0}

# 分辨率配置 (保持和你之前一致或使用性能档)
RES_X = 896
RES_Y = 504

class CarlaVideoTrack(VideoStreamTrack):
    def __init__(self, camera_actor, fps=30):
        super().__init__()
        self.camera = camera_actor
        self.fps = fps
        self.frame_count = 0
        self.time_base = Fraction(1, 90000)
        self.pts_step = int(90000 / self.fps)
        self.start_time = None

    async def recv(self):
        # 初始化启动时间
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time

        # --- 1. 头部姿态更新 (保持不变) ---
        global latest_rotation, latest_frame_data
        if self.camera and self.camera.is_alive:
            try:
                self.camera.set_transform(carla.Transform(
                    carla.Location(x=0.4, y=-0.3, z=1.2),
                    carla.Rotation(pitch=-latest_rotation['pitch'], yaw=latest_rotation['yaw'], roll=0)
                ))
            except:
                pass

        # --- 2. 智能 FPS 控制与时间戳校准 (修改重点) ---
        
        # 理论上这一帧应该发送的时间点
        target_time = self.start_time + (self.frame_count * (1 / self.fps))
        wait_time = target_time - current_time
        
        # [新增] 防漂移机制：每 100 帧检查一次时间误差
        if self.frame_count % 100 == 0:
            # 如果误差超过 0.5秒 (说明系统变慢了或者时钟漂移了)
            if abs(wait_time) > 0.5:
                # 重置基准时间，对齐现在的墙钟时间
                # 这样可以消除累积误差，防止延迟无限扩大
                self.start_time = current_time - (self.frame_count * (1 / self.fps))
                wait_time = 0 # 本帧立即发送

        # 正常的帧率控制
        if wait_time > 0:
            await asyncio.sleep(wait_time)

        # --- 3. 图像处理 (保持不变) ---
        if latest_frame_data is not None:
            array = np.frombuffer(latest_frame_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (RES_Y, RES_X, 4))
            rgb_array = array[:, :, :3][:, :, ::-1]
            frame = VideoFrame.from_ndarray(rgb_array, format="rgb24")
        else:
            frame = VideoFrame.from_ndarray(np.full((RES_Y, RES_X, 3), 128, dtype=np.uint8), format="rgb24")

        # --- 4. 设置 PTS ---
        frame.pts = self.frame_count * self.pts_step
        frame.time_base = self.time_base
        self.frame_count += 1
        
        return frame
# --- 2. Carla 初始化 ---
def setup_carla():
    global vehicle, camera
    logger.info(">>> Connecting to Carla...")
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(30.0)

        # 直接生成隧道地图（跳过可能损坏的旧世界）
        logger.info(">>> Loading QingShiLing map ...")
        xodr_path = r"E:\carla\Unreal\CarlaUE4\Content\Carla\OpenDrive\QingShiLing.xodr"
        with open(xodr_path, "r", encoding="utf-8") as f:
            xodr = f.read()
        client.generate_opendrive_world(xodr)
        world = client.get_world()
        logger.info(">>> Map loaded.")

        # 清理
        client.apply_batch([carla.command.DestroyActor(x) for x in world.get_actors().filter("vehicle.*")])
        client.apply_batch([carla.command.DestroyActor(x) for x in world.get_actors().filter("sensor.*")])

        # 生成车辆 — 优先已有车辆，否则在隧道入口生成
        bp_lib = world.get_blueprint_library()
        existing = list(world.get_actors().filter("vehicle.*"))
        if existing:
            vehicle = existing[0]
            logger.info(f">>> Using existing vehicle: {vehicle.type_id}")
        else:
            bp = bp_lib.filter('model3')[0]
            # 隧道入口位置
            tunnel_tf = carla.Transform(
                carla.Location(x=6.0, y=-672.0, z=1.2),
                carla.Rotation(yaw=0.0)
            )
            vehicle = world.spawn_actor(bp, tunnel_tf)
            vehicle.set_autopilot(True)
            logger.info(f">>> Spawned vehicle at tunnel entry")

        # 生成摄像头
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(RES_X))
        camera_bp.set_attribute('image_size_y', str(RES_Y))
        camera_bp.set_attribute('fov', '90')
        camera_bp.set_attribute('sensor_tick', '0.0') # 0.0 = 跟随引擎帧率
        
        camera = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=0.4, y=-0.3, z=1.2)), attach_to=vehicle)

        # 回调只存数据
        def sensor_callback(image):
            global latest_frame_data
            latest_frame_data = image.raw_data

        camera.listen(sensor_callback)
        logger.info(">>> Carla Ready.")
        return world, vehicle, camera

    except Exception as e:
        logger.error(f"Carla Setup Failed: {e}")
        import os
        os._exit(1)

# --- 3. WebSocket & WebRTC (已恢复旧版逻辑) ---
async def connection_handler(websocket):
    logger.info(f"Client connected: {websocket.remote_address}")
    
    pc = RTCPeerConnection()
    pc.addTrack(CarlaVideoTrack(camera))

    @pc.on("datachannel")
    def on_datachannel(channel):
        logger.info(f"DataChannel OPEN: {channel.label}")
        
        @channel.on("message")
        def on_message(message):
            if isinstance(message, bytes) and len(message) == 8:
                try:
                    yaw, pitch = struct.unpack('<ff', message)
                    latest_rotation['yaw'] = yaw
                    latest_rotation['pitch'] = pitch
                except:
                    pass

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState in ["failed", "closed"]:
            await pc.close()

    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except:
                continue
            
            # --- 恢复：严格匹配 C# SignalingMsg 的逻辑 ---
            msg_type = data.get("msg") # "sdp" or "ice"

            if msg_type == "sdp":
                # 获取 offer/answer
                type_str = data.get("type") 
                sdp_str = data.get("sdp")
                
                logger.info(f"Received SDP: {type_str}")
                
                offer = RTCSessionDescription(sdp=sdp_str, type=type_str)
                await pc.setRemoteDescription(offer)
                
                # 如果收到 Offer，回复 Answer
                if offer.type == "offer":
                    answer = await pc.createAnswer()
                    await pc.setLocalDescription(answer)
                    
                    # 发送回 C#
                    await websocket.send(json.dumps({
                        "msg": "sdp",
                        "type": "answer",
                        "sdp": pc.localDescription.sdp
                    }))

            elif msg_type == "ice":
                candidate_str = data.get("candidate")
                sdpMid = data.get("sdpMid", "0")
                sdpMLineIndex = data.get("sdpMlineIndex", 0)
                
                if candidate_str:
                    # 处理 "candidate:" 前缀
                    if "candidate:" in candidate_str:
                        candidate_str = candidate_str.split(":", 1)[1]
                    
                    ice = candidate_from_sdp(candidate_str)
                    ice.sdpMid = sdpMid
                    ice.sdpMLineIndex = sdpMLineIndex
                    await pc.addIceCandidate(ice)

    except (ConnectionClosedError, ConnectionClosedOK):
        logger.info("Client disconnected normally (or without close frame).")
    except Exception as e:
        logger.error(f"WS Handling Error: {e}")
    finally:
        await pc.close()

# --- 4. 主程序入口 (必须保留 AsyncIO Run 修复) ---
async def main():
    setup_carla()
    logger.info("WebRTC Server listening on 0.0.0.0:8765")
    
    # 兼容新版 websockets 的写法
    async with websockets.serve(connection_handler, "0.0.0.0", 8765, ping_interval=None):
        await asyncio.Future()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Cleanup...")
        if camera: camera.destroy()
        if vehicle: vehicle.destroy()