import asyncio
import json
import logging
import struct
import math
import time
import cv2
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder

# --- 配置 ---
SERVER_IP = "127.0.0.1"  # 本地测试
SERVER_PORT = 8765

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("HoloSimClient")

class HoloLensSimulator:
    def __init__(self):
        self.pc = RTCPeerConnection()
        self.ws = None
        self.control_channel = None
        self.running = True

        # 模拟头部运动的参数
        self.sim_time = 0.0

    async def run(self):
        uri = f"ws://{SERVER_IP}:{SERVER_PORT}"
        logger.info(f"Connecting to Signaling Server: {uri}")

        try:
            async with websockets.connect(uri) as websocket:
                self.ws = websocket
                logger.info("WebSocket Connected.")

                # 1. 监听远端轨道 (接收视频)
                @self.pc.on("track")
                def on_track(track):
                    logger.info(f"Track received: {track.kind}")
                    if track.kind == "video":
                        asyncio.ensure_future(self.consume_video(track))

                # 2. 创建 DataChannel (发送控制数据)
                # 注意：客户端通常主动创建 Channel
                self.control_channel = self.pc.createDataChannel("controls")
                
                @self.control_channel.on("open")
                def on_open():
                    logger.info("DataChannel 'controls' OPEN! Starting to send rotation.")
                    asyncio.ensure_future(self.send_fake_rotation())

                # 3. 添加 Transceiver (告诉服务器我想要视频，只收不发)
                self.pc.addTransceiver("video", direction="recvonly")

                # 4. 创建 Offer 并发送
                offer = await self.pc.createOffer()
                await self.pc.setLocalDescription(offer)
                
                await self.ws.send(json.dumps({
                    "msg": "sdp",
                    "type": "offer",
                    "sdp": self.pc.localDescription.sdp
                }))
                logger.info("Sent Offer.")

                # 5. 处理 ICE Candidate
                # 监听本地 ICE 并发送给服务器 (aiortc 自动处理收集，但需要手动发信令)
                # 注意：aiortc 目前不像 JS 那样有 onicecandidate 回调，
                # 简单的做法是在 createOffer 后，sdp 通常已经包含了 candidate (如果是非 Trickle ICE)
                # 或者依靠 aiortc 内部机制。对于 localhost/LAN，通常 SDP 里的 host candidate 就够了。

                # 6. 信令循环
                async for message in websocket:
                    data = json.loads(message)
                    msg_type = data.get("msg")

                    if msg_type == "sdp":
                        sdp_type = data.get("type")
                        sdp_str = data.get("sdp")
                        if sdp_type == "answer":
                            logger.info("Received Answer.")
                            await self.pc.setRemoteDescription(
                                RTCSessionDescription(sdp=sdp_str, type=sdp_type)
                            )
                    
                    elif msg_type == "ice":
                        # 处理服务器发来的 ICE (虽然服务器端是 Host 模式可能不需要，但写上比较稳)
                        candidate_str = data.get("candidate")
                        sdp_mid = data.get("sdpMid")
                        sdp_mline_index = data.get("sdpMlineIndex")
                        if candidate_str:
                             # 清理前缀
                            clean_cand = candidate_str.split(":", 1)[1] if "candidate:" in candidate_str else candidate_str
                            candidate = RTCIceCandidate(
                                component=1, 
                                foundation=clean_cand.split()[0], 
                                ip=clean_cand.split()[4], 
                                port=int(clean_cand.split()[5]), 
                                priority=int(clean_cand.split()[3]), 
                                protocol=clean_cand.split()[2], 
                                type=clean_cand.split()[7],
                                sdpMid=sdp_mid, 
                                sdpMLineIndex=sdp_mline_index
                            )
                            await self.pc.addIceCandidate(candidate)

        except Exception as e:
            logger.error(f"Error: {e}")
        finally:
            await self.cleanup()

    async def consume_video(self, track):
        """接收并显示视频流 (模拟 HoloLens 屏幕)"""
        logger.info("Starting video consumer...")
        cv2.namedWindow("HoloLens View", cv2.WINDOW_NORMAL)
        try:
            while self.running:
                try:
                    frame = await track.recv()
                    # 转换 aiortc VideoFrame 到 numpy array (RGB -> BGR for OpenCV)
                    img = frame.to_ndarray(format="bgr24")
                    
                    cv2.imshow("HoloLens View", img)
                    # 必须要有 waitKey 才能刷新窗口，1ms 延迟
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.running = False
                        break
                except Exception as e:
                    logger.warning(f"Video frame error: {e}")
                    break
        finally:
            cv2.destroyAllWindows()

    async def send_fake_rotation(self):
        """模拟头部转动并发送数据"""
        logger.info("Starting rotation simulator...")
        while self.running and self.control_channel.readyState == "open":
            # 模拟一个正弦波运动，像人在左右摇头
            self.sim_time += 0.05
            
            # 产生 -45 到 45 度的 Yaw (左右)
            fake_yaw = math.sin(self.sim_time) * 45.0
            # 产生 -10 到 10 度的 Pitch (上下)
            fake_pitch = math.cos(self.sim_time * 0.5) * 10.0

            # 打包数据: Little-Endian, 2个 float (8 bytes)
            # 对应 C# 的 BitConverter.GetBytes(float)
            packet = struct.pack('<ff', fake_yaw, fake_pitch)

            try:
                self.control_channel.send(packet)
                # 约 30Hz 发送频率
                await asyncio.sleep(0.033)
            except Exception as e:
                logger.error(f"Send failed: {e}")
                break

    async def cleanup(self):
        self.running = False
        await self.pc.close()
        logger.info("Simulation stopped.")

if __name__ == "__main__":
    client = HoloLensSimulator()
    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        pass