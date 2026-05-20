import asyncio
import websockets
import json
import time
import cv2

class RobotClient:
    def __init__(self):
        # Hotspot server target IP configuration
        self.laptop_ip = "10.239.162.133"
        self.uri = f"ws://{self.laptop_ip}:8000/ws/robot"
        self.video_uri = f"ws://{self.laptop_ip}:8000/ws/video"
        
        # Shared state
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_command_time = 0.0
        self.connected = False
    
    async def video_stream_loop(self):
        """Captures hardware video frames, compresses them, and streams via WebSockets."""
        while True:
            # Only stream video if the primary websocket command pipeline is active
            if not self.connected:
                await asyncio.sleep(1)
                continue
                
            try:
                print("[VIDEO] Connecting to video stream channel...")
                async with websockets.connect(self.video_uri) as ws:
                    print("[VIDEO] Stream connected successfully!")
                    
                    camera = cv2.VideoCapture(0)
                    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                    
                    while self.connected:
                        success, frame = camera.read()
                        if not success:
                            await asyncio.sleep(0.03)
                            continue
                        else:
                            # Compress frame to JPEG format
                            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                            
                            # Ship raw binary frame tokens directly up to your laptop
                            await ws.send(buffer.tobytes())
                            
                            # Cap at 25 FPS
                            await asyncio.sleep(0.04)
                        
                    camera.release()
                    print("[VIDEO] Camera resource released cleanly.")
                    
            except Exception as e:
                print(f"[VIDEO ERROR] Stream disconnected: {e}. Retrying in 2 seconds...")
                await asyncio.sleep(2)

    def reset_commands_on_disconnect(self):
        """Locally set the last command to zero upon disconnect (motor safety)."""
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_command_time = time.time()
        print("[DISCONNECT] Set local command states to 0.0 (motors stopped).")

    async def telemetry_loop(self, websocket):
        """Sends telemetry JSON back to the base station every 1 second."""
        while self.connected:
            telemetry = {
                "type": "telemetry",
                "timestamp": int(time.time() * 1000),
                "battery_capacity": 60,    
                "power_consumption": 12.4, 
                "imu_angle": 0.2,        
                "last_linear": self.last_linear,
                "last_angular": self.last_angular
            }
            
            try:
                await websocket.send(json.dumps(telemetry))
                print(f"[TELEMETRY] Sent heartbeat payload: {telemetry}")
            except websockets.exceptions.ConnectionClosed:
                break
                
            await asyncio.sleep(1)

    async def receive_loop(self, websocket):
        """Listens for incoming joystick commands formatted as '<linear,angular>'."""
        try:
            async for message in websocket:
                if message.startswith('<') and message.endswith('>'):
                    inner_content = message[1:-1]
                    try:
                        lin_str, ang_str = inner_content.split(',')
                        self.last_linear = float(lin_str)
                        self.last_angular = float(ang_str)
                        self.last_command_time = time.time()
                        
                        print(f"[COMMAND RECV] Linear: {self.last_linear:.2f} | Angular: {self.last_angular:.2f}")
                        
                    except ValueError:
                        print(f"[ERROR] Failed to parse command floats: {message}")
                else:
                    print(f"[WARNING] Unrecognized message format received: {message}")
                    
        except websockets.exceptions.ConnectionClosed:
            print(f"[CLOSED] WebSocket connection closed in receive loop")

    async def run(self):
        """Main engine loop handling auto-reconnect and task scheduling."""
        backoff = 1
        max_backoff = 8

        # 🚀 CRITICAL FIX: Spin up the video routine as a concurrent background task!
        asyncio.create_task(self.video_stream_loop())

        while True:
            try:
                print(f"\n[CONNECTING] Attempting connection to {self.uri} ...")
                async with websockets.connect(self.uri) as websocket:
                    print("[CONNECTED] Successfully connected to Base Station.")
                    self.connected = True
                    backoff = 1  

                    # Fire off standard UI data tracking tasks concurrently
                    telemetry_task = asyncio.create_task(self.telemetry_loop(websocket))
                    receive_task = asyncio.create_task(self.receive_loop(websocket))

                    # Monitor connection state
                    done, pending = await asyncio.wait(
                        [telemetry_task, receive_task],
                        return_when=asyncio.FIRST_COMPLETED
                    )

                    self.connected = False
                    for task in pending:
                        task.cancel()

            except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError, OSError) as e:
                self.connected = False
                self.reset_commands_on_disconnect()
                
                print(f"[ERROR] Connection failed: {e}")
                print(f"[RETRY] Reconnecting in {backoff} seconds...")
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, max_backoff)

if __name__ == "__main__":
    client = RobotClient()
    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        print("\n[STOPPED] Client terminated manually via keyboard interrupt.")