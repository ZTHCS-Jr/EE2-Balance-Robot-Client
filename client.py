import asyncio
import websockets
import json
import time
import cv2
from picamera2 import Picamera2
import os
import serial


class RobotClient:
    def __init__(self):
        # Hotspot server target IP configuration
        self.laptop_ip = "10.245.27.34"
        self.uri = f"ws://{self.laptop_ip}:8000/ws/robot"
        self.video_uri = f"ws://{self.laptop_ip}:8000/ws/video"
        
        # Shared state
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_command_time = 0.0
        self.connected = False
        self.SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
        self.BAUD_RATE = int(os.environ.get("BAUD_RATE", "115200"))

        self._DEAD_ZONE = 0.1
        self._FWD_MIN, self._FWD_MAX = 2.0, 7.0
        self._BWD_MIN, self._BWD_MAX = -4.0, -9.0
        self._MAX_TURN = 4.0
        
    def map_velocity(self, y: float) -> float:
        if abs(y) < self._DEAD_ZONE:
            return 0.0
        if y > 0:
            t = (y - self._DEAD_ZONE) / (1.0 - self._DEAD_ZONE)
            return self._FWD_MIN + t * (self._FWD_MAX - self._FWD_MIN)
        else:
            t = (abs(y) - self._DEAD_ZONE) / (1.0 - self._DEAD_ZONE)
            return self._BWD_MIN + t * (self._BWD_MAX - self._BWD_MIN)
    
    async def video_stream_loop(self):
        """Captures hardware video frames, compresses them, and streams via WebSockets."""
        while True:
            if not self.connected:
                await asyncio.sleep(1)
                continue
            
            camera = None

            try:
                print("[VIDEO] Connecting to video stream channel...")
                async with websockets.connect(self.video_uri) as ws:
                    print("[VIDEO] Stream connected successfully!")

                    camera = Picamera2()
                    config = camera.create_video_configuration(
                        main={"size": (1000, 1000), "format": "RGB888"}
                    )
                    camera.configure(config)
                    camera.start()
                    await asyncio.sleep(1.0)  # sensor warm-up

                    while self.connected:
                        try:
                            frame = await asyncio.to_thread(camera.capture_array)
                        except Exception as e:
                            print(f"[VIDEO WARNING] Dropping corrupted hardware frame: {e}")
                            await asyncio.sleep(0.01)
                            continue

                        if frame is None or frame.size == 0:
                            await asyncio.sleep(0.02)
                            continue

                        # NoIR -> grayscale, then JPEG-encode and send
                        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        _, buffer = cv2.imencode('.jpg', gray_frame, [cv2.IMWRITE_JPEG_QUALITY, 50])

                        await ws.send(buffer.tobytes())
                        await asyncio.sleep(0)

                    
            except Exception as e:
                print(f"[VIDEO ERROR] Stream disconnected: {e}. Retrying in 2 seconds...")
                await asyncio.sleep(2)
            finally:
                if camera is not None:
                    camera.release()
                    print("[VIDEO] Camera resource released cleanly.")

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
            
    async def receive_and_send(self, websocket, ser):
        # Receive <linear,angular> from server, map to our required ranges and forward to ESP32
        async for message in websocket:
            try:
                linear_v_str, angular_v_str = message[1:-1].split(',', 1)
                vel = self.map_velocity(-float(linear_v_str))   # negative so that our direction is correct
                angular = float(angular_v_str) * self._MAX_TURN
            except ValueError:
                print(f"[WARN] Could not parse command: {message!r}")
                continue
            v_cmd = f"V:{vel:.2f}\n"
            a_cmd = f"A:{angular:.2f}\n"
            print(f"[CMD -> ESP] {message!r} -> {v_cmd!r} {a_cmd!r}")
            ser.write(v_cmd.encode())
            ser.write(a_cmd.encode())

    async def run(self):
        """Main engine loop handling auto-reconnect and task scheduling."""
        backoff = 1
        max_backoff = 8
        ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=0.1)
        print(f"[SERIAL] Opened {self.SERIAL_PORT} @ {self.BAUD_RATE} baud")

        asyncio.create_task(self.video_stream_loop())

        while True:
            try:
                print(f"\n[CONNECTING] Attempting connection to {self.uri} ...")
                async with websockets.connect(self.uri) as websocket:
                    print("[CONNECTED] Successfully connected to Base Station.")
                    self.connected = True
                    backoff = 1  

                    telemetry_task = asyncio.create_task(self.telemetry_loop(websocket))
                    receive_task = asyncio.create_task(self.receive_and_send(websocket, ser))

                    done, pending = await asyncio.wait(
                        [telemetry_task, receive_task],
                        return_when=asyncio.FIRST_EXCEPTION
                    )

                    self.connected = False
                    # Cancel whichever task is still running 
                    for task in pending:
                        task.cancel()
                    if pending:
                        await asyncio.gather(*pending, return_exceptions=True)

                    # Raise any exceptions
                    for task in done:
                        exc = task.exception()
                        if exc is not None:
                            raise exc

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