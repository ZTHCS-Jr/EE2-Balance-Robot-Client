import asyncio
import websockets
import json
import time
import cv2
from picamera2 import Picamera2
import os
import serial
import math
import sys
import socket

class RobotClient:
    def __init__(self):        
        # Base Station configuration
        self.laptop_ip = "10.144.216.133"
        self.uri = f"ws://{self.laptop_ip}:8000/ws/robot"
        self.video_uri = f"ws://{self.laptop_ip}:8000/ws/video"
        
        # UDP Configuration
        self.udp_port = 31415
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.connected = False
        
        # Serial Configs
        self.ESP_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
        self.ESP_BAUD = int(os.environ.get("BAUD_RATE", "115200"))
        self.LIDAR_PORT = '/dev/serial0'
        self.LIDAR_BAUD = 230400

        # Motor State
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_command_time = 0.0
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

    def parse_lidar_packet(self, packet):
        if len(packet) != 47 or packet[0] != 0x54 or packet[1] != 0x2C:
            return []
        
        startAngle = (packet[5] << 8 | packet[4]) / 100.0
        endAngle = (packet[43] << 8 | packet[42]) / 100.0

        step = (360 - (startAngle - endAngle)) / 11.0 if endAngle < startAngle else (endAngle - startAngle) / 11.0

        points = []
        for i in range(12):
            base = 6 + (i * 3) 
            distance = packet[base+1] << 8 | packet[base]
            intensity = packet[base+2]
            angle = startAngle + step * i
            if angle >= 360.0: angle -= 360.0
            points.append((math.radians(angle), distance, intensity))

        return points

    async def esp32_sensor_loop(self, ser):
        """Continuously reads IMU/motor and sends via udp, independent of websockets."""
        while True:
            if ser.in_waiting > 0:
                try:
                    raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if raw_line.startswith("IMU:"):
                        latest_gyro_x = float(raw_line.split(":")[1])
                        payload = { "imu": { "gyro_x": latest_gyro_x } }
                        self.udp_sock.sendto(json.dumps(payload).encode("utf-8"), (self.laptop_ip, self.udp_port))
                        
                    elif raw_line.startswith("MOTOR:"):
                        parts = raw_line.split(":")[1].split(",")
                        left_steps, right_steps = int(parts[0]), int(parts[1])
                        payload = { "odom": { "left_steps": left_steps, "right_steps": right_steps } }
                        self.udp_sock.sendto(json.dumps(payload).encode("utf-8"), (self.laptop_ip, self.udp_port))
                except (ValueError, IndexError):
                    pass 
            
            # Yield to event loop to prevent blocking video/commands
            await asyncio.sleep(0.005)

    async def lidar_sensor_loop(self, lidar_ser):
        """Continuously reads LIDAR and sends via udp, independent of websockets."""
        scanData = {}
        packetsRead = 0

        while True:
            if lidar_ser.in_waiting >= 47:
                first_byte = lidar_ser.read(1)
                if first_byte and first_byte[0] == 0x54:
                    remaining = lidar_ser.read(46)
                    if len(remaining) == 46:
                        packet = bytes([0x54]) + remaining
                        points = self.parse_lidar_packet(packet)
                        
                        for angle, radius, intensity in points:
                            if 0 < radius < 2000 and intensity >= 30:
                                deg = int(math.degrees(angle))
                                scanData[deg] = (angle, radius)
                        packetsRead += 1

                        if packetsRead >= 150:
                            payload = { "lidar": scanData }
                            self.udp_sock.sendto(json.dumps(payload).encode("utf-8"), (self.laptop_ip, self.udp_port))
                            scanData = {}
                            packetsRead = 0
            
            await asyncio.sleep(0.01)

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
        """Receives velocity commands via WebSocket and writes to ESP32."""
        async for message in websocket:
            try:
                linear_v_str, angular_v_str = message[1:-1].split(',', 1)
                vel = self.map_velocity(-float(linear_v_str))
                angular = float(angular_v_str) * self._MAX_TURN
                
                v_cmd = f"V:{vel:.2f}\n"
                a_cmd = f"A:{angular:.2f}\n"
                
                ser.write(v_cmd.encode())
                ser.write(a_cmd.encode())
            except ValueError:
                print(f"[WARN] Could not parse command: {message!r}")

    async def run(self):
        backoff = 1
        max_backoff = 8
        try:
            esp_ser = serial.Serial(self.ESP_PORT, self.ESP_BAUD, timeout=0.1)
            print(f"[SERIAL] Opened esp32 {self.ESP_PORT}")
        except Exception as e:
            print(f"[ERROR] Failed to open serial: {e}")
            sys.exit(1)

        try:
            lidar_ser = serial.Serial(self.LIDAR_PORT, self.LIDAR_BAUD, timeout=0.1)
            print(f"[SERIAL] Opened LiDAR {self.LIDAR_PORT}")
        except Exception as e:
            print(f"[ERROR] Failed to open LiDAR serial: {e}")
            sys.exit(1)

        asyncio.create_task(self.video_stream_loop())
        asyncio.create_task(self.esp32_sensor_loop(esp_ser))
        asyncio.create_task(self.lidar_sensor_loop(lidar_ser))

        while True:
            try:
                print(f"\n[CONNECTING] Attempting connection to {self.uri} ...")
                async with websockets.connect(self.uri) as websocket:
                    print("[CONNECTED] Successfully connected to Base Station.")
                    self.connected = True
                    backoff = 1  

                    telemetry_task = asyncio.create_task(self.telemetry_loop(websocket))
                    receive_task = asyncio.create_task(self.receive_and_send(websocket, esp_ser))

                    done, pending = await asyncio.wait(
                        [telemetry_task, receive_task],
                        return_when=asyncio.FIRST_EXCEPTION
                    )

                    self.connected = False
                    for task in pending:
                        task.cancel()
                    if pending:
                        await asyncio.gather(*pending, return_exceptions=True)

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