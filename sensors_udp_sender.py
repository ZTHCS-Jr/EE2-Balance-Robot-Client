import serial
import math
import time
import socket
import json
import sys

SERIAL_PORT = '/dev/serial0'
USB_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 230400
IMU_BAUD = 115200
IP = "10.245.27.34"
UDP_PORT = 31415

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"connected to {IP}:{UDP_PORT}")

try:
    lidar_ser = serial.Serial(SERIAL_PORT, LIDAR_BAUD, timeout=1)
    print(f"connected to {lidar_ser.name}")
except Exception as e:
    print(f"failed to connect: {e}")
    sys.exit(1) 

try:
    imu_ser = serial.Serial(USB_PORT, IMU_BAUD, timeout=0.01)
    print(f"connected to {imu_ser.name}")
except Exception as e:
    print(f"failed to connect: {e}")
    sys.exit(1)

def parse_packet(packet):
    # check packet size, header value, VerLen value
    if len(packet)!=47 or packet[0]!=0x54 or packet[1]!=0x2C:
        return None, []

    startAngle = (packet[5] << 8 | packet[4])/100.0
    endAngle = (packet[43] << 8 | packet[42])/100.0

    if endAngle < startAngle:
        step = (360-(startAngle-endAngle))/11.0
    else:
        step = (endAngle-startAngle)/11.0

    points = []
    for i in range(12):
        base = 6+(i*3) # data starts at index 6
        distance = packet[base+1] << 8 | packet[base]
        intensity = packet[base+2]
        angle = startAngle + step*i
        if angle >= 360.0: angle-=360.0
        points.append((math.radians(angle), distance, intensity))

    return startAngle, points

time.sleep(1)

try:
    scanData = {}
    prev_start_angle = None

    # Integrated absolute yaw (rad). gyro_x_cal from the ESP is a calibrated,
    # gravity-referenced yaw RATE in rad/s, so we just integrate it here where
    # the sample timing is real (the PC's WiFi arrival jitter is unreliable).
    yaw = 0.0
    last_imu_t = None

    while True:
        # Drain all buffered IMU lines, keep only the freshest rate.
        latest_rate = None
        while imu_ser.in_waiting > 0:
            raw_line = imu_ser.readline().decode('utf-8', errors='ignore').strip()
            if raw_line.startswith("IMU:"):
                try:
                    latest_rate = float(raw_line.split(":")[1])
                except ValueError:
                    pass

        # Integrate once with the real elapsed time (robust to serial buffering).
        if latest_rate is not None:
            now = time.monotonic()
            if last_imu_t is not None:
                dt = now - last_imu_t
                if 0.0 < dt < 0.5:
                    yaw += latest_rate * dt
                    yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # wrap to (-pi, pi]
            last_imu_t = now
            imu_payload = { "imu": { "yaw": yaw, "rate": latest_rate } }
            sock.sendto(json.dumps(imu_payload).encode("utf-8"), (IP, UDP_PORT))

        # Lidar: accumulate one full revolution, then flush (no motion smear).
        if lidar_ser.in_waiting >= 47 and lidar_ser.read(1)[0] == 0x54:
            remaining = lidar_ser.read(46)
            if len(remaining) == 46:
                packet = bytes([0x54]) + remaining
                start_angle, points = parse_packet(packet)

                if start_angle is not None:
                    # Angle wrapped back past 0 -> a revolution completed, flush it.
                    if prev_start_angle is not None and start_angle < prev_start_angle - 180.0:
                        lidar_payload = { "lidar": scanData, "t": time.monotonic() }
                        sock.sendto(json.dumps(lidar_payload).encode("utf-8"), (IP, UDP_PORT))
                        scanData = {}
                    prev_start_angle = start_angle

                    for angle, radius, intensity in points:
                        if 0 < radius < 8000 and intensity >= 30:
                            deg = int(math.degrees(angle))
                            scanData[deg] = (angle, radius)

except KeyboardInterrupt:
    print("\nExit and closed port")
    lidar_ser.close()
    imu_ser.close()