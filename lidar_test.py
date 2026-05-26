import serial
import math
import plotext as plt
import time

# --- CONFIGURATION ---
PORT = '/dev/serial0'  # Standard GPIO Serial port on Raspberry Pi
BAUD_RATE = 230400

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"Failed to connect to {PORT}.")
    print("Did you enable the Serial Port in raspi-config and reboot?")
    print(e)
    exit()

def parse_ld06_packet(packet):
    """Parses a single 47-byte LD06 protocol packet."""
    if len(packet) != 47 or packet[0] != 0x54 or packet[1] != 0x2C:
        return []
    
    start_angle = (packet[5] << 8 | packet[4]) / 100.0
    end_angle = (packet[43] << 8 | packet[42]) / 100.0
    
    step = (end_angle - start_angle) / 11.0
    if end_angle < start_angle:
        step = (end_angle + 360.0 - start_angle) / 11.0
        
    points = []
    for i in range(12):
        base = 6 + (i * 3)
        distance = packet[base+1] << 8 | packet[base]
        angle = start_angle + step * i
        if angle >= 360.0: angle -= 360.0
        points.append((math.radians(angle), distance))
        
    return points

print("Gathering Lidar data... Press Ctrl+C to stop.")
print("Make sure your terminal panel in VS Code is pulled up nice and tall!")
time.sleep(1)

try:
    while True:
        scan_data = {}
        
        # Read enough packets for a full 360 sweep (~150-200 packets)
        packets_read = 0
        while packets_read < 150:
            if ser.in_waiting >= 47 and ser.read(1)[0] == 0x54:
                rest = ser.read(46)
                if len(rest) == 46:
                    packet = bytes([0x54]) + rest
                    points = parse_ld06_packet(packet)
                    for angle, dist in points:
                        if dist > 0: # Ignore 0-distance readings
                            deg = int(math.degrees(angle))
                            scan_data[deg] = (angle, dist)
                    packets_read += 1

        # Convert polar (angle, distance) to cartesian (X, Y) for the terminal
        x_vals = []
        y_vals = []
        for angle, dist in scan_data.values():
            x = dist * math.sin(angle)
            y = dist * math.cos(angle)
            x_vals.append(x)
            y_vals.append(y)

        # Draw the terminal plot
        plt.clt() # Clear terminal
        plt.cld() # Clear data
        
        plt.scatter(x_vals, y_vals, marker="dot")
        plt.title("OKDO Lidar Live Terminal Plot")
        plt.plotsize(80, 40) # Width, Height in terminal characters
        
        # Lock the axis limits to 5 meters (5000mm) so the map doesn't scale wildly
        plt.xlim(-100, 100)
        plt.ylim(-100, 100)
        
        plt.show()
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting and closing port...")
    ser.close()