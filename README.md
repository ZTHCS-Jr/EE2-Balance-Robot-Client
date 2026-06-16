# EE2-Balance-Robot-Client

Code that runs on the robot's **Raspberry Pi**. Its used to bridge the
ESP32 balance controller (serial), the LiDAR (serial), and the Pi camera to the
**laptop base station** (`main.py`) and the **ROS 2 mapping / face-seeker** stack.


## I/O map

| Link | Port / device | Direction | Carries |
|---|---|---|---|
| ESP32 serial | `/dev/ttyUSB0` @ 115200 | in + out | reads `ODOM:` / `POWER:`; writes `V:` / `A:` |
| LiDAR serial | `/dev/serial0` @ 230400 | in | raw LD19 packets |
| Pi camera | Picamera2 | in | video frames |
| Base station WS | `ws://<laptop>:8000/ws/robot` | in + out | telemetry out; joystick `<linear,angular>` in |
| Base station WS | `ws://<laptop>:8000/ws/video` | out | grayscale JPEG frames |
| Sensor UDP | `<laptop>:31415` | out | `odom` / `power` / `lidar` JSON |
| Command UDP | `:31416` (listen) | in | autonomous `{"v","w"}` + `{"audio"}` triggers |

## `client.py` - main Pi client

The full runtime. Opens both serial ports, then runs these asyncio loops concurrently:

* **`video_stream_loop`** - captures camera frames, converts to grayscale JPEG
  (1000x1000, quality 50) off the event loop, streams to `/ws/video`.
* **`esp32_sensor_loop`** - reads the ESP32's consolidated lines and forwards them to the
  laptop over UDP 31415:
  * `ODOM:<t_ms>,<left_steps>,<right_steps>,<yaw_rate>[,<pitch>]`
  * `POWER:<volts>,<amps>,<watts>,<soc>`
* **`lidar_sensor_loop`** - drains the LD19 buffer and sends one UDP scan per ~revolution
  (~40 packets) so `/scan` refreshes at the lidar's ~10 Hz.
* **`receive_and_send`** - manual joystick `<linear,angular>` from `/ws/robot` → ESP32
  `V:`/`A:` (linear mapped through a dead-zone + min/max speed curve).
* **`cmd_listener_loop`** - autonomous `{"v","w"}` velocity commands over UDP 31416 →
  ESP32 `V:`/`A:`, with a 0.5 s **deadman** (sends `V:0`/`A:0` if commands stop). Also
  handles `{"audio":"registered"|"unregistered"}` by playing the matching `.wav`.
* **`telemetry_loop`** - pushes a telemetry JSON heartbeat to `/ws/robot` every 1 s.

The WebSocket connection auto-reconnects and zeroes the motors
on disconnect.

### Configuration

| Name | Set via | Default |
|---|---|---|
| `laptop_ip` | edit in file | `Enter Yourself` |
| `SERIAL_PORT` | env var | `/dev/ttyUSB0` |
| `BAUD_RATE` | env var | `115200` |
| `CMD_PORT` | env var | `31416` |
| `LIDAR_PORT` / `LIDAR_BAUD` | edit in file | `/dev/serial0` / `230400` |


### Run

```bash
python client.py
```

Requires the camera, ESP32 (`/dev/ttyUSB0`) and LiDAR (`/dev/serial0`) connected, and the
base station reachable at `laptop_ip`. Audio greetings need SoX (`play`) and the
`registered.wav` / `not_registered.wav` files in the repo.


## Setup

On Raspberry Pi OS, enable the serial port used by the LiDAR (`/dev/serial0`) and install:

```bash
pip install pyserial websockets opencv-python
sudo apt install sox            # for audio 
# picamera2 ships with Raspberry Pi OS
```

`lidar_test.py` and `test_serial_bridge.py` are standalone bring-up utilities for the
LiDAR and the ESP32 serial link.