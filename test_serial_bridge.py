import asyncio
import json
import os

import serial
import websockets

SERVER_URL = os.environ.get("SERVER_URL", "ws://10.22.179.34:8001")
SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("BAUD_RATE", "115200"))

_DEAD_ZONE = 0.1
_FWD_MIN, _FWD_MAX = 2.0, 7.0
_BWD_MIN, _BWD_MAX = -4.0, -9.0
_MAX_TURN = 4.0


def map_velocity(y: float) -> float:
    if abs(y) < _DEAD_ZONE:
        return 0.0
    if y > 0:
        t = (y - _DEAD_ZONE) / (1.0 - _DEAD_ZONE)
        return _FWD_MIN + t * (_FWD_MAX - _FWD_MIN)
    else:
        t = (abs(y) - _DEAD_ZONE) / (1.0 - _DEAD_ZONE)
        return _BWD_MIN + t * (_BWD_MAX - _BWD_MIN)


async def receive_commands(ws, ser):
    """Receive <linear,angular> from server, map and forward to ESP32."""
    async for message in ws:
        if not (isinstance(message, str) and message.startswith('<') and message.endswith('>')):
            continue
        try:
            lin_str, ang_str = message[1:-1].split(',', 1)
            vel = map_velocity(-float(lin_str))   # negated: joystick forward = correct direction
            angular = float(ang_str) * _MAX_TURN
        except ValueError:
            print(f"[WARN] Could not parse command: {message!r}")
            continue
        v_cmd = f"V:{vel:.2f}\n"
        a_cmd = f"A:{angular:.2f}\n"
        print(f"[CMD -> ESP] {message!r} -> {v_cmd!r} {a_cmd!r}")
        ser.write(v_cmd.encode())
        ser.write(a_cmd.encode())


async def read_telemetry(ws, ser):
    """Read newline-terminated JSON telemetry from the ESP and forward to the server."""
    buf = b""
    while True:
        data = await asyncio.to_thread(ser.read, 256)
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
                print(f"[TELEMETRY <- ESP] {payload}")
                await ws.send(json.dumps(payload))
            except json.JSONDecodeError:
                print(f"[WARN] Non-JSON from ESP: {line!r}")


async def run():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"[SERIAL] Opened {SERIAL_PORT} @ {BAUD_RATE} baud")

    backoff = 1
    while True:
        try:
            print(f"[CONNECTING] {SERVER_URL}/ws/robot ...")
            async with websockets.connect(
                f"{SERVER_URL}/ws/robot",
                ping_interval=20,
                ping_timeout=10,
            ) as ws:
                print("[CONNECTED] WebSocket established.")
                backoff = 1
                await asyncio.gather(
                    receive_commands(ws, ser),
                    read_telemetry(ws, ser),
                )
        except (websockets.exceptions.ConnectionClosedError, ConnectionRefusedError, OSError) as e:
            print(f"[ERROR] {e}. Retrying in {backoff}s...")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 8)


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\n[STOPPED] Bridge terminated.")
