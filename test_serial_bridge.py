import asyncio
import json
import os

import serial
import websockets

SERVER_URL = os.environ.get("SERVER_URL", "ws://10.130.108.34:8000")
SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
BAUD_RATE = int(os.environ.get("BAUD_RATE", "115200"))


async def receive_commands(ws, ser):
    """Forward <linear,angular> commands from the server to the ESP over serial."""
    async for message in ws:
        if isinstance(message, str) and message.startswith('<'):
            print(f"[CMD → ESP] {message}")
            ser.write(message.encode())


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
                print(f"[TELEMETRY ← ESP] {payload}")
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
            async with websockets.connect(f"{SERVER_URL}/ws/robot") as ws:
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
