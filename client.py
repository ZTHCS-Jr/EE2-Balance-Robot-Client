import asyncio
import websockets
import json
import time

class RobotClient:
    def __init__(self):
        self.uri = "ws://10.130.108.34:8000/ws/robot"
        
        # Shared state
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_command_time = 0.0
        self.connected = False

    def reset_commands_on_disconnect(self):
        """Locally set the last command to zero upon disconnect (motor safety)."""
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_command_time = time.time()
        print("[DISCONNECT] Set local command states to 0.0 (motors stopped).")

    async def telemetry_loop(self, websocket):
        """Sends telemetry JSON back to the base station every 1 second."""
        while self.connected:
            # TODO: Replace 'null' placeholders with actual sensor readings from I2C/SPI
            telemetry = {
                "type": "telemetry",
                "timestamp": int(time.time() * 1000),
                "battery_capacity": 60,    # Placeholder: 85.5% 
                "power_consumption": 12.4,   # Placeholder: 12.4 Watts
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
                # Parse incoming string like "<0.5,-1.2>"
                if message.startswith('<') and message.endswith('>'):
                    inner_content = message[1:-1]
                    try:
                        lin_str, ang_str = inner_content.split(',')
                        self.last_linear = float(lin_str)
                        self.last_angular = float(ang_str)
                        self.last_command_time = time.time()
                        
                        print(f"[COMMAND RECV] Linear: {self.last_linear:.2f} | Angular: {self.last_angular:.2f}")
                        # TODO: Forward self.last_linear and self.last_angular to hardware motor controllers (e.g., via PWM or UART)
                        
                    except ValueError:
                        print(f"[ERROR] Failed to parse command floats: {message}")
                else:
                    print(f"[WARNING] Unrecognized message format received: {message}")
                    
        except websockets.exceptions.ConnectionClosed as e:
            print(f"[CLOSED] WebSocket connection closed in receive loop: {e}")

    async def run(self):
        """Main loop handling auto-reconnect with exponential backoff."""
        backoff = 1
        max_backoff = 8

        while True:
            try:
                print(f"\n[CONNECTING] Attempting connection to {self.uri} ...")
                async with websockets.connect(self.uri) as websocket:
                    print("[CONNECTED] Successfully connected to Base Station.")
                    self.connected = True
                    backoff = 1  # Reset backoff on successful connection

                    # Run both sending and receiving concurrently
                    telemetry_task = asyncio.create_task(self.telemetry_loop(websocket))
                    receive_task = asyncio.create_task(self.receive_loop(websocket))

                    # Wait until connection drops
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
                
                # Exponential backoff up to max_backoff
                backoff = min(backoff * 2, max_backoff)

if __name__ == "__main__":
    client = RobotClient()
    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        print("\n[STOPPED] Client terminated manually via keyboard interrupt.")


"""
=========================================================
 SETUP & RUN INSTRUCTIONS FOR RASPBERRY PI
=========================================================

1. Install Python 3 and pip on your Raspberry Pi:
   sudo apt update
   sudo apt install python3 python3-pip python3-venv

2. Create and activate a Python virtual environment 
   (required for installing packages via pip on modern Raspberry Pi OS):
   python3 -m venv ~/robot_env
   source ~/robot_env/bin/activate

3. Install the required websockets library:
   pip install websockets

4. Run the client script:
   python client.py
=========================================================
"""