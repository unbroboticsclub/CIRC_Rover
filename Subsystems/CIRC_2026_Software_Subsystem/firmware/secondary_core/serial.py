# src/comms/serial.py
import serial, json, time

# Adjust port as needed (Linux: /dev/ttyACM0, Windows: COM3)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def send(cmd_dict: dict):
    """Send a JSON command to the Teensy over UART."""
    msg = json.dumps(cmd_dict) + "\n"
    ser.write(msg.encode())

def receive():
    """Read and parse a JSON message from the Teensy."""
    try:
        line = ser.readline().decode().strip()
        if not line:
            return None
        return json.loads(line)
    except json.JSONDecodeError:
        return None

def test_serial():
    """Quick round-trip test."""
    send({"cmd": "ping"})
    time.sleep(0.1)
    resp = receive()
    print("Received:", resp)

if __name__ == "__main__":
    test_serial()
