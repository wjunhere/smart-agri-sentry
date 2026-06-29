#!/usr/bin/env python3
"""Send commands to STM32 via COM9 and read responses."""
import serial
import sys
import time

PORT = "COM9"
BAUD = 115200
TIMEOUT = 2.0

def send_cmd(ser, cmd):
    """Send a command and print all responses within timeout window."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    time.sleep(0.3)
    lines = []
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                lines.append(line)
        except Exception:
            break
    return lines

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        print(f"[OK] Opened {PORT} at {BAUD} baud")
    except serial.SerialException as e:
        print(f"[FAIL] Cannot open {PORT}: {e}")
        print("Hint: close comNG or any other app using COM9 first.")
        sys.exit(1)

    # Read any initial banner output
    time.sleep(1.0)
    while ser.in_waiting > 0:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"  STM32> {line}")

    # Send commands
    for cmd in sys.argv[1:] if len(sys.argv) > 1 else ["STATUS"]:
        print(f"\n>>> SEND: {cmd}")
        responses = send_cmd(ser, cmd)
        for r in responses:
            print(f"  STM32> {r}")
        if not responses:
            print("  (no response)")

    ser.close()
    print("\n[Done]")

if __name__ == "__main__":
    main()
