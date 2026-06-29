#!/usr/bin/env python3
"""Keep listening on COM9. User presses reset button on STM32 board."""
import serial
import time
import sys

PORT = "COM9"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)
print(f"[Listening on {PORT}... Press RESET button on the STM32 board NOW]")
print(f"[Waiting for data for 10 seconds...]")
print()

deadline = time.time() + 15.0
while time.time() < deadline:
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        sys.stdout.buffer.write(data)
        sys.stdout.buffer.flush()
    time.sleep(0.05)

if ser.in_waiting == 0:
    # Send a newline to maybe trigger response
    print("\n[Sending newline to nudge STM32...]")
    ser.write(b"\r\n")
    time.sleep(1.0)
    while ser.in_waiting > 0:
        sys.stdout.buffer.write(ser.read(ser.in_waiting))
        sys.stdout.buffer.flush()

    # Send STATUS
    print("\n[Sending STATUS...]")
    ser.write(b"STATUS\r\n")
    time.sleep(1.0)
    while ser.in_waiting > 0:
        sys.stdout.buffer.write(ser.read(ser.in_waiting))
        sys.stdout.buffer.flush()

ser.close()
print("\n[Done listening]")
