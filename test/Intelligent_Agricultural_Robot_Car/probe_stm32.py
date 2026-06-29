#!/usr/bin/env python3
"""Probe STM32 on COM9 — toggle DTR for reset, catch boot banner, test commands."""
import serial
import time

PORT = "COM9"
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    print(f"[OK] Opened {PORT}")

    # Toggle DTR to trigger reset (CH343 DTR → STM32 RESET on many boards)
    print("[...] Toggling DTR to reset STM32...")
    ser.dtr = False
    time.sleep(0.2)
    ser.dtr = True
    time.sleep(0.2)
    ser.dtr = False
    time.sleep(0.2)

    # Read any output for 3 seconds (boot banner + periodic status)
    print("[...] Reading for 3 seconds...")
    deadline = time.time() + 3.0
    total_bytes = 0
    while time.time() < deadline:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            total_bytes += len(data)
            print(data.decode(errors="replace"), end="", flush=True)
        time.sleep(0.05)

    if total_bytes == 0:
        print("\n[!] ZERO bytes received. Checking raw electrical activity...")
        # Check if any byte (even garbage) arrives at different baud rates
        for test_baud in [9600, 38400, 57600, 115200, 230400]:
            try:
                ser.baudrate = test_baud
                time.sleep(0.3)
                if ser.in_waiting > 0:
                    raw = ser.read(ser.in_waiting)
                    print(f"  [{test_baud}] {len(raw)} bytes: {raw.hex()}")
            except:
                pass
        ser.baudrate = BAUD

    # Now try sending a command
    print("\n[...] Sending 'STATUS'...")
    ser.reset_input_buffer()
    ser.write(b"STATUS\r\n")
    time.sleep(1.0)
    resp = b""
    while ser.in_waiting > 0:
        resp += ser.read(ser.in_waiting)
    if resp:
        print(resp.decode(errors="replace"))
    else:
        print("[!] No response to STATUS command")

    ser.close()
    print("[Done]")

except Exception as e:
    print(f"[FAIL] {e}")
