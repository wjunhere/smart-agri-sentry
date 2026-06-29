#!/usr/bin/env python3
"""Scan all COM ports for STM32 boot banner."""
import serial
import time
import sys

BAUD = 115200
ports_to_try = [f"COM{i}" for i in range(1, 33)]

for port in ports_to_try:
    try:
        ser = serial.Serial(port, BAUD, timeout=0.3)
        # Read any data within 0.5s
        time.sleep(0.5)
        data = b""
        while ser.in_waiting > 0:
            data += ser.read(ser.in_waiting)
            time.sleep(0.1)
        if data:
            text = data.decode(errors="replace")[:200]
            print(f"[HIT] {port}: {repr(text)}")
        else:
            print(f"[---] {port}: no data")
        ser.close()
    except serial.SerialException as e:
        # Port doesn't exist or is busy
        err = str(e)
        if "could not open" in err.lower() or "access" in err.lower():
            print(f"[BUSY] {port}: in use by another app")
        else:
            pass  # Port doesn't exist, skip silently
    except Exception as e:
        print(f"[ERR] {port}: {e}")
