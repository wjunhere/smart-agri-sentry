#!/usr/bin/env python3
"""Debug: scan for STM32 response on COM9 with different line endings and DTR toggle."""
import serial
import time

PORT = "COM9"
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    print(f"[OK] Opened {PORT}")

    # Toggle DTR to trigger reset (some boards use DTR for auto-reset)
    ser.dtr = False
    time.sleep(0.1)
    ser.dtr = True
    time.sleep(2.0)  # Wait for boot banner

    # Read all buffered data (boot banner)
    print("--- Reading boot banner ---")
    while ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        print(data.decode(errors="replace"), end="")

    # Send STATUS with \r\n
    print("\n--- Sending STATUS (CRLF) ---")
    ser.write(b"STATUS\r\n")
    time.sleep(0.5)
    while ser.in_waiting > 0:
        print(ser.read(ser.in_waiting).decode(errors="replace"), end="")

    # Send STATUS with just \r
    print("\n--- Sending STATUS (CR only) ---")
    ser.write(b"STATUS\r")
    time.sleep(0.5)
    while ser.in_waiting > 0:
        print(ser.read(ser.in_waiting).decode(errors="replace"), end="")

    # Try binary protocol format (USART2 style) in case CH343 is on USART2
    print("\n--- Trying USART2 protocol format (AA 01 04 ...) ---")
    # CMD=0x01 speed control: left=100, right=200, checksum=0xAA+0x01+0x04+100+0+200+0=...
    frame = bytes([0xAA, 0x01, 0x04, 100, 0, 200, 0])
    crc = sum(frame) & 0xFF
    frame += bytes([crc])
    ser.write(frame)
    time.sleep(0.5)
    while ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        print(f"  RAW ({len(data)} bytes):", data.hex())

    ser.close()
    print("\n[Done]")

except Exception as e:
    print(f"[FAIL] {e}")
