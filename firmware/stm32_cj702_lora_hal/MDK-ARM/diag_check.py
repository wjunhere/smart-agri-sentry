#!/usr/bin/env python3
"""OpenOCD diagnostic checker - connects via TCL port, runs device, reads variables."""
import socket
import subprocess
import time
import sys
import os

OPENOCD = r"D:\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe"
SCRIPTS = r"D:\openocd\xpack-openocd-0.12.0-7\openocd\scripts"

def tcl_cmd(sock, cmd):
    """Send a TCL command and read the response."""
    sock.sendall((cmd + "\n").encode())
    time.sleep(0.1)
    try:
        sock.settimeout(2.0)
        data = b""
        while True:
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                data += chunk
                if b"\x1a" in chunk:  # TCL prompt terminator
                    break
            except socket.timeout:
                break
        # Clean up: remove TCL prompt marker and strip
        text = data.decode('utf-8', errors='replace')
        # Remove the TCL prompt marker
        text = text.replace('\x1a', '')
        return text.strip()
    except Exception as e:
        return f"ERROR: {e}"

def main():
    # Kill any existing OpenOCD
    subprocess.run(["taskkill", "/F", "/IM", "openocd.exe"],
                   capture_output=True, shell=True)
    time.sleep(1)

    # Start OpenOCD daemon
    print("Starting OpenOCD daemon...")
    proc = subprocess.Popen(
        [OPENOCD, "-s", SCRIPTS,
         "-f", "interface/stlink-v2.cfg",
         "-f", "target/stm32f1x.cfg"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(3)
    print(f"OpenOCD PID: {proc.pid}")

    # Connect to TCL port
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(("127.0.0.1", 6666))
        sock.settimeout(5.0)
        print("Connected to OpenOCD TCL port")
    except Exception as e:
        print(f"Failed to connect: {e}")
        proc.terminate()
        return 1

    # Consume initial banner
    time.sleep(0.5)
    try:
        sock.recv(4096)
    except:
        pass

    # Step 1: Reset and halt
    print("\n--- Reset & Halt ---")
    resp = tcl_cmd(sock, "reset halt")
    print(resp[:200] if resp else "(no response)")

    # Step 2: Resume and wait 70 seconds
    print("\n--- Running for 70 seconds... ---")
    resp = tcl_cmd(sock, "resume")
    print(resp[:200] if resp else "(no response)")

    # Wait 70 seconds for data collection + LoRa transmission
    for i in range(70):
        time.sleep(1)
        if i % 10 == 9:
            print(f"  {i+1}s elapsed...")

    # Step 3: Halt
    print("\n--- Halting ---")
    resp = tcl_cmd(sock, "halt")
    print(resp[:200] if resp else "(no response)")

    # Step 4: Read diagnostics
    print("\n========== DIAGNOSTICS ==========")

    # Read memory regions
    regions = [
        ("g_lora_tx (status len count)", "0x20000008", 6, "mdb"),
        ("g_dbg leaf+soil ok+temps+hums", "0x2000000e", 26, "mdb"),
        ("soil diag (rx_count tx_ok rx_status diag_state)", "0x20000028", 8, "mdb"),
        ("soil_raw_tx (8 bytes)", "0x2000002e", 8, "mdb"),
        ("g_last_sample (24 bytes)", "0x2000005a", 24, "mdb"),
        ("g_lora_tx_buf (first 32 bytes)", "0x20000210", 32, "mdb"),
        ("soil_raw_rx (21 bytes)", "0x20000289", 21, "mdb"),
        ("leaf_raw_rx (9 bytes)", "0x20000280", 9, "mdb"),
    ]

    for label, addr, count, cmd_type in regions:
        # Try both mdb and mdw
        cmd = f"mdb {addr} {count}"
        resp = tcl_cmd(sock, cmd)
        # Parse hex bytes from response
        lines = resp.split('\n')
        hex_bytes = []
        for line in lines:
            # Look for hex patterns like "0x20000008: 00 01 02 ..."
            if ':' in line and any(c in line for c in '0123456789abcdef'):
                parts = line.split(':')
                if len(parts) >= 2:
                    hex_part = parts[-1].strip()
                    hex_bytes.extend(hex_part.split())

        if hex_bytes:
            print(f"\n{label}: {' '.join(hex_bytes)}")
        else:
            print(f"\n{label}: (no data - raw: {resp[:100]})")

    # Step 5: Exit
    print("\n--- Shutting down ---")
    tcl_cmd(sock, "exit")
    sock.close()
    proc.wait(timeout=5)
    print("Done.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
