"""Full pipeline diagnostic: flash, run 70s, read all sensor + LoRa variables."""
import socket
import subprocess
import time
import sys

def telnet_cmd(s, cmd, wait=0.3):
    """Send command via telnet and return response text."""
    s.sendall((cmd + "\n").encode())
    time.sleep(wait)
    data = b""
    s.settimeout(2)
    while True:
        try:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk
            if b"\r\n> " in data:
                break
        except:
            break
    # Clean telnet escape sequences and prompt
    text = data.decode('utf-8', errors='replace')
    # Remove telnet negotiation
    lines = []
    for line in text.split('\n'):
        line = line.strip()
        if line and not line.startswith('>') and 'mdw' not in line and 'mdb' not in line:
            lines.append(line)
    return lines

def main():
    # Connect to telnet
    s = socket.socket()
    s.settimeout(5)
    s.connect(('127.0.0.1', 4444))
    time.sleep(0.3)
    s.recv(4096)  # consume banner

    # Init and flash
    print("=== Init & Flash ===")
    resp = telnet_cmd(s, "init")
    for r in resp: print(r[:120])

    resp = telnet_cmd(s, "reset halt", wait=1)
    for r in resp: print(r[:120])

    resp = telnet_cmd(s, "program D:/stm32_cj702_lora_hal/MDK-ARM/Output/stm32_cj702_lora_hal.axf verify", wait=5)
    for r in resp: print(r[:120])

    # Resume and wait 70s
    print("\n=== Running for 70s ===")
    telnet_cmd(s, "reset", wait=1)
    telnet_cmd(s, "resume", wait=0.5)

    for i in range(0, 70, 5):
        time.sleep(5)
        print(f"  {i+5}s...")

    # Halt
    print("\n=== Halting ===")
    resp = telnet_cmd(s, "halt", wait=1)
    for r in resp: print(r[:120])

    # Read ALL diagnostics
    print("\n========== DIAGNOSTICS ==========")

    cmds = [
        ("=== LoRa TX ===", "mdw 0x20000008 4"),
        ("  g_lora_tx_status, g_lora_tx_len(=30), g_lora_tx_count, leaf_ok+soil_ok+addr", None),
        ("=== Debug flags (addr 0x0E-0x13) ===", "mdw 0x2000000e 2"),
        ("  leaf_ok, soil_ok, soil_addr, soil_addr_ok", None),
        ("=== Leaf/Soil temp+hum (addr 0x14-0x27) ===", "mdw 0x20000014 6"),
        ("  leaf_temp, leaf_hum, soil_temp, soil_hum, ec, salt, n, p, k, ph", None),
        ("=== Soil sensor raw diag ===", "mdw 0x20000028 4"),
        ("  rx_count, tx_ok, rx_status, diag_state, crc_calc, raw_tx[0-3]", None),
        ("=== Soil raw TX (8 bytes) ===", "mdb 0x2000002e 8"),
        ("=== Soil raw RX (21 bytes) ===", "mdb 0x20000289 21"),
        ("=== Leaf raw RX (9 bytes) ===", "mdb 0x20000280 9"),
        ("=== g_last_sample (24 bytes) ===", "mdb 0x2000005a 24"),
        ("=== g_lora_tx_buf (first 32 bytes) ===", "mdb 0x20000210 32"),
        ("=== g_last_soil_data (16 bytes) ===", "mdb 0x20000240 16"),
    ]

    for label, cmd in cmds:
        if cmd is None:
            print(label)
        else:
            print(label)
            resp = telnet_cmd(s, cmd, wait=0.3)
            for r in resp:
                print(f"  {r}")

    print("\n=== Done ===")
    telnet_cmd(s, "exit")
    s.close()

if __name__ == "__main__":
    main()
