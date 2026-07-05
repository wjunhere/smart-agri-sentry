"""Run pyocd commander commands one at a time for each round."""
import subprocess, time, re, os

PYOCD = r"C:\Users\ASUS\AppData\Local\Programs\Python\Python314\Scripts\pyocd.exe"
os.environ['PATH'] = os.path.dirname(PYOCD) + ';' + os.environ.get('PATH', '')

# Preload libusb
import ctypes
dll_path = str(__import__('libusb_package').get_library_path())
ctypes.cdll.LoadLibrary(dll_path)

CMDS = [
    ("init", ""),
    ("halt", ""),
    ("read8 0x20000001", "new_sample_ready"),
    ("read8 0x20000003", "fsm_state"),
    ("read8 0x20000004", "fsm_one_sec"),
    ("read8 0x20000005", "fsm_seconds"),
    ("read8 0x20000006", "fsm_samples"),
    ("read8 0x20000007", "fsm_poll_cntdn"),
    ("read16 0x2000000c", "lora_tx_count"),
    ("read8 0x2000000e", "dbg_leaf_ok"),
    ("read8 0x2000000f", "dbg_soil_ok"),
    ("read16 0x20000014", "dbg_leaf_temp"),
    ("read16 0x20000016", "dbg_leaf_hum"),
    ("read16 0x20000018", "dbg_soil_temp"),
    ("read16 0x2000001a", "dbg_soil_hum"),
    ("read16 0x2000001c", "dbg_soil_ec"),
    ("read16 0x2000001e", "dbg_soil_salt"),
    ("read16 0x20000020", "dbg_soil_n"),
    ("read16 0x20000022", "dbg_soil_p"),
    ("read16 0x20000024", "dbg_soil_k"),
    ("read16 0x20000026", "dbg_soil_ph"),
    ("read32 0x2000005a", "last_sample+0"),
    ("read32 0x2000005e", "last_sample+4"),
    ("read32 0x20000062", "last_sample+8"),
    ("read32 0x20000066", "last_sample+12"),
    ("read32 0x2000006a", "last_sample+16"),
    ("read32 0x2000006e", "last_sample+20"),
    ("read32 0x20000280", "leaf_raw_rx+0"),
    ("read32 0x20000284", "leaf_raw_rx+4"),
    ("read8 0x20000288", "leaf_raw_rx+8"),
    ("read32 0x20000289", "soil_raw_rx+0"),
    ("read32 0x2000028d", "soil_raw_rx+4"),
    ("read32 0x20000291", "soil_raw_rx+8"),
    ("read32 0x20000295", "soil_raw_rx+12"),
    ("read32 0x20000299", "soil_raw_rx+16"),
    ("read8 0x2000029d", "soil_raw_rx+20"),
    ("read32 0x20000210", "lora_tx_buf+0"),
    ("read32 0x20000214", "lora_tx_buf+4"),
    ("read32 0x20000218", "lora_tx_buf+8"),
]

def parse_output(out):
    # pyocd commander output format: "ADDR:  HH HH HH HH   |ASCII|"
    results = {}
    for line in out.strip().split('\n'):
        line = line.strip()
        m = re.match(r'([0-9a-fA-F]+):\s+([0-9a-fA-F ]+)\s*\|', line)
        if m:
            addr = int(m.group(1), 16)
            hexbytes = m.group(2).strip()
            results[addr] = hexbytes
    return results

for rnd in range(1, 4):
    print(f"\n{'='*60}")
    print(f"  ROUND {rnd}")
    print(f"{'='*60}")

    all_results = {}

    for cmd, label in CMDS:
        result = subprocess.run(
            [PYOCD, "commander", "--target", "stm32f103rc", "-c", cmd],
            capture_output=True, text=True, timeout=15
        )
        out = result.stdout.strip()
        # Also check stderr for "halted device" message
        if not out:
            out = result.stderr.strip()
        parsed = parse_output(out)
        if parsed:
            for addr, hexbytes in parsed.items():
                all_results[addr] = hexbytes
                if label:
                    print(f"  {label:20s} 0x{addr:08x}: {hexbytes}")
                else:
                    print(f"  {'':20s} 0x{addr:08x}: {hexbytes}")
        elif out:
            # Just print non-matching lines for status
            if "halted" in out.lower() or "connected" in out.lower():
                pass  # skip status lines

    if rnd < 3:
        print(f"\n  [Running 15s...]")
        # Resume with a separate command
        subprocess.run([PYOCD, "commander", "--target", "stm32f103rc", "-c", "continue"],
                       capture_output=True, timeout=5)
        time.sleep(15)
        subprocess.run([PYOCD, "commander", "--target", "stm32f103rc", "-c", "halt"],
                       capture_output=True, timeout=5)

# Final reset
subprocess.run([PYOCD, "commander", "--target", "stm32f103rc", "-c", "reset"],
               capture_output=True, timeout=5)

print("\nDone!")
