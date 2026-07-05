"""Diagnose using batched pyocd commander with per-round connections."""
import subprocess, time, re, os

PYOCD = r"C:\Users\ASUS\AppData\Local\Programs\Python\Python314\Scripts\pyocd.exe"
os.environ['PATH'] = os.path.dirname(PYOCD) + ';' + os.environ.get('PATH', '')

import ctypes
dll_path = str(__import__('libusb_package').get_library_path())
ctypes.cdll.LoadLibrary(dll_path)

LABELS = {
    0x20000001: 'new_sample_ready',
    0x20000003: 'fsm_state',
    0x20000004: 'fsm_1sec',
    0x20000005: 'fsm_seconds',
    0x20000006: 'fsm_samples',
    0x20000007: 'fsm_poll_cnt',
    0x2000000c: 'lora_tx_count',
    0x2000000e: 'dbg_leaf_ok',
    0x2000000f: 'dbg_soil_ok',
    0x20000014: 'dbg_leaf_temp',
    0x20000016: 'dbg_leaf_hum',
    0x20000018: 'dbg_soil_temp',
    0x2000001a: 'dbg_soil_hum',
    0x2000001c: 'dbg_soil_ec',
    0x2000001e: 'dbg_soil_salt',
    0x20000020: 'dbg_soil_n',
    0x20000022: 'dbg_soil_p',
    0x20000024: 'dbg_soil_k',
    0x20000026: 'dbg_soil_ph',
    0x2000005a: 'last_sample+0',
    0x2000005e: 'last_sample+4',
    0x20000062: 'last_sample+8',
    0x20000066: 'last_sample+12',
    0x2000006a: 'last_sample+16',
    0x2000006e: 'last_sample+20',
    0x20000210: 'lora_tx_buf+0',
    0x20000214: 'lora_tx_buf+4',
    0x20000218: 'lora_tx_buf+8',
    0x20000280: 'leaf_raw_rx+0',
    0x20000284: 'leaf_raw_rx+4',
    0x20000288: 'leaf_raw_rx+8',
    0x20000289: 'soil_raw_rx+0',
    0x2000028d: 'soil_raw_rx+4',
    0x20000291: 'soil_raw_rx+8',
    0x20000295: 'soil_raw_rx+12',
    0x20000299: 'soil_raw_rx+16',
    0x2000029d: 'soil_raw_rx+20',
}

# Build consolidated command string with all reads, separated by semicolons or newlines
READ_CMDS = [
    "read8 0x20000001",
    "read8 0x20000003",
    "read8 0x20000004",
    "read8 0x20000005",
    "read8 0x20000006",
    "read8 0x20000007",
    "read16 0x2000000c",
    "read8 0x2000000e",
    "read8 0x2000000f",
    "read16 0x20000014",
    "read16 0x20000016",
    "read16 0x20000018",
    "read16 0x2000001a",
    "read16 0x2000001c",
    "read16 0x2000001e",
    "read16 0x20000020",
    "read16 0x20000022",
    "read16 0x20000024",
    "read16 0x20000026",
    "read32 0x2000005a",
    "read32 0x2000005e",
    "read32 0x20000062",
    "read32 0x20000066",
    "read32 0x2000006a",
    "read32 0x2000006e",
    "read32 0x20000210",
    "read32 0x20000214",
    "read32 0x20000218",
    "read32 0x20000280",
    "read32 0x20000284",
    "read8 0x20000288",
    "read32 0x20000289",
    "read32 0x2000028d",
    "read32 0x20000291",
    "read32 0x20000295",
    "read32 0x20000299",
    "read8 0x2000029d",
]

def parse_output(out):
    results = {}
    for line in out.split('\n'):
        line = line.strip()
        m = re.match(r'([0-9a-fA-F]+):\s+([0-9a-fA-F ]+)\s*\|', line)
        if m:
            addr = int(m.group(1), 16)
            hexbytes = m.group(2).strip()
            results[addr] = hexbytes
    return results

def dump_round():
    """Run pyocd commander with all read commands, return parsed results."""
    cmd_str = "; ".join(["init", "halt"] + READ_CMDS)
    result = subprocess.run(
        [PYOCD, "commander", "--target", "stm32f103rc", "-c", cmd_str],
        capture_output=True, text=True, timeout=30
    )
    return parse_output(result.stdout + "\n" + result.stderr)

def resume_device():
    subprocess.run(
        [PYOCD, "commander", "--target", "stm32f103rc", "-c", "continue"],
        capture_output=True, timeout=5
    )

for rnd in range(1, 4):
    print(f"\n{'='*60}")
    print(f"  ROUND {rnd}")
    print(f"{'='*60}")

    results = dump_round()

    for addr in sorted(results.keys()):
        label = LABELS.get(addr, '')
        hexbytes = results[addr]
        # Convert to int for display
        vals = [int(b, 16) for b in hexbytes.split()]
        # Build meaningful display
        if len(vals) <= 4:
            intval = 0
            for i, v in enumerate(vals):
                intval |= v << (8 * i)
            if len(vals) <= 2:
                print(f"  {label:20s} 0x{addr:08x}: {hexbytes:12s} = {intval}")
            else:
                print(f"  {label:20s} 0x{addr:08x}: {hexbytes:24s} = {intval}")
        else:
            print(f"  {label:20s} 0x{addr:08x}: {hexbytes}")

    if rnd < 3:
        print(f"\n  [Resume, wait 20s...]")
        resume_device()
        time.sleep(20)

print("\nDone!")
