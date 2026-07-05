"""Diagnose by talking to pyocd commander interactively via stdin/stdout."""
import subprocess, time, re, os

PYOCD = r"C:\Users\ASUS\AppData\Local\Programs\Python\Python314\Scripts\pyocd.exe"
os.environ['PATH'] = os.path.dirname(PYOCD) + ';' + os.environ.get('PATH', '')

# Preload libusb
import ctypes
dll_path = str(__import__('libusb_package').get_library_path())
ctypes.cdll.LoadLibrary(dll_path)

p = subprocess.Popen(
    [PYOCD, "commander", "--target", "stm32f103rc"],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    text=True, bufsize=1
)

def send_cmd(cmd, wait=0.5):
    p.stdin.write(cmd + "\n")
    p.stdin.flush()
    time.sleep(wait)

def read_output(timeout_s=3):
    """Read available output from pyocd commander."""
    import select
    output = []
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if select.select([p.stdout], [], [], 0.1)[0]:
            line = p.stdout.readline()
            if line:
                output.append(line.rstrip())
            else:
                break
        else:
            break
    return output

def parse_line(line):
    m = re.match(r'([0-9a-fA-F]+):\s+([0-9a-fA-F ]+)\s*\|', line)
    if m:
        return int(m.group(1), 16), m.group(2).strip()
    return None, None

# Initialize connection
send_cmd("init")
send_cmd("halt")
time.sleep(1)
_ = read_output()  # consume init/halt output

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

CMDS_ADDRS = [
    0x20000001, 0x20000003, 0x20000004, 0x20000005, 0x20000006, 0x20000007,
    0x2000000c, 0x2000000e, 0x2000000f,
    0x20000014, 0x20000016, 0x20000018, 0x2000001a,
    0x2000001c, 0x2000001e, 0x20000020, 0x20000022, 0x20000024, 0x20000026,
    0x2000005a, 0x2000005e, 0x20000062, 0x20000066, 0x2000006a, 0x2000006e,
    0x20000210, 0x20000214, 0x20000218,
    0x20000280, 0x20000284, 0x20000288,
    0x20000289, 0x2000028d, 0x20000291, 0x20000295, 0x20000299, 0x2000029d,
]

for rnd in range(1, 4):
    print(f"\n{'='*60}")
    print(f"  ROUND {rnd}")
    print(f"{'='*60}")

    # Halt and flush
    send_cmd("halt")
    time.sleep(0.5)
    _ = read_output()

    results = {}
    for addr in CMDS_ADDRS:
        # Pick command type
        label = LABELS[addr]
        if 'rx+8' in label or 'rx+20' in label:
            cmd = f"read8 0x{addr:08x}"
        elif addr in [0x2000000c, 0x20000014, 0x20000016, 0x20000018, 0x2000001a,
                       0x2000001c, 0x2000001e, 0x20000020, 0x20000022, 0x20000024, 0x20000026]:
            cmd = f"read16 0x{addr:08x}"
        else:
            cmd = f"read32 0x{addr:08x}"
        send_cmd(cmd)
        time.sleep(0.15)

    time.sleep(1)
    lines = read_output(2)

    for line in lines:
        addr, hexbytes = parse_line(line)
        if addr is not None:
            label = LABELS.get(addr, '')
            print(f"  {label:20s} 0x{addr:08x}: {hexbytes}")

    if rnd < 3:
        print(f"\n  [Resume, wait 15s...]")
        send_cmd("continue")
        time.sleep(15)

# Cleanup
send_cmd("reset")
send_cmd("exit")
time.sleep(0.5)
p.terminate()

print("\nDone!")
