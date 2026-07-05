import ctypes, os, sys, time

dll_path = str(__import__('libusb_package').get_library_path())
os.environ['PATH'] = os.path.dirname(dll_path) + ';' + os.environ.get('PATH', '')
ctypes.cdll.LoadLibrary(dll_path)

from pyocd.core.helpers import ConnectHelper

ADDRS = {
    'new_sample_ready':  (0x20000001, 1),
    'fsm_state':         (0x20000003, 1),
    'fsm_1sec':          (0x20000004, 1),
    'fsm_seconds':       (0x20000005, 1),
    'fsm_samples':       (0x20000006, 1),
    'lora_tx_count':     (0x2000000c, 2),
    'dbg_leaf_ok':       (0x2000000e, 1),
    'dbg_soil_ok':       (0x2000000f, 1),
    'dbg_leaf_temp':     (0x20000014, 2),
    'dbg_leaf_hum':      (0x20000016, 2),
    'dbg_soil_temp':     (0x20000018, 2),
    'dbg_soil_hum':      (0x2000001a, 2),
    'dbg_soil_ec':       (0x2000001c, 2),
    'dbg_soil_salt':     (0x2000001e, 2),
    'dbg_soil_n':        (0x20000020, 2),
    'dbg_soil_p':        (0x20000022, 2),
    'dbg_soil_k':        (0x20000024, 2),
    'dbg_soil_ph':       (0x20000026, 2),
    'last_sample_0':     (0x2000005a, 4),
    'last_sample_4':     (0x2000005e, 4),
    'last_sample_8':     (0x20000062, 4),
    'last_sample_12':    (0x20000066, 4),
    'last_sample_16':    (0x2000006a, 4),
    'last_sample_20':    (0x2000006e, 4),
    'leaf_raw_0':        (0x20000280, 4),
    'leaf_raw_4':        (0x20000284, 4),
    'leaf_raw_8':        (0x20000288, 1),
    'soil_raw_0':        (0x20000289, 4),
    'soil_raw_4':        (0x2000028d, 4),
    'soil_raw_8':        (0x20000291, 4),
    'soil_raw_12':       (0x20000295, 4),
    'soil_raw_16':       (0x20000299, 4),
    'soil_raw_20':       (0x2000029d, 1),
}

def fmt_hex(data):
    if isinstance(data, int):
        n = len(data) if hasattr(data, '__len__') else 1
    return ' '.join(f'{b:02x}' for b in (data if hasattr(data, '__len__') else [data]))

print("Connecting...")
with ConnectHelper.session_with_chosen_probe(target_override='stm32f103rc') as session:
    target = session.target
    target.halt()
    print("Halted. Starting 3 rounds...")

    for rnd in range(1, 4):
        print(f"\n{'='*60}")
        print(f"ROUND {rnd}")
        print(f"{'='*60}")

        for name, (addr, size) in ADDRS.items():
            try:
                val = target.read_memory(addr, size)
                hexstr = ' '.join(f'{b:02x}' for b in val)
                if size <= 2:
                    intval = int.from_bytes(val, 'little')
                    print(f"  {name:20s} 0x{addr:08x} {hexstr:12s} = {intval}")
                else:
                    print(f"  {name:20s} 0x{addr:08x} {hexstr}")
            except Exception as e:
                print(f"  {name:20s} 0x{addr:08x} ERROR: {e}")

        if rnd < 3:
            print(f"\n  [Resume, wait 12s...]")
            target.resume()
            time.sleep(12)
            target.halt()

    target.reset_and_halt()
    print("\nDone!")
