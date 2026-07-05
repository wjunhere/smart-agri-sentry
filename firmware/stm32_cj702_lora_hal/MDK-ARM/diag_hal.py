import ctypes, os, sys, time, re

dll_path = str(__import__('libusb_package').get_library_path())
os.environ['PATH'] = os.path.dirname(dll_path) + ';' + os.environ.get('PATH', '')
ctypes.cdll.LoadLibrary(dll_path)

from pyocd.core.helpers import ConnectHelper
from pyocd.commands.commander import PyOCDCommander
from pyocd.utility.cmdline import convert_session_options
from io import StringIO

def read_vars(session):
    """Read key variables via commander and return parsed results."""
    cmds = [
        "read8 0x20000001",   # g_new_sample_ready
        "read8 0x20000003",   # g_app_fsm.state
        "read8 0x20000004",   # g_app_fsm.one_second_elapsed
        "read8 0x20000005",   # g_app_fsm.seconds
        "read8 0x20000006",   # g_app_fsm.sample_count
        "read16 0x2000000c",  # g_lora_tx_count
        "read8 0x2000000e",   # g_dbg_leaf_ok
        "read8 0x2000000f",   # g_dbg_soil_ok
        "read16 0x20000014",  # g_dbg_leaf_temp
        "read16 0x20000016",  # g_dbg_leaf_hum
        "read16 0x20000018",  # g_dbg_soil_temp
        "read16 0x2000001a",  # g_dbg_soil_hum
        "read16 0x2000001c",  # g_dbg_soil_ec
        "read16 0x2000001e",  # g_dbg_soil_salt
        "read16 0x20000020",  # g_dbg_soil_n
        "read16 0x20000022",  # g_dbg_soil_p
        "read16 0x20000024",  # g_dbg_soil_k
        "read16 0x20000026",  # g_dbg_soil_ph
        "read32 0x2000005a",  # g_last_sample[0:4]
        "read32 0x2000005e",  # g_last_sample[4:8]
        "read32 0x20000062",  # g_last_sample[8:12]
        "read32 0x20000066",  # g_last_sample[12:16]
        "read32 0x2000006a",  # g_last_sample[16:20]
        "read32 0x2000006e",  # g_last_sample[20:24]
        "read32 0x20000280",  # leaf_raw_rx[0:4]
        "read32 0x20000284",  # leaf_raw_rx[4:8]
        "read32 0x20000289",  # soil_raw_rx[0:4]
        "read32 0x2000028d",  # soil_raw_rx[4:8]
        "read32 0x20000291",  # soil_raw_rx[8:12]
        "read32 0x20000295",  # soil_raw_rx[12:16]
        "read32 0x20000299",  # soil_raw_rx[16:20]
        "read32 0x2000029d",  # soil_raw_rx[20:21] + padding
    ]
    results = []
    for cmd in cmds:
        buf = StringIO()
        c = PyOCDCommander(session, stream=buf)
        c.one_command(cmd)
        out = buf.getvalue().strip()
        # Parse hex value
        m = re.search(r'([0-9a-fA-F]+):\s+([0-9a-fA-F ]+)', out)
        if m:
            addr = int(m.group(1), 16)
            hexbytes = m.group(2).strip()
            results.append((addr, hexbytes, out))
        else:
            results.append((0, out, out))
    return results

print("Connecting to STM32F103RC...")
with ConnectHelper.session_with_chosen_probe(target_override='stm32f103rc') as session:
    session.target.halt()
    print("Target halted, starting 3-round diagnostic...")

    for round_num in range(1, 4):
        print(f"\n{'='*60}")
        print(f"  ROUND {round_num}")
        print(f"{'='*60}")
        results = read_vars(session)

        for addr, hexbytes, raw in results:
            print(f"  {raw}")

        if round_num < 3:
            print(f"\n  Running for 12 seconds...")
            session.target.resume()
            time.sleep(12)
            session.target.halt()

    session.target.reset_and_halt()
    print("\nDone!")
