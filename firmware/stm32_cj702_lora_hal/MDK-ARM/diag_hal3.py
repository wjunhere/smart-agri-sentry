import ctypes, os, sys, time, re, subprocess

dll_path = str(__import__('libusb_package').get_library_path())
os.environ['PATH'] = os.path.dirname(dll_path) + ';' + os.environ.get('PATH', '')
ctypes.cdll.LoadLibrary(dll_path)

from pyocd.core.helpers import ConnectHelper
from pyocd.commands.commander import PyOCDCommander
from io import StringIO
import tempfile

def run_cmd(session, cmd):
    buf = StringIO()
    c = PyOCDCommander(session, stream=buf)
    try:
        c.one_command(cmd)
    except Exception as e:
        pass
    return buf.getvalue().strip()

CMDS = [
    "read8 0x20000001",
    "read8 0x20000003",
    "read8 0x20000004",
    "read8 0x20000005",
    "read8 0x20000006",
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
    "read32 0x20000280",
    "read32 0x20000284",
    "read32 0x20000289",
    "read32 0x2000028d",
    "read32 0x20000291",
    "read32 0x20000295",
    "read32 0x20000299",
]

print("Connecting...")
with ConnectHelper.session_with_chosen_probe(target_override='stm32f103rc') as session:
    target = session.target
    target.halt()
    print("Halted. Starting 3 rounds...")

    for rnd in range(1, 4):
        print(f"\n{'='*60}")
        print(f"ROUND {rnd}")
        print(f"{'='*60}")

        for cmd in CMDS:
            out = run_cmd(session, cmd)
            # Parse hex
            m = re.search(r'([0-9a-fA-F]+):\s+([0-9a-fA-F\s]+)', out)
            if m:
                addr = m.group(1)
                hexbytes = m.group(2).strip()
                print(f"  {cmd:25s} 0x{addr}: {hexbytes}")
            else:
                print(f"  {cmd:25s} -> {out[:80]}")

        if rnd < 3:
            print(f"\n  [Resume, wait 15s...]")
            target.resume()
            time.sleep(15)
            target.halt()

    target.reset_and_halt()
    print("\nDone!")
