"""Open COM9, then reset MCU, capture boot output."""
import serial, subprocess, time, sys

for baud in [115200, 9600]:
    try:
        p = serial.Serial('COM9', baud, timeout=8, bytesize=8, parity='N', stopbits=1)
        p.dtr = p.rts = True
        p.reset_input_buffer()
        print(f"Opened COM9 at {baud} baud", flush=True)

        # Reset MCU while serial port is open
        r = subprocess.run([
            r"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe",
            "-c","SWD","-Rst","-NoPrompt"
        ], capture_output=True, text=True, timeout=10)
        print(f"MCU reset: {r.stdout.strip()}")

        # Read all data
        data = b''
        t0 = time.time()
        while time.time() - t0 < 5:
            try:
                chunk = p.read(4096)
                if not chunk:
                    break
                data += chunk
            except:
                break

        p.close()

        if data:
            print(f"\n=== {len(data)} bytes at {baud} baud ===")
            print(data.decode('ascii', errors='replace'))
            sys.exit(0)
        else:
            print(f"No data at {baud} baud")
    except Exception as e:
        print(f"Error at {baud}: {e}")

print("No data at any baud rate")
