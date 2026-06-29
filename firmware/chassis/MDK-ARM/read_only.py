"""Just read COM9 — no flash, no reset. MCU should already be running."""
import serial, time, sys

for baud in [115200, 9600]:
    try:
        p = serial.Serial('COM9', baud, timeout=4, bytesize=8, parity='N', stopbits=1)
        p.dtr = False
        p.rts = False
        p.reset_input_buffer()
        print(f"Opened COM9 at {baud} baud, reading...", flush=True)

        data = b''
        t0 = time.time()
        while time.time() - t0 < 4:
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
