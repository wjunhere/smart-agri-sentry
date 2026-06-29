import serial
import subprocess
import time
import sys

# Flash the MCU first
print("Flashing MCU...")
result = subprocess.run([
    r"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe",
    "-c", "SWD",
    "-P", r"D:\stm_design\Intelligent_Agricultural_Robot_Car\MDK-ARM\Intelligent_Agricultural_Robot_Car\Intelligent_Agricultural_Robot_Car.hex",
    "-Rst", "-NoPrompt"
], capture_output=True, text=True, timeout=15)
print(result.stdout)
if result.stderr:
    print("STDERR:", result.stderr)

# Now read from COM9
baud = 115200
print(f"\nAttempting to read from COM9 at {baud} baud...")
try:
    port = serial.Serial('COM9', baud, timeout=10, bytesize=8, parity='N', stopbits=1)
    port.dtr = True
    port.rts = True
    time.sleep(0.3)

    data = b''
    while True:
        try:
            chunk = port.read(1024)
            if not chunk:
                break
            data += chunk
        except serial.SerialTimeoutException:
            break

    port.close()

    if data:
        print(f"\n=== CAPTURED {len(data)} bytes at {baud} baud ===")
        # Try to decode as ASCII (replace non-printable)
        text = data.decode('ascii', errors='replace')
        print(text)
    else:
        print(f"\nNo data received at {baud} baud. Trying 9600...")
        port = serial.Serial('COM9', 9600, timeout=10, bytesize=8, parity='N', stopbits=1)
        port.dtr = True
        port.rts = True
        time.sleep(0.3)

        data = b''
        while True:
            try:
                chunk = port.read(1024)
                if not chunk:
                    break
                data += chunk
            except serial.SerialTimeoutException:
                break

        port.close()

        if data:
            print(f"\n=== CAPTURED {len(data)} bytes at 9600 baud ===")
            text = data.decode('ascii', errors='replace')
            print(text)
        else:
            print("No data received at either baud rate.")
except Exception as e:
    print(f"Error: {e}")
