import serial, subprocess, time, sys

# Flash
r = subprocess.run([
    r"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe",
    "-c","SWD",
    "-P",r"D:\stm_design\Intelligent_Agricultural_Robot_Car\MDK-ARM\Intelligent_Agricultural_Robot_Car\Intelligent_Agricultural_Robot_Car.hex",
    "-Rst","-NoPrompt"
], capture_output=True, text=True, timeout=15)
print("FLASH:", r.stdout.strip())

# Try 115200 first, then 9600
for baud in [115200, 9600]:
    try:
        p = serial.Serial('COM9', baud, timeout=5, bytesize=8, parity='N', stopbits=1)
        p.dtr = p.rts = True
        time.sleep(0.5)
        data = b''
        t0 = time.time()
        while time.time() - t0 < 6:
            c = p.read(4096)
            if not c: break
            data += c
        p.close()
        if data:
            print(f"\n=== {len(data)} bytes at {baud} baud ===")
            print(data.decode('ascii', errors='replace'))
            break
        else:
            print(f"No data at {baud}")
    except Exception as e:
        print(f"Error at {baud}: {e}")
else:
    print("\nNo data at either baud rate.")
