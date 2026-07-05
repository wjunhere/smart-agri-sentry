"""Run pyocd commander with script file to read memory, using the approach from the previous session."""
import subprocess, os, time, re

PYOCD = r"C:\Users\ASUS\AppData\Local\Programs\Python\Python314\Scripts\pyocd.exe"

CMDS_FILE = "diag_cmds.txt"

for rnd in range(1, 4):
    print(f"\n{'='*60}")
    print(f"ROUND {rnd}")
    print(f"{'='*60}")

    # Write the command script
    with open("D:/stm32_cj702_lora_hal/MDK-ARM/diag_cmds_live.txt", "w") as f:
        f.write("init\nhalt\n")
        f.write("read8 0x20000001\n")     # g_new_sample_ready
        f.write("read8 0x20000003\n")     # fsm state
        f.write("read8 0x20000004\n")     # fsm 1sec
        f.write("read8 0x20000005\n")     # fsm seconds
        f.write("read8 0x20000006\n")     # fsm samples
        f.write("read16 0x2000000c\n")    # lora_tx_count
        f.write("read8 0x2000000e\n")     # dbg_leaf_ok
        f.write("read8 0x2000000f\n")     # dbg_soil_ok
        f.write("read16 0x20000014\n")    # dbg_leaf_temp
        f.write("read16 0x20000016\n")    # dbg_leaf_hum
        f.write("read16 0x20000018\n")    # dbg_soil_temp
        f.write("read16 0x2000001a\n")    # dbg_soil_hum
        f.write("read16 0x2000001c\n")    # dbg_soil_ec
        f.write("read16 0x2000001e\n")    # dbg_soil_salt
        f.write("read16 0x20000020\n")    # dbg_soil_n
        f.write("read16 0x20000022\n")    # dbg_soil_p
        f.write("read16 0x20000024\n")    # dbg_soil_k
        f.write("read16 0x20000026\n")    # dbg_soil_ph
        f.write("read32 0x2000005a\n")    # g_last_sample[0:4]
        f.write("read32 0x2000005e\n")    # g_last_sample[4:8]
        f.write("read32 0x20000062\n")    # g_last_sample[8:12]
        f.write("read32 0x20000066\n")    # g_last_sample[12:16]
        f.write("read32 0x2000006a\n")    # g_last_sample[16:20]
        f.write("read32 0x2000006e\n")    # g_last_sample[20:24]
        f.write("read32 0x20000280\n")    # leaf_raw_rx[0:4]
        f.write("read32 0x20000284\n")    # leaf_raw_rx[4:8]
        f.write("read32 0x20000289\n")    # soil_raw_rx[0:4]
        f.write("read32 0x2000028d\n")    # soil_raw_rx[4:8]
        f.write("read32 0x20000291\n")    # soil_raw_rx[8:12]
        f.write("read32 0x20000295\n")    # soil_raw_rx[12:16]
        f.write("read32 0x20000299\n")    # soil_raw_rx[16:20]
        f.write("read32 0x20000210\n")    # lora_tx_buf[0:4]
        f.write("read32 0x20000214\n")    # lora_tx_buf[4:8]
        f.write("read32 0x20000218\n")    # lora_tx_buf[8:12]
        if rnd < 3:
            f.write("continue\n")
        else:
            f.write("exit\n")

    result = subprocess.run(
        [PYOCD, "commander", "--target", "stm32f103rc", "--script", "D:/stm32_cj702_lora_hal/MDK-ARM/diag_cmds_live.txt"],
        capture_output=True, text=True,
        cwd="D:/stm32_cj702_lora_hal/MDK-ARM",
        timeout=30
    )

    out = result.stdout

    # Parse key values
    for line in out.split('\n'):
        line = line.strip()
        m = re.search(r'([0-9a-fA-F]+):\s+([0-9a-fA-F\s\|\.]+)', line)
        if m:
            addr = int(m.group(1), 16)
            hex_ascii = m.group(2)
            # Extract only hex bytes
            hexbytes = ' '.join(
                h for h in hex_ascii.split('|')[0].split()
                if len(h) == 2 and h not in ('..', '  ')
            )
            # Find raw hex
            raw_match = re.findall(r'([0-9a-fA-F]{2})', hex_ascii.split('|')[0])
            hexbytes = ' '.join(raw_match)

            # Labels
            labels = {
                0x20000001: 'new_sample_ready',
                0x20000003: 'fsm_state',
                0x20000004: 'fsm_1sec',
                0x20000005: 'fsm_seconds',
                0x20000006: 'fsm_samples',
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
                0x20000289: 'soil_raw_rx+0',
                0x2000028d: 'soil_raw_rx+4',
                0x20000291: 'soil_raw_rx+8',
                0x20000295: 'soil_raw_rx+12',
                0x20000299: 'soil_raw_rx+16',
            }
            label = labels.get(addr, '')
            print(f"  {label:20s} 0x{addr:08x}: {hexbytes}")

    if rnd < 3:
        print(f"\n  [Waiting 15s before next round...]")
        time.sleep(15)

print("\nDone!")
