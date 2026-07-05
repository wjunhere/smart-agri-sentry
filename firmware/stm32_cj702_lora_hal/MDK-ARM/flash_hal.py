import ctypes, os, sys, traceback

try:
    import libusb_package
    dll_path = str(libusb_package.get_library_path())
    os.environ['PATH'] = os.path.dirname(dll_path) + ';' + os.environ.get('PATH', '')
    ctypes.cdll.LoadLibrary(dll_path)
    print(f"libusb loaded: {dll_path}")

    from pyocd.core.helpers import ConnectHelper
    from pyocd.flash.file_programmer import FileProgrammer

    print("Connecting to STM32F103RC...")
    with ConnectHelper.session_with_chosen_probe(target_override='stm32f103rc') as session:
        print(f"Connected! Board: {session.board}")
        # Halt first!
        session.target.halt()
        print("Target halted")
        # Disable watchdog by writing to DBGMCU_CR
        session.target.write_memory(0xE0042004, 0x00000007)
        print("Watchdog disabled")
        print("Flashing...")
        programmer = FileProgrammer(session)
        programmer.program('D:/stm32_cj702_lora_hal/MDK-ARM/Output/stm32_cj702_lora_hal.hex')
        print("FLASH OK")
        session.target.reset_and_halt()
        print("Device reset and halted")
except Exception as e:
    traceback.print_exc()
    sys.exit(1)
