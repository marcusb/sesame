import pytest
import os
import time
import re
import threading

@pytest.fixture
def hardware_device(device_port, test_app_config, openocd):
    elf_paths = [
        "build/sesame_test/zephyr/zephyr.elf",
        "build/sesame/sesame_test/zephyr/zephyr.elf",
    ]
    elf_path = None
    for p in elf_paths:
        if os.path.exists(p):
            elf_path = p
            break

    if not elf_path:
        pytest.fail(f"sesame_test zephyr.elf not found. Run: west build -b genie_idcm/88mw320/cpu0 -d build/sesame --sysbuild")

    print("Connecting to hardware via OpenOCD semihost console...")
    
    import serial
    
    firmware_logs = []
    stop_reader = threading.Event()

    def log_reader():
        try:
            with serial.Serial(device_port, 115200, timeout=0.1) as ser:
                while not stop_reader.is_set():
                    line = ser.readline()
                    if line:
                        try:
                            decoded = line.decode('utf-8', errors='ignore').strip()
                            if decoded:
                                firmware_logs.append(decoded)
                        except:
                            pass
        except Exception as e:
            print(f"Serial port error: {e}")

    reader_thread = threading.Thread(target=log_reader, daemon=True)
    reader_thread.start()

    # Boot the device. The firmware has CONFIG_BOOT_DELAY=2000,
    # so it will zero .bss and then spin for 2 seconds.
    openocd.reboot()

    # Wait for .bss zeroing to finish (openocd.reboot waits 1.0s, so we are in the 2s window)
    time.sleep(0.1)

    # Write NetworkConfig to file for semihosting access
    config_bytes = test_app_config.SerializeToString()
    with open("test_config.bin", "wb") as f:
        f.write(config_bytes)
    print("Wrote AppConfig to test_config.bin for semihosting")

    # Wait for Wi-Fi connection and IP
    device_ip = None
    start_time = time.time()
    while time.time() - start_time < 60:
        for line in list(firmware_logs):
            # Match either IPv4 or IPv6 address
            ip_match = re.search(r"IPv[46] address: ([0-9a-fA-F:\.]+)", line)
            if ip_match and not device_ip:
                # If it's IPv6 link-local (fe80), skip it, we want a routable one if possible,
                # but if it's the only one, we can use it (might need zone id though).
                # Actually, the zephyr device gets a global IPv6 addr (e.g. 2600:...)
                ip_str = ip_match.group(1)
                if not ip_str.startswith("fe80"):
                    device_ip = f"[{ip_str}]" if ":" in ip_str else ip_str
                    break
        if device_ip:
            print(f"Parsed IP: {device_ip}. Firmware logs so far:\n" + "\n".join(firmware_logs))
            break
        time.sleep(0.1)

    assert device_ip, f"Device failed to connect to Wi-Fi and acquire IP. Logs:\n{chr(10).join(firmware_logs)}"
    print(f"Device ready on Wi-Fi at IP {device_ip}")

    yield {
        "ip": device_ip,
        "logs": firmware_logs
    }

    stop_reader.set()
