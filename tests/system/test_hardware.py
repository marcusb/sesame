import pytest
import subprocess
import time
import os
import re
import struct
import tempfile
import threading
import requests

def crc8_ccitt(data: bytes, initial=0xFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
    return crc & 0xFF

def align(length: int, wbs: int) -> int:
    if wbs <= 1:
        return length
    return (length + wbs - 1) & ~(wbs - 1)

def generate_nvs_blob(entries, flash_size=32768, sector_size=4096, wbs=4):
    blob = bytearray(b'\xff' * flash_size)
    data_wra = 0
    ate_wra = sector_size
    ate_size = align(8, wbs) # struct nvs_ate is 8 bytes

    for _id, data in entries:
        data_len = len(data)
        aligned_data_len = align(data_len, wbs)

        # write data
        blob[data_wra : data_wra + data_len] = data

        # create ATE (uint16_t id, uint16_t offset, uint16_t len, uint8_t part, uint8_t crc8)
        offset = data_wra
        part = 0xff
        ate_head = struct.pack("<HHHBB", _id, offset, data_len, part, 0xFF)
        crc = crc8_ccitt(ate_head)
        ate = struct.pack("<HHHBB", _id, offset, data_len, part, crc)

        ate_wra -= ate_size
        blob[ate_wra : ate_wra + 8] = ate

        data_wra += aligned_data_len

    return bytes(blob)

def resolve_mock_flash_address(elf_path):
    res = subprocess.run(["/home/marcus/zephyr-sdk/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm", elf_path], capture_output=True, text=True)
    if res.returncode != 0:
        pytest.fail(f"nm failed on {elf_path}: {res.stderr.strip()}")
    for line in res.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2] == "mock_flash_0":
            return int(parts[0], 16)
    pytest.fail(f"mock_flash_0 symbol missing from {elf_path}")

def inject_nvs_direct(ocd, flash_addr, nvs_id, data):
    """Halts target during boot delay, writes generated NVS blob directly to RAM flash simulator, and resumes."""
    blob = generate_nvs_blob([(nvs_id, data)])

    with tempfile.NamedTemporaryFile(mode="wb", suffix=".bin", delete=False) as f:
        f.write(blob)
        blob_path = f.name

    try:
        # Halt the board (which should be in the boot delay loop)
        ocd.halt()

        # Load the binary image into RAM at the mock_flash_0 address
        # OpenOCD load_image writes the file directly
        # Syntax: load_image filename address bin
        ocd.run(f"load_image {blob_path} {hex(flash_addr)} bin", wait=1.0)

        # Resume to boot the application with the flashed data
        ocd.resume()
    finally:
        os.remove(blob_path)


@pytest.fixture
def hardware_device(request, test_app_config, openocd):
    port = request.config.getoption("--device-port")
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

    flash_addr = resolve_mock_flash_address(elf_path)

    print("Connecting to hardware via OpenOCD semihost console...")
    
    import serial
    
    firmware_logs = []
    stop_reader = threading.Event()

    def log_reader():
        try:
            with serial.Serial(port, 115200, timeout=0.1) as ser:
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


def test_http_door_endpoints(hardware_device):
    ip = hardware_device["ip"]
    logs = hardware_device["logs"]

    print(f"Testing POST http://{ip}/open")

    from requests.adapters import HTTPAdapter
    from urllib3.util.retry import Retry

    session = requests.Session()
    retry = Retry(connect=5, backoff_factor=0.5)
    adapter = HTTPAdapter(max_retries=retry)
    session.mount('http://', adapter)
    session.mount('https://', adapter)

    resp = session.post(f"http://{ip}/open", timeout=5)
    assert resp.status_code == 200

    time.sleep(0.5)
    assert any("PIC: OPEN" in line for line in logs), "Door open command not logged by firmware"

    print(f"Testing POST http://{ip}/close")
    resp = session.post(f"http://{ip}/close", timeout=5)
    assert resp.status_code == 200

    time.sleep(7.5)
    if not any("PIC: CLOSE" in line for line in logs):
        print("Last 20 log lines:")
        for line in logs[-20:]:
            print(line)
        assert False, "Door close command not logged by firmware"

    print("HTTP endpoints verified successfully!")
