import pytest
import subprocess
import time
import os
import tempfile
import serial
import json
import threading
import requests

def inject_nvs_via_gdb(nvs_id: int, data: bytes, elf_path: str):
    data_hex = ",".join(str(b) for b in data)
    commands = [
        "target extended-remote localhost:3333",
        f"set test_nvs_inject_id = {nvs_id}",
        f"set test_nvs_inject_len = {len(data)}"
    ]
    if len(data) > 0:
        commands.append(f"set test_nvs_inject_buf = {{{data_hex}}}")
    commands.append("set test_nvs_inject_cmd = 1")
    commands.append("detach")
    commands.append("quit")
    
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
        f.write("\n".join(commands) + "\n")
        script_path = f.name
        
    try:
        res = subprocess.run(["gdb-multiarch", "-batch", "-x", script_path, elf_path], capture_output=True, text=True)
        if res.returncode != 0:
            print("GDB Error:", res.stderr)
            raise RuntimeError("GDB injection failed. Is OpenOCD running on localhost:3333?")
    finally:
        os.remove(script_path)


@pytest.fixture
def hardware_device(request, test_network_config):
    port = request.config.getoption("--device-port")
    elf_path = "build/sesame_test/zephyr/zephyr.elf"
    
    if not os.path.exists(elf_path):
        pytest.fail(f"{elf_path} not found. Run: west build -b marvell_mw302 -d build/sesame_test --sysbuild")
        
    print(f"Connecting to hardware on {port}...")
    ser = serial.Serial(port, 115200, timeout=1)
    
    firmware_logs = []
    stop_reader = threading.Event()
    
    def log_reader():
        while not stop_reader.is_set():
            line = ser.readline()
            if line:
                line_str = line.decode(errors="ignore").strip()
                firmware_logs.append(line_str)
                # print(f"[FIRMWARE] {line_str}")

    reader_thread = threading.Thread(target=log_reader, daemon=True)
    reader_thread.start()
    
    def gdb_reset():
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
            f.write("target extended-remote localhost:3333\nmonitor reset run\ndetach\nquit\n")
            script_path = f.name
        try:
            subprocess.run(["gdb-multiarch", "-batch", "-x", script_path], capture_output=True)
        finally:
            os.remove(script_path)
            
    # Force a reboot so it starts clean
    gdb_reset()
                   
    time.sleep(2) # wait for boot
    
    # Inject NetworkConfig via NVS hook (ID 1)
    config_bytes = test_network_config.SerializeToString()
    print("Injecting NetworkConfig into NVS...")
    inject_nvs_via_gdb(1, config_bytes, elf_path)
    
    # Wait for the hook log
    start_time = time.time()
    hook_triggered = False
    while time.time() - start_time < 5:
        if any("Test hook triggered" in line for line in firmware_logs):
            hook_triggered = True
            break
        time.sleep(0.1)
        
    assert hook_triggered, "Firmware did not process NVS injection hook"
    
    # Reboot again so firmware uses the newly injected NVS
    print("Rebooting device to apply NVS config...")
    gdb_reset()
                   
    # Wait for Wi-Fi connection and IP
    device_ip = None
    start_time = time.time()
    while time.time() - start_time < 30:
        for line in list(firmware_logs):
            import re
            ip_match = re.search(r"IPv4 address: ([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)", line)
            if ip_match and not device_ip:
                device_ip = ip_match.group(1)
                break
        if device_ip:
            break
        time.sleep(0.1)
        
    assert device_ip, "Device failed to connect to Wi-Fi and acquire IP"
    print(f"Device ready on Wi-Fi at IP {device_ip}")
    
    yield {
        "ip": device_ip,
        "logs": firmware_logs
    }
    
    stop_reader.set()
    ser.close()


def test_http_door_endpoints(hardware_device):
    ip = hardware_device["ip"]
    logs = hardware_device["logs"]
    
    # Test /open
    print(f"Testing POST http://{ip}/open")
    resp = requests.post(f"http://{ip}/open", timeout=5)
    assert resp.status_code == 200
    
    # Verify log output
    time.sleep(0.5)
    assert any("Target=Open" in line for line in logs), "Door open command not logged by firmware"
    
    # Test /close
    print(f"Testing POST http://{ip}/close")
    resp = requests.post(f"http://{ip}/close", timeout=5)
    assert resp.status_code == 200
    
    time.sleep(0.5)
    assert any("Target=Close" in line for line in logs), "Door close command not logged by firmware"
    
    print("HTTP endpoints verified successfully!")
