import pytest
import subprocess
import time
import os
import signal
import pty

def test_matter_provisioning():
    zephyr_exe = "build/integration/zephyr/zephyr.exe"
    if not os.path.exists(zephyr_exe):
        pytest.skip(f"{zephyr_exe} not found. Please build integration target first.")
    
    print("Starting Zephyr native_sim...")
    
    # We use a PTY so that libc uses line-buffering and no data is lost
    master, slave = pty.openpty()
    
    process = subprocess.Popen([
        zephyr_exe,
    ], stdout=slave, stderr=slave, text=True)
    
    os.close(slave)
    
    try:
        # Wait for boot
        booted = False
        start_time = time.time()
        buf = b""
        while time.time() - start_time < 30:
            try:
                data = os.read(master, 1024)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line_str = line.decode(errors="ignore").strip()
                        print(f"[FIRMWARE] {line_str}")
                        if "Network is UP. Opening commissioning window" in line_str:
                            booted = True
                            break
                    if booted: break
            except BlockingIOError:
                time.sleep(0.1)
                
        assert booted, "Firmware failed to boot or initialize network"
        print("Firmware booted successfully on native_sim!")
        
    finally:
        os.close(master)
        process.kill()
        process.wait(timeout=5)
