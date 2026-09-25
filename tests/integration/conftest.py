import os
import pty
import re
import subprocess
import threading
import time

import pytest


def cleanup_flash():
    """Helper to remove flash simulation files from the CWD."""
    for f in ["flash.bin", "repl_storage.json"]:
        try:
            os.remove(f)
        except FileNotFoundError:
            pass


@pytest.fixture
def zephyr_app():
    """
    Spawns the Zephyr native_sim application in the background and reads its logs.
    Yields the firmware logs and manual pairing code.
    Cleans up the process and flash files automatically after the test finishes.
    """
    zephyr_exe = "build/native_sim/zephyr/zephyr.exe"
    if not os.path.exists(zephyr_exe):
        pytest.fail(f"{zephyr_exe} not found. Please build integration target first.")

    cleanup_flash()

    master, slave = pty.openpty()
    process = subprocess.Popen([zephyr_exe], stdout=slave, stderr=slave, text=True)
    os.close(slave)

    firmware_logs = []
    stop_reader = threading.Event()

    def log_reader():
        import select

        buf = b""
        while not stop_reader.is_set():
            try:
                r, _, _ = select.select([master], [], [], 0.1)
                if r:
                    data = os.read(master, 1024)
                    if data:
                        buf += data
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            line_str = line.decode(errors="ignore").strip()
                            firmware_logs.append(line_str)
                            print(f"[FIRMWARE] {line_str}")
            except Exception:
                pass

    reader_thread = threading.Thread(target=log_reader, daemon=True)
    reader_thread.start()

    booted = False
    pairing_code = None
    start_time = time.time()

    print("Waiting for Zephyr native_sim to boot...")
    while time.time() - start_time < 30:
        for line in list(firmware_logs):
            match = re.search(r"Manual pairing code: \[([0-9]+)\]", line)
            if match and not pairing_code:
                pairing_code = match.group(1)
            if "Commissioning window opened successfully" in line:
                booted = True

        if booted and pairing_code:
            break
        time.sleep(0.1)

    if not booted or not pairing_code:
        # Cleanup before asserting so we don't leak process
        stop_reader.set()
        reader_thread.join(timeout=2)
        os.close(master)
        process.kill()
        process.wait(timeout=5)
        cleanup_flash()
        pytest.fail("Firmware failed to boot or output pairing code")

    yield {"process": process, "logs": firmware_logs, "pairing_code": pairing_code}

    # Teardown
    stop_reader.set()
    reader_thread.join(timeout=2)
    os.close(master)
    process.kill()
    process.wait(timeout=5)
    cleanup_flash()
