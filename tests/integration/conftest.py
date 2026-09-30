import os
import pty
import re
import subprocess
import threading
import time

import paho.mqtt.client as mqtt
import pytest

import sys

sys.path.insert(0, os.path.dirname(__file__))

from mqtt_server import EmbeddedMqttServer  # noqa: E402
from pic_simulator import PicSimulator  # noqa: E402
from proto import app_config_pb2  # noqa: E402


def cleanup_flash():
    """Helper to remove flash simulation files and configs from the CWD."""
    for f in [
        "flash.bin",
        "repl_storage.json",
        "test_config.bin",
        "/tmp/sesame_pic_uart",
    ]:
        try:
            os.remove(f)
        except FileNotFoundError:
            pass


@pytest.fixture(scope="session")
def mqtt_server():
    """Starts an embedded MQTT 3.1.1 server on a free local port."""
    server = EmbeddedMqttServer()
    yield server
    server.close()


@pytest.fixture
def config(mqtt_server):
    """
    Config fixture: Prepares and writes test_config.bin with embedded MQTT configuration
    before the firmware launches.
    """
    cfg = app_config_pb2.AppConfig()
    cfg.mqtt_config.enabled = True
    cfg.mqtt_config.broker_host = "127.0.0.1"
    cfg.mqtt_config.broker_port = mqtt_server.port
    cfg.mqtt_config.prefix = "sesame"

    with open("test_config.bin", "wb") as f:
        f.write(cfg.SerializeToString())

    yield cfg

    try:
        os.remove("test_config.bin")
    except FileNotFoundError:
        pass


@pytest.fixture
def paho_client(mqtt_server):
    """Provides a connected paho-mqtt client instance for testing."""
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect("127.0.0.1", mqtt_server.port, 60)
    client.loop_start()

    yield client

    client.loop_stop()
    client.disconnect()


@pytest.fixture
def zephyr_app(request, mqtt_server):
    """
    Spawns the Zephyr native_sim application in the background and reads its logs.
    Also provisions a virtual UART PTY for PIC communications and attaches PicSimulator.
    Yields the firmware logs, manual pairing code, pic_sim, and mqtt_server.
    """
    zephyr_exe = "build/native_sim/zephyr/zephyr.exe"
    if not os.path.exists(zephyr_exe):
        pytest.fail(f"{zephyr_exe} not found. Please build integration target first.")

    cleanup_flash()

    # If test requested config fixture, ensure it is instantiated before zephyr_app boots
    if "config" in request.fixturenames:
        request.getfixturevalue("config")

    # Create virtual UART PTY for PIC comms
    pic_master, pic_slave = pty.openpty()
    pic_slave_name = os.ttyname(pic_slave)

    try:
        os.unlink("/tmp/sesame_pic_uart")
    except FileNotFoundError:
        pass
    try:
        os.symlink(pic_slave_name, "/tmp/sesame_pic_uart")
    except Exception:
        pass

    pic_sim = PicSimulator(pic_master)

    # Console PTY
    master, slave = pty.openpty()
    process = subprocess.Popen(
        [zephyr_exe, f"-uart_1_port={pic_slave_name}"],
        stdout=slave,
        stderr=slave,
        text=True,
    )
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
        pic_sim.close()
        os.close(pic_slave)
        process.kill()
        process.wait(timeout=5)
        cleanup_flash()
        pytest.fail("Firmware failed to boot or output pairing code")

    yield {
        "process": process,
        "logs": firmware_logs,
        "pairing_code": pairing_code,
        "pic_sim": pic_sim,
        "mqtt_server": mqtt_server,
    }

    # Teardown
    stop_reader.set()
    reader_thread.join(timeout=2)
    os.close(master)
    pic_sim.close()
    try:
        os.close(pic_slave)
    except Exception:
        pass
    process.kill()
    process.wait(timeout=5)
    cleanup_flash()
