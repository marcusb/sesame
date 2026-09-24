import asyncio
import logging
import os
import pty
import re
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import pytest

logging.basicConfig(level=logging.DEBUG)

# The device signs its Matter attestation with the development (VID 0xFFF1)
# example creds, which chain to a dev PAA root. The Matter python controller
# defaults to a CWD-relative trust store that only resolves if `./credentials`
# is a symlink, so point it at the real PAA root-cert dir explicitly.
PAA_TRUST_STORE = str(
    Path(__file__).resolve().parents[2]
    / "third_party" / "connectedhomeip" / "credentials" / "development" / "paa-root-certs"
)


def cleanup_flash():
    """Helper to remove flash simulation files from the CWD."""
    for f in ["flash.bin", "repl_storage.json"]:
        try:
            os.remove(f)
        except FileNotFoundError:
            pass


@pytest.fixture(scope="module")
def matter_classes():
    """
    Dynamically loads the Matter Python bindings.
    Skips tests if the bindings are not built or not in the Python path.
    """
    try:
        import matter.native

        matter.native.Init()
        import matter.ChipDeviceCtrl
        import matter.clusters as Clusters
        import matter.storage
        from matter.CertificateAuthority import CertificateAuthorityManager
        from matter.ChipStack import ChipStack
        from matter.setup_payload.setup_payload import SetupPayload

        return {
            "CertificateAuthorityManager": CertificateAuthorityManager,
            "ChipStack": ChipStack,
            "ChipDeviceCtrl": matter.ChipDeviceCtrl,
            "storage": matter.storage,
            "Clusters": Clusters,
            "SetupPayload": SetupPayload,
        }
    except ImportError:
        pytest.fail(
            "Matter Python bindings not found. Please build them first and activate the venv."
        )


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


@pytest.fixture
def matter_controller(matter_classes):
    """
    Initializes the Matter controller stack and provides a controller instance.
    Cleans up the stack and temporary persistent storage after the test.
    """
    temp_dir = tempfile.mkdtemp(dir="build")

    storage = matter_classes["storage"].PersistentStorageJSON(
        os.path.join(temp_dir, "repl_storage.json")
    )
    stack = matter_classes["ChipStack"](persistentStorage=storage)
    ca_manager = matter_classes["CertificateAuthorityManager"](
        stack, stack.GetStorageManager()
    )
    ca_manager.LoadAuthoritiesFromStorage()

    if len(ca_manager.activeCaList) == 0:
        ca = ca_manager.NewCertificateAuthority()
        ca.NewFabricAdmin(vendorId=0xFFF1, fabricId=1)

    ca = ca_manager.activeCaList[0]
    admin = ca.adminList[0]
    controller = admin.NewController(
        nodeId=112233, paaTrustStorePath=PAA_TRUST_STORE
    )

    yield controller

    # Teardown
    try:
        controller.Shutdown()
        ca_manager.Shutdown()
        stack.Shutdown()
    except Exception:
        pass
    shutil.rmtree(temp_dir)


def test_matter_provisioning(zephyr_app, matter_controller, matter_classes):
    """
    Verifies that the firmware can be commissioned over PASE
    and properly processes operational commands (like UpOrOpen).
    """
    pairing_code = zephyr_app["pairing_code"]
    firmware_logs = zephyr_app["logs"]

    parser = matter_classes["SetupPayload"]()
    parser.ParseManualPairingCode(pairing_code)
    setup_pin = int(parser.attributes["SetUpPINCode"])

    print(f"Commissioning Node 1 with setup PIN {setup_pin} via IP...")
    time.sleep(1.0)

    async def commission():
        await matter_controller.EstablishPASESessionIP("::1", setup_pin, 1)

    asyncio.run(commission())

    print("Commissioning successful! Sending Door Open command...")

    async def send_command():
        Clusters = matter_classes["Clusters"]
        # 1 is endpoint 1
        await matter_controller.SendCommand(
            1, 1, Clusters.WindowCovering.Commands.UpOrOpen()
        )

    asyncio.run(send_command())
    time.sleep(2)  # Give it time to process and log

    command_received = False
    end_time = time.time() + 10
    while time.time() < end_time:
        if any("Matter: Target=Open" in line for line in firmware_logs):
            command_received = True
            break
        time.sleep(0.1)

    assert command_received, "Firmware did not log receipt of the Open command"
    print("SUCCESS! Integration test passed.")
