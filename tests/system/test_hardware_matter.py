#!/usr/bin/env python3
import asyncio
import os
import re
import sys
import threading
import time
import subprocess
import serial
import pytest

# Ensure matter python bindings are loaded
try:
    import matter.native
    matter.native.Init()
    from matter.CertificateAuthority import CertificateAuthorityManager
    from matter.ChipStack import ChipStack
    import matter.ChipDeviceCtrl
    import matter.storage
    from matter.setup_payload.setup_payload import SetupPayload
except ImportError:
    print("Matter Python bindings not found. Activate venv first.")
    sys.exit(1)

def test_hardware_case_provisioning(port='/dev/ttyUSB0'):
    if not os.path.exists(port):
        pytest.skip(f"Hardware not connected at {port}")

    print(f"Connecting to hardware on {port}...")
    ser = serial.Serial(port, 115200, timeout=1)

    # Reboot device
    ser.write(b'\x03') # Ctrl-C
    time.sleep(0.5)
    ser.write(b'reboot\r\n')

    firmware_logs = []
    stop_reader = threading.Event()

    def log_reader():
        while not stop_reader.is_set():
            line = ser.readline()
            if line:
                line_str = line.decode(errors="ignore").strip()
                firmware_logs.append(line_str)
                print(f"[FIRMWARE] {line_str}")

    reader_thread = threading.Thread(target=log_reader, daemon=True)
    reader_thread.start()

    try:
        booted = False
        pairing_code = None
        device_ip = None

        print("Waiting for boot, Wi-Fi connection, and pairing code...")
        start_time = time.time()
        while time.time() - start_time < 45:
            for line in list(firmware_logs):
                match = re.search(r"Manual pairing code: \[([0-9]+)\]", line)
                if match and not pairing_code:
                    pairing_code = match.group(1)
                    print(f"--> Found pairing code: {pairing_code}")

                # Check for IP address in logs
                ip_match = re.search(r"IPv4 address: ([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)", line)
                if ip_match and not device_ip:
                    device_ip = ip_match.group(1)
                    print(f"--> Found Device IP: {device_ip}")

                if "Network is UP" in line:
                    booted = True

            if booted and pairing_code and device_ip:
                break
            time.sleep(0.1)

        assert booted, "Firmware failed to boot/connect to Wi-Fi"
        assert pairing_code, "Firmware did not output pairing code"
        assert device_ip, "Firmware did not output IP address"

        print(f"Hardware ready at {device_ip}. Commissioning...")

        storage = matter.storage.PersistentStorageJSON('hw_repl_storage.json')
        stack = ChipStack(persistentStorage=storage)
        ca_manager = CertificateAuthorityManager(stack, stack.GetStorageManager())
        ca_manager.LoadAuthoritiesFromStorage()
        if len(ca_manager.activeCaList) == 0:
            ca = ca_manager.NewCertificateAuthority()
            ca.NewFabricAdmin(vendorId=0xFFF1, fabricId=1)

        ca = ca_manager.activeCaList[0]
        admin = ca.adminList[0]
        controller = admin.NewController(nodeId=112233)

        parser = SetupPayload()
        parser.ParseManualPairingCode(pairing_code)
        setup_pin = int(parser.attributes["SetUpPINCode"])

        async def commission():
            # For physical hardware, mDNS works seamlessly over the real Wi-Fi network!
            # We can use the full standard Commissioning flow.
            print("Establishing PASE Session...")
            await controller.EstablishPASESessionIP(device_ip, setup_pin, 1)
            print("Proceeding with CASE Operational Discovery (mDNS)...")
            await controller.Commission(1)
            print("SUCCESS! Device fully commissioned using CASE.")

        asyncio.run(commission())

    finally:
        stop_reader.set()
        ser.close()

if __name__ == '__main__':
    test_hardware_case_provisioning()
