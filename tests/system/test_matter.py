import asyncio
import os
import re
import shutil
import tempfile
import time

import matter.native

matter.native.Init()
import matter.ChipDeviceCtrl  # noqa: E402
import matter.clusters as Clusters  # noqa: E402
import matter.storage  # noqa: E402
from matter.CertificateAuthority import CertificateAuthorityManager  # noqa: E402
from matter.ChipStack import ChipStack  # noqa: E402
from matter.setup_payload.setup_payload import SetupPayload  # noqa: E402


def test_hardware_case_provisioning(hardware_device):
    logs = hardware_device["logs"]
    device_ip = hardware_device["ip"]

    pairing_code = None
    for line in logs:
        match = re.search(r"Manual pairing code: \[([0-9]+)\]", line)
        if match:
            pairing_code = match.group(1)
            break

    assert pairing_code, "Firmware did not output pairing code"
    print(f"--> Found pairing code: {pairing_code}")

    print(f"Hardware ready at {device_ip}. Commissioning...")

    # Use a temporary persistent storage for this test run
    temp_dir = tempfile.mkdtemp(dir="build")
    storage_path = os.path.join(temp_dir, "hw_repl_storage.json")

    try:
        storage = matter.storage.PersistentStorageJSON(storage_path)
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
        discriminator = int(parser.attributes["LongDiscriminator"])

        async def commission_and_control():
            print("Commissioning on Network (mDNS discovery + PASE + CASE)...")
            from matter.ChipDeviceCtrl import DiscoveryFilterType

            # This handles mDNS discovery, PASE, and CASE
            await controller.CommissionOnNetwork(
                nodeId=1,
                setupPinCode=setup_pin,
                filterType=DiscoveryFilterType.LONG_DISCRIMINATOR,
                filter=discriminator,
            )
            print("SUCCESS! Device fully commissioned using CASE.")

            print("Sending Door Open command...")
            await controller.SendCommand(
                1, 1, Clusters.WindowCovering.Commands.UpOrOpen()
            )

            print("Sending Door Close command...")
            await controller.SendCommand(
                1, 1, Clusters.WindowCovering.Commands.DownOrClose()
            )

        asyncio.run(commission_and_control())

        # Wait a bit to ensure logs are flushed
        time.sleep(2)

        # Verify the firmware actually received and executed the Matter commands
        # The PIC driver logs "PIC: OPEN" and "PIC: CLOSE"
        assert any(
            "PIC: OPEN" in line for line in hardware_device["logs"]
        ), "Door open command not logged by firmware"
        assert any(
            "PIC: CLOSE" in line for line in hardware_device["logs"]
        ), "Door close command not logged by firmware"

    finally:
        try:
            controller.Shutdown()
            ca_manager.Shutdown()
            stack.Shutdown()
        except Exception:
            pass
        shutil.rmtree(temp_dir)
