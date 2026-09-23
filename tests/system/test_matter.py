import asyncio
import re
import time
from pathlib import Path

# The device signs its Matter attestation with the development (VID 0xFFF1)
# example creds, which chain to a dev PAA root. The Matter python controller
# defaults to a CWD-relative trust store that only resolves if `./credentials`
# is a symlink, so point it at the real PAA root-cert dir explicitly.
PAA_TRUST_STORE = str(
    Path(__file__).resolve().parents[2]
    / "third_party" / "connectedhomeip" / "credentials" / "development" / "paa-root-certs"
)


def test_commission_and_control(hardware_device, matter, fabric_admin):
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

    controller = fabric_admin.NewController(
        nodeId=112233, paaTrustStorePath=PAA_TRUST_STORE
    )
    try:
        parser = matter["SetupPayload"]()
        parser.ParseManualPairingCode(pairing_code)
        setup_pin = int(parser.attributes["SetUpPINCode"])
        discriminator = int(parser.attributes["Short discriminator"])

        async def commission_and_control():
            print("Commissioning on Network (mDNS discovery + PASE + CASE)...")
            DiscoveryFilterType = matter["ChipDeviceCtrl"].DiscoveryFilterType

            # This handles mDNS discovery, PASE, and CASE
            await controller.CommissionOnNetwork(
                nodeId=1,
                setupPinCode=setup_pin,
                filterType=DiscoveryFilterType.SHORT_DISCRIMINATOR,
                filter=discriminator,
            )
            print("SUCCESS! Device fully commissioned using CASE.")

            print("Sending Door Open command...")
            await controller.SendCommand(
                1, 1, matter["Clusters"].WindowCovering.Commands.UpOrOpen()
            )

            print("Sending Door Close command...")
            await controller.SendCommand(
                1, 1, matter["Clusters"].WindowCovering.Commands.DownOrClose()
            )

        asyncio.run(commission_and_control())

        # Wait a bit to ensure logs are flushed
        time.sleep(8)

        # Verify the firmware actually received and executed the Matter commands
        # The PIC driver logs "PIC: OPEN" and "PIC: CLOSE"
        assert any(
            "PIC: OPEN" in line for line in hardware_device["logs"]
        ), "Door open command not logged by firmware"
        assert any(
            "PIC: CLOSE" in line for line in hardware_device["logs"]
        ), "Door close command not logged by firmware"

    finally:
        print("\n--- FIRMWARE LOGS DURING COMMISSIONING ---")
        for line in hardware_device["logs"]:
            print(line)
        try:
            controller.Shutdown()
        except Exception:
            pass
