"""System test: one OTA over Matter (BDX).

The hardware sesame_test device boots as the OtaSoftwareUpdateRequestor
(endpoint 0), and a Linux ota-provider-app acts as the
OtaSoftwareUpdateProvider. Flow:

  1. Device boots Image A (v0.2.0) from slot 0 with its commissioning
     window open.
  2. Commission the device (requestor) as node 1.
  3. Start the Linux provider and commission it as node 2.
  4. Grant the requestor an ACL entry on the provider for the
     OtaSoftwareUpdateProvider cluster, then send AnnounceOTAProvider.
  5. The device auto-queries, downloads the image over BDX, writes it to
     the inactive slot, and reboots into the new (test) image from slot 1.
"""

import asyncio
import os
import re
import shutil
import subprocess
import tempfile
import threading
from pathlib import Path

import pytest
from test_ota import (  # noqa: E402
    get_system_test_versions,
    semihosting_keepalive,
    wait_for_boot,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
# The device signs its Matter attestation with the development (VID 0xFFF1)
# example creds, which chain to a dev PAA root. Point the controller at the
# real PAA root-cert dir explicitly (matches test_matter.py).
PAA_TRUST_STORE = str(
    REPO_ROOT
    / "third_party"
    / "connectedhomeip"
    / "credentials"
    / "development"
    / "paa-root-certs"
)
PROVIDER_BIN = (
    REPO_ROOT
    / "third_party"
    / "connectedhomeip"
    / "out"
    / "ota_provider"
    / "chip-ota-provider-app"
)
OTA_CONTAINER = REPO_ROOT / "build" / "sesame_test" / "zephyr" / "image_b.ota"

REQUESTOR_NODE_ID = 1
PROVIDER_NODE_ID = 2
ADMIN_NODE_ID = 112233
PROVIDER_VENDOR_ID = 0xFFF1
PROVIDER_DISCRIMINATOR = 1111
PROVIDER_PINCODE = 20202021
VERSIONS = get_system_test_versions()
NEW_VERSION = VERSIONS["b"]


def _find_pairing_code(logs):
    for line in logs:
        match = re.search(r"Manual pairing code: \[([0-9]+)\]", line)
        if match:
            return match.group(1)
    return None


def _start_provider(kvs_path):
    """Launch the Linux OTA provider and wait until it is ready to commission.

    The provider serves the pre-built OTA container and opens a commissioning
    window (discriminator 1111, PIN 20202021). Returns the Popen object.
    """
    cmd = [
        str(PROVIDER_BIN),
        "--KVS",
        kvs_path,
        "--discriminator",
        str(PROVIDER_DISCRIMINATOR),
        "--passcode",
        str(PROVIDER_PINCODE),
        "-f",
        str(OTA_CONTAINER),
    ]
    print("Starting OTA provider: " + " ".join(cmd))
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    # Drain the provider's stdout in a background thread for its whole
    # lifetime. If we stopped reading once it became ready, the 64KB pipe
    # buffer would fill with its (verbose) Matter logs and the provider's
    # synchronous write() to stdout would block, stalling the Matter event
    # loop and making it unable to answer the commissioning NOC.
    ready = threading.Event()

    def _reader():
        for line in proc.stdout:
            line = line.rstrip()
            print(f"[provider] {line}", flush=True)
            if "Server initialization complete" in line:
                ready.set()

    threading.Thread(target=_reader, daemon=True).start()
    if not ready.wait(timeout=60):
        proc.kill()
        raise AssertionError("OTA provider did not become ready within 60s")
    return proc


def test_matter_ota(hardware_device, openocd, matter, fabric_admin):
    if not PROVIDER_BIN.exists():
        pytest.skip(f"OTA provider binary not built: {PROVIDER_BIN}")
    if not OTA_CONTAINER.exists():
        pytest.skip(f"Matter OTA container missing: {OTA_CONTAINER}")

    logs = hardware_device["logs"]

    print("\n[Step 1] Verifying initial boot of Image A from slot 0...")
    wait_for_boot(
        logs,
        0,
        expected_slot=0,
        expected_version=VERSIONS["base"],
        expect_test_running=False,
        timeout=20,
    )
    pairing_code = _find_pairing_code(logs)
    assert pairing_code, "Firmware did not output pairing code"
    print(f"--> Found device pairing code: {pairing_code}")

    temp_dir = tempfile.mkdtemp(dir="build")
    provider = None
    controller = None
    try:
        controller = fabric_admin.NewController(
            nodeId=ADMIN_NODE_ID, paaTrustStorePath=PAA_TRUST_STORE
        )

        Clusters = matter["Clusters"]
        NullValue = Clusters.Types.NullValue
        AC = Clusters.AccessControl
        OtaReq = Clusters.OtaSoftwareUpdateRequestor
        OtaProv = Clusters.OtaSoftwareUpdateProvider
        DiscoveryFilterType = matter["ChipDeviceCtrl"].DiscoveryFilterType

        parser = matter["SetupPayload"]()
        parser.ParseManualPairingCode(pairing_code)
        setup_pin = int(parser.attributes["SetUpPINCode"])
        discriminator = int(parser.attributes["Short discriminator"])

        async def commission_and_announce():
            nonlocal provider
            # Commission the requestor FIRST, before the (local) provider is
            # started. The provider runs on this host; if it were already
            # advertising _matterc via mDNS it would shadow the remote device
            # and the controller would commission the wrong node.
            print(
                "\n[Step 2] Commissioning the device (requestor) as node "
                f"{REQUESTOR_NODE_ID}..."
            )
            await controller.CommissionOnNetwork(
                nodeId=REQUESTOR_NODE_ID,
                setupPinCode=setup_pin,
                filterType=DiscoveryFilterType.SHORT_DISCRIMINATOR,
                filter=discriminator,
            )
            print(f"Device commissioned as node {REQUESTOR_NODE_ID}.")

            print("\n[Step 3] Starting the OTA provider...")
            provider = _start_provider(os.path.join(temp_dir, "provider_kv"))

            print(
                "\n[Step 3] Commissioning the provider as node "
                f"{PROVIDER_NODE_ID}..."
            )
            await controller.CommissionOnNetwork(
                nodeId=PROVIDER_NODE_ID,
                setupPinCode=PROVIDER_PINCODE,
                filterType=DiscoveryFilterType.LONG_DISCRIMINATOR,
                filter=PROVIDER_DISCRIMINATOR,
            )
            print(f"Provider commissioned as node {PROVIDER_NODE_ID}.")

            # Grant the requestor access to the provider's OTA cluster.
            acl = [
                AC.Structs.AccessControlEntryStruct(
                    privilege=AC.Enums.AccessControlEntryPrivilegeEnum.kAdminister,
                    authMode=AC.Enums.AccessControlEntryAuthModeEnum.kCase,
                    subjects=[ADMIN_NODE_ID],
                    targets=NullValue,
                ),
                AC.Structs.AccessControlEntryStruct(
                    privilege=AC.Enums.AccessControlEntryPrivilegeEnum.kOperate,
                    authMode=AC.Enums.AccessControlEntryAuthModeEnum.kCase,
                    subjects=[REQUESTOR_NODE_ID],
                    targets=[
                        AC.Structs.AccessControlTargetStruct(
                            cluster=OtaProv.id,
                            endpoint=NullValue,
                            deviceType=NullValue,
                        )
                    ],
                ),
            ]
            await controller.WriteAttribute(
                nodeId=PROVIDER_NODE_ID,
                attributes=[(0, AC.Attributes.Acl(acl))],
            )
            print("ACL written on provider granting requestor OTA access.")

            # Tell the device (requestor) an update is available from the provider.
            print(
                "\n[Step 4] Sending AnnounceOTAProvider; the device should now "
                "query and download over BDX..."
            )
            await controller.SendCommand(
                REQUESTOR_NODE_ID,
                0,
                OtaReq.Commands.AnnounceOTAProvider(
                    providerNodeID=PROVIDER_NODE_ID,
                    vendorID=PROVIDER_VENDOR_ID,
                    announcementReason=OtaReq.Enums.AnnouncementReasonEnum.kUpdateAvailable,
                    metadataForNode=None,
                    endpoint=0,
                ),
            )
            print("AnnounceOTAProvider sent.")

        asyncio.run(commission_and_announce())

        print(
            "\n[Step 5] Waiting for the device to download, flash, and reboot "
            "into the new image from slot 1..."
        )
        step_start = len(logs)
        with semihosting_keepalive(openocd):
            wait_for_boot(
                logs,
                step_start,
                expected_slot=1,
                expected_version=NEW_VERSION,
                expect_test_running=True,
                timeout=300,
            )
        print(
            "\nMatter OTA complete: device rebooted into the new test image from slot 1."
        )

    finally:
        if provider is not None:
            provider.terminate()
            try:
                provider.wait(timeout=5)
            except Exception:
                provider.kill()
        if controller is not None:
            try:
                controller.Shutdown()
            except Exception:
                pass
        shutil.rmtree(temp_dir, ignore_errors=True)
